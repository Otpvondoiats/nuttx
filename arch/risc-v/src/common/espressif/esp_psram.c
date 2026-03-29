/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_psram.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include "esp_psram.h"

#include "esp_attr.h"
#include "hal/psram_ctrlr_ll.h"
#include "hal/mspi_ll.h"
#include "soc/hp_sys_clkrst_reg.h"
#include "soc/spi_mem_s_reg.h"
#include "soc/spi1_mem_s_reg.h"
#include "soc/spi1_mem_s_struct.h"
#include "esp_private/mspi_timing_tuning.h"

extern int ets_printf(const char *fmt, ...);
extern void rtc_clk_mpll_enable(void);
extern void rtc_clk_mpll_configure(uint32_t xtal_freq, uint32_t mpll_freq,
                                   bool thread_safe);
extern void ets_delay_us(uint32_t us);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AP Memory HEX PSRAM commands (16-bit DDR) */

#define HEX_PSRAM_SYNC_READ        0x0000
#define HEX_PSRAM_SYNC_WRITE       0x8080
#define HEX_PSRAM_REG_READ         0x4040
#define HEX_PSRAM_REG_WRITE        0xC0C0
#define HEX_PSRAM_RD_CMD_BITLEN    16
#define HEX_PSRAM_WR_CMD_BITLEN    16
#define HEX_PSRAM_ADDR_BITLEN      32

/* Dummy cycles matching PSRAM chip DEFAULT power-up state.
 * AP HEX PSRAM defaults: read_latency=3 (variable), wr_latency=0
 * Default MR8: bl=0(16B wrap), x16=0(x8 mode)
 * We must match these defaults since we can't send mode reg commands.
 */

#define HEX_PSRAM_RD_DUMMY_BITLEN       (2*(10-1))
#define HEX_PSRAM_RD_REG_DUMMY_BITLEN   (2*(5-1))
#define HEX_PSRAM_WR_DUMMY_BITLEN       (2*(5-1))
#define HEX_PSRAM_RD_LATENCY            2
#define HEX_PSRAM_WR_LATENCY            2

/* MPLL frequency */

#define HEX_PSRAM_MPLL_FREQ_MHZ    400
#define HEX_PSRAM_XTAL_FREQ_MHZ    40
#define HEX_PSRAM_SPEED_MHZ        200

/* CS timing */

#define HEX_PSRAM_CS_SETUP_TIME    4
#define HEX_PSRAM_CS_HOLD_TIME     4
#define HEX_PSRAM_CS_HOLD_DELAY    3

/* Vendor IDs */

#define HEX_PSRAM_VENDOR_ID_AP     0xD
#define HEX_PSRAM_VENDOR_ID_UNILC  0x1A

/* Reference data for connection check */

#define HEX_PSRAM_REF_DATA         0x5a6b7c8d

/* PSRAM sizes */

#define PSRAM_SIZE_4MB             (4 * 1024 * 1024)
#define PSRAM_SIZE_8MB             (8 * 1024 * 1024)
#define PSRAM_SIZE_16MB            (16 * 1024 * 1024)
#define PSRAM_SIZE_32MB            (32 * 1024 * 1024)
#define PSRAM_SIZE_64MB            (64 * 1024 * 1024)

/* MSPI IDs */

#define PSRAM_CACHE_MSPI           PSRAM_CTRLR_LL_MSPI_ID_2
#define PSRAM_USER_MSPI            PSRAM_CTRLR_LL_MSPI_ID_3

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  union
  {
    struct
    {
      uint8_t drive_str: 2;
      uint8_t read_latency: 3;
      uint8_t lt: 1;
      uint8_t rsvd6: 1;
      uint8_t tso: 1;
    };
    uint8_t val;
  } mr0;

  union
  {
    struct
    {
      uint8_t vendor_id: 5;
      uint8_t rsvd0_2: 2;
      uint8_t ulp: 1;
    };
    uint8_t val;
  } mr1;

  union
  {
    struct
    {
      uint8_t density: 3;
      uint8_t dev_id: 2;
      uint8_t kgd: 3;
    };
    uint8_t val;
  } mr2;

  uint8_t mr3_val;

  union
  {
    struct
    {
      uint8_t pasr: 3;
      uint8_t rf: 2;
      uint8_t wr_latency: 3;
    };
    uint8_t val;
  } mr4;

  union
  {
    struct
    {
      uint8_t bl: 2;
      uint8_t bt: 1;
      uint8_t rbx: 1;
      uint8_t rsvd5: 2;
      uint8_t x16: 1;
      uint8_t rsvd7: 1;
    };
    uint8_t val;
  } mr8;
} hex_psram_mode_reg_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_psram_size;
static bool g_psram_initialized;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void IRAM_ATTR psram_transaction(uint32_t mspi_id,
                                        uint32_t cmd, uint32_t cmd_bitlen,
                                        uint32_t addr, uint32_t addr_bitlen,
                                        uint32_t dummy_bits,
                                        uint8_t *mosi, uint32_t mosi_bitlen,
                                        uint8_t *miso, uint32_t miso_bitlen,
                                        bool is_write)
{
  while (SPIMEM3.cmd.usr) { }

  /* Configure DDR + OCT + DQS_LOOP for HEX PSRAM */

  SPIMEM3.ddr.fmem_ddr_dqs_loop = 1;
  SPIMEM3.ddr.fmem_var_dummy = 1;
  SPIMEM3.ddr.fmem_ddr_en = 1;
  SPIMEM3.ctrl.fcmd_oct = 1;
  SPIMEM3.ctrl.faddr_oct = 1;
  SPIMEM3.ctrl.fdin_oct = 1;
  SPIMEM3.ctrl.fdout_oct = 1;

  /* Copy PSRAM clock config from MSPI2 to MSPI3 */

  SPIMEM3.clock.val = SPIMEM2.mem_sram_clk.val;

  /* Configure phases */

  SPIMEM3.user.usr_command = (cmd_bitlen > 0) ? 1 : 0;
  SPIMEM3.user.usr_addr = (addr_bitlen > 0) ? 1 : 0;
  SPIMEM3.user.usr_dummy = (dummy_bits > 0) ? 1 : 0;
  SPIMEM3.user.usr_mosi = (mosi_bitlen > 0 && mosi) ? 1 : 0;
  SPIMEM3.user.usr_miso = (miso_bitlen > 0) ? 1 : 0;

  if (cmd_bitlen > 0)
    {
      SPIMEM3.user2.usr_command_bitlen = cmd_bitlen - 1;
      SPIMEM3.user2.usr_command_value = cmd;
    }

  if (addr_bitlen > 0)
    {
      SPIMEM3.user1.usr_addr_bitlen = addr_bitlen - 1;
      SPIMEM3.addr.usr_addr_value = addr;
    }

  if (dummy_bits > 0)
    {
      SPIMEM3.user1.usr_dummy_cyclelen = dummy_bits - 1;
    }

  if (mosi_bitlen > 0 && mosi)
    {
      SPIMEM3.mosi_dlen.usr_mosi_dbitlen = mosi_bitlen - 1;
      SPIMEM3.w0.val = *(uint32_t *)mosi;
    }

  if (miso_bitlen > 0)
    {
      SPIMEM3.miso_dlen.usr_miso_dbitlen = miso_bitlen - 1;
    }

  /* Trigger and wait - use timeout to prevent WDT */

  SPIMEM3.cmd.val = 0;
  SPIMEM3.cmd.usr = 1;

  volatile int timeout = 1000000;
  while (SPIMEM3.cmd.usr && --timeout > 0) { }

  if (miso_bitlen > 0 && miso)
    {
      *(uint32_t *)miso = SPIMEM3.w0.val;
    }

  /* Force clear USR bit if timeout - MSPI3 may hold the bus
   * preventing MSPI2 cache access. Write 0 to entire CMD reg.
   */

  SPIMEM3.cmd.val = 0;

  /* Restore DDR/OCT settings */

  SPIMEM3.ddr.fmem_ddr_en = 0;
  SPIMEM3.ddr.fmem_ddr_dqs_loop = 0;
  SPIMEM3.ddr.fmem_var_dummy = 0;
  SPIMEM3.ctrl.fcmd_oct = 0;
  SPIMEM3.ctrl.faddr_oct = 0;
  SPIMEM3.ctrl.fdin_oct = 0;
  SPIMEM3.ctrl.fdout_oct = 0;

  if (timeout <= 0)
    {
      ets_printf("PSRAM: SPI3 TIMEOUT!\n");
    }
}

static void IRAM_ATTR psram_init_mode_reg(int spi_num,
                                          hex_psram_mode_reg_t *cfg)
{
  hex_psram_mode_reg_t reg = {0};
  int dummy = HEX_PSRAM_RD_REG_DUMMY_BITLEN;

  /* Read MR0 and MR1 */

  psram_transaction(spi_num,
                    HEX_PSRAM_REG_READ, 16,
                    0x0, 32, dummy,
                    NULL, 0, &reg.mr0.val, 16, false);

  /* Read MR4 and MR8 */

  psram_transaction(spi_num,
                    HEX_PSRAM_REG_READ, 16,
                    0x4, 32, dummy,
                    NULL, 0, &reg.mr4.val, 16, false);

  /* Modify */

  reg.mr0.lt = cfg->mr0.lt;
  reg.mr0.read_latency = cfg->mr0.read_latency;
  reg.mr0.drive_str = cfg->mr0.drive_str;
  reg.mr4.wr_latency = cfg->mr4.wr_latency;

  /* Write MR0 */

  psram_transaction(spi_num,
                    HEX_PSRAM_REG_WRITE, 16,
                    0x0, 32, 0,
                    &reg.mr0.val, 8, NULL, 0, true);

  /* Write MR4 */

  psram_transaction(spi_num,
                    HEX_PSRAM_REG_WRITE, 16,
                    0x4, 32, 0,
                    &reg.mr4.val, 8, NULL, 0, true);

  /* Write MR8 */

  reg.mr8.bl = cfg->mr8.bl;
  reg.mr8.bt = cfg->mr8.bt;
  reg.mr8.rbx = cfg->mr8.rbx;
  reg.mr8.x16 = cfg->mr8.x16;

  psram_transaction(spi_num,
                    HEX_PSRAM_REG_WRITE, 16,
                    0x8, 32, 0,
                    &reg.mr8.val, 8, NULL, 0, true);
}

static int IRAM_ATTR psram_check_connected(int spi_num)
{
  uint32_t addr = 0;
  uint32_t ref = HEX_PSRAM_REF_DATA;
  uint32_t rd = 0;

  psram_transaction(spi_num,
                    HEX_PSRAM_SYNC_WRITE, HEX_PSRAM_WR_CMD_BITLEN,
                    addr, HEX_PSRAM_ADDR_BITLEN,
                    HEX_PSRAM_WR_DUMMY_BITLEN,
                    (uint8_t *)&ref, 32, NULL, 0, true);

  psram_transaction(spi_num,
                    HEX_PSRAM_SYNC_READ, HEX_PSRAM_RD_CMD_BITLEN,
                    addr, HEX_PSRAM_ADDR_BITLEN,
                    HEX_PSRAM_RD_DUMMY_BITLEN,
                    NULL, 0, (uint8_t *)&rd, 32, false);

  if (rd != ref)
    {
      ets_printf("PSRAM: check fail w=0x%08lx r=0x%08lx\n",
                 (unsigned long)ref, (unsigned long)rd);
      return -ENODEV;
    }

  return OK;
}

static void IRAM_ATTR psram_get_mode_reg(int spi_num,
                                         hex_psram_mode_reg_t *reg)
{
  int dummy = HEX_PSRAM_RD_REG_DUMMY_BITLEN;

  psram_transaction(spi_num,
                    HEX_PSRAM_REG_READ, 16,
                    0x0, 32, dummy,
                    NULL, 0, &reg->mr0.val, 16, false);

  psram_transaction(spi_num,
                    HEX_PSRAM_REG_READ, 16,
                    0x2, 32, dummy,
                    NULL, 0, &reg->mr2.val, 16, false);

  psram_transaction(spi_num,
                    HEX_PSRAM_REG_READ, 16,
                    0x4, 32, dummy,
                    NULL, 0, &reg->mr4.val, 16, false);

  psram_transaction(spi_num,
                    HEX_PSRAM_REG_READ, 16,
                    0x8, 32, dummy,
                    NULL, 0, &reg->mr8.val, 16, false);
}

static void IRAM_ATTR psram_config_mspi(void)
{
  /* Configure cache controller (MSPI2) for PSRAM access.
   * Since we can't send mode register commands via MSPI3,
   * match the chip's DEFAULT power-up state:
   *   x8 (octal) mode, NOT x16 (hex)
   *   read_latency=3, lt=0 (fixed latency)
   *   wr_latency=0
   *   bl=0 (16-byte wrap)
   */

  psram_ctrlr_ll_set_wr_cmd(PSRAM_CACHE_MSPI,
                             HEX_PSRAM_WR_CMD_BITLEN,
                             HEX_PSRAM_SYNC_WRITE);
  psram_ctrlr_ll_set_rd_cmd(PSRAM_CACHE_MSPI,
                             HEX_PSRAM_RD_CMD_BITLEN,
                             HEX_PSRAM_SYNC_READ);
  psram_ctrlr_ll_set_addr_bitlen(PSRAM_CACHE_MSPI,
                                  HEX_PSRAM_ADDR_BITLEN);
  psram_ctrlr_ll_enable_4byte_addr(PSRAM_CACHE_MSPI, true);
  /* Mode regs written: lt=1 (fixed), read_latency=2, wr_latency=2
   * Fixed latency rd_dummy = 2*(read_latency+1) = 2*(2+1) = 6
   * But ESP-IDF uses HEX_PSRAM_RD_DUMMY_BITLEN = 2*(10-1) = 18
   * Use same values as ESP-IDF for 200MHz speed.
   */

  /* Chip default: lt=0 (variable), read_latency=3, wr_latency=0
   * Variable rd_dummy = 2 * read_latency = 6
   * Variable wr_dummy = 2 * wr_latency = 0 (min 1)
   */

  psram_ctrlr_ll_set_rd_dummy(PSRAM_CACHE_MSPI,
                               HEX_PSRAM_RD_DUMMY_BITLEN);
  psram_ctrlr_ll_set_wr_dummy(PSRAM_CACHE_MSPI,
                               HEX_PSRAM_WR_DUMMY_BITLEN);
  psram_ctrlr_ll_enable_wr_dummy_level_control(PSRAM_CACHE_MSPI, true);

  /* DDR mode - set DQS_LOOP BEFORE enabling DDR */

  SET_PERI_REG_MASK(SPI_MEM_S_SMEM_DDR_REG,
                    SPI_MEM_S_SMEM_DDR_DQS_LOOP_M |
                    SPI_MEM_S_SMEM_VAR_DUMMY_M);
  psram_ctrlr_ll_enable_ddr_mode(PSRAM_CACHE_MSPI, true);
  psram_ctrlr_ll_enable_ddr_wr_data_swap(PSRAM_CACHE_MSPI, false);
  psram_ctrlr_ll_enable_ddr_rd_data_swap(PSRAM_CACHE_MSPI, false);

  /* x16 HEX mode (mode regs written via MSPI3) */

  psram_ctrlr_ll_enable_oct_line_mode(PSRAM_CACHE_MSPI, true);
  psram_ctrlr_ll_enable_hex_data_line_mode(PSRAM_CACHE_MSPI, true);

  /* Enable DQS loopback on MSPI2 cache controller */

  SET_PERI_REG_MASK(SPI_MEM_S_SMEM_DDR_REG,
                    SPI_MEM_S_SMEM_DDR_DQS_LOOP_M |
                    SPI_MEM_S_SMEM_VAR_DUMMY_M);

  /* Split transactions */

  psram_ctrlr_ll_enable_split_trans(PSRAM_CACHE_MSPI, true);
  psram_ctrlr_ll_set_page_size(PSRAM_CACHE_MSPI, 2048);

  /* AXI access - CRITICAL for L2 cache to access PSRAM */

  psram_ctrlr_ll_enable_axi_access(PSRAM_CACHE_MSPI, true);
  psram_ctrlr_ll_enable_wr_splice(PSRAM_CACHE_MSPI, true);
  psram_ctrlr_ll_enable_rd_splice(PSRAM_CACHE_MSPI, true);

  /* PMS - allow read/write access to entire PSRAM region */

  psram_ctrlr_ll_set_pms_region_start_addr(PSRAM_CACHE_MSPI, 0, 0);
  psram_ctrlr_ll_set_pms_region_size(PSRAM_CACHE_MSPI, 0, 4096);
  psram_ctrlr_ll_set_pms_region_attr(PSRAM_CACHE_MSPI, 0,
                                      PSRAM_CTRLR_LL_PMS_ATTR_WRITABLE |
                                      PSRAM_CTRLR_LL_PMS_ATTR_READABLE);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int IRAM_ATTR psram_enable(void)
{
  int ret;

  if (g_psram_initialized)
    {
      return OK;
    }

  ets_printf("PSRAM: init HEX mode\n");

  /* Step 1: Enable MPLL (only touches RTC I2C regs, safe) */

  rtc_clk_mpll_enable();
  rtc_clk_mpll_configure(HEX_PSRAM_XTAL_FREQ_MHZ,
                          HEX_PSRAM_MPLL_FREQ_MHZ, false);

  ets_printf("PSRAM: MPLL ok\n");

  /* Step 2: Configure PSRAM MSPI clocks.
   * Use direct register writes instead of HAL struct access
   * to avoid read-modify-write issues with shared registers.
   */

  /* Enable module clock */

  SET_PERI_REG_MASK(HP_SYS_CLKRST_SOC_CLK_CTRL0_REG,
                    HP_SYS_CLKRST_REG_PSRAM_SYS_CLK_EN_M);
  SET_PERI_REG_MASK(HP_SYS_CLKRST_PERI_CLK_CTRL00_REG,
                    HP_SYS_CLKRST_REG_PSRAM_PLL_CLK_EN_M |
                    HP_SYS_CLKRST_REG_PSRAM_CORE_CLK_EN_M);

  /* Select MPLL clock source */

  SET_PERI_REG_BITS(HP_SYS_CLKRST_PERI_CLK_CTRL00_REG,
                    HP_SYS_CLKRST_REG_PSRAM_CLK_SRC_SEL_V,
                    1,  /* MPLL */
                    HP_SYS_CLKRST_REG_PSRAM_CLK_SRC_SEL_S);

  ets_printf("PSRAM: clk src ok\n");

  /* Reset only APB bus (not AXI) to make SPI1_MEM_S accessible
   * after clock source change. AXI reset would disrupt L2 cache.
   */

  SET_PERI_REG_MASK(HP_SYS_CLKRST_HP_RST_EN0_REG,
                    HP_SYS_CLKRST_REG_RST_EN_DUAL_MSPI_APB_M);
  CLEAR_PERI_REG_MASK(HP_SYS_CLKRST_HP_RST_EN0_REG,
                      HP_SYS_CLKRST_REG_RST_EN_DUAL_MSPI_APB_M);

  ets_printf("PSRAM: apb reset ok\n");

  mspi_timing_ll_pin_drv_set(2);
  mspi_timing_ll_enable_dqs(true);

  psram_ctrlr_ll_set_cs_setup(PSRAM_CACHE_MSPI, HEX_PSRAM_CS_SETUP_TIME);
  psram_ctrlr_ll_set_cs_hold(PSRAM_CACHE_MSPI, HEX_PSRAM_CS_HOLD_TIME);
  psram_ctrlr_ll_set_cs_hold_delay(PSRAM_CACHE_MSPI,
                                    HEX_PSRAM_CS_HOLD_DELAY);

  psram_ctrlr_ll_enable_split_trans(PSRAM_CACHE_MSPI, true);
  psram_ctrlr_ll_set_page_size(PSRAM_CACHE_MSPI, 2048);

  ets_printf("PSRAM: cs+page ok\n");

  /* Set bus clock (MPLL=400MHz / 4 = 100MHz)
   * 200MHz (div=2) requires DLL + timing tuning which is not yet supported.
   */

  psram_ctrlr_ll_set_bus_clock(PSRAM_CACHE_MSPI, 4);
  psram_ctrlr_ll_set_bus_clock(PSRAM_USER_MSPI, 4);

  /* Disable DLL on both - use DQS_LOOP for DDR timing instead.
   * DLL causes MSPI3 command timeout and MSPI2 cache crash.
   */

  psram_ctrlr_ll_enable_dll(PSRAM_CACHE_MSPI, false);
  psram_ctrlr_ll_enable_dll(PSRAM_USER_MSPI, false);

  ets_printf("PSRAM: bus+dll ok\n");

  ets_delay_us(1000);

  /* Init PSRAM mode registers */

  ets_printf("PSRAM: init chip...\n");

  /* Init PSRAM mode registers via MSPI3 DDR commands.
   * Commands DO complete on the bus (confirmed by density=0x5 read)
   * but USR bit doesn't auto-clear in DDR mode, causing timeout.
   * This is OK - data is still transferred.
   */

  hex_psram_mode_reg_t mode_reg = {0};
  mode_reg.mr0.lt = 1;
  mode_reg.mr0.read_latency = HEX_PSRAM_RD_LATENCY;
  mode_reg.mr0.drive_str = 0;
  mode_reg.mr4.wr_latency = HEX_PSRAM_WR_LATENCY;
  mode_reg.mr8.bl = 3;
  mode_reg.mr8.bt = 0;
  mode_reg.mr8.rbx = 1;
  mode_reg.mr8.x16 = 1;

  psram_init_mode_reg(PSRAM_USER_MSPI, &mode_reg);

  ets_printf("PSRAM: mode ok\n");

#ifdef CONFIG_ESP32P4_SPIRAM_SIZE
  g_psram_size = CONFIG_ESP32P4_SPIRAM_SIZE;
#else
  g_psram_size = PSRAM_SIZE_32MB;
#endif

  /* Configure MSPI2 cache for HEX DDR access */

  psram_config_mspi();

  /* Keep 100MHz (div=4). 200MHz requires:
   * 1. PSRAM init in bootloader (before flash XIP)
   * 2. Full MSPI module reset (safe before XIP)
   * 3. DLL + mspi_timing_psram_tuning via working MSPI3
   * Currently MSPI3 DDR commands don't complete properly
   * after flash XIP is enabled, blocking timing tuning.
   */

  SET_PERI_REG_MASK(SPI_MEM_S_SMEM_DDR_REG,
                    SPI_MEM_S_SMEM_DDR_DQS_LOOP_M |
                    SPI_MEM_S_SMEM_VAR_DUMMY_M);

  psram_ctrlr_ll_enable_variable_dummy(PSRAM_CACHE_MSPI, true);
  psram_ctrlr_ll_enable_variable_dummy(PSRAM_USER_MSPI, true);

  ets_printf("PSRAM: %lu MB @ 100MHz\n",
             (unsigned long)(g_psram_size / (1024 * 1024)));

  g_psram_initialized = true;
  return OK;
}

int psram_get_physical_size(uint32_t *out_size)
{
  if (out_size == NULL)
    {
      return -EINVAL;
    }

  *out_size = g_psram_size;
  return OK;
}

int psram_get_available_size(uint32_t *out_size)
{
  if (out_size == NULL)
    {
      return -EINVAL;
    }

  *out_size = g_psram_size;
  return OK;
}
