/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_spiram.c
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
#include <string.h>
#include <errno.h>
#include <debug.h>

#include "esp_psram.h"
#include "esp_spiram.h"

#include "esp_rom_sys.h"
#include "esp_attr.h"
#include "hal/cache_hal.h"
#include "hal/cache_ll.h"
#include "hal/mmu_hal.h"
#include "hal/mmu_types.h"
#include "soc/ext_mem_defs.h"
#include "rom/cache.h"

/* P4 has a dedicated MMU for PSRAM (mmu_id=1), separate from Flash (mmu_id=0) */

#define MMU_PSRAM_ID  1

extern int ets_printf(const char *fmt, ...);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPIRAM_VADDR_START    ESP_PSRAM_VADDR_START

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_spiram_inited;
static uint32_t g_spiram_size;
static uintptr_t g_allocable_vaddr_start;
static uintptr_t g_allocable_vaddr_end;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_spiram_init
 *
 * Description:
 *   Initialize the SPIRAM chip and detect its size.
 *
 ****************************************************************************/

int esp_spiram_init(void)
{
  int ret;

  if (g_spiram_inited)
    {
      return OK;
    }

  ret = psram_enable();
  if (ret != OK)
    {
      merr("PSRAM enable failed: %d\n", ret);
      return ret;
    }

  uint32_t psram_physical_size = 0;
  ret = psram_get_available_size(&psram_physical_size);
  if (ret != OK)
    {
      merr("PSRAM get size failed: %d\n", ret);
      return ret;
    }

  g_spiram_size = psram_physical_size;
  g_spiram_inited = true;

  minfo("Found %lu MB PSRAM device\n",
        psram_physical_size / (1024 * 1024));

  return OK;
}

/****************************************************************************
 * Name: esp_spiram_init_cache
 *
 * Description:
 *   Map the PSRAM into the CPU virtual address space via MMU/Cache.
 *
 ****************************************************************************/

int esp_spiram_init_cache(void)
{
  uint32_t actual_mapped_len = 0;
  uint32_t psram_vaddr_start = SPIRAM_VADDR_START;
  uint32_t map_size = g_spiram_size;

  ets_printf("SPIRAM: map vaddr=0x%08lx size=%lu\n",
             (unsigned long)psram_vaddr_start,
             (unsigned long)map_size);

  mmu_hal_map_region(MMU_PSRAM_ID, MMU_TARGET_PSRAM0,
                     psram_vaddr_start, 0,
                     map_size, &actual_mapped_len);

  ets_printf("SPIRAM: mmu ok, actual=%lu\n",
             (unsigned long)actual_mapped_len);

  cache_bus_mask_t bus_mask;
  bus_mask = cache_ll_l1_get_bus(0, psram_vaddr_start, map_size);
  cache_ll_l1_enable_bus(0, bus_mask);

  ets_printf("SPIRAM: bus ok\n");

  /* Invalidate L2 cache for the newly mapped PSRAM region */

  Cache_Invalidate_Addr(CACHE_MAP_L2_CACHE, psram_vaddr_start,
                        actual_mapped_len);

  g_allocable_vaddr_start = psram_vaddr_start;
  g_allocable_vaddr_end = psram_vaddr_start + actual_mapped_len;

  volatile uint32_t *test = (volatile uint32_t *)psram_vaddr_start;
  ets_printf("SPIRAM: test 0x%08lx\n", (unsigned long)psram_vaddr_start);
  *test = 0xdeadbeef;
  uint32_t rd = *test;
  ets_printf("SPIRAM: w=0xdeadbeef r=0x%08lx\n", (unsigned long)rd);

  if (rd != 0xdeadbeef)
    {
      ets_printf("SPIRAM: FAILED\n");
      g_allocable_vaddr_start = 0;
      g_allocable_vaddr_end = 0;
      return -EIO;
    }

  ets_printf("SPIRAM: %lu MB OK!\n",
             (unsigned long)(g_spiram_size / (1024 * 1024)));
  return OK;
}

/****************************************************************************
 * Name: esp_spiram_test
 *
 * Description:
 *   Simple read/write test over the PSRAM mapped region.
 *
 ****************************************************************************/

int esp_spiram_test(void)
{
  volatile uint32_t *psram = (volatile uint32_t *)g_allocable_vaddr_start;

  ets_printf("PSRAM test at 0x%08lx...\n",
             (unsigned long)g_allocable_vaddr_start);

  /* Simple single-word test first to check basic access */

  psram[0] = 0xdeadbeef;
  if (psram[0] != 0xdeadbeef)
    {
      ets_printf("PSRAM test FAILED: wrote 0xdeadbeef, read 0x%08lx\n",
                 (unsigned long)psram[0]);
      return -EIO;
    }

  /* Small pattern test: 256 words = 1KB */

  uint32_t i;
  for (i = 0; i < 256; i++)
    {
      psram[i] = i ^ 0xaaaaaaaa;
    }

  for (i = 0; i < 256; i++)
    {
      uint32_t expected = i ^ 0xaaaaaaaa;
      if (psram[i] != expected)
        {
          ets_printf("PSRAM test FAILED at %lu\n", (unsigned long)i);
          return -EIO;
        }
    }

  /* Clear test data */

  for (i = 0; i < 256; i++)
    {
      psram[i] = 0;
    }

  ets_printf("PSRAM test PASSED (%lu MB)\n",
             (unsigned long)(g_spiram_size / (1024 * 1024)));

  return OK;
}

/****************************************************************************
 * Name: esp_spiram_get_size
 ****************************************************************************/

uint32_t esp_spiram_get_size(void)
{
  return g_spiram_size;
}

/****************************************************************************
 * Name: esp_spiram_allocable_vaddr_start
 ****************************************************************************/

uintptr_t esp_spiram_allocable_vaddr_start(void)
{
  return g_allocable_vaddr_start;
}

/****************************************************************************
 * Name: esp_spiram_allocable_vaddr_end
 ****************************************************************************/

uintptr_t esp_spiram_allocable_vaddr_end(void)
{
  return g_allocable_vaddr_end;
}

/****************************************************************************
 * Name: esp_spiram_clear_region
 *
 * Description:
 *   Clear the allocable region so heap won't try to use PSRAM.
 *
 ****************************************************************************/

void esp_spiram_clear_region(void)
{
  g_allocable_vaddr_start = 0;
  g_allocable_vaddr_end = 0;
}
