/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_spiram.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_SPIRAM_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_SPIRAM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: esp_spiram_init
 *
 * Description:
 *   Initialize the SPIRAM chip and detect its size.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   OK on success; a negated errno on failure.
 *
 ****************************************************************************/

int esp_spiram_init(void);

/****************************************************************************
 * Name: esp_spiram_init_cache
 *
 * Description:
 *   Map the PSRAM into the CPU virtual address space via MMU/Cache.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   OK on success; a negated errno on failure.
 *
 ****************************************************************************/

int esp_spiram_init_cache(void);

/****************************************************************************
 * Name: esp_spiram_test
 *
 * Description:
 *   Simple read/write test over the PSRAM mapped region.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   OK on success; a negated errno on failure.
 *
 ****************************************************************************/

int esp_spiram_test(void);

/****************************************************************************
 * Name: esp_spiram_get_size
 *
 * Description:
 *   Get the usable size of the PSRAM.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   PSRAM size in bytes.
 *
 ****************************************************************************/

uint32_t esp_spiram_get_size(void);

/****************************************************************************
 * Name: esp_spiram_allocable_vaddr_start
 *
 * Description:
 *   Get the start virtual address of the allocable PSRAM region.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Start virtual address.
 *
 ****************************************************************************/

uintptr_t esp_spiram_allocable_vaddr_start(void);

/****************************************************************************
 * Name: esp_spiram_allocable_vaddr_end
 *
 * Description:
 *   Get the end virtual address of the allocable PSRAM region.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   End virtual address.
 *
 ****************************************************************************/

uintptr_t esp_spiram_allocable_vaddr_end(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_SPIRAM_H */
