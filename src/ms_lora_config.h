/*
 * Copyright (c) 2015-2020 ACOINFO Co., Ltd.
 * All rights reserved.
 *
 * Detailed license information can be found in the LICENSE file.
 *
 * File: ms_lora_config.h LoRa MS-RTOS configuration.
 *
 * Author: Song.xiaolong
 *
 */

#ifndef MS_LORA_CONFIG_H
#define MS_LORA_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define ACTIVE_REGION                   LORAMAC_REGION_CN470

#define LORA_CFG_DEV_EUI                {0x47, 0x36, 0x54, 0x9f, 0x00, 0x31, 0x00, 0x05}
#define LORA_CFG_JOIN_EUI               {0x52, 0x69, 0x73, 0x69, 0x6e, 0x67, 0x48, 0x46}
#define LORA_CFG_APP_KEY                {0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, \
                                         0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c}
#define LORA_CFG_SE_PIN                 {0xFF, 0x00, 0x00, 0xE5}
#define LORA_CFG_CHAN_MASK              {MS_BIT(0), 0, 0, 0, 0, 0}

#define LORA_CFG_THREAD_STK_SIZE        4096U
#define LORA_CFG_THREAD_PRIO            16U
#define LORA_CFG_THREAD_TIME_SLICE      0U
#define LORA_CFG_THREAD_OPT             (MS_THREAD_OPT_REENT_EN | MS_THREAD_OPT_USER)

#ifdef __cplusplus
}
#endif

#endif /* MS_LORA_CONFIG_H */
