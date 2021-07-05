/*
 * Copyright (c) 2015-2020 ACOINFO Co., Ltd.
 * All rights reserved.
 *
 * Detailed license information can be found in the LICENSE file.
 *
 * File: ms_lora_porting.c LoRa MS-RTOS porting.
 *
 * Author: Song.xiaolong
 *
 */

#ifndef MS_LORA_PORTING_H
#define MS_LORA_PORTING_H

#ifdef __cplusplus
extern "C" {
#endif

#define return_if_fail(p)                               \
    if (!(p)) {                                         \
        ms_printf("%s:%d " #p "\n", __func__, __LINE__);\
        return;                                         \
    }

#define return_value_if_fail(p, value)                  \
    if (!(p)) {                                         \
        ms_printf("%s:%d " #p "\n", __func__, __LINE__);\
        return (value);                                 \
    }

#define goto_error_if_fail(p, err)                      \
    if (!(p)) {                                         \
        ms_printf("%s:%d " #p "\n", __func__, __LINE__);\
        goto err;                                       \
    }

/*!
 * \brief LoRa Initializes OS
 */
int lora_os_init( void );

/*!
 * \brief LoRa Start OS
 */
int lora_os_start( void );

/*!
 * \brief De-initialize OS
 */
void lora_os_deinit( void );

#ifdef __cplusplus
}
#endif

#endif /* MS_LORA_PORTING_H */
