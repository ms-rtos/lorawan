/*
 * Copyright (c) 2015-2020 ACOINFO Co., Ltd.
 * All rights reserved.
 *
 * Detailed license information can be found in the LICENSE file.
 *
 * File: ms_lora_sx1276_board.c LoRa sx1276 board driver.
 *
 * Author: Song.xiaolong
 *
 */
#include <ms_rtos.h>
#include "sx1276-board.h"
#include "delay.h"
#include "ms_lora_config.h"
#include "ms_lora_porting.h"

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio = {
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby,
    SX1276SetRx,
    SX1276StartCad,
    SX1276SetTxContinuousWave,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength,
    SX1276SetPublicNetwork,
    SX1276GetWakeupTime,
    NULL, /* IrqProcess,     available only on SX126x radios */
    NULL, /* RxBoosted,      available only on SX126x radios */
    NULL, /* SetRxDutyCycle, available only on SX126x radios */
};

/*!
 * \brief Initializes DIO IRQ handlers
 *
 * \param [IN] irqHandlers Array containing the IRQ callback functions
 */
void SX1276IoIrqInit( DioIrqHandler **irqHandlers )
{
    GpioSetInterrupt(&SX1276.DIO0, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[0]);
    GpioSetInterrupt(&SX1276.DIO1, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[1]);
    GpioSetInterrupt(&SX1276.DIO2, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[2]);
    GpioSetInterrupt(&SX1276.DIO3, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[3]);
    GpioSetInterrupt(&SX1276.DIO4, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[4]);

#if 0 /* 没有使用 */
    GpioSetInterrupt(&SX1276.DIO5, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[5]);
#endif

    lora_os_start();
}

/*!
 * \brief Resets the radio
 */
void SX1276Reset( void )
{
    SX1276SetBoardTcxo(true);               /* 使能 TXCO 模式               */

    GpioWrite(&SX1276.Reset, 0);            /* 复位引脚拉低                 */
    ms_thread_sleep_ms(1);

    GpioWrite(&SX1276.Reset, 1);            /* 复位引脚拉高                 */
    ms_thread_sleep_ms(6);
}

static uint8_t SX1276GetPaSelect( uint32_t channel )
{
    return  (RF_PACONFIG_PASELECT_PABOOST);
}

/*!
 * \brief Sets the radio output power.
 *
 * \param [IN] power Sets the RF output power
 */
void SX1276SetRfTxPower( int8_t power )
{
    uint8_t paCfg = 0;
    uint8_t paDac = 0;

    paCfg = SX1276Read(REG_PACONFIG);
    paDac = SX1276Read(REG_PADAC);

    paCfg = (paCfg & RF_PACONFIG_PASELECT_MASK) | SX1276GetPaSelect(SX1276.Settings.Channel);

    if ((paCfg & RF_PACONFIG_PASELECT_PABOOST) == RF_PACONFIG_PASELECT_PABOOST) {
        if (power > 17) {
            paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_ON;
        } else {
            paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_OFF;
        }

        if ((paDac & RF_PADAC_20DBM_ON) == RF_PADAC_20DBM_ON) {
            if (power < 5) {
                power = 5;
            }

            if (power > 20) {
                power = 20;
            }

            paCfg = (paCfg & RF_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t)((uint16_t)(power - 5) & 0x0F);

        } else {
            if (power < 2) {
                power = 2;
            }

            if (power > 17) {
                power = 17;
            }
            paCfg = (paCfg & RF_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t)((uint16_t)(power - 2) & 0x0F);
        }

    } else {
        if (power > 0) {
            if (power > 15) {
                power = 15;
            }
            paCfg = (paCfg & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK) | (7 << 4) | (power);

        } else {
            if (power < -4) {
                power = -4;
            }
            paCfg = (paCfg & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK) | (0 << 4) | (power + 4);
        }
    }

    SX1276Write(REG_PACONFIG, paCfg);
    SX1276Write(REG_PADAC,    paDac);
}

/*!
 * \brief Set the RF Switch I/Os pins in low power mode
 *
 * \param [IN] status enable or disable
 */
void SX1276SetAntSwLowPower( bool status )
{

}

/*!
 * \brief Controls the antenna switch if necessary.
 *
 * \remark see errata note
 *
 * \param [IN] opMode Current radio operating mode
 */
void SX1276SetAntSw( uint8_t opMode )
{
    /*
     * 控制天线引脚切换收发天线，不需要实现, 硬件自动切换
     */
}

/*!
 * \brief Enables/disables the TCXO if available on board design.
 *
 * \param [IN] state TCXO enabled when true and disabled when false.
 */
void SX1276SetBoardTcxo( uint8_t state )
{
    /*
     * 未使用 Board TCXO
     */
}

/*!
 * \brief Gets the Defines the time required for the TCXO to wakeup [ms].
 *
 * \retval time Board TCXO wakeup time in ms.
 */
uint32_t SX1276GetBoardTcxoWakeupTime( void )
{
    /*
     * 未使用 Board TCXO
     */
    return  (0);
}

/*!
 * \brief Checks if the given RF frequency is supported by the hardware
 *
 * \param [IN] frequency RF frequency to be checked
 * \retval isSupported [true: supported, false: unsupported]
 */
bool SX1276CheckRfFrequency( uint32_t frequency )
{
    /*
     * 所有的频率都支持，不需要检查
     */
    return  (true);
}

/*!
 * \brief Get the current battery level
 *
 * \retval value  battery level [  0: USB,
 *                                 1: Min level,
 *                                 x: level
 *                               254: fully charged,
 *                               255: Error]
 */
uint8_t BoardGetBatteryLevel( void )
{
    return 0;
}

/*!
 * \brief Initializes the mcu.
 */
void BoardInitMcu( void )
{
    int fd;

    lora_os_init();

    fd = ms_io_open("/dev/spi1", O_RDWR, 0666);
    return_if_fail(fd >= 0);

    SpiInit(&SX1276.Spi, fd, "/dev/lora");

    GpioInit(&SX1276.DIO0,  "/dev/ldio0", PIN_INPUT,  PIN_PUSH_PULL, PIN_PULL_UP, 0);
    GpioInit(&SX1276.DIO1,  "/dev/ldio1", PIN_INPUT,  PIN_PUSH_PULL, PIN_PULL_UP, 0);
    GpioInit(&SX1276.DIO2,  "/dev/ldio2", PIN_INPUT,  PIN_PUSH_PULL, PIN_PULL_UP, 0);
    GpioInit(&SX1276.DIO3,  "/dev/ldio3", PIN_INPUT,  PIN_PUSH_PULL, PIN_PULL_UP, 0);
    GpioInit(&SX1276.DIO4,  "/dev/ldio4", PIN_INPUT,  PIN_PUSH_PULL, PIN_PULL_UP, 0);
    GpioInit(&SX1276.Reset, "/dev/lrst",  PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);
}

/*!
 * \brief Initializes the boards peripherals.
 */
void BoardInitPeriph( void )
{
}

/*!
 * \brief De-initializes the target board peripherals to decrease power
 *        consumption.
 */
void BoardDeInitMcu( void )
{
    SpiDeInit(&SX1276.Spi);

    GpioDeInit(&SX1276.DIO0);
    GpioDeInit(&SX1276.DIO1);
    GpioDeInit(&SX1276.DIO2);
    GpioDeInit(&SX1276.DIO3);
    GpioDeInit(&SX1276.DIO4);
    GpioDeInit(&SX1276.Reset);

    lora_os_deinit();
}

/*!
 * \brief Gets the board 64 bits unique ID
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId( uint8_t *id )
{
}
