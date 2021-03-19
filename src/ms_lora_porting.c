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

#include <ms_rtos.h>
#include <stdint.h>
#include "timer.h"
#include "gpio.h"
#include "spi.h"
#include "ms_lora_config.h"
#include "ms_lora_porting.h"

       ms_handle_t  lora_mutex_id;
static ms_handle_t  lora_timer_id;
static ms_handle_t  lora_sleep_sembid;
static ms_handle_t  lora_thread_id;
static ms_bool_t    lora_thread_quit;
static uint32_t     timer_saved_time;
static uint32_t     timer_bkup_data[2];
static MS_LIST_HEAD(lora_gpio_list);

#define __LORA_LOCK()               ms_mutex_lock(lora_mutex_id, MS_TIMEOUT_FOREVER)
#define __LORA_UNLOCK()             ms_mutex_unlock(lora_mutex_id)
#define __LORA_LOW_POWER_EXIT()     ms_semb_post(lora_sleep_sembid)

/*!
 * \brief Manages the entry into CPU deep-sleep mode
 */
void BoardLowPowerHandler( void )
{
    __LORA_UNLOCK();

    ms_semb_trywait(lora_sleep_sembid);
    ms_semb_wait(lora_sleep_sembid, MS_TIMEOUT_FOREVER);

    __LORA_LOCK();
}

/*!
 * \brief Blocking delay of "ms" milliseconds
 *
 * \param [IN] ms    delay in milliseconds
 */
void DelayMsMcu( uint32_t ms )
{
    ms_thread_sleep_ms(ms);
}

/*
 * Software timer service routine
 */
static void RtcTimerHandler( ms_ptr_t arg )
{
    __LORA_LOW_POWER_EXIT();
    TimerIrqHandler();
}

/*
 * Returns the minimum timeout value
 */
uint32_t RtcGetMinimumTimeout( void )
{
    return 1;
}

/*
 * converts time in ms to time in ticks
 */
uint32_t RtcMs2Tick( TimerTime_t milliseconds )
{
    return MS_MS_TO_TICK(milliseconds);
}

/*!
 * \brief converts time in ticks to time in ms
 *
 * \param[IN] time in timer ticks
 * \retval returns time in milliseconds
 */
TimerTime_t RtcTick2Ms( uint32_t tick )
{
    return MS_TICK_TO_MS(tick);
}

/*!
 * \brief Sets the alarm
 *
 * \note The alarm is set at now (read in this funtion) + timeout
 *
 * \param timeout [IN] Duration of the Timer ticks
 */
void RtcDelayMs( TimerTime_t milliseconds )
{
    ms_thread_sleep_ms(milliseconds);
}

/*!
 * \brief Sets the alarm
 *
 * \note The alarm is set at now (read in this funtion) + timeout
 *
 * \param timeout [IN] Duration of the Timer ticks
 */
void RtcSetAlarm( uint32_t timeout )
{
    ms_timer_start(lora_timer_id, timeout, 0U, MS_TIMER_OPT_ONE_SHOT);
}

/*!
 * \brief Stops the Alarm
 */
void RtcStopAlarm( void )
{
    ms_timer_stop(lora_timer_id);
}

/*!
 * \brief Sets the RTC timer reference
 *
 * \retval value Timer reference value in ticks
 */
uint32_t RtcSetTimerContext( void )
{
    timer_saved_time = (uint32_t)ms_time_get();

    return timer_saved_time;
}

/*!
 * \brief Gets the RTC timer reference
 *
 * \retval value Timer value in ticks
 */
uint32_t RtcGetTimerContext( void )
{
    return timer_saved_time;
}

/*!
 * \brief Gets the system time with the number of seconds elapsed since epoch
 *
 * \param [OUT] milliseconds Number of milliseconds elapsed since epoch
 * \retval seconds Number of seconds elapsed since epoch
 */
uint32_t RtcGetCalendarTime( uint16_t *milliseconds )
{
    ms_timeval_t tv;

    ms_gettimeofday(&tv);

    if (milliseconds != MS_NULL) {
        *milliseconds = tv.tv_usec / 1000;
    }

    return tv.tv_sec;
}

/*!
 * \brief Get the RTC timer value
 *
 * \retval RTC Timer value
 */
uint32_t RtcGetTimerValue( void )
{
    return ms_time_get();
}

/*!
 * \brief Get the RTC timer elapsed time since the last Alarm was set
 *
 * \retval RTC Elapsed time since the last alarm in ticks.
 */
uint32_t RtcGetTimerElapsedTime( void )
{
    return ms_time_get() - timer_saved_time;
}

/*!
 * \brief Writes data0 and data1 to the RTC backup registers
 *
 * \param [IN] data0 1st Data to be written
 * \param [IN] data1 2nd Data to be written
 */
void RtcBkupWrite( uint32_t data0, uint32_t data1 )
{
    timer_bkup_data[0] = data0;
    timer_bkup_data[1] = data1;
}

/*!
 * \brief Reads data0 and data1 from the RTC backup registers
 *
 * \param [OUT] data0 1st Data to be read
 * \param [OUT] data1 2nd Data to be read
 */
void RtcBkupRead( uint32_t* data0, uint32_t* data1 )
{
    if (data0 != MS_NULL) {
        *data0 = timer_bkup_data[0];
    }

    if (data1 != MS_NULL) {
        *data1 = timer_bkup_data[1];
    }
}

/*!
 * \brief Processes pending timer events
 */
void RtcProcess( void )
{
}

/*
 *  Computes the temperature compensation for a period of time on a
 *  specific temperature.
 */
TimerTime_t RtcTempCompensation( TimerTime_t period, float temperature )
{
    return 0;
}

/*!
 * \brief LoRa GPIO interrupt service thread
 *
 * \param [IN] arg    Argument
 */
static void GpioServiceThread( ms_ptr_t arg )
{
    ms_list_head_t *itervar;
    struct timeval  tv;
    Gpio_t         *gpio;
    fd_set          rfds;
    int             ret;
    int             max_fd = 0;

    while (!lora_thread_quit) {
        FD_ZERO(&rfds);

        __LORA_LOCK();
        ms_list_for_each(itervar, &lora_gpio_list) {
            gpio = MS_CONTAINER_OF(itervar, Gpio_t, node);
            FD_SET(gpio->fd, &rfds);
            max_fd = MS_MAX(max_fd, gpio->fd);
        }
        __LORA_UNLOCK();

        tv.tv_sec  = 20;
        tv.tv_usec = 0;
        ret = select(max_fd + 1, &rfds, MS_NULL, MS_NULL, &tv);
        if (ret < 0) {
            ms_printf("lorawan: gpio server failed to select!\n");

        } else {
            __LORA_LOCK();
            ms_list_for_each(itervar, &lora_gpio_list) {
                gpio = MS_CONTAINER_OF(itervar, Gpio_t, node);
                if (FD_ISSET(gpio->fd, &rfds)) {
                    if (gpio->IrqHandler) {
                        gpio->IrqHandler(gpio->Context);
                    }

                    __LORA_LOW_POWER_EXIT();
                }
            }
            __LORA_UNLOCK();
        }
    }
}

/*!
 * \brief Initializes the given GPIO object
 *
 * \param [IN] obj    Pointer to the GPIO object
 * \param [IN] pin    Pin name ( please look in pinName-board.h file )
 * \param [IN] mode   Pin mode [PIN_INPUT, PIN_OUTPUT,
 *                              PIN_ALTERNATE_FCT, PIN_ANALOGIC]
 * \param [IN] config Pin config [PIN_PUSH_PULL, PIN_OPEN_DRAIN]
 * \param [IN] type   Pin type [PIN_NO_PULL, PIN_PULL_UP, PIN_PULL_DOWN]
 * \param [IN] value  Default output value at initialization
 */
void GpioInit( Gpio_t *obj, PinNames pin_name, PinModes mode,  PinConfigs config, PinTypes type, uint32_t value )
{
    ms_gpio_param_t param;

    return_if_fail(obj != NULL);

    obj->fd = ms_io_open(pin_name, O_RDWR, 0666);
    return_if_fail(obj->fd >= 0);

    ms_io_ioctl(obj->fd, MS_GPIO_CMD_GET_PARAM, &param);
    if (mode == PIN_INPUT) {
        param.mode = MS_GPIO_MODE_INPUT;
    } else if (mode == PIN_OUTPUT) {
        param.mode = MS_GPIO_MODE_OUTPUT_PP;
    }

    if (config == PIN_PUSH_PULL) {
        if (type == PIN_NO_PULL) {
            param.pull = MS_GPIO_PULL_NONE;
        } else if (type == PIN_PULL_UP) {
            param.pull = MS_GPIO_PULL_UP;
        } else if (type == PIN_PULL_DOWN) {
            param.pull = MS_GPIO_PULL_DOWN;
        }
    } else {
        param.mode = MS_GPIO_MODE_OUTPUT_OD;
    }

    param.speed = MS_GPIO_SPEED_VERY_HIGH;

    ms_io_ioctl(obj->fd, MS_GPIO_CMD_SET_PARAM, &param);

    ms_io_write(obj->fd, &value, sizeof(value));

    MS_LIST_HEAD_INIT(&obj->node);
}

/*!
 * \brief De-initializes the given GPIO object
 *
 * \param [IN] obj    Pointer to the GPIO object
 */
void GpioDeInit( Gpio_t *obj )
{
    return_if_fail(obj != NULL);
    return_if_fail(obj->fd >= 0);

    GpioRemoveInterrupt(obj);
    ms_io_close(obj->fd);
    obj->fd = -1;
}

/*!
 * \brief Sets a user defined object pointer
 *
 * \param [IN] context User defined data object pointer to pass back
 *                     on IRQ handler callback
 */
void GpioSetContext( Gpio_t *obj, void* context )
{
    return_if_fail(obj != NULL);
    return_if_fail(obj->fd >= 0);

    __LORA_LOCK();
    obj->Context = context;
    __LORA_UNLOCK();
}

/*!
 * \brief GPIO IRQ Initialization
 *
 * \param [IN] obj         Pointer to the GPIO object
 * \param [IN] irqMode     IRQ mode [NO_IRQ, IRQ_RISING_EDGE,
 *                                   IRQ_FALLING_EDGE, IRQ_RISING_FALLING_EDGE]
 * \param [IN] irqPriority IRQ priority [IRQ_VERY_LOW_PRIORITY, IRQ_LOW_PRIORITY
 *                                       IRQ_MEDIUM_PRIORITY, IRQ_HIGH_PRIORITY
 *                                       IRQ_VERY_HIGH_PRIORITY]
 * \param [IN] irqHandler  Callback function pointer
 */
void GpioSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
{
    ms_gpio_param_t  param;

    return_if_fail(obj != NULL);
    return_if_fail(obj->fd >= 0);

    if (irqMode == NO_IRQ) {
        GpioRemoveInterrupt(obj);

    } else {
        return_if_fail(irqHandler != NULL);

        ms_io_ioctl(obj->fd, MS_GPIO_CMD_GET_PARAM, &param);
        if (irqMode == IRQ_RISING_EDGE) {
            param.mode = MS_GPIO_MODE_IRQ_RISING;
        } else if (irqMode == IRQ_FALLING_EDGE) {
            param.mode = MS_GPIO_MODE_IRQ_FALLING;
        } else if (irqMode == IRQ_RISING_FALLING_EDGE) {
            param.mode = MS_GPIO_MODE_IRQ_BOTH;
        }
        ms_io_ioctl(obj->fd, MS_GPIO_CMD_SET_PARAM, &param);

        __LORA_LOCK();
        obj->IrqHandler = irqHandler;
        ms_list_add_tail(&obj->node, &lora_gpio_list);
        __LORA_UNLOCK();
    }
}

/*!
 * \brief Removes the interrupt from the object
 *
 * \param [IN] obj Pointer to the GPIO object
 */
void GpioRemoveInterrupt( Gpio_t *obj )
{
    ms_gpio_param_t  param;

    return_if_fail(obj != NULL);
    return_if_fail(obj->fd >= 0);

    ms_io_ioctl(obj->fd, MS_GPIO_CMD_GET_PARAM, &param);
    param.mode = 0;
    ms_io_ioctl(obj->fd, MS_GPIO_CMD_SET_PARAM, &param);

    __LORA_LOCK();
    obj->IrqHandler = MS_NULL;
    ms_list_del_init(&obj->node);
    __LORA_UNLOCK();
}

/*!
 * \brief Writes the given value to the GPIO output
 *
 * \param [IN] obj   Pointer to the GPIO object
 * \param [IN] value New GPIO output value
 */
void GpioWrite( Gpio_t *obj, uint32_t value )
{
    return_if_fail(obj != NULL);

    ms_io_write(obj->fd, &value, sizeof(value));
}

/*!
 * \brief Toggle the value to the GPIO output
 *
 * \param [IN] obj   Pointer to the GPIO object
 */
void GpioToggle( Gpio_t *obj )
{
    uint32_t value;

    return_if_fail(obj != NULL);
    return_if_fail(obj->fd >= 0);

    ms_io_read(obj->fd, &value, sizeof(value));
    value = !value;
    ms_io_write(obj->fd, &value, sizeof(value));
}

/*!
 * \brief Reads the current GPIO input value
 *
 * \param [IN] obj Pointer to the GPIO object
 * \retval value   Current GPIO input value
 */
uint32_t GpioRead( Gpio_t *obj )
{
    uint32_t value;

    return_value_if_fail(obj != NULL, -1);
    return_value_if_fail(obj->fd >= 0, -1);

    ms_io_read(obj->fd, &value, sizeof(value));

    return value;
}

/*!
 * \brief Initializes the SPI object and MCU peripheral
 *
 * \remark When NSS pin is software controlled set the pin name to NC otherwise
 *         set the pin name to be used.
 *
 * \param [IN] obj       SPI object
 * \param [IN] spiId     SPI FD
 * \param [IN] dev_name  SPI device name
 */
void SpiInit( Spi_t *obj, SpiId_t spiId, const char *dev_name )
{
    return_if_fail(obj != NULL);
    return_if_fail(spiId >= 0);

    obj->fd = spiId;

    if( dev_name != MS_NULL ) {
        ms_io_ioctl(obj->fd, MS_SPI_CMD_SEL_DEV, (ms_ptr_t)dev_name);
        SpiFormat( obj, MS_SPI_DATA_SIZE_8BIT, MS_SPI_CLK_POLARITY_LOW, MS_SPI_CLK_PHASE_1EDGE, 0 );
    } else {
        SpiFormat( obj, MS_SPI_DATA_SIZE_8BIT, MS_SPI_CLK_POLARITY_LOW, MS_SPI_CLK_PHASE_1EDGE, 1 );
    }
}

/*!
 * \brief De-initializes the SPI object and MCU peripheral
 *
 * \param [IN] obj SPI object
 */
void SpiDeInit( Spi_t *obj )
{
    return_if_fail(obj != NULL);
    return_if_fail(obj->fd >= 0);

    ms_io_close(obj->fd);
}

/*!
 * \brief Configures the SPI peripheral
 *
 * \remark Slave mode isn't currently handled
 *
 * \param [IN] obj   SPI object
 * \param [IN] bits  Number of bits to be used. [8 or 16]
 * \param [IN] cpol  Clock polarity
 * \param [IN] cpha  Clock phase
 * \param [IN] slave When set the peripheral acts in slave mode
 */
void SpiFormat( Spi_t *obj, int8_t bits, int8_t cpol, int8_t cpha, int8_t slave )
{
    ms_spi_param_t param;

    return_if_fail(obj != NULL);
    return_if_fail(obj->fd >= 0);

    ms_io_ioctl(obj->fd, MS_SPI_CMD_GET_PARAM, &param);
    param.baud_rate      = 2000000;
    param.direction      = MS_SPI_DIRECTION_2LINES;
    param.data_size      = bits;
    param.frame_mode     = cpol | cpha | MS_SPI_FIRST_BIT_MSB | MS_SPI_TI_MODE_DISABLE | MS_SPI_CRC_CALC_DISABLE;
    param.crc_polynomial = 7;

    if( slave == 1 ) {
        param.mode = MS_SPI_MODE_SLAVE;
    } else {
        param.nss  = MS_SPI_NSS_SOFT;
        param.mode = MS_SPI_MODE_MASTER;
    }

    ms_io_ioctl(obj->fd, MS_SPI_CMD_SET_PARAM, &param);
}

/*!
 * \brief Sets the SPI speed
 *
 * \param [IN] obj SPI object
 * \param [IN] hz  SPI clock frequency in hz
 */
void SpiFrequency( Spi_t *obj, uint32_t hz )
{
    return_if_fail(obj != NULL);
    return_if_fail(obj->fd >= 0);
}

/*!
 * \brief SPI message transfer
 *
 * \param [IN] obj     SPI object
 * \param [IN] msg     SPI message
 * \param [IN] size    SPI message number
 *
 * \retval Send length
 */
uint16_t SpiInOut( Spi_t *obj, const ms_spi_msg_t *msg, int n_msg )
{
    uint16_t len = sizeof(ms_spi_msg_t) * n_msg;

    return_value_if_fail(obj != NULL, -1);
    return_value_if_fail(obj->fd >= 0, -1);
    return_value_if_fail(msg != NULL, -1);
    return_value_if_fail(n_msg >= 0, -1);
    return_value_if_fail(ms_io_write(obj->fd, msg, len) == len, -1)

    return len;
}

/*!
 * \brief LoRa Initializes OS
 */
int lora_os_init( void )
{
    int ret;

    /*
     * Create global LoRa lock
     */
    ret = ms_mutex_create("lora_mutex", MS_WAIT_TYPE_PRIO, &lora_mutex_id);
    return_value_if_fail(ret == MS_ERR_NONE, -1);

    /*
     * Create a software timer
     */
    ret = ms_timer_create("lora_timer", RtcTimerHandler, MS_NULL, &lora_timer_id);
    goto_error_if_fail(ret == MS_ERR_NONE, __error_handle0);

    ret = ms_semb_create("lora_sleep_semb", MS_FALSE, MS_WAIT_TYPE_PRIO, &lora_sleep_sembid);
    goto_error_if_fail(ret == MS_ERR_NONE, __error_handle1);

    lora_thread_quit = MS_FALSE;

    /*
     * Create LoRa OS thread
     */
    ret = ms_thread_init("t_lora", GpioServiceThread, MS_NULL,
                         LORA_CFG_THREAD_STK_SIZE, LORA_CFG_THREAD_PRIO, LORA_CFG_THREAD_TIME_SLICE,
                         LORA_CFG_THREAD_OPT, &lora_thread_id);
    goto_error_if_fail(ret == MS_ERR_NONE, __error_handle2);

    return 0;

__error_handle2:
    ms_semb_destroy(lora_sleep_sembid);

__error_handle1:
    ms_timer_destroy(lora_timer_id);

__error_handle0:
    ms_mutex_destroy(lora_mutex_id);

    return -1;
}

/*!
 * \brief LoRa Start OS
 */
int lora_os_start( void )
{
    ms_thread_resume(lora_thread_id);

    return 0;
}

/*!
 * \brief De-initialize OS
 */
void lora_os_deinit( void )
{
    lora_thread_quit = MS_TRUE;

    ms_timer_destroy(lora_timer_id);
    ms_mutex_destroy(lora_mutex_id);
}
