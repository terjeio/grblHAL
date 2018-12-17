/*
 * Copyright (c) 2017-2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       MSP_EXP432E401Y.h
 *
 *  @brief      MSP_EXP432E401Y Board Specific APIs
 *
 *  The MSP_EXP432E401Y header file should be included in an application as
 *  follows:
 *  @code
 *  #include <MSP_EXP432E401Y.h>
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef __MSP_EXP432E401Y_H
#define __MSP_EXP432E401Y_H

#ifdef __cplusplus
extern "C" {
#endif

/* LEDs on MSP_EXP432E401Y are active high. */
#define MSP_EXP432E401Y_GPIO_LED_OFF (0)
#define MSP_EXP432E401Y_GPIO_LED_ON  (1)

/*!
 *  @def    MSP_EXP432E401Y_ADCName
 *  @brief  Enum of ADC channels on the MSP_EXP432E401Y dev board
 */
typedef enum MSP_EXP432E401Y_ADCName {
    MSP_EXP432E401Y_ADC0 = 0,
    MSP_EXP432E401Y_ADC1,

    MSP_EXP432E401Y_ADCCOUNT
} MSP_EXP432E401Y_ADCName;

/*!
 *  @def    MSP_EXP432E401Y_ADCBufName
 *  @brief  Enum of ADC hardware peripherals on the MSP_EXP432E401Y dev board
 */
typedef enum MSP_EXP432E401Y_ADCBufName {
    MSP_EXP432E401Y_ADCBUF0 = 0,

    MSP_EXP432E401Y_ADCBUFCOUNT
} MSP_EXP432E401Y_ADCBufName;

/*!
 *  @def    MSP_EXP432E401Y_ADCBuf0ChannelName
 *  @brief  Enum of ADCBuf channels on the MSP_EXP432E401Y dev board
 */
typedef enum MSP_EXP432E401Y_ADCBuf0ChannelName {
    MSP_EXP432E401Y_ADCBUF0CHANNEL0 = 0,
    MSP_EXP432E401Y_ADCBUF0CHANNEL1,
    MSP_EXP432E401Y_ADCBUF0CHANNEL2,
    MSP_EXP432E401Y_ADCBUF0CHANNEL3,
    MSP_EXP432E401Y_ADCBUF0CHANNEL4,

    MSP_EXP432E401Y_ADCBUF0CHANNELCOUNT
} MSP_EXP432E401Y_ADCBuf0ChannelName;

/*!
 *  @def    MSP_EXP432E401Y_CANName
 *  @brief  Enum of CAN controllers on the MSP_EXP432E401Y dev board
 */
typedef enum MSP_EXP432E401Y_CANName {
    MSP_EXP432E401Y_CAN0 = 0,
    MSP_EXP432E401Y_CAN1,

    MSP_EXP432E401Y_CANCOUNT
} MSP_EXP432E401Y_CANName;

/*!
 *  @def    MSP_EXP432E401Y_GPIOName
 *  @brief  Enum of LED names on the MSP_EXP432E401Y dev board
 */
typedef enum MSP_EXP432E401Y_GPIOName {
    MSP_EXP432E401Y_SDSPI_CS = 0,
    MSP_EXP432E401Y_GPIO_USR_SW1,
    MSP_EXP432E401Y_GPIO_USR_SW2,
    MSP_EXP432E401Y_SPI_MASTER_READY,
    MSP_EXP432E401Y_SPI_SLAVE_READY,
    MSP_EXP432E401Y_GPIO_D1,
    MSP_EXP432E401Y_GPIO_D2,


    /* Sharp LCD Pins */
    MSP_EXP432E401Y_LCD_CS,
    MSP_EXP432E401Y_LCD_POWER,
    MSP_EXP432E401Y_LCD_ENABLE,

    MSP_EXP432E401Y_GPIOCOUNT = 1
} MSP_EXP432E401Y_GPIOName;

/*!
 *  @def    MSP_EXP432E401Y_I2CName
 *  @brief  Enum of I2C names on the MSP_EXP432E401Y dev board
 */
typedef enum MSP_EXP432E401Y_I2CName {
    MSP_EXP432E401Y_I2C0 = 0,
    MSP_EXP432E401Y_I2C7,

    MSP_EXP432E401Y_I2CCOUNT
} MSP_EXP432E401Y_I2CName;

/*!
 *  @def    MSP_EXP432E401Y_NVSName
 *  @brief  Enum of NVS names on the MSP_EXP432E401Y dev board
 */
typedef enum MSP_EXP432E401Y_NVSName {
    MSP_EXP432E401Y_NVSMSP432E40 = 0,

    MSP_EXP432E401Y_NVSCOUNT
} MSP_EXP432E401Y_NVSName;

/*!
 *  @def    MSP_EXP432E401Y_PWMName
 *  @brief  Enum of PWM names on the MSP_EXP432E401Y dev board
 */
typedef enum MSP_EXP432E401Y_PWMName {
    MSP_EXP432E401Y_PWM0 = 0,

    MSP_EXP432E401Y_PWMCOUNT
} MSP_EXP432E401Y_PWMName;

/*!
 *  @def    MSP_EXP432E401Y_SDFatFSName
 *  @brief  Enum of SDFatFS names on the MSP_EXP432E401Y dev board
 */
typedef enum MSP_EXP432E401Y_SDFatFSName {
    MSP_EXP432E401Y_SDFatFS0 = 0,

    MSP_EXP432E401Y_SDFatFSCOUNT
} MSP_EXP432E401Y_SDFatFSName;

/*!
 *  @def    MSP_EXP432E401Y_SDName
 *  @brief  Enum of SD names on the MSP_EXP432E401Y dev board
 */
typedef enum MSP_EXP432E401Y_SDName {
    MSP_EXP432E401Y_SDSPI0 = 0,

    MSP_EXP432E401Y_SDCOUNT
} MSP_EXP432E401Y_SDName;

/*!
 *  @def    MSP_EXP432E401Y_SPIName
 *  @brief  Enum of SPI names on the MSP_EXP432E401Y dev board
 */
typedef enum MSP_EXP432E401Y_SPIName {
    MSP_EXP432E401Y_SPI2 = 0,
    MSP_EXP432E401Y_SPI3,

    MSP_EXP432E401Y_SPICOUNT
} MSP_EXP432E401Y_SPIName;

/*!
 *  @def    MSP_EXP432E401Y_TimerName
 *  @brief  Enum of Timer names on the MSP_EXP432E401Y dev board
 */
typedef enum MSP_EXP432E401Y_TimerName {
    MSP_EXP432E401Y_TIMER0 = 0,
    MSP_EXP432E401Y_TIMER1,
    MSP_EXP432E401Y_TIMER2,

    MSP_EXP432E401Y_TIMERCOUNT
} MSP_EXP432E401Y_TimerName;

/*!
 *  @def    MSP_EXP432E401Y_UARTName
 *  @brief  Enum of UARTs on the MSP_EXP432E401Y dev board
 */
typedef enum MSP_EXP432E401Y_UARTName {
    MSP_EXP432E401Y_UART0 = 0,
    MSP_EXP432E401Y_UART2,

    MSP_EXP432E401Y_UARTCOUNT
} MSP_EXP432E401Y_UARTName;

/*
 *  @def    MSP_EXP432E401Y_WatchdogName
 *  @brief  Enum of Watchdogs on the MSP_EXP432E401Y dev board
 */
typedef enum MSP_EXP432E401Y_WatchdogName {
    MSP_EXP432E401Y_WATCHDOG0 = 0,

    MSP_EXP432E401Y_WATCHDOGCOUNT
} MSP_EXP432E401Y_WatchdogName;

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings.
 *  This includes:
 *     - Enable clock sources for peripherals
 */
extern void MSP_EXP432E401Y_initGeneral(void);

#ifdef __cplusplus
}
#endif

#endif /* __MSP_EXP432E401Y_H */
