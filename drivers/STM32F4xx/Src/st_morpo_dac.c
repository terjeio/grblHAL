/*
  st_morpho_dac.c - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2020 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"

#if defined(BOARD_MORPHO_DAC_CNC)

#include <string.h>

#include "main.h"
#include "grbl/protocol.h"
#include "grbl/settings.h"

I2S_HandleTypeDef hi2s1;
DMA_HandleTypeDef hdma_spi1_tx;

static driver_setting_ptrs_t driver_settings;
static stepper_pulse_start_ptr stepper_pulse_start;
static settings_changed_ptr settings_changed;
uint16_t data[4];
int32_t count[2];

static const setting_group_detail_t aux_groups [] = {
    { Group_Root, Group_AuxPorts, "Aux ports"}
};

static const setting_detail_t aux_settings[] = {
    { Settings_IoPort_InvertIn, Group_AuxPorts, "Invert I/O Port inputs", NULL, Format_Bitfield, "Port 0,Port 1", NULL, NULL },
//    { Settings_IoPort_Pullup_Disable, Group_AuxPorts, "I/O Port inputs pullup disable", NULL, Format_Bitfield, "Port 0,Port 1,Port 2,Port 3,Port 4,Port 5,Port 6,Port 7", NULL, NULL },
    { Settings_IoPort_InvertOut, Group_AuxPorts, "Invert I/O Port outputs", NULL, Format_Bitfield, "Port 0", NULL, NULL },
//    { Settings_IoPort_OD_Enable, Group_AuxPorts, "I/O Port outputs as open drain", NULL, Format_Bitfield, "Port 0,Port 1,Port 2,Port 3,Port 4,Port 5,Port 6,Port 7", NULL, NULL }
};

static setting_details_t details = {
    .groups = aux_groups,
    .n_groups = sizeof(aux_groups) / sizeof(setting_group_detail_t),
    .settings = aux_settings,
    .n_settings = sizeof(aux_settings) / sizeof(setting_detail_t)
};

static setting_details_t *onReportSettings (void)
{
    return &details;
}

static status_code_t aux_settings_set (setting_type_t setting, float value, char *svalue)
{
    status_code_t status = Status_OK;

    switch(setting) {

        case Settings_IoPort_InvertIn:
            settings.ioport.invert_in.mask = (uint8_t)value & 0xFF;
            break;
/*
        case Settings_IoPort_Pullup_Disable:
            settings.ioport.pullup_disable_in.mask = (uint8_t)(int_value & 0xFF);
            break;
*/
        case Settings_IoPort_InvertOut:
            settings.ioport.invert_out.mask = (uint8_t)value & 0xFF;
            break;
/*
        case Settings_IoPort_OD_Enable:
            settings.ioport.od_enable_out.mask = (uint8_t)(int_value & 0xFF);
            break;
*/
        default:
            status = Status_Unhandled;
            break;
    }


    if(status == Status_OK)
        settings_write_global();

    return status == Status_Unhandled && driver_settings.set ? driver_settings.set(setting, value, svalue) : status;
}

static void aux_settings_report (setting_type_t setting)
{
    bool reported = true;

    switch(setting) {

        case Settings_IoPort_InvertIn:
            if(hal.port.num_digital_in)
                report_uint_setting(Settings_IoPort_InvertIn, settings.ioport.invert_in.mask);
            break;
/*
        case Settings_IoPort_Pullup_Disable:
            if(hal.port.num_digital_in)
                report_uint_setting(Settings_IoPort_Pullup_Disable, settings.ioport.pullup_disable_in.mask);
            break;
*/
        case Settings_IoPort_InvertOut:
            if(hal.port.num_digital_out)
                report_uint_setting(Settings_IoPort_InvertOut, settings.ioport.invert_out.mask);
            break;
/*
        case Settings_IoPort_OD_Enable:
            if(hal.port.num_digital_out)
                report_uint_setting(Settings_IoPort_OD_Enable, settings.ioport.od_enable_out.mask);
            break;
*/
        default:
            reported = false;
            break;
    }

    if(!reported && driver_settings.report)
        driver_settings.report(setting);
}
/*
static void aux_settings_restore (void)
{

    if(driver_settings.restore)
        driver_settings.restore();
}

static void aux_settings_load (void)
{
    if(driver_settings.load)
        driver_settings.load();
}
*/
static void digital_out (uint8_t port, bool on)
{
    switch(port) {
        case 0:
            BITBAND_PERI(AUXOUTPUT0_PORT->ODR, AUXOUTPUT0_PIN) = settings.ioport.invert_out.bit0 ? !on : on;
            break;

        case 1:
            BITBAND_PERI(AUXOUTPUT1_PORT->ODR, AUXOUTPUT1_PIN) = settings.ioport.invert_out.bit1 ? !on : on;
            break;
    }
}
/*
inline static __attribute__((always_inline)) int32_t get_input(gpio_t *gpio, wait_mode_t wait_mode, float timeout)
{
    uint_fast16_t delay = wait_mode == WaitMode_Immediate || timeout == 0.0f ? 0 : (uint_fast16_t)ceilf((1000.0f / 50.0f) * timeout) + 1;

    bool wait_for = wait_mode != WaitMode_Low;

    do {
        if(!!(gpio->reg->DR & gpio->bit) == wait_for)
            return !!(gpio->reg->DR & gpio->bit);

        if(delay) {
            protocol_execute_realtime();
            hal.delay_ms(50, NULL);
        } else
            break;
    } while(--delay && !sys.abort);

    return -1;
}
*/
static int32_t wait_on_input (bool digital, uint8_t port, wait_mode_t wait_mode, float timeout)
{
    int32_t value = -1;
/*
    if(digital) {
        switch(port) {
            case 0:
                value = get_input(&st0, wait_mode, timeout);
                break;

            case 1:
                value = get_input(&st1, wait_mode, timeout);
                break;
        }
    }
*/
    return value;
}

static void stepperPulseStart (stepper_t *stepper)
{
    count[0] = sys_position[0] << 14;
    count[1] = sys_position[1] << 14;

    stepper_pulse_start(stepper);
}

void HAL_I2S_MspInit (I2S_HandleTypeDef* hi2s)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2s->Instance==SPI1)
  {

    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**I2S1 GPIO Configuration
    PA4     ------> I2S1_WS
    PA5     ------> I2S1_CK
    PA7     ------> I2S1_SD
    */

    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hdma_spi1_tx.Instance = DMA2_Stream3;
    hdma_spi1_tx.Init.Channel = DMA_CHANNEL_3;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi1_tx.Init.Mode = DMA_CIRCULAR;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hi2s,hdmatx,hdma_spi1_tx);
  }
}

static void MX_I2S1_Init (void)
{
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_32B;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  hi2s1.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s1.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_I2S_TxCpltCallback (I2S_HandleTypeDef *hi2s)
{
//    count[0] += 1024 << 10;
    data[0] = count[0] >> 16;
    data[1] = count[0] & 0xFFFF;

//    count[1] -= 1024 << 10;
    data[2] = count[1] >> 16;
    data[3] = count[1] & 0xFFFF;
}


/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler (void)
{
    HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

static void MX_DMA_Init (void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

// Reclaim entry points that may have been changed on settings change.
static void onSettingsChanged (settings_t *settings)
{
    settings_changed(settings);

    if(hal.stepper.pulse_start != stepperPulseStart) {
        stepper_pulse_start = hal.stepper.pulse_start;
        hal.stepper.pulse_start = stepperPulseStart;
    }
}

void board_init (void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB2;
    PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
    PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLI2SP_DIV2;
    PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
    PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
    PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
    PeriphClkInitStruct.PLLI2SDivQ = 1;
    PeriphClkInitStruct.I2sApb2ClockSelection = RCC_I2SAPB2CLKSOURCE_PLLI2S;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    MX_DMA_Init();
    MX_I2S1_Init();

    hal.port.wait_on_input = wait_on_input;
    hal.port.digital_out = digital_out;
    hal.port.num_digital_in = 2;
    hal.port.num_digital_out = 2;

    memcpy(&driver_settings, &hal.driver_settings, sizeof(driver_setting_ptrs_t));

    hal.driver_settings.set = aux_settings_set;
    hal.driver_settings.report = aux_settings_report;
//    hal.driver_settings.load = aux_settings_load;
//    hal.driver_settings.restore = aux_settings_restore;

    details.on_get_settings = grbl.on_get_settings;
    grbl.on_get_settings = onReportSettings;

    settings_changed = hal.settings_changed;
    hal.settings_changed = onSettingsChanged;

    stepper_pulse_start = hal.stepper.pulse_start;
    hal.stepper.pulse_start = stepperPulseStart;

    GPIO_InitTypeDef GPIO_Init = {0};

    GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;

    GPIO_Init.Pin = AUXOUTPUT0_BIT;
    HAL_GPIO_Init(AUXOUTPUT0_PORT, &GPIO_Init);

    GPIO_Init.Pin = AUXOUTPUT1_BIT;
    HAL_GPIO_Init(AUXOUTPUT1_PORT, &GPIO_Init);

    if (HAL_I2S_Transmit_DMA(&hi2s1, data, 2) != HAL_OK) {
      Error_Handler();
    }
}

#endif
