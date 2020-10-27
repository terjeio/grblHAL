/*
  st_morpho.c - driver code for STM32F4xx ARM processors

  Part of GrblHAL

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

#include "main.h"
#include "driver.h"
#include "grbl/protocol.h"

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

void board_init (void)
{
    hal.port.wait_on_input = wait_on_input;
    hal.port.digital_out = digital_out;
    hal.port.num_digital_in = 2;
    hal.port.num_digital_out = 2;

    GPIO_InitTypeDef GPIO_Init = {0};

    GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;

    GPIO_Init.Pin = AUXOUTPUT0_BIT;
    HAL_GPIO_Init(AUXOUTPUT0_PORT, &GPIO_Init);

    GPIO_Init.Pin = AUXOUTPUT1_BIT;
    HAL_GPIO_Init(AUXOUTPUT1_PORT, &GPIO_Init);
}
