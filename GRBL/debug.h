#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"

#define ZAXISDPORT       GPIO_PORTE_BASE
#define ZAXISDPIN        GPIO_PIN_5
#define ZAXISDBIT        5

#define DEBUGOUT

#define debugout(p) HWREGBITW(ZAXISDPORT + GPIO_O_DATA + (ZAXISDPIN << 2), ZAXISDBIT) = p;
