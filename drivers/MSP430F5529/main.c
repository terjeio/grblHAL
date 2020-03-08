#include <msp430.h>

#include "GRBL\grbllib.h"
#include "driver.h"

/*
 * main.c
 */


void SetVCoreUp (unsigned int level)
{
    // Open PMM registers for write access
    PMMCTL0_H = 0xA5;
    // Set SVS/SVM high side new level
    SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
    // Set SVM low side to new level
    SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
    // Wait till SVM is settled
    while ((PMMIFG & SVSMLDLYIFG) == 0);
    // Clear already set flags
    PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
    // Set VCore to new level
    PMMCTL0_L = PMMCOREV0 * level;
    // Wait till new level reached
    if ((PMMIFG & SVMLIFG))
        while ((PMMIFG & SVMLVLRIFG) == 0);
    // Set SVS/SVM low side to new level
    SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
    // Lock PMM registers for write access
    PMMCTL0_H = 0x00;
}

int main (void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    /* Power settings */
     SetVCoreUp(1u);
     SetVCoreUp(2u);
     SetVCoreUp(3u);

     UCSCTL3 = SELREF__REFOCLK;    // select REFO as FLL source
     UCSCTL6 = XT1OFF | XT2OFF;    // turn off XT1 and XT2

     /* Initialize DCO to 25.00MHz */
     __bis_SR_register(SCG0);                  // Disable the FLL control loop
     UCSCTL0 = 0x0000u;                        // Set lowest possible DCOx, MODx
     UCSCTL1 = DCORSEL_6;                      // Set RSELx for DCO = 50 MHz
     UCSCTL2 = 762u;                            // Set DCO Multiplier for 25MHz
                                               // (N + 1) * FLLRef = Fdco
                                               // (762 + 1) * 32768 = 25.00MHz
     UCSCTL4 = SELA__REFOCLK | SELS__DCOCLK | SELM__DCOCLK;
//     UCSCTL4 = SELA__XT1CLK | SELS__DCOCLK | SELM__DCOCLK;
     __bic_SR_register(SCG0);                  // Enable the FLL control loop

     // Worst-case settling time for the DCO when the DCO range bits have been
     // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
     // UG for optimization.
     // 32*32*25MHz/32768Hz = 781250 = MCLK cycles for DCO to settle
     __delay_cycles(781250u);

     /* Loop until XT1,XT2 & DCO fault flag is cleared */
     do {
       UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);  // Clear XT2,XT1,DCO fault flags
       SFRIFG1 &= ~OFIFG;                           // Clear fault flags
     } while (SFRIFG1&OFIFG);                       // Test oscillator fault flag

    grbl_enter();
    return 0;
}
