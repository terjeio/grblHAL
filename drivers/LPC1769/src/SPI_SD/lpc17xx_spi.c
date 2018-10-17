/**************************************************************************//**
 * @file     lpc17xx_spi.c
 * @brief    Drivers for SSP peripheral in lpc17xx.
 * @version  1.0
 * @date     18. Nov. 2010
 *
 * @note
 * Copyright (C) 2010 NXP Semiconductors(NXP), Martin Thomas. 
 * All rights reserved.
 *
 * General SPI/SSP drivers: NXP
 * SSP FIFO: Martin Thomas 
 *
 ******************************************************************************/

#include "lpc17xx_spi.h"

/* Macro defines for SSP SR register */
#define SSP_SR_TFE      ((uint32_t)(1<<0)) /** SSP status TX FIFO Empty bit */
#define SSP_SR_TNF      ((uint32_t)(1<<1)) /** SSP status TX FIFO not full bit */
#define SSP_SR_RNE      ((uint32_t)(1<<2)) /** SSP status RX FIFO not empty bit */
#define SSP_SR_RFF      ((uint32_t)(1<<3)) /** SSP status RX FIFO full bit */
#define SSP_SR_BSY      ((uint32_t)(1<<4)) /** SSP status SSP Busy bit */
#define SSP_SR_BITMASK	((uint32_t)(0x1F)) /** SSP SR bit mask */


/**
  * @brief  Initializes the SSP0.
  *
  * @param  None
  * @retval None 
  */
void SPI_Init (void) 
{
    /* Enable SSPI0 block */
    LPC_SC->PCONP |= (1 << 21);
    
    /* Set SSEL0 as GPIO, output high */
    LPC_PINCON->PINSEL1 &= ~(3 << 0);          /* Configure P0.16(SSEL) as GPIO */
    LPC_GPIO0->FIODIR |= (1 << 16);            /* set P0.16 as output */
    
    /* Configure other SSP pins: SCK, MISO, MOSI */
    LPC_PINCON->PINSEL0 &= ~(3UL << 30);
    LPC_PINCON->PINSEL0 |=  (2UL << 30);          /* P0.15: SCK0 */
    LPC_PINCON->PINSEL1 &= ~((3<<2) | (3<<4));
    LPC_PINCON->PINSEL1 |=  ((2<<2) | (2<<4));  /* P0.17: MISO0, P0.18: MOSI0 */
    
    /* Configure SSP0_PCLK to CCLK(100MHz), default value is CCLK/4 */
    LPC_SC->PCLKSEL1 &= ~(3 << 10);
    LPC_SC->PCLKSEL1 |=  (1 << 10);  /* SSP0_PCLK=CCLK */
    
    /* 8bit, SPI frame format, CPOL=0, CPHA=0, SCR=0 */  
    LPC_SSP0->CR0 = (0x07 << 0) |     /* data width: 8bit*/
                    (0x00 << 4) |     /* frame format: SPI */
                    (0x00 << 6) |     /* CPOL: low level */
                    (0x00 << 7) |     /* CPHA: first edge */
                    (0x00 << 8);      /* SCR = 0 */

    /* Enable SSP0 as a master */
    LPC_SSP0->CR1 = (0x00 << 0) |   /* Normal mode */
                    (0x01 << 1) |   /* Enable SSP0 */
                    (0x00 << 2) |   /* Master */
                    (0x00 << 3);    /* slave output disabled */

    /* Configure SSP0 clock rate to 400kHz (100MHz/250) */
    SPI_ConfigClockRate (SPI_CLOCKRATE_LOW);

    /* Set SSEL to high */
    SPI_CS_High ();
}

/**
  * @brief  Configure SSP0 clock rate.
  *
  * @param  SPI_CLOCKRATE: Specifies the SPI clock rate.
  *         The value should be SPI_CLOCKRATE_LOW or SPI_CLOCKRATE_HIGH.
  * @retval None 
  *
  * SSP0_CLK = CCLK / SPI_CLOCKRATE
  */
void SPI_ConfigClockRate (uint32_t SPI_CLOCKRATE)
{
    /* CPSR must be an even value between 2 and 254 */
    LPC_SSP0->CPSR = (SPI_CLOCKRATE & 0x1FE);    
}

/**
  * @brief  Set SSEL to low: select spi slave.
  *
  * @param  None.
  * @retval None 
  */
void SPI_CS_Low (void)
{
    /* SSEL is GPIO, set to high.  */
    LPC_GPIO0->FIOPIN &= ~(1 << 16);            
}

/**
  * @brief  Set SSEL to high: de-select spi slave.
  *
  * @param  None.
  * @retval None 
  */
void SPI_CS_High (void)
{
    /* SSEL is GPIO, set to high.  */
    LPC_GPIO0->FIOPIN |= (1 << 16);             
}

/**
  * @brief  Send one byte via MOSI and simutaniously receive one byte via MISO.
  *
  * @param  data: Specifies the byte to be sent out.
  * @retval Returned byte.
  *
  * Note: Each time send out one byte at MOSI, Rx FIFO will receive one byte. 
  */
uint8_t SPI_SendByte (uint8_t data)
{
    /* Put the data on the FIFO */
    LPC_SSP0->DR = data;
    /* Wait for sending to complete */
    while (LPC_SSP0->SR & SSP_SR_BSY); 
    /* Return the received value */              
    return (LPC_SSP0->DR);                        
}

/**
  * @brief  Receive one byte via MISO.
  *
  * @param  None.
  * @retval Returned received byte.
  */
uint8_t SPI_RecvByte (void)
{
    /* Send 0xFF to provide clock for MISO to receive one byte */
    return SPI_SendByte (0xFF);
}

/* SPI FIFO functions are from Martin Thomas */
#ifdef USE_FIFO

/* 8 frame FIFOs for both transmit and receive */
#define SSP_FIFO_DEPTH       8 

/**
  * @brief  Send data block using FIFO.
  *
  * @param  buf: Pointer to the byte array to be sent
  * @param  len: length (in byte) of the byte array.
  *              Should be multiple of 4.   
  * @retval None.
  */
void SPI_SendBlock_FIFO (const uint8_t *buf, uint32_t len)
{
	uint32_t cnt;
	uint16_t data;

	LPC_SSP0->CR0 |= 0x0f;  /* DSS to 16 bit */

	/* fill the FIFO unless it is full */
	for ( cnt = 0; cnt < ( len / 2 ); cnt++ ) 
	{
		/* wait for TX FIFO not full (TNF) */
		while ( !( LPC_SSP0->SR & SSP_SR_TNF) );
		
		data  = (*buf++) << 8;
		data |= *buf++;
		LPC_SSP0->DR = data;
	}

	/* wait for BSY gone */
	while ( LPC_SSP0->SR & SSP_SR_BSY);

	/* drain receive FIFO */
	while ( LPC_SSP0->SR & SSP_SR_RNE ) {
		data = LPC_SSP0->DR; 
	}

	LPC_SSP0->CR0 &= ~0x08;  /* DSS to 8 bit */    
}


/**
  * @brief  Receive data block using FIFO.
  *
  * @param  buf: Pointer to the byte array to store received data
  * @param  len: Specifies the length (in byte) to receive.
  *              Should be multiple of 4.   
  * @retval None.
  */
void SPI_RecvBlock_FIFO (uint8_t *buf, uint32_t len)
{
	uint32_t hwtr, startcnt, i, rec;

	hwtr = len/2;
	if ( len < SSP_FIFO_DEPTH ) {
		startcnt = hwtr;
	} else {
		startcnt = SSP_FIFO_DEPTH;
	}

	LPC_SSP0 -> CR0 |= 0x0f;  /* DSS to 16 bit */

	for ( i = startcnt; i; i-- ) {
		LPC_SSP0->DR = 0xffff;  // fill TX FIFO
	}

	do {
		while ( !(LPC_SSP0->SR & SSP_SR_RNE ) ) {
			// wait for data in RX FIFO (RNE set)
		}
		rec = LPC_SSP0->DR;
		if ( i < ( hwtr - startcnt ) ) {
			LPC_SSP0->DR = 0xffff;
		}
		*buf++ = (uint8_t)(rec >> 8);
		*buf++ = (uint8_t)(rec);
		i++;
	} while ( i < hwtr );

    LPC_SSP0->CR0 &= ~0x08;  /* DSS to 8 bit */
}

#endif

/* --------------------------------- End Of File ------------------------------ */
