// portmacros.h - some useful macros for Texas Instruments MSP430 processor family

#ifndef __portmacros_h__

#define __portmacros_h__

#define portIn(p) portI(p)
#define portI(p) P ## p ## IN
#define portOut(p) portO(p)
#define portO(p) P ## p ## OUT
#define portDir(p) portD(p)
#define portD(p) P ## p ## DIR
#define portSel(p) portS(p)
#define portS(p) P ## p ## SEL
#define portSel2(p) portS2(p)
#define portS2(p) P ## p ## SEL2
#define portRen(p) portR(p)
#define portR(p) P ## p ## REN
#define portIe(p) portF(p)
#define portF(p) P ## p ## IE
#define portEs(p) portG(p)
#define portG(p) P ## p ## IES
#define portIfg(p) portH(p)
#define portH(p) P ## p ## IFG
#define portInt(p) portW(p)
#define portW(p) PORT ## p ## _VECTOR

#define timerR(p, i) timerRx(p, i)
#define timerRx(p, i) T ## p ## i ## R
#define timerCtl(p, i) timerC(p, i)
#define timerC(p, i) T ## p ## i ## CTL
#define timerCCtl(p, i, q) timerD(p, i, q)
#define timerD(p, i, q) T ## p ## i ## CCTL ## q
#define timerCcr(p, i, q) timerA(p, i, q)
#define timerA(p, i, q) T ## p ## i ## CCR ## q
#define timerIv(p, i) timerI(p, i)
#define timerI(p, i) T ## p ## i ## IV
#define timerEx(p, i) timerE(p, i)
#define timerE(p, i) T ## p ## i ## EX0
#define timerInt(p, i, q) timerW(p, i, q)
#define timerW(p, i, q) TIMER ## i ## _ ## p ## q ## _VECTOR

#define uartCTL(m, i) uartC(m, i)
#define uartBR(m, i) uartB(m, i)
#define uartB(m, i) UC ## m ## BR ## i
#define uartMCTL(m) uartM(m)
#define uartM(m) UC ## m ## MCTL

#define uartC(m, i) UC ## m ## CTL ## i
#ifdef __MSP430F5310__
#define uartIE(m) uartI(m)
#define uartI(m) UC ## m ## IE
#define uartIFG(m) uartF(m)
#define uartF(m) UC ## m ## IFG
#define uartSTAT(m) uartS(m)
#define uartS(m) UC ## m ## STAT
#define uartRXD(m) uartR(m)
#define uartR(m) UC ## m ## RXBUF
#define uartTXD(m) uartT(m)
#define uartT(m) UC ## m ## TXBUF
#define uartI2CSA(m) uartA(m)
#define uartA(m) UC ## m ## I2CSA
#else
#define uartIE(m) uartI(m)
#define uartI(m) IE2
#define uartIFG(m) uartF(m)
#define uartF(m) IFG2
#define uartSTAT(m) uartS(m)
#define uartS(m) UC ## m ## STAT
#define uartRXD(m) uartR(m)
#define uartR(m) UC ## m ## RXBUF
#define uartTXD(m) uartT(m)
#define uartT(m) UC ## m ## TXBUF
#define uartI2CSA(m) uartA(m)
#define uartA(m) UC ## m ## I2CSA
#endif


#endif
