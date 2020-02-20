/*
  serial.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Template driver code for ARM processors

  Part of GrblHAL

  By Terje Io, public domain

*/

#ifndef _HAL_SERIAL_H_
#define _HAL_SERIAL_H_

#include "src/grbl/grbl.h"

void serialInit (void);
int16_t serialGetC (void);
void serialWriteS (const char *data);
bool serialSuspendInput (bool suspend);
uint16_t serialRxFree (void);
void serialRxFlush (void);
void serialRxCancel (void);

#endif
