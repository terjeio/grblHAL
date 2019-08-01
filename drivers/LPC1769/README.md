## LPC1769GrblDriver

A GrblHAL driver for the NXP LPC1769 processor.

Based on the official [grbl-LPC port](https://github.com/gnea/grbl-LPC).

*** Incomplete ***

I did some work on this in early 2018 but I did not complete it for some reason \(IIRC something to do with limit pins on a popular board not beeing interrupt capable?\). I cannot remember now how far I got but at least it compiles.

Since the HAL now has entry points that can be used for SD card support, I googled for a FatFs port. Since I found one I have included that but be aware that I have not checked the pin assignments for SPI neither that it works - then again, it compiles.

I decided to publish it so somebody may have a go at making it work.

---
2019-08-01
