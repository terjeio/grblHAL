## FatFS middleware for SD card plugin

Download the FatFs source code from [www.elm-chan.org](http://www.elm-chan.org/fsw/ff/00index_e.html) and unpack to this directory.

Delete the skeleton `discio.c` driver file in the FatFs folder and edit `ffconf.h` as follows:

Set `FF_FS_READONLY` to `1`.

Set `FF_CODE_PAGE` to a codepage small enough to fit in flash. I have tested with `850`.

Set `FF_USE_LFN` to `1` or another suitable value.

Other options may be edited as well depending on your needs.

__NOTE:__ The `discio.c` implementation provided as part of this driver has been tested with FatFs `R0.13c`.

__NOTE:__ I have tested with a 10K pullup connected to `MOSI (PB5)`, not sure if it is really needed...

---
2019-08-03
