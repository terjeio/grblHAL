## grblHAL plugins

grblHAL's HAL interface is based on function pointers that may be used to add functionality without any need to change the core grbl code. They may also be changed on the fly to redirect calls, eg. the SD-card interface utilizes this to temporarily redirect input from the serial stream to the SD card.

NOTE: A plugin needs to be supported by the processor specific driver - as a minimum a initialization call has to be made. 

---
2019-07-25