## Odometer plugin

This plugin is used for logging distances travelled per axis, machining time and spindle on time.

Additional `$` commands provided :

`$ODOMETERS`

Sends current odometer values as messages to the sender. Distances in meters:

```
[MSG:SPINDLEHRS 4:20]
[MSG:MOTORHRS 5:52]
[MSG:ODOMETERX 22.4]
[MSG:ODOMETERY 19.4]
[MSG:ODOMETERZ 8.2]
```

`$ODOMETERS=PREV`

Sends previous odometer values as messages to the sender when available.

`$ODOMETERS=RST`

Copies current odometer values to previous values and then resets current odometer values to 0.

---

Dependencies:

Driver must support optional elapsed time HAL entry point and EEPROM/FRAM for non-volatile storage. Not available for flash storage, FRAM recommended.

---
2020-09-26
