## SD Card driver plugin

This plugin adds a few `$` commands for listing files and running G-code from a SD Card:

`$FM`

Mount card.

`$F`

List files on the card recursively.

`$F=<filename>`

Run g-code in file. If the file ends with `M2` or rewind mode is active then it will be "rewound" and a cycle start command will start it again.

`$FR`

Enable rewind mode for next file to be run.

`$F<<filename>`

Dump file content to output stream.

Dependencies:

[FatFS library](http://www.elm-chan.org/fsw/ff/00index_e.html)

__NOTE:__ some drivers uses ports of FatFS provided by the MCU supplier.

---
2020-10-27
