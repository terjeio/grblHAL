## MSP432E401Y

A GrblHAL driver for the Texas Instruments [MSP432E401Y LaunchPad](http://www.ti.com/tool/MSP-EXP432E401Y#).

See the Wiki-page for [compiling grblHAL](https://github.com/terjeio/grblHAL/wiki/Compiling-GrblHAL) for instructions for how to import the project, configure the driver and compile.

Available driver options can be found [here](base/my_machine.h).

Default pin assignments matches the [CNC BoosterPack](https://github.com/terjeio/CNC_Boosterpack).

To enable IP streaming use setting:

$70=2

**NOTE:** The serial interface remains fully active until a connection is established, when the connection is dropped it is reenabled.
The real-time status report will be streamed to the serial interface regardless of connection status.

It is possible to build the driver without FreeRTOS and ethernet enabled, how to do that will be published on request.

### How to build:

#### Requirements:

[Code Composer Studio (CCS)](http://www.ti.com/tool/ccstudio) v8 - free download.

This is an Eclipse based IDE with debugger support etc.

[SIMPLELINK-MSP432-SDK](http://www.ti.com/tool/simplelink-msp432-sdk) v2.30.00.14 or later - free download.

[FreeRTOS](https://www.freertos.org/) source, v10.1.1 - free download.

As an alternative it may be possible to build online using [CCS cloud](https://dev.ti.com/), but I have not tested that. Note that registration is required to use the online version.

#### Setup:

After installation CCS has to be configured and and a library built for FreeRTOS.

Navigate to _Window > Preferences > General > Workspace > Linked Resources_, add FREERTOS_INSTALL_DIR and set the location to the directory where this was installed.

Import the empty FreeRTOS template project: _File > Import > CCS Projects_ and browse to _examples\rtos\MSP_EXP432E401Y\drivers\empty\freertos_ the SimpleLink installation folder.

There is a more detailed description of this in the SDK documentation, this is located in the docs subdirectory in the SimpleLink installation directory and can also be found [online here](http://dev.ti.com/tirex/content/simplelink_msp432e4_sdk_1_55_00_21/docs/simplelink_mcu_sdk/Quick_Start_Guide.html).

The empty project is of no interest - it can be deleted, we need to build the _freertos_builds_MSP_EXP432E401Y_release_ccs_ project that came with the empty one so we can link against that later. Rigth-click on the project and select _Build Project_ to do that.

Prepare the driver project in the MSP432E401Y folder by copying the grbl source code from the GRBL folder in the root of the download to GRBL folder inside the MSP432E401Y folder.

---
2020-08-23
