## Networking services plugin

This plugin contains code for "stream based" network protocol support on top of the lwIP TCP/IP stack plus some utilities.

"stream based" in this context means that the HAL entry points for streaming are used as the API \(by dynamic pointer swapping on connect\).

#### Protocols supported:

* Telnet \("raw" mode\)
* Websocket

#### Dependencies:

[lwIP library](http://savannah.nongnu.org/projects/lwip/)

Driver capable of networking and having the required "middleware" layer on top of lwIP.

These drivers has this "middleware" layer, further details for how to configure networking can be found on the driver page:

* [iMXRT1062](../../drivers/IMXRT1062/README.md) for Teensy 4.1, cabled Ethernet.

* [MSP432E401Y](../../drivers/MSP432E401Y/README.md) for MSP432E401Y LaunchPad, cabled Ethernet.

* [TM4C129](../../drivers/TM4C129/README.md) for EK-TM4C1294XL Launchpad, cabled Ethernet.

* [ESP32](../../drivers/ESP32/README.md), wireless \(wifi\).

#### Credits:

Parts of WsStream.c are pulled from [patch 9525](http://savannah.nongnu.org/patch/?9525) by Sakari Kapanen.

base64.c, sha1.c by Brad Conte, pulled from from the same patch as mentioned above.

[multipartparser.c](https://github.com/francoiscolas/multipart-parser) by François Colas. 

wschat.html modified from original by [tutorialspoint.com](https://www.tutorialspoint.com/websockets/websockets_javascript_application.htm), for simple websocket testing \(edit line 103 to set address and port before use\).

__NOTE:__ some drivers uses ports of lwIP provided by the MCU supplier.  
__NOTE:__ this plugin is only for the protocol layer. Driver specific code is required for initialising lwIP and start/stop/polling the services.

---
#### Simple websocket test app:
![Test](../../media/websocket.png)

---
2020-10-20
