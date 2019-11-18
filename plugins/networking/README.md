## Networking services plugin

This plugin contains code for "stream based" network protocol support on top of the lwIP TCP/IP stack.

"stream based" in this context means that the HAL entry points for streaming are used as the API \(by dynamic pointer swapping on connect\).

#### Protocols supported:

* Telnet ("raw" mode)  
* Websocket - work in progress, initial test results are promising.  

#### Dependencies:

[lwIP library](http://savannah.nongnu.org/projects/lwip/)

#### Credits:

Parts of WsStream.c are pulled from [patch 9525](http://savannah.nongnu.org/patch/?9525) by Sakari Kapanen.

base64.c, sha1.c by Brad Conte, pulled from from the same patch as mentioned above.

__NOTE:__ some drivers uses ports of lwIP provided by the MCU supplier.  
__NOTE:__ this plugin is only for the protocol layer. Driver specific code is required for initialising lwIP and start/stop/polling the services.

---
2019-11-18
