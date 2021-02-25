# SAEMS Firmware Repo

This is the embedded firmware for project, it is designed for the central [CC2652P](https://www.ti.com/product/CC2652P) SimpleLink microcontroller from Texas Instruments. Development is done in the TI CCS IDE and debugged with a [LAUNCHXL-CC1352P-2](https://www.ti.com/tool/LAUNCHXL-CC1352P) (with jumpers disconnected) or with compatible JTAG debuggers such as the SEGGER [J-Link](https://www.segger.com/products/debug-probes/j-link/). 

Firmware is running TI-RTOS and TI Z-Stack for Zigbee. This is to easily support multiple tasks and multiple wireless smart home base stations.
