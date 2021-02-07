/*
 * myconfig.h
 *
 *  Created on: Jan 31, 2021
 *      Author: Clayton
 */

#ifndef APPLICATION_MYCONFIG_H_
#define APPLICATION_MYCONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif


#define SAEMS_HARDWARE_VERSION 0
enum MCP_PinMap {
    // RGB Indicator LED
    I_LED_B = 0, I_LED_G, I_LED_R,
    // Onboard Up/Right/Left/Down/Select Button
    Multi_A, Multi_B, Multi_C, Multi_D, Multi_Press,
    // Expansion Connector Sleep
    EXT_Sleep,
    // Display Connections
    SD_CS, FONT_CS, LCD_DC, LCD_RESET,
    // Ethernet Reset
    ETH_RST,
    // VOC Interrupt
    VOC_INT,
    // 5V Boost Enable / Battery Operation Enable
    EN_5V_BST
};

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_MYCONFIG_H_ */
