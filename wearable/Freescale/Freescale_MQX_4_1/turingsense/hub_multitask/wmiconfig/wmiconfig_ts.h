/*
 * Copyright TuringSense, Inc © 2015
 *
 * wmiconfig_ts.h
 *
 *  Created on: June 26, 2015
 *      Author: cwati
 */

#ifndef WMICONFIG_TS_H_
#define WMICONFIG_TS_H_

//#include <stdbool.h>
////#include <stdio.h>
//#include <string.h>
//
//#include <mqx.h>
//#include <bsp.h>
//#include <spi.h>

/* Set wifi password */
A_INT32 wmiconfig_set_passphrase(char* passphrase);

/* Set WPA version and cipher */
A_INT32 wmiconfig_set_wpa( A_INT32 ver, char* umode, char* mmode);

/* Set TX power */
A_INT32 wmiconfig_set_tx_power(uint32_t dBm);

/* Set PHY Mode 802.11b/802.11g/802.11n */
A_INT32 wmiconfig_set_phy_mode(char* phy_mode);

/* Get PHY Mode */
A_INT32 wmiconfig_get_phy_mode (char* phy_mode);

/* Connect to SSID */
A_INT32 wmiconfig_connect_handler (char* ssid_str);

/* Check wifi connectivity */
A_UINT32 wifi_connected();

/* Asking for static IP for now */
A_INT32 wmiconfig_ipconfig_static(char* ip_addr_string, char* mask_string, char* gw_string);

void wmiconfig_set_wmi_powermode(WMI_POWER_MODE new_power_mode);

#endif /* WMICONFIG_TS_H_ */
