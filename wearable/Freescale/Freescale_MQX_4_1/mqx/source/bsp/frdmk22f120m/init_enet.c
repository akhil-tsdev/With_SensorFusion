/**HEADER********************************************************************
* 
* Copyright (c) 2008 Freescale Semiconductor;
* All Rights Reserved
*
*************************************************************************** 
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR 
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
* THE POSSIBILITY OF SUCH DAMAGE.
*
**************************************************************************
*
* $FileName: init_enet.c$
* $Version : 3.8.7.0$
* $Date    : Aug-30-2011$
*
* Comments:
*
*   This file contains the function that reads the timer and returns
*   the number of nanoseconds elapsed since the last interrupt.
*
*END************************************************************************/

#include "mqx.h"
#include "bsp.h"
#include "bsp_prv.h"
#include "enet.h"
#include "enetprv.h"
#include "psptypes_legacy.h" //TONY

#if !(BSPCFG_DB142_IOT_MODE_SUPPORT)
#include <macnet_prv.h>          
#include "phy_ksz8041.h"
#endif 

#if BSPCFG_ENABLE_ATHEROS_WIFI
#include <atheros_wifi.h>  
#include <atheros_phy.h>
#endif /* BSPCFG_ENABLE_ATHEROS_WIFI */

#if BSPCFG_DB142_IOT_MODE_SUPPORT
//#include "macnet_mk21_ath_wifi.h"
#endif 

#define MACNET_DEVICE_0                0

#if BSPCFG_DB142_IOT_MODE_SUPPORT
#define MACNET_DEVICE_COUNT            0
#else
#define MACNET_DEVICE_COUNT            1
#endif 

#if !(BSPCFG_DB142_IOT_MODE_SUPPORT)
const MACNET_INIT_STRUCT MACNET_device[MACNET_DEVICE_COUNT] =  {
    { BSP_SYSTEM_CLOCK, BSP_MACNET0_INT_RX_LEVEL, 0, BSP_MACNET0_INT_TX_LEVEL, 0 },
};


const ENET_IF_STRUCT ENET_0 = {
    &MACNET_IF,
    &phy_ksz8041_IF,
    MACNET_DEVICE_0,
    MACNET_DEVICE_0,
    BSP_ENET0_PHY_ADDR,
    BSP_ENET0_PHY_MII_SPEED
};
#endif 

#if BSPCFG_ENABLE_ATHEROS_WIFI 

	#if BSPCFG_DB142_IOT_MODE_SUPPORT
	const ENET_IF_STRUCT ENET_0 = {
		 &ATHEROS_WIFI_IF,
		 &ATHEROS_PHY_IF,
		 MACNET_DEVICE_0,
		 MACNET_DEVICE_0,
		 0,
	};
	#else 
	const ENET_IF_STRUCT ENET_1 = {
		 &ATHEROS_WIFI_IF,
		 &ATHEROS_PHY_IF,
		 1,
		 1,
		 0,
	};
	#endif 

ATHEROS_PARAM_WIFI_STRUCT atheros_wifi_param = {	
	BSP_ATHEROS_WIFI_SPI_DEVICE,
    BSP_ATHEROS_WIFI_GPIO_INT_DEVICE,
    BSP_ATHEROS_WIFI_GPIO_INT_PIN,
    BSP_ATHEROS_WIFI_GPIO_PWD_DEVICE,
    BSP_ATHEROS_WIFI_GPIO_PWD_PIN
};
#endif /* BSPCFG_ENABLE_ATHEROS_WIFI */


/*E.Y. add FOR DB142*/
const ENET_PARAM_STRUCT ENET_default_params[BSP_ENET_DEVICE_COUNT] = {
    {
     &ENET_0,
       // # Default WiFi Device parameter
     Auto_Negotiate,
     0,

     BSPCFG_TX_RING_LEN,   //  # NOT USED IN ATHEROS WIFI
     BSPCFG_TX_RING_LEN,   //  # NOT USED IN ATHEROS WIFI
     ENET_FRAMESIZE,       //  # NOT USED IN ATHEROS WIFI
     
     BSPCFG_RX_RING_LEN,   //  # NOT USED IN ATHEROS WIFI
     BSPCFG_RX_RING_LEN,   //  # NOT USED IN ATHEROS WIFI
     ENET_FRAMESIZE,       //  # NOT USED IN ATHEROS WIFI
     BSP_CONFIG_ATHEROS_PCB,   //  # rx PCBs 
     
     0,     
     0 ,
     (pointer)&atheros_wifi_param    
   }
   
};

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_get_mac_address
* Returned Value   : uint_32
* Comments         :
*    This function returns the mac address associated with the specified device
*
* If the MAC address is stored in NV-storage, this fuction should read the
* MAC address from NV-storage and set it.
*
* If the MAC address is generated from a device serial number and an OUI, the
* serial number should be passed in, and the MAC address should be constructed
*
*END*----------------------------------------------------------------------*/

const _enet_address _enet_oui = BSP_DEFAULT_ENET_OUI;

boolean _bsp_get_mac_address
   (
      uint_32        device,
      uint_32        value,
      _enet_address  address
   )
{ /* Body */
   char_ptr value_ptr = (char_ptr) &value;

   if (device >= BSP_ENET_DEVICE_COUNT) 
      return FALSE;

   address[0] = _enet_oui[0];
   address[1] = _enet_oui[1];
   address[2] = _enet_oui[2];
   address[3] = (value & 0xFF0000) >> 16;
   address[4] = (value & 0xFF00) >> 8;
   address[5] = (value & 0xFF);

   return TRUE;

} /* Endbody */


/* EOF */
