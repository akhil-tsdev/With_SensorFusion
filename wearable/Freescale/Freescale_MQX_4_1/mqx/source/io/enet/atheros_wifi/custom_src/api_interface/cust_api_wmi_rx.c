//------------------------------------------------------------------------------
// Copyright (c) 2011 Qualcomm Atheros, Inc.
// All Rights Reserved.
// Qualcomm Atheros Confidential and Proprietary.
// Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is
// hereby granted, provided that the above copyright notice and this permission notice appear in all copies.
//
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE
// INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
// ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF
// USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
// OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
//------------------------------------------------------------------------------
//==============================================================================
// Author(s): ="Atheros"
//==============================================================================
#include <a_config.h>
#include <a_types.h>
#include <a_osapi.h>
#include <common_api.h>
#include <custom_api.h>
#include <driver_cxt.h>
#include <wmi_host.h>
#include "mqx.h"
#include "bsp.h"
#include "enet.h"
#include "enetprv.h"
#include "enet_wifi.h"

#if ENABLE_P2P_MODE
#include "p2p.h"
#include "wmi.h"
#include "wmi_api.h"
#include "wmi_host.h"
#endif

#include <atheros_wifi_api.h>
#include <atheros_wifi_internal.h>
#if ENABLE_P2P_MODE
extern A_CONST WMI_SCAN_PARAMS_CMD default_scan_param;
extern A_CONST A_UINT8 max_performance_power_param;
#endif
static A_VOID
fill_scan_info(A_VOID *pCxt, ENET_SCAN_INFO *pScanInfo, WMI_BSS_INFO_HDR *bih, A_INT32 len)
{
	A_SCAN_SUMMARY scan_summary;


	if(A_OK == Api_ParseInfoElem(pCxt, bih, len, &scan_summary)){
		pScanInfo->channel = scan_summary.channel;
		pScanInfo->rssi = scan_summary.rssi;
		A_MEMCPY(pScanInfo->bssid, scan_summary.bssid, ATH_MAC_LEN);
		pScanInfo->ssid_len = scan_summary.ssid_len;

		A_MEMCPY(pScanInfo->ssid, &(scan_summary.ssid[0]), scan_summary.ssid_len);
		pScanInfo->security_enabled = (A_UINT8)((scan_summary.caps & IEEE80211_CAPINFO_PRIVACY)? 1:0);
		pScanInfo->preamble = (A_UINT8)((scan_summary.caps & IEEE80211_CAPINFO_SHORT_PREAMBLE)? 1:0);

		if((scan_summary.caps & (IEEE80211_CAPINFO_ESS|IEEE80211_CAPINFO_IBSS)) == IEEE80211_CAPINFO_ESS){
	    	pScanInfo->bss_type = ENET_MEDIACTL_MODE_INFRA;
	    }else if((scan_summary.caps & (IEEE80211_CAPINFO_ESS|IEEE80211_CAPINFO_IBSS)) == IEEE80211_CAPINFO_IBSS){
	    	pScanInfo->bss_type = ENET_MEDIACTL_MODE_ADHOC;
	    }else{
	    	/* error condition report it as ad-hoc in this case*/
	    	pScanInfo->bss_type = ENET_MEDIACTL_MODE_ADHOC;
	    }

	    pScanInfo->beacon_period = scan_summary.beacon_period;

	    if(scan_summary.rsn_cipher & TKIP_CRYPT){
	    	pScanInfo->rsn_cipher |= ATH_CIPHER_TYPE_TKIP;	
	    }
	    
	    if(scan_summary.rsn_cipher & AES_CRYPT){
	    	pScanInfo->rsn_cipher |= ATH_CIPHER_TYPE_CCMP;	
	    }
	    
	    if(scan_summary.rsn_cipher & WEP_CRYPT){
	    	pScanInfo->rsn_cipher |= ATH_CIPHER_TYPE_WEP;	
	    }
	    
	    if(scan_summary.wpa_cipher & TKIP_CRYPT){
	    	pScanInfo->wpa_cipher |= ATH_CIPHER_TYPE_TKIP;	
	    }
	    
	    if(scan_summary.wpa_cipher & AES_CRYPT){
	    	pScanInfo->wpa_cipher |= ATH_CIPHER_TYPE_CCMP;	
	    }
	    
	    if(scan_summary.wpa_cipher & WEP_CRYPT){
	    	pScanInfo->wpa_cipher |= ATH_CIPHER_TYPE_WEP;	
	    }
	    
	    if(scan_summary.rsn_auth & WPA2_AUTH){
	    	pScanInfo->rsn_auth |= SECURITY_AUTH_1X;
	    }
	    
	    if(scan_summary.rsn_auth & WPA2_PSK_AUTH){
	    	pScanInfo->rsn_auth |= SECURITY_AUTH_PSK;
        }    
    
	    if(scan_summary.wpa_auth & WPA_AUTH){
	    	pScanInfo->wpa_auth |= SECURITY_AUTH_1X;
	    }

	    if(scan_summary.wpa_auth & WPA_PSK_AUTH){
	    	pScanInfo->wpa_auth |= SECURITY_AUTH_PSK;
	    }		
	}
}

static A_VOID
fill_ext_scan_info(A_VOID *pCxt, ATH_SCAN_EXT *pExtScanInfo, WMI_BSS_INFO_HDR *bih, A_INT32 len)
{
	A_SCAN_SUMMARY scan_summary;


	if(A_OK == Api_ParseInfoElem(pCxt, bih, len, &scan_summary)){
		A_MEMZERO(pExtScanInfo, sizeof(ATH_SCAN_EXT));
		pExtScanInfo->channel = scan_summary.channel;
		pExtScanInfo->rssi = scan_summary.rssi;
		A_MEMCPY(pExtScanInfo->bssid, scan_summary.bssid, ATH_MAC_LEN);
		pExtScanInfo->ssid_len = scan_summary.ssid_len;

		A_MEMCPY(pExtScanInfo->ssid, &(scan_summary.ssid[0]), scan_summary.ssid_len);
		pExtScanInfo->security_enabled = (A_UINT8)((scan_summary.caps & IEEE80211_CAPINFO_PRIVACY)? 1:0);
		pExtScanInfo->preamble = (A_UINT8)((scan_summary.caps & IEEE80211_CAPINFO_SHORT_PREAMBLE)? 1:0);

		if((scan_summary.caps & (IEEE80211_CAPINFO_ESS|IEEE80211_CAPINFO_IBSS)) == IEEE80211_CAPINFO_ESS){
	    	pExtScanInfo->bss_type = ENET_MEDIACTL_MODE_INFRA;
	    }else if((scan_summary.caps & (IEEE80211_CAPINFO_ESS|IEEE80211_CAPINFO_IBSS)) == IEEE80211_CAPINFO_IBSS){
	    	pExtScanInfo->bss_type = ENET_MEDIACTL_MODE_ADHOC;
	    }else{
	    	/* error condition report it as ad-hoc in this case*/
	    	pExtScanInfo->bss_type = ENET_MEDIACTL_MODE_ADHOC;
	    }

	    pExtScanInfo->beacon_period = scan_summary.beacon_period;

	    if(scan_summary.rsn_cipher & TKIP_CRYPT){
	    	pExtScanInfo->rsn_cipher |= ATH_CIPHER_TYPE_TKIP;
	    }

	    if(scan_summary.rsn_cipher & AES_CRYPT){
	    	pExtScanInfo->rsn_cipher |= ATH_CIPHER_TYPE_CCMP;
	    }

	    if(scan_summary.rsn_cipher & WEP_CRYPT){
	    	pExtScanInfo->rsn_cipher |= ATH_CIPHER_TYPE_WEP;
	    }

	    if(scan_summary.wpa_cipher & TKIP_CRYPT){
	    	pExtScanInfo->wpa_cipher |= ATH_CIPHER_TYPE_TKIP;
	    }

	    if(scan_summary.wpa_cipher & AES_CRYPT){
	    	pExtScanInfo->wpa_cipher |= ATH_CIPHER_TYPE_CCMP;
	    }

	    if(scan_summary.wpa_cipher & WEP_CRYPT){
	    	pExtScanInfo->wpa_cipher |= ATH_CIPHER_TYPE_WEP;
	    }

	    if(scan_summary.rsn_auth & WPA2_AUTH){
	    	pExtScanInfo->rsn_auth |= SECURITY_AUTH_1X;
	    }

	    if(scan_summary.rsn_auth & WPA2_PSK_AUTH){
	    	pExtScanInfo->rsn_auth |= SECURITY_AUTH_PSK;
	    }

	    if(scan_summary.wpa_auth & WPA_AUTH){
	    	pExtScanInfo->wpa_auth |= SECURITY_AUTH_1X;
	    }

	    if(scan_summary.wpa_auth & WPA_PSK_AUTH){
	    	pExtScanInfo->wpa_auth |= SECURITY_AUTH_PSK;
	    }
	}
}

A_VOID
Custom_Api_BssInfoEvent(A_VOID *pCxt, A_UINT8 *datap, A_INT32 len)
{
    A_DRIVER_CONTEXT *pDCxt = GET_DRIVER_COMMON(pCxt);
    WMI_BSS_INFO_HDR *bih = (WMI_BSS_INFO_HDR *)datap;
	ENET_SCAN_INFO_PTR pScanInfo;
	ATH_SCAN_EXT *pExtScanInfo;
	A_UINT8 i,worst_snr_idx;
	A_UINT8 worst_snr = 0xff;
    A_UINT16 scanCount;

	/* add/replace entry in application scan results */
	if(GET_DRIVER_CXT(pCxt)->extended_scan){
		pExtScanInfo = (ATH_SCAN_EXT*)(GET_DRIVER_CXT(pCxt)->pScanOut);
		scanCount = GET_DRIVER_CXT(pCxt)->scanOutCount;
		//look for previous match
    	for(i=0 ; i<scanCount ; i++){
    		if(0==A_MEMCMP(bih->bssid, pExtScanInfo[i].bssid, ATH_MAC_LEN)){
    			fill_ext_scan_info(pCxt, &pExtScanInfo[i], bih, len);
    			break;
    		}
    		/* keep worst rssi entry for optional use below */
    		if(worst_snr > pExtScanInfo[i].rssi){
    			worst_snr = pExtScanInfo[i].rssi;
    			worst_snr_idx = i;
    		}
    	}

    	if(i >= scanCount){
    		if(GET_DRIVER_CXT(pCxt)->scanOutSize <= scanCount){
    			/* replace other non-matching entry based on rssi */
    			if(bih->snr > worst_snr){
    				fill_ext_scan_info(pCxt, &pExtScanInfo[worst_snr_idx], bih, len);
    			}
    		}else{
    			/* populate new entry */
    			fill_ext_scan_info(pCxt, &pExtScanInfo[scanCount], bih, len);
    			scanCount++;
                GET_DRIVER_CXT(pCxt)->scanOutCount = scanCount;
    		}
    	}
	}else{
    	pScanInfo = (ENET_SCAN_INFO_PTR)(GET_DRIVER_CXT(pCxt)->pScanOut);
        scanCount = GET_DRIVER_CXT(pCxt)->scanOutCount;
    	//look for previous match
    	for(i=0 ; i<scanCount ; i++){
    		if(0==A_MEMCMP(bih->bssid, pScanInfo[i].bssid, ATH_MAC_LEN)){
    			fill_scan_info(pCxt, &pScanInfo[i], bih, len);
    			break;
    		}
    		/* keep worst rssi entry for optional use below */
    		if(worst_snr > pScanInfo[i].rssi){
    			worst_snr = pScanInfo[i].rssi;
    			worst_snr_idx = i;
    		}
    	}

    	if(i >= scanCount){
    		if(GET_DRIVER_CXT(pCxt)->scanOutSize <= scanCount){
    			/* replace other non-matching entry based on rssi */
    			if(bih->snr > worst_snr){
    				fill_scan_info(pCxt, &pScanInfo[worst_snr_idx], bih, len);
    			}
    		}else{
    			/* populate new entry */
    			fill_scan_info(pCxt, &pScanInfo[scanCount], bih, len);
    			scanCount++;
                GET_DRIVER_CXT(pCxt)->scanOutCount = scanCount;
    		}
    	}
    }
}

A_VOID
Custom_Api_ConnectEvent(A_VOID *pCxt)
{
	ATH_CONNECT_CB cb = NULL;

    /* Update connect & link status atomically */
    DRIVER_SHARED_RESOURCE_ACCESS_ACQUIRE(pCxt);
    {
        if(GET_DRIVER_CXT(pCxt)->connectStateCB != NULL){
		    cb = (ATH_CONNECT_CB)GET_DRIVER_CXT(pCxt)->connectStateCB;
		    /* call this later from outside the spinlock */
	    }
    }
    DRIVER_SHARED_RESOURCE_ACCESS_RELEASE(pCxt);

    CUSTOM_DRIVER_WAKE_USER(pCxt);

    /* call the callback function provided by application to
     * indicate connection state */
	if(cb != NULL){
		cb(A_TRUE);
	}
}


A_VOID
Custom_Api_DisconnectEvent(A_VOID *pCxt, A_UINT8 reason, A_UINT8 *bssid,
                        A_UINT8 assocRespLen, A_UINT8 *assocInfo, A_UINT16 protocolReasonStatus)
{
    ATH_CONNECT_CB cb = NULL;


    /* Update connect & link status atomically */
    DRIVER_SHARED_RESOURCE_ACCESS_ACQUIRE(pCxt);
    if(GET_DRIVER_CXT(pCxt)->connectStateCB != NULL){
		cb = (ATH_CONNECT_CB)GET_DRIVER_CXT(pCxt)->connectStateCB;
		/* call this later from outside the spinlock */
	}
    DRIVER_SHARED_RESOURCE_ACCESS_RELEASE(pCxt);
	/* call the callback function provided by application to
     * indicate connection state */
	if(cb != NULL){
		if(reason == INVALID_PROFILE)
		{
			cb(INVALID_PROFILE);
		}
		else
		{
		cb(A_FALSE);
	}
}
}


A_VOID
Custom_Api_ReadyEvent(A_VOID *pCxt, A_UINT8 *datap, A_UINT8 phyCap, A_UINT32 sw_ver, A_UINT32 abi_ver)
{
    ENET_CONTEXT_STRUCT_PTR enet_ptr = (ENET_CONTEXT_STRUCT_PTR)GET_DRIVER_CXT(pCxt)->pUpperCxt;
    /* this custom implementation sets an event after setting CXT_WMI_READY
     * so as to allow the blocked user thread to wake up. */
    A_MEMCPY(enet_ptr->ADDRESS, datap, ATH_MAC_LEN);
    CUSTOM_DRIVER_WAKE_USER(pCxt);
    UNUSED_ARGUMENT(phyCap);
    UNUSED_ARGUMENT(sw_ver);
    UNUSED_ARGUMENT(abi_ver);
}

A_VOID
Custom_Api_RSNASuccessEvent(A_VOID *pCxt, A_UINT8 code)
{

	 ATH_CONNECT_CB  cb = NULL;
	/* this is the event that the customer has to use to send a callback
	 * to the application so that it will print the success event */

    /* get the callback handler from the device context */
    DRIVER_SHARED_RESOURCE_ACCESS_ACQUIRE(pCxt);
    if(GET_DRIVER_CXT(pCxt)->connectStateCB != NULL){
		cb = (ATH_CONNECT_CB)GET_DRIVER_CXT(pCxt)->connectStateCB;
		/* call this later from outside the spinlock */
	}
    DRIVER_SHARED_RESOURCE_ACCESS_RELEASE(pCxt);
	/* call the callback function provided by application to
     * indicate 4 way handshake status */
	if(cb != NULL){

		cb(code);
	}

}


A_VOID
Custom_Api_BitRateEvent_tx(A_VOID *pCxt,A_INT8 rateIndex)
{
	 ATH_CONNECT_CB  cb = NULL;
	/* the driver will get the index of the last tx rate from chip
	 * based on this index we get the rate tx from the array */

    /* get the callback handler from the device context */
    DRIVER_SHARED_RESOURCE_ACCESS_ACQUIRE(pCxt);
    if(GET_DRIVER_CXT(pCxt)->connectStateCB != NULL){
		cb = (ATH_CONNECT_CB)GET_DRIVER_CXT(pCxt)->connectStateCB;
		/* call this later from outside the spinlock */
	}
    DRIVER_SHARED_RESOURCE_ACCESS_RELEASE(pCxt);
	/* call the callback function provided by application to
     * indicate last transmitted rate */

	if(cb != NULL){

		cb(wmi_rateTable[rateIndex][0]);
	}
}

#if ANTENNA_DIVERSITY
A_VOID
Custom_get_Ant_div_stat(A_VOID *pCxt,A_UINT8 *datap,A_UINT32 len)
{
    ANTENNA_STAT_CB cb = NULL;


    /* Update connect & link status atomically */
    DRIVER_SHARED_RESOURCE_ACCESS_ACQUIRE(pCxt);
    if(GET_DRIVER_CXT(pCxt)->antennadivstatCB != NULL){
		cb = (ANTENNA_STAT_CB)GET_DRIVER_CXT(pCxt)->antennadivstatCB;
		/* call this later from outside the spinlock */
	}
    DRIVER_SHARED_RESOURCE_ACCESS_RELEASE(pCxt);
	/* call the callback function provided by application to
     * indicate connection state */
	if(cb != NULL){
          cb((void *)datap);
    }
}
#endif

#if ENABLE_P2P_MODE
A_VOID
Custom_Api_p2p_go_neg_event(A_VOID *pCxt, A_UINT8 *datap, A_UINT32 len,WMI_P2P_PROV_INFO *wps_info)
{
    A_UINT32 evt_id = 0;
    evt_id = WMI_P2P_GO_NEG_RESULT_EVENTID;
    A_DRIVER_CONTEXT* pDCxt = GET_DRIVER_COMMON(pCxt);
    
    DRIVER_SHARED_RESOURCE_ACCESS_ACQUIRE(pCxt);
    {
        A_MEMZERO(GET_DRIVER_CXT(pCxt)->pScanOut,  pDCxt->tempStorageLength);
              
    	A_MEMCPY(GET_DRIVER_CXT(pCxt)->pScanOut, &evt_id, sizeof(A_UINT32));

        A_MEMCPY((GET_DRIVER_CXT(pCxt)->pScanOut)+sizeof(A_UINT32) , datap, sizeof(WMI_P2P_GO_NEG_RESULT_EVENT));
     
        if(GET_DRIVER_CXT(pCxt)->p2pEvtState == A_TRUE)
        {
            GET_DRIVER_CXT(pCxt)->p2pEvtState = A_FALSE;
        } else {
            GET_DRIVER_CXT(pCxt)->p2pevtflag = A_TRUE;
        }
    }
    DRIVER_SHARED_RESOURCE_ACCESS_RELEASE(pCxt);

    CUSTOM_DRIVER_WAKE_USER(pCxt);
}

static A_VOID
fill_P2PLite(struct p2p_device  *p2pDev,struct p2p_device_lite *p2pDevLite)
{
    //A_MEMCPY(p2pDevLite->pri_dev_type,p2pDev->pri_dev_type,8);
    A_MEMCPY(p2pDevLite->p2p_device_addr,p2pDev->p2p_device_addr, ATH_MAC_LEN);
    A_MEMCPY(p2pDevLite->interface_addr,p2pDev->interface_addr, ATH_MAC_LEN);
    A_MEMCPY(p2pDevLite->device_name,p2pDev->device_name,33);
    
 //    p2pDevLite->wps_method = A_LE2CPU32(p2pDev->wps_method);
 //    p2pDevLite->config_methods = p2pDev->config_methods;
    p2pDevLite->dev_capab = p2pDev->dev_capab;
    p2pDevLite->group_capab = p2pDev->group_capab;
    p2pDevLite->persistent_grp = p2pDev->persistent_grp;
}

A_VOID
Custom_Api_p2p_node_list_event(A_VOID *pCxt, A_UINT8 *datap, A_UINT32 len)
{

    A_UINT32 evt_id = 0;
    A_UINT8 *tmpBuf,i;
    struct p2p_device_lite *p2pDevLite;
    struct p2p_device *p2pDev;
    evt_id = WMI_P2P_NODE_LIST_EVENTID;
    WMI_P2P_NODE_LIST_EVENT *handleP2PDev = (WMI_P2P_NODE_LIST_EVENT *)datap;
    A_DRIVER_CONTEXT* pDCxt = GET_DRIVER_COMMON(pCxt);  
    
    tmpBuf = GET_DRIVER_CXT(pCxt)->pScanOut;
   
    DRIVER_SHARED_RESOURCE_ACCESS_ACQUIRE(pCxt);
    {

       A_MEMZERO(GET_DRIVER_CXT(pCxt)->pScanOut,  pDCxt->tempStorageLength);
       A_MEMCPY(GET_DRIVER_CXT(pCxt)->pScanOut, &evt_id, sizeof(A_UINT32));
       tmpBuf += sizeof(A_UINT32);
       *tmpBuf = handleP2PDev->num_p2p_dev;
       tmpBuf++;
       
       for(i=0;i<handleP2PDev->num_p2p_dev;i++){
           p2pDevLite = (struct p2p_device_lite *)(tmpBuf + i * sizeof(struct p2p_device_lite));
           p2pDev = (struct p2p_device *) ((A_UINT8 *)handleP2PDev->data + (i * sizeof(struct p2p_device)));
           fill_P2PLite(p2pDev,p2pDevLite);
       }
           
       //A_MEMCPY(((GET_DRIVER_CXT(pCxt)->pScanOut)+sizeof(A_UINT32)) , (datap), sizeof(struct p2p_device));
      
        GET_DRIVER_CXT(pCxt)->p2pEvtState = A_FALSE;

    }
    DRIVER_SHARED_RESOURCE_ACCESS_RELEASE(pCxt);

    CUSTOM_DRIVER_WAKE_USER(pCxt);    

}



A_VOID
Custom_Api_p2p_req_auth_event(A_VOID *pCxt, A_UINT8 *datap, A_UINT32 len)
{
    A_UINT32 evt_id = 0;
    evt_id =  WMI_P2P_REQ_TO_AUTH_EVENTID;
    A_DRIVER_CONTEXT* pDCxt = GET_DRIVER_COMMON(pCxt);
   
    DRIVER_SHARED_RESOURCE_ACCESS_ACQUIRE(pCxt);
    {
        A_MEMZERO(GET_DRIVER_CXT(pCxt)->pScanOut,  pDCxt->tempStorageLength);
       
	    A_MEMCPY(GET_DRIVER_CXT(pCxt)->pScanOut, &evt_id, sizeof(A_UINT32));

        A_MEMCPY((GET_DRIVER_CXT(pCxt)->pScanOut)+sizeof(A_UINT32) , datap, sizeof(WMI_P2P_REQ_TO_AUTH_EVENT));

        GET_DRIVER_CXT(pCxt)->p2pevtflag = A_TRUE;
    }
    DRIVER_SHARED_RESOURCE_ACCESS_RELEASE(pCxt);

    CUSTOM_DRIVER_WAKE_USER(pCxt);
}

A_VOID
Custom_Api_p2p_list_persistent_network_event(A_VOID *pCxt, A_UINT8 *datap, A_UINT32 len)
{
    A_UINT32 evt_id = 0;
    evt_id =  WMI_P2P_LIST_PERSISTENT_NETWORK_EVENTID;
    A_DRIVER_CONTEXT* pDCxt = GET_DRIVER_COMMON(pCxt);
    WMI_P2P_PERSISTENT_LIST_NETWORK_EVENT *ev = (WMI_P2P_PERSISTENT_LIST_NETWORK_EVENT *)datap;
    
    DRIVER_SHARED_RESOURCE_ACCESS_ACQUIRE(pCxt);
    {
        A_MEMZERO(GET_DRIVER_CXT(pCxt)->pScanOut,  pDCxt->tempStorageLength);

	A_MEMCPY(GET_DRIVER_CXT(pCxt)->pScanOut, &evt_id, sizeof(A_UINT32));

        A_MEMCPY((GET_DRIVER_CXT(pCxt)->pScanOut)+sizeof(A_UINT32) , ev->data, (MAX_LIST_COUNT * sizeof(WMI_PERSISTENT_MAC_LIST)));
        
        GET_DRIVER_CXT(pCxt)->p2pEvtState = A_FALSE;
    }
    DRIVER_SHARED_RESOURCE_ACCESS_RELEASE(pCxt);

    CUSTOM_DRIVER_WAKE_USER(pCxt);    
}

A_VOID
Custom_Api_get_p2p_ctx(A_VOID *pCxt, A_UINT8 *datap, A_UINT32 len)
{

    A_UINT32 evt_id = 0;
    evt_id = WMI_P2P_PROV_DISC_RESP_EVENTID;
    A_DRIVER_CONTEXT* pDCxt = GET_DRIVER_COMMON(pCxt);
    
    DRIVER_SHARED_RESOURCE_ACCESS_ACQUIRE(pCxt);
    {
        A_MEMZERO(GET_DRIVER_CXT(pCxt)->pScanOut,  pDCxt->tempStorageLength);
              
    	A_MEMCPY(GET_DRIVER_CXT(pCxt)->pScanOut, &evt_id, sizeof(A_UINT32));

        A_MEMCPY((GET_DRIVER_CXT(pCxt)->pScanOut)+sizeof(A_UINT32) , datap, sizeof(WMI_P2P_PROV_DISC_RESP_EVENT));

        GET_DRIVER_CXT(pCxt)->p2pEvtState = A_FALSE;

    }
    DRIVER_SHARED_RESOURCE_ACCESS_RELEASE(pCxt);

    CUSTOM_DRIVER_WAKE_USER(pCxt);

}


A_VOID
Custom_Api_p2p_prov_disc_req(A_VOID *pCxt, A_UINT8 *datap, A_UINT32 len)
{
   A_UINT32 evt_id = 0;
   evt_id = WMI_P2P_PROV_DISC_REQ_EVENTID;
   A_DRIVER_CONTEXT* pDCxt = GET_DRIVER_COMMON(pCxt);

    DRIVER_SHARED_RESOURCE_ACCESS_ACQUIRE(pCxt);
    {
        A_MEMZERO(GET_DRIVER_CXT(pCxt)->pScanOut,  pDCxt->tempStorageLength);
      
        A_MEMCPY(GET_DRIVER_CXT(pCxt)->pScanOut, &evt_id, sizeof(A_UINT32));

        A_MEMCPY((GET_DRIVER_CXT(pCxt)->pScanOut)+sizeof(A_UINT32) , datap, sizeof(WMI_P2P_PROV_DISC_REQ_EVENT));

        GET_DRIVER_CXT(pCxt)->p2pevtflag = A_TRUE;
 
    }
    DRIVER_SHARED_RESOURCE_ACCESS_RELEASE(pCxt);

    CUSTOM_DRIVER_WAKE_USER(pCxt);

}

A_VOID
Custom_Api_p2p_serv_disc_req(A_VOID *pCxt, A_UINT8 *datap, A_UINT32 len)
{

   A_UINT32 evt_id = 0;
   evt_id = WMI_P2P_SDPD_RX_EVENTID;
   A_DRIVER_CONTEXT* pDCxt = GET_DRIVER_COMMON(pCxt);
    DRIVER_SHARED_RESOURCE_ACCESS_ACQUIRE(pCxt);
    {
        A_MEMZERO(GET_DRIVER_CXT(pCxt)->pScanOut,  pDCxt->tempStorageLength);
        
        A_MEMCPY(GET_DRIVER_CXT(pCxt)->pScanOut, &evt_id, sizeof(A_UINT32));

        A_MEMCPY((GET_DRIVER_CXT(pCxt)->pScanOut)+sizeof(A_UINT32) , datap, sizeof(WMI_P2P_SDPD_RX_EVENT));

        GET_DRIVER_CXT(pCxt)->p2pevtflag = A_TRUE;
 
    }
    
    DRIVER_SHARED_RESOURCE_ACCESS_RELEASE(pCxt);

    CUSTOM_DRIVER_WAKE_USER(pCxt);
}

A_VOID
Custom_Api_p2p_invite_req(A_VOID *pCxt, A_UINT8 *datap, A_UINT32 len)
{
    //async event for peer invite
   A_UINT32 evt_id = 0;
   evt_id = WMI_P2P_INVITE_REQ_EVENTID;
   A_DRIVER_CONTEXT* pDCxt = GET_DRIVER_COMMON(pCxt);

   DRIVER_SHARED_RESOURCE_ACCESS_ACQUIRE(pCxt);
    {
        A_MEMZERO(GET_DRIVER_CXT(pCxt)->pScanOut,  pDCxt->tempStorageLength);
      
    	A_MEMCPY(GET_DRIVER_CXT(pCxt)->pScanOut, &evt_id, sizeof(A_UINT32));

        A_MEMCPY((GET_DRIVER_CXT(pCxt)->pScanOut)+sizeof(A_UINT32) , datap, sizeof(WMI_P2P_FW_INVITE_REQ_EVENT));

        GET_DRIVER_CXT(pCxt)->p2pevtflag = A_TRUE;
 
    }
    
    DRIVER_SHARED_RESOURCE_ACCESS_RELEASE(pCxt);

    CUSTOM_DRIVER_WAKE_USER(pCxt);    

    
}

A_VOID
Custom_Api_p2p_invite_rcvd_result(A_VOID *pCxt, A_UINT8 *datap, A_UINT32 len)
{
  
   A_UINT32 evt_id = 0;
   evt_id = WMI_P2P_INVITE_RCVD_RESULT_EVENTID;
   A_DRIVER_CONTEXT* pDCxt = GET_DRIVER_COMMON(pCxt);
   
    DRIVER_SHARED_RESOURCE_ACCESS_ACQUIRE(pCxt);
    {
        A_MEMZERO(GET_DRIVER_CXT(pCxt)->pScanOut,  pDCxt->tempStorageLength);

	    A_MEMCPY(GET_DRIVER_CXT(pCxt)->pScanOut, &evt_id, sizeof(A_UINT32));

        A_MEMCPY((GET_DRIVER_CXT(pCxt)->pScanOut)+sizeof(A_UINT32) , datap, sizeof(WMI_P2P_INVITE_RCVD_RESULT_EVENT));

        GET_DRIVER_CXT(pCxt)->p2pEvtState = A_FALSE;

    }
    DRIVER_SHARED_RESOURCE_ACCESS_RELEASE(pCxt);

    CUSTOM_DRIVER_WAKE_USER(pCxt);    
    
}

A_VOID
Custom_Api_p2p_invite_send_result(A_VOID *pCxt, A_UINT8 *datap, A_UINT32 len)
{
    A_UINT32 evt_id = 0;
    A_DRIVER_CONTEXT* pDCxt = GET_DRIVER_COMMON(pCxt);
    evt_id = WMI_P2P_INVITE_SENT_RESULT_EVENTID;

    DRIVER_SHARED_RESOURCE_ACCESS_ACQUIRE(pCxt);
    {
    
        A_MEMZERO(GET_DRIVER_CXT(pCxt)->pScanOut,  pDCxt->tempStorageLength);

	    A_MEMCPY(GET_DRIVER_CXT(pCxt)->pScanOut, &evt_id, sizeof(A_UINT32));

        A_MEMCPY((GET_DRIVER_CXT(pCxt)->pScanOut)+sizeof(A_UINT32) , datap, sizeof(WMI_P2P_INVITE_SENT_RESULT_EVENT));

        GET_DRIVER_CXT(pCxt)->p2pevtflag = A_TRUE;

    }
    DRIVER_SHARED_RESOURCE_ACCESS_RELEASE(pCxt);

    CUSTOM_DRIVER_WAKE_USER(pCxt);       
    
}

#endif
#if MANUFACTURING_SUPPORT
A_VOID
Custom_Api_Test_Cmd_Event(A_VOID *pCxt, A_UINT8 *datap, A_UINT32 len)
{
  UNUSED_ARGUMENT(len);
  A_MEMCPY(GET_DRIVER_CXT(pCxt)->pScanOut,datap,len);
  GET_DRIVER_CXT(pCxt)->testCmdRespBufLen = len;
}
#endif

A_VOID
Custom_Api_GpioDataEvent(A_VOID *wmip, A_UINT8 *datap, A_INT32 len)
{
    if(ath_custom_init.Api_GpioDataEventRx != NULL){
	ath_custom_init.Api_GpioDataEventRx(datap, len);					
    }
}

/* EOF */
