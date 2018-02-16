/*
 * Copyright (c) 2015 Turingsense
 *
 * cwati June 26, 15
 *
 * wmiconfig_ts.c
 *
 */

#include <main.h>
#include <throughput.h>
#include "wmiconfig_ts.h"		/* Turingsense */

#if ENABLE_P2P_MODE
#include "p2p.h"
char p2p_wps_pin_val[P2P_WPS_PIN_LEN], set_channel_p2p[P2P_CHANNEL_LEN];
A_INT32 p2p_callback(A_UINT8 evt_flag);
#endif

/**************************Globals ************************************************/
A_UINT8 pmk_flag = 0, hidden_flag = 0, wps_flag = 0, tx_raw = 0;
AP_CFG_CMD ap_config_param;
mode_t mode_flag = MODE_STATION;
WMI_POWER_MODE power_mode = REC_POWER;
char ssid[MAX_SSID_LENGTH];
A_UINT8 original_ssid[MAX_SSID_LENGTH];               //Used to store user specified SSID
wps_context_t wps_context;
A_UINT32 wifi_connected_flag = 0;
#if ENABLE_P2P_MODE
A_INT32 task2_msec = 100;
A_UINT8 p2p_connection_default = 0, p2p_cancel_enable = 0; /* 
                                       0 - Allow other P2P Device to connect(Accept connection)
                                       1 - Allow IOE Device to connect(Issue connect)
                                    */
A_CHAR p2p_wps_pin[WPS_PIN_LEN];
A_UINT8 p2p_intent = 0;
A_BOOL autogo_newpp = FALSE;
WMI_PERSISTENT_MAC_LIST         p2p_pers_data[MAX_LIST_COUNT];
A_UINT8 invitation_index=0;
#endif
A_UINT16 channel_array[] =
{
	0,2412,2417,2422,2427,2432,2437,2442,2447,2452,2457,2462,2467,2472,2482
};
static int security_mode = 0;
cipher_t cipher;
char wpa_ver[8];
key_t key[MAX_NUM_WEP_KEYS];
char wpsPin[MAX_WPS_PIN_SIZE];
char wpa_passphrase[MAX_PASSPHRASE_SIZE];
volatile A_UINT8 wifi_state = 0;
//A_INT32 block_on_dhcp();
/************ Function Prototypes *********************************************/
A_UINT32 set_scan
    (
        A_UINT32 dev_num,
        char* ssid,
        A_UINT32 *pNum_ap /* [OUT] */
    );

//void resetTarget();
//void init_wep_keys();
A_INT32 clear_wep_keys();
static A_INT32 connect_handler( A_INT32 index, A_INT32 argc, char* argv[]);
static A_INT32 disconnect_handler( );
static A_INT32 set_tx_power(char* dBm);
static A_INT32 set_channel(char* channel);
static A_INT32 set_pmparams(A_INT32 index, A_INT32 argc, char* argv[]);
//static A_INT32 set_host_mode(char* host_mode);
static A_INT32 set_listen_interval(char* listen_int);
static A_INT32 set_phy_mode(char* phy_mode);
static A_INT32 set_cipher(A_UINT32 dev_num,char* cipher);
static A_INT32 get_rssi();
static A_INT32 get_rate();
static A_INT32 set_rate(A_INT32 index, A_INT32 argc, char* argv[]);
static A_INT32 set_tx_power_scale(A_INT32 index, A_INT32 argc,char* argv[]);
static A_INT32 set_gtx(char* gtx);
static A_INT32 set_lpl(char* lpl);
static A_INT32 mac_store(char *argv[]);
static A_INT32 gpio_handle(A_INT32 argc,char *argv[]);

static A_INT32 set_wep_key(char* key_index, char* key_val);
A_INT32 set_wep_keys(
        A_UINT32 dev_num,
        char* wep_key1,
        char* wep_key2,
        char* wep_key3,
        char* wep_key4,
        A_UINT32 key_len,
        A_UINT32 key_index);

void wifiCallback(int val);

#if ENABLE_P2P_MODE
static int_32 p2p_handler( int_32 index, int_32 argc, char_ptr argv[]);
uint_32 StartP2P(A_UINT8 p2p_command, A_UINT8 *p2p_param_ptr);
void P2PCallBackfn(A_UINT8 *ptr);
#endif

#if ENABLE_AP_MODE
static A_INT32 ap_handler( int_32 index, int_32 argc, char_ptr argv[]);
static A_INT32 set_ap_params(A_UINT16 param_cmd, A_UINT8 *data_val);
#endif

A_INT32 get_phy_mode
    (
        A_UINT32 dev_num,
       char* phy_mode
    );

static A_INT32 set_passphrase(char* passphrase);
A_INT32 dev_susp_enable();
A_INT32 dev_susp_start(char* susp_time);
A_INT32 get_version();

static A_INT32 ad_hoc_connect_handler( A_INT32 index, A_INT32 argc, char* argv[]);
static A_INT32 wps_handler( A_INT32 index, A_INT32 argc, char* argv[]);
#if ENABLE_P2P_MODE
static A_STATUS wmic_ether_aton(const char *orig, A_UINT8 *eth);
#endif
static A_INT32 scan_control( A_INT32 argc, char* argv[]);
static A_INT32 ath_assert_dump( A_INT32 argc, char* argv[]);

unsigned char ascii_to_hex(char val);



A_INT32 wps_query(A_UINT8 block);
static A_INT32 test_promiscuous(A_INT32 argc, char* argv[]);
static A_INT32 test_raw(A_INT32 argc, char* argv[]);

static A_INT32 allow_aggr(A_INT32 argc, char* argv[]);

static A_INT32 get_mac_addr(char* mac_addr);
static A_INT32 get_channel_val(char_ptr channel_val);
static A_INT32 set_wpa( A_INT32 index, A_INT32 argc, char* argv[]);
static A_INT32 set_wep( A_INT32 index, A_INT32 argc, char* argv[]);
A_INT32 get_last_error();
A_INT32 parse_ipv4_ad(unsigned long* ip_addr, unsigned* sbits, char* ip_addr_str);

static A_INT32 get_reg_domain( );

void SystemReset();
int ishexdigit(char digit);
//extern void ath_tcp_rx_multi_TCP (int port);
#if ENABLE_STACK_OFFLOAD
A_INT32 custom_ipdns(void *handle,A_INT32 command, char *domain_name, IP46ADDR *dnsaddr);
A_INT32 ipconfig_dhcp_pool(A_INT32 argc, char* argv[]);
A_INT32 ip6_set_router_prefix(A_INT32 argc,char *argv[]);
static A_INT32 ip_set_tcp_exponential_backoff_retry(A_INT32 argc,char *argv[]);
static A_INT32 mac_store(char *argv[]);
static A_INT32 ip6_set_status(A_INT32 argc,char *argv[]);
static A_INT32 ipconfig_dhcp_release(A_INT32 argc,char *argv[]);
static A_INT32 ipconfig_tcp_rxbuf(A_INT32 argc,char *argv[]);
#if ENABLE_HTTP_SERVER
static A_INT32 ip_http_server(A_INT32 argc,char *argv[]);
static A_INT32 ip_http_server_post(A_INT32 argc,char *argv[]);
static A_INT32 ip_http_server_get(A_INT32 argc,char *argv[]);
#endif
#if ENABLE_HTTPS_SERVER
//static A_INT32 https_server(A_INT32 argc,char *argv[]);
#endif
#if ENABLE_HTTPS_CLIENT
//static A_INT32 https_client(A_INT32 argc,char *argv[]);
#endif
#if ENABLE_DNS_CLIENT
static A_INT32 ip_resolve_hostname(A_INT32 argc,char *argv[]);
A_INT32 ip_get_addr_for_host_name(char *hostname,A_UINT32 *v4addr,IP6_ADDR_T *ip6addr,A_UINT32 domain);
static A_INT32 ip_gethostbyname(A_INT32 argc,char *argv[]);
static A_INT32 ip_gethostbyname2(A_INT32 argc,char *argv[]);
static A_INT32 ip_dns_client(A_INT32 argc,char *argv[]);
static A_INT32 ip_dns_server_addr(A_INT32 argc,char *argv[]);
static A_INT32 ip_dns_delete_server_addr(A_INT32 argc,char *argv[]);
#endif
#if ENABLE_SSL
static A_INT32 ssl_start(A_INT32 argc,char *argv[]);
static A_INT32 ssl_stop(A_INT32 argc,char *argv[]);
static A_INT32 ssl_config(A_INT32 argc,char *argv[]);
static A_INT32 ssl_add_cert(A_INT32 argc,char *argv[]);
static A_INT32 ssl_store_cert(A_INT32 argc,char *argv[]);
static A_INT32 ssl_delete_cert(A_INT32 argc,char *argv[]);
static A_INT32 ssl_list_cert(A_INT32 argc,char *argv[]);
#endif
static A_INT32 gpio_handle(A_INT32 argc,char *argv[]);
void configure_auto_ipv4_addr();
static A_INT32 iphostname(A_INT32 argc,char *argv[]);
#if ENABLE_DNS_SERVER
static A_INT32 ip_dns_local_domain(A_INT32 argc,char *argv[]);
static A_INT32 ipdns(A_INT32 argc,char *argv[]);
#endif
#if ENABLE_SNTP_CLIENT
static A_INT32 ip_sntp_srvr(A_INT32 argc,char *argv[]);
static A_INT32 ip_sntp_get_time(A_INT32 argc,char *argv[]);
static A_INT32 ip_sntp_get_time_of_day(A_INT32 argc,char *argv[]);
static A_INT32 ip_sntp_modify_timezone_dse(A_INT32 argc, char *argv[]);
static A_INT32 ip_query_sntp_server_address(A_UINT32 argc,char *argv[]);
static A_INT32 ip_sntp_client_start_stop(A_UINT32 argc,char *argv[]);
#endif
#endif /* ENABLE_STACK_OFFLOAD */
#if ANTENNA_DIVERSITY
static A_INT32 hwantdivset(A_INT32 argc,char *argv[]);
static A_INT32 hwantdivget(A_INT32 argc,char *argv[]);
void AntennaStatCallback(void *ptr);
#endif

int read_shared_spi_flash (int argc, char *argv[]);
int open_shared_spi_flash (int argc, char *argv[]);
int close_shared_spi_flash (int argc, char *argv[]);
int kf_read_shared_spi_flash (int argc, char *argv[]);

/*FUNCTION*-----------------------------------------------------------------
*
* Function Name  : mystrtomac
* Returned Value : NA
* Comments       : converts a string to a 6 byte MAC address.
*
*END------------------------------------------------------------------*/
static int mystrtomac(const char* arg, unsigned char* dst)
{
	int i=0;
	int j=0;
	int left_nibble = 1;
	char base_char;
	unsigned char base_val;

	if(dst == NULL || arg == NULL){
		return -1;
	}

	A_MEMZERO(dst, 6);

	do{
		if(arg[i] >= '0' && arg[i] <= '9'){
			base_char = '0';
			base_val = 0x00;
		}else if(arg[i] == ':'){
			base_char = ':';
		}else if(arg[i] >= 'a' && arg[i] <= 'f'){
			base_char = 'a';
			base_val = 0x0a;
		}else if(arg[i] >= 'A' && arg[i] <= 'F'){
			base_char = 'A';
			base_val = 0x0a;
		}else{
			return -1;//error
		}

		if(base_char != ':'){
			dst[j] |= (arg[i] - base_char + base_val)<<(4*left_nibble);
			left_nibble = (left_nibble)? 0:1;

			if(left_nibble){
				j++;

				if(j>5){
					break;
				}
			}
		}

		i++;
	}while(1);

	return 0;//success
}



/*FUNCTION*-----------------------------------------------------------------
*
* Function Name  : wmiconfig_handler
* Returned Value : ERROR code on error else success
* Comments       : 1st stop for all wmiconfig commands
*
*END------------------------------------------------------------------*/

A_INT32 wmiconfig_handler(A_INT32 argc, char* argv[] )
{ /* Body */
    A_BOOL                 shorthelp = FALSE;
    A_UINT8 		    printhelp = 0;
    A_INT32                 return_code = A_OK;
    A_UINT32                 enet_device = DEMOCFG_DEFAULT_DEVICE, index = 1;

    if (argc > index)
    {
        if (ATH_STRCMP(argv[index], "--connect") == 0)
        {
            if(argc == index+1)
            {
                printf("Incomplete parameters for connect command\n");
                return A_ERROR;
            }
            return_code = connect_handler(index+1,argc,argv);
        }
        else if (ATH_STRCMP(argv[index], "--adhoc") == 0)
        {
            if(argc == index+1)
            {
                printf("Incomplete parameters for ad-hoc command\n");
                return A_ERROR;
            }
            mode_flag = MODE_ADHOC;
            return_code = ad_hoc_connect_handler(index+1,argc,argv);
        }
#if ENABLE_AP_MODE
        else if (ATH_STRCMP(argv[index], "--mode") == 0)
        {
            if(argc == index+1)
            {
                printf("Usage:: --mode <ap[<hidden>][<wps>]|station>  \n");
                return A_ERROR;
            }

            ath_set_essid (DEMOCFG_DEFAULT_DEVICE,"");

            if(ATH_STRCMP(argv[index+1], "ap") == 0)
            {
                mode_flag = MODE_AP;
                if (ATH_STRCMP(argv[index+2], "hidden") == 0)
                {
                    hidden_flag = 1;
                }
                if (ATH_STRCMP(argv[index+2], "wps") == 0)
                {
                    wps_flag = 1;
                }
            }
            else if(ATH_STRCMP(argv[index+1], "station") == 0)
            {
                if (mode_flag == MODE_AP) /*check previous mode*/
                {
                    disconnect_handler();
                    /* AP Mode is always set to maxper; if we are switching mode and prev mode is REC_POWER then
                       retain the power mode for STA */
                    if(power_mode == REC_POWER){
                        SET_POWER_MODE("1");
                    }
                }

                mode_flag = MODE_STATION;

                if (ATH_STRCMP(argv[index+2], "hidden") == 0)
                {
                    printf("Usage:: --mode <ap[<hidden>]|station>  \n");
                    return A_ERROR;
                }
                if (ATH_STRCMP(argv[index+2], "wps") == 0)
                {
                    printf("Usage:: --mode <ap[<wps>]|station>  \n");
                    return A_ERROR;
                }
            }
            else
            {
                printf("Usage:: --mode <ap[<hidden>][<wps>]|station>  \n");
                return A_ERROR;
            }
        }
        else if (ATH_STRCMP(argv[index], "--ap") == 0)
        {
            if(argc == index+1)
            {
                printf ("usage:--ap [country <country code>]\n"
                         "      --ap [bconint<intvl>]\n"
                         "      --ap [dtim<val>]\n"
                         "      --ap [inact <minutes>]> \n"
                         "      --ap [psbuf <1/0-enable/disable> <buf_count for enable>]>\n");
                return A_ERROR;
            }
            return_code = ap_handler(index+1,argc,argv);
        }
#endif
        else if (ATH_STRCMP(argv[index], "--wps") == 0)
        {
            return_code = wps_handler(index+1,argc,argv);
        }
#if ENABLE_P2P_MODE
        else if (ATH_STRCMP(argv[index], "--p2p") == 0)
        {
            return_code = p2p_handler(index+1,argc,argv);
        }
#endif
        else if (ATH_STRCMP(argv[index], "--disc") == 0)
        {
            return_code = disconnect_handler();
        }
        else if (ATH_STRCMP(argv[index], "--settxpower") == 0)
        {
            return_code = set_tx_power(argv[index+1]);
        }
        else if (ATH_STRCMP(argv[index], "--pwrmode") == 0)
        {
            if(ATH_STRCMP(argv[index+1], "0") == 0){
                power_mode = MAX_PERF_POWER;
            }else{
                if(mode_flag == MODE_AP) {
                    printf("Setting REC Power is not allowed MODE_AP \n");
                    return A_ERROR;
                }
                power_mode = REC_POWER;
            }
            return_code = SET_POWER_MODE(argv[index+1]);
        }
        else if (ATH_STRCMP(argv[index], "--pmparams") == 0)
        {
            return_code = set_pmparams(index+1,argc,argv);
        }
        else if (ATH_STRCMP(argv[index], "--channel") == 0)
        {
            return_code = set_channel(argv[index+1]);
        }
        else if (ATH_STRCMP(argv[index], "--listen") == 0)
        {
            return_code = set_listen_interval(argv[index+1]);
        }
        else if (ATH_STRCMP(argv[index], "--wmode") == 0)
        {
            return_code = set_phy_mode(argv[index+1]);
        }
        else if (ATH_STRCMP(argv[index], "--rssi") == 0)
        {
            return_code = get_rssi();
        }
        else if (ATH_STRCMP(argv[index], "--gettxstat") == 0)
        {
            return_code = get_tx_status();
        }
        else if (ATH_STRCMP(argv[index], "--getrate") == 0)
        {
            return_code = get_rate();
        }
        else if (ATH_STRCMP(argv[index], "--setrate") == 0)
        {
            return_code = set_rate(index+1, argc, argv);
        }
        else if (ATH_STRCMP(argv[index], "--txpwrscale") == 0)
        {
            return_code = set_tx_power_scale(index+1, argc, argv);
        }
        else if (ATH_STRCMP(argv[index], "--wepkey") == 0)
        {
            if((argc - index) > 2)
                return_code = set_wep_key(argv[index+1],argv[index+2]);
            else
            {
                printf("Missing parameters\n\n");
                return A_ERROR;
            }
        }
        else if (ATH_STRCMP(argv[index], "--p") == 0)
        {
            /*We expect one parameter with this command*/
            if((argc - index) > 1)
                return_code = set_passphrase(argv[index+1]);
            else
            {
                printf("Missing parameters\n\n");
                return A_ERROR;
            }
        }
        else if (ATH_STRCMP(argv[index], "--wpa") == 0)
        {
            return_code = set_wpa(index+1,argc,argv);
        }
        else if (ATH_STRCMP(argv[index], "--wep") == 0)
        {
            return_code = set_wep(index+1,argc,argv);
        }
        else if (ATH_STRCMP(argv[index], "--suspend") == 0)
        {
            return_code = dev_susp_enable();
        }
        else if (ATH_STRCMP(argv[index], "--suspstart") == 0)
        {
            return_code = dev_susp_start(argv[index+1]);
        }
        else if (ATH_STRCMP(argv[index], "--version") == 0)
        {
            return_code = get_version();
        }
        else if (ATH_STRCMP(argv[index], "--help") == 0)
        {
            //   return_code = A_ERROR;
            printhelp = 1;
        }

        else if (ATH_STRCMP(argv[index], "--testflash") == 0)
        {
            return_code = TEST_FLASH;
        }
        else if (ATH_STRCMP(argv[index], "--scanctrl") == 0)
        {
            return_code = scan_control(argc-(index+1), &argv[index+1]);
        }
        else if (ATH_STRCMP(argv[index], "--reg") == 0)
        {
            return_code = ATH_REG_QUERY(argc-(index+1), &argv[index+1]);
        }
        else if(ATH_STRCMP(argv[index], "--fwmem") == 0)
        {
            return_code = ATH_MEM_QUERY(argc-(index+1), &argv[index+1]);
        }
        else if (ATH_STRCMP(argv[index], "--assdump") == 0)
        {
            return_code = ath_assert_dump(argc-(index+1), &argv[index+1]);
        }
        else if (ATH_STRCMP(argv[index], "--driver") == 0)
        {
            return_code = ath_driver_state(argc-(index+1), &argv[index+1]);
        }
        else if (ATH_STRCMP(argv[index], "--raw") == 0)
        {
            return_code = test_raw(argc-(index+1), &argv[index+1]);
        }
        else if (ATH_STRCMP(argv[index], "--promisc") == 0)
        {
            return_code = test_promiscuous(argc-(index+1), &argv[index+1]);
        }
        else if (ATH_STRCMP(argv[index], "--allow_aggr") == 0)
        {
            return_code = allow_aggr(argc-(index+1), &argv[index+1]);
        }
        else if (ATH_STRCMP(argv[index], "--reset") == 0)
        {
            SystemReset();
        }
        else if (ATH_STRCMP(argv[index], "--regdomain") == 0)
        {
            get_reg_domain();
        }
        else if (ATH_STRCMP(argv[index], "--getlasterr") == 0)
        {
            get_last_error();
        }
        else if (ATH_STRCMP(argv[index], "--gtx") == 0)
        {
            return_code = set_gtx (argv[index+1]);
        }
        else if (ATH_STRCMP(argv[index], "--lpl") == 0)
        {
            return_code = set_lpl (argv[index+1]);
        }
        else if (ATH_STRCMP(argv[index], "--macstore") == 0)
        {
            return_code = mac_store(&argv[index+1]);
        }
        else if (ATH_STRCMP(argv[index], "--gpio") == 0)
        {
            return_code = gpio_handle(argc,argv);
        }

#if ENABLE_STACK_OFFLOAD
        else if (ATH_STRCMP(argv[index], "--ipconfig") == 0)
        {
            return_code = ipconfig_query(argc,argv);
        }
        else if (ATH_STRCMP(argv[index], "--ipstatic") == 0)
        {
            return_code = ipconfig_static(argc,argv);
        }
        else if (ATH_STRCMP(argv[index], "--ipdhcp") == 0)
        {
            return_code = ipconfig_dhcp(argc,argv);
        }
#if 0
        else if (ATH_STRCMP(argv[index], "--blockdhcp") == 0)
        {
            return_code = block_on_dhcp(argc,argv);
        }
#endif
        else if (ATH_STRCMP(argv[index], "--udpecho") == 0)
        {
            //ath_udp_echo(argc,argv);
        }
        /*else if (ATH_STRCMP(argv[index], "--multi") == 0)
        {
        ath_tcp_rx_multi_TCP(atoi(argv[2]));
    }*/
        else if(ATH_STRCMP(argv[index],"--ipdhcppool") == 0)
        {
            return_code = ipconfig_dhcp_pool(argc,argv);
        }
		else if(ATH_STRCMP(argv[index],"--ip6rtprfx") == 0)
        {
            return_code = ip6_set_router_prefix(argc,argv);
        }
		else if(ATH_STRCMP(argv[index],"--tcp_backoff_retry") == 0)
		{
            return_code = ip_set_tcp_exponential_backoff_retry(argc,argv);
		}
		else if(ATH_STRCMP(argv[index],"--ipv6") == 0)
		{
            return_code = ip6_set_status(argc,argv);
		}
        else if(ATH_STRCMP(argv[index],"--ipdhcp_release") == 0)
		{
            return_code = ipconfig_dhcp_release(argc,argv);
		}
        else if(ATH_STRCMP(argv[index],"--tcp_rxbuf") == 0)
		{
            return_code = ipconfig_tcp_rxbuf(argc,argv);
		}
#if ENABLE_DNS_CLIENT
        else if (ATH_STRCMP(argv[index],"--ip_resolve_hostname") == 0)
        {
            return_code = ip_resolve_hostname(argc, argv);
        }
		else if(ATH_STRCMP(argv[index],"--ip_gethostbyname") == 0)
        {
            return_code = ip_gethostbyname(argc, argv);
		}
        else if(ATH_STRCMP(argv[index],"--ip_gethostbyname2") == 0)
		{
            return_code = ip_gethostbyname2(argc, argv);
        }
        else if(ATH_STRCMP(argv[index],"--ip_dns_client") == 0)
        {
            return_code = ip_dns_client(argc, argv);
        }
        else if(ATH_STRCMP(argv[index],"--ip_dns_server_addr") == 0)
        {
            return_code = ip_dns_server_addr(argc, argv);
		}
        else if(ATH_STRCMP(argv[index],"--ip_dns_delete_server_addr") == 0)
        {
            return_code = ip_dns_delete_server_addr(argc, argv);
	}
#endif /* ENABLE_DNS_CLIENT */
#if ENABLE_HTTP_SERVER
        else if (ATH_STRCMP(argv[index],"--ip_http_server") == 0)
        {
            return_code = ip_http_server(argc, argv);
        }
        else if (ATH_STRCMP(argv[index],"--ip_http_post") == 0)
        {
            return_code = ip_http_server_post(argc, argv);
        }
        else if (ATH_STRCMP(argv[index],"--ip_http_get") == 0)
        {
            return_code = ip_http_server_get(argc, argv);
        }
#endif /* ENABLE_HTTP_SERVER */
#if ENABLE_HTTP_CLIENT
        else if(ATH_STRCMP(argv[index], "--ip_http_client") == 0)
        {
             return_code = httpc_command_parser(argc, argv);
        }
#endif /* ENABLE_HTTP_CLIENT */

        else if(ATH_STRCMP(argv[index], "--iphostname") == 0)
        {
             return_code = iphostname(argc, argv);
        }
#if ENABLE_DNS_SERVER
        else if(ATH_STRCMP(argv[index], "--ip_dns_local_domain") == 0)
        {
             return_code = ip_dns_local_domain(argc, argv);
        }
        else if(ATH_STRCMP(argv[index], "--ip_dns") == 0)
        {
             return_code = ipdns(argc, argv);
        }
#endif /* ENABLE_DNS_SERVER */
#if ENABLE_SNTP_CLIENT
	else if(ATH_STRCMP(argv[index],"--ip_sntp_srvr") == 0)
	{
	     return_code = ip_sntp_srvr(argc,argv);
	}
        else if(ATH_STRCMP(argv[index],"--ip_sntp_get_time") == 0)
	{
	     return_code = ip_sntp_get_time(argc,argv);
	}
	else if(ATH_STRCMP(argv[index],"--ip_sntp_get_time_of_day") == 0)
	{
	     return_code = ip_sntp_get_time_of_day(argc,argv);

	}
	else if(ATH_STRCMP(argv[index],"--ip_sntp_zone") == 0)
	{
 	     return_code = ip_sntp_modify_timezone_dse(argc,argv);
	}
	else if(ATH_STRCMP(argv[index],"--ip_show_sntpconfig") == 0)
	{
	     return_code = ip_query_sntp_server_address(argc,argv);
	}
	else if(ATH_STRCMP(argv[index],"--ip_sntp_client") == 0)
	{
	     return_code = ip_sntp_client_start_stop(argc,argv);
	}
#endif /* ENABLE_SNTP_CLIENT */
#if ENABLE_SSL
        else if(ATH_STRCMP(argv[index], "--ssl_start") == 0)
        {
            return_code = ssl_start(argc, argv);
        }
        else if(ATH_STRCMP(argv[index], "--ssl_stop") == 0)
        {
            return_code = ssl_stop(argc, argv);
        }
        else if(ATH_STRCMP(argv[index], "--ssl_config") == 0)
        {
            return_code = ssl_config(argc, argv);
        }
        else if(ATH_STRCMP(argv[index], "--ssl_add_cert") == 0)
        {
            return_code = ssl_add_cert(argc, argv);
        }
        else if(ATH_STRCMP(argv[index], "--ssl_store_cert") == 0)
        {
            return_code = ssl_store_cert(argc, argv);
        }
        else if(ATH_STRCMP(argv[index], "--ssl_delete_cert") == 0)
        {
            return_code = ssl_delete_cert(argc, argv);
        }
        else if(ATH_STRCMP(argv[index], "--ssl_list_cert") == 0)
        {
            return_code = ssl_list_cert(argc, argv);
        }
#endif
#endif /* ENABLE_STACK_OFFLOAD */
#if ANTENNA_DIVERSITY
        else if(ATH_STRCMP(argv[index], "--hwantdivset") == 0)
        {
             return_code = hwantdivset(argc, argv);
        }
        else if(ATH_STRCMP(argv[index], "--hwantdivget") == 0)
        {
             return_code = hwantdivget(argc, argv);
        }
#endif
        else if (ATH_STRCMP(argv[index],"--read_flash") == 0)
        {
            return_code = read_shared_spi_flash(argc, argv);
        }
        else if (ATH_STRCMP(argv[index],"--open_flash") == 0)
        {
			return_code = open_shared_spi_flash (argc, argv);
        }
        else if(strcmp (argv[index], "--close_flash") == 0)
        {
			return_code = close_shared_spi_flash (argc, argv);
        }
        else if(strcmp (argv[index], "--kf_read_flash") == 0)
        {
			return_code = kf_read_shared_spi_flash (argc, argv);
        }
        else
        {
            printf ("Unknown wmiconfig command!\n");
            return_code = A_ERROR;
        }
    }
    else if (argc == 1)
    {
        char data[32+1];
        A_INT32 error;
        A_UINT16 channel_val = 0;
        error = ATH_GET_ESSID (enet_device,(char*)data);

        if (error != 0)
            return_code = A_ERROR;

        printf("ssid         =   %s\n",data);

        //ATH_GET_SECTYPE (enet_device,(char*)data);
        //printf("security type=   %s\n",data);

        get_phy_mode(enet_device,(char*)data);
        printf("Phy mode     =   %s\n",data);

        get_power_mode((char*)data);
        printf("Power mode   =   %s\n",data);

        get_mac_addr((char*)data);
        printf("Mac Addr     =   %02x:%02x:%02x:%02x:%02x:%02x \n", data[0], data[1], data[2], data[3], data[4], data[5]);
#if ENABLE_AP_MODE
        if(mode_flag == MODE_AP)
        {
            printf("mode         =   softap \n");
            if(pmk_flag)
            {
                printf("passphrase   =   %s \n",wpa_passphrase);
            }

        }
        else
#endif
            if(mode_flag == MODE_STATION)
            {
                printf("mode         =   station \n");
            }
            else
            {
                printf("mode         =   adhoc \n");
            }
        if(wifi_connected_flag == 1)
        {
            get_channel_val((char *)&channel_val);
            if(channel_val < 3000)
            {
              channel_val -= ATH_IOCTL_FREQ_1;
              if((channel_val/5) == 14)
              {
                  channel_val = 14;
              }
              else
              {
                  channel_val = (channel_val/5) + 1;
              }
            }
            else
            {
               channel_val -= ATH_IOCTL_FREQ_36;
               channel_val = 36 + (channel_val/5); // since in 11a channel 36 is the starting number
            }
            printf("channel      =   %d \n", channel_val);
        }
    }

    if ((return_code != A_OK))
    {
    	printf("To see a list of commands- wmiconfig --help\n");
    }

    if(printhelp)
    {
    	printhelp = 0;
        if (shorthelp)
        {
            printf ("%s [<device>] [<command>]\n", argv[0]);
        }
        else
        {
#if ENABLE_STACK_OFFLOAD
            printf ("IOT Stack Offload Feature Enabled\n");
#else
            printf ("IOT Stack Offload Feature Disabled\n");
#endif
            printf ("Usage: %s [<command>]\n", argv[0]);
            printf ("  Commands:\n");
            printf ("    --version        = Displays versions\n");
            printf ("    --reset          = reset host processor\n");
            printf ("    --suspend        = Enable device suspend mode(before connect command)\n");
            printf ("    --regdomain      = Get Regulatory Domain\n");
            printf ("    --getlasterr     = Prints last driver error \n");
            printf ("    --connect <ssid>\n");
            printf ("   \n ");
            printf ("   WPA Configuration\n ");
            printf ("      --p <passphrase>\n");
            printf ("      --wpa <ver> <ucipher> <mcipher>\n");
            printf ("      --connect <ssid> \n");
            printf ("   \n ");
            printf ("   WEP Configuration\n");
            printf ("      --wepkey <key_index> <key> \n");
            printf (" 	   --wep <def_keyix> \n");
            printf ("      --connect <ssid>\n");
            printf ("              where  <ssid>   :SSID of network\n");
            printf ("                     <ver>    :1-WPA, 2-RSN\n");
            printf ("                     <ucipher>:TKIP or CCMP\n");
            printf ("                     <mcipher>:TKIP or CCMP\n");
            printf ("                     <p>    :Passphrase for WPA\n");
            //printf ("                     <mode>   :<open> or <shared>\n");
            printf ("                     <key_index>: Entered WEP key index [1-4]\n");
            printf ("                     <def_keyix>: Default WEP key index [1-4]\n");
            printf ("                     <key>    :WEP key\n");
            printf ("                     *        :optional parameters\n");
            printf ("          Note: key can only be composed of Hex characters of length 10 or 26\n");
            printf ("   \n");
            printf ("    --adhoc <ssid> --channel <1-14>  = Connect in ad-hoc mode\n");
            printf ("    --adhoc <ssid> --wep  <def_keyix> --channel <1-14> = Connect in WEP ad-hoc mode\n");
            printf ("                                    where channel is optional and ranges from 1-14\n");
            printf ("   \n");
            printf ("    --wps <connect> <mode> <*pin> <*ssid> <*mac> <*channel>\n");
            printf ("              where  <connect> : 1 - Attempts to connect after wps\n");
            printf ("                               : 0 - No Attempts to connect after wps\n");
            printf ("                     <mode>    : <push> or <pin>\n");
            printf ("                     *         :optional parameters\n");
            printf ("   \n");
            printf ("    --p2p <mode> <*>\n");
            printf ("           where <mode>  :  <on/off> <setconfig> <connect> <find> <prov> <listen> \n");
            printf ("                            <cancel> <join> <auth> <wpspin> <autogo> <invite_auth> <passphrase> \n");
            printf ("                  *      :  optional parameters that need to be entered based on mode\n");
            printf ("   \n");
            printf ("    --disc            = Disconect from current AP\n");
            printf ("    --setrate <rate> , where rate can be 1,2,5,6,9,11,12,18,24,36,48,54 mbps or MCS \n");
            printf ("    --txpwrscale <val> , where val is in dbm \n");
            printf ("      eg:  wmiconfig --setrate 54 will set rate to 54 mbps \n");
            printf ("           wmiconfig --setrate mcs 0 will set rate to mcs_0 \n");
            printf ("    --settxpower <>   = set transmit power\n");
            printf ("    --pwrmode    <>   = set power mode 1=Power save, 0= Max Perf \n");
            printf ("    --setchannel <1-14>   = set channel hint\n");
            printf ("    --wmode      <>   = set mode <b,g,n>\n");
            printf ("    --listen     <>   = set listen interval\n");
            printf ("    --rssi       <>   = prints Link Quality (SNR) \n");
            printf ("    --suspstart <time>    = Suspend device for specified time in milliseconds\n");
            printf ("    --gettxstat           = get current status of TX from driver\n");
            printf ("    --pmparams        = Set Power management parameters\n");
            printf ("           --idle <time in ms>                        Idle period\n");
            printf ("           --np   < >                                 PS Poll Number\n");
            printf ("           --dp <1=Ignore 2=Normal 3=Stick 4=Auto>    DTIM Policy\n");
            printf ("           --txwp <1=wake 2=do not wake>              Tx Wakeup Policy\n");
            printf ("           --ntxw < >                                 Number of TX to Wakeup\n");
            printf ("           --psfp <1=send fail event 2=Ignore event>  PS Fail Event Policy\n");
            printf ("    --scanctrl [0|1] [0|1]       = Control firmware scan behavior. Specifically, disable/enable foreground OR background scanning.\n");
        	printf ("    --reg [op] [addr] [value] [mask] = read or write specified AR600X register.\n");
        	printf ("    --assdump = collects assert data from AR600X and prints it to stdout.\n");
        	printf ("    --driver [mode]     \n");
        	printf ("					mode = down - Sets the driver State 0 unloads the driver \n");
        	printf ("					mode = flushdown - Waits for TX clear before unloading the driver \n");
        	printf ("					       up   - Reloads the driver\n");
        	printf ("					       raw  - Reloads the driver in raw mode\n");
        	printf ("					       quad - Reloads the driver in quad spi flash mode\n");
        	printf ("					       rawq - Reloads the driver in raw + quad mode\n");
        	printf ("					       normal - Reloads the driver in normal mode\n");
        	printf ("					       pmu - Reloads the driver in pmu enabled mode (Only MQX)\n");
        	printf ("    --allow_aggr <tx_tid_mask> <rx_tid_mask> Enables aggregation based on the provided bit mask where\n");
        	printf ("		    each bit represents a TID valid TID's are 0-7\n");
        	printf ("    --macstore <addr> = program new mac address into wifi device.\n");
        	printf ("iwconfig scan <ssid*>  =scan channels, ssid is optional, if provided will scan only specified SSID\n");
#if ENABLE_AP_MODE
            printf ("    --mode [ap <hidden><wps>|station]\n");
            printf ("    --ap [country <country code>]\n"
                    "    --ap [bconint<intvl>]\n"
                        "    --ap [dtim<val>]\n"
                            "    --ap [inact <minutes>]> \n"
                                "    --ap [psbuf <1/0-enable/disable> <buf_count for enable>]>\n");
#endif
#if ENABLE_STACK_OFFLOAD
            printf ("    --ipconfig                                     = show IP parameters\n");
            printf ("    --ipstatic <ip addr> <subnet mask> <Gateway>   = set static IP parameters\n");
            printf ("    --ipdhcp                                       = Run DHCP client\n");
            printf ("    --udpecho <port>                               = run UDP echo server\n");
            printf ("    --ipdhcppool <start ipaddr> <End ipaddr> <Lease time> = set ip dhcp pool \n");
            printf ("    ----ip6rtprfx <prefix> <prefixlen> <prefix_lifetime> <valid_lifetime> = set ipv6 router prefix\n");
            printf ("    ----tcp_backoff_retry <no of retries> = set ip tcp max exponetial backoff retries\n");
            printf ("    ----ipv6 <status> = set ip6 status as either enable or disable\n");
            printf ("    ----ipdhcp_release = Release the DHCP Address\n");
            printf ("    ----tcp_rxbuf = Configure the tcp rx buf space\n");
#if ENABLE_DNS_CLIENT
            printf ("    --ip_resolve_hostname [<host name> <domain_type]  = resolve hostname for domain_type (ipv4 =2, ipv6 =3)\n");
            printf ("    --ip_gethostbyname [<host name>]  = resolve hostname\n");
            printf ("    --ip_gethostbyname2 [<host name> <domain_type]  = resolve hostname for domain_type (ipv4 =2 ipv6 =3\n");
            printf ("    --ip_dns_client [<start/stop>]         = Start/Stop the DNS Client\n");
            printf ("    --ip_dns_server_addr <ip addr>         = Address of the DNS Server\n");
            printf ("    --ip_dns_delete_server_addr <ip addr>         = Address of the DNS Server to be deleted \n");
#endif /* ENABLE_DNS_CLIENT */
#if ENABLE_HTTP_SERVER
            printf ("    --ip_http_server [<start/stop> <port>]         = Start/Stop the HTTP server\n");
            printf ("    --ip_http_post <page name>  <obj_name> <obj_type> <obj_len> <obj_value>         = Post/Update the object in HTML page\n");
            printf ("               where <page name>       = page name to update\n");
            printf ("                     <obj_name>        = object name to update\n");
            printf ("                     <obj_type>        = <1-Bool 2-Integer 3-String>\n");
            printf ("                     <obj_len>         = Length of object<1-Bool 4-Integer Length-String>\n");
            printf ("                     <obj_value>       = Object value\n");
            printf ("    --ip_http_get <page name>  <obj_name>          = Get the object in HTML page\n");
            printf ("               where <page name>       = page name to get\n");
            printf ("                     <obj_name>        = object name to get\n");
#endif
#if ENABLE_HTTP_CLIENT
            printf ("    --ip_http_client [<connect/get/post/query/disc> <data1> <data2>]\n");
            printf ("              where <connect> - Used to connect to HTTP server\n");
            printf ("                              <data1> - Hostname or IPaddress of http server\n");
            printf ("                              <data2> - port no of http server(optional)\n");
            printf ("              where <get>     - Used to get a page from HTTP server\n");
            printf ("                              <data1> - Page name to retrieve\n");
            printf ("              where <post>    - Used to post to HTTP server\n");
            printf ("                              <data1> - URL to post\n");
            printf ("              where <query>   - Used to update a variable for get/post\n");
            printf ("                              <data1> - Name of variable\n");
            printf ("                              <data2> - value of variable\n");
            printf ("              where <disc>    - Used to disconnect from HTTP server\n");
#endif

            printf ("    --iphostname [<domain name>]  = configures local host name \n");
#if ENABLE_DNS_SERVER
            printf ("    --ip_dns_local_domain [<domain name>]  = configures local domain \n");
            printf ("    --ip_dns <add/delete> <Host name> <addr>  = configures the domain name and its corresponding address \n");
#endif
#if ENABLE_SNTP_CLIENT
            printf ("    --ip_sntp_srvr <add/delete> <Host name>  = configures the host name or the IP address of the sntp server\n");
            printf ("    --ip_sntp_get_time  = Command to Display the UTC Time \n");
	    printf ("    --ip_sntp_get_time_of_day  = Command to Display the seconds \n");
	    printf ("    --ip_sntp_zone [<UTC+hour:min>] [DSE] [enable/disable] = \n Command to specify time zone and DSE(day light saving enable/disable)Eg:UTC+05:30 dse enable/disable \n");
	    printf ("    --ip_show_sntpconfig = Displays the SNTP server address name \n");
            printf ("    --ip_sntp_client [<start/stop>]         = Start/Stop the SNTP Client\n");

#endif
#if ENABLE_SSL
            printf ("    --ssl_start server|client = start SSL as either server or client\n");
            printf ("    --ssl_stop server|client = stop SSL server or client\n");
            printf ("    --ssl_config server|client [protocol <protocol>] [time 0|1] [domain 0|<name>] [cipher <cipher>] = configure SSL server or client\n");
            printf ("        where protocol <protocol> = select protocol: SSL3, TLS1.0, TLS1.1, TLS1.2 \n");
            printf ("              time 0|1            = disable|enable certificate time validation (optional)\n");
            printf ("              domain 0|<name>     = disable|enable validation of peer's domain name against <name>\n");
            printf ("              alert 0|1           = disable|enable sending of SSL alert if certificate validation fails.\n");
            printf ("              cipher <cipher>     = select <cipher> (enum name) suite to use, can be repeated 8 times (optional)\n");
            printf ("    --ssl_add_cert server|client  certificate|calist [<name>] = add a certificate or CA list to either SSL server or client. \n");
            printf ("        where <name> = name of file to load from FLASH. A default certificate or CA list will be added if <name> is omitted.\n");
            printf ("    --ssl_store_cert <name> = store a certificate or CA list in FLASH with <name>\n");
            printf ("    --ssl_delete_cert [<name>] = delete the certificate or CA list with <name> from FLASH. \n");
            printf ("    --ssl_list_cert  = list the names of the certificates and CA lists stored in FLASH. \n");
#endif
#endif /* ENABLE_STACK_OFFLOAD */
        }
    }

    return return_code;
} /* Endbody */




/*FUNCTION*-----------------------------------------------------------------
*
* Function Name  : wifiCallback
* Returned Value : N/A
* Comments       : Called from driver on a WiFI connection event
*
*END------------------------------------------------------------------*/
void wifiCallback(int val)
{

	wifi_state = val;
	if(val == A_TRUE)
	{
		wifi_connected_flag = 1;
		printf("Connected\n");
                TURN_LED_ON;
	}
	else if(val == INVALID_PROFILE) // this event is used to indicate RSNA failure
	{
		printf("4 way handshake failure \n");
	}
	else if(val == PEER_FIRST_NODE_JOIN_EVENT) //this event is used to RSNA success
	{
		printf("4 way handshake success \n");
        if(wifi_connected_flag == 0)
           wifi_connected_flag = 1;
	}
	else if(val == A_FALSE)
	{
		wifi_connected_flag = 0;
		printf("Not Connected\n");
                TURN_LED_OFF;
	}
	else
	{
		printf("last tx rate : %d kbps\n", val);
	}
}



/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : ascii_to_hex()
* Returned Value  : hex counterpart for ascii
* Comments	: Converts ascii character to correesponding hex value
*
*END*-----------------------------------------------------------------*/
static unsigned char ascii_to_hex(char val)
{
	if('0' <= val && '9' >= val){
		return (unsigned char)(val - '0');
	}else if('a' <= val && 'f' >= val){
		return (unsigned char)(val - 'a') + 0x0a;
	}else if('A' <= val && 'F' >= val){
		return (unsigned char)(val - 'A') + 0x0a;
	}

	return 0xff;/* error */
}


/*FUNCTION*-----------------------------------------------------------------
*
* Function Name  : handle_pmk
* Returned Value : A_ERROR on error else A_OK
* Comments       :
*
*END------------------------------------------------------------------*/

int handle_pmk(char* pmk)
{
    int j;
    A_UINT8 val = 0;
    WMI_SET_PMK_CMD cmd;
    ATH_IOCTL_PARAM_STRUCT  inout_param;
    A_INT32 error;


    memset(cmd.pmk, 0, 32);
    cmd.pmk_len = WMI_PMK_LEN;

    for(j=0;j<64;j++)
    {
        val = ascii_to_hex(pmk[j]);
        if(val == 0xff)
        {
                printf("Invalid character\n");
                return A_ERROR;
        }
        else
        {
                if((j&1) == 0)
                        val <<= 4;

                cmd.pmk[j>>1] |= val;
        }
    }

    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    inout_param.cmd_id = ATH_SET_PMK;
    inout_param.data = &cmd;
    inout_param.length = sizeof(cmd);
    error = HANDLE_IOCTL(&inout_param);

    return error;
}

/*FUNCTION*-----------------------------------------------------------------
*
* Function Name  : set_passphrase
* Returned Value : A_OK
* Comments       : Store WPA Passphrase for later use
*
*END------------------------------------------------------------------*/
static A_INT32 set_passphrase(char* passphrase)
{
	strcpy(wpa_passphrase,passphrase);
	return A_OK;
}

/*FUNCTION*-----------------------------------------------------------------
*
* Function Name  : set_wep_key
* Returned Value : A_ERROR on error else A_OK
* Comments       : Store WEP key for later use. Size of Key must be 10 or 26
*                  Hex characters
*
*END------------------------------------------------------------------*/
static A_INT32 set_wep_key(char* key_index, char* key_val)
{
	int key_idx = atoi(key_index), i;

	if(strlen(key_val) != MIN_WEP_KEY_SIZE && strlen(key_val) != MAX_WEP_KEY_SIZE &&
	    strlen(key_val) != (MIN_WEP_KEY_SIZE/2) && strlen(key_val) != (MAX_WEP_KEY_SIZE/2))
	{
		printf("Invalid WEP Key length, only 10 or 26 HEX characters allowed (or) 5 or 13 ascii keys allowed\n\n");
		return A_ERROR;
	}

	if(key_idx < 1 || key_idx > 4)
    {
    	printf("Invalid key index, Please enter between 1-4\n\n");
    	return A_ERROR;
    }

	if((strlen(key_val) == MIN_WEP_KEY_SIZE) || (strlen(key_val) == MAX_WEP_KEY_SIZE))
	{
		for (i = 0; i < strlen(key_val); i++)
		{
			if(ishexdigit(key_val[i]))
			{
				continue;
			}
			else
			{
    			printf("for hex enter [0-9] or [A-F]\n\n");
    			return A_ERROR;
			}
		}
	}

	key[key_idx-1].key_index = 1;		//set flag to indicate occupancy

	memcpy(key[key_idx-1].key,key_val,strlen(key_val));

	return A_OK;
}



/*FUNCTION*-----------------------------------------------------------------
*
* Function Name  : clear_wep_keys
* Returned Value : Success
* Comments       : Clear wep keys
*
*END------------------------------------------------------------------*/
A_INT32 clear_wep_keys()
{
	int i;
	for(i=0;i<4;i++)
	{
		key[i].key_index = 0;

		key[i].key[0] = '\0';
	}
        return A_OK;
}






/*FUNCTION*-----------------------------------------------------------------
*
* Function Name  : set_callback
* Returned Value : A_ERROR on error else A_OK
* Comments       : Sets callback function for WiFi connect/disconnect event
*
*END------------------------------------------------------------------*/
A_INT32 set_callback()
{
    ATH_IOCTL_PARAM_STRUCT  inout_param;
    A_INT32 error;

    inout_param.cmd_id = ATH_SET_CONNECT_STATE_CALLBACK;
    inout_param.data = (void*)wifiCallback;
    inout_param.length = 4;
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    error = HANDLE_IOCTL(&inout_param);

    return error;
}





/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : connect_handler()
* Returned Value  : A_OK - successful completion or
*					A_ERROR - failed.
* Comments		  : Handles Connect commands for infrastructure mode, Open
*                   WEP,WPA/WPA2 security is supported
*
*END*-----------------------------------------------------------------*/
A_INT32 connect_handler( A_INT32 index, A_INT32 argc, char* argv[])
{
    A_INT32 error= A_OK, i;
    char ssid_str[64];
    int offset = 0, ssidLength=MAX_SSID_LENGTH ;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    set_callback();
#if ENABLE_AP_MODE
	if (mode_flag == MODE_AP)
	{
		printf("setting to ap mode \n");
    	ATH_SET_MODE (DEMOCFG_DEFAULT_DEVICE,"softap");
        /* Set to MAX PERF Mode */
        SET_POWER_MODE("0");
    	if (hidden_flag == 1)
    	{
			hidden_flag = 1;
			ap_config_param.u.wmi_ap_hidden_ssid_cmd.hidden_ssid = 1;
			if (set_ap_params(AP_SUB_CMD_HIDDEN_FLAG, (A_UINT8 *)&ap_config_param.u.wmi_ap_hidden_ssid_cmd.hidden_ssid) != SHELL_EXIT_SUCCESS)
			{
				printf("not able to set hidden mode for AP \n");
				return A_ERROR;
			}
		}
        if (wps_flag == 1)
        {
            if (set_ap_params (AP_SUB_CMD_WPS_FLAG, (A_UINT8 *)&wps_flag) != SHELL_EXIT_SUCCESS)
            {
                printf("not able to set wps mode for AP \n");
                return SHELL_EXIT_ERROR;
            }
            if (SEC_MODE_OPEN == security_mode)
		 	{
				printf("AP in OPEN mode!\n");
			}
        }
	}
	else
#endif
    {
    	ATH_SET_MODE (DEMOCFG_DEFAULT_DEVICE,"managed");
	}
    if(argc > 2)
    {
    	for(i=index;i<argc;i++)
    	{
    		if((ssidLength < 0) || (strlen(argv[i]) > ssidLength))
			{
				printf("SSID length exceeds Maximum value\n");
				return A_ERROR;
			}
    		strcpy(&ssid_str[offset], argv[i]);
    		offset += strlen(argv[i]);
    		ssidLength -= strlen(argv[i]);

    		/*Add empty space in ssid except for last word*/
    		if(i != argc-1)
    		{
    			strcpy(&ssid_str[offset], " ");
    			offset += 1;
    			ssidLength -= 1;
    		}
    	}
    }
    else
    {
    	printf("Missing ssid\n");
    	return A_ERROR;
    }

    error = ATH_SET_ESSID (DEMOCFG_DEFAULT_DEVICE,ssid_str/*argv[index]*/);
    if (error != 0)
    {
        printf ("Error during setting of ssid %s error=%08x!\n",argv[index], error);
        return A_ERROR;
    }

    printf ("\nSetting SSID to %s \n\n",ssid_str/*argv[index]*/);
    strcpy((char*)original_ssid,ssid_str /*argv[index]*/);

	if(SEC_MODE_WPA == security_mode)
	{
		error = set_cipher(DEMOCFG_DEFAULT_DEVICE,(char *)&cipher);
		if(error != A_OK)
		{
			return A_ERROR;
		}
		error = ATH_SET_SEC_TYPE (DEMOCFG_DEFAULT_DEVICE,wpa_ver);
		if(error != A_OK)
		{
			return A_ERROR;
		}
		i++;
		if(strlen(wpa_passphrase) != 64)
		{
			error = ATH_SET_PASSPHRASE (DEMOCFG_DEFAULT_DEVICE,wpa_passphrase);
			if(error != A_OK)
			{
				return A_ERROR;
			}
			pmk_flag = 1;
		}
		else
		{
			if(handle_pmk(wpa_passphrase)!= A_OK)
			{
				printf("Unable to set pmk\n");
				return A_ERROR;
			}
			pmk_flag = 1;
		}
		security_mode = SEC_MODE_OPEN;
	}

    error = ATH_COMMIT(DEMOCFG_DEFAULT_DEVICE);
    if(error != A_OK)
    {
#if ENABLE_AP_MODE
		if(mode_flag == MODE_AP)
		{
			printf("AP supports only AES and OPEN mode \n");
			// Clear Invalid configurations
			error = ATH_SET_SEC_TYPE (DEMOCFG_DEFAULT_DEVICE,"none");
			if(error != A_OK)
			{
				printf("Unable to clear Sec mode\n");
				return A_ERROR;
			}

			security_mode = SEC_MODE_OPEN;
			pmk_flag = 0;
		}
#endif
    	return A_ERROR;
    }
#if ENABLE_AP_MODE
    if (mode_flag == MODE_AP)
	{
		/* setting the AP's default IP to 192.168.1.1 */
#if ENABLE_STACK_OFFLOAD
		//A_UINT32 ip_addr = 0xc0a80101,mask = 0xffffff00,gateway = 0xc0a80101;
		//t_ipconfig((void*)handle,IPCFG_STATIC, &ip_addr, &mask, &gateway);
#else

#define AP_IPADDR  IPADDR(192,168,1,1)
#define AP_IPMASK  IPADDR(255,255,255,0)
#define AP_IPGATEWAY  IPADDR(192,168,1,1)

                A_UINT32 error;
                IPCFG_IP_ADDRESS_DATA   ip_data;

                ip_data.ip = AP_IPADDR;
                ip_data.mask = AP_IPMASK;
                ip_data.gateway = AP_IPGATEWAY;

                error = ipcfg_bind_staticip (DEMOCFG_DEFAULT_DEVICE, &ip_data);
                if (error != 0)
                {
                    printf ("Error during static ip bind %08x!\n", error);
                    return SHELL_EXIT_ERROR;
                }
#endif
	}
#endif

    return A_OK;
}







/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : ad_hoc_connect_handler()
* Returned Value  : 1 - successful completion or
*					0 - failed.
* Comments		  : Handles Connect commands for ad-hoc mode, Open
*                   & WEP security is supported
*
*END*-----------------------------------------------------------------*/

static A_INT32 ad_hoc_connect_handler( A_INT32 index, A_INT32 argc, char* argv[])
{
	A_INT32 error= A_OK,i;
    int key_idx = 0;
    char* key0,*key1,*key2,*key3;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    set_callback();
    ATH_SET_MODE (DEMOCFG_DEFAULT_DEVICE,"adhoc");
    error = ATH_SET_ESSID (DEMOCFG_DEFAULT_DEVICE,argv[index]);
    if (error != 0)
    {
        printf ("Error during setting of ssid %s error=%08x!\n",argv[index], error);
        return A_ERROR;
    }

    printf ("Setting SSID to %s \n\n",argv[index]);
	strcpy((char*)original_ssid,argv[index]);

	if(index+1 < argc)
	{
		i = index +1;
		if(ATH_STRCMP(argv[i],"--wep") == 0)
		{
		    if( argc < 4)  //User did not enter parameter
		    {
		    	printf("Missing parameters\n");
		    	return A_ERROR;
		    }

		    i++;
		    key_idx = atoi(argv[i]);

		    if(key_idx < 1 || key_idx > 4)
		    {
		    	printf("Invalid default key index, Please enter between 1-4\n");
		    	return A_ERROR;
		    }
		    else
		    {

		    }

		    if(key[key_idx-1].key_index != 1)
		    {
		    	printf("This key is not present, please enter WEP key first\n");
		    	return A_ERROR;
		    }

		    error = ATH_SET_SEC_TYPE (DEMOCFG_DEFAULT_DEVICE,"wep");
		    if(error != A_OK)
		    {
		    	return A_ERROR;
		    }

		    if(key[0].key[0] == '\0')
		        key0 = NULL;
		    else
		    	key0 = key[0].key;

		    if(key[1].key[0] == '\0')
		        key1 = NULL;
		    else
		    	key1 = key[1].key;

		    if(key[2].key[0] == '\0')
		        key2 = NULL;
		    else
		    	key2 = key[2].key;

		    if(key[3].key[0] == '\0')
		        key3 = NULL;
		    else
		    	key3 = key[3].key;

		    error = set_wep_keys (DEMOCFG_DEFAULT_DEVICE,
	  							key0,
						      	key1,//argv[i],
						      	key2,//argv[i],
						      	key3,//argv[i],
						      	strlen(key[key_idx-1].key),
						      	key_idx);
			if(error != A_OK)
		    {
		    	return A_ERROR;
		    }
		    clear_wep_keys();

		    if(argc > 5)
            {
              i++;
              if(ATH_STRCMP(argv[i],"--channel") == 0)
              {
                  if( argc < 7)  //User did not enter parameter
                  {
                      printf("Missing Channel parameter\n");
                      return A_ERROR;
                  }
                  i++;
                  printf("Setting Channel to %d\n",atoi(argv[i]));
                  if(set_channel(argv[i]) == A_ERROR)
                    return A_ERROR;
              }
            }
		}
		else if(ATH_STRCMP(argv[i],"--channel") == 0)
		{
		    if( argc < 4)  //User did not enter parameter
			{
				printf("Missing Channel parameter\n");
				return A_ERROR;
			}
			i++;
		    printf("Setting Channel to %d\n",atoi(argv[i]));
		    if(set_channel(argv[i]) == A_ERROR)
		      return A_ERROR;
		}

	}
    error = ATH_COMMIT(DEMOCFG_DEFAULT_DEVICE);
    if(error != A_OK)
    {
    	return A_ERROR;
    }

    return A_OK;
}


#if ENABLE_P2P_MODE


/*FUNCTION*-----------------------------------------------------------------
*
* Function Name  : P2PCallbackfn
* Returned Value : N/A
* Comments       : Called from driver on a WiFI connection event
*
*END------------------------------------------------------------------*/
void P2PCallBackfn(A_UINT8 *ptr)
{
   A_UINT8 *local_ptr2 = ptr;
   WMI_EVT_TO_APP *local_ptr = NULL;
   union {
       WMI_SET_PASSPHRASE_CMD          setPassPhrase;
       WMI_P2P_GRP_INIT_CMD            grpInit;
   }stkP2p;
   WMI_P2P_SET_CMD                 p2p_set_params;
   ATH_IOCTL_PARAM_STRUCT  inout_param;
   ATH_WPS_START wps_start;
   WMI_P2P_GO_NEG_RESULT_EVENT *p2pNeg;
   uint_32 error,chnl, loop_index;
   
   local_ptr = (WMI_EVT_TO_APP *)ptr;
   if (((WMI_EVT_TO_APP *)ptr)->EVENT_ID == WMI_P2P_PROV_DISC_REQ_EVENTID)
   {
          local_ptr2 += sizeof(A_UINT32); 
          
          A_UINT16 wps_method;

            printf("\n source addr : %x:%x:%x:%x:%x:%x \n",    ((WMI_P2P_PROV_DISC_REQ_EVENT *)local_ptr2)->sa[0],
            ((WMI_P2P_PROV_DISC_REQ_EVENT *)local_ptr2)->sa[1],
            ((WMI_P2P_PROV_DISC_REQ_EVENT *)local_ptr2)->sa[2],
            ((WMI_P2P_PROV_DISC_REQ_EVENT *)local_ptr2)->sa[3],
            ((WMI_P2P_PROV_DISC_REQ_EVENT *)local_ptr2)->sa[4],
            ((WMI_P2P_PROV_DISC_REQ_EVENT *)local_ptr2)->sa[5]);

            printf("\n wps_config_method : %x \n",             A_LE2CPU16(((WMI_P2P_PROV_DISC_REQ_EVENT *)local_ptr2)->wps_config_method));

            wps_method = A_LE2CPU16(((WMI_P2P_PROV_DISC_REQ_EVENT *)local_ptr2)->wps_config_method);

            if(WPS_CONFIG_DISPLAY == wps_method)
            {
                printf("Provisional Disc Request - Display WPS PIN [%s] \n",p2p_wps_pin);  
              
            }else if(WPS_CONFIG_KEYPAD == wps_method)
            {
                printf("Provisional Disc Request - Enter WPS PIN \n");  
            }else if(WPS_CONFIG_PUSHBUTTON == wps_method)
            {
                printf("Provisional Disc Request - Push Button \n");  
            }else{
                printf("Invalid Provisional Request \n");
            }
   }
   else if (((WMI_EVT_TO_APP *)ptr)->EVENT_ID == WMI_P2P_REQ_TO_AUTH_EVENTID)
   {
	  local_ptr2 += sizeof(A_UINT32);
          printf("\n source addr : %x:%x:%x:%x:%x:%x \n",    ((WMI_P2P_REQ_TO_AUTH_EVENT *)local_ptr2)->sa[0],
                                                       ((WMI_P2P_REQ_TO_AUTH_EVENT *)local_ptr2)->sa[1],
                                                       ((WMI_P2P_REQ_TO_AUTH_EVENT *)local_ptr2)->sa[2],
                                                       ((WMI_P2P_REQ_TO_AUTH_EVENT *)local_ptr2)->sa[3],
                                                       ((WMI_P2P_REQ_TO_AUTH_EVENT *)local_ptr2)->sa[4],
                                                       ((WMI_P2P_REQ_TO_AUTH_EVENT *)local_ptr2)->sa[5]);

         printf("\n dev_password_id : %x \n",               ((WMI_P2P_REQ_TO_AUTH_EVENT *)local_ptr2)->dev_password_id);
   }
   else if (((WMI_EVT_TO_APP *)ptr)->EVENT_ID == WMI_P2P_LIST_PERSISTENT_NETWORK_EVENTID)
   {
         local_ptr2 += sizeof(A_UINT32);
         A_MEMZERO(p2p_pers_data, (MAX_LIST_COUNT * sizeof(WMI_PERSISTENT_MAC_LIST)));
         memcpy(p2p_pers_data, local_ptr2, (MAX_LIST_COUNT * sizeof(WMI_PERSISTENT_MAC_LIST)));
         printf("\n");
         if(p2p_cancel_enable == 0)
         {
           for(loop_index = 0; loop_index < MAX_LIST_COUNT; loop_index++)
           {
             
             printf("mac_addr[%d] : %x:%x:%x:%x:%x:%x  \n", loop_index, ((WMI_PERSISTENT_MAC_LIST *)(local_ptr2))->macaddr[0],
                    ((WMI_PERSISTENT_MAC_LIST *)(local_ptr2))->macaddr[1],
                    ((WMI_PERSISTENT_MAC_LIST *)(local_ptr2))->macaddr[2],
                    ((WMI_PERSISTENT_MAC_LIST *)(local_ptr2))->macaddr[3],
                    ((WMI_PERSISTENT_MAC_LIST *)(local_ptr2))->macaddr[4],
                    ((WMI_PERSISTENT_MAC_LIST *)(local_ptr2))->macaddr[5]);
             printf("ssid[%d] : %s \n", loop_index, ((WMI_PERSISTENT_MAC_LIST *)(local_ptr2))->ssid);
             //printf("role[%d] : %s \n", loop_index, ((WMI_PERSISTENT_MAC_LIST *)(local_ptr2))->role);
             local_ptr2 += sizeof(WMI_PERSISTENT_MAC_LIST);
           }
         }
         else
         {
           p2p_cancel_enable = 0;
         }
       
   }
   else if (((WMI_EVT_TO_APP *)ptr)->EVENT_ID == WMI_P2P_SDPD_RX_EVENTID)
   {
        local_ptr2 += sizeof(A_UINT32);
     	printf("Custom_Api_p2p_serv_disc_req event \n");
	printf("type : %d   frag id : %x \n", ((WMI_P2P_SDPD_RX_EVENT *)local_ptr2)->type, ((WMI_P2P_SDPD_RX_EVENT *)local_ptr2)->frag_id);
	printf("transaction_status : %x \n", ((WMI_P2P_SDPD_RX_EVENT *)local_ptr2)->transaction_status);
	printf("freq : %d status_code : %d comeback_delay : %d tlv_length : %d update_indic : %d \n",
												  A_LE2CPU16(((WMI_P2P_SDPD_RX_EVENT *)local_ptr2)->freq),
												  A_LE2CPU16(((WMI_P2P_SDPD_RX_EVENT *)local_ptr2)->status_code),
												  A_LE2CPU16(((WMI_P2P_SDPD_RX_EVENT *)local_ptr2)->comeback_delay),
												  A_LE2CPU16(((WMI_P2P_SDPD_RX_EVENT *)local_ptr2)->tlv_length),
												  A_LE2CPU16(((WMI_P2P_SDPD_RX_EVENT *)local_ptr2)->update_indic));
	printf("source addr : %x:%x:%x:%x:%x:%x \n", ((WMI_P2P_SDPD_RX_EVENT *)local_ptr2)->peer_addr[0],
												 ((WMI_P2P_SDPD_RX_EVENT *)local_ptr2)->peer_addr[1],
												 ((WMI_P2P_SDPD_RX_EVENT *)local_ptr2)->peer_addr[2],
												 ((WMI_P2P_SDPD_RX_EVENT *)local_ptr2)->peer_addr[3],
												 ((WMI_P2P_SDPD_RX_EVENT *)local_ptr2)->peer_addr[4],
												 ((WMI_P2P_SDPD_RX_EVENT *)local_ptr2)->peer_addr[5]);
   }   
   else if (((WMI_EVT_TO_APP *)ptr)->EVENT_ID == WMI_P2P_PROV_DISC_RESP_EVENTID)
   {   
            local_ptr2 += sizeof(A_UINT32);
            printf("\n peer addr : %x:%x:%x:%x:%x:%x \n",      ((WMI_P2P_PROV_DISC_RESP_EVENT *)local_ptr2)->peer[0], ((WMI_P2P_PROV_DISC_RESP_EVENT *)local_ptr2)->peer[1],
                                                               ((WMI_P2P_PROV_DISC_RESP_EVENT *)local_ptr2)->peer[2], ((WMI_P2P_PROV_DISC_RESP_EVENT *)local_ptr2)->peer[3],
                                                               ((WMI_P2P_PROV_DISC_RESP_EVENT *)local_ptr2)->peer[4], ((WMI_P2P_PROV_DISC_RESP_EVENT *)local_ptr2)->peer[5]);
            
            if(WPS_CONFIG_KEYPAD == A_LE2CPU16(((WMI_P2P_PROV_DISC_RESP_EVENT *)local_ptr2)->config_methods))
            {
                printf("Provisional Disc Response Keypad - WPS PIN [%s] \n",p2p_wps_pin);  
            }else if(WPS_CONFIG_DISPLAY == A_LE2CPU16(((WMI_P2P_PROV_DISC_RESP_EVENT *)local_ptr2)->config_methods))
            {
                printf("Provisional Disc Response Display \n");  
            }else if(WPS_CONFIG_PUSHBUTTON == A_LE2CPU16(((WMI_P2P_PROV_DISC_RESP_EVENT *)local_ptr2)->config_methods))
            {
                printf("Provisional Disc Response Push Button  \n");  
            }else{
                printf("Invalid Provisional Response \n");
            }
            //printf("\n config_methods : %x \n",                A_LE2CPU16(((WMI_P2P_PROV_DISC_RESP_EVENT *)local_ptr2)->config_methods));  
   }
   else if (((WMI_EVT_TO_APP *)ptr)->EVENT_ID == WMI_P2P_NODE_LIST_EVENTID)
   {
          local_ptr2 += sizeof(A_UINT32);
          A_UINT8  index, temp_val, *temp_ptr = NULL;
          A_UINT16 config_method;
          //temp_val = ((WMI_P2P_NODE_LIST_EVENT *)(local_ptr2))->num_p2p_dev;
          temp_val = *local_ptr2;
          local_ptr2 ++;
          
          temp_ptr = local_ptr2;
          //local_ptr2 += sizeof(A_UINT8);
    if (temp_val > 0)
    {
        for (index = 0; index < temp_val; index++)
        {
            printf("\n \t p2p_device_name       : %s \n",                        ((struct p2p_device_lite *)(local_ptr2))->device_name);
            //printf("\t p2p_primary_dev_type  : %s \n",                        ((struct p2p_device *)(local_ptr2))->pri_dev_type);
            printf("\t p2p_interface_addr    : %x:%x:%x:%x:%x:%x \n",         ((struct p2p_device_lite *)(local_ptr2))->interface_addr[0],
                    ((struct p2p_device_lite *)(local_ptr2))->interface_addr[1],
                    ((struct p2p_device_lite *)(local_ptr2))->interface_addr[2],
                    ((struct p2p_device_lite *)(local_ptr2))->interface_addr[3],
                    ((struct p2p_device_lite *)(local_ptr2))->interface_addr[4],
                    ((struct p2p_device_lite *)(local_ptr2))->interface_addr[5]);
            printf("\t p2p_device_addr       : %x:%x:%x:%x:%x:%x \n",         ((struct p2p_device_lite *)(local_ptr2))->p2p_device_addr[0],
                    ((struct p2p_device_lite *)(local_ptr2))->p2p_device_addr[1],
                    ((struct p2p_device_lite *)(local_ptr2))->p2p_device_addr[2],
                    ((struct p2p_device_lite *)(local_ptr2))->p2p_device_addr[3],
                    ((struct p2p_device_lite *)(local_ptr2))->p2p_device_addr[4],
                    ((struct p2p_device_lite *)(local_ptr2))->p2p_device_addr[5]);
            printf("\t p2p_device_capability : %x \n",                        ((struct p2p_device_lite *)(local_ptr2))->dev_capab);
            printf("\t p2p_group_capability  : %x \n",                        ((struct p2p_device_lite *)(local_ptr2))->group_capab);          
             //printf("\t p2p_wps_method        : %x \n",                        ((struct p2p_device *)(local_ptr2))->wps_method);
            //printf("\t Peer Listen channel   : %d \n",                        A_LE2CPU16(((struct p2p_device *)(local_ptr2->dataptr))->listen_freq));
            //printf("\t Peer Oper   channel   : %d \n",                        A_LE2CPU16(((struct p2p_device *)(local_ptr2->dataptr))->oper_freq)); 
            //printf("\t p2p_config_method     : %x \n",                        A_LE2CPU16(((struct p2p_device *)(local_ptr->dataptr))->config_methods)); 
             local_ptr2     += sizeof(struct p2p_device_lite);
        }

    }
    local_ptr2 = temp_ptr;       
   }
    else if (((WMI_EVT_TO_APP *)ptr)->EVENT_ID == WMI_P2P_INVITE_SENT_RESULT_EVENTID)
   {
      local_ptr2 += sizeof(A_UINT32);   
      printf("Invitation Result %d\n",((WMI_P2P_INVITE_SENT_RESULT_EVENT *)(local_ptr2))->status);
    
      printf("SSID %x:%x:%x:%x:%x:%x \n",((WMI_P2P_INVITE_SENT_RESULT_EVENT *)(local_ptr2))->bssid[0], ((WMI_P2P_INVITE_SENT_RESULT_EVENT *)(local_ptr2))->bssid[1],
                                         ((WMI_P2P_INVITE_SENT_RESULT_EVENT *)(local_ptr2))->bssid[2], ((WMI_P2P_INVITE_SENT_RESULT_EVENT *)(local_ptr2))->bssid[3],
                                         ((WMI_P2P_INVITE_SENT_RESULT_EVENT *)(local_ptr2))->bssid[4], ((WMI_P2P_INVITE_SENT_RESULT_EVENT *)(local_ptr2))->bssid[5]);
#if 0      
      A_MEMZERO(&p2p_set_params,sizeof(WMI_P2P_SET_CMD));
      p2p_set_params.config_id = WMI_P2P_CONFID_P2P_OPMODE;
      p2p_set_params.val.mode.p2pmode = P2P_CLIENT;
      if(StartP2P(SET, (A_UINT8 *)&p2p_set_params) == 0)
      {
        printf("\nStartP2P command(p2p_client mode) did not execute properly\n");
        return;
      }
    
      if(StartP2P(INV_CONNECT, (A_UINT8 *)&p2p_pers_data[invitation_index]) == 0)
      {
           printf("\nStartP2P command(invite connect) did not execute properly\n");
           return;
      }
#endif       
   }
   else if (((WMI_EVT_TO_APP *)ptr)->EVENT_ID == WMI_P2P_INVITE_RCVD_RESULT_EVENTID)
   {
        local_ptr2 += sizeof(A_UINT32);
        int i;
        printf("Invite Result Status : %x \n",((WMI_P2P_INVITE_RCVD_RESULT_EVENT  *)(local_ptr2))->status);

        for (i=0;i<ATH_MAC_LEN;i++)
        {
          printf(" [%x] ",((WMI_P2P_INVITE_RCVD_RESULT_EVENT  *)(local_ptr2))->sa[i]);
        }
        printf("\n Start Join Process connect to P2P GO\n");          
     
     
   }
   else if (((WMI_EVT_TO_APP *)ptr)->EVENT_ID == WMI_P2P_INVITE_REQ_EVENTID)
   {
        local_ptr2 += sizeof(A_UINT32); 
        
        int i;
        WMI_P2P_FW_INVITE_REQ_RSP_CMD invite_rsp_cmd;
        
        printf("Invitation Req Received From : ");
        for (i = 0; i < ATH_MAC_LEN; i++)
        {
          printf(" %x: ",((WMI_P2P_FW_INVITE_REQ_EVENT *)(local_ptr2))->go_dev_addr[i]);
          
        }
        printf("\n");
        
        A_MEMZERO(&invite_rsp_cmd,sizeof(WMI_P2P_FW_INVITE_REQ_RSP_CMD));
        
        if (((WMI_P2P_FW_INVITE_REQ_EVENT *)(local_ptr2))->is_persistent)
        {
           for (i = 0; i < MAX_LIST_COUNT; i++)
           {
              if(A_MEMCMP(((WMI_P2P_FW_INVITE_REQ_EVENT *)(local_ptr2))->go_dev_addr, p2p_pers_data[i].macaddr, ATH_MAC_LEN) == 0)
              {
                invite_rsp_cmd.status = 0;
                A_MEMCPY(invite_rsp_cmd.group_bssid, p2p_pers_data[i].macaddr, ATH_MAC_LEN);
                break;
              }
           }
           
           if(i == MAX_LIST_COUNT)
           {
             invite_rsp_cmd.status = 1;
             i = 0;
           }
           
           /* send invite auth event */
           if(StartP2P(P2P_INVITE_AUTH, (A_UINT8 *)&invite_rsp_cmd) == 0)
           {
             printf("\nStartP2P (P2P invite auth persistent)command did not execute properly\n");
             return;
           }
          
           
        }

        
   }else if (((WMI_EVT_TO_APP *)ptr)->EVENT_ID == WMI_P2P_GO_NEG_RESULT_EVENTID)
   {
       
       local_ptr2 += sizeof(A_UINT32); 
       p2pNeg = (WMI_P2P_GO_NEG_RESULT_EVENT *)local_ptr2;
       task2_msec = 100;
       printf("P2P GO Negotion Result :\n");
       printf("Status       :%s \n",(p2pNeg->status) ? "FAILURE":"SUCCESS");
       printf("P2P Role     :%s \n",(p2pNeg->role_go) ? "P2P GO": "P2P Client");
       printf("SSID         :%s \n",p2pNeg->ssid);
       printf("Channel      :%d \n",p2pNeg->freq);
       printf("WPS Method   :%s \n",(p2pNeg->wps_method == WPS_PBC) ? "PBC": "PIN");
       if((p2pNeg->status == 0) &&
               (p2pNeg->role_go == 1)) {
           /* Set Passphrase */
           {
               mode_flag = MODE_AP;
               A_MEMZERO(&stkP2p.setPassPhrase, sizeof(WMI_SET_PASSPHRASE_CMD));

               strcpy((char *)stkP2p.setPassPhrase.passphrase,p2pNeg->pass_phrase);
               stkP2p.setPassPhrase.passphrase_len = strlen(p2pNeg->pass_phrase);

               strcpy((char *)stkP2p.setPassPhrase.ssid,p2pNeg->ssid);
               stkP2p.setPassPhrase.ssid_len = p2pNeg->ssid_len;

               error = ATH_SET_ESSID (DEMOCFG_DEFAULT_DEVICE,(char*)stkP2p.setPassPhrase.ssid);
               if(error != A_OK)
               {
                   printf("Unable to set SSID\n");
                   return;
               }

               cipher.ucipher = 0x08;
               cipher.mcipher = 0x08;
               error = set_cipher(DEMOCFG_DEFAULT_DEVICE,(char *)&cipher);

               if(error != A_OK)
               {
                   return;
               }

               strcpy(wpa_ver,"wpa2");
               error = ATH_SET_SEC_TYPE (DEMOCFG_DEFAULT_DEVICE,wpa_ver);

               if(error != A_OK)
               {
                   return;
               }

               security_mode = SEC_MODE_WPA;

               if (StartP2P(AP_MODE_PP,(A_UINT8 *)&stkP2p.setPassPhrase) == 0)
               {
                   printf("\nStartP2P command did not execute properly\n");
                   return ;
               }
           }
           app_time_delay(1000);
           /* Start P2P GO */
           {
               ATH_SET_MODE (DEMOCFG_DEFAULT_DEVICE,"softap");
               wps_flag = 0x01;
               chnl = p2pNeg->freq;     
               inout_param.cmd_id = ATH_SET_CHANNEL;
               inout_param.data = &chnl;
               inout_param.length = 4;

               error = HANDLE_IOCTL (&inout_param);
               if (A_OK != error)
               {
                   return;
               }

               if (set_ap_params (AP_SUB_CMD_WPS_FLAG, (A_UINT8 *)&wps_flag) != SHELL_EXIT_SUCCESS)
               {
                   printf("not able to set wps mode for AP \n");
                   return;
               }

               A_MEMZERO(&stkP2p.grpInit ,sizeof(WMI_P2P_GRP_INIT_CMD));

               stkP2p.grpInit.group_formation = 1;
               mode_flag = MODE_AP;
               if(StartP2P(AP_MODE,(A_UINT8 *)&stkP2p.grpInit) == 0)
               {
                   printf("\nStartP2P command did not execute properly\n");
                   return;
               }
           }
           app_time_delay(1000);
           /* Set MAX PERF */
           SET_POWER_MODE("0");
       } else if((p2pNeg->status == 0) &&
               (p2pNeg->role_go == 0)) {
       }
       /* do WPS */
       if(p2pNeg->status == 0)
       {
           mode_flag = MODE_STATION;
           A_MEMZERO(&wps_start, sizeof(ATH_WPS_START));
           inout_param.cmd_id = ATH_START_WPS;
           inout_param.data = &wps_start;
           inout_param.length = sizeof(wps_start);
           
           wps_context.connect_flag = (p2pNeg->role_go) ? 0:8;
               
           wps_start.connect_flag = wps_context.connect_flag;

           if(p2pNeg->wps_method != WPS_PBC){
               wps_start.wps_mode = ATH_WPS_MODE_PIN;
              // wps_start.pin_length = strlen (wpsPin);
               wps_start.pin_length = 8;
               //FIXME: This hardcoded pin value needs to be changed
               // for production to reflect what is on a sticker/label
               memcpy (wps_start.pin, p2p_wps_pin, wps_start.pin_length);
               //memcpy (wps_start.pin, wpsPin, wps_start.pin_length+1);
               
           }else{
               wps_start.wps_mode = ATH_WPS_MODE_PUSHBUTTON;
           }
           
           wps_start.timeout_seconds = 30;
           
           memcpy(wps_start.ssid_info.ssid,p2pNeg->ssid,sizeof(p2pNeg->ssid));
           memcpy(wps_start.ssid_info.macaddress,p2pNeg->peer_device_addr,6);
           wps_start.ssid_info.channel = p2pNeg->freq;
           wps_start.ssid_info.ssid_len = p2pNeg->ssid_len;
           
           
           /* this starts WPS on the Aheros wifi */
           error = HANDLE_IOCTL (&inout_param);
           if (A_OK != error){
               return;
           }
           wps_context.wps_in_progress = 1;
       }
   }   
}

/*FUNCTION*-----------------------------------------------------------------
*
* Function Name  : p2p_callback
* Returned Value : A_ERROR on error else A_OK
* Comments       : Sets callback function for WiFi P2P events
*
*END------------------------------------------------------------------*/
A_INT32 p2p_callback(A_UINT8 evt_flag)
{
    ATH_IOCTL_PARAM_STRUCT  inout_param;
    A_INT32 error;
    WMI_EVT_TO_APP p2p_evt;
    if (evt_flag == 1)
    {
      inout_param.cmd_id = ATH_P2P_EVENT_CALLBACK;
    } 
    else
    { 
      inout_param.cmd_id = ATH_SET_P2P_CALLBACK;
    }
    A_MEMZERO(&p2p_evt, sizeof(WMI_EVT_TO_APP));
    inout_param.data = (void*)&p2p_evt;
    inout_param.length = sizeof(WMI_EVT_TO_APP);
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    error = HANDLE_IOCTL(&inout_param);
 
    if (error == A_OK)
    {
      P2PCallBackfn((A_UINT8 *)&p2p_evt);
    }
    
    return error;
}


/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : StartP2P()
* Returned Value  : 1 - successful completion or
*                   0 - failed.
* Comments        : This function inititates various P2P commands with GO.
*                   The caller must specify the type of P2P command type,
*                   channel_scan type and timeout in seconds. If timeout
*                   is not specified default value is used.
*
*
*END*-----------------------------------------------------------------*/
uint_32 StartP2P(A_UINT8 p2p_command,A_UINT8 *p2p_ptr)
{

    uint_32 error,dev_status;
    ATH_IOCTL_PARAM_STRUCT param;
    uint_32 status = 0;

    do{
#if NOT_USED 
        error = ENET_mediactl (handle,ENET_MEDIACTL_IS_INITIALIZED,&dev_status);
        if (ENET_OK != error || dev_status == FALSE)
        {
            printf("\n StartP2P Exit \n");
            break;
        }
#endif

        if(p2p_command == P2P_FIND)
        {
            param.cmd_id = ATH_P2P_FIND;
            param.data   = p2p_ptr;
            param.length = sizeof(WMI_P2P_FIND_CMD);

        }
        else if(p2p_command == CONNECT)
        {
            param.cmd_id = ATH_P2P_CONNECT;
            param.data   = p2p_ptr;
            param.length = sizeof(WMI_P2P_GO_NEG_START_CMD);
        }
        else if(p2p_command == LISTEN)
        {
            param.cmd_id = ATH_P2P_LISTEN;
            param.data   = p2p_ptr;
            param.length = sizeof(A_UINT32);
        }
        else if(p2p_command == CONNECT_CLIENT)
        {
            param.cmd_id = ATH_P2P_CONNECT_CLIENT;
            param.data   = p2p_ptr;
            param.length = sizeof( WMI_P2P_FW_CONNECT_CMD_STRUCT);
        }
        else if(p2p_command == AUTH)
        {
            param.cmd_id = ATH_P2P_AUTH;
            param.data   = p2p_ptr;
            param.length = sizeof( WMI_P2P_FW_CONNECT_CMD_STRUCT);
        }
        else if(p2p_command == CANCEL)
        {
            param.cmd_id = ATH_P2P_CANCEL;
            param.data   = NULL;
            param.length = 0;
        }
        else if(p2p_command == STOP)
        {
            param.cmd_id = ATH_P2P_STOP;
            param.data   = NULL;
            param.length = 0;
        }
        else if(p2p_command == NODE_LIST)
        {
            param.cmd_id = ATH_P2P_NODE_LIST;
            param.data   = NULL;
            param.length = 0;
        }
        else if(p2p_command == SET_CONFIG)
        {
            param.cmd_id = ATH_P2P_SET_CONFIG;
            param.data   = p2p_ptr;
            param.length = sizeof(WMI_P2P_FW_SET_CONFIG_CMD);
        }
        else if(p2p_command == WPS_CONFIG)
        {
            param.cmd_id = ATH_P2P_WPS_CONFIG;
            param.data   = p2p_ptr;
            param.length = sizeof(WMI_WPS_SET_CONFIG_CMD);
        }
        else if(p2p_command == DISC_REQ)
        {
            param.cmd_id = ATH_P2P_DISC_REQ;
            param.data   = p2p_ptr;
            param.length = sizeof(WMI_P2P_FW_PROV_DISC_REQ_CMD);
        }
        else if(p2p_command == SET)
        {
            param.cmd_id = ATH_P2P_SET;
            param.data   = p2p_ptr;
            param.length = sizeof(WMI_P2P_SET_CMD);
        }
        else if(p2p_command == JOIN)
        {
            param.cmd_id = ATH_P2P_JOIN;
            param.data   = p2p_ptr;
            param.length = sizeof( WMI_P2P_FW_CONNECT_CMD_STRUCT);
        }
        else if(p2p_command == INV_CONNECT)
        {
            param.cmd_id = ATH_P2P_INV_CONNECT;
            param.data   = p2p_ptr;
            param.length = sizeof(A_UINT8);
        }
        else if(p2p_command == P2P_INVITE_AUTH)
        {
            param.cmd_id = ATH_P2P_INVITE_AUTH;
            param.data   = p2p_ptr;
            param.length = sizeof(WMI_P2P_FW_INVITE_REQ_RSP_CMD);
        }
        else if(p2p_command == LIST_NETWORK)
        {
            param.cmd_id = ATH_P2P_PERSISTENT_LIST;
            param.data   = NULL;
            param.length = sizeof(WPS_CREDENTIAL);
        }
        else if(p2p_command == P2P_INVITE)
        {
            param.cmd_id = ATH_P2P_INVITE;
            param.data   = p2p_ptr;
            param.length = sizeof(WMI_P2P_INVITE_CMD);
        }
           else if(p2p_command == JOIN_PROFILE)
        {
            param.cmd_id = ATH_P2P_JOIN_PROFILE;
            param.data   = p2p_ptr;
            param.length = sizeof( WMI_P2P_FW_CONNECT_CMD_STRUCT);
        }
            else if(p2p_command == AP_MODE)
        {
            param.cmd_id = ATH_P2P_APMODE;
            param.data   = p2p_ptr;
            param.length = sizeof(WMI_P2P_GRP_INIT_CMD);
        }
            else if(p2p_command == AP_MODE_PP)
        {
            param.cmd_id = ATH_P2P_APMODE_PP;
            param.data   = p2p_ptr;
            param.length = sizeof(WMI_SET_PASSPHRASE_CMD);
        }
            else if(p2p_command == P2P_ON_OFF)
        {
            param.cmd_id = ATH_P2P_SWITCH;
            param.data   = p2p_ptr;
            param.length = sizeof(WMI_P2P_SET_PROFILE_CMD);
        }
         else if(p2p_command == P2P_SET_NOA)
        {
            param.cmd_id = ATH_P2P_SET_NOA;
            param.data   = p2p_ptr;
            param.length = sizeof(WMI_NOA_INFO_STRUCT) - 1 + sizeof(P2P_NOA_DESCRIPTOR);
        }
         else if(p2p_command == P2P_SET_OPPPS)
        {
            param.cmd_id = ATH_P2P_SET_OPPPS;
            param.data   = p2p_ptr;
            param.length = sizeof(WMI_OPPPS_INFO_STRUCT);
        }
        else
        {
            printf("\n wrong p2p command received\n");
            break;
        }

#if NOT_USED 
        /* this starts P2P on the Aheros wifi */
        error = ENET_mediactl (handle,ENET_MEDIACTL_VENDOR_SPECIFIC,&param);
        if (ENET_OK != error)
        {
            printf("\n StartP2P Exit 1\n");
            break;
        }
#else
        error = HANDLE_IOCTL (&param);
#if NOT_USED 
        if (ENET_OK != error)
        {
            printf("\n StartP2P Exit 1\n");
            break;
        }
#endif
#endif

        status = 1;
    }while(0);

    return status;
}
#endif



/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : set_wpa()
* Returned Value  : A_OK - successful completion or
*		    A_ERROR - failed.
* Comments	: Sets WPA mode
*
*END*-----------------------------------------------------------------*/
static A_INT32 set_wpa( A_INT32 index, A_INT32 argc, char* argv[])
{

	int i = index;

	A_INT32 error= A_OK;

	if(5 != argc)  //User did not enter parameter
	{
		printf("Missing parameters\n\n");
		return A_ERROR;
	}

	/********** version ****************/
	if(ATH_STRCMP(argv[i],"1")==0)
	{
		strcpy(wpa_ver,"wpa");
		//this is a hack we currently only support wpa2 AES in AP mode
		if (mode_flag == MODE_AP)
		{
			strcpy(wpa_ver,"wpa2");
		}
	}
	else if(ATH_STRCMP(argv[i],"2")==0)
	{
		strcpy(wpa_ver,"wpa2");
	}
	else
	{
		printf("Invalid version\n");
		return A_ERROR;
	}

	/**************** ucipher **********/
	i++;
	if(ATH_STRCMP(argv[i],"TKIP")==0)
	{
		cipher.ucipher = 0x04;
	}
	else if(ATH_STRCMP(argv[i],"CCMP")==0)
	{
		cipher.ucipher = 0x08;
	}
	else
	{
		printf("Invalid ucipher\n");
		return A_ERROR;
	}


	/*********** mcipher ************/
	i++;
	if(ATH_STRCMP(argv[i],"TKIP")==0)
	{
		cipher.mcipher = 0x04;
	}
	else if(ATH_STRCMP(argv[i],"CCMP")==0)
	{
		cipher.mcipher = 0x08;
	}
	else
	{
		printf("Invalid mcipher\n");
		return A_ERROR;
	}
    security_mode = SEC_MODE_WPA;
    return error;
}


/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : set_wep()
* Returned Value  : A_OK - successful completion or
*		    A_ERROR - failed.
* Comments	: Sets WEP parameters
*
*END*-----------------------------------------------------------------*/
static A_INT32 set_wep( A_INT32 index, A_INT32 argc, char* argv[])
{
	int i = index;
	int key_idx = 0;
	char* key0,*key1,*key2,*key3;
	A_INT32 error= A_OK;

	if( argc < 3)  //User did not enter parameter
	{
		printf("Missing parameters\n");
		return A_ERROR;
	}

	key_idx = atoi(argv[i]);

	if(key_idx < 1 || key_idx > 4)
	{
		printf("Invalid default key index, Please enter between 1-4\n");
		return A_ERROR;
	}
	else
	{

	}

	if(key[key_idx-1].key_index != 1)
	{
		printf("This key is not present, please enter WEP key first\n");
		return A_ERROR;
	}

	error = ATH_SET_SEC_TYPE (DEMOCFG_DEFAULT_DEVICE,"wep");
	if(error != A_OK)
	{
		return A_ERROR;
	}

	if(key[0].key[0] == '\0')
		key0 = NULL;
	else
		key0 = key[0].key;

	if(key[1].key[0] == '\0')
		key1 = NULL;
	else
		key1 = key[1].key;

	if(key[2].key[0] == '\0')
		key2 = NULL;
	else
		key2 = key[2].key;

	if(key[3].key[0] == '\0')
		key3 = NULL;
	else
		key3 = key[3].key;

	error = set_wep_keys (DEMOCFG_DEFAULT_DEVICE,
						key0,
						key1,//argv[i],
						key2,//argv[i],
						key3,//argv[i],
						strlen(key[key_idx-1].key),
						key_idx);
    security_mode = SEC_MODE_WEP;
    clear_wep_keys();

    return error;
}




/*FUNCTION*--------------------------------------------------------------------
*
* Function Name   : set_wep_key()
* Returned Value  : A_ERROR on error else A_OK
* Comments        : Sets the Mode for Wifi Device.
*
*END*------------------------------------------------------------------------*/

A_INT32 set_wep_keys
    (
        A_UINT32 dev_num,
        char* wep_key1,
        char* wep_key2,
        char* wep_key3,
        char* wep_key4,
        A_UINT32 key_len,
        A_UINT32 key_index

    )
{

    A_INT32 error;
    ENET_WEPKEYS wep_param;


        /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

        wep_param.defKeyIndex = key_index;
        wep_param.numKeys = 0;
        wep_param.keyLength = key_len;

        do{
	        if(wep_key1 == NULL){
	        	break;
	        }

	        wep_param.keys[0] = wep_key1;
	        wep_param.numKeys++;

	        if(wep_key2 == NULL){
	        	break;
	        }

	        wep_param.keys[1] = wep_key2;
	        wep_param.numKeys++;

	        if(wep_key3 == NULL){
	        	break;
	        }

	        wep_param.keys[2] = wep_key3;
	        wep_param.numKeys++;

	        if(wep_key4 == NULL){
	        	break;
	        }

	        wep_param.keys[3] = wep_key4;
	        wep_param.numKeys++;

		}while(0);

        error = ATH_SEND_WEP_PARAM(&wep_param);
        return error;
}














/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : set_cipher()
* Returned Value  : A_OK if ESSID set successfully else ERROR CODE
* Comments        : Sets the Unicast and multicast cipher
*
*END*-----------------------------------------------------------------*/

static A_INT32 set_cipher
    (
        A_UINT32 dev_num,
       char* cipher
    )
{
     ATH_IOCTL_PARAM_STRUCT  inout_param;
    A_INT32 error;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }
    inout_param.cmd_id = ATH_SET_CIPHER;
    inout_param.data = cipher;
    inout_param.length = 8;

    error = HANDLE_IOCTL (&inout_param);

    return error;
}





/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : set_sec_mode()
* Returned Value  : A_OK if mode set successfully else ERROR CODE
* Comments        : Sets the mode.
*
*END*-----------------------------------------------------------------*/

static A_INT32 set_sec_mode
    (
        A_UINT32 dev_num,
        char* mode
    )
{
    A_INT32 error;
    ATH_IOCTL_PARAM_STRUCT param;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }
    param.cmd_id = ATH_SET_SEC_MODE;
    param.data = mode;
    param.length = strlen(mode);

    error = HANDLE_IOCTL (&param);

    return error;
}



static A_INT32 disconnect_handler()
{
	A_INT32 error;

    // reset wps flag
    wps_flag    = 0;
    hidden_flag = 0;
    pmk_flag    = 0;
    security_mode = SEC_MODE_OPEN;
    
    error = ATH_SET_ESSID (DEMOCFG_DEFAULT_DEVICE,"");
    if (error != 0)
    {
        //printf ("Error during setting of ssid %s error=%08x!\n",ssid, error);
        return A_ERROR;
    }

    /*Reset stored SSID*/
    memset(original_ssid,'\0',33);

    error = ATH_COMMIT(DEMOCFG_DEFAULT_DEVICE);

    return A_OK;
}


/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : StartWPS()
* Returned Value  : 1 - successful completion or
*					0 - failed.
* Comments		  : Attempts to start a WPS connection with a peer.
*					The caller must specify the type of WPS to start
*					push-button mode | pin mode.  if pin mode is used
*					the caller must specify the pin value to pass to
*					the Atheros driver.
*
*END*-----------------------------------------------------------------*/
A_INT32 StartWPS(ATH_NETPARAMS *pNetparams, A_UINT8 use_pinmode, wps_scan_params* wpsScan_p )
{
#define WPS_STANDARD_TIMEOUT (30)
	A_INT32 error;
    ATH_IOCTL_PARAM_STRUCT param;
    A_UINT32 status = 0;
    ATH_WPS_START wps_start;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }
    do{

        A_MEMZERO(&wps_start, sizeof(ATH_WPS_START));
        param.cmd_id = ATH_START_WPS;
        param.data = &wps_start;
        param.length = sizeof(wps_start);

        wps_start.connect_flag = wps_context.connect_flag;
        if(use_pinmode){
        	wps_start.wps_mode = ATH_WPS_MODE_PIN;
            wps_start.pin_length = strlen (wpsPin);
        	//FIXME: This hardcoded pin value needs to be changed
        	// for production to reflect what is on a sticker/label
            memcpy (wps_start.pin, wpsPin, wps_start.pin_length+1);

        }else{
        	wps_start.wps_mode = ATH_WPS_MODE_PUSHBUTTON;
        }

        wps_start.timeout_seconds = WPS_STANDARD_TIMEOUT;

        if(wpsScan_p == NULL)
        {
        	wps_start.ssid_info.ssid_len = 0;
        }
        else
        {
        	memcpy(wps_start.ssid_info.ssid,wpsScan_p->ssid,sizeof(wpsScan_p->ssid));
        	memcpy(wps_start.ssid_info.macaddress,wpsScan_p->macaddress,6);
        	wps_start.ssid_info.channel = channel_array[wpsScan_p->channel];
        	wps_start.ssid_info.ssid_len = strlen((char *)wpsScan_p->ssid);
        }

        /* this starts WPS on the Aheros wifi */
       error = HANDLE_IOCTL (&param);
       if (A_OK != error){
            break;
       }

       status = 1;
    }while(0);

    return status;
}










/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : wps_query()
* Returned Value  : 0 - success, 1 - failure
* Comments	: Queries WPS status
*
*END*-----------------------------------------------------------------*/
A_INT32 wps_query(A_UINT8 block)
{
	ATH_NETPARAMS netparams;
	A_INT32 status = 1;

    if (wps_context.wps_in_progress){
        if (CompleteWPS(&netparams, block, mode_flag)){
			status = 0;
			wps_context.wps_in_progress = 0;

			if(wps_context.connect_flag){
				if(ATH_NET_CONNECT(&netparams)){
					printf("connection failed\n");
				}
			}
		}
        else
        {
            if (A_EBUSY != netparams.error){
                wps_context.wps_in_progress = 0;
                status = 0;
            }
        }
	}

	return status;
}



static A_INT32 allow_aggr(A_INT32 argc, char* argv[])
{

	ATH_SET_AGGREGATION_PARAM param;
	ATH_IOCTL_PARAM_STRUCT ath_param;

	do{
		if(argc != 2){
			break;
		}

		param.txTIDMask = mystrtoul(argv[0], NULL, 16);
		param.rxTIDMask = mystrtoul(argv[1], NULL, 16);

		if(param.txTIDMask > 0xff || param.rxTIDMask > 0xff){
			break;
		}

		ath_param.cmd_id = ATH_SET_AGGREGATION;
		ath_param.data = &param;
		ath_param.length = sizeof(ATH_SET_AGGREGATION_PARAM);

	    if(A_OK != HANDLE_IOCTL (&ath_param)){
	    	printf("ERROR: driver command failed\n");
	    }

    	return A_OK;
    }while(0);

   	printf ("wmiconfig --allow_aggr <tx_tid_mask> <rx_tid_mask> Enables aggregation based on the provided bit mask where\n");
    printf ("each bit represents a TID valid TID's are 0-7\n");

    return A_OK;
}





/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : test_promiscuous()
* Returned Value  : A_OK - success
* Comments	: Tests promiscuous mode
*
*END*-----------------------------------------------------------------*/
static A_INT32 test_promiscuous(A_INT32 argc, char* argv[])
{
	ATH_IOCTL_PARAM_STRUCT param;
	ATH_PROMISCUOUS_MODE prom_mode_details;
	param.cmd_id = ATH_SET_PROMISCUOUS_MODE;
    param.data = &prom_mode_details;
    param.length = sizeof(prom_mode_details);

    A_MEMZERO(&prom_mode_details, sizeof(ATH_PROMISCUOUS_MODE));

   	prom_mode_details.enable = atoi(argv[0]);
    /* Set to MAX PERF Mode */
    SET_POWER_MODE("0");
   	if(argc > 1){
	   	prom_mode_details.filter_flags = atoi(argv[1]);

	   	if(argc > 2){
		   	if(mystrtomac(argv[2], &(prom_mode_details.src_mac[0])))
		   	{
		   		printf("ERROR: MAC address translation failed.\n");
		   		return A_OK;
		   	}

		   	if(argc > 3){
			   	if(mystrtomac(argv[3], &(prom_mode_details.dst_mac[0])))
			   	{
			   		printf("ERROR: MAC address translation failed.\n");
			   		return A_OK;
			   	}
			}
		}
	}

	prom_mode_details.cb = application_frame_cb;
        /* call the driver */
        if (A_OK != HANDLE_IOCTL (&param))
        {
            printf("ERROR: promiscuous command failed\n");
        }

    return A_OK;
}

/* test_raw -- makes use of special API's to send a raw MAC frame irregardless of connection state
 */
static A_INT32 test_raw(A_INT32 argc, char* argv[])
{
	A_UINT8 rate_index = 0;
	A_UINT8 access_cat;
	A_UINT8 tries = 1;
	A_UINT8 header_type;
	A_INT32 i,error;
	A_UINT32 chan, size;
    ATH_IOCTL_PARAM_STRUCT param;
    A_UINT32 status = 0;
    ATH_MAC_TX_RAW_S details;
    A_UINT8 *buffer = NULL;
    typedef struct{
    	A_UINT8 fc[2];
    	A_UINT8 duration[2];
    	A_UINT8 addr[3][6];
    	A_UINT8 seq[2];
    }WIFI_HDR;

    typedef struct{
    	A_UINT8 fc[2];
    	A_UINT8 duration[2];
    	A_UINT8 addr[3][6];
    	A_UINT8 seq[2];
    	A_UINT8 addr_3[6];
    	A_UINT8 pad[2]; // pad required by firmware
    }WIFI_4_HDR;

    typedef struct{
    	A_UINT8 fc[2];
    	A_UINT8 duration[2];
    	A_UINT8 addr[3][6];
    	A_UINT8 seq[2];
    	A_UINT8 qos[2];
    	A_UINT8 pad[2]; // pad required by firmware
    }WIFI_QOS_HDR;
#define WIFI_HDR_SZ (24)
#define WIFI_4_HDR_SZ (32)
#define WIFI_QOS_HDR_SZ (28)
	A_UINT8 header_sz[3]={WIFI_HDR_SZ, WIFI_QOS_HDR_SZ, WIFI_4_HDR_SZ};
	WIFI_4_HDR* p4Hdr;
    WIFI_HDR* pHdr;
    WIFI_QOS_HDR *pQosHdr;


    do{
    	/* collect user inputs if any */

    	if(argc != 5){
    		goto RAW_USAGE;
    	}


    	rate_index = atoi(argv[0]);
    	tries = atoi(argv[1]);
    	size = atoi(argv[2]);
    	chan = atoi(argv[3]);
    	header_type = atoi(argv[4]);


    	if(rate_index > 19 ||
    		tries > 14 ||
    		size > 1400 ||
    		chan < 1 || chan > 13 ||
    		header_type > 2){
    		goto RAW_USAGE;
    	}


    	size += header_sz[header_type];

    	if(NULL == (buffer = A_MALLOC(size,MALLOC_ID_CONTEXT))){
          printf("malloc failure\n");
    		break;
    	}

        /*Check if driver is loaded*/
        if(IS_DRIVER_READY != A_OK){
            return A_ERROR;
        }

        /* Set to MAX PERF Mode */
        SET_POWER_MODE("0");
        A_MDELAY(300);
    	/* convert channel from number to freq */
        if (chan < 27) {
    	    chan = ATH_IOCTL_FREQ_1 + (chan-1)*5;
        } else {
            chan = (5000 + (chan*5));
        }

        param.cmd_id = ATH_SET_CHANNEL;
        param.data = &chan;
        param.length = sizeof(A_UINT32);

        error = HANDLE_IOCTL (&param);
        if (A_OK != error)
        {
            break;
        }



    	pHdr = (WIFI_HDR*)&buffer[0];

    	memset(pHdr, 0, WIFI_HDR_SZ);

        pHdr->fc[0] = 0x80; //beacon

        pHdr->addr[0][0] = 0xff;
        pHdr->addr[0][1] = 0xff;
        pHdr->addr[0][2] = 0xff;
        pHdr->addr[0][3] = 0xff;
        pHdr->addr[0][4] = 0xff;
        pHdr->addr[0][5] = 0xff;

        pHdr->addr[1][0] = 0x00;
        pHdr->addr[1][1] = 0x03;
        pHdr->addr[1][2] = 0x7f;
        pHdr->addr[1][3] = 0xdd;
        pHdr->addr[1][4] = 0xdd;
        pHdr->addr[1][5] = 0xdd;

        pHdr->addr[2][0] = 0x00;
        pHdr->addr[2][1] = 0x03;
        pHdr->addr[2][2] = 0x7f;
        pHdr->addr[2][3] = 0xdd;
        pHdr->addr[2][4] = 0xdd;
        pHdr->addr[2][5] = 0xdd;

        access_cat = ATH_ACCESS_CAT_BE;

        if(header_type == 1){
        	//change frame control to qos data
        	pHdr->fc[0] = 0x88;
        	pHdr->fc[1] = 0x01; //to-ds
        	pQosHdr = (WIFI_QOS_HDR*)pHdr;
        	pQosHdr->qos[0] = 0x03;
        	pQosHdr->qos[1] = 0x00;
        	//bssid == addr[0]
        	memcpy(&(pHdr->addr[0][0]), &(pHdr->addr[1][0]), 6);
        	//change destination address
        	pHdr->addr[2][3] = 0xaa;
        	pHdr->addr[2][4] = 0xaa;
        	pHdr->addr[2][5] = 0xaa;
        	access_cat = ATH_ACCESS_CAT_VI;
        }else if(header_type == 2){
        	pHdr->fc[0] = 0x08;
        	pHdr->fc[1] = 0x03; //to-ds | from-ds
        	p4Hdr = (WIFI_4_HDR*)pHdr;
        	p4Hdr->addr_3[0] = 0x00;
        	p4Hdr->addr_3[1] = 0x03;
        	p4Hdr->addr_3[2] = 0x7f;
        	p4Hdr->addr_3[3] = 0xee;
        	p4Hdr->addr_3[4] = 0xee;
        	p4Hdr->addr_3[5] = 0xee;
        }

        /* now setup the frame for transmission */
    	for(i=header_sz[header_type];i<size;i++)
    		buffer[i] = (i-header_sz[header_type])&0xff;

        memset(&details, 0, sizeof(details));

        details.buffer = buffer;
        details.length = size;
        details.params.pktID = 1;
        details.params.rateSched.accessCategory = access_cat;
        /* we set the ACK bit simply to force the desired number of retries */
        details.params.rateSched.flags = ATH_MAC_TX_FLAG_ACK;
        details.params.rateSched.trySeries[0] = tries;
        details.params.rateSched.rateSeries[0] = rate_index;
        details.params.rateSched.rateSeries[1] = 0xff;//terminate series

        param.cmd_id = ATH_MAC_TX_RAW;
        param.data = &details;
        param.length = sizeof(details);

        /* this starts WPS on the Aheros wifi */
        error = HANDLE_IOCTL (&param);
        if (A_OK != error)
        {
            break;
        }

        status = A_OK;
        break;
 RAW_USAGE:
		printf("raw input error\n");
    	printf("usage = wmiconfig --raw rate num_tries num_bytes channel header_type\n");
   	 	printf("rate = rate index where 0==1mbps; 1==2mbps; 2==5.5mbps etc\n");
   		printf("num_tries = number of transmits 1 - 14\n");
   		printf("num_bytes = payload size 0 to 1400\n");
   		printf("channel = 1 - 11\n");
   		printf("header_type = 0==beacon frame; 1==QOS data frame; 2==4 address data frame\n");
    }while(0);

    if(buffer)
    	A_FREE(buffer,MALLOC_ID_CONTEXT);
    
    tx_raw = 1;   
    return status;
}

/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : wps_handler()
* Returned Value  : 1 - successful completion or
*					0 - failed.
* Comments		  : Handles WPS related functionality
*
*END*-----------------------------------------------------------------*/
static A_INT32 wps_handler( A_INT32 index, A_INT32 argc, char* argv[])
{
        A_UINT32 i;
	A_UINT8 val;
    A_UINT8 wps_mode = 0;
    ATH_NETPARAMS netparams;
    char delimter[] = "-#";
    char *result = NULL;
	wps_scan_params wpsScan, *wpsScan_p;

	int j;

	//init context
 	wps_context.wps_in_progress = 0;
 	wps_context.connect_flag = 0;
    A_MEMZERO(&netparams, sizeof(ATH_NETPARAMS));
    A_MEMZERO(wpsPin, MAX_WPS_PIN_SIZE);
	if(index+1 < argc)
	{
		i = index;

		wps_context.connect_flag = atoi(argv[i]);

		i++;

		if(ATH_STRCMP(argv[i],"push") == 0)
		{
			wps_mode = PUSH_BUTTON;
		}
		else if (ATH_STRCMP(argv[i],"pin") == 0)
		{
			wps_mode = PIN;
		    if( argc < 4)  //User did not enter parameter
		    {
		    	printf("Missing parameters: please enter pin\n");
		    	return A_ERROR;
		    }

		    i++;

			result = strtok( argv[i], delimter );
			if(result == NULL)
			{
				strcpy(wpsPin, argv[i]);
			}
			else
			{
				while( result != NULL )
				{
					strcat(wpsPin, result);
					result = strtok( NULL, delimter );
				}
			}

		}

		i++;

		if(argc > i)
		{
			if((argc-i) < 3)
			{
				printf("missing parameters\n");
				return A_ERROR;
			}
			else
			{
				strcpy((char*)(wpsScan.ssid),argv[i]);
				i++;

				if(strlen(argv[i]) != 12)
				{
					printf("Invalid MAC address length\n");
					return A_ERROR;
				}

				for(j=0;j<6;j++)
				{
					wpsScan.macaddress[j] = 0;
				}
				for(j=0;j<strlen(argv[i]);j++)
				{
					val = ascii_to_hex(argv[i][j]);
					if(val == 0xff)
					{
						printf("Invalid character\n");
						return A_ERROR;
					}
					else
					{
						if((j&1) == 0)
							val <<= 4;

						wpsScan.macaddress[j>>1] |= val;
					}
				}

				i++;
				wpsScan.channel = atoi(argv[i]);

			}
			wpsScan_p = &wpsScan;
		}
		else
		{
			wpsScan_p = NULL;
		}
	}

	if(wps_mode == PUSH_BUTTON)
	{
		if(StartWPS(&netparams, 0,wpsScan_p) == 0)
		{
			printf("WPS connect error\n");
			return A_ERROR;
		}
	}
	else if(wps_mode == PIN)
	{
		if(StartWPS(&netparams, 1,wpsScan_p) == 0)
		{
			printf("WPS connect error\n");
			return A_ERROR;
		}
	}

    wps_context.wps_in_progress = 1;
    return A_OK;
}
#if ENABLE_P2P_MODE
#define isdigit(c)  ( (((c) >= '0') && ((c) <= '9')) ? (1) : (0) )
/*
 * Input an Ethernet address and convert to binary.
 */

static A_STATUS wmic_ether_aton(const char *orig, A_UINT8 *eth)
{
  const char *bufp;
  int i;

  i = 0;
  for(bufp = orig; *bufp != '\0'; ++bufp) {
    unsigned int val;
    unsigned char c = *bufp++;
    if (isdigit(c)) val = c - '0';
    else if (c >= 'a' && c <= 'f') val = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F') val = c - 'A' + 10;
    else break;

    val <<= 4;
    c = *bufp++;
    if (isdigit(c)) val |= c - '0';
    else if (c >= 'a' && c <= 'f') val |= c - 'a' + 10;
    else if (c >= 'A' && c <= 'F') val |= c - 'A' + 10;
    else break;

    eth[i] = (unsigned char) (val & 0377);
    if(++i == ATH_MAC_LEN) {
        /* That's it.  Any trailing junk? */
        if (*bufp != '\0') {
            printf("iw_ether_aton(%s): trailing junk!\n", orig);
            return(A_EINVAL);

        }
        return(A_OK);
    }
    if (*bufp != ':')
        break;
  }

  return(A_EINVAL);
}

#endif



/*FUNCTION*--------------------------------------------------------------------
*
* Function Name   : ath_assert_dump()
* Returned Value  : A_OK - on successful completion
*					A_ERROR - on any failure.
* Comments        : causes assert information to be collected from chip and
*				    printed to stdout.
*
*END*------------------------------------------------------------------------*/
static A_INT32 ath_assert_dump( A_INT32 argc, char* argv[])
{
	ATH_IOCTL_PARAM_STRUCT ath_param;
	A_INT32 error;

	ath_param.cmd_id = ATH_ASSERT_DUMP;
    ath_param.data = NULL;
    ath_param.length = 0;

	error = HANDLE_IOCTL ( &ath_param);

	return error;
}




/*FUNCTION*--------------------------------------------------------------------
*
* Function Name   : scan_control()
* Returned Value  : A_OK - on successful completion
*					A_ERROR - on any failure.
* Comments        : Disables/Enables foreground and background scan operations
*					in the Atheros device.  Both params must be provided where
*					foreground param is first followed by background param. a
*					'0' param disables the scan type while a '1' enables the
*					scan type.  Background scan -- firmware occasionally scans
*					while connected to a network. Foreground scan -- firmware
*					occasionally scans while disconnected to a network.
*
*END*------------------------------------------------------------------------*/
static A_INT32 scan_control( A_INT32 argc, char* argv[])
{
	A_INT32 error;
	ATH_IOCTL_PARAM_STRUCT ath_param;
	A_INT32 fg, bg;
	ATH_SCANPARAMS ath_scan;

        /*Check if driver is loaded*/
        if(IS_DRIVER_READY != A_OK){
            return A_ERROR;
        }
	do{
		if(argc != 2) break;

		fg = atoi(argv[0]);

		if(fg != 0 && fg != 1) break;

		bg = atoi(argv[1]);

		if(bg != 0 && bg != 1) break;

		ath_param.cmd_id = ATH_SCAN_CTRL;
	    ath_param.data = &ath_scan;
	    ath_param.length = sizeof(ath_scan);

		ath_scan.flags = ((fg)? 0:ATH_DISABLE_FG_SCAN) | ((bg)? 0:ATH_DISABLE_BG_SCAN);

		error = HANDLE_IOCTL (&ath_param);

		if(error == A_OK){
			return A_OK;
		}else{
			printf("driver ioctl error\n");
			return A_ERROR;
		}
	}while(0);

	printf("param error: scan control requires 2 inputs [0|1] [0|1]\n");
	return A_ERROR;
}

/*FUNCTION*--------------------------------------------------------------------
*
* Function Name   : get_rate()
* Returned Value  : A_OK - on successful completion
*					A_ERROR - on any failure.
* Comments        : gets TX rate from chip
*
*END*------------------------------------------------------------------------*/
A_INT32 get_rate()
{
    ATH_IOCTL_PARAM_STRUCT ath_param;
    A_INT32 error = A_ERROR;


    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    ath_param.cmd_id = ATH_GET_RATE;
    ath_param.data   = NULL;
    ath_param.length = 0;
    error = HANDLE_IOCTL (&ath_param);

    if(error != A_OK){

    	printf("Command Failed\n");

    }
    return error;
}

/*FUNCTION*--------------------------------------------------------------------
*
* Function Name   : set_tx_power_scale()
* Returned Value  : A_OK - on successful completion
*					A_ERROR - on any failure.
* Comments        : sets TX power scale 
*
*END*------------------------------------------------------------------------*/
static A_INT32 set_tx_power_scale(A_INT32 index, A_INT32 argc,char* argv[])
{
    ATH_IOCTL_PARAM_STRUCT ath_param;
    A_INT32 error = A_ERROR; 
    WMI_SET_TX_POWER_SCALE_CMD pwr_scale;
    
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if((argc > 3) || (argc < 2))
    {
      printf("incorrect arguments refer help \n");
      return error;
    }    
    
    if((atoi(argv[index]) < -15) || (atoi(argv[index]) > 15))
    {
      printf("power value to be from -15dbm to 15dbm \n");
      return error;
    }
    
    pwr_scale.param_val = atoi(argv[index]);
    
    ath_param.cmd_id = ATH_SET_TX_PWR_SCALE;
    ath_param.data = &pwr_scale;
    ath_param.length = sizeof(pwr_scale);
    error = HANDLE_IOCTL (&ath_param);
    
    if(error != A_OK){

    	printf("Command Failed\n");

    }
    return error;    
    
}
static A_INT32 set_tx_power(char* pwr)
{

     ATH_IOCTL_PARAM_STRUCT  inout_param;
    A_INT32 error;
    A_UINT8 dBm;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }
    dBm = atoi(pwr);
    inout_param.cmd_id = ATH_SET_TXPWR;
    inout_param.data = &dBm;
    inout_param.length = 1;//strlen(dBm);

    error = HANDLE_IOCTL (&inout_param);
    if (A_OK != error)
    {
        return error;
    }
    return A_OK;
}
/*FUNCTION*--------------------------------------------------------------------
*
* Function Name   : set_rate()
* Returned Value  : A_OK - on successful completion
*					A_ERROR - on any failure.
* Comments        : sets TX data rate 
*
*END*------------------------------------------------------------------------*/
static A_INT32 set_rate(A_INT32 index, A_INT32 argc, char* argv[])
{
    ATH_IOCTL_PARAM_STRUCT ath_param;
    A_INT32 error = A_ERROR;
    WMI_BIT_RATE_CMD bitrate; 

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if(argc > 4)
    {
      printf("too many arguments refer help \n");
      return error;
    }
    
    if((argc == 3) && (atoi(argv[index]) > 0) && (atoi(argv[index]) <= 54))
    {
      switch(atoi(argv[index])) {
        case 1: 
        bitrate.rateIndex = RATE_1Mb;
        break;
        case 2: 
        bitrate.rateIndex = RATE_2Mb;
        break;
        case 5: 
        bitrate.rateIndex = RATE_5_5Mb;
        break;
        case 11: 
        bitrate.rateIndex = RATE_11Mb;
        break;       
        case 6: 
        bitrate.rateIndex = RATE_6Mb;
        break;
        case 9: 
        bitrate.rateIndex = RATE_9Mb;
        break;
        case 12: 
        bitrate.rateIndex = RATE_12Mb;
        break;
        case 18: 
        bitrate.rateIndex = RATE_18Mb;
        break; 
        case 24: 
        bitrate.rateIndex = RATE_24Mb;
        break;
        case 36: 
        bitrate.rateIndex = RATE_36Mb;
        break;
        case 48: 
        bitrate.rateIndex = RATE_48Mb;
        break;
        case 54: 
        bitrate.rateIndex = RATE_54Mb;
        break;
        default:
          printf("undefined rate \n");
          return error;
          break;
      }
      
    }
    else
    {
      if((argc == 3))
      {  
        printf("invalid command see help\n");
        return error;
      }
    }
   
    
    if((argc == 4) && (strcmp(argv[index], "mcs") == 0))
    {
       switch(atoi(argv[index+1])) {
        case 0: 
        bitrate.rateIndex = RATE_MCS_0_20;
        break;
        case 1: 
        bitrate.rateIndex = RATE_MCS_1_20;
        break;
        case 2: 
        bitrate.rateIndex = RATE_MCS_2_20;
        break;
        case 3: 
        bitrate.rateIndex = RATE_MCS_3_20;
        break;       
        case 4: 
        bitrate.rateIndex = RATE_MCS_4_20;
        break;  
        case 5: 
        bitrate.rateIndex = RATE_MCS_5_20;
        break;
        case 6: 
        bitrate.rateIndex = RATE_MCS_6_20;
        break;       
        case 7: 
        bitrate.rateIndex = RATE_MCS_7_20;
        break;  
        default:
          printf("undefined mcs rate \n");
          return error;
          break;        
       }
       
    }
    else
    {
       if((argc == 4))
      {  
        printf("invalid command see help\n");
        return error;
      }     
    }
    
    ath_param.cmd_id = ATH_SET_RATE;
    ath_param.data = &bitrate;
    ath_param.length = sizeof(bitrate);
    error = HANDLE_IOCTL (&ath_param);

    if(error != A_OK){

    	printf("Command Failed\n");

    }
    return error;
}

/*FUNCTION*--------------------------------------------------------------------
*
* Function Name   : get_tx_status()
* Returned Value  : A_OK - on successful completion
*					A_ERROR - on any failure.
* Comments        : gets TX status from driver
*
*END*------------------------------------------------------------------------*/
A_INT32 get_tx_status( )
{
    ATH_IOCTL_PARAM_STRUCT ath_param;
    A_INT32 error = A_ERROR;
    ATH_TX_STATUS result;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    ath_param.cmd_id = ATH_GET_TX_STATUS;
    ath_param.data = &result;
    ath_param.length = 4;


    error = HANDLE_IOCTL (&ath_param);

    if(error == A_OK){

      switch (result.status)
      {
        case ATH_TX_STATUS_IDLE:
          printf("TX Idle\n");
          break;
        case ATH_TX_STATUS_WIFI_PENDING:
          printf("TX status WIFI Pending \n");
          break;
        case ATH_TX_STATUS_HOST_PENDING:
          printf("TX status Host Pending\n");
          break;
        default:
          printf("Invalid result\n");
          break;
      }
      return result.status;
    }
    else
    {
    	printf("Command Failed\n");
    	return A_ERROR;
    }
}

#if ENABLE_P2P_MODE
/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : p2p_handler()
* Returned Value  : 1 - successful completion or
*                   0 - failed.
* Comments        : Handles P2P related functionality
*
*END*-----------------------------------------------------------------*/
static int_32 p2p_handler( int_32 index, int_32 argc, char_ptr argv[])
{
#define P2P_STANDARD_TIMEOUT (300)
  union{
    WMI_P2P_SET_CMD                 p2p_set_params;
    WMI_P2P_FIND_CMD                find_params;
     WMI_P2P_FW_CONNECT_CMD_STRUCT             p2p_connect;
    WMI_P2P_FW_PROV_DISC_REQ_CMD    p2p_prov_disc;
    WMI_P2P_PROV_INFO               p2p_info;

    //WMI_P2P_PERSISTENT_PROFILE_CMD  persistentCred;

    WMI_SET_PASSPHRASE_CMD          setPassPhrase;
    WMI_P2P_GRP_INIT_CMD            grpInit;
	WMI_P2P_SET_PROFILE_CMD 		p2p;
	WMI_P2P_FW_SET_CONFIG_CMD 	    p2p_set_config;
    WMI_P2P_INVITE_CMD              p2pInvite;
    WMI_P2P_FW_INVITE_REQ_RSP_CMD      invite_req_rsp;
    WMI_NOA_INFO_STRUCT             p2p_noa;
    P2P_NOA_DESCRIPTOR              p2p_desc[4];
    WMI_OPPPS_INFO_STRUCT           p2p_oppps;
  }stack_p2p;
    A_UINT32 timeout_val = 0;
    A_UINT8 invIndex = 0, p2p_invite_role;
    uint_32 error= SHELL_EXIT_SUCCESS,i,j,k;//, val;
    i = index+1;
	set_callback();
    if (index <= argc)
    {

        if(ATH_STRCMP(argv[index],"invite_auth") == 0)
        {
            if(ATH_STRCMP(argv[i],"help") == 0)
            {
                printf("\n Incomplete parameter list for p2p invite_auth          \n");
                printf("\n Usage : wmiconfig --p2p invite_auth <peer_mac_addr> \n");
                printf("                                       <* reject>      \n");
                printf("   * - Only when you reject the invitation             \n");
                return SHELL_EXIT_ERROR;
            }else {
                A_MEMZERO(&stack_p2p.invite_req_rsp,sizeof(WMI_P2P_FW_INVITE_REQ_RSP_CMD));
                if ((wmic_ether_aton(argv[i],stack_p2p.invite_req_rsp.group_bssid)) != A_OK)
                {
                    printf("\nInvalid MAC Address\n");
                    return SHELL_EXIT_ERROR;
                }

            if(ATH_STRCMP(argv[i + 1],"reject") == 0) {
                stack_p2p.invite_req_rsp.status = 1;
            }
            printf("\n Interface MAC addr : %x:%x:%x:%x:%x:%x \n",stack_p2p.invite_req_rsp.group_bssid[0],
                    stack_p2p.invite_req_rsp.group_bssid[1],
                    stack_p2p.invite_req_rsp.group_bssid[2],
                    stack_p2p.invite_req_rsp.group_bssid[3],
                    stack_p2p.invite_req_rsp.group_bssid[4],
                    stack_p2p.invite_req_rsp.group_bssid[5]);

                if(StartP2P(P2P_INVITE_AUTH, (A_UINT8 *)&stack_p2p.invite_req_rsp) == 0)
                {
                    printf("\nStartP2P command did not execute properly\n");
                    return SHELL_EXIT_ERROR;
                }
                p2p_callback(0);
            }
        }
        else if(ATH_STRCMP(argv[index],"join") == 0)
        {
            if(argc < 5)
            {
                printf("\n Incomplete parameter list for p2p join \n");
                printf("\n Usage : wmiconfig --p2p join <intf_mac_addr> \n");
                printf("                                   <push | keypad | display> <* wpspin> \n");
                printf("           * Only for PIN Mode \n");
                return SHELL_EXIT_ERROR;
            }
            else
            {
                /* Set P2P Mode to P2P Client */
                A_MEMZERO(&stack_p2p.p2p_set_params,sizeof(WMI_P2P_SET_CMD));
                stack_p2p.p2p_set_params.config_id = WMI_P2P_CONFID_P2P_OPMODE;
                stack_p2p.p2p_set_params.val.mode.p2pmode = P2P_CLIENT;
                if(StartP2P(SET, (A_UINT8 *)&stack_p2p.p2p_set_params) == 0)
                {
                    printf("\nStartP2P command did not execute properly\n");
                    return SHELL_EXIT_ERROR;
                }
                /* Send the join profile to fw */
                A_MEMZERO(&stack_p2p.p2p_connect,sizeof( WMI_P2P_FW_CONNECT_CMD_STRUCT));
                if ((wmic_ether_aton(argv[i],stack_p2p.p2p_connect.peer_addr)) != A_OK)
                {
                printf("\nInvalid PEER MAC Address\n");
                return SHELL_EXIT_ERROR;
                }
                if(StartP2P(JOIN_PROFILE, (A_UINT8 *)&stack_p2p.p2p_connect) == 0)
                {
                    printf("\nStartP2P command did not execute properly\n");
                    return SHELL_EXIT_ERROR;
                }
                /* Send Join request */
                A_MEMZERO(&stack_p2p.p2p_connect,sizeof( WMI_P2P_FW_CONNECT_CMD_STRUCT));
                if(get_mac_addr((char *)stack_p2p.p2p_connect.own_interface_addr) != SHELL_EXIT_SUCCESS)
                {
                    printf("\n unable to obtain device mac address\n");
                    return SHELL_EXIT_ERROR;
                }

            printf("\n Interface MAC addr : %x:%x:%x:%x:%x:%x \n",stack_p2p.p2p_connect.peer_addr[0],
                    stack_p2p.p2p_connect.peer_addr[1],
                    stack_p2p.p2p_connect.peer_addr[2],
                    stack_p2p.p2p_connect.peer_addr[3],
                    stack_p2p.p2p_connect.peer_addr[4],
                    stack_p2p.p2p_connect.peer_addr[5]);
            if(ATH_STRCMP(argv[i + 1],"push") == 0)
            {
                stack_p2p.p2p_connect.wps_method = WPS_PBC;
            }
            else if(ATH_STRCMP(argv[i + 1],"display") == 0)
            {
                stack_p2p.p2p_connect.wps_method = WPS_PIN_DISPLAY;
            }
            else if(ATH_STRCMP(argv[i + 1],"keypad") == 0)
            {
                stack_p2p.p2p_connect.wps_method = WPS_PIN_KEYPAD;
            }
#if 1           
            if(stack_p2p.p2p_connect.wps_method == WPS_PIN_DISPLAY ||
               stack_p2p.p2p_connect.wps_method == WPS_PIN_KEYPAD)
            {
                strcpy(stack_p2p.p2p_info.wps_pin,argv[i + 2]);
                strcpy(p2p_wps_pin,argv[i + 2]);
                wmi_save_key_info(&stack_p2p.p2p_info);
            }
#endif
            if(StartP2P(JOIN, (A_UINT8 *)&stack_p2p.p2p_connect) == 0)
            {
                printf("\nStartP2P command did not execute properly\n");
                return SHELL_EXIT_ERROR;
            }
         }
        }
        else if(ATH_STRCMP(argv[index],"joinprofile") == 0)
        {
           A_MEMZERO(&stack_p2p.p2p_connect,sizeof( WMI_P2P_FW_CONNECT_CMD_STRUCT));
            if ((wmic_ether_aton(argv[i],stack_p2p.p2p_connect.peer_addr)) != A_OK)
            {
                printf("\nInvalid PEER MAC Address\n");
                return SHELL_EXIT_ERROR;
            }
            printf("\n Join Profile Interface MAC addr : %x:%x:%x:%x:%x:%x \n",stack_p2p.p2p_connect.peer_addr[0],
                    stack_p2p.p2p_connect.peer_addr[1],
                    stack_p2p.p2p_connect.peer_addr[2],
                    stack_p2p.p2p_connect.peer_addr[3],
                    stack_p2p.p2p_connect.peer_addr[4],
                    stack_p2p.p2p_connect.peer_addr[5]);
            if(StartP2P(JOIN_PROFILE, (A_UINT8 *)&stack_p2p.p2p_connect) == 0)
            {
                printf("\nStartP2P command did not execute properly\n");
                return SHELL_EXIT_ERROR;
            }
        }
        else if(ATH_STRCMP(argv[index],"invite") == 0)
        {
            if(argc < 6)
            {
                printf("Incomplete parameter list \n");
                printf("Usage: wmiconfig --p2p invite <SSID> <MACADDR> <WPS_METHOD>\n");                
                return SHELL_EXIT_ERROR;
            }
            for(k = 0; k < MAX_LIST_COUNT; k++)
            {
               if(strncmp(p2p_pers_data[k].ssid, argv[i], strlen(argv[i])) == 0)
               {
                 break;
               }
 
            }
            if(k == MAX_LIST_COUNT)
            {
              printf("the SSID and MAC is not a persistent one \n");
              k = 0;
              return SHELL_EXIT_ERROR;
            }
            invitation_index = k;
            p2p_invite_role = p2p_pers_data[k].role;
           
            A_MEMZERO(&stack_p2p.p2pInvite,sizeof(WMI_P2P_INVITE_CMD));
            
            A_MEMCPY(stack_p2p.p2pInvite.ssid.ssid, argv[i], strlen(argv[i]));
            stack_p2p.p2pInvite.ssid.ssidLength = (strlen(argv[i]));
            stack_p2p.p2pInvite.role = p2p_invite_role;
            
              
            if ((wmic_ether_aton(argv[i+1],stack_p2p.p2pInvite.peer_addr)) != A_OK)
            {
              printf("\nInvalid Invitation MAC Address\n");
              return SHELL_EXIT_ERROR;
            }
            
            if(ATH_STRCMP(argv[i+2], "push") == 0)
            {
              stack_p2p.p2pInvite.wps_method = WPS_PBC;
            }
            else if(ATH_STRCMP(argv[i+2], "display") == 0)
            {
              stack_p2p.p2pInvite.wps_method = WPS_NOT_READY;
            }
            else if(ATH_STRCMP(argv[i+2],"keypad") == 0)
            {
              stack_p2p.p2pInvite.wps_method = WPS_PIN_DISPLAY;
            }
            else
            {
              printf("wps method not proper P2P Invite \n");
              return SHELL_EXIT_ERROR;
            }   
#if 0           
            printf("ssid : %s  mac : %x:%x:%x:%x:%x:%x \n", stack_p2p.p2pInvite.ssid.ssid, stack_p2p.p2pInvite.go_dev_addr[0], 
                                                            stack_p2p.p2pInvite.go_dev_addr[1], stack_p2p.p2pInvite.go_dev_addr[2],
                                                            stack_p2p.p2pInvite.go_dev_addr[3], stack_p2p.p2pInvite.go_dev_addr[4],
                                                            stack_p2p.p2pInvite.go_dev_addr[5]);
#endif            
            if(StartP2P(P2P_INVITE, (A_UINT8 *)&stack_p2p.p2pInvite) == 0)
            {
                printf("\nStartP2P command did not execute properly\n");
                return SHELL_EXIT_ERROR;
            }
            
        }
        else if(ATH_STRCMP(argv[index],"wpspin") == 0)
        {
            if((argc < 4) || (ATH_STRCMP(argv[i],"help") == 0))
            {
                printf("Incomplete parameter list \n");
                printf("Usage: wmiconfig --p2p wpspin <WPSPIN> \n");
                return SHELL_EXIT_ERROR;
            }
#if 0
            if ((wmic_ether_aton(argv[i+1],stack_p2p.p2p_info.peer_addr)) != A_OK)
            {
                printf("\nInvalid PEER MAC Address\n");
                return SHELL_EXIT_ERROR;
            }
#endif
            if((strlen(argv[i]) < 8) && (strlen(argv[i]) > 8))
            {
                printf("\nPIN has to be 8 digit value\n");
                return SHELL_EXIT_ERROR;
            }
            else
            {
                strcpy(stack_p2p.p2p_info.wps_pin,argv[i]);
                strcpy(p2p_wps_pin,argv[i]);
            }

            wmi_save_key_info(& stack_p2p.p2p_info);
        }
        else if(ATH_STRCMP(argv[index],"find") == 0)
        {
            if(ATH_STRCMP(argv[i],"help") == 0){
                printf("\n Usage : wmiconfig --p2p find <* type> <* timeout> \n");
                printf("\n * - optional \n");
                return SHELL_EXIT_ERROR;
            }
            A_MEMZERO(&stack_p2p.find_params, sizeof(WMI_P2P_FIND_CMD));
            if(argc == 4)
            {
                if(ATH_STRCMP(argv[i],"1") == 0)
                {
                    stack_p2p.find_params.type = A_CPU2LE32(WMI_P2P_FIND_START_WITH_FULL);
                    stack_p2p.find_params.timeout = A_CPU2LE32(P2P_STANDARD_TIMEOUT);
                }
                else if(ATH_STRCMP(argv[i],"2") == 0)
                {
                    stack_p2p.find_params.type = A_CPU2LE32(WMI_P2P_FIND_ONLY_SOCIAL);
                    stack_p2p.find_params.timeout = A_CPU2LE32(P2P_STANDARD_TIMEOUT);
                }
                else if(ATH_STRCMP(argv[i],"3") == 0)
                {
                    stack_p2p.find_params.type = A_CPU2LE32(WMI_P2P_FIND_PROGRESSIVE);
                    stack_p2p.find_params.timeout = A_CPU2LE32(P2P_STANDARD_TIMEOUT);
                }
                else
                {
                    printf("\n wrong option enter option 1,2 or 3\n");
                    return SHELL_EXIT_ERROR;
                }

            }
            else if(argc == 5)
            {
                if(ATH_STRCMP(argv[i],"1") == 0)
                {
                    stack_p2p.find_params.type = A_CPU2LE32(WMI_P2P_FIND_START_WITH_FULL);
                    stack_p2p.find_params.timeout = A_CPU2LE32(atoi(argv[i+1]));
                }
                else if(ATH_STRCMP(argv[i],"2") == 0)
                {
                    stack_p2p.find_params.type = A_CPU2LE32(WMI_P2P_FIND_ONLY_SOCIAL);
                    stack_p2p.find_params.timeout = A_CPU2LE32(atoi(argv[i+1]));
                }
                else if(ATH_STRCMP(argv[i],"3") == 0)
                {
                    stack_p2p.find_params.type = A_CPU2LE32(WMI_P2P_FIND_PROGRESSIVE);
                    stack_p2p.find_params.timeout = A_CPU2LE32(atoi(argv[i+1]));
                }
                else
                {
                    printf("\n wrong option enter option 1,2 or 3\n");
                    return SHELL_EXIT_ERROR;
                }
            }
            else
            {
                stack_p2p.find_params.type = A_CPU2LE32(WMI_P2P_FIND_ONLY_SOCIAL);
                stack_p2p.find_params.timeout = A_CPU2LE32(P2P_STANDARD_TIMEOUT);
            }

            if(StartP2P(P2P_FIND, (A_UINT8 *)&(stack_p2p.find_params)) == 0)
            {
                printf("\nStartP2P command did not execute properly\n");
                return SHELL_EXIT_ERROR;
            }
        }
        else if(ATH_STRCMP(argv[index],"list_network") == 0)
        {
 
            if(StartP2P(LIST_NETWORK, NULL) == 0)
            {
                printf("\nStartP2P command did not execute properly\n");
                return SHELL_EXIT_ERROR;
            }

         p2p_callback(0);         
        }
        else if(ATH_STRCMP(argv[index],"prov_resp") == 0)
        {
           p2p_callback(1);
        }
        else if(ATH_STRCMP(argv[index],"connect_req") == 0)
        {
           p2p_callback(1);
        }
        else if(ATH_STRCMP(argv[index],"serv_disc") == 0)
        {
           p2p_callback(1);
        }        
        else if(ATH_STRCMP(argv[index],"set") == 0)
        {
            A_MEMZERO(&stack_p2p.p2p_set_params,sizeof(WMI_P2P_SET_CMD));
            if(ATH_STRCMP(argv[i],"p2pmode") == 0)
            {
                stack_p2p.p2p_set_params.config_id = WMI_P2P_CONFID_P2P_OPMODE;
                if(ATH_STRCMP(argv[i +1],"p2pdev") == 0)
                {
                    stack_p2p.p2p_set_params.val.mode.p2pmode = P2P_DEV;
                }
                else if(ATH_STRCMP(argv[i +1],"p2pclient") == 0)
                {
                    stack_p2p.p2p_set_params.val.mode.p2pmode = P2P_CLIENT;
                }
                else if(ATH_STRCMP(argv[i +1],"p2pgo") == 0)
                {
                    stack_p2p.p2p_set_params.val.mode.p2pmode = P2P_GO;
                }
                else
                {
                    printf("Input can be \"p2pdev/p2pclient/p2pgo\"");
                }

                printf("p2p mode :%x, Config Id\n",stack_p2p.p2p_set_params.val.mode.p2pmode,stack_p2p.p2p_set_params.config_id);
            }else if(ATH_STRCMP(argv[i],"postfix") == 0){

                stack_p2p.p2p_set_params.config_id = WMI_P2P_CONFID_SSID_POSTFIX;
                if(strlen(argv[i+1])) {
                    A_MEMCPY(stack_p2p.p2p_set_params.val.ssid_postfix.ssid_postfix,argv[i+1],strlen(argv[i+1]));
                    stack_p2p.p2p_set_params.val.ssid_postfix.ssid_postfix_len = strlen(argv[i+1]);
                    printf("PostFix string %s, Len %d\n",stack_p2p.p2p_set_params.val.ssid_postfix.ssid_postfix, stack_p2p.p2p_set_params.val.ssid_postfix.ssid_postfix_len);
                }
            }
            else if(ATH_STRCMP(argv[i], "intrabss") == 0)
            {
				stack_p2p.p2p_set_params.config_id = WMI_P2P_CONFID_INTRA_BSS;
				stack_p2p.p2p_set_params.val.intra_bss.flag = atoi(argv[i+1]);
			}
            else if(ATH_STRCMP(argv[i], "gointent") == 0)
	        {
		        stack_p2p.p2p_set_params.config_id = WMI_P2P_CONFID_GO_INTENT;
		        stack_p2p.p2p_set_params.val.go_intent.value = atoi(argv[i+1]);
	        }
            if(StartP2P(SET, (A_UINT8 *)&stack_p2p.p2p_set_params) == 0)
            {
                printf("\nStartP2P command did not execute properly\n");
                return SHELL_EXIT_ERROR;
            }

        }
        else if(ATH_STRCMP(argv[index],"cancel") == 0)
        {
            if(argc > 3)
            {
                printf("\n Usage : wmiconfig --p2p cancel \n");
                return SHELL_EXIT_ERROR;
            } else { 
                if(StartP2P(CANCEL, NULL) == 0)
                {
                    printf("\nStartP2P command did not execute properly\n");
                    return SHELL_EXIT_ERROR;
                }
                autogo_newpp = FALSE;
                
                p2p_cancel_enable = 1;
                
                if(StartP2P(LIST_NETWORK, NULL) == 0)
                {
                  printf("\nStartP2P command did not execute properly\n");
                  return SHELL_EXIT_ERROR;
                }
                p2p_callback(0);                 
            }
            /* Reset to mode station */
            mode_flag = MODE_STATION;
        }
        else if(ATH_STRCMP(argv[index],"prov") == 0)
        {
            if(argc < 4 || (ATH_STRCMP(argv[i], "help") == 0))
            {
                printf("\n Incomplete parameter list for p2p prov discovery \n");
                printf("\n Usage : wmiconfig --p2p prov <peer_mac_addr> <wps_method> \n");
                return SHELL_EXIT_ERROR;
            }
            else //if((argc >= 5) && (argc < 7))
            {
                if(ATH_STRCMP(argv[i+1], "push") == 0)
                {
                    rand();
                    stack_p2p.p2p_prov_disc.wps_method = A_CPU2LE16((A_UINT16)WPS_CONFIG_PUSHBUTTON);
                }
                else if(ATH_STRCMP(argv[i+1], "display") == 0)
                {
                    stack_p2p.p2p_prov_disc.wps_method = A_CPU2LE16((A_UINT16)WPS_CONFIG_DISPLAY);
                }
                else if(ATH_STRCMP(argv[i+1],"keypad") == 0)
                {
                    stack_p2p.p2p_prov_disc.wps_method = A_CPU2LE16((A_UINT16)WPS_CONFIG_KEYPAD);
                }
                stack_p2p.p2p_prov_disc.dialog_token = 1;

                if ((wmic_ether_aton(argv[i],stack_p2p.p2p_prov_disc.peer)) != A_OK)
                {
                    printf("\nInvalid PEER MAC Address\n");
                    return SHELL_EXIT_ERROR;
                }
				
                if(StartP2P(DISC_REQ, (A_UINT8 *)&stack_p2p.p2p_prov_disc) == 0)
                {
                    printf("\nStartP2P command did not execute properly\n");
                    return SHELL_EXIT_ERROR;
                }
                p2p_callback(0);
                
                /* Authorize P2P Device */
                A_MEMZERO(&stack_p2p.p2p_connect, sizeof( WMI_P2P_FW_CONNECT_CMD_STRUCT));
                if(ATH_STRCMP(argv[i+1], "push") == 0)
                {
                    stack_p2p.p2p_connect.wps_method = WPS_PBC;
                }
                else if(ATH_STRCMP(argv[i+1], "display") == 0)
                {
                    stack_p2p.p2p_connect.wps_method = WPS_NOT_READY;
                }
                else if(ATH_STRCMP(argv[i+1],"keypad") == 0)
                {
                    stack_p2p.p2p_connect.wps_method = WPS_PIN_DISPLAY;
                }

                if ((wmic_ether_aton(argv[i],stack_p2p.p2p_connect.peer_addr)) != A_OK)
                {
                    printf("\nInvalid PEER MAC Address\n");
                    return SHELL_EXIT_ERROR;
                }
                
                stack_p2p.p2p_connect.go_intent = p2p_intent;
                
                if(stack_p2p.p2p_connect.wps_method != WPS_NOT_READY) {
                    if(StartP2P(AUTH, (A_UINT8 *)&stack_p2p.p2p_connect) == 0)
                    {
                        printf("\nStartP2P command did not execute properly\n");
                        return SHELL_EXIT_ERROR;
                    }
                }
            }

        }
        else if(ATH_STRCMP(argv[index],"auth") == 0)
        {
            A_MEMZERO(&stack_p2p.p2p_connect, sizeof( WMI_P2P_FW_CONNECT_CMD_STRUCT));
            if(argc < 5)
            {
                printf("\n Incomplete parameter list for p2p auth \n");
                printf("\n Usage : wmiconfig --p2p auth <peer_mac_addr> <wps_method> <* wps_pin>\n");
                return SHELL_EXIT_ERROR;
            }
            else //if((argc >= 5) && (argc < 7))
            {
                if(strlen(argv[i+2]) == 8){
                    strcpy(stack_p2p.p2p_info.wps_pin,argv[i + 2]);
                    strcpy(p2p_wps_pin,argv[i + 2]);
                    wmi_save_key_info(&stack_p2p.p2p_info);
                    printf("WPS Pin %s\n",p2p_wps_pin);
                }
                if(ATH_STRCMP(argv[i+1], "deauth") == 0)
                {
                    stack_p2p.p2p_connect.dev_auth = 1;
                }
                if(ATH_STRCMP(argv[i+1], "push") == 0)
                {
                    stack_p2p.p2p_connect.wps_method = WPS_PBC;
                }
                else if(ATH_STRCMP(argv[i+1], "display") == 0)
                {
                    stack_p2p.p2p_connect.wps_method = WPS_PIN_DISPLAY;
                    printf("WPS PIN %s \n",p2p_wps_pin);
                }
                else if(ATH_STRCMP(argv[i+1],"keypad") == 0)
                {
                    stack_p2p.p2p_connect.wps_method = WPS_PIN_KEYPAD;
                }

                if ((wmic_ether_aton(argv[i],stack_p2p.p2p_connect.peer_addr)) != A_OK)
                {
                    printf("\nInvalid PEER MAC Address\n");
                    return SHELL_EXIT_ERROR;
                }
                stack_p2p.p2p_connect.go_intent = p2p_intent;
                if(StartP2P(AUTH, (A_UINT8 *)&stack_p2p.p2p_connect) == 0)
                {
                    printf("\nStartP2P command did not execute properly\n");
                    return SHELL_EXIT_ERROR;
                }
            }

            task2_msec = 50;
        }
        else if(ATH_STRCMP(argv[index],"stop") == 0)
        {
            if(StartP2P(STOP, NULL) == 0)
            {
                printf("\nStartP2P command did not execute properly\n");
                return SHELL_EXIT_ERROR;
            }
        }
        else if(ATH_STRCMP(argv[index],"nodelist") == 0)
        {
            if(StartP2P(NODE_LIST, NULL) == 0)
            {
                printf("\nStartP2P command did not execute properly\n");
                return SHELL_EXIT_ERROR;
            }
            app_time_delay(100);
            p2p_callback(0);
        }
        else if(ATH_STRCMP(argv[index],"listen") == 0)
        {
            if (argc == 4)
            {
                timeout_val = atoi(argv[i]);
            }
            else
            {
                timeout_val = P2P_STANDARD_TIMEOUT;
            }
            if(StartP2P(LISTEN, (A_UINT8 *)&timeout_val) == 0)
            {
                printf("\nStartP2P command did not execute properly\n");
                return SHELL_EXIT_ERROR;
            }
        }
        else if(ATH_STRCMP(argv[index], "connect") == 0)
        {
            if(argc < 5)
            {
                printf("\n Incomplete parameter list for p2p connect \n");
                printf("\n Usage : wmiconfig --p2p connect <peer_mac_addr> \n");
                printf("                                   <wps_method>\n");
                return SHELL_EXIT_ERROR;
            }
            else //if ((argc >= 6) && (argc < 8))
            {
                A_MEMZERO(&stack_p2p.p2p_info,sizeof(WMI_P2P_PROV_INFO)); 
                
                if(strlen(argv[i+2]) == 8){
                    strcpy(stack_p2p.p2p_info.wps_pin,argv[i + 2]);
                    strcpy(p2p_wps_pin,argv[i + 2]);
                    wmi_save_key_info(&stack_p2p.p2p_info);
                    printf("WPS Pin %s\n",p2p_wps_pin);
                }
                
                A_MEMZERO(&stack_p2p.p2p_connect,sizeof( WMI_P2P_FW_CONNECT_CMD_STRUCT));
                if(get_mac_addr((char *)stack_p2p.p2p_connect.own_interface_addr) != SHELL_EXIT_SUCCESS)
                {
                    printf("\n unable to obtain device mac address\n");
                    return SHELL_EXIT_ERROR;
                }
                printf("\n own MAC addr : %x:%x:%x:%x:%x:%x \n", stack_p2p.p2p_connect.own_interface_addr[0],
                        stack_p2p.p2p_connect.own_interface_addr[1],
                        stack_p2p.p2p_connect.own_interface_addr[2],
                        stack_p2p.p2p_connect.own_interface_addr[3],
                        stack_p2p.p2p_connect.own_interface_addr[4],
                        stack_p2p.p2p_connect.own_interface_addr[5]);

                if ((wmic_ether_aton(argv[i],stack_p2p.p2p_connect.peer_addr)) != A_OK)
                {
                    printf("\nInvalid PEER MAC Address\n");
                    return SHELL_EXIT_ERROR;
                }
                printf("\n Peer MAC addr : %x:%x:%x:%x:%x:%x \n",stack_p2p.p2p_connect.peer_addr[0],
                        stack_p2p.p2p_connect.peer_addr[1],
                        stack_p2p.p2p_connect.peer_addr[2],
                        stack_p2p.p2p_connect.peer_addr[3],
                        stack_p2p.p2p_connect.peer_addr[4],
                        stack_p2p.p2p_connect.peer_addr[5]);
                if(ATH_STRCMP(argv[i + 1],"push") == 0)
                {
                    stack_p2p.p2p_connect.wps_method = WPS_PBC;
                }
                else if(ATH_STRCMP(argv[i + 1],"display") == 0)
                {
                    stack_p2p.p2p_connect.wps_method = WPS_PIN_DISPLAY;
                    printf("WPS PIN %s \n",p2p_wps_pin);
                }
                else if(ATH_STRCMP(argv[i + 1],"keypad") == 0)
                {
                    stack_p2p.p2p_connect.wps_method = WPS_PIN_KEYPAD;
                    //strcpy(p2p_wps_pin_val, argv[i+2]);
                }
#if 0
                printf("\n go_intent : %d \n", atoi(argv[i+2]));
#endif
                /* Setting Default Value for now !!! */
                stack_p2p.p2p_connect.dialog_token = 1;
                stack_p2p.p2p_connect.go_intent    = p2p_intent;
                stack_p2p.p2p_connect.go_dev_dialog_token = 0;
                stack_p2p.p2p_connect.dev_capab    = 0x23;
                if(StartP2P(CONNECT_CLIENT, (A_UINT8 *)&stack_p2p.p2p_connect) == 0)
                {
                    printf("\nStartP2P command did not execute properly\n");
                    return SHELL_EXIT_ERROR;
                }
                p2p_callback(0);
            }
        }
        else if(ATH_STRCMP(argv[index], "autogo") == 0)
        {
            if(ATH_STRCMP(argv[i],"help") == 0){
                printf("\n Usage : wmiconfig --p2p autogo \n");
                return SHELL_EXIT_ERROR;
            }
            if(FALSE == autogo_newpp) {
                A_MEMZERO(&stack_p2p.setPassPhrase, sizeof(WMI_SET_PASSPHRASE_CMD));

            strcpy((char *)stack_p2p.setPassPhrase.passphrase,wpa_passphrase);
            stack_p2p.setPassPhrase.passphrase_len = strlen(wpa_passphrase);

            strcpy((char *)stack_p2p.setPassPhrase.ssid,original_ssid);
            stack_p2p.setPassPhrase.ssid_len = strlen(original_ssid);

            error = ATH_SET_ESSID (DEMOCFG_DEFAULT_DEVICE,(char*)stack_p2p.setPassPhrase.ssid);
            if(error != A_OK)
            {
                printf("Unable to set SSID\n");
                return error;
            }

            cipher.ucipher = 0x08;
            cipher.mcipher = 0x08;
            error = set_cipher(DEMOCFG_DEFAULT_DEVICE,(char *)&cipher);

            if(error != A_OK)
            {
                return A_ERROR;
            }

            strcpy(wpa_ver,"wpa2");
            error = ATH_SET_SEC_TYPE (DEMOCFG_DEFAULT_DEVICE,wpa_ver);

            if(error != A_OK)
            {
                return A_ERROR;
            }

            security_mode = SEC_MODE_WPA;

            if (StartP2P(AP_MODE_PP,(A_UINT8 *)&stack_p2p.setPassPhrase) == 0)
            {
                printf("\nStartP2P command did not execute properly\n");
                return SHELL_EXIT_ERROR;
            }
            }
            //
            // make AP Mode and WPS default settings for P2P GO
            //
            ATH_SET_MODE (DEMOCFG_DEFAULT_DEVICE,"softap");
            wps_flag = 0x01;

			if(set_channel(set_channel_p2p) == A_ERROR)
			{
				printf("setting channel failed in p2p setconfig \n");
				return SHELL_EXIT_ERROR;
			}

            if (set_ap_params (AP_SUB_CMD_WPS_FLAG, (A_UINT8 *)&wps_flag) != SHELL_EXIT_SUCCESS)
            {
                printf("not able to set wps mode for AP \n");
                return SHELL_EXIT_ERROR;
            }

            A_MEMZERO(&stack_p2p.grpInit ,sizeof(WMI_P2P_GRP_INIT_CMD));

            {
                printf("Starting Autonomous GO \n");
                stack_p2p.grpInit.group_formation = 1;
                if(ATH_STRCMP(argv[i + 1],"persistent") == 0)
                {
                    stack_p2p.grpInit.persistent_group = 1;
                }
            }
            mode_flag = MODE_AP;
            if(StartP2P(AP_MODE,(A_UINT8 *)&stack_p2p.grpInit) == 0)
            {
                printf("\nStartP2P command did not execute properly\n");
                return SHELL_EXIT_ERROR;
            }
            /* Set MAX PERF */
            SET_POWER_MODE("0");
        }
    else if((ATH_STRCMP(argv[index], "on") == 0))
    {
        if(argc > 3)
        {
            printf("\n Usage : wmiconfig --p2p <on / off> \n");
            return SHELL_EXIT_ERROR;
        } else {
            A_MEMZERO(&stack_p2p.p2p,sizeof(WMI_P2P_SET_PROFILE_CMD));
            stack_p2p.p2p.enable = TRUE;
            if(StartP2P(P2P_ON_OFF,(A_UINT8 *)&stack_p2p.p2p) == 0)
            {
                printf("\nStartP2P command did not execute properly\n");
                return SHELL_EXIT_ERROR;
            }
            
            A_MEMZERO(&stack_p2p.p2p_set_config,sizeof(WMI_P2P_FW_SET_CONFIG_CMD));
            stack_p2p.p2p_set_config.go_intent  = 0;
            p2p_intent = 0;
            stack_p2p.p2p_set_config.reg_class      = 81;
            stack_p2p.p2p_set_config.op_reg_class   = 81;
            stack_p2p.p2p_set_config.listen_channel = 1;
            stack_p2p.p2p_set_config.op_channel     = 1;
            strcpy((char *)stack_p2p.p2p_set_config.country, "US");
        stack_p2p.p2p_set_config.node_age_to    = 3000;
        stack_p2p.p2p_set_config.max_node_count = 4;
            A_MEMCPY(set_channel_p2p,"6" ,1);
    
        
            if(stack_p2p.p2p_set_config.max_node_count == 0)
            {
                printf("\nStartP2P node count should not be zero\n");
                return SHELL_EXIT_ERROR;
        }
            
            if(StartP2P(SET_CONFIG, (A_UINT8 *)&stack_p2p.p2p_set_config) == 0)
            {
                printf("\nStartP2P command did not execute properly\n");
                return SHELL_EXIT_ERROR;
            }
            A_MEMZERO(&stack_p2p.p2p_info,sizeof(WMI_P2P_PROV_INFO));
            strcpy(stack_p2p.p2p_info.wps_pin,"12345670");
            strcpy(p2p_wps_pin,"12345670");
            wmi_save_key_info(&stack_p2p.p2p_info);
            
            /* Autonomous GO configurations */
          
             set_passphrase("1234567890");
             strcpy(original_ssid,"DIRECT-iO");
                         
        }
    }
        else if((ATH_STRCMP(argv[index], "off") == 0))
        {
            if(argc > 3)
            {
                printf("\n Usage : wmiconfig --p2p <on / off> \n");
                return SHELL_EXIT_ERROR;
            } else {
                A_MEMZERO(&stack_p2p.p2p,sizeof(WMI_P2P_SET_PROFILE_CMD));
                stack_p2p.p2p.enable = FALSE;
                if(StartP2P(P2P_ON_OFF,(A_UINT8 *)&stack_p2p.p2p) == 0)
                {
                    printf("\nStartP2P command did not execute properly\n");
                    return SHELL_EXIT_ERROR;
                }
            }
            mode_flag = MODE_STATION;
        }
        else if(ATH_STRCMP(argv[index], "setopclass") == 0)
        {
            if(argc < 6 )
            {
                printf("\n Usage : wmiconfig --p2p setopclass <go_intent> <oper_regclass> <oper_regchannel>\n");
                return SHELL_EXIT_ERROR;
            } else {
                A_MEMZERO(&stack_p2p.p2p_set_config,sizeof(WMI_P2P_FW_SET_CONFIG_CMD));
                
                p2p_intent = stack_p2p.p2p_set_config.go_intent  = atoi(argv[i]);
                stack_p2p.p2p_set_config.reg_class      = 81;
                stack_p2p.p2p_set_config.op_reg_class   = atoi(argv[i+1]);
                stack_p2p.p2p_set_config.listen_channel = 6;
                stack_p2p.p2p_set_config.op_channel     = atoi(argv[i+2]);
                strcpy((char *)stack_p2p.p2p_set_config.country, "US");
                stack_p2p.p2p_set_config.node_age_to    = 3000;
                stack_p2p.p2p_set_config.max_node_count = 4;
                A_MEMCPY(set_channel_p2p, argv[i+2], sizeof(argv[i+2])); // for autogo

                if(StartP2P(SET_CONFIG, (A_UINT8 *)&stack_p2p.p2p_set_config) == 0)
                {
                    printf("\nStartP2P command did not execute properly\n");
                    return SHELL_EXIT_ERROR;
                }
            }
        }
        else if(ATH_STRCMP(argv[index], "setconfig") == 0)
        {

            if(argc < 8)
            {
                printf("\n Incomplete parameter list for p2p set config \n");
                printf("\n Usage : wmiconfig --p2p setconfig <go_intent> <listen_channel> \n");
                printf("                                           <oper_channel> <country> <node_age_timeout>\n");
                return SHELL_EXIT_ERROR;
            }
            else
            {
                A_MEMZERO(&stack_p2p.p2p_set_config,sizeof(WMI_P2P_FW_SET_CONFIG_CMD));
                p2p_intent = stack_p2p.p2p_set_config.go_intent  = atoi(argv[i]);

            	stack_p2p.p2p_set_config.reg_class      = 81;
            	stack_p2p.p2p_set_config.op_reg_class   = 81;
            	stack_p2p.p2p_set_config.listen_channel = atoi(argv[i+1]);
            	stack_p2p.p2p_set_config.op_channel     = atoi(argv[i+2]);
            	strcpy((char *)stack_p2p.p2p_set_config.country, argv[i+3]);
				stack_p2p.p2p_set_config.node_age_to    = atoi(argv[i+4]);
				stack_p2p.p2p_set_config.max_node_count = 5;
                A_MEMCPY(set_channel_p2p, argv[i+2], sizeof(argv[i+2])); // for autogo
				if(stack_p2p.p2p_set_config.max_node_count == 0)
				{
            	    printf("\nStartP2P node count should not be zero\n");
            	    return SHELL_EXIT_ERROR;
				}

            	if(StartP2P(SET_CONFIG, (A_UINT8 *)&stack_p2p.p2p_set_config) == 0)
            	{
            	    printf("\nStartP2P command did not execute properly\n");
            	    return SHELL_EXIT_ERROR;
            	}
			}
        }
            else if(ATH_STRCMP(argv[index], "name") == 0)
            {
                strcpy((char *)ssid,argv[index+1]);                  
            }
            else if(ATH_STRCMP(argv[index], "defaultconnect") == 0)
            {
                if((ATH_STRCMP(argv[index+1], "1") == 0)) {
                    p2p_connection_default = 1;          
                }else if((ATH_STRCMP(argv[index+1], "0") == 0)){
                    p2p_connection_default = 0;
                }else{
                    printf("\n Invalid Argument: 0 - Allow other P2P Device to connect; 1 - Allow IOE Device to connect(Issue connect) \n");
                    return SHELL_EXIT_ERROR;
                }
            }
            else if(ATH_STRCMP(argv[index], "passphrase") == 0)
            {
                if(argc < 5)
                {
                    printf("\n Incomplete parameter list for p2p set config \n");
                    printf("\n Usage : wmiconfig --p2p passphrase <passphrase> <SSID> \n");
                    return SHELL_EXIT_ERROR;
                }
                else {
                    A_MEMZERO(&stack_p2p.setPassPhrase, sizeof(WMI_SET_PASSPHRASE_CMD));

            strcpy((char *)stack_p2p.setPassPhrase.passphrase,argv[index+1]);
            stack_p2p.setPassPhrase.passphrase_len = strlen(argv[index+1]);

            strcpy((char *)stack_p2p.setPassPhrase.ssid,argv[index+2]);
            stack_p2p.setPassPhrase.ssid_len = strlen(argv[index+2]);

            error = ATH_SET_ESSID (DEMOCFG_DEFAULT_DEVICE,(char*)stack_p2p.setPassPhrase.ssid);
            if(error != A_OK)
            {
                printf("Unable to set SSID\n");
                return error;
            }

            cipher.ucipher = 0x08;
            cipher.mcipher = 0x08;
            error = set_cipher(DEMOCFG_DEFAULT_DEVICE,(char *)&cipher);

            if(error != A_OK)
            {
                return A_ERROR;
            }

            strcpy(wpa_ver,"wpa2");
            error = ATH_SET_SEC_TYPE (DEMOCFG_DEFAULT_DEVICE,wpa_ver);

            if(error != A_OK)
            {
                return A_ERROR;
            }

            security_mode = SEC_MODE_WPA;
            autogo_newpp = TRUE;

                    if (StartP2P(AP_MODE_PP,(A_UINT8 *)&stack_p2p.setPassPhrase) == 0)
                    {
                        printf("\nStartP2P command did not execute properly\n");
                        return SHELL_EXIT_ERROR;
                    }
                }
            }
#if 0
        else if(ATH_STRCMP(argv[index], "setoppps") == 0)
        {
            if(argc < 5)
            {
                printf("\n Incomplete parameter list for p2p set oppps \n");
                printf("\n Usage : wmiconfig --p2p setoppps <enable> <ctwin> \n");
                return SHELL_EXIT_ERROR;
            }
            else
            {
                A_MEMZERO(&stack_p2p.p2p_oppps, sizeof(WMI_OPPPS_INFO_STRUCT));
                stack_p2p.p2p_oppps.enable = atoi(argv[i]);
                stack_p2p.p2p_oppps.ctwin = atoi(argv[i + 1]);

                if (StartP2P(P2P_SET_OPPPS,(A_UINT8 *)&stack_p2p.p2p_oppps) == 0)
                {
                    printf("\nStartP2P command did not execute properly\n");
                    return SHELL_EXIT_ERROR;
                }
            }
             
        }
        else if(ATH_STRCMP(argv[index], "setnoa") == 0)
        {
            if(argc < 6)
            {
                printf("\n Incomplete parameter list for p2p set noa \n");
                printf("\n Usage : wmiconfig --p2p setnoa <total Desc Count> <count> <start> \n");
                printf("                                           <duration> \n");
                return SHELL_EXIT_ERROR;
            }
            else
            {
		int cnt = 0;
                A_MEMZERO(&stack_p2p.p2p_noa, sizeof(WMI_NOA_INFO_STRUCT));
                A_MEMZERO(stack_p2p.p2p_desc, 4 * sizeof(P2P_NOA_DESCRIPTOR));
                stack_p2p.p2p_noa.enable = 1;
              //  p2p_noa.count = 1;
                stack_p2p.p2p_noa.count = atoi(argv[i]);
                for(;cnt<atoi(argv[i]);cnt++) {
                   // p2p_desc.count_or_type = atoi(argv[i+1]);
                   // p2p_desc.start_or_offset =  A_CPU2LE32(atoi(argv[i+2]));
                   // p2p_desc.duration = A_CPU2LE32(atoi(argv[i+3]));
                   // p2p_desc.interval =  A_CPU2LE32(0);
		}

                A_MEMCPY(stack_p2p.p2p_noa.noas, stack_p2p.p2p_desc, stack_p2p.p2p_noa.count * sizeof(P2P_NOA_DESCRIPTOR));

                if (StartP2P(P2P_SET_NOA,(A_UINT8 *)&stack_p2p.p2p_noa) == 0)
                {
                    printf("\nStartP2P command did not execute properly\n");
                    return SHELL_EXIT_ERROR;
                }
            }

        }
#endif
        else
        {
            printf("\n This option is currently not supported\n");
            return SHELL_EXIT_ERROR;
        }
    }
    else
    {
        printf("\n Incorrect Incomplete parameter list \n");
        return SHELL_EXIT_ERROR;
    }
    return SHELL_EXIT_SUCCESS;
}
#endif

/*FUNCTION*--------------------------------------------------------------------
*
* Function Name   : get_reg_domain()
* Returned Value  : A_OK - on successful completion
*		    A_ERROR - on any failure.
* Comments        : gets Regulatory domain from driver
*
*END*------------------------------------------------------------------------*/
static A_INT32 get_reg_domain( )
{
    ATH_IOCTL_PARAM_STRUCT ath_param;
    A_INT32 error = A_ERROR;
    ATH_REG_DOMAIN result;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    ath_param.cmd_id = ATH_GET_REG_DOMAIN;
    ath_param.data = &result;
    ath_param.length = 0;

    error = HANDLE_IOCTL (&ath_param);

    if(error == A_OK){
      printf("Regulatory Domain 0x%x\n",result.domain);
    }
    else{
      printf("Command Failed\n");
      return error;
    }
    return A_OK;
}






static A_INT32 set_channel(char* channel)
{
     ATH_IOCTL_PARAM_STRUCT  inout_param;
    A_INT32 error, chnl;
    char* p = 0;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    chnl = strtoul(channel, &p,0);

    if(chnl < 1 || chnl > 165)
    {
      printf("invalid channel: should range between 1-165\n");
      return A_ERROR;
    }

    /* convert channel from number to freq */
        if (chnl < 27) {
    	    chnl = ATH_IOCTL_FREQ_1 + (chnl-1)*5;
        } else {
            chnl = (5000 + (chnl*5));
        }

    inout_param.cmd_id = ATH_SET_CHANNEL;
    inout_param.data = &chnl;
    inout_param.length = 4;

    error = HANDLE_IOCTL (&inout_param);
    if (A_OK != error)
    {
        return error;
    }
    return A_OK;
}






static A_INT32 set_pmparams(A_INT32 index, A_INT32 argc, char* argv[])
{
    A_UINT32 i;
    ATH_WMI_POWER_PARAMS_CMD pm;
    A_UINT8  iflag = 0, npflag = 0, dpflag = 0, txwpflag = 0, ntxwflag = 0, pspflag = 0;
    ATH_IOCTL_PARAM_STRUCT  inout_param;
    A_INT32 error;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }
	for(i=index; i<argc;i++)
	{
		if (ATH_STRCMP(argv[i],"--idle") == 0)
		{
		    if(i+1 == argc)  //User did not enter parameter
		    {
		    	printf("Missing parameter\n");
		    	return A_ERROR;
		    }
		    pm.idle_period = atoi(argv[i+1]);
		    i++;
			iflag = 1;
		}
		else if(ATH_STRCMP(argv[i], "--np") == 0)
		{
		     if(i+1 == argc)  //User did not enter parameter
		    {
		    	printf("Missing parameter\n");
		    	return A_ERROR;
		    }

			npflag = 1;
			pm.pspoll_number = atoi(argv[i+1]);
			i++;
		}
		else if(ATH_STRCMP(argv[i], "--dp") == 0)
		{
		     if(i+1 == argc)  //User did not enter parameter
		    {
		    	printf("Missing parameter\n");
		    	return A_ERROR;
		    }

			dpflag = 1;
			pm.dtim_policy = atoi(argv[i+1]);
			i++;
		}
		else if(ATH_STRCMP(argv[i], "--txwp") == 0)
		{
		     if(i+1 == argc)  //User did not enter parameter
		    {
		    	printf("Missing parameter\n");
		    	return A_ERROR;
		    }

			txwpflag = 1;
			pm.tx_wakeup_policy = atoi(argv[i+1]);
			i++;
		}
		else if(ATH_STRCMP(argv[i], "--ntxw") == 0)
		{
		     if(i+1 == argc)  //User did not enter parameter
		    {
		    	printf("Missing parameter\n");
		    	return A_ERROR;
		    }

			ntxwflag = 1;
			pm.num_tx_to_wakeup = atoi(argv[i+1]);
			i++;
		}
		else if(ATH_STRCMP(argv[i], "--psfp") == 0)
		{
		     if(i+1 == argc)  //User did not enter parameter
		    {
		    	printf("Missing parameter\n");
		    	return A_ERROR;
		    }

			pspflag = 1;
			pm.ps_fail_event_policy = atoi(argv[i+1]);
			i++;
		}
		else
		{
			printf("Invalid parameter\n");
			return A_ERROR;
		}
	}


    /*Fill in default values if needed*/
	if(!iflag)
	{
		pm.idle_period = 0;
	}

	if(!npflag)
	{
		pm.pspoll_number = 10;
	}

	if(!dpflag)
	{
		pm.dtim_policy = 3;
	}

	if(!txwpflag)
	{
		pm.tx_wakeup_policy = 2;
	}

	if(!ntxwflag)
	{
		pm.num_tx_to_wakeup = 1;
	}

	if(!pspflag)
	{
		pm.ps_fail_event_policy = 2;
	}

    inout_param.cmd_id = ATH_SET_PMPARAMS;
    inout_param.data = &pm;
    inout_param.length = 12;

    error = HANDLE_IOCTL (&inout_param);
    if (A_OK != error)
    {
        return error;
    }
    return A_OK;
}





/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : set_listen_interval
* Returned Value  : A_OK if success else ERROR
*
* Comments        : Sets listen interval
*
*END*-----------------------------------------------------------------*/
static A_INT32 set_listen_interval(char* listen_int)
{

    ATH_IOCTL_PARAM_STRUCT  inout_param;
    A_INT32 error,listen_interval;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }
    inout_param.cmd_id = ATH_SET_LISTEN_INT;
    listen_interval = atoi(listen_int);
    inout_param.data = &listen_interval;
    inout_param.length = 4;


    error = HANDLE_IOCTL (&inout_param);
    if (A_OK != error)
    {
        return error;
    }
    return A_OK;
}





/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : set_phy_mode
* Returned Value  : A_OK if success else ERROR
*
* Comments        : Sets current phy mode
*
*END*-----------------------------------------------------------------*/
static A_INT32 set_phy_mode(char* phy_mode)
{
    ATH_IOCTL_PARAM_STRUCT  inout_param;
    A_INT32 error;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if ((ATH_STRCMP(phy_mode,"n") != 0) &&
        (ATH_STRCMP(phy_mode,"b") != 0) &&
        (ATH_STRCMP(phy_mode,"g") != 0) &&
        (ATH_STRCMP(phy_mode,"ht40") != 0) &&
        (ATH_STRCMP(phy_mode,"a") != 0))
	{
		printf("Invalid Phy mode\n");
		return A_ERROR;
	}

    inout_param.cmd_id = ATH_SET_PHY_MODE;
    inout_param.data = phy_mode;
    inout_param.length = sizeof(phy_mode);


    error = HANDLE_IOCTL (&inout_param);
    if (A_OK != error)
    {
        return error;
    }
    return A_OK;
}











/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : get_phy_mode()
* Returned Value  : A_OK if phy_mode set successfully else ERROR CODE
* Comments        : Gets the phy_mode.
*
*END*-----------------------------------------------------------------*/

A_INT32 get_phy_mode
    (
        A_UINT32 dev_num,
       char* phy_mode
    )
{
    ATH_IOCTL_PARAM_STRUCT inout_param;
    A_INT32 error;
    A_UINT32 value = 0;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    inout_param.cmd_id = ATH_GET_PHY_MODE;
    inout_param.data = &value;

    error = HANDLE_IOCTL (&inout_param);
    if (A_OK != error)
    {
        return error;
    }

    if(*(A_UINT32*)(inout_param.data) == 0x01)
    {
    	strcpy(phy_mode,"a");
    }
    else if(*(A_UINT32*)(inout_param.data) == 0x02)
    {
    	strcpy(phy_mode,"g");
    }
    else if(*(A_UINT32*)(inout_param.data) == 0x04)
    {
    	strcpy(phy_mode,"b");
    }
    else
    {
    	strcpy(phy_mode,"mixed");
    }
    return A_OK;
}


/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : get_mac_
* Returned Value  : A_OK if success else ERROR
*
* Comments        : Gets current firmware mac addr
*
*END*-----------------------------------------------------------------*/
static A_INT32 get_mac_addr(char* mac_addr)
{

    ATH_IOCTL_PARAM_STRUCT inout_param;
    A_INT32 error = A_OK;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }
    inout_param.cmd_id = ATH_GET_MACADDR;
    inout_param.data = mac_addr;
    inout_param.length = 6;

    error = HANDLE_IOCTL (&inout_param);

    return error;
}


/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : get_version()
* Returned Value  : A_OK version is retrieved successfully else ERROR CODE
* Comments        : gets driver,firmware version.
*
*END*-----------------------------------------------------------------*/
A_INT32 get_version()
{
	ATH_IOCTL_PARAM_STRUCT inout_param;
    A_INT32 error;
    versionstr_t versionstr;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    inout_param.cmd_id = ATH_GET_VERSION_STR;
    inout_param.data = &versionstr;
    inout_param.length = sizeof(versionstr);

    error = HANDLE_IOCTL(&inout_param);
    if (A_OK != error)
    {
        return error;
    }

    printf("Host version :  %s\n", versionstr.host_ver);
    printf("Target version   :  %s\n", versionstr.target_ver);
    printf("Firmware version :  %s\n", versionstr.wlan_ver);
    printf("Interface version:  %s\n", versionstr.abi_ver);
    return A_OK;
}


static A_INT32 get_channel_val(char_ptr channel_val)
{
    ATH_IOCTL_PARAM_STRUCT inout_param;
    A_INT32 error = A_OK;

    inout_param.cmd_id = ATH_GET_CHANNEL;
    inout_param.data = channel_val;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }
    error = HANDLE_IOCTL (&inout_param);

    return error;
}

/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : get_last_error()
* Returned Value  : A_OK if phy_mode set successfully else ERROR CODE
* Comments        : Gets the last error in the host driver
*
*END*-----------------------------------------------------------------*/

A_INT32 get_last_error()
{
    ATH_IOCTL_PARAM_STRUCT inout_param;
    A_INT32 error = A_ERROR;
    A_INT32 result;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    inout_param.cmd_id = ATH_GET_LAST_ERROR;
    inout_param.data = &result;
    inout_param.length = 0;

    error = HANDLE_IOCTL (&inout_param);
    if (A_OK != error)
    {
        return error;
    }

    printf("Last Driver error %d\n",(*(A_UINT32*)(inout_param.data)));
    return A_OK;
}


/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : dev_susp_enable()
* Returned Value  : A_OK if device suspend is enabled
*                   successfully else ERROR CODE
* Comments        : Sets the device suspend mode.
*
*END*-----------------------------------------------------------------*/
A_INT32 dev_susp_enable()
{
    ATH_IOCTL_PARAM_STRUCT inout_param;
    A_INT32 error;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }
    inout_param.cmd_id = ATH_DEVICE_SUSPEND_ENABLE;
    inout_param.data = NULL;
    inout_param.length = 0;

    error = HANDLE_IOCTL (&inout_param);
    if (A_OK != error)
    {
        return error;
    }
    return A_OK;
}



/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : dev_susp_start()
* Returned Value  : A_OK if device suspend is started
*                   successfully else ERROR CODE
* Comments        : Suspends device for requested time period
*
*END*-----------------------------------------------------------------*/
A_INT32 dev_susp_start(char* susp_time)
{
	int suspend_time;
    ATH_IOCTL_PARAM_STRUCT inout_param;
    A_INT32 error;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }
    
    if(mode_flag == MODE_AP)
    {
		printf("Store-Recall Mechanism NOT supported in 'MODE_AP' \n");
		return A_ERROR;
	}
	
#if ENABLE_SSL
	if (ssl_inst[SSL_CLIENT_INST].sslCtx != NULL)
	{
		printf("Please disable SSL and start Store-Recall\n");
		return A_ERROR;
	}
#endif

    /*Check if no packets are queued, if TX is pending, then wait*/
    while(get_tx_status() != ATH_TX_STATUS_IDLE){
        ATH_TIME_DELAY_MSEC(500);
        _sched_yield();
    }
    
    suspend_time = atoi(susp_time);

    inout_param.cmd_id = ATH_DEVICE_SUSPEND_START;
    inout_param.data = &suspend_time;
    inout_param.length = 4;



    error = HANDLE_IOCTL (&inout_param);

    if (A_OK != error)
    {
        return error;
    }
    return A_OK;
}





/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : get_rssi()
* Returned Value  : A_OK if rssi is retrieved
*                   successfully else ERROR CODE
* Comments        : Gets RSSI value from driver.
*
*END*-----------------------------------------------------------------*/

A_INT32 get_rssi()
{
    A_INT32 error;
    ATH_IOCTL_PARAM_STRUCT inout_param;
    A_UINT32 rssi=0;
    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    inout_param.cmd_id = ATH_GET_RX_RSSI;
    inout_param.data = &rssi;
    error = ATH_GET_ESSID (DEMOCFG_DEFAULT_DEVICE,ssid);


    if(ssid[0] == '\0')
    {
        printf("Not associated\n");
        return A_OK;
    }


    error = HANDLE_IOCTL( &inout_param);

    if (A_OK != error)
    {
        return error;
    }

    printf("\nindicator = %d dB",rssi);
        printf("\n\r");

    return A_OK;
}







/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : wmi_iwconfig()
* Returned Value  : A_OK if success else ERROR
*
* Comments        : Setup for scan command
*
*END*-----------------------------------------------------------------*/
A_INT32 wmi_iwconfig(A_INT32 argc, char* argv[])
{
	A_UINT8 scan_ssid_flag = 0;			//Flag to decide whether to scan all ssid/channels
	A_UINT8 scan_ssid[MAX_SSID_LENGTH];
	A_INT32 error = A_OK;

	memset(scan_ssid,0,MAX_SSID_LENGTH);

	if(argc < 2)
	{
		printf("Missing Parameters\n");
		return A_ERROR;
	}
    else if (mode_flag == MODE_AP)
    {
       printf("AP mode cannot scan\n");
       return A_ERROR;
    }
	else if(argc > 1)
	{
		if(ATH_STRCMP(argv[1],"scan")!=0)
		{
			printf("Incorrect Parameter\n");
			return A_ERROR;
		}

		if(argc == 3)
		{
			if(strlen(argv[2]) > MAX_SSID_LENGTH)
            {
                printf("SSID length exceeds Maximum value\n");
                return A_ERROR;
            }
			/*Scan specified SSID*/
			scan_ssid_flag = 1;
			strcpy((char*)scan_ssid,argv[2]);
		}
	}
     /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }
	/*Set SSID for scan*/
	if(scan_ssid_flag)
	{
		error = ATH_SET_ESSID (DEMOCFG_DEFAULT_DEVICE,(char*)scan_ssid);
		if(error != A_OK)
		{
			printf("Unable to set SSID\n");
			return error;
		}
	}
	else
	{
		error = ATH_SET_ESSID (DEMOCFG_DEFAULT_DEVICE,"");
		if(error != A_OK)
		{
			printf("Unable to set SSID\n");
			return error;
		}
	}

	/*Do the actual scan*/
	wmi_set_scan(DEMOCFG_DEFAULT_DEVICE);

	/*Revert to original SSID*/

	error = ATH_SET_ESSID (DEMOCFG_DEFAULT_DEVICE,(char*)original_ssid);
	if(error != A_OK)
	{
		printf("Unable to set SSID\n");
		return error;
	}

	return error;
}



#if ENABLE_AP_MODE
static A_INT32 ap_handler( int_32 index, int_32 argc, char_ptr argv[])
{
    uint_32  error= A_OK, i, ret_val;
    char* p = 0;

    if (MODE_AP != mode_flag)
    {
      printf ("Err:Set AP Mode to apply AP settings\n");
      return A_ERROR;
    }

    if (index+1 < argc)
    {
        i = index;

        if (ATH_STRCMP(argv[i],"country") == 0)
        {


            ap_config_param.u.wmi_ap_set_country_cmd.countryCode[0] = argv[i+1][0];
            ap_config_param.u.wmi_ap_set_country_cmd.countryCode[1] = argv[i+1][1];
            ap_config_param.u.wmi_ap_set_country_cmd.countryCode[2] = 0x20;

            if ( ('F' == ap_config_param.u.wmi_ap_set_country_cmd.countryCode[0] && 'F' == ap_config_param.u.wmi_ap_set_country_cmd.countryCode[1]) ||
                 ('f' == ap_config_param.u.wmi_ap_set_country_cmd.countryCode[0] && 'f' == ap_config_param.u.wmi_ap_set_country_cmd.countryCode[1]))
            {
              printf ("AP in WWR Mode\n");
            }

            ret_val = set_ap_params (AP_SUB_CMD_SET_COUNTRY, (A_UINT8 *)ap_config_param.u.wmi_ap_set_country_cmd.countryCode);

            if (ret_val != A_OK)
            {
      			printf ("Err:set country code \n");
      			return A_ERROR;
			}
        }
        else
        if(ATH_STRCMP(argv[i],"inact") == 0)
        {
            ap_config_param.u.wmi_ap_conn_inact_cmd.period = strtoul(argv[i+1], &p,0);

            if(ap_config_param.u.wmi_ap_conn_inact_cmd.period == 0)
            {
     			printf ("incorrect inactive timeout \n");
      			return A_ERROR;
			}

            ret_val = set_ap_params(AP_SUB_CMD_INACT_TIME, (A_UINT8 *)&ap_config_param.u.wmi_ap_conn_inact_cmd.period);

	        if (ret_val != A_OK)
            {
      			printf ("Err:set inactive_time\n");
      			return A_ERROR;
			}
        }
        else
        if(ATH_STRCMP(argv[i],"bconint") == 0)
        {
            ap_config_param.u.wmi_beacon_int_cmd.beaconInterval = strtoul(argv[i+1], &p,0);

            if((ap_config_param.u.wmi_beacon_int_cmd.beaconInterval < 100) || (ap_config_param.u.wmi_beacon_int_cmd.beaconInterval > 1000))
            {
      			printf ("beacon interval has to be within 100-1000 in units of 100 \n");
      			return A_ERROR;
			}

            ret_val = set_ap_params(AP_SUB_CMD_BCON_INT, (A_UINT8 *)&ap_config_param.u.wmi_beacon_int_cmd.beaconInterval);

            if (ret_val != A_OK)
            {
      			printf ("Err:set beacon interval \n");
      			return A_ERROR;
			}
        }
        else
        if(ATH_STRCMP(argv[i],"dtim") == 0)
        {
            ap_config_param.u.wmi_ap_dtim_cmd.dtimval = strtoul(argv[i+1], &p,0);

            if((ap_config_param.u.wmi_ap_dtim_cmd.dtimval < 1) || (ap_config_param.u.wmi_ap_dtim_cmd.dtimval > 255))
            {
      			printf ("dtim interval has to be within 1-255 \n");
      			return A_ERROR;
			}

            ret_val = set_ap_params(AP_SUB_CMD_DTIM_INT, (A_UINT8 *)&ap_config_param.u.wmi_ap_dtim_cmd.dtimval);

            if (ret_val != A_OK)
            {
      			printf ("Err:set dtim interval \n");
      			return A_ERROR;
			}
		}
        else
        if(ATH_STRCMP(argv[i],"psbuf") == 0)
        {
            ap_config_param.u.wmi_ap_ps_buf_cmd.ps_val[0] = strtoul(argv[i+1], &p,0);

            if(ap_config_param.u.wmi_ap_ps_buf_cmd.ps_val[0] == 0)
            {

				ap_config_param.u.wmi_ap_ps_buf_cmd.ps_val[1] = 0x0;

            	ret_val = set_ap_params(AP_SUB_CMD_PSBUF, (A_UINT8 *)ap_config_param.u.wmi_ap_ps_buf_cmd.ps_val);

            	if (ret_val != A_OK)
            	{
      				printf ("Err:set ps buf \n");
      				return A_ERROR;
				}
			}
			else
			{
				ap_config_param.u.wmi_ap_ps_buf_cmd.ps_val[1] = strtoul(argv[i+2], &p,0);

            	ret_val = set_ap_params(AP_SUB_CMD_PSBUF, (A_UINT8 *)ap_config_param.u.wmi_ap_ps_buf_cmd.ps_val);

            	if (ret_val != A_OK)
            	{
      				printf ("Err:set ps buf \n");
      				return A_ERROR;
				}
			}
		}
		else
        {
            printf ("usage:--ap [country <country code>]\n"
                    "      --ap [bconint<intvl>]\n"
                    "      --ap [dtim<val>]\n"
                    "      --ap [inact <minutes>]>\n"
                    "      --ap [psbuf <1/0-enable/disable> <buf_count for enable>]>\n");
        }
    }
    else
    {
            printf ("usage:--ap [country <country code>]\n"
                    "      --ap [bconint<intvl>]\n"
                    "      --ap [dtim<val>]\n"
                    "      --ap [inact <minutes>]>\n"
                    "      --ap [psbuf <1/0-enable/disable> <buf_count for enable>]>\n");
    }

    return error;
}

static A_INT32 set_ap_params(A_UINT16 param_cmd, A_UINT8 *data_val)
{
	ATH_IOCTL_PARAM_STRUCT inout_param;
	ATH_AP_PARAM_STRUCT ap_param;
	uint_32 error;

	inout_param.cmd_id = ATH_CONFIG_AP;
	ap_param.cmd_subset = param_cmd;
	ap_param.data		= data_val;
	inout_param.data	= &ap_param;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    error = HANDLE_IOCTL (&inout_param);
    if (A_OK != error)
    {
        return error;
    }

    return A_OK;
}
#endif

















#if ENABLE_STACK_OFFLOAD



/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : ipconfig_query()
* Returned Value  : A_OK if success else A_ERROR
*
* Comments        : Sample function to depict ipconfig functionality.
*                   Queries IP parameters from target.
*
*END*-----------------------------------------------------------------*/
A_INT32 ipconfig_query(A_INT32 argc, char* argv[])
{
    A_UINT32 addr = 0, mask = 0, gw = 0;
    IP6_ADDR_T v6Global;
    IP6_ADDR_T v6GlobalExtd;
    IP6_ADDR_T v6LinkLocal;
    IP6_ADDR_T v6DefGw;
    char ip6buf[48];
    char mac_data[6];
    char *ip6ptr =  NULL;
    A_INT32 LinkPrefix = 0;
    A_INT32 GlobalPrefix = 0;
    A_INT32 DefGwPrefix = 0;
    A_INT32 GlobalPrefixExtd = 0;
    IP46ADDR dnsaddr[MAX_DNSADDRS];
    int i = 0;
    A_CHAR hostname[33];
    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }
    get_mac_addr((char*)mac_data);
    printf("\n mac addr    =   %02x:%02x:%02x:%02x:%02x:%02x \n", mac_data[0], mac_data[1], mac_data[2],
                                                      mac_data[3], mac_data[4], mac_data[5]);

        memset(hostname,0,33);
	t_ipconfig((void*)handle,IPCFG_QUERY, &addr, &mask, &gw,dnsaddr,hostname);
        printf("IP:%x Mask:%x, Gateway:%x\n", addr, mask, gw);
	printf("IP:%d.%d.%d.%d Mask:%d.%d.%d.%d, Gateway:%d.%d.%d.%d\n",
				getByte(3, addr), getByte(2, addr), getByte(1, addr), getByte(0, addr),
				getByte(3, mask), getByte(2, mask), getByte(1, mask), getByte(0, mask),
				getByte(3, gw), getByte(2, gw), getByte(1, gw), getByte(0, gw));

	for(i = 0;  i < MAX_DNSADDRS;i++)
	{
           if(dnsaddr[i].type == ATH_AF_INET)
           {
              addr = A_CPU2LE32(dnsaddr[i].addr4);  		   
	      printf("DNS Server:%d.%d.%d.%d \n", getByte(0, addr), getByte(1,addr), 
			                          getByte(2,addr), getByte(3, addr));
           } 		   
	}	
        if(hostname[0] != 0)
	printf("HostName: %s \n",hostname);       	
	memset(&v6LinkLocal,0, sizeof(IP6_ADDR_T));
        memset(&v6Global,0, sizeof(IP6_ADDR_T));
        memset(&v6DefGw,0, sizeof(IP6_ADDR_T));
	memset(&v6GlobalExtd,0, sizeof(IP6_ADDR_T));
	t_ip6config((void *)handle, IPCFG_QUERY, &v6Global,&v6LinkLocal,&v6DefGw,&v6GlobalExtd,&LinkPrefix,&GlobalPrefix,&DefGwPrefix,&GlobalPrefixExtd);

        if(v6LinkLocal.addr[0] )
	{
           ip6ptr = print_ip6(&v6LinkLocal,ip6buf);
	   if(ip6ptr)
           {
             if(LinkPrefix)
             printf("Link-local IPv6 Address ..... : %s/%d \n", ip6ptr,LinkPrefix);
	     else
             printf("Link-local IPv6 Address ..... : %s \n", ip6ptr);
	   }
           ip6ptr = NULL;
           ip6ptr = print_ip6(&v6Global,ip6buf);
           if(ip6ptr)
           {
              if(GlobalPrefix)
	      printf("Global IPv6 Address ......... : %s/%d \n", ip6ptr,GlobalPrefix);
	      else
	      printf("Global IPv6 Address ......... : %s \n", ip6ptr);
	    }
	    ip6ptr = NULL;
            ip6ptr = print_ip6(&v6DefGw, ip6buf);
	    if(ip6ptr)
	    {
	      if(DefGwPrefix)
	      printf("Default Gateway  ............. : %s/%d \n", ip6ptr,DefGwPrefix);
	      else
	      printf("Default Gateway  ............. : %s \n", ip6ptr);

	    }
            ip6ptr = NULL;
            ip6ptr = print_ip6(&v6GlobalExtd,ip6buf);
            if(ip6ptr)
            {
              if(GlobalPrefixExtd)
	      printf("Global IPv6 Address 2 ......... : %s/%d \n", ip6ptr,GlobalPrefixExtd);
	      else
	      printf("Global IPv6 Address 2 ......... : %s \n", ip6ptr);
	    }

	}
	for(i = 0;  i < MAX_DNSADDRS;i++)
	{
           ip6ptr = NULL;
           if(dnsaddr[i].type == ATH_AF_INET6)
           {
             ip6ptr = print_ip6(&dnsaddr[i].addr6,ip6buf);
	     printf("DNS Server ......... : %s \n", ip6ptr);
           } 		  
	}   
     	return 0; 
}


/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : ipconfig_static()
* Returned Value  : A_OK if success else A_ERROR
*
* Comments        : Sample function to depict ipconfig functionality.
*                   Sets static IPv4 parameters.
*
*END*-----------------------------------------------------------------*/
A_INT32 ipconfig_static(A_INT32 argc, char* argv[])
{
	A_UINT32 addr, mask, gw;
	char* ip_addr_string;
	char* mask_string;
	char* gw_string;
        A_INT32 error;
	unsigned int sbits;

        /*Check if driver is loaded*/
        if(IS_DRIVER_READY != A_OK){
            return A_ERROR;
        }
	if(argc < 5)
	{
		printf("incomplete params\n");
		return A_ERROR;
	}
	ip_addr_string = argv[2];
	mask_string = argv[3];
	gw_string = argv[4];

	error=parse_ipv4_ad(&addr, &sbits, ip_addr_string);
	if(error!=0)
	{
		printf("Invalid IP address\n");
		return error;
	}
	addr = A_BE2CPU32(addr);
	error=parse_ipv4_ad(&mask, &sbits, mask_string);
	if(error!=0)
	{
		printf("Invalid Mask address\n");
		return error;
	}
	mask = A_BE2CPU32(mask);
	error=parse_ipv4_ad(&gw, &sbits, gw_string);
	if(error!=0)
	{
		printf("Invalid Gateway address\n");
		return error;
	}
	gw = A_BE2CPU32(gw);
	t_ipconfig((void*)handle,IPCFG_STATIC, &addr, &mask, &gw,NULL,NULL);
	printf("IP:%x Mask:%x, Gateway:%x\n", addr, mask, gw);
	printf("IP:%d.%d.%d.%d Mask:%d.%d.%d.%d, Gateway:%d.%d.%d.%d\n",
				getByte(3, addr), getByte(2, addr), getByte(1, addr), getByte(0, addr),
				getByte(3, mask), getByte(2, mask), getByte(1, mask), getByte(0, mask),
				getByte(3, gw), getByte(2, gw), getByte(1, gw), getByte(0, gw));


	return 0;
}

void configure_auto_ipv4_addr()
{
	A_UINT32 addr, mask, gw;
	char* ip_addr_string;
	char* mask_string;
	char* gw_string;
        A_INT32 error;
	unsigned int sbits;

	ip_addr_string = "169.254.1.1";
	mask_string = "255.255.0.0";
	gw_string = "0.0.0.0";

	error=parse_ipv4_ad(&addr, &sbits, ip_addr_string);
	if(error!=0)
	{
		printf("Invalid IP address\n");
		return; 
	}
	addr = A_BE2CPU32(addr);
	error=parse_ipv4_ad(&mask, &sbits, mask_string);
	if(error!=0)
	{
		printf("Invalid Mask address\n");
		return;
	}
	mask = A_BE2CPU32(mask);
	error=parse_ipv4_ad(&gw, &sbits, gw_string);
	if(error!=0)
	{
		printf("Invalid Gateway address\n");
		return;
	}
	gw = A_BE2CPU32(gw);
	t_ipconfig((void*)handle,IPCFG_STATIC, &addr, &mask, &gw,NULL,NULL);
	printf("Acquired IP address via AutoIPv4\r\n");
	printf("IP:%x Mask:%x, Gateway:%x\n", addr, mask, gw);
	printf("IP:%d.%d.%d.%d Mask:%d.%d.%d.%d, Gateway:%d.%d.%d.%d\n",
				getByte(3, addr), getByte(2, addr), getByte(1, addr), getByte(0, addr),
				getByte(3, mask), getByte(2, mask), getByte(1, mask), getByte(0, mask),
				getByte(3, gw), getByte(2, gw), getByte(1, gw), getByte(0, gw));


	return ;
}


/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : ipconfig_dhcp()
* Returned Value  : A_OK if success else A_ERROR
*
* Comments        : Sample function to depict ipconfig functionality.
*                   Runs DHCP on target (only IPv4).
*
*END*-----------------------------------------------------------------*/
A_INT32 ipconfig_dhcp(A_INT32 argc, char* argv[])
{
    A_UINT32 addr, mask, gw;
    IP46ADDR dnsaddr[MAX_DNSADDRS];
    int err = 0;
    A_CHAR hostname[33];

    if(wifi_connected_flag == 0)
    {
      printf("ERROR: No WiFi connection available, please connect to an Access Point\n");
      return A_ERROR;
    }	    
    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }
    err = t_ipconfig((void*)handle,IPCFG_DHCP, &addr, &mask, &gw,NULL,NULL);
    if(err == -1 )
    {
       printf("dhcp timed out. Configuring auto ip \r\n");
       configure_auto_ipv4_addr();
       return 0;
    }	    
    addr = 0;
    t_ipconfig((void*)handle,IPCFG_QUERY, &addr, &mask, &gw,dnsaddr,hostname);
    if(addr != 0){
	printf("IP:%d.%d.%d.%d Mask:%d.%d.%d.%d, Gateway:%d.%d.%d.%d\n",
				getByte(3, addr), getByte(2, addr), getByte(1, addr), getByte(0, addr),
				getByte(3, mask), getByte(2, mask), getByte(1, mask), getByte(0, mask),
				getByte(3, gw), getByte(2, gw), getByte(1, gw), getByte(0, gw));
    }
    return 0;
}

#if 0
A_INT32 block_on_dhcp()
{
    A_UINT32 addr=0, mask=0, gw=0;
    IP46ADDR dnsaddr[MAX_DNSADDRS];
    A_CHAR hostname[33];  

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    /*Start DHCP*/
    t_ipconfig((void*)handle,IPCFG_DHCP, &addr, &mask, &gw,NULL,NULL);

    /*Wait till we get an IP address, instead of indefinte loop, develop may
      use a timeout logic here*/
    while(addr == 0){
      t_ipconfig((void*)handle,IPCFG_QUERY, &addr, &mask, &gw,dnsaddr,hostname);
      A_MDELAY(100);
    }
    printf("addr 0x%x\n",addr);
    return A_OK;
}
#endif
/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : ipconfig_dhcp_pool()
* Returned Value  : A_OK if success else A_ERROR
*
* Comments        : Sample function to configure dhcp pool.
*
*END*-----------------------------------------------------------------*/
A_INT32 ipconfig_dhcp_pool(A_INT32 argc, char* argv[])
{
	A_UINT32 startaddr, endaddr;
	char* start_ip_addr_string;
	char* end_ip_addr_string;
        A_INT32 error;
	unsigned int sbits;
	int leasetime;
        A_UINT32 addr=0, mask=0, gw=0;
	A_CHAR hostname[33];

        /*Check if driver is loaded*/
        if(IS_DRIVER_READY != A_OK){
            return A_ERROR;
        }
	if(argc < 5)
	{
		printf("incomplete params\n");
		return A_ERROR;
	}
	start_ip_addr_string = argv[2];
	end_ip_addr_string = argv[3];
	leasetime = atoi(argv[4]);

	error=parse_ipv4_ad(&startaddr, &sbits, start_ip_addr_string);
	if(error!=0)
	{
		printf("Invalid start IP address\n");
		return error;
	}
	startaddr = A_BE2CPU32(startaddr);
	error=parse_ipv4_ad(&endaddr, &sbits, end_ip_addr_string);
	if(error!=0)
	{
		printf("Invalid end ip address\n");
		return error;
	}
	endaddr = A_BE2CPU32(endaddr);
        t_ipconfig((void*)handle,IPCFG_QUERY, &addr, &mask, &gw,NULL,hostname);
	if((addr & mask) != (startaddr & mask))
        {
            printf("LAN IP address and start ip address are not in the same subnet \n");
            return error;
        }
	if((addr & mask) != (endaddr & mask))
        {
            printf("LAN IP address and end ip address are not in the same subnet \n");
            return error;
        }
	if((addr  >= startaddr) && (addr  <= endaddr))
	{
            printf("Please configure pool beyond LAN IP Address \n");
            return error;
        }
	if(startaddr >= endaddr)
        {
            printf("Staraddr must be less than end addr \n");
            return error;
        }
	t_ipconfig_dhcp_pool((void*)handle,&startaddr, &endaddr, leasetime);
	return 0;
}

A_INT32 ip6_set_router_prefix(A_INT32 argc,char *argv[])
{

	unsigned char  v6addr[16];
	A_INT32 retval=-1;
	int prefixlen = 0;
        int prefix_lifetime = 0;
        int valid_lifetime = 0;

        /*Check if driver is loaded*/
        if(IS_DRIVER_READY != A_OK){
            return A_ERROR;
        }
	if(argc < 6)
	{
		printf("incomplete params\n");
		return A_ERROR;
	}
    if(mode_flag == MODE_STATION)
    {
		printf("ipv6 rt prfx support not possible in station \n");
		return A_ERROR;
	}

    retval = Inet6Pton(argv[2],v6addr);
    if(retval == 1)
	{
           printf("Invalid ipv6 prefix \n");
           return (A_ERROR);
	}
	prefixlen =  atoi(argv[3]);
        prefix_lifetime = atoi(argv[4]);
        valid_lifetime =  atoi(argv[5]);
        t_ip6config_router_prefix((void *)handle,v6addr,prefixlen,prefix_lifetime,valid_lifetime);
        return 0;
}

static A_INT32 ip_set_tcp_exponential_backoff_retry(A_INT32 argc,char *argv[])
{

    A_INT32 retry = 0;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
          return A_ERROR;
    }
    if(argc < 2)
    {
       printf("incomplete params\n");
       return A_ERROR;
    }
    retry=atoi(argv[2]);
    if((retry > 12) || (retry < 4)) 
    {
        printf("Confiure TCP Max retries between 4 and 12");
	return (A_ERROR);	
     }	    
    custom_ipconfig_set_tcp_exponential_backoff_retry((void *)handle,retry);
    return A_OK;
}

static A_INT32 ip6_set_status(A_INT32 argc,char *argv[])
{

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
          return A_ERROR;
    }
    if(argc < 2)
    {
       printf("incomplete params\n");
       return A_ERROR;
    }

    if(ATH_STRCMP(argv[2],"enable") == 0)
    {
      custom_ipconfig_set_ip6_status((void *)handle,1);	    
      return A_OK;
    }	    
    else if(ATH_STRCMP(argv[2],"disable") == 0)
    {
       custom_ipconfig_set_ip6_status((void *)handle,0);
       return A_OK; 
    }	    
    printf("Allowed values are enable and disable only");
    return A_OK;
}

static A_INT32 ipconfig_dhcp_release(A_INT32 argc,char *argv[])
{

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }
    custom_ipconfig_dhcp_release((void*)handle);
    return 0;
}

static A_INT32 ipconfig_tcp_rxbuf(A_INT32 argc,char *argv[])
{
     A_INT32 rxbuf = 0;	
    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if(argc < 2)
    {
       printf("incomplete params\n");
       return A_ERROR;
    }
    rxbuf=atoi(argv[2]);
    if(rxbuf > 10) 
    {
        printf("Max Allowed RX Buffer is 10");
	return (A_ERROR);	
    }	    
    custom_ipconfig_set_tcp_rx_buffer((void *)handle,rxbuf);
    return A_OK;
}

#if ENABLE_DNS_CLIENT
static A_INT32 ip_dns_client(A_INT32 argc,char *argv[])
{

    A_INT32 command = -1;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if(argc < 3)
    {
       printf("Incomplete params\n");
       return A_ERROR;
    }
    if(ATH_STRCMP(argv[2], "start") == 0) 
    {
        command = 1;
    }
    else if(ATH_STRCMP(argv[2], "stop") == 0)
    {
        command = 0;
    }
    else
    {
        printf("Supported commands: start, stop");
        return (A_ERROR);	
    }	    
    if (A_OK != custom_ip_dns_client((void *)handle, command))
    {
        printf("Error! DNS Client %s failed\n", command?"start":"stop");
    }
    return A_OK;

}

static A_INT32 ip_dns_server_addr(A_INT32 argc,char *argv[])
{
    A_UINT32 addr = 0;
    char * ip_addr_string = NULL;
    A_INT32 error;
    unsigned int sbits;
    IP46ADDR dnsaddr;
    A_INT32 retval = -1;
    unsigned char  v6addr[16];


    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if(argc < 3)
    {
       printf("Incomplete params\n");
       return A_ERROR;
    }

    memset(&dnsaddr,0,sizeof(IP46ADDR));
    ip_addr_string = argv[2];

    error=parse_ipv4_ad(&addr, &sbits, ip_addr_string);
    if(error == 0)
    {
        dnsaddr.type = ATH_AF_INET;
	dnsaddr.addr4 = addr;
        if (A_OK != custom_ip_dns_server_addr((void *)handle, &dnsaddr))
        {
          printf("Error! DNS Server address conf failed \r\n");
	  return (A_ERROR);
        }
	return A_OK;
    }	   

    retval = Inet6Pton(argv[2],v6addr);
    if(retval == 1)
    {
        printf("Invalid ipv6 prefix \n");
        return (A_ERROR);
    }
    dnsaddr.type = ATH_AF_INET6;
    memcpy(&dnsaddr.addr6,&v6addr,sizeof(IP6_ADDR_T));
    if (A_OK != custom_ip_dns_server_addr((void *)handle, &dnsaddr))
    {
        printf("Error! DNS Server address conf failed \r\n");
        return (A_ERROR);
    }
    return A_OK;
}

static A_INT32 ip_dns_delete_server_addr(A_INT32 argc,char *argv[])
{
    A_UINT32 addr = 0;
    char * ip_addr_string = NULL;
    A_INT32 error;
    unsigned int sbits;
    IP46ADDR dnsaddr;
    A_INT32 retval = -1;
    unsigned char  v6addr[16];

#define DELETE_DNS_SERVER_ADDR 1

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if(argc < 3)
    {
       printf("Incomplete params\n");
       return A_ERROR;
    }
    memset(&dnsaddr,0,sizeof(IP46ADDR)); 
    ip_addr_string = argv[2];

    error=parse_ipv4_ad(&addr, &sbits, ip_addr_string);
    if(error == 0)
    {
        dnsaddr.type = ATH_AF_INET;
	dnsaddr.addr4 = addr;
	dnsaddr.au1Rsvd[0] = DELETE_DNS_SERVER_ADDR;  
        if (A_OK != custom_ip_dns_server_addr((void *)handle, &dnsaddr))
        {
          printf("Error! DNS Server address conf failed \r\n");
	  return (A_ERROR);
        }
	return A_OK;
    }	   

    retval = Inet6Pton(argv[2],v6addr);
    if(retval == 1)
    {
        printf("Invalid ipv6 prefix \n");
        return (A_ERROR);
    }
    dnsaddr.type = ATH_AF_INET6;
    memcpy(&dnsaddr.addr6,&v6addr,sizeof(IP6_ADDR_T));
    dnsaddr.au1Rsvd[0] = DELETE_DNS_SERVER_ADDR;  
    if (A_OK != custom_ip_dns_server_addr((void *)handle, &dnsaddr))
    {
        printf("Error! DNS Server address conf failed \r\n");
        return (A_ERROR);
    }
    return A_OK;
}
#endif /* ENABLE_DNS_CLIENT*/


#if ENABLE_HTTP_SERVER
static A_INT32 ip_http_server(A_INT32 argc,char *argv[])
{
    A_INT32 command = -1;
    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }
	    
    if(argc < 2)
    {
       printf("Incomplete params\n");
       return A_ERROR;
    }
    if(ATH_STRCMP(argv[2], "start") == 0) 
    {
        command = 1;
    }
    else if(ATH_STRCMP(argv[2], "stop") == 0)
    {
        command = 0;
    }
    else
    {
        printf("Supported commands: start, stop");
        return (A_ERROR);	
    }	    
    if (A_OK != custom_ip_http_server((void *)handle, command))
        printf("Error! HTTP Server %s\n", command?"start failed":"not running");
    else
        printf("HTTP server %s success\n",command?"start":"stop");
    return A_OK;
}

static A_INT32 ip_http_server_post(A_INT32 argc,char *argv[])
{
    A_INT32 command = 0; /* Command 0 for HTTP server post */
    A_INT32 objtype, objlen;
	    
    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if(argc < 7)
    {
       printf("Incomplete params\n");
       return A_ERROR;
    }
    
    objtype = atoi(argv[4]);
    objlen = atoi(argv[5]);
    if((objtype != 1) && (objtype != 2) && (objtype != 3))  
    {
        printf("Error: <objtype> can be 1 ,2 or 3\n");
        return A_ERROR;
    }
	    
    if (A_OK != custom_ip_http_server_method((void *)handle, command, argv[2], argv[3], objtype, objlen, argv[6]))
    {
        printf("Error! HTTP post failed\n");
    }
    return A_OK;
}

static A_INT32 ip_http_server_get(A_INT32 argc,char *argv[])
{
    A_INT32 command = 1; /* Command 1 for HTTP server get */
    A_INT32 objtype, objlen;
    A_UINT8 *value;
	    
    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if(argc < 4)
    {
       printf("Incomplete params\n");
       return A_ERROR;
    }
    objtype = 0;
    objlen = 0;
#if 0
    if((objtype != 1) && (objtype != 2) && (objtype != 3))  
    {
        printf("Error: <objtype> can be 1 ,2 or 3\n");
        return A_ERROR;
    }
#endif
    value = A_MALLOC(512, MALLOC_ID_TEMPORARY);
    if (!value)
        return A_ERROR;
	    
    if (A_OK != custom_ip_http_server_method((void *)handle, command, argv[2], argv[3], objtype, objlen, value))
    {
        printf("Error! HTTP post failed\n");
    }
    else
    {
        printf("Value is %s\n", value);
    }
    A_FREE(value, MALLOC_ID_TEMPORARY);
    return A_OK;
}

#endif // ENABLE_HTTP_SERVER

#if ENABLE_DNS_CLIENT
static A_INT32 ip_resolve_hostname(A_INT32 argc,char *argv[])
{

    DNC_CFG_CMD DnsCfg;
    DNC_RESP_INFO  DnsRespInfo;     
    A_UINT32 addr;
    char ip6buf[48];
    char *ip6ptr =  NULL;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if(argc < 4)
    {
       printf("Incomplete params\n");
       return A_ERROR;
    }
    if(strlen(argv[2]) > 32) 
    {
        printf("host name cannot be more then 32 Bytes\n");
        return A_ERROR;
    }
    strcpy(DnsCfg.ahostname,argv[2]);
    DnsCfg.domain = atoi(argv[3]);
    DnsCfg.mode =  RESOLVEHOSTNAME;
    if (A_OK != custom_ip_resolve_hostname((void *)handle, &DnsCfg,&DnsRespInfo))
    {
        printf("Unable to resolve host name\r\n");
        return A_ERROR;
    }
    if(DnsCfg.domain == 2)
    {	    
       addr = A_CPU2BE32(DnsRespInfo.ipaddrs_list[0]);
       printf("resolved IP:%d.%d.%d.%d \r\n",getByte(3, addr), getByte(2, addr), getByte(1, addr), getByte(0, addr));
    }
    if(DnsCfg.domain == 3)
    {
      ip6ptr = print_ip6(&DnsRespInfo.ip6addrs_list[0],ip6buf);
      printf("Resolved IP6Addr ......... : %s \n", ip6ptr);
    }	    
    return A_OK;
}

A_INT32 ip_get_addr_for_host_name(char *hostname,A_UINT32 *v4addr,IP6_ADDR_T *ip6addr,A_UINT32 domain)
{
    DNC_CFG_CMD DnsCfg;
    DNC_RESP_INFO  DnsRespInfo;     
    A_UINT32 addr = 0;
    char ip6buf[48];
    char *ip6ptr =  NULL;
   
    /*Check if driver is loaded*/
    strcpy(DnsCfg.ahostname,hostname);
    DnsCfg.domain = domain;
    DnsCfg.mode =  RESOLVEHOSTNAME;
    if (A_OK != custom_ip_resolve_hostname((void *)handle, &DnsCfg,&DnsRespInfo))
    {
        printf("Unable to resolve host name\r\n");
        return A_ERROR;
    }
    if(DnsCfg.domain == 2)
    {	    
       addr = A_CPU2BE32(DnsRespInfo.ipaddrs_list[0]);
       printf("resolved IP:%d.%d.%d.%d \r\n",getByte(3, addr), getByte(2, addr), 
              getByte(1, addr), getByte(0, addr));
       *v4addr = DnsRespInfo.ipaddrs_list[0];
    }
    if(DnsCfg.domain == 3)
    {
      ip6ptr = print_ip6(&DnsRespInfo.ip6addrs_list[0],ip6buf);
      printf("Resolved IP6Addr ......... : %s \n", ip6ptr);
      memcpy(ip6addr,&DnsRespInfo.ip6addrs_list[0],16);
    }	    
    return A_OK;
}

static A_INT32 ip_gethostbyname(A_INT32 argc,char *argv[])
{

    DNC_CFG_CMD DnsCfg;
    DNC_RESP_INFO  DnsRespInfo;     
    A_UINT32 addr;

    memset(&DnsRespInfo,0,sizeof(DnsRespInfo));
    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if(argc < 3)
    {
       printf("Incomplete params\n");
       return A_ERROR;
    }
    if(strlen(argv[2]) > 32) 
    {
        printf("host name cannot be more then 32 Bytes\n");
        return A_ERROR;
    }
    strcpy(DnsCfg.ahostname,argv[2]);
    DnsCfg.domain = 2;
    DnsCfg.mode =  GETHOSTBYNAME;
    if (A_OK != custom_ip_resolve_hostname((void *)handle, &DnsCfg,&DnsRespInfo))
    {
        printf("Unable to resolve host name\r\n");
        return A_ERROR;
    }
    if(DnsRespInfo.dns_names[0] != 0)
    {
       printf("host: name=%s\r\n",DnsRespInfo.dns_names);
    } 	    
    printf("addrtype %d \r\n", A_CPU2LE32(DnsRespInfo.h_addrtype));
    printf("length %d \r\n", A_CPU2LE32(DnsRespInfo.h_length));
    addr = A_CPU2BE32(DnsRespInfo.ipaddrs_list[0]);
    printf("addr:%d.%d.%d.%d \r\n",getByte(3, addr), getByte(2, addr), getByte(1, addr), getByte(0, addr));
    return A_OK;
}
static A_INT32 ip_gethostbyname2(A_INT32 argc,char *argv[])
{

    DNC_CFG_CMD DnsCfg;
    DNC_RESP_INFO  DnsRespInfo;     
    A_UINT32 addr;
    char ip6buf[48];
    char *ip6ptr =  NULL;

    memset(&DnsRespInfo,0,sizeof(DnsRespInfo));
    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if(argc < 4)
    {
       printf("Incomplete params\n");
       return A_ERROR;
    }
    if(strlen(argv[2]) > 32) 
    {
        printf("host name cannot be more then 32 Bytes\n");
        return A_ERROR;
    }
    strcpy(DnsCfg.ahostname,argv[2]);
    DnsCfg.domain = atoi(argv[3]);
    DnsCfg.mode =  GETHOSTBYNAME2;
    if (A_OK != custom_ip_resolve_hostname((void *)handle, &DnsCfg,&DnsRespInfo))
    {
        printf("Unable to resolve host name\r\n");
        return A_ERROR;
    }
    if(DnsRespInfo.dns_names[0] != 0)
    {
       printf("host: name=%s\r\n",DnsRespInfo.dns_names);
    } 	    
    printf("addrtype %d \r\n", A_CPU2LE32(DnsRespInfo.h_addrtype));
    printf("length %d \r\n",A_CPU2LE32(DnsRespInfo.h_length));
    if(DnsCfg.domain == 2) 
    { 	    
      addr = A_CPU2BE32(DnsRespInfo.ipaddrs_list[0]);
      printf("addr:%d.%d.%d.%d \r\n",getByte(3, addr), getByte(2, addr), getByte(1, addr), getByte(0, addr));
    }
    else if(DnsCfg.domain == 3)
    {
      ip6ptr = print_ip6(&DnsRespInfo.ip6addrs_list[0],ip6buf);
      printf("Resolved IP6Addr ......... : %s \n", ip6ptr);
    } 	    
    return A_OK;
}
#endif /* ENABLE_DNS_CLIENT */

static A_INT32 iphostname(A_INT32 argc,char *argv[])
{

    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if(argc < 3)
    {
       printf("Incomplete params\n");
       return A_ERROR;
    }
    if(strlen(argv[2]) > 32) 
    {
        printf("domain name cannot be more then 32 Bytes\n");
        return A_ERROR;
    }
    if (A_OK != custom_ip_hostname((void *)handle, argv[2]))
    {
        printf("Release DHCP, Configure the hostname and renew DHCP\r\n");
        return A_ERROR;
    }
    return A_OK;
}

#if ENABLE_DNS_SERVER
static A_INT32 ip_dns_local_domain(A_INT32 argc,char *argv[])
{

    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if(argc < 3)
    {
       printf("Incomplete params\n");
       return A_ERROR;
    }
    if(strlen(argv[2]) > 32) 
    {
        printf("domain name cannot be more then 32 Bytes\n");
        return A_ERROR;
    }
    if (A_OK != custom_ip_dns_local_domain((void *)handle, argv[2]))
    {
        printf("Unable to configure local domain name\r\n");
        return A_ERROR;
    }
    return A_OK;
}

static A_INT32 ipdns(A_INT32 argc,char *argv[])
{
    A_UINT32 addr = 0;
    char * ip_addr_string = NULL;
    A_INT32 error;
    unsigned int sbits;
    IP46ADDR dnsaddr;
    A_INT32 retval = -1;
    unsigned char  v6addr[16];
    A_INT32 command = 0;

    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if(argc < 5)
    {
       printf("Incomplete params\n");
       return A_ERROR;
    }
    if(strcmp(argv[2] ,"add") == 0) 
    {
      command =1;
    }
    if(strcmp(argv[2],"delete") == 0) 
    {
      command = 2;
    }	    
    if(strlen(argv[3]) > 32) 
    {
        printf("domain name cannot be more then 32 Bytes\n");
        return A_ERROR;
    }
    memset(&dnsaddr,0,sizeof(IP46ADDR));
    ip_addr_string = argv[4];
    error=parse_ipv4_ad(&addr, &sbits, ip_addr_string);
    if(error == 0)
    {
        //printf("add is v4 %x \r\n",addr);
        dnsaddr.type = ATH_AF_INET;
	dnsaddr.addr4 = addr;
        if (A_OK != custom_ipdns((void *)handle,command,argv[3], &dnsaddr))
        {
          printf("Error! DNS database conf failed \r\n");
	  return (A_ERROR);
        }
	return A_OK;
    }	   
    retval = Inet6Pton(argv[4],v6addr);
    if(retval == 1)
    {
        printf("Invalid ipv6 prefix \n");
        return (A_ERROR);
    }
    dnsaddr.type = ATH_AF_INET6;
    memcpy(&dnsaddr.addr6,&v6addr,sizeof(IP6_ADDR_T));
    if (A_OK != custom_ipdns((void *)handle,command,argv[3],&dnsaddr))
    {
        return A_ERROR;
    }
    return A_OK;
}
#endif /* ENABLE_DNS_SERVER */

#if ENABLE_SNTP_CLIENT
static A_INT32 ip_sntp_srvr(A_INT32 argc,char *argv[])
{
    A_INT32 command = 0;

    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if(argc < 4)
    {
       printf("Incomplete params\n");
       return A_ERROR;
    }
    if(strcmp(argv[2] ,"add") == 0) 
    {
      command =1;
    }
    if(strcmp(argv[2],"delete") == 0) 
    {
      command = 2;
    }
    if(strlen(argv[3]) > 64) 
    {
        printf("domain name or address cannot be more then 64 Bytes\n");
        return A_ERROR;
    }
    
    if (A_OK != custom_ip_sntp_srvr_addr((void *)handle, command,argv[3]))
    {
        printf("Unable to configure sntp server address\r\n");
        return A_ERROR;
    }
    return A_OK;
}

static A_INT32 ip_sntp_get_time(A_INT32 argc,char *argv[])
{
    tSntpTime SntpTime ;
    char *months[12] = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
    char *Day[7] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
     A_UINT8 m[4];

    memset(&SntpTime,0,sizeof(tSntpTime));

    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if (A_OK != custom_ip_sntp_get_time((void *)handle, &SntpTime))
    {
        printf("Unable to get SNTP Time\r\n");
        return A_ERROR;
    }

    memset(m,0,4); 
    memcpy(m,&(months[SntpTime.mon][0]),3); 
    printf("TimeStamp: %s %s %d, %d %d:%d:%d\r\n",Day[SntpTime.wday],m,SntpTime.yday,SntpTime.year,SntpTime.hour,SntpTime.min,SntpTime.Sec);
    return A_OK;
}

static A_INT32 ip_sntp_get_time_of_day(A_INT32 argc,char *argv[])
{
    tSntpTM SntpTm;

    memset(&SntpTm,0,sizeof(tSntpTM));

    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if (A_OK != custom_ip_sntp_get_time_of_day((void *)handle, &SntpTm))
    {
        printf("Unable to get SNTP Time of day\r\n");
        return A_ERROR;
    }

    printf("sntp time of day done. \n\r Seconds = %d \r\n",SntpTm.tv_sec);
    return A_OK;
}

static A_INT32 ip_sntp_modify_timezone_dse(A_INT32 argc, char *argv[])
{
    tSntpTime SntpTime ;
    A_UINT8 hour,min;
    A_UINT8 dse,add_sub;
    char hr[3],mn[3],parsing_hour_min[10];
    
    memset(&SntpTime,0,sizeof(tSntpTime));
    
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if(argc < 5)
    {
       printf("Incomplete params\n");
       return A_ERROR;
    }

    if(strlen(argv[2]) > 9)
     {
       printf("Error : Invalid UTC string. Valid string(UTC+hour:min).Eg UTC+05:30\n\r");
       return A_ERROR;
     }
 
    if(strlen(argv[3]) > 3)
    {
	    printf("Error : Invalid string\n");
	    return A_ERROR;
    }
    if(strlen(argv[4]) > 7)
     {
        printf("Error : Invalid DSE string.Valid string DSE\n\r");
        return A_ERROR;
     }
 
     strcpy(parsing_hour_min,argv[2]);
 
     if(parsing_hour_min[3] == '+')
     {
                add_sub = 1; // time is to be added
     }
     else if(parsing_hour_min[3] == '-')
     {
                add_sub = 0; // time is to be subtracted
     }
     else{
                printf("Error : Only + / - operation is allowed\n\r");
                return A_ERROR;
      }
 
 
     hr[0]=parsing_hour_min[4];
     hr[1]=parsing_hour_min[5];
     hr[2]='\0';
 
     hour=strtol(hr,NULL,10);
 
     mn[0]=parsing_hour_min[7];
     mn[1]=parsing_hour_min[8];
     mn[2]='\0';
 
     min=strtol(mn,NULL,10);
 
     if(ATH_STRCMP(argv[4],"enable") == 0)
     {
               // printf("Enabling dse\n\r");
                dse = 1;
     }
     else if(ATH_STRCMP(argv[4],"disable") == 0){
               // printf("disabling dse\n\r");
                dse = 0;
     }
     else{
                printf("Invalid entry for DSE\n\r");
                return A_ERROR;
     }

    
    if (A_OK != custom_ip_sntp_modify_zone_dse((void *)handle,hour,min,add_sub,dse))
    {
        printf("Unable to set the modified time stamp\r\n");
        return A_ERROR;
    }
    
    
    return A_OK;
}


static A_INT32 ip_query_sntp_server_address(A_UINT32 argc,char *argv[])
{
    A_UINT8 i = 0;	
    tSntpDnsAddr SntpDnsAddr[MAX_SNTP_SERVERS];
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }
   
    if(wifi_connected_flag == 0)
    {
       printf("device not connected \r\n");
       return A_ERROR;
    }   
    
	memset(&SntpDnsAddr[0],0,(sizeof(tSntpDnsAddr) * MAX_SNTP_SERVERS));
    
    
    if (A_OK != custom_ip_sntp_query_srvr_address((void *)handle, &SntpDnsAddr[0]))
    {
        printf("Unable to get the SNTP server address\r\n");
        return A_ERROR;
    }

 

    for(i = 0;i < MAX_SNTP_SERVERS;i++)
    {   
       
      if(SntpDnsAddr[i].resolve != DNS_NAME_NOT_SPECIFIED) 
      {
         printf("SNTP SERVER ADDRESS : %s\n",SntpDnsAddr[i].addr);
      }
      
    }

    return A_OK;
}

static A_INT32 ip_sntp_client_start_stop(A_UINT32 argc,char *argv[])
{
    A_INT32 command = -1;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if(argc < 3)
    {
       printf("Incomplete params\n");
       return A_ERROR;
    }
    if(ATH_STRCMP(argv[2], "start") == 0) 
    {
        command = 1;
    }
    else if(ATH_STRCMP(argv[2], "stop") == 0)
    {
        command = 0;
    }
    else
    {
        printf("Supported commands: start, stop");
        return (A_ERROR);	
    }	    
    if (A_OK != custom_ip_sntp_client((void *)handle, command))
    {
        printf("Error! SNTP Client %s failed\n", command?"start":"stop");
        return A_ERROR;
    }
    return A_OK;
	
}
#endif /* ENABLE_SNTP_CLIENT */

#if ENABLE_SSL
static A_INT32 ssl_start(A_INT32 argc, char *argv[])
{
    SSL_INST *ssl;
    A_INT32 res = A_ERROR;
    SSL_ROLE_T role;

    if(IS_DRIVER_READY != A_OK)
    {
        printf("Driver not loaded!\n");
        return A_ERROR;
    }

    if(argc < 3)
    {
       printf("ERROR: Incomplete params\n");
       return A_ERROR;
    }

    if (0 == strcmp(argv[2], "server"))
    {
        ssl = &ssl_inst[SSL_SERVER_INST];
        role = SSL_SERVER;
    }
    else if (0 == strcmp(argv[2], "client"))
    {
        ssl = &ssl_inst[SSL_CLIENT_INST];
        role = SSL_CLIENT;
    }
    else
    {
        printf("ERROR: Invalid parameter: %s\n", argv[2]);
        return A_ERROR;
    }

    if (ssl->sslCtx != NULL)
    {
        printf("ERROR: one SSL context only!\n");
        return A_ERROR;
    }

    // Create SSL context
    memset(ssl, 0, sizeof(SSL_INST));
    ssl->role = role;
    ssl->sslCtx = SSL_ctx_new(role, SSL_INBUF_SIZE, SSL_OUTBUF_SIZE, 0);
    if (ssl->sslCtx == NULL)
    {
        printf("ERROR: Unable to create SSL context\n");
        return A_ERROR;
    }

    // Reset config struct
    memset(&ssl->config, 0, sizeof(SSL_CONFIG));

    // Done
    printf("SSL %s started\n", argv[2]);
    return A_OK;
}

static A_INT32 ssl_stop(A_INT32 argc, char *argv[])
{
    SSL_INST *ssl;
    A_INT32 res = A_OK;
    SSL_ROLE_T role;

    if(IS_DRIVER_READY != A_OK)
    {
        printf("Driver not loaded!\n");
        return A_ERROR;
    }

    if(argc < 3)
    {
       printf("ERROR: Incomplete params\n");
       return A_ERROR;
    }

    if (0 == strcmp(argv[2], "server"))
    {
        ssl = &ssl_inst[SSL_SERVER_INST];
        role = SSL_SERVER;
    }
    else if (0 == strcmp(argv[2], "client"))
    {
        ssl = &ssl_inst[SSL_CLIENT_INST];
        role = SSL_CLIENT;
    }
    else
    {
        printf("ERROR: Invalid parameter: %s\n", argv[2]);
        return A_ERROR;
    }

    if (ssl->sslCtx == NULL || role != ssl->role)
    {
        printf("ERROR: SSL %s not started\n", argv[2]);
        return A_ERROR;
    }

    if (ssl->ssl)
    {
        SSL_shutdown(ssl->ssl);
        ssl->ssl = NULL;
    }

    if (ssl->sslCtx)
    {
        SSL_ctx_free(ssl->sslCtx);
        ssl->sslCtx = NULL;
    }

    printf("SSL %s stopped\n", argv[2]);
    return res;
}

static A_INT32 ssl_config(A_INT32 argc, char *argv[])
{
    SSL_INST *ssl;
    A_INT32 res;
    int argn, cipher_count = 0;

    if(IS_DRIVER_READY != A_OK)
    {
        printf("Driver not loaded!\n");
        return A_ERROR;
    }

    if(argc < 3)
    {
       printf("ERROR: Incomplete params\n");
    }

    if (0 == strcmp(argv[2], "server"))
    {
        ssl = &ssl_inst[SSL_SERVER_INST];
    }
    else if (0 == strcmp(argv[2], "client"))
    {
        ssl = &ssl_inst[SSL_CLIENT_INST];
    }
    else
    {
        printf("ERROR: Invalid parameter: %s\n", argv[2]);
        return A_ERROR;
    }

    if (ssl->sslCtx == NULL)
    {
        printf("ERROR: SSL %s not started\n", argv[2]);
        return A_ERROR;
    }

    argn = 3;
    ssl->config_set = 0;
    while (argn < argc)
    {
        if (argn == argc-1)
        {
            printf("ERROR: Incomplete params\n");
            return A_ERROR;
        }

        if (0 == strcmp("protocol", argv[argn]))
        {
            // Setting of protocol option is supported for SSL client only
            if (strcmp(argv[2], "client"))
            {
                printf("ERROR: Protocol option is not supported for %s\n", argv[2]);
                return A_ERROR;
            }
            argn++;
            if (0 == strcmp("SSL3", argv[argn]))
            {
                ssl->config.protocol = SSL_PROTOCOL_SSL_3_0;
            }
            else if (0 == strcmp("TLS1.0", argv[argn]))
            {
                ssl->config.protocol = SSL_PROTOCOL_TLS_1_0;
            }
            else if (0 == strcmp("TLS1.1", argv[argn]))
            {
                ssl->config.protocol = SSL_PROTOCOL_TLS_1_1;
            }
            else if (0 == strcmp("TLS1.2", argv[argn]))
            {
                ssl->config.protocol = SSL_PROTOCOL_TLS_1_2;
            }
            else
            {
                printf("ERROR: Invalid protocol: %s\n", argv[argn]);
                return A_ERROR;
            }
        }

        if (0 == strcmp("time", argv[argn]))
        {
            argn++;
            if (0 == strcmp("1", argv[argn]))
            {
                ssl->config.verify.timeValidity = 1;
            }
            else if (0 == strcmp("0", argv[argn]))
            {
                ssl->config.verify.timeValidity = 0;
            }
            else
            {
                printf("ERROR: Invalid option: %s\n", argv[argn]);
                return A_ERROR;
            }
        }

        if (0 == strcmp("alert", argv[argn]))
        {
            argn++;
            if (0 == strcmp("1", argv[argn]))
            {
                ssl->config.verify.sendAlert = 1;
            }
            else if (0 == strcmp("0", argv[argn]))
            {
                ssl->config.verify.sendAlert = 0;
            }
            else
            {
                printf("ERROR: Invalid option: %s\n", argv[argn]);
                return A_ERROR;
            }
        }

        if (0 == strcmp("domain", argv[argn]))
        {
            argn++;
            if (0 == strcmp("0", argv[argn]))
            {
                ssl->config.verify.domain = 0;
                ssl->config.matchName[0] = '\0';
            }
            else
            {
                ssl->config.verify.domain = 1;
                if (strlen(argv[argn]) >= sizeof(ssl->config.matchName))
                {
                    printf("ERROR: %s too long (max %d chars)\n", argv[argn], sizeof(ssl->config.matchName));
                    return A_ERROR;
                }
                strcpy(ssl->config.matchName, argv[argn]);
            }
        }

        if (0 == strcmp("cipher", argv[argn]))
        {
            argn++;
            if (cipher_count == 0)
            {
                memset(ssl->config.cipher, 0, sizeof(ssl->config.cipher));
            }
            if (cipher_count == SSL_CIPHERSUITE_LIST_DEPTH)
            {
                printf("ERROR: Too many cipher options (max %d)\n", argv[argn], SSL_CIPHERSUITE_LIST_DEPTH);
                return A_ERROR;
            }
            if (0 == strcmp("TLS_RSA_WITH_AES_256_GCM_SHA384", argv[argn]))
            {
                ssl->config.cipher[cipher_count] = TLS_RSA_WITH_AES_256_GCM_SHA384;
            }
            else if (0 == strcmp("TLS_RSA_WITH_AES_256_CBC_SHA256", argv[argn]))
            {
                ssl->config.cipher[cipher_count] = TLS_RSA_WITH_AES_256_CBC_SHA256;
            }
            else if (0 == strcmp("TLS_RSA_WITH_AES_256_CBC_SHA", argv[argn]))
            {
                ssl->config.cipher[cipher_count] = TLS_RSA_WITH_AES_256_CBC_SHA;
            }
            else if (0 == strcmp("TLS_RSA_WITH_AES_128_GCM_SHA256", argv[argn]))
            {
                ssl->config.cipher[cipher_count] = TLS_RSA_WITH_AES_128_GCM_SHA256;
            }
            else if (0 == strcmp("TLS_RSA_WITH_AES_128_CBC_SHA256", argv[argn]))
            {
                ssl->config.cipher[cipher_count] = TLS_RSA_WITH_AES_128_CBC_SHA256;
            }
            else if (0 == strcmp("TLS_RSA_WITH_AES_128_CBC_SHA", argv[argn]))
            {
                ssl->config.cipher[cipher_count] = TLS_RSA_WITH_AES_128_CBC_SHA;
            }
            else if (0 == strcmp("TLS_RSA_WITH_3DES_EDE_CBC_SHA", argv[argn]))
            {
                ssl->config.cipher[cipher_count] = TLS_RSA_WITH_3DES_EDE_CBC_SHA;
            }
            else
            {
                printf("ERROR: Invalid protocol: %s\n", argv[argn]);
                return A_ERROR;
            }
            cipher_count++;
        }

        argn++;
    }

    if (ssl->ssl == NULL)
    {
        // Create SSL connection object
        ssl->ssl = SSL_new(ssl->sslCtx);
        if (ssl->ssl == NULL)
        {
            printf("ERROR: SSL configure failed (Unable to create SSL context)\n");
            return A_ERROR;;
        }
    }

    // configure the SSL connection
    ssl->config_set = 1;
    res = SSL_configure(ssl->ssl, &ssl->config);
    if (res < A_OK)
    {
        printf("ERROR: SSL configure failed (%d)\n", res);
        return A_ERROR;
    }
    printf("SSL %s configuration changed\n", argv[2]);
    return A_OK;
}

static A_INT32 ssl_add_cert(A_INT32 argc,char *argv[])
{
    SSL_INST *ssl;
    SSL_CERT_TYPE_T type;
    char *name = NULL;

    if(IS_DRIVER_READY != A_OK)
    {
        printf("Driver not loaded!\n");
        return A_ERROR;
    }

    if(argc < 4)
    {
       printf("Incomplete params\n");
       return A_ERROR;
    }

    if (0 == strcmp(argv[2], "server"))
    {
        ssl = &ssl_inst[SSL_SERVER_INST];
        if (ssl->sslCtx == NULL)
        {
            printf("ERROR: SSL server not started\n");
            return A_ERROR;
        }
    }
    else if (0 == strcmp(argv[2], "client"))
    {
        ssl = &ssl_inst[SSL_CLIENT_INST];
        if (ssl->sslCtx == NULL)
        {
            printf("ERROR: SSL client not started\n");
            return A_ERROR;
        }
    }
    else
    {
        printf("ERROR: Invalid parameter: %s\n", argv[2]);
        return A_ERROR;
    }

    if (0 == strcmp("certificate", argv[3]))
    {
        type = SSL_CERTIFICATE;
    }
    else if (0 == strcmp("calist", argv[3]))
    {
        type = SSL_CA_LIST;
    }
    else
    {
        printf("ERROR: Invalid parameter: %s\n", argv[3]);
        return A_ERROR;
    }

    if (argc > 4)
    {
        name = argv[4];
    }

    // Load/add certificate
    if (name != NULL)
    {
        if (SSL_loadCert(ssl->sslCtx, type, name) < A_OK)
        {
            printf("ERROR: Unable to load %s from FLASH\n" , name);
            return A_ERROR;
        }
        printf("%s loaded from FLASH\n", name);
    }
    else
    {
        A_UINT8 *cert = ssl_cert_data_buf;
        A_UINT16 cert_len = ssl_cert_data_buf_len;
        if (type == SSL_CERTIFICATE)
        {
            if (cert_len == 0)
            {
                // Load the default certificate
                printf("Note: using the default certificate\n");
                cert = (A_UINT8*)ssl_default_cert;
                cert_len = ssl_default_cert_len;
            }
            if (SSL_addCert(ssl->sslCtx, cert, cert_len) < A_OK)
            {
                printf("ERROR: Unable to add certificate\n");
                return A_OK;
            }
            printf("Certificate added to SSL server\n");
        }
        else
        {
            if (cert_len == 0)
            {
                // Load the default CA list
                printf("Note: using the default CA list\n");
                cert = (A_UINT8*)ssl_default_calist;
                cert_len = ssl_default_calist_len;
            }
            if (SSL_setCaList(ssl->sslCtx, cert, cert_len)< A_OK)
            {
                printf("ERROR: Unable to set CA list\n");
                return A_ERROR;
            }
            printf("CA list added to SSL client\n");
        }
    }
    return A_OK;
}

static A_INT32 ssl_store_cert(A_INT32 argc,char *argv[])
{
    char *name;

    if(IS_DRIVER_READY != A_OK)
    {
        printf("Driver not loaded!\n");
        return A_ERROR;
    }

    if(argc < 3)
    {
       printf("Incomplete params\n");
       return A_ERROR;
    }
    name = argv[2];

    if (ssl_cert_data_buf_len == 0)
    {
        printf("ERROR: no certificate data.\nHint: Use the wmiconfig --ssl_get_cert to read a certificate from a certificate server.\n");
        return A_ERROR;
    }
    if (A_OK == SSL_storeCert(name, ssl_cert_data_buf, ssl_cert_data_buf_len))
    {
        printf("%s is stored in FLASH\n", name);
        return A_OK;
    }
    else
    {
        printf("ERROR: failed to store %s\n", name);
        return A_ERROR;
    }
}

static A_INT32 ssl_delete_cert(A_INT32 argc,char *argv[])
{
    char *name;

    if(IS_DRIVER_READY != A_OK)
    {
        printf("Driver not loaded!\n");
        return A_ERROR;
    }

    if(argc < 3)
    {
        ssl_cert_data_buf_len = 0;
        printf("Deleted the certificate data stored in RAM.\n");
        return A_OK;
    }
    name = argv[2];

    if (A_OK == SSL_storeCert(name, NULL, 0))
    {
        printf("Deleted %s from FLASH\n", name);
        return A_OK;
    }
    else
    {
        printf("ERROR: failed to delete %s\n", name);
        return A_ERROR;
    }
}

static A_INT32 ssl_list_cert(A_INT32 argc,char *argv[])
{
    SSL_FILE_NAME_LIST *fileNames;
    A_INT32 i, numFiles, namesLen = sizeof(SSL_FILE_NAME_LIST); // 10 files

    if(IS_DRIVER_READY != A_OK)
    {
        printf("Driver not loaded!\n");
        return A_ERROR;
    }

    fileNames = A_MALLOC(namesLen, MALLOC_ID_TEMPORARY);
    if (fileNames == NULL)
    {
        printf("ERROR: not enough memory\n");
        return A_ERROR;
    }

    numFiles = SSL_listCert(fileNames);
    if (numFiles < 0)
    {
        printf("ERROR: failed to list files (%d)\n", numFiles);
        A_FREE(fileNames, MALLOC_ID_TEMPORARY);
        return A_ERROR;
    }

    printf("%d %s stored in FLASH\n", numFiles, numFiles == 1 ? "file" : "files");
    for (i=0; i<numFiles; i++)
    {
        char str[21];
        if (fileNames->name[i][0])
        {
            strncpy(str, (char*)fileNames->name[i], sizeof(str)-1);
            str[sizeof(str)-1] = '\0';
            printf("%s\n", str);
        }
    }
    return A_OK;
}
#endif // ENABLE_SSL
#endif /* ENABLE_STACK_OFFLOAD */

#if ANTENNA_DIVERSITY

void AntennaStatCallback(void *ptr)
{
   WMI_HW_ANT_DIV_CMD_STAT *antdivstat = (WMI_HW_ANT_DIV_CMD_STAT *)ptr;
   
   printf("\nHWANTDIV:hw_ant_div=%d\n",antdivstat->hw_ant_div);
   printf("HWANTDIV:hw_rx_curr_ant=%d\n",antdivstat->hw_rx_curr_ant);
   printf("HWANTDIV:hw_tx_curr_ant=%d\n",antdivstat->hw_tx_curr_ant);	
   printf("HWANTDIV:tx_ant_adjust=%d\n",antdivstat->tx_ant_adjust);
   printf("HWANTDIV:total_pkt_count=%d\n",antdivstat->total_pkt_count);
   printf("HWANTDIV:main_total_rssi=%d\n",antdivstat->main_total_rssi);
   printf("HWANTDIV:alt_total_rssi=%d\n",antdivstat->alt_total_rssi);
   printf("HWANTDIV:ant_select_cnt=%d\n",antdivstat->ant_select_cnt);
   printf("HWANTDIV:ant_switch_cnt=%d\n",antdivstat->ant_switch_cnt);
   
}

/*FUNCTION*-----------------------------------------------------------------
*
* Function Name  : set_antenna_callback
* Returned Value : A_ERROR on error else A_OK
* Comments       : Sets callback function for antenna statistics event
*
*END------------------------------------------------------------------*/

A_INT32 set_antenna_callback()
{
    ATH_IOCTL_PARAM_STRUCT  inout_param;
    A_INT32 error;

    inout_param.cmd_id = ATH_SET_ANTENNA_STAT_CALLBACK;
    inout_param.data = (void*)AntennaStatCallback;
    inout_param.length = 4;
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    error = HANDLE_IOCTL(&inout_param);

    return error;
}

static A_INT32 hwantdivset(A_INT32 argc,char *argv[])
{
	ATH_IOCTL_PARAM_STRUCT  inout_param;
    A_INT32 error;
	WMI_HW_ANT_DIV_CMD_STAT hwantdiv;	

	if (argc < 4) 
	{
		printf(" usage : wmiconfig --hwantdivset <div> <adjust>\n");
		return (A_ERROR);
	}

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }
    hwantdiv.hw_ant_div = atoi(argv[2]);
	hwantdiv.tx_ant_adjust = atoi(argv[3]);
	
    inout_param.cmd_id = ATH_HW_ANTDIV_SET_CMD;
    inout_param.data = &hwantdiv;
    inout_param.length = 2;

    error = HANDLE_IOCTL (&inout_param);
    if (A_OK != error)
    {
		printf("hwantdivset Failed\n");
        return error;
    }

    return A_OK;

}

static A_INT32 hwantdivget(A_INT32 argc,char *argv[])
{
	ATH_IOCTL_PARAM_STRUCT  inout_param;
    A_INT32 error;
	WMI_HW_ANT_DIV_CMD_STAT hwantdiv;	

	if (argc < 2) 
	{
		printf(" usage : wmiconfig --hwantdivget\n");
		return A_ERROR;
	}

    set_antenna_callback();

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK) {
        return A_ERROR;
    }
	
	inout_param.cmd_id = ATH_HW_ANTDIV_GET_CMD;
	inout_param.data = &hwantdiv;
	inout_param.length = 2;

	error = HANDLE_IOCTL (&inout_param);
	if (A_OK != error)
	{
		printf(" hwantdivget Failed \n");
	    return error;
	}

    return A_OK;

}
#endif

/*FUNCTION*-----------------------------------------------------------------
*
* Function Name  : set_gtx
* Returned Value : A_OK
* Comments       : Enable/Disable GTX
*
*END------------------------------------------------------------------*/
static A_INT32 set_gtx(char* gtx)
{
    A_UINT8 b_gtx = atoi(gtx);
    ATH_IOCTL_PARAM_STRUCT  inout_param;
    A_INT32 error;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    inout_param.cmd_id = ATH_ONOFF_GTX;
    inout_param.data   = &b_gtx;
    inout_param.length = sizeof (A_BOOL);

    error = HANDLE_IOCTL (&inout_param);
    return A_OK;
}

/*FUNCTION*-----------------------------------------------------------------
*
* Function Name  : set_lpl
* Returned Value : A_OK
* Comments       : Enable/Disable LPL
*
*END------------------------------------------------------------------*/
static A_INT32 set_lpl(char* lpl)
{
    A_UINT8 b_lpl = atoi(lpl);
    ATH_IOCTL_PARAM_STRUCT  inout_param;
    A_INT32 error;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    inout_param.cmd_id = ATH_ONOFF_LPL;
    inout_param.data   = &b_lpl;
    inout_param.length = sizeof (A_BOOL);

    error = HANDLE_IOCTL (&inout_param);
    return A_OK;
}





static A_INT32 mac_store(char *argv[])
{
	A_INT32 error;
    ATH_IOCTL_PARAM_STRUCT ath_param;
    A_UINT8 val;
   	A_INT32 j;
   	ATH_PROGRAM_MAC_ADDR_PARAM data;

	ath_param.cmd_id = ATH_PROGRAM_MAC_ADDR;
    ath_param.length = sizeof(data);
	/* get mac address from argv */
	if(strlen(argv[0]) != 12)
	{
		printf("Invalid MAC address length\n");
		return A_ERROR;
	}

	for(j=0;j<6;j++)
	{
		data.addr[j] = 0;
	}
	for(j=0;j<strlen(argv[0]);j++)
	{
		val = ascii_to_hex(argv[0][j]);
		if(val == 0xff)
		{
			printf("Invalid character\n");
			return A_ERROR;
		}
		else
		{
			if((j&1) == 0)
				val <<= 4;

			data.addr[j>>1] |= val;
		}
	}


    ath_param.data = (A_UINT8*)&data;

    if((error = HANDLE_IOCTL (&ath_param)) != A_OK){
    	printf("MAC PROGRAM ERROR: unknown driver error\n");
    	return A_ERROR;
    }else{
    	/* macaddr[0] contains result code */
    	switch(data.result)
    	{
    	case ATH_PROGRAM_MAC_RESULT_DRIVER_FAILED:
    		printf("MAC PROGRAM ERROR: the driver was unable to complete the request.\n");
    	break;
    	case ATH_PROGRAM_MAC_RESULT_SUCCESS:
    		printf("MAC PROGRAM SUCCESS.\n");
    	break;
    	case ATH_PROGRAM_MAC_RESULT_DEV_DENIED:
    		printf("MAC PROGRAM ERROR: the firmware failed to program the mac address.\n");
    		printf(" possibly the same MAC address is already programmed.\n");
    	break;
    	case ATH_PROGRAM_MAC_RESULT_DEV_FAILED:
    	default:
    		printf("MAC PROGRAM ERROR: Device unknown failure.\n");
    	break;
    	}
    }

    return A_OK;

}


/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : gpio_handler()
* Returned Value  : 1 - successful completion or
*                   0 - failed.
* Comments        : Handles GPIO related functionality
*
*END*-----------------------------------------------------------------*/
static A_INT32 gpio_handle(A_INT32 argc,char *argv[])
{
    ATH_IOCTL_PARAM_STRUCT inout_param;
    A_INT32 error;
    A_INT32 index = 2;

    if(ATH_STRCMP(argv[index],"output_set") == 0)
    {
        struct WMIX_GPIO_OUTPUT_SET {
            WMIX_CMD_HDR cmd;
            WMIX_GPIO_OUTPUT_SET_CMD setting;
        } gpio_output_set;

        if(argc != 7) {
            printf("\n Incomplete parameter list for gpio output_set \n");
            printf("\n Usage : wmiconfig --gpio output_set  <set_mask> <clear_mask> <enable_mask> <disable_mask>\n");
            return A_ERROR;
        }

        gpio_output_set.cmd.commandId = WMIX_GPIO_OUTPUT_SET_CMDID;
        gpio_output_set.setting.set_mask = strtol(argv[3], NULL, 16);
        gpio_output_set.setting.clear_mask = strtol(argv[4], NULL, 16);
        gpio_output_set.setting.enable_mask = strtol(argv[5], NULL, 16);
        gpio_output_set.setting.disable_mask = strtol(argv[6], NULL, 16);

        inout_param.cmd_id = ATH_GPIO_CMD;
        inout_param.data = (void *)&gpio_output_set;
        inout_param.length = sizeof(gpio_output_set);

        error = HANDLE_IOCTL(&inout_param);
        return error;
    }
    else if(ATH_STRCMP(argv[index],"input_get") == 0)
    {
        WMIX_CMD_HDR cmd;
        cmd.commandId = WMIX_GPIO_INPUT_GET_CMDID;

        inout_param.cmd_id = ATH_GPIO_CMD;
        inout_param.data = (void *)&cmd;
        inout_param.length = sizeof(cmd);

        error = HANDLE_IOCTL(&inout_param);
        return error;
    }
    else if(ATH_STRCMP(argv[index],"register_set") == 0)
    {
        struct WMIX_GPIO_REGISTER_SET {
            WMIX_CMD_HDR cmd;
            WMIX_GPIO_REGISTER_SET_CMD setting;
        } gpio_register_set;

        if(argc != 5) {
            printf("\n Incomplete parameter list for gpio output_set \n");
            printf("\n Usage : wmiconfig --gpio register_set  <gpioreg_id:hex> <value:hex>\n");
            return A_ERROR;
        }

        gpio_register_set.cmd.commandId =WMIX_GPIO_REGISTER_SET_CMDID;
        gpio_register_set.setting.gpioreg_id = strtol(argv[3], NULL, 16);
        gpio_register_set.setting.value = strtol(argv[4], NULL, 16);

        inout_param.cmd_id = ATH_GPIO_CMD;
        inout_param.data = (void *)&gpio_register_set;
        inout_param.length = sizeof(gpio_register_set);

        error = HANDLE_IOCTL(&inout_param);
        return error;
    }
    else if(ATH_STRCMP(argv[index],"register_get") == 0)
    {
        struct WMIX_GPIO_REGISTER_GET {
            WMIX_CMD_HDR cmd;
            WMIX_GPIO_REGISTER_GET_CMD setting;
        } gpio_register_get;

        if(argc != 4) {
            printf("\n Incomplete parameter list for gpio output_set \n");
            printf("\n Usage : wmiconfig --gpio register_get  <gpioreg_id:hex>\n");
            return A_ERROR;
        }

        gpio_register_get.cmd.commandId =WMIX_GPIO_REGISTER_GET_CMDID;
        gpio_register_get.setting.gpioreg_id = strtol(argv[3], NULL, 16);
    
        inout_param.cmd_id = ATH_GPIO_CMD;
        inout_param.data = (void *)&gpio_register_get;
        inout_param.length = sizeof(gpio_register_get);

        error = HANDLE_IOCTL(&inout_param);
        return error;
    }
    else
    {
        printf("\n Incorrect Incomplete parameter list \n");
        return SHELL_EXIT_ERROR;
    }
    return A_OK;
}


/* The start of wmiconfig TS specific TODO */

/*FUNCTION*-----------------------------------------------------------------
*
* Function Name  : set_passphrase
* Returned Value : A_OK
* Comments       : Store WPA Passphrase for later use
*
*END------------------------------------------------------------------*/
A_INT32 wmiconfig_set_passphrase(char* passphrase)
{
	strcpy(wpa_passphrase,passphrase);
	return A_OK;
}

/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : wmiconfig_set_mode_ap()
* Returned Value  : A_OK - successful completion or
*					A_ERROR - failed.
* Comments		  : Turingsense specific, non static function for setting
* 					"wmiconfig --mode ap"
*
*END*-----------------------------------------------------------------*/
A_INT32 wmiconfig_set_mode_ap ()
{
    ath_set_essid (DEMOCFG_DEFAULT_DEVICE,"");
	mode_flag = MODE_AP;
	return A_OK;
}

/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : wmiconfig_ap_handler()
* Returned Value  : A_OK - successful completion or
*					A_ERROR - failed.
* Comments		  : Turingsense specific, non static function for setting
* 					"ap" options (wmiconfig_ap_handler).
*
*END*-----------------------------------------------------------------*/
A_INT32 wmiconfig_ap_handler( int_32 index, int_32 argc, char_ptr argv[])
{
    uint_32  error= A_OK, i, ret_val;
    char* p = 0;

    if (MODE_AP != mode_flag)
    {
      printf ("Err:Set AP Mode to apply AP settings\n");
      return A_ERROR;
    }

    if (index+1 < argc)
    {
        i = index;

        if (ATH_STRCMP(argv[i],"country") == 0)
        {


            ap_config_param.u.wmi_ap_set_country_cmd.countryCode[0] = argv[i+1][0];
            ap_config_param.u.wmi_ap_set_country_cmd.countryCode[1] = argv[i+1][1];
            ap_config_param.u.wmi_ap_set_country_cmd.countryCode[2] = 0x20;

            if ( ('F' == ap_config_param.u.wmi_ap_set_country_cmd.countryCode[0] && 'F' == ap_config_param.u.wmi_ap_set_country_cmd.countryCode[1]) ||
                 ('f' == ap_config_param.u.wmi_ap_set_country_cmd.countryCode[0] && 'f' == ap_config_param.u.wmi_ap_set_country_cmd.countryCode[1]))
            {
              printf ("AP in WWR Mode\n");
            }

            ret_val = set_ap_params (AP_SUB_CMD_SET_COUNTRY, (A_UINT8 *)ap_config_param.u.wmi_ap_set_country_cmd.countryCode);

            if (ret_val != A_OK)
            {
      			printf ("Err:set country code \n");
      			return A_ERROR;
			}
        }
        else
        if(ATH_STRCMP(argv[i],"inact") == 0)
        {
            ap_config_param.u.wmi_ap_conn_inact_cmd.period = strtoul(argv[i+1], &p,0);

            if(ap_config_param.u.wmi_ap_conn_inact_cmd.period == 0)
            {
     			printf ("incorrect inactive timeout \n");
      			return A_ERROR;
			}

            ret_val = set_ap_params(AP_SUB_CMD_INACT_TIME, (A_UINT8 *)&ap_config_param.u.wmi_ap_conn_inact_cmd.period);

	        if (ret_val != A_OK)
            {
      			printf ("Err:set inactive_time\n");
      			return A_ERROR;
			}
        }
        else
        if(ATH_STRCMP(argv[i],"bconint") == 0)
        {
            ap_config_param.u.wmi_beacon_int_cmd.beaconInterval = strtoul(argv[i+1], &p,0);

            if((ap_config_param.u.wmi_beacon_int_cmd.beaconInterval < 100) || (ap_config_param.u.wmi_beacon_int_cmd.beaconInterval > 1000))
            {
      			printf ("beacon interval has to be within 100-1000 in units of 100 \n");
      			return A_ERROR;
			}

            ret_val = set_ap_params(AP_SUB_CMD_BCON_INT, (A_UINT8 *)&ap_config_param.u.wmi_beacon_int_cmd.beaconInterval);

            if (ret_val != A_OK)
            {
      			printf ("Err:set beacon interval \n");
      			return A_ERROR;
			}
        }
        else
        if(ATH_STRCMP(argv[i],"dtim") == 0)
        {
            ap_config_param.u.wmi_ap_dtim_cmd.dtimval = strtoul(argv[i+1], &p,0);

            if((ap_config_param.u.wmi_ap_dtim_cmd.dtimval < 1) || (ap_config_param.u.wmi_ap_dtim_cmd.dtimval > 255))
            {
      			printf ("dtim interval has to be within 1-255 \n");
      			return A_ERROR;
			}

            ret_val = set_ap_params(AP_SUB_CMD_DTIM_INT, (A_UINT8 *)&ap_config_param.u.wmi_ap_dtim_cmd.dtimval);

            if (ret_val != A_OK)
            {
      			printf ("Err:set dtim interval \n");
      			return A_ERROR;
			}
		}
        else
        if(ATH_STRCMP(argv[i],"psbuf") == 0)
        {
            ap_config_param.u.wmi_ap_ps_buf_cmd.ps_val[0] = strtoul(argv[i+1], &p,0);

            if(ap_config_param.u.wmi_ap_ps_buf_cmd.ps_val[0] == 0)
            {

				ap_config_param.u.wmi_ap_ps_buf_cmd.ps_val[1] = 0x0;

            	ret_val = set_ap_params(AP_SUB_CMD_PSBUF, (A_UINT8 *)ap_config_param.u.wmi_ap_ps_buf_cmd.ps_val);

            	if (ret_val != A_OK)
            	{
      				printf ("Err:set ps buf \n");
      				return A_ERROR;
				}
			}
			else
			{
				ap_config_param.u.wmi_ap_ps_buf_cmd.ps_val[1] = strtoul(argv[i+2], &p,0);

            	ret_val = set_ap_params(AP_SUB_CMD_PSBUF, (A_UINT8 *)ap_config_param.u.wmi_ap_ps_buf_cmd.ps_val);

            	if (ret_val != A_OK)
            	{
      				printf ("Err:set ps buf \n");
      				return A_ERROR;
				}
			}
		}
		else
        {
            printf ("usage:--ap [country <country code>]\n"
                    "      --ap [bconint<intvl>]\n"
                    "      --ap [dtim<val>]\n"
                    "      --ap [inact <minutes>]>\n"
                    "      --ap [psbuf <1/0-enable/disable> <buf_count for enable>]>\n");
        }
    }
    else
    {
            printf ("usage:--ap [country <country code>]\n"
                    "      --ap [bconint<intvl>]\n"
                    "      --ap [dtim<val>]\n"
                    "      --ap [inact <minutes>]>\n"
                    "      --ap [psbuf <1/0-enable/disable> <buf_count for enable>]>\n");
    }

    return error;
}


/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : wmiconfig_connect_handler()
* Returned Value  : A_OK - successful completion or
*					A_ERROR - failed.
* Comments		  : Handles Connect commands for infrastructure mode, Open
*                   WEP,WPA/WPA2 security is supported
*
*END*-----------------------------------------------------------------*/
A_INT32 wmiconfig_connect_handler (char* ssid_str)
{
    A_INT32 error= A_OK, i;
    int offset = 0, ssidLength=MAX_SSID_LENGTH ;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    set_callback();
#if ENABLE_AP_MODE
	if (mode_flag == MODE_AP)
	{
		printf("setting to ap mode \n");
    	ATH_SET_MODE (DEMOCFG_DEFAULT_DEVICE,"softap");
        /* Set to MAX PERF Mode */
        SET_POWER_MODE("0");
    	if (hidden_flag == 1)
    	{
			hidden_flag = 1;
			ap_config_param.u.wmi_ap_hidden_ssid_cmd.hidden_ssid = 1;
			if (set_ap_params(AP_SUB_CMD_HIDDEN_FLAG, (A_UINT8 *)&ap_config_param.u.wmi_ap_hidden_ssid_cmd.hidden_ssid) != SHELL_EXIT_SUCCESS)
			{
				printf("not able to set hidden mode for AP \n");
				return A_ERROR;
			}
		}
        if (wps_flag == 1)
        {
            if (set_ap_params (AP_SUB_CMD_WPS_FLAG, (A_UINT8 *)&wps_flag) != SHELL_EXIT_SUCCESS)
            {
                printf("not able to set wps mode for AP \n");
                return SHELL_EXIT_ERROR;
            }
            if (SEC_MODE_OPEN == security_mode)
		 	{
				printf("AP in OPEN mode!\n");
			}
        }
	}
	else
#endif
    {
    	ATH_SET_MODE (DEMOCFG_DEFAULT_DEVICE,"managed");
	}

	if((ssidLength < 0) || (strlen(ssid_str) > ssidLength))
	{
		printf("SSID length exceeds Maximum value\n");
		return A_ERROR;
	}


    error = ATH_SET_ESSID (DEMOCFG_DEFAULT_DEVICE,ssid_str);
    if (error != 0)
    {
        printf ("Error during setting of ssid %s error=%08x!\n",ssid_str, error);
        return A_ERROR;
    }

    printf ("\nSetting SSID to %s \n\n",ssid_str);
    strcpy((char*)original_ssid,ssid_str);

	if(SEC_MODE_WPA == security_mode)
	{
		error = set_cipher(DEMOCFG_DEFAULT_DEVICE,(char *)&cipher);
		if(error != A_OK)
		{
			return A_ERROR;
		}
		error = ATH_SET_SEC_TYPE (DEMOCFG_DEFAULT_DEVICE,wpa_ver);
		if(error != A_OK)
		{
			return A_ERROR;
		}
		if(strlen(wpa_passphrase) != 64)
		{
			error = ATH_SET_PASSPHRASE (DEMOCFG_DEFAULT_DEVICE,wpa_passphrase);
			if(error != A_OK)
			{
				return A_ERROR;
			}
			pmk_flag = 1;
		}
		else
		{
			if(handle_pmk(wpa_passphrase)!= A_OK)
			{
				printf("Unable to set pmk\n");
				return A_ERROR;
			}
			pmk_flag = 1;
		}
		security_mode = SEC_MODE_OPEN;
	}

    error = ATH_COMMIT(DEMOCFG_DEFAULT_DEVICE);
    if(error != A_OK)
    {
#if ENABLE_AP_MODE
		if(mode_flag == MODE_AP)
		{
			printf("AP supports only AES and OPEN mode \n");
			// Clear Invalid configurations
			error = ATH_SET_SEC_TYPE (DEMOCFG_DEFAULT_DEVICE,"none");
			if(error != A_OK)
			{
				printf("Unable to clear Sec mode\n");
				return A_ERROR;
			}

			security_mode = SEC_MODE_OPEN;
			pmk_flag = 0;
		}
#endif
    	return A_ERROR;
    }
#if ENABLE_AP_MODE
    if (mode_flag == MODE_AP)
	{
		/* setting the AP's default IP to 192.168.1.1 */
#if ENABLE_STACK_OFFLOAD
		//A_UINT32 ip_addr = 0xc0a80101,mask = 0xffffff00,gateway = 0xc0a80101;
		//t_ipconfig((void*)handle,IPCFG_STATIC, &ip_addr, &mask, &gateway);
#else

#define AP_IPADDR  IPADDR(192,168,1,1)
#define AP_IPMASK  IPADDR(255,255,255,0)
#define AP_IPGATEWAY  IPADDR(192,168,1,1)

                A_UINT32 error;
                IPCFG_IP_ADDRESS_DATA   ip_data;

                ip_data.ip = AP_IPADDR;
                ip_data.mask = AP_IPMASK;
                ip_data.gateway = AP_IPGATEWAY;

                error = ipcfg_bind_staticip (DEMOCFG_DEFAULT_DEVICE, &ip_data);
                if (error != 0)
                {
                    printf ("Error during static ip bind %08x!\n", error);
                    return SHELL_EXIT_ERROR;
                }
#endif
	}
#endif

    return A_OK;
}

/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : wmiconfig_set_wpa()
* Returned Value  : A_OK - successful completion or
*		    		A_ERROR - failed.
* Parameters: ver 	 - 1 or 2
* 			  umode  - "CCMP" or "TKIP"
* 			  mmode  - "CCMP" or "TKIP"
* Comments	: Sets WPA mode
* 			  If the parameters are not as expected, this will set to
* 			  WPA2, CCMP, CCMP.
*
*END*-----------------------------------------------------------------*/
A_INT32 wmiconfig_set_wpa( A_INT32 ver, char* umode, char* mmode)
{

	/********** version ****************/
	if(ver == 1)
	{
		strcpy(wpa_ver,"wpa");
		//this is a hack we currently only support wpa2 AES in AP mode
		if (mode_flag == MODE_AP)
		{
			strcpy(wpa_ver,"wpa2");
		}
	}
	else /* Defaults to wpa2 */
	{
		strcpy(wpa_ver,"wpa2");
	}

	/**************** ucipher **********/
	if(ATH_STRCMP(umode,"TKIP")==0)
	{
		cipher.ucipher = 0x04;
	}
	else /* Defaults to CCMP */
	{
		cipher.ucipher = 0x08;
	}

	/*********** mcipher ************/
	if(ATH_STRCMP(mmode,"TKIP")==0)
	{
		cipher.mcipher = 0x04;
	}
	else
	{
		cipher.mcipher = 0x08;
	}
    security_mode = SEC_MODE_WPA;
    return A_OK;
}

#if TURINGSENSE
/* For US, can only use 1-11 */
A_INT32 wmiconfig_set_channel(uint32_t chnl)
{
   ATH_IOCTL_PARAM_STRUCT  inout_param;
   A_INT32 error;
   /*Check if driver is loaded*/
   if(IS_DRIVER_READY != A_OK){
       return A_ERROR;
   }

   if(chnl < 1 || chnl > 165)
   {
//     printf("invalid channel: should range between 1-165\n");
     return A_ERROR;
   }

   /* convert channel from number to freq */
   if (chnl < 27) {
	chnl = ATH_IOCTL_FREQ_1 + (chnl-1)*5;
   } else {
	   chnl = (5000 + (chnl*5));
   }

   inout_param.cmd_id = ATH_SET_CHANNEL;
   inout_param.data = &chnl;
   inout_param.length = 4;

   error = HANDLE_IOCTL (&inout_param);
   if (A_OK != error)
   {
       return error;
   }
   return A_OK;
}
#endif

/*FUNCTION*--------------------------------------------------------------------
*
* Function Name   : wmiconfig_set_tx_power ()
* Returned Value  : A_OK - on successful completion
*		   			or any other error
* Comments        : sets transmit power.
*
*END*------------------------------------------------------------------------*/
A_INT32 wmiconfig_set_tx_power(uint32_t dBm)
{

    ATH_IOCTL_PARAM_STRUCT  inout_param;
    A_INT32 error;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    inout_param.cmd_id = ATH_SET_TXPWR;
    inout_param.data = &dBm;
    inout_param.length = 1;//strlen(dBm);

    error = HANDLE_IOCTL (&inout_param);
    if (A_OK != error)
    {
        return error;
    }
    return A_OK;
}

/* Check wifi connectivity */
A_UINT32 wifi_connected(void) {
	return wifi_connected_flag;
}

/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : wmiconfig_ipconfig_static()
* Returned Value  : A_OK if success else A_ERROR
* Parameters	  : ip_addr_string
* 					mask_string
* 					gw_string
*
* Comments        : Sample function to depict ipconfig functionality.
*                   Sets static IPv4 parameters.
*
*
*END*-----------------------------------------------------------------*/
A_INT32 wmiconfig_ipconfig_static(	char* ip_addr_string, char* mask_string, char* gw_string)
{
	A_UINT32 addr, mask, gw;

    A_INT32 error;
	unsigned int sbits;

	/*Check if driver is loaded*/
	if(IS_DRIVER_READY != A_OK){
		return A_ERROR;
	}

	error=parse_ipv4_ad(&addr, &sbits, ip_addr_string);
	if(error!=0)
	{
		printf("Invalid IP address\n");
		return error;
	}
	addr = A_BE2CPU32(addr);
	error=parse_ipv4_ad(&mask, &sbits, mask_string);
	if(error!=0)
	{
		printf("Invalid Mask address\n");
		return error;
	}
	mask = A_BE2CPU32(mask);
	error=parse_ipv4_ad(&gw, &sbits, gw_string);
	if(error!=0)
	{
		printf("Invalid Gateway address\n");
		return error;
	}
	gw = A_BE2CPU32(gw);
	t_ipconfig((void*)handle,IPCFG_STATIC, &addr, &mask, &gw,NULL,NULL);
	printf("IP:%x Mask:%x, Gateway:%x\n", addr, mask, gw);
	printf("IP:%d.%d.%d.%d Mask:%d.%d.%d.%d, Gateway:%d.%d.%d.%d\n",
				getByte(3, addr), getByte(2, addr), getByte(1, addr), getByte(0, addr),
				getByte(3, mask), getByte(2, mask), getByte(1, mask), getByte(0, mask),
				getByte(3, gw), getByte(2, gw), getByte(1, gw), getByte(0, gw));


	return 0;
}

void wmiconfig_set_wmi_powermode(WMI_POWER_MODE new_power_mode) {
	WMI_POWER_MODE power_mode = new_power_mode;
}

/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : wmiconfig_set_phy_mode
* Returned Value  : A_OK if success else ERROR
*
* Comments        : Sets current phy mode
*
*END*-----------------------------------------------------------------*/
A_INT32 wmiconfig_set_phy_mode(char* phy_mode)
{
    ATH_IOCTL_PARAM_STRUCT  inout_param;
    A_INT32 error;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    if ((ATH_STRCMP(phy_mode,"n") != 0) &&
        (ATH_STRCMP(phy_mode,"b") != 0) &&
        (ATH_STRCMP(phy_mode,"g") != 0) &&
        (ATH_STRCMP(phy_mode,"ht40") != 0))
	{
		printf("Invalid Phy mode\n");
		return A_ERROR;
	}

    inout_param.cmd_id = ATH_SET_PHY_MODE;
    inout_param.data = phy_mode;
    inout_param.length = sizeof(phy_mode);


    error = HANDLE_IOCTL (&inout_param);
    if (A_OK != error)
    {
        return error;
    }
    return A_OK;
}


/*FUNCTION*-------------------------------------------------------------
*
* Function Name   : wmiconfig_get_phy_mode()
* Returned Value  : A_OK if phy_mode set successfully else ERROR CODE
* Comments        : Gets the phy_mode.
*
*END*-----------------------------------------------------------------*/

A_INT32 wmiconfig_get_phy_mode (char* phy_mode)
{
    ATH_IOCTL_PARAM_STRUCT inout_param;
    A_INT32 error;
    A_UINT32 value = 0;

    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK){
        return A_ERROR;
    }

    inout_param.cmd_id = ATH_GET_PHY_MODE;
    inout_param.data = &value;

    error = HANDLE_IOCTL (&inout_param);
    if (A_OK != error)
    {
        return error;
    }

    if(*(A_UINT32*)(inout_param.data) == 0x01)
    {
    	strcpy(phy_mode,"a");
    }
    else if(*(A_UINT32*)(inout_param.data) == 0x02)
    {
    	strcpy(phy_mode,"g");
    }
    else if(*(A_UINT32*)(inout_param.data) == 0x04)
    {
    	strcpy(phy_mode,"b");
    }
    else
    {
    	strcpy(phy_mode,"mixed");
    }
    return A_OK;
}

