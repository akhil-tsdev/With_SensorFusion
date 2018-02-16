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
/*
* $FileName: Command_Lists.c$
* $Version : 3.5.16.0$
* $Date    : Dec-2-2009$
*
* Comments:
*
*   
*
*END************************************************************************/

#include "main.h"
#include "throughput.h"
#include "atheros_stack_offload.h"
#include "shell.h"

#if ENABLE_STACK_OFFLOAD
#if ENABLE_HTTPS_SERVER || ENABLE_HTTPS_CLIENT
#include <https.h>
#endif
#endif

extern A_INT32 sensor_handle(A_INT32 argc, char* argv[] );
const SHELL_COMMAND_STRUCT Shell_commands[] = {
   { "exit",      Shell_exit },      
   { "help",      Shell_help }, 
#if TS_HUB_WIFI
   { "starttx",   worker_cmd_handler }, //fapp_benchtx_cmd},
#endif /* TS_HUB_WIFI */
   { "benchtx",   worker_cmd_handler }, //fapp_benchtx_cmd},   
   { "benchrx",   worker_cmd_handler }, //fapp_benchrx_cmd},
#if READ_HOST_MEMORY
   { "hostmemmap", worker_cmd_handler},
#endif
#if (ENABLE_STACK_OFFLOAD && MULTI_SOCKET_SUPPORT)
   { "benchrx_multi_socket", worker_cmd_handler	}, 
#if T_SELECT_VER1
   { "benchtx_multi_socket", worker_cmd_handler	}, 
#endif
#endif
   { "benchquit", worker_cmd_quit },
   { "benchmode", worker_cmd_handler }, 
   { "perf",     print_perf},
#if DEMOCFG_ENABLE_RTCS
#if ENABLE_STACK_OFFLOAD
   { "ipconfig",  ipconfig_query },
   { "ping",      wmi_ping},      
   { "ping6",     wmi_ping6},
#else
   { "netstat",   Shell_netstat },     
   { "ipconfig",  Shell_ipconfig },
   { "ping",      Shell_ping },      
#endif   
#if DEMOCFG_USE_WIFI   
   { "iwconfig",  worker_cmd_handler }, //wmi_iwconfig },
   { "wmiconfig", worker_cmd_handler }, //wmiconfig_handler },
#endif      
#if DEMOCFG_USE_SENSOR
   { "sensor", sensor_handle}, // sensor command list entry
#endif      
#endif
#if ENABLE_STACK_OFFLOAD
#if ENABLE_SSL
   { "getcert", worker_cmd_handler},
#if ENABLE_HTTPS_SERVER
   { "httpss", worker_cmd_handler},
#endif
#if ENABLE_HTTPS_CLIENT
   { "httpsc", worker_cmd_handler},
#endif
#endif
#endif
   { "?",         Shell_command_list },     
   { NULL,        NULL } 
};

const SHELL_COMMAND_STRUCT Telnet_commands[] = {
   { "exit",      Shell_exit },      
   { "help",      Shell_help }, 
#if DEMOCFG_ENABLE_USB_FILESYSTEM
   { "log",       Shell_log },
#endif


   { "?",         Shell_command_list },     
   
   { NULL,        NULL } 
};


/* EOF */
