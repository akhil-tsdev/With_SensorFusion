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

#include <main.h>
#include <throughput.h>
#if ENABLE_STACK_OFFLOAD
#if ENABLE_SSL
#include <https.h>

#define DEFAULT_PACKET_SIZE  1400
#define CLIENT_WAIT_TIME     1000

#define HTTPS_DEFAULT_PORT   443

#define HTTPD_RESP_200       "200 OK"
#define HTTPD_RESP_404       "404 Not found"
#define HTTPD_RESP_PROTOCOL  "HTTP/1.0"

#define HTTPS_PRINTF(args...)  // no printing...
//#define HTTPS_PRINTF(args...) printf(args)

/* Include certificate used by HTTPS servert*/
//#include "cert-kingfisher.inc"
extern const A_UINT8 sharkSslRSACertKingfisher[1016];

typedef struct
{
    SSL_CTX *sslCtx;
    SSL *ssl;
    A_UINT32 serverSocket;
    A_UINT32 clientSocket;
} HTTPS_SERVER_DATA_T;

HTTPS_SERVER_DATA_T HttpsServerData;

#if ENABLE_HTTPS_SERVER
const char * notfound_html=
  "<!DOCTYPE HTML PUBLIC \"-//IETF//DTD HTML 2.0//EN\">\r\n"
  "<html><head><title>404 Not Found</title></head>\r\n"
  "<body>\r\n"
    "<h1>Not Found</h1>\r\n"
  "</body></html>\r\n";

const char * index_html=
  "<html>\r\n"
    "<head><title>It Works</title></head>\r\n"
    "<body><h1>It Works!</h1>\r\n"
      "<p>This is the index.html page.</p>\r\n"
    "</body>\r\n"
  "</html>\r\n";
#endif

#if ENABLE_HTTPS_CLIENT
static A_INT32 send_http_get_request(SSL *ssl, char *host, char *path)
{
    int n = 0, l = CFG_PACKET_SIZE_MAX_TX;
    char *pBuf = CUSTOM_ALLOC(l);

    if (pBuf == NULL)
    {
        return A_ERROR;
    }

    n += snprintf(&pBuf[n], l-n, "GET %s HTTP/1.0\r\n", path);
    n += snprintf(&pBuf[n], l-n, "Host: %s\r\n", host);
    n += snprintf(&pBuf[n], l-n, "User-Agent: IOE Client\r\n");
    n += snprintf(&pBuf[n], l-n, "Accept: text/html\r\n");
    n += snprintf(&pBuf[n], l-n, "Content-length: 0\r\n\r\n");
    A_UINT32 res = SSL_write(ssl, pBuf, n);

    HTTPS_PRINTF("%s", pBuf);
    HTTPS_PRINTF("SSL_write(%d) %d\n", n, res);

#if !NON_BLOCKING_TX
    // Free the buffer only if NON_BLOCKING_TX is not enabled
    CUSTOM_FREE(pBuf);
#endif

    return res;
}

static A_INT32 read_http_response(SSL *ssl, A_UINT32 socketHandle)
{
    A_INT32 res;
    A_INT32 received;
    A_INT32 total = 0;
    char *pBuf;

    do
    {
        res = t_select((void*)handle, socketHandle, CLIENT_WAIT_TIME);
        HTTPS_PRINTF("t_select() %d\n", res);
        if(res == A_OK)
        {
            // HTTP response packet is available
#if ZERO_COPY
            received = SSL_read(ssl, (void**)&pBuf, CFG_PACKET_SIZE_MAX_RX);
            HTTPS_PRINTF("SSL_read() %d\n", received);
            if(received > 0)
            {
                A_INT32 i;
                for (i=0; i < received; i++)
                {
                    printf("%c", pBuf[i]);
                }
                zero_copy_free(pBuf);
                total += received;
            }
#else
            pBuf = A_MALLOC(CFG_PACKET_SIZE_MAX_RX, MALLOC_ID_CONTEXT);
            if(pBuf == NULL)
            {
               printf("ERROR: Out of memory error\n");
               res = A_ERROR;
            }
            else
            {
                do
                {
                    received = SSL_read(ssl, pBuf, CFG_PACKET_SIZE_MAX_RX);
                    if(received > 0)
                    {
                        A_INT32 i;
                        for (i=0; i < received; i++)
                        {
                            printf("%c", pBuf[i]);
                        }
                        total += received;
                    }
                } while (received > 0);
                A_FREE(pBuf, MALLOC_ID_CONTEXT);
            }
#endif
        }
    } while (res == A_OK);

    return total;
}

A_INT32 https_client_get(char *host, char *path, A_UINT32 port, A_UINT32 inputBufSize, A_UINT32 outputBufSize)
{
    A_INT32 res = A_ERROR;
    SSL_CTX *sslCtx = NULL;
    SSL *ssl = NULL;
    A_UINT32 socketHandle = 0;
    DNC_CFG_CMD dnsCfg;
    DNC_RESP_INFO dnsRespInfo;
    SOCKADDR_T hostAddr;
    A_UINT8 *caList = ssl_cert_data_buf;
    A_UINT16 caListLen = ssl_cert_data_buf_len;

    if (port == 0)
    {
        port = HTTPS_DEFAULT_PORT;
    }
    if (inputBufSize == 0)
    {
        inputBufSize = SSL_INBUF_SIZE;
    }
    if (outputBufSize == 0)
    {
        outputBufSize = SSL_OUTBUF_SIZE;
    }

    do
    {
        // resolve the IP address of the host
        if (0 == ath_inet_aton(host, &dnsRespInfo.ipaddrs_list[0]))
        {
            if (strlen(host) >= sizeof(dnsCfg.ahostname))
            {
                printf("GetERROR: host name too long\n");
                break;
            }
            strcpy((char*)dnsCfg.ahostname, host);
            dnsCfg.domain = ATH_AF_INET;
            dnsCfg.mode =  RESOLVEHOSTNAME;
            if (A_OK != custom_ip_resolve_hostname(handle, &dnsCfg, &dnsRespInfo))
            {
                printf("GetERROR: Unable to resolve host name\r\n");
                break;
            }
        }

        // create SSL context
        sslCtx = SSL_ctx_new(SSL_CLIENT, inputBufSize, outputBufSize, 0);
        HTTPS_PRINTF("SSL_ctx_new(SSL_CLIENT,%d,%d,%d) %x\n", inputBufSize, outputBufSize, 0, sslCtx);
        if (sslCtx == NULL)
        {
            printf("GetERROR: Unable to create SSL context\n");
            break;
        }

        // Load certificate
        if (caListLen != 0)
        {
            res = SSL_setCaList(sslCtx, caList, caListLen);
            HTTPS_PRINTF("SSL_setCaList() %d\n", res);
            if (res < A_OK)
            {
                printf("GetERROR: Unable to load CA list\n");
                break;
            }
        }

        // Create socket
        socketHandle = t_socket((void*)handle, ATH_AF_INET, SOCK_STREAM_TYPE, 0);
        if (socketHandle == A_ERROR)
        {
            printf("GetERROR: Unable to create socket\n");
            res = A_ERROR;
            break;
        }

        // Connect to the HTTPS server
        memset(&hostAddr, 0, sizeof(hostAddr));
        hostAddr.sin_addr = dnsRespInfo.ipaddrs_list[0];
        hostAddr.sin_port = port;
        hostAddr.sin_family = ATH_AF_INET;
        res = t_connect((void*)handle, socketHandle, (&hostAddr), sizeof(hostAddr));
        if(res != A_OK)
        {
            printf("GetERROR: Connection failed.\n");
            break;
        }

        // Create SSL connection inst
        ssl = SSL_new(sslCtx);
        HTTPS_PRINTF("SSL_new() %x\n", ssl);
        if (ssl == NULL)
        {
            printf("GetERROR: Unable to create SSL context\n");
            res = A_ERROR;
            break;
        }

        // Add socket handle to SSL connection
        res = SSL_set_fd(ssl, socketHandle);
        HTTPS_PRINTF("SSL_set_fd() %d\n", res);
        if (res < 0)
        {
            printf("GetERROR: Unable to add socket handle to SSL\n");
            break;
        }

        // SSL handshake with server
        res = SSL_connect(ssl);
        HTTPS_PRINTF("SSL_connect() %d\n", res);
        if (res < 0)
        {
            printf("GetERROR: SSL connect failed\n");
            break;
        }

        // Send HTTP GET request
        res = send_http_get_request(ssl, host, path);
        if (res < 0)
        {
            printf("GetERROR: Failed to send HTTP GET request\n");
            break;
        }

        // Receive and print the HTTP response
        res = read_http_response(ssl, socketHandle);
        if (res <= 0)
        {
            printf("GetERROR: No HTTP response message received\n");
            break;
        }
        printf("GetOK\n");
    } while (0);

    // Clean up
    if (ssl != NULL)
    {
        SSL_shutdown(ssl);
    }
    if (sslCtx != NULL)
    {
        SSL_ctx_free(sslCtx);
    }
    if (socketHandle > 0)
    {
        t_shutdown(handle, socketHandle);
    }

    if (res > A_OK) // some of the SSL functions return 1 for success!
    {
        res = A_OK;
    }
    return res;
}

A_INT32 https_client_handler(A_INT32 argc,char *argv[])
{
    if(IS_DRIVER_READY != A_OK)
    {
        printf("Driver not loaded!\n");
        return A_ERROR;
    }

    if(argc < 4)
    {
        if (argc > 1)
        {
            printf("Incomplete params\n");
        }
        printf ("Usage: httpsc get <host> <path> [options]  - get a page from a HTTPS server\n");
        printf ("  where <host>  = Host name or IPaddress of HTTPS server\n");
        printf ("        <path>  = Page to retrieve\n");
        printf ("  options:\n");
        printf ("        -p<num> = Port number of HTTPS server (optional)\n");
        printf ("        -i<num> = SharkSSL input buffer size (optional)\n");
        printf ("        -o<num> = SharkSSL output buffer size (optional)\n");
        return A_ERROR;
    }

    if(ATH_STRCMP(argv[1], "get") == 0)
    {
        char *host = argv[2];
        char *path = argv[3];
        A_UINT32 port = 0;
        A_UINT32 inputBufSize = 0;
        A_UINT32 outputBufSize = 0;
        int argn = 4;

        // Handle optional arguments
        while (argn < argc)
        {
            if ((argv[argn][0] == '-') && (strlen(argv[argn]) > 2))
            {
                switch (argv[argn][1])
                {
                    case 'p':
                        port = atol(&argv[argn][2]);
                        break;
                    case 'i':
                        inputBufSize = atol(&argv[argn][2]);
                        break;
                    case 'o':
                        outputBufSize = atol(&argv[argn][2]);
                        break;
                }
            }
            argn++;
        }
        return https_client_get(host, path, port, inputBufSize, outputBufSize);
    }
    else
    {
        printf("Unknown command %s, supported command: get\n", argv[1]);
        return (A_ERROR);
    }
}
#endif // ENABLE_HTTPS_CLIENT

#if ENABLE_HTTPS_SERVER
static A_UINT32 send_http_response(const char *respCode, const char *content, int contentLen)
{
    int n = 0, l = CFG_PACKET_SIZE_MAX_TX;
    char *pBuf = CUSTOM_ALLOC(l);

    if (pBuf == NULL)
    {
        HTTPS_PRINTF("NO TX buffer!!!!\n");
        return A_ERROR;
    }

    // NOTE: This code does NOT split the HTTPS response message in multiple
    // buffers if it is too big to fit in one TX buffer!
    n += snprintf(pBuf, l-n, "%s %s\r\n", HTTPD_RESP_PROTOCOL, respCode);
    n += snprintf(&pBuf[n], l-n, "Content-Type: text/html\r\n");
    n += snprintf(&pBuf[n], l-n, "Content-Length: %d\r\n", contentLen);
    n += snprintf(&pBuf[n], l-n, "\r\n");
    memcpy(&pBuf[n], content, contentLen);
    n += contentLen;
    n += snprintf(&pBuf[n], l-n, "\r\n");
    int i;
    for (i=0; i<n; i++)
    {
        HTTPS_PRINTF("%c", pBuf[i]);
    }

    A_UINT32 res = SSL_write(HttpsServerData.ssl, pBuf, n);
    HTTPS_PRINTF("SSL_write(%d) %d\n", n, res);

#if !NON_BLOCKING_TX
    // Free the buffer only if NON_BLOCKING_TX is not enabled
    CUSTOM_FREE(pBuf);
#endif
    return res;
}

static A_INT32 process_http_request(char *buf, int size)
{
    const char *content;
    int contentLen;
    const char *respCode = HTTPD_RESP_200;
    int len = 0;

    if (strncmp(buf, "GET ", 4) == 0)
    {
        len = 4;
    }
    //else if (strncmp(buf, "POST ", 5) == 0)
    //    len = 5;
    else
    {
        HTTPS_PRINTF("Unsupported HTTP request\n");
        return -2;
    }

    // Parsing "GET /some/path" or "GET /some/path HTTP/1.X"
    buf += len;
    char *q = strchr(buf, ' ');
    if (q == NULL )
    {
        q = strchr(buf, '\r');
    }

    // Terminate file name
    *q = '\0';

    // find file
    if (strcmp(buf, "/") == 0 ||
        strcmp(buf, "/index.html") == 0)
    {
        content = index_html;
    }
    else
    {
        respCode = HTTPD_RESP_404;
        content = notfound_html;
    }
    contentLen = strlen(content);

    // Send HTTP response
    return send_http_response(respCode, content, contentLen);
}


static void handle_client(void)
{
    A_INT32 res;
    A_INT32 received;
    char *pBuf;

    while (HttpsServerData.clientSocket > 0)
    {
        res = t_select(handle, HttpsServerData.clientSocket, CLIENT_WAIT_TIME);
        if(res == A_SOCK_INVALID)
        {
            HTTPS_PRINTF("t_select() %d\n", res);
            t_shutdown(handle, HttpsServerData.clientSocket);
            HttpsServerData.clientSocket = 0;
            return;
        }

        if(res == A_OK)
        {
            // HTTP response packet is available
#if ZERO_COPY
            do
            {
                received = SSL_read(HttpsServerData.ssl, (void**)&pBuf, CFG_PACKET_SIZE_MAX_RX);
                HTTPS_PRINTF("SSL_read() %d\n", received);
                if(received > 0)
                {
                    // Simple solution that requres the HTTP request to be sent in one packet!
                    if (pBuf[received - 1] == '\n' && (pBuf[received - 3] == '\n' || pBuf[received - 2] == '\n'))
                    {
                        pBuf[received - 1] = '\0';
                        HTTPS_PRINTF("%s", pBuf);
                        process_http_request(pBuf, received);
                        t_shutdown(handle, HttpsServerData.clientSocket);
                        HttpsServerData.clientSocket = 0;
                    }
                    zero_copy_free(pBuf);
                }
            } while (received > 0 && HttpsServerData.clientSocket > 0);
#else
            pBuf = A_MALLOC(CFG_PACKET_SIZE_MAX_RX, MALLOC_ID_CONTEXT);
            if(pBuf == NULL)
            {
               printf("ERROR: Out of memory error\n");
               received = -1;
               res = A_ERROR;
            }
            else
            {
                do
                {
                    received = SSL_read(HttpsServerData.ssl, pBuf, CFG_PACKET_SIZE_MAX_RX);
                    if(received > 0)
                    {
                        // Simple solution that requres the HTTP request to be sent in one packet!
                        if (pBuf[received - 1] == '\n' && (pBuf[received - 3] == '\n' || pBuf[received - 2] == '\n'))
                        {
                            pBuf[received - 1] = '\0';
                            HTTPS_PRINTF("%s", pBuf);
                            process_http_request(pBuf, received);
                            t_shutdown(handle, HttpsServerData.clientSocket);
                            HttpsServerData.clientSocket = 0;
                        }
                    }
                } while (received > 0);
                A_FREE(pBuf, MALLOC_ID_CONTEXT);
            }
#endif
        }
    }
}

static void https_server_close_all(void)
{
    // Make sure sockets and SLL contexts are free'ed
    if (HttpsServerData.ssl != NULL)
    {
        SSL_shutdown(HttpsServerData.ssl);
        HttpsServerData.ssl = NULL;
    }
    if (HttpsServerData.sslCtx != NULL)
    {
        SSL_ctx_free(HttpsServerData.sslCtx);
        HttpsServerData.sslCtx = NULL;
    }
    if (HttpsServerData.clientSocket != 0)
    {
        if (HttpsServerData.clientSocket > 0)
        {
            t_shutdown(handle, HttpsServerData.clientSocket);
        }
        HttpsServerData.clientSocket = 0;
    }
    if (HttpsServerData.serverSocket != 0)
    {
        A_UINT32 socket = HttpsServerData.serverSocket;
        HttpsServerData.serverSocket = 0;
        if (socket > 0)
        {
            t_shutdown(handle, socket);
        }
    }
}

static A_INT32 https_server_start(A_UINT32 Port, A_UINT32 InputBufSize, A_UINT32 OutputBufSize)
{
    A_INT32 res = A_ERROR;
    SOCKADDR_T addr;
    A_UINT8 *cert = ssl_cert_data_buf;
    A_UINT16 cert_len = ssl_cert_data_buf_len;

    if (HttpsServerData.serverSocket)
    {
        printf("ERROR: server started");
        return A_ERROR;
    }

    if (Port == 0)
    {
        Port = HTTPS_DEFAULT_PORT;
    }
    if (InputBufSize == 0)
    {
        InputBufSize = SSL_INBUF_SIZE;
    }
    if (OutputBufSize == 0)
    {
        OutputBufSize = SSL_OUTBUF_SIZE;
    }

    while (1)
    {
        // Create server socket
        HttpsServerData.serverSocket = t_socket(handle, ATH_AF_INET, SOCK_STREAM_TYPE, 0);
        if (HttpsServerData.serverSocket == A_ERROR)
        {
            printf("ERROR: Unable to create socket\n");
            res = A_ERROR;
            break;
        }

        // Bind
        memset(&addr, 0, sizeof(addr));
        addr.sin_port = Port;
        addr.sin_family = ATH_AF_INET;
        res = t_bind(handle, HttpsServerData.serverSocket, &addr, sizeof(addr));
        if(res != A_OK)
        {
            printf("ERROR: Socket bind error\n");
            break;
        }

        // Create SSL context
        HttpsServerData.sslCtx = SSL_ctx_new(SSL_SERVER, InputBufSize, OutputBufSize, 0);
        if (HttpsServerData.sslCtx == NULL)
        {
            printf("ERROR: Unable to create SSL context\n");
            break;
        }

        // Load certificate
        if (cert_len == 0)
        {
            // Load the default certificate
            cert = (A_UINT8*)sharkSslRSACertKingfisher;
            cert_len = sizeof(sharkSslRSACertKingfisher);
            printf("Using the default  certificate\n");
        }
        if (SSL_addCert(HttpsServerData.sslCtx, cert, cert_len) < A_OK)
        {
            printf("ERROR: Unable to load certificate\n");
            break;
        }

        printf("HTTPS server started - waiting for a client to connect on port %d\n", Port);

        // Listen
        res = t_listen(handle, HttpsServerData.serverSocket, 1);
        if(res != A_OK)
        {
            printf("ERROR: Socket listen error\n");
            break;
        }

        // block until a client connects
        do
        {
            res = t_select(handle, HttpsServerData.serverSocket, 50);
        }while(res == A_ERROR && HttpsServerData.serverSocket != 0);
        if(HttpsServerData.serverSocket == 0)
        {
            break;// socket no longer valid
        }

        // Accept incoming connection
        HttpsServerData.clientSocket = t_accept(handle, HttpsServerData.serverSocket, &addr, sizeof(addr));
        if (HttpsServerData.clientSocket != A_ERROR)
        {
            do
            {
                // Create SSL connection inst
                HttpsServerData.ssl = SSL_new(HttpsServerData.sslCtx);
                if (HttpsServerData.ssl == NULL)
                {
                    printf("ERROR: Unable to create SSL context\n");
                    res = A_ERROR;
                    break;
                }

                // Add socket handle to SSL connection
                res = SSL_set_fd(HttpsServerData.ssl, HttpsServerData.clientSocket);
                if (res < 0)
                {
                    printf("ERROR: Unable to add socket handle to SSL\n");
                    break;
                }

                // SSL handshake with server
                res = SSL_accept(HttpsServerData.ssl);
                if (res < 0)
                {
                    printf("ERROR: SSL accept failed\n");
                    break;
                }

                // Handle HTTP requests from client
                handle_client();
            } while (0);
        }
        else
        {
            HttpsServerData.clientSocket = 0;
        }

        // clean up
        https_server_close_all();
    };


    // clean up
    https_server_close_all();
    printf("HTTPS server stopped\n");
    return A_OK;
}

static A_INT32 https_server_stop(void)
{
    if (HttpsServerData.serverSocket == 0)
    {
        printf("ERROR: server not started\n");
        return A_ERROR;
    }

    https_server_close_all();
    return A_OK;
}

A_INT32 https_server_handler(A_INT32 argc, char *argv[])
{
    /*Check if driver is loaded*/
    if(IS_DRIVER_READY != A_OK)
    {
        return A_ERROR;
    }

    if(argc < 2)
    {
        printf ("Usage: httpss start|stop> [options] = Start/Stop the HTTPS server\n");
        printf ("  options: (start only)\n");
        printf ("    -p<num> = Port number of HTTPS server (optional)\n");
        printf ("    -i<num> = SharkSSL input buffer size (optional)\n");
        printf ("    -o<num> = SharkSSL output buffer size (optional)\n");
        return A_ERROR;
    }

    if(ATH_STRCMP(argv[1], "start") == 0)
    {
        A_UINT32 port = 0;
        A_UINT32 inputBufSize = 0;
        A_UINT32 outputBufSize = 0;
        int argn = 2;

        // Handle optional arguments
        while (argn < argc)
        {
            if ((argv[argn][0] == '-') && (strlen(argv[argn]) > 2))
            {
                switch (argv[argn][1])
                {
                    case 'p':
                        port = atol(&argv[argn][2]);
                        break;
                    case 'i':
                        inputBufSize = atol(&argv[argn][2]);
                        break;
                    case 'o':
                        outputBufSize = atol(&argv[argn][2]);
                        break;
                    default:
                        printf("Invalid option: %s\n", argv[argn]);
                        return (A_ERROR);
                }
            }
            argn++;
        }
        return https_server_start(port, inputBufSize, outputBufSize);
    }

    if(ATH_STRCMP(argv[1], "stop") == 0)
    {
        return https_server_stop();
    }

    printf("Unknown command: %s, supported: start, stop\n", argv[1]);
    return (A_ERROR);
}
#endif
#endif // ENABLE_SSL
#endif // ENABLE_STACK_OFFLOAD
