//###########################################################################
// FILE:   uip-conf.h
// TITLE:  uIP Project Specific Configuration File
//###########################################################################
// $TI Release: F28M36x Support Library v202 $
// $Release Date: Tue Apr  8 12:36:34 CDT 2014 $
//###########################################################################

#ifndef __UIP_CONF_H__
#define __UIP_CONF_H__

// 8 bit datatype
// This typedef defines the 8-bit type used throughout uIP.
typedef unsigned char u8_t;

// 16 bit datatype
// This typedef defines the 16-bit type used throughout uIP.
typedef unsigned short u16_t;

// Statistics datatype
// This typedef defines the dataype used for keeping statistics in
// uIP.
typedef unsigned short uip_stats_t;

// Ping IP address assignment
// Use first incoming "ping" packet to derive host IP address
#define UIP_CONF_PINGADDRCONF       0

// UDP support on or off
#define UIP_CONF_UDP                1

// UDP checksums on or off
// (not currently supported ... should be 0)
#define UIP_CONF_UDP_CHECKSUMS      0

// UDP Maximum Connections
#define UIP_CONF_UDP_CONNS          4

// Maximum number of TCP connections.
#define UIP_CONF_MAX_CONNECTIONS    2

// Maximum number of listening TCP ports.
#define UIP_CONF_MAX_LISTENPORTS    4

// Size of advertised receiver's window
//#define UIP_CONF_RECEIVE_WINDOW     400

// Size of ARP table
#define UIP_CONF_ARPTAB_SIZE        8

// uIP buffer size.
#define UIP_CONF_BUFFER_SIZE        1600

// uIP buffer is defined in the application.
#define UIP_CONF_EXTERNAL_BUFFER

// uIP statistics on or off
#define UIP_CONF_STATISTICS         1

// Logging on or off
#define UIP_CONF_LOGGING            0

// Broadcast Support
#define UIP_CONF_BROADCAST          0

// Link-Level Header length
#define UIP_CONF_LLH_LEN            14

// CPU byte order.
#define UIP_CONF_BYTE_ORDER         LITTLE_ENDIAN

// Here we include the header file for the application we are using in
// this example
#include "httpd/httpd.h"

// Define the uIP Application State type, based on the httpd.h state variable.
typedef struct httpd_state uip_tcp_appstate_t;

// UIP_APPCALL: the name of the application function. This function
// must return void and take no arguments (i.e., C type "void
// appfunc(void)").
#ifndef UIP_APPCALL
#define UIP_APPCALL     httpd_appcall
#endif

// Here we include the header file for the DPCP client we are using in
// this example
#include "dhcpc/dhcpc.h"

#endif // __UIP_CONF_H_


