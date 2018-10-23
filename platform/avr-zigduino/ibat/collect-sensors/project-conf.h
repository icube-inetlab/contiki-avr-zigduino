#ifndef __PROJECT_CONF_H__
#define __PROJECT_CONF_H__


#undef NETSTACK_CONF_RDC
#undef NETSTACK_CONF_FRAMER
#undef NETSTACK_CONF_MAC
#undef NETSTACK_CONF_NETWORK

#undef LINKADDR_CONF_SIZE
#define LINKADDR_CONF_SIZE        8

#if 1 /* No radio cycling */

#define NETSTACK_CONF_MAC         nullmac_driver
#define NETSTACK_CONF_RDC         nullrdc_driver
#define NETSTACK_CONF_FRAMER      framer_802154
#define NETSTACK_CONF_RADIO       rf230_driver
#define CHANNEL_802_15_4          26
/* AUTOACK receive mode gives better rssi measurements, even if ACK is never requested */
#define RF230_CONF_AUTOACK        1
/* 1 + Number of auto retry attempts 0-15 (0 implies don't use extended TX_ARET_ON mode) */
#define RF230_CONF_FRAME_RETRIES    2
/* Number of csma retry attempts 0-5 in extended tx mode (7 does immediate tx with no csma) */
#define RF230_CONF_CSMA_RETRIES   5
/* Default is one RAM buffer for received packets. More than one may benefit multiple TCP connections or ports */
#define RF230_CONF_RX_BUFFERS     3
#define SICSLOWPAN_CONF_FRAG      1
/* Most browsers reissue GETs after 3 seconds which stops fragment reassembly so a longer MAXAGE does no good */
#define SICSLOWPAN_CONF_MAXAGE    3
/* How long to wait before terminating an idle TCP connection. Smaller to allow faster sleep. Default is 120 seconds */
/* If wait is too short the connection can be reset as a result of multiple fragment reassembly timeouts */
#define UIP_CONF_WAIT_TIMEOUT    20
/* 211 bytes per queue buffer */
#define QUEUEBUF_CONF_NUM         8
/* 54 bytes per queue ref buffer */
#define QUEUEBUF_CONF_REF_NUM     2
/* Allocate remaining RAM as desired */
/* 30 bytes per TCP connection */
/* 6LoWPAN does not do well with concurrent TCP streams, as new browser GETs collide with packets coming */
/* from previous GETs, causing decreased throughput, retransmissions, and timeouts. Increase to study this. */
/* ACKs to other ports become interleaved with computation-intensive GETs, so ACKs are particularly missed. */
/* Increasing the number of packet receive buffers in RAM helps to keep ACKs from being lost */
#define UIP_CONF_MAX_CONNECTIONS  4
/* 2 bytes per TCP listening port */
#define UIP_CONF_MAX_LISTENPORTS  4
/* 25 bytes per UDP connection */
#define UIP_CONF_UDP_CONNS       10
/* See uip-ds6.h */
#undef NBR_TABLE_CONF_MAX_NEIGHBORS
#define NBR_TABLE_CONF_MAX_NEIGHBORS      20
#define UIP_CONF_DS6_DEFRT_NBU    2
#define UIP_CONF_DS6_PREFIX_NBU   3
#define UIP_CONF_MAX_ROUTES    20
#define UIP_CONF_DS6_ADDR_NBU     3
#define UIP_CONF_DS6_MADDR_NBU    0
#define UIP_CONF_DS6_AADDR_NBU    0

#else
#error Network configuration not specified!
#endif   /* Network setup */


#endif /* __PROJECT_CONF_H__ */
