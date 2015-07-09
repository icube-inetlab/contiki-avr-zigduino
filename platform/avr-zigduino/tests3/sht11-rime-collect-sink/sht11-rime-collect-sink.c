#include "contiki.h"
#include <stdio.h>

#include "lib/random.h"
#include "net/rime.h"
#include "net/rime/collect.h"
#include "net/rime/broadcast-announcement.h"
#include "net/netstack.h"

/* sink node id */
#define SINK 12

static struct collect_conn tc;

/*---------------------------------------------------------------------------*/
PROCESS(example_collect_process, "Test collect process");
AUTOSTART_PROCESSES(&example_collect_process);
/*---------------------------------------------------------------------------*/
static void
recv(const rimeaddr_t *originator, uint8_t seqno, uint8_t hops)
{
  printf("sink_received;from=%d.%d;seqno=%d;hops=%d;len=%d;payload=%s;\n",
#if RIMEADDR_SIZE == 8	
         originator->u8[6], originator->u8[7],
#else
		 originator->u8[0], originator->u8[1],
#endif
         seqno, hops,
         packetbuf_datalen(),
         (char *)packetbuf_dataptr());
}
/*---------------------------------------------------------------------------*/
static const struct collect_callbacks callbacks = { recv };
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_collect_process, ev, data)
{
	static struct etimer periodic;

	PROCESS_BEGIN();
	printf("[INIT] Node %d must begin\n", 
#if RIMEADDR_SIZE == 8	
	rimeaddr_node_addr.u8[7]
#else
	rimeaddr_node_addr.u8[1]
#endif	
	);
	
	collect_open(&tc, 130, COLLECT_ROUTER, &callbacks);

#if RIMEADDR_SIZE == 8	
	if(rimeaddr_node_addr.u8[7] == SINK)
#else
	if(rimeaddr_node_addr.u8[1] == SINK)
#endif
	{
		printf("[INIT] I am sink\n");
		collect_set_sink(&tc, 1);
	}

	/* Allow some time for the network to settle. */
	etimer_set(&periodic, 20 * CLOCK_SECOND);
	PROCESS_WAIT_UNTIL(etimer_expired(&periodic));
	
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
