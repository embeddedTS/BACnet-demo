/**************************************************************************
*
* Copyright (C) 2006 Steve Karg <skarg@users.sourceforge.net>
* Copyright (C) 2017-2022 Technologic Systems, Inc. dba embeddedTS <kris@embeddedTS.com>
*
* Permission is hereby granted, free of charge, to any person obtaining
* a copy of this software and associated documentation files (the
* "Software"), to deal in the Software without restriction, including
* without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to
* permit persons to whom the Software is furnished to do so, subject to
* the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
*********************************************************************/

/* This example is designed to connect to a single device on a BACnetMS/TP
 * network, set the output relay, read the input, toggle the output and read
 * again. This test assumes that the common and NO relay contacts are wired to
 * the dry contact input leads.
 *
 * This application will return a 0 on success, and a 1 on a read/write failure,
 * or a failure of the loopback test.
 *
 * To compile the application, first build bacnet-stack; it requires the
 * following build command to support MS/TP via RS-485:
 * make BACDL_DEFINE=-DBACDL_MSTP=1 clean all
 *
 * Once bacnet-stack is built, this example application can be compiled with:
 * gcc -g -Wall bacnetmstp-test.c demo/object/device-client.c -I ports/linux/ 
 *   -I include/ -I demo/object/ -L lib/ -DBACDL_MSTP=1 -o bacnetip-test
 *   -lbacnet -lpthread -lm
 *
 * This example was built and tested against bacnet-stack-0.8.3, using the
 * device RIBTW2401B-BC, on the TS-7680.  See more information on this example
 * here: https://wiki.embeddedTS.com/wiki/TS-7680_BACnet
 */ 


/* command line tool that sends a BACnet service, and displays the reply */
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include "iam.h"
#include "address.h"
#include "device.h"
#include "datalink.h"
#include "handlers.h"
#include "client.h"
#include "dlenv.h"
#include "tsm.h"

/* buffer used for receive */
static uint8_t Rx_Buf[MAX_MPDU] = { 0 };
static BACNET_APPLICATION_DATA_VALUE value = {0};

/* global variable used in this file */
static uint8_t req_id = 0;

#define BAC_ADDRESS_MULT 1

struct address_entry {
	uint8_t flags;
	uint32_t device_id;
	uint32_t last_val;
	unsigned max_apdu;
	BACNET_ADDRESS address;
} device_info;

unsigned int write_handled = 0;
void MyWritePropertyAckHandler(BACNET_ADDRESS * src, uint8_t invoke_id)
{
	if (address_match(&device_info.address, src) && (invoke_id == req_id)) {
		write_handled = 1;
	}
}

unsigned int read_handled = 0;
void MyReadPropertyAckHandler(uint8_t * service_request, uint16_t service_len,
  BACNET_ADDRESS * src, BACNET_CONFIRMED_SERVICE_ACK_DATA * service_data)
{
	BACNET_READ_PROPERTY_DATA data;
	BACNET_APPLICATION_DATA_VALUE value;

	if (address_match(&device_info.address, src) &&
	  (service_data->invoke_id == req_id)) {
		rp_ack_decode_service_request(service_request, service_len,
		  &data);
		bacapp_decode_application_data(data.application_data,
		  (uint8_t) data.application_data_len, &value);
		device_info.last_val = value.type.Enumerated;
		read_handled = 1;
	}
}

unsigned int i_am_handled = 0;
void MyIAmHandler (uint8_t * service_request, uint16_t service_len,
  BACNET_ADDRESS * src)
{
	int len = 0;
	uint32_t device_id = 0;
	unsigned max_apdu = 0;
	int segmentation = 0;
	uint16_t vendor_id = 0;

	(void) service_len;
	len = iam_decode_service_request(service_request, &device_id,
	  &max_apdu, &segmentation, &vendor_id);
	fprintf(stderr, "Received I-Am Request");
	if (len != -1) {
		fprintf(stderr, " from %lu, MAC = %d.%d.%d.%d.%d.%d\n",
		  (unsigned long) device_id, src->mac[0], src->mac[1],
		  src->mac[2], src->mac[3], src->mac[4], src->mac[5]);

		device_info.flags = BAC_ADDRESS_MULT;
		device_info.device_id = device_id;
		device_info.max_apdu = max_apdu;
		device_info.address = *src;
		i_am_handled = 1;
		address_add_binding(device_id, max_apdu, &device_info.address);
	} else {
		fprintf(stderr, ", but unable to decode it.\n");
	}
	return;
}

static void init_service_handlers(void)
{
	Device_Init(NULL);
	/* set the handler for all the services we don't implement
	It is required to send the proper reject message... */
	apdu_set_unrecognized_service_handler_handler(
	  handler_unrecognized_service);
	/* we must implement read property - it's required! */
	apdu_set_confirmed_handler(SERVICE_CONFIRMED_READ_PROPERTY,
	  handler_read_property);
	/* handle the data coming back from confirmed requests */
	apdu_set_confirmed_ack_handler(SERVICE_CONFIRMED_READ_PROPERTY,
	  MyReadPropertyAckHandler);
	/* handle the ack coming back */
	apdu_set_confirmed_simple_ack_handler(SERVICE_CONFIRMED_WRITE_PROPERTY,
	  MyWritePropertyAckHandler);
	/* handle the reply (request) coming back */
	apdu_set_unconfirmed_handler(SERVICE_UNCONFIRMED_I_AM, MyIAmHandler);
}

int handler_loop(BACNET_ADDRESS src, volatile unsigned int *check)
{
	time_t current_seconds = 0;
	time_t timeout_seconds = apdu_timeout() / 1000;
	uint16_t pdu_len = 0;
	time_t last_seconds = 0;
	int ret = 1;

	/* configure the timeout values */
	last_seconds = time(NULL);
	timeout_seconds = (apdu_timeout() / 1000);

	/* Loop until timeout or handler function returns */
	for (;;) {
		usleep(1000);
		tsm_timer_milliseconds(1);
		current_seconds = time(NULL);

		/* returns 0 bytes on timeout */
		pdu_len = datalink_receive(&src, &Rx_Buf[0], MAX_MPDU, 100);
		/* process */
		if (pdu_len) {
			npdu_handler(&src, &Rx_Buf[0], pdu_len);
		}

		if(*check) {
			*check = 0;
			ret = 0;
			break;
		}

		if ((current_seconds - last_seconds) > timeout_seconds)
		  break;
	}

	return ret;
}


int main(int argc, char *argv[])
{
	BACNET_ADDRESS src = {0};  /* address where message came from */
	BACNET_ADDRESS dest;
	bool found = 0;
	int err_cnt = 0;
	int i;

	/* setup my info */
	Device_Set_Object_Instance_Number(BACNET_MAX_INSTANCE);
	init_service_handlers();
	address_init();
	dlenv_init();
	atexit(datalink_cleanup);

	datalink_get_broadcast_address(&dest);

	for (;;) {
		found = address_bind_request(device_info.device_id,
		  &device_info.max_apdu, &device_info.address);
		if (!found) {
			Send_WhoIs(-1, -1);
			err_cnt += handler_loop(src, &i_am_handled);
		} else {
			break;
		}

		if (err_cnt > 4) {
			printf("Failed to locate a device on the network!\n");
			return 1;
		}
	}
	printf("Located and bound to ID %d\n", device_info.device_id);

	printf("Setting and clearing relay and checking loopback.\n");

	value.tag = BACNET_APPLICATION_TAG_ENUMERATED;
	for(i = 0; i < 2; i++) {
		value.type.Enumerated = i;
		req_id = Send_Write_Property_Request(
		  device_info.device_id, OBJECT_BINARY_OUTPUT, 1,
		  PROP_PRESENT_VALUE, &value, 0, BACNET_ARRAY_ALL);

		if (handler_loop(src, &write_handled) &&
		  !tsm_invoke_id_free(req_id)) {
			tsm_free_invoke_id(req_id);
			printf("Failed to toggle relay!\n");
			return 1;
		}
	
		usleep(1000000);
		req_id = Send_Read_Property_Request(
		  device_info.device_id,
		OBJECT_BINARY_INPUT, 1,
		PROP_PRESENT_VALUE, BACNET_ARRAY_ALL);

		if (handler_loop(src, &read_handled) &&
		  !tsm_invoke_id_free(req_id)) {
			tsm_free_invoke_id(req_id);
			printf("\nFailed to read input!\n");
			return 1;
		}

		if(device_info.last_val != i) {
			printf("ERROR! Input was not %d\n", i);
			return 1;
		}
	}

	return 0;
}
