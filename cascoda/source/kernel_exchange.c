#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "../include/cascoda_api.h"

/******************************************************************************/

#define DriverFilePath "/dev/ca8210_test"

/******************************************************************************/

static int ca8210_test_int_exchange(
	const uint8_t *buf,
	size_t len,
	uint8_t *response,
	void *pDeviceRef
);

/******************************************************************************/

static int DriverFileDescriptor;
static pthread_t rx_thread;
static pthread_mutex_t driver_mutex = PTHREAD_MUTEX_INITIALIZER;

/******************************************************************************/

static void *ca8210_test_int_read_worker(void *arg)
{
	uint8_t rx_buf[512];
	size_t rx_len;
	/* TODO: while not told to exit? */
	while (1) {
		if(pthread_mutex_lock(&driver_mutex)) fputs("MUTEX ERROR", stderr);
		rx_len = read(DriverFileDescriptor, rx_buf, 0);
		pthread_mutex_unlock(&driver_mutex);
		if (rx_len > 0) {
			fputs("ASYNC READ SOMETHING!", stderr);
			cascoda_downstream_dispatch(rx_buf, rx_len);
		}
	}

	return NULL;
}

int kernel_exchange_init(void)
{
	int ret;
	pthread_attr_t attrs;
	//pthread_mutex_init(&rx_mutex, PTHREAD_MUTEX_NORMAL);
	DriverFileDescriptor = open(DriverFilePath, O_RDWR);

	cascoda_api_downstream = ca8210_test_int_exchange;

	ret = pthread_create(&rx_thread, NULL, ca8210_test_int_read_worker, NULL);
	fprintf(stderr, "Error %d ", ret);
	return ret;
}

static void ca8210_test_int_write(const uint8_t *buf, size_t len)
{
	int returnvalue, remaining = len;

	do {
		returnvalue = write(DriverFileDescriptor, buf+len-remaining, remaining);
		if (returnvalue > 0)
			remaining -= returnvalue;
	} while (remaining > 0);
}

static int ca8210_test_int_exchange(
	const uint8_t *buf,
	size_t len,
	uint8_t *response,
	void *pDeviceRef
)
{
	int status, Rx_Length;
	fprintf(stderr, "Aquiring driver mutex for command %#04x...", buf[0]);

	pthread_mutex_lock(&driver_mutex);	//Enforce synchronous write then read

    fputs("Driver mutex aquired!", stderr);

	ca8210_test_int_write(buf, len);

	fputs("File written!", stderr);

	if (((buf[0] & SPI_SYN) && response)) {
		fputs("Waiting for reply...", stderr);
		do {
			Rx_Length = read(DriverFileDescriptor, response, NULL);
		} while (Rx_Length == 0);
	}

	pthread_mutex_unlock(&driver_mutex);

	fputs("Exchange complete!", stderr);

	return 0;
}
