#ifndef MAIN_SPI_H_
#define MAIN_SPI_H_

#include "driver/spi_master.h"

typedef struct {
	spi_device_handle_t _handle;
} MAX35101_t;

//static const char *TAG = "wifi station";
#define TAG "wifi station"

#define SPI_MASTER_FREQ_1M      (APB_CLK_FREQ/80)
#define SPI_MASTER_FREQ_2M      (APB_CLK_FREQ/40)
#define SPI_MASTER_FREQ_4M      (APB_CLK_FREQ/20)

void MAX35101_Init(int16_t GPIO_MISO, int16_t GPIO_MOSI, int16_t GPIO_SCLK, int16_t GPIO_CS);

#endif /* MAIN_SPI_H_ */
