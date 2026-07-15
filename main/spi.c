#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "esp_log.h"

#include "spi.h"


// SPI Stuff
#if CONFIG_SPI2_HOST
#define HOST_ID SPI2_HOST
#elif CONFIG_SPI3_HOST
#define HOST_ID SPI3_HOST
#endif

MAX35101_t dev;



void MAX35101_Init(int16_t GPIO_MISO, int16_t GPIO_MOSI, int16_t GPIO_SCLK, int16_t GPIO_CS)
{
	esp_err_t ret;

	ESP_LOGI(TAG, "GPIO_CS=%d",GPIO_CS);
	gpio_reset_pin( GPIO_CS );
	gpio_set_direction( GPIO_CS, GPIO_MODE_OUTPUT );
	gpio_set_level( GPIO_CS, 1 );
	

	ESP_LOGI(TAG, "GPIO_MOSI=%d",GPIO_MOSI);
	ESP_LOGI(TAG, "GPIO_SCLK=%d",GPIO_SCLK);
	ESP_LOGI(TAG, "GPIO_MISO=%d",GPIO_MISO);
	spi_bus_config_t buscfg = {
		.sclk_io_num = GPIO_SCLK,
		.mosi_io_num = GPIO_MOSI,
		.miso_io_num = GPIO_MISO,
		.max_transfer_sz =128,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};

	ret = spi_bus_initialize( HOST_ID, &buscfg, SPI_DMA_CH_AUTO);
	ESP_LOGI(TAG, "spi_bus_initialize=%d",ret);
	
	assert(ret==ESP_OK);

	spi_device_interface_config_t devcfg={
		/*.clock_speed_hz = SPI_MASTER_FREQ_1M,
		.spics_io_num = GPIO_CS,
		.queue_size = 7,
		.mode = 0,*/
		.flags = SPI_DEVICE_NO_DUMMY,
		.command_bits=0,
        	.address_bits=0,
        	.dummy_bits=0,
        	.clock_speed_hz=5000000,
        	.duty_cycle_pos=128,        //50% duty cycle
        	.mode=1,
        	.spics_io_num=GPIO_CS,
        	.cs_ena_posttrans=3,        //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        	.queue_size=7
	};
	
	spi_device_handle_t handle;
	ret = spi_bus_add_device( HOST_ID, &devcfg, &handle);
	ESP_LOGI(TAG, "spi_bus_add_device=%d",ret);	
	assert(ret==ESP_OK);	
	dev._handle = handle;	
	
}

