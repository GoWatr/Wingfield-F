/*******************************************************************************
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/


// Smart Valve

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "global.h"
#include "spi.h"
#include "board.h"
#include "transducer.h"


#include <stdarg.h>
#include <string.h>
#include <stdio.h>


max3510x_t g_max3510x;
max3510x_t g_max35103;

typedef struct spim_req spim_req_t;

extern MAX35101_t dev;
extern max3510x_tof_results_t 	s_tof_results;

struct spim_req {
    uint8_t             ssel;       /**< Number of the Slave Select to use. */
    uint8_t             deass;      /**< Set to de-assert slave select at the completions of the transaction.*/
    const uint8_t       *tx_data;   /**< Pointer to a buffer to transmit data from. */
    uint8_t             *rx_data;   /**< Pointer to a buffer to store data received. */
    unsigned            len;        /**< Number of bytes to send from the \p tx_data buffer. */
    unsigned            read_num;   /**< Number of bytes read and stored in \p rx_data buffer. */
    unsigned            write_num;  /**< Number of bytes sent from the \p tx_data buffer, this will be filled by the driver after up to \p len bytes have been transmitted. */
};

void board_init(void)
{
	uint8_t i;
  	uint16_t val;
	uint8_t config_buf[25] = {0x00, 0x0f, 0x10, 0xc3, 0x00, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 
	0x00, 0x3c, 0x00, 0x3c, 0xf1, 0x00, 0x03, 0xc8, 0x00, 0xd0, 0x0e, 0x50, 0x00, 0x00};
	const uint16_t board_rev_addr = (MAX3510X_FLASH_BLOCK_SIZE_WORDS*MAX3510X_FLASH_BLOCK_COUNT)-2;
	
	board_wait_ms(10);									// wait for power to settle
	transducer_init();
		
	gpio_set_level(GPIO_OUTPUT_MAX35101_RST, 1);	
	board_wait_ms(100);
	
	max3510x_wait_for_reset_complete( &g_max3510x );
	board_wait_ms(200);
	
	
	//max3510x_init(&g_max35103, transducer_config() );	// configure the MAX3510x according per transducer requirements
	
	max3510x_write_register_multiple(&g_max35103 , 0x38, config_buf, 25);

	/*max3510x_write_register_var(&g_max35103 , 0x38, 0x0f10);
	max3510x_write_register_var(&g_max35103 , 0x39, 0xC300);
	max3510x_write_register_var(&g_max35103 , 0x3A, 0x0708);
	max3510x_write_register_var(&g_max35103 , 0x3B, 0x090a);
	max3510x_write_register_var(&g_max35103 , 0x3C, 0x0b0c);
	max3510x_write_register_var(&g_max35103 , 0x3D, 0x003c);
	max3510x_write_register_var(&g_max35103 , 0x3E, 0x003c);
	max3510x_write_register_var(&g_max35103 , 0x3F, 0xf100);
	max3510x_write_register_var(&g_max35103 , 0x40, 0x0388);
	max3510x_write_register_var(&g_max35103 , 0x41, 0x00d0);
	max3510x_write_register_var(&g_max35103 , 0x42, 0x0e50);
	max3510x_write_register_var(&g_max35103 , 0x43, 0x0000);*/
	
	for(i=0;i<12;i++){
	val = max3510x_read_register(&g_max35103 , 0x38 + i);
	printf("val=%04x \r\n", val);
	}

	board_wait_ms(1);			// give the MAX35103 some time to init
	
	max3510x_ldo(&g_max35103,max3510x_ldo_mode_on);		
	while((max3510x_interrupt_status( &g_max35103 ) & 0x0400)!=0x0400);	
	max3510x_erase_flash_block( &g_max35103, board_rev_addr);
	while(max3510x_interrupt_status( &g_max35103 )!=0x80);
	max3510x_write_flash( &g_max35103, board_rev_addr , 0x4243);
	while(max3510x_interrupt_status( &g_max35103 )!=0x80);
	uint16_t sig = max3510x_read_flash( &g_max35103, board_rev_addr);
	printf("sig = %04x\r\n", sig);

	max3510x_ldo(&g_max35103,max3510x_ldo_mode_off);
	while((max3510x_interrupt_status( &g_max35103 ) & 0x0400)!=0x0000);
		
//	max3510x_opcode(&g_max35103,0x06);
//	while(max3510x_interrupt_status( &g_max35103 )!=0x80);
}



void board_wait_ms( uint32_t ms )
{
	uint32_t i;
	for(i=0;i<=ms*20000;i++);
}

void max3510x_spi_xfer( max3510x_t p, void *pv_in, const void *pv_out, uint8_t count )
{
	// used by the MAX3510x module to interface with the hardware
	uint8_t txbuf[128], rxbuf[128];
	memset(txbuf, 0, sizeof(txbuf));
	memset(rxbuf, 0, sizeof(rxbuf));
	
	spi_transaction_t SPITransaction;
	esp_err_t ret;

	uint8_t i;
	spim_req_t req;
	req.ssel = 0;
	req.deass = 1;
	req.tx_data = pv_out;
	req.rx_data = pv_in;
	req.len = count;

	//ESP_LOGI(TAG,"txbuf = ");
	for(i=0;i<req.len;i++){
		txbuf[i] =req.tx_data[i];
	//	ESP_LOGI(TAG,"%02x ", txbuf[i]);
	}
	//ESP_LOGI(TAG,"\r\n");
	
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = req.len * 8;
	SPITransaction.rxlength = req.len * 8;
	SPITransaction.tx_buffer = txbuf;
	SPITransaction.rx_buffer = rxbuf;

	//board_wait_ms(5);
	ret = spi_device_transmit( dev._handle, &SPITransaction );
	assert(ret==ESP_OK); 

	//ESP_LOGI(TAG,"rxbuf = ");
	for(i=0;i<req.len;i++){
		req.rx_data[i] = rxbuf[i];
	//	ESP_LOGI(TAG,"%02x, %02x", req.len ,req.rx_data[i]);
	}
	//ESP_LOGI(TAG,"\r\n");
	
}	


void max3510x_spi_xfer_var( max3510x_t p, uint8_t *rxreg, uint8_t *txreg, uint8_t count )
{
	// used by the MAX3510x module to interface with the hardware
	uint8_t txbuf[128], rxbuf[128];
	memset(txbuf, 0, sizeof(txbuf));
	memset(rxbuf, 0, sizeof(rxbuf));
	
	spi_transaction_t SPITransaction;
	esp_err_t ret;

	uint8_t i;
	spim_req_t req;
	req.ssel = 0;
	req.deass = 1;
	req.len = count;

	//ESP_LOGI(TAG,"txbuf = ");
	for(i=0;i<req.len;i++){
		txbuf[i] =txreg[i];
		//ESP_LOGI(TAG,"%02x ", txbuf[i]);
	}
	//ESP_LOGI(TAG,"\r\n");
	

	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = req.len * 8;
	SPITransaction.rxlength = req.len * 8;
	SPITransaction.tx_buffer = txbuf;
	SPITransaction.rx_buffer = rxbuf;

	//board_wait_ms(5);
	ret = spi_device_transmit( dev._handle, &SPITransaction );
	assert(ret==ESP_OK); 

	//ESP_LOGI(TAG,"rxbuf = ");
	for(i=0;i<req.len;i++){
		rxreg[i] = rxbuf[i];
		//ESP_LOGI(TAG,"%02x, %02x", req.len ,rxreg[i]);
	}
	//ESP_LOGI(TAG,"\r\n");
	
}	


bool board_flash_write( const void *p_data, uint16_t size )
{
	return false;
}

void board_flash_read( void *p_data, uint16_t size )
{
}


void board_relay( bool on )
{

}

void board_led( bool on )
{
	 
}
