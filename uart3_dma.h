/*
 * uart3_dma.h
 *
 *  Created on: 20 Mar 2019
 *      Author: Stefan Jaritz
 */

#ifndef DRV_UART3_DMA_H_
#define DRV_UART3_DMA_H_

#include <device.h>
#include <stdbool.h>
#include <stm32f4xx_ll_usart.h>


#define UART3_DMA_NAME "UART3_DMA"

typedef enum uart3_parities{
	uart3_parity_none = LL_USART_PARITY_NONE,
	uart3_parity_even = LL_USART_PARITY_EVEN,
	uart3_parity_odd = LL_USART_PARITY_ODD
} uart3_parity_t;

typedef enum uart3_dma_errors {
	uart3_dma_error_success = 0,
	uart3_dma_error_devNotFound = 1,
	uart3_dma_error_configFailed = 2,
	uart3_dma_error_timeout = 3
} uart3_dma_error_t;

typedef void (* uart3_dma_fkt_rx_t) (u8_t * pD, size_t len);

typedef struct uart3_dma_api_s {
	uart3_dma_error_t (* init) (bool start, uart3_dma_fkt_rx_t rxcb);
	uart3_dma_error_t (* setupUart)(u32_t baud_rate, uart3_parity_t p);
	uart3_dma_error_t (* writeBuffer)(u8_t * pB, size_t len, u32_t timeout);
} uart3_dma_api_t;

#endif /* DRV_UART3_DMA_H_ */
