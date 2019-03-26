/*
 * uart3_dma.c
 *
 *  Created on: 20 Mar 2019
 *      Author: Stefan Jaritz
 */

/* how it works:
Goal let's setup the DMA and let it do the memcpy stuff and get only the notifications when the tasks are done.
rx:
- data flow:
	UART RX > UART->DR > DMA STREAM > DMA channel > ring buffer
- signals:
	DMA HTC: shoots when half of the ring buffer is filled
	DMA TC: shoots when the lasy ring buffer byte is filled and the address wraps
	UART idle: shoots when the burst is over
- Callbacks:
	rx callback with memory block reference on the rx'ed part of the message
tx:
- data flow:
	memory block > DMA STREAM > DMA channel > UART->DR
- signals:
	DMA TC: shoots when the DMA transfer is complete
	tx semaphore: is set if TC is successful

notes on rx:

The ring buffer is constantly filled with incoming data. the UART idle interrupt shoots when no data is rx/tx.
So it shoots after every burst of data. Without the TC, HTC interrupts the ring buffer will be filled with the complete burst.
The TC and HTC interrupts are used to process the rx data while receiving.

pros for hooking on the uart IDLE interrupt instead of the uart TX interrupt, is that in the worst case we will get an interrupt after
each rx byte. In the best and more common case we will have the interrupt after we got the full message/burst.

pros for enabling the DMA HTC and TC interrupts, are that we can process parts of the rx data while receiving them.

the core rx idea is from Tilen Majerle:
https://github.com/MaJerle/STM32_USART_DMA_RX/blob/master/projects/idle_line_irq_F4/Src/main.c
*/

#include "uart3_dma.h"

#include <zephyr.h>
#include <errno.h>
#include <misc/util.h>
#include <device.h>
#include <uart.h>

#include <init.h>
#include <soc.h>

#include <stm32f4xx_ll_dma.h>
#include <stm32f4xx_ll_usart.h>

#include <logging/log.h>
#include "../dbglog.h"
LOG_MODULE_REGISTER(UART3DMA, LOG_LEVEL_APP_UART3DMA);

#include <clock_control/stm32_clock_control.h>
#include <serial/uart_stm32.h>


#define UART3_DMA_RX DMA1
#define UART3_DMA_RX_STREAM LL_DMA_STREAM_1
#define UART3_DMA_RX_CHANNEL LL_DMA_CHANNEL_4

#define UART3_DMA_TX DMA1
#define UART3_DMA_TX_STREAM LL_DMA_STREAM_4
#define UART3_DMA_TX_CHANNEL LL_DMA_CHANNEL_7

#define UART3_INTR_PRIO 2

#define UART3_DMA_RX_BUFF_SIZE (512)

#define UART_DEV_CFG(dev)							\
	((const struct uart_stm32_config * const)(dev)->config->config_info)
#define UART_DEV_DATA(dev)							\
	((struct uart_stm32_data * const)(dev)->driver_data)
#define UART_STRUCT(dev)					\
	((USART_TypeDef *)(UART_DEV_CFG(dev))->uconf.base)

typedef struct uart3_dma {
	struct {
		u8_t buffer[UART3_DMA_RX_BUFF_SIZE];
		uart3_dma_fkt_rx_t cb;
	} rx;

	struct {
		struct k_sem txDone;
	} tx;
} uart3_dma_t;

static uart3_dma_t uart3dma;

static void uart3_dma_usart_rx_check(void) {
    static size_t old_pos;
    size_t pos;

    // Calculate current position in buffer
    pos = sizeof(uart3dma.rx.buffer) - LL_DMA_GetDataLength(UART3_DMA_RX, UART3_DMA_RX_STREAM);
    // Check change in received data
    if (pos != old_pos) {
    	// Current position is over previous one
        if (pos > old_pos) {
            // We are in "linear" mode
            // Process data directly by subtracting "pointers"
        	uart3dma.rx.cb(&uart3dma.rx.buffer[old_pos], pos - old_pos);
        } else {
            // We are in "overflow" mode
            // First process data to the end of buffer
        	uart3dma.rx.cb(&uart3dma.rx.buffer[old_pos], sizeof(uart3dma.rx.buffer) - old_pos);
            // Check and continue with beginning of buffer */
            if (pos) {
            	uart3dma.rx.cb(&uart3dma.rx.buffer[0], pos);
            }
        }
    }
    // Save current position as old
    old_pos = pos;

    // Check and manually update if we reached end of buffer
    if (old_pos == sizeof(uart3dma.rx.buffer)) {
        old_pos = 0;
    }
}

// rx dma isr handler
ISR_DIRECT_DECLARE(DMA1_Stream1_IRQHandler) {
	ISR_DIRECT_HEADER();
	// Check half-transfer complete interrupt
	if (LL_DMA_IsEnabledIT_HT(UART3_DMA_RX, UART3_DMA_RX_STREAM) && LL_DMA_IsActiveFlag_HT1(UART3_DMA_RX)) {
		LL_DMA_ClearFlag_HT1(UART3_DMA_RX);             // Clear half-transfer complete flag
		uart3_dma_usart_rx_check();                       // Check for data to process
	}

	// Check transfer-complete interrupt
	if (LL_DMA_IsEnabledIT_TC(UART3_DMA_RX, UART3_DMA_RX_STREAM) && LL_DMA_IsActiveFlag_TC1(UART3_DMA_RX)) {
		LL_DMA_ClearFlag_TC1(UART3_DMA_RX);             // Clear half-transfer complete flag
		uart3_dma_usart_rx_check();                       // Check for data to process
	}
	ISR_DIRECT_FOOTER(1);
	ISR_DIRECT_PM(); // PM done after servicing interrupt for best latency
	return 1; // We should check if scheduling decision should be made
}

// tx dma isr handler
ISR_DIRECT_DECLARE(DMA1_Stream4_IRQHandler) {
	ISR_DIRECT_HEADER();

	// transfer complete
	if (LL_DMA_IsActiveFlag_TC4(UART3_DMA_TX)) {
		LL_DMA_ClearFlag_TC4(UART3_DMA_TX);
		LL_DMA_DisableStream(UART3_DMA_TX, UART3_DMA_TX_STREAM);
		k_sem_give(&uart3dma.tx.txDone);
	}
	ISR_DIRECT_FOOTER(1);
	ISR_DIRECT_PM(); // power management
	return 1; // We should check if scheduling decision should be made
}

ISR_DIRECT_DECLARE(UART3_IRQHandler) {
	ISR_DIRECT_HEADER();
	/* Check for IDLE line interrupt */
	if (LL_USART_IsEnabledIT_IDLE(USART3) && LL_USART_IsActiveFlag_IDLE(USART3)) {
		LL_USART_ClearFlag_IDLE(USART3);        /* Clear IDLE line flag */
		uart3_dma_usart_rx_check();                       /* Check for data to process */
	}
	ISR_DIRECT_FOOTER(1);
	ISR_DIRECT_PM(); // power management
	return 1; // We should check if scheduling decision should be made
}

static uart3_dma_error_t uart3_dma_init (bool start, uart3_dma_fkt_rx_t rxcb) {
	if (true == start) {
		LOG_DBG("UART3 DMA on");
		uart3dma.rx.cb = rxcb;
		LL_DMA_EnableStream(UART3_DMA_RX, UART3_DMA_RX_STREAM);
		LL_USART_Enable(USART3);
	} else {
		LOG_DBG("UART3 DMA off");
		uart3dma.rx.cb = NULL;
		LL_USART_Disable(USART3);
		LL_DMA_DisableStream(UART3_DMA_RX, UART3_DMA_RX_STREAM);
		LL_DMA_DisableStream(UART3_DMA_TX, UART3_DMA_TX_STREAM);
	}
	return uart3_dma_error_success;
}

static uart3_dma_error_t uart3_dma_setupUart (u32_t baud_rate, uart3_parity_t p) {
	// UART3 is connected through AHP and APB1
	// SYSCLK = 96Mhz
	// AHP = 2
	// APB1 = 4
	// 96Mhz / 4 / 2 = 12MHz
	static const uint32_t clock_rate = 12000000;

	// switch off uart
	LL_USART_Disable(USART3);
	LL_DMA_DisableStream(UART3_DMA_RX, UART3_DMA_RX_STREAM);

	// set baudrate and partity
	LL_USART_SetBaudRate(USART3, clock_rate, LL_USART_OVERSAMPLING_16, baud_rate);
	LL_USART_SetParity (USART3,p);

	// reset dma and start uart
	memset(uart3dma.rx.buffer, 0, sizeof(uart3dma.rx.buffer));
	LL_DMA_EnableStream(UART3_DMA_RX, UART3_DMA_RX_STREAM);
	LL_USART_Enable(USART3);
	return uart3_dma_error_success;
}

static int uart3_dma_initilize(struct device *dev) {

	memset(&uart3dma, 0, sizeof(uart3dma));

	k_sem_init(&uart3dma.tx.txDone, 0, 1);

	// dma general setup
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	// rx dma setup
	// DMA 1 request mapping
	// USART3_RX @ Stream 1, channel 4
	LL_DMA_SetChannelSelection(UART3_DMA_RX, UART3_DMA_RX_STREAM, UART3_DMA_RX_CHANNEL);
	LL_DMA_SetDataTransferDirection(UART3_DMA_RX, UART3_DMA_RX_STREAM, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetStreamPriorityLevel(UART3_DMA_RX, UART3_DMA_RX_STREAM, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(UART3_DMA_RX, UART3_DMA_RX_STREAM, LL_DMA_MODE_CIRCULAR);
	LL_DMA_SetPeriphIncMode(UART3_DMA_RX, UART3_DMA_RX_STREAM, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(UART3_DMA_RX, UART3_DMA_RX_STREAM, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(UART3_DMA_RX, UART3_DMA_RX_STREAM, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(UART3_DMA_RX, UART3_DMA_RX_STREAM, LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_DisableFifoMode(UART3_DMA_RX, UART3_DMA_RX_STREAM);

	LL_DMA_SetPeriphAddress(UART3_DMA_RX, UART3_DMA_RX_STREAM, (u32_t)&USART3->DR);
	LL_DMA_SetMemoryAddress(UART3_DMA_RX, UART3_DMA_RX_STREAM, (u32_t)uart3dma.rx.buffer);
	LL_DMA_SetDataLength(UART3_DMA_RX, UART3_DMA_RX_STREAM, sizeof(uart3dma.rx.buffer));

	LL_DMA_EnableIT_HT(UART3_DMA_RX, UART3_DMA_RX_STREAM);
	LL_DMA_EnableIT_TC(UART3_DMA_RX, UART3_DMA_RX_STREAM);

	// USART3_TX Init
	// USART3_TX @ Stream 4, channel 7
	LL_DMA_SetChannelSelection(UART3_DMA_TX, UART3_DMA_TX_STREAM, UART3_DMA_TX_CHANNEL);
	LL_DMA_SetDataTransferDirection(UART3_DMA_TX, UART3_DMA_TX_STREAM, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetStreamPriorityLevel(UART3_DMA_TX, UART3_DMA_TX_STREAM, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(UART3_DMA_TX, UART3_DMA_TX_STREAM, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(UART3_DMA_TX, UART3_DMA_TX_STREAM, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(UART3_DMA_TX, UART3_DMA_TX_STREAM, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(UART3_DMA_TX, UART3_DMA_TX_STREAM, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(UART3_DMA_TX, UART3_DMA_TX_STREAM, LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_DisableFifoMode(UART3_DMA_TX, UART3_DMA_TX_STREAM);

	LL_DMA_SetPeriphAddress(UART3_DMA_TX, UART3_DMA_TX_STREAM, (u32_t)&USART3->DR);
	LL_DMA_SetMemoryAddress(UART3_DMA_TX, UART3_DMA_TX_STREAM, (u32_t)NULL);
	LL_DMA_SetDataLength(UART3_DMA_TX, UART3_DMA_TX_STREAM, 0);

	// enable transfer complete interrupt
	LL_DMA_EnableIT_TC(UART3_DMA_TX, UART3_DMA_TX_STREAM);

	IRQ_DIRECT_CONNECT(DMA1_Stream1_IRQn, UART3_INTR_PRIO, DMA1_Stream1_IRQHandler, 0);
	IRQ_DIRECT_CONNECT(DMA1_Stream4_IRQn, UART3_INTR_PRIO, DMA1_Stream4_IRQHandler, 0);
	irq_enable(DMA1_Stream1_IRQn);
	irq_enable(DMA1_Stream4_IRQn);

	// UART3 setup
	LL_USART_SetTransferDirection(USART3, LL_USART_DIRECTION_TX_RX);
	LL_USART_ConfigCharacter(USART3, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
	//LL_USART_SetHWFlowCtrl(USART3, LL_USART_HWCONTROL_RTS_CTS);
	LL_USART_SetHWFlowCtrl(USART3, LL_USART_HWCONTROL_NONE);
	LL_USART_ConfigAsyncMode(USART3);
	LL_USART_EnableDMAReq_RX(USART3);
	LL_USART_EnableDMAReq_TX(USART3);
	LL_USART_EnableIT_IDLE(USART3);

	IRQ_DIRECT_CONNECT(USART3_IRQn, UART3_INTR_PRIO, UART3_IRQHandler, 0);
	irq_enable(USART3_IRQn);

	uart3_dma_setupUart(115200, uart3_parity_none);

	LOG_INF("UART3 DMA initialised");
	return uart3_dma_error_success;
}

static uart3_dma_error_t uart3_dma_writeBuffer(u8_t * pB, size_t len, u32_t timeout) {
	LL_DMA_SetMemoryAddress(UART3_DMA_TX, UART3_DMA_TX_STREAM, (uint32_t)pB);
	LL_DMA_SetDataLength(UART3_DMA_TX, UART3_DMA_TX_STREAM, len);
	LL_DMA_EnableStream(UART3_DMA_TX, UART3_DMA_TX_STREAM);
	if (k_sem_take(&uart3dma.tx.txDone, timeout)) {
		return uart3_dma_error_timeout;
	}
	return uart3_dma_error_success;
}

static const uart3_dma_api_t uart3dmaAPI = {
	.init = uart3_dma_init,
	.setupUart = uart3_dma_setupUart,
	.writeBuffer = uart3_dma_writeBuffer,
};

DEVICE_AND_API_INIT(uart3dma, UART3_DMA_NAME, &uart3_dma_initilize,
	NULL, NULL,
	POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
	&uart3dmaAPI
	);

