# STM32F4 UART3 rx and tx via DMA

Simple ZephyrOS device driver using UART3 and the DMA1 (channel 4 & 7).

Main features:
- getting bursts of uart bytes without knowing the amount of data to be received before.
- sending uart data via DMA to reduce MCU load

The RX is received via DMA into a ring buffer. After the finish of the message burst or when the ringbuffer is half full(HTC)/full(TC) the rx callback is called. This callback is not blocking the rx/tx.

The TX is done via DMA transfer. The write function sets up the DMA transfer and waits for the complete semaphore. This semaphore is given in the DMA transfer complete(TC) interrupt.

Keeping in mind that vector based processing is the most efficent way, the RX ring buffer is set to 512 Bytes. Means that in the worst case the rx callback has to process 256 Bytes. But processing 256 Bytes than having 256 uart tx interrupts is much more efficent.

sources:
- The core rx idea is from Tilen Majerle and can be found here: [uart3 rx dma](https://github.com/MaJerle/STM32_USART_DMA_RX/blob/master/projects/idle_line_irq_F4/Src/main.c)
- stm32f412 datasheet[(RM0402)](https://www.st.com/content/ccc/resource/technical/document/reference_manual/group0/4f/7b/2b/bd/04/b3/49/25/DM00180369/files/DM00180369.pdf/jcr:content/translations/en.DM00180369.pdf)
- stm32HAL LL documentation [(UM1725)](https://www.st.com/content/ccc/resource/technical/document/user_manual/2f/71/ba/b8/75/54/47/cf/DM00105879.pdf/files/DM00105879.pdf/jcr:content/translations/en.DM00105879.pdf)
