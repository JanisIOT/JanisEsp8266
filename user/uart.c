/*
 *  Copyright (C) 2014 -2016  Espressif System
 *
 */

#include "esp_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "uart.h"

enum {
    UART_EVENT_RX_CHAR,
    UART_EVENT_MAX
};

typedef struct _os_event_ {
    uint32 event;
    uint32 param;
} os_event_t;
volatile uint16_t uart_rx_overruns;
volatile uint16_t uart_rx_bytes;
xTaskHandle xUartTaskHandle;
xQueueHandle xQueueUart;

LOCAL STATUS
uart_tx_one_char(uint8 uart, uint8 TxChar)
{
    while (true) {
        uint32 fifo_cnt = READ_PERI_REG(UART_STATUS(uart)) & (UART_TXFIFO_CNT << UART_TXFIFO_CNT_S);

        if ((fifo_cnt >> UART_TXFIFO_CNT_S & UART_TXFIFO_CNT) < 126) {
            break;
        }
    }

    WRITE_PERI_REG(UART_FIFO(uart) , TxChar);
    return OK;
}

void 
uart1_write_char(char c)
{
    if (c == '\n') {
        uart_tx_one_char(UART1, '\r');
        uart_tx_one_char(UART1, '\n');
    } else if (c == '\r') {
    } else {
        uart_tx_one_char(UART1, c);
    }
}
void  uart1_puts(const char *str)
{
    while (*str)
    {
        uart1_write_char(*str++);
    }
}
LOCAL void ICACHE_FLASH_ATTR
uart0_write_char(char c)
{
    if (c == '\n') {
        uart_tx_one_char(UART0, '\r');
        uart_tx_one_char(UART0, '\n');
    } else if (c == '\r') {
    } else {
        uart_tx_one_char(UART0, c);
    }
}

LOCAL void  
uart_rx_intr_handler_ssc(void *p)
{
    /* uart0 and uart1 intr combine togther, when interrupt occur, see reg 0x3ff20020, bit2, bit0 represents
      * uart1 and uart0 respectively
      */
    uint8_t temp;
   signed portBASE_TYPE ret;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    os_event_t e;
    //portBASE_TYPE xHigherPriorityTaskWoken;

    uint8 RcvChar;
    uint8 uart_no = 0;

    if (UART_RXFIFO_FULL_INT_ST != (READ_PERI_REG(UART_INT_ST(uart_no)) & UART_RXFIFO_FULL_INT_ST)) {
        return;
    }

    WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_RXFIFO_FULL_INT_CLR);

    while (READ_PERI_REG(UART_STATUS(UART0)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S)) {
		temp = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF;
		ret = xQueueSendToBackFromISR(xQueueUart,&temp,&xHigherPriorityTaskWoken);
		if (ret != pdTRUE){
			uart_rx_overruns++;
		} 
		else{
			uart_rx_bytes++;
		}
	}
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    /*RcvChar = READ_PERI_REG(UART_FIFO(uart_no)) & 0xFF;


    e.event = UART_EVENT_RX_CHAR;
    e.param = RcvChar;

    //xQueueSendFromISR(xQueueUart, (void *)&e, &xHigherPriorityTaskWoken);
    xQueueSendToBackFromISR(xQueueUart, (void *)&e, &xHigherPriorityTaskWoken);

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    */
}



//=================================================================

void ICACHE_FLASH_ATTR 
UART_SetWordLength(UART_Port uart_no, UART_WordLength len)
{
    SET_PERI_REG_BITS(UART_CONF0(uart_no), UART_BIT_NUM, len, UART_BIT_NUM_S);
}

void ICACHE_FLASH_ATTR
UART_SetStopBits(UART_Port uart_no, UART_StopBits bit_num)
{
    SET_PERI_REG_BITS(UART_CONF0(uart_no), UART_STOP_BIT_NUM, bit_num, UART_STOP_BIT_NUM_S);
}

void ICACHE_FLASH_ATTR
UART_SetLineInverse(UART_Port uart_no, UART_LineLevelInverse inverse_mask)
{
    CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_LINE_INV_MASK);
    SET_PERI_REG_MASK(UART_CONF0(uart_no), inverse_mask);
}

void ICACHE_FLASH_ATTR 
UART_SetParity(UART_Port uart_no, UART_ParityMode Parity_mode)
{
    CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_PARITY | UART_PARITY_EN);

    if (Parity_mode == USART_Parity_None) {
    } else {
        SET_PERI_REG_MASK(UART_CONF0(uart_no), Parity_mode | UART_PARITY_EN);
    }
}

void ICACHE_FLASH_ATTR 
UART_SetBaudrate(UART_Port uart_no, uint32 baud_rate)
{
    uart_div_modify(uart_no, UART_CLK_FREQ / baud_rate);
}

//only when USART_HardwareFlowControl_RTS is set , will the rx_thresh value be set.
void ICACHE_FLASH_ATTR 
UART_SetFlowCtrl(UART_Port uart_no, UART_HwFlowCtrl flow_ctrl, uint8 rx_thresh)
{
    if (flow_ctrl & USART_HardwareFlowControl_RTS) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_U0RTS);
        SET_PERI_REG_BITS(UART_CONF1(uart_no), UART_RX_FLOW_THRHD, rx_thresh, UART_RX_FLOW_THRHD_S);
        SET_PERI_REG_MASK(UART_CONF1(uart_no), UART_RX_FLOW_EN);
    } else {
        CLEAR_PERI_REG_MASK(UART_CONF1(uart_no), UART_RX_FLOW_EN);
    }

    if (flow_ctrl & USART_HardwareFlowControl_CTS) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_UART0_CTS);
        SET_PERI_REG_MASK(UART_CONF0(uart_no), UART_TX_FLOW_EN);
    } else {
        CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_TX_FLOW_EN);
    }
}

void ICACHE_FLASH_ATTR 
UART_WaitTxFifoEmpty(UART_Port uart_no) //do not use if tx flow control enabled
{
    while (READ_PERI_REG(UART_STATUS(uart_no)) & (UART_TXFIFO_CNT << UART_TXFIFO_CNT_S));
}

void ICACHE_FLASH_ATTR
UART_ResetFifo(UART_Port uart_no)
{
    SET_PERI_REG_MASK(UART_CONF0(uart_no), UART_RXFIFO_RST | UART_TXFIFO_RST);
    CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_RXFIFO_RST | UART_TXFIFO_RST);
}

void ICACHE_FLASH_ATTR 
UART_ClearIntrStatus(UART_Port uart_no, uint32 clr_mask)
{
    WRITE_PERI_REG(UART_INT_CLR(uart_no), clr_mask);
}

void ICACHE_FLASH_ATTR 
UART_SetIntrEna(UART_Port uart_no, uint32 ena_mask)
{
    SET_PERI_REG_MASK(UART_INT_ENA(uart_no), ena_mask);
}

void ICACHE_FLASH_ATTR 
UART_intr_handler_register(void *fn)
{
    _xt_isr_attach(ETS_UART_INUM, fn,NULL);
}

void ICACHE_FLASH_ATTR 
UART_SetPrintPort(UART_Port uart_no)
{
    if (uart_no == 1) {
        os_install_putc1(uart1_write_char);
    } else {
        os_install_putc1(uart0_write_char);
    }
}

void ICACHE_FLASH_ATTR 
UART_ParamConfig(UART_Port uart_no,  UART_ConfigTypeDef *pUARTConfig)
{
    if (uart_no == UART1) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_U1TXD_BK);
    } else {
        PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
    }

    UART_SetFlowCtrl(uart_no, pUARTConfig->flow_ctrl, pUARTConfig->UART_RxFlowThresh);
    UART_SetBaudrate(uart_no, pUARTConfig->baud_rate);

    WRITE_PERI_REG(UART_CONF0(uart_no),
                   ((pUARTConfig->parity == USART_Parity_None) ? 0x0 : (UART_PARITY_EN | pUARTConfig->parity))
                   | (pUARTConfig->stop_bits << UART_STOP_BIT_NUM_S)
                   | (pUARTConfig->data_bits << UART_BIT_NUM_S)
                   | ((pUARTConfig->flow_ctrl & USART_HardwareFlowControl_CTS) ? UART_TX_FLOW_EN : 0x0)
                   | pUARTConfig->UART_InverseMask);

    UART_ResetFifo(uart_no);
}

void ICACHE_FLASH_ATTR 
UART_IntrConfig(UART_Port uart_no,  UART_IntrConfTypeDef *pUARTIntrConf)
{

    uint32 reg_val = 0;
    UART_ClearIntrStatus(uart_no, UART_INTR_MASK);
    reg_val = READ_PERI_REG(UART_CONF1(uart_no)) & ~((UART_RX_FLOW_THRHD << UART_RX_FLOW_THRHD_S) | UART_RX_FLOW_EN) ;

    reg_val |= ((pUARTIntrConf->UART_IntrEnMask & UART_RXFIFO_TOUT_INT_ENA) ?
                ((((pUARTIntrConf->UART_RX_TimeOutIntrThresh)&UART_RX_TOUT_THRHD) << UART_RX_TOUT_THRHD_S) | UART_RX_TOUT_EN) : 0);

    reg_val |= ((pUARTIntrConf->UART_IntrEnMask & UART_RXFIFO_FULL_INT_ENA) ?
                (((pUARTIntrConf->UART_RX_FifoFullIntrThresh)&UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S) : 0);

    reg_val |= ((pUARTIntrConf->UART_IntrEnMask & UART_TXFIFO_EMPTY_INT_ENA) ?
                (((pUARTIntrConf->UART_TX_FifoEmptyIntrThresh)&UART_TXFIFO_EMPTY_THRHD) << UART_TXFIFO_EMPTY_THRHD_S) : 0);

    WRITE_PERI_REG(UART_CONF1(uart_no), reg_val);
    CLEAR_PERI_REG_MASK(UART_INT_ENA(uart_no), UART_INTR_MASK);
    SET_PERI_REG_MASK(UART_INT_ENA(uart_no), pUARTIntrConf->UART_IntrEnMask);
}

LOCAL void
uart0_rx_intr_handler(void *para)
{
    /* uart0 and uart1 intr combine togther, when interrupt occur, see reg 0x3ff20020, bit2, bit0 represents
    * uart1 and uart0 respectively
    */
    uint8 RcvChar;
    uint8_t temp;
    uint8 uart_no = UART0;//UartDev.buff_uart_no;
    uint8 fifo_len = 0;
    uint8 buf_idx = 0;
    uint8 fifo_tmp[128] = {0};
 signed portBASE_TYPE ret;

    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    uint32 uart_intr_status = READ_PERI_REG(UART_INT_ST(uart_no)) ;

    while (uart_intr_status != 0x0) {
        if (UART_FRM_ERR_INT_ST == (uart_intr_status & UART_FRM_ERR_INT_ST)) {
            //printf("FRM_ERR\r\n");
            WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_FRM_ERR_INT_CLR);
        } else if (UART_RXFIFO_FULL_INT_ST == (uart_intr_status & UART_RXFIFO_FULL_INT_ST)) {
            //printf("full\r\n");
            fifo_len = (READ_PERI_REG(UART_STATUS(UART0)) >> UART_RXFIFO_CNT_S)&UART_RXFIFO_CNT;
            buf_idx = 0;

            while (buf_idx < fifo_len) {
                //uart_tx_one_char(UART0, READ_PERI_REG(UART_FIFO(UART0)) & 0xFF);
                buf_idx++;
		RcvChar = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF;
		ret = xQueueSendToBackFromISR(xQueueUart,&RcvChar,&xHigherPriorityTaskWoken);
		//os_putc(RcvChar);

		if (ret != pdTRUE){
			uart_rx_overruns++;
		} 
		else{
			uart_rx_bytes++;
		}
                //uart_tx_one_char(UART1, RcvChar);
               // uart_tx_one_char(UART0, RcvChar);
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

            }

            WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_FULL_INT_CLR);
        } else if (UART_RXFIFO_TOUT_INT_ST == (uart_intr_status & UART_RXFIFO_TOUT_INT_ST)) {
            //printf("tout\r\n");
	    //
            fifo_len = (READ_PERI_REG(UART_STATUS(UART0)) >> UART_RXFIFO_CNT_S)&UART_RXFIFO_CNT;
            buf_idx = 0;

            while (buf_idx < fifo_len) {
		    
                buf_idx++;
		RcvChar = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF;
		ret = xQueueSendToBackFromISR(xQueueUart,&RcvChar,&xHigherPriorityTaskWoken);
		//os_putc(RcvChar);

		if (ret != pdTRUE){
			uart_rx_overruns++;
		} 
		else{
			uart_rx_bytes++;
		}

                //uart_tx_one_char(UART1, RcvChar);
                //uart_tx_one_char(UART0, RcvChar);
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

            }

            WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_TOUT_INT_CLR);
        } else if (UART_TXFIFO_EMPTY_INT_ST == (uart_intr_status & UART_TXFIFO_EMPTY_INT_ST)) {
            //printf("empty\n\r");
            WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_TXFIFO_EMPTY_INT_CLR);
            CLEAR_PERI_REG_MASK(UART_INT_ENA(UART0), UART_TXFIFO_EMPTY_INT_ENA);
        } else {
            //skip
        }

        uart_intr_status = READ_PERI_REG(UART_INT_ST(uart_no)) ;
    }
}
/*
 * Read at most len-1 chars from the UART
 * last char will in buf will be a null
 */
char * ICACHE_FLASH_ATTR 
uart_gets(char *buf, int len)
{
	int i=0; 
	char *p=buf;
	int ch;
	while (i<len)
	{
		ch = uart_getchar();
		if (ch == '\r' || ch == '\n' || ch == -1 || ch == 0x03)
		{
			*p='\0';
			return buf;	
		}
		*p=(char)ch;
		os_putc(*p);
		p++;
		i++;
	}
	*p='\0';
	return buf;
}
void ICACHE_FLASH_ATTR 
uart_tx_buffer(uint8 *buf, uint16 len)
{
  uint16 i;

  for (i = 0; i < len; i++){
    uart_tx_one_char(UART0, buf[i]);
  }
}

char  ICACHE_FLASH_ATTR
uart_getchar_ms(int timeout)
{
	portBASE_TYPE ticks;
	unsigned char ch;
	if (timeout < 0)
	{
		ticks = portMAX_DELAY;
	}
	else
	{
		ticks = timeout / portTICK_RATE_MS; 
	}
	if ( xQueueReceive(xQueueUart, &ch, ticks) != pdTRUE)
	{
		return '.';
	}
	return ch;
}
int ICACHE_FLASH_ATTR 
uart_rx_available(void)
{
	return uxQueueMessagesWaiting(xQueueUart);
}


LOCAL void 
uart_task(void *pvParameters)
{
    unsigned char e;

    for (;;) {
        if (xQueueReceive(xQueueUart, &e, (portTickType)portMAX_DELAY)) {
                    printf("%c", e);
                    //printf("%s", " yuhu! ");
        }
    }

    vTaskDelete(NULL);
}


void ICACHE_FLASH_ATTR 
uart_init_new(void)
{
    UART_WaitTxFifoEmpty(UART0);
    UART_WaitTxFifoEmpty(UART1);

    UART_ConfigTypeDef uart_config;
    uart_config.baud_rate    = BIT_RATE_38400;
    uart_config.data_bits     = UART_WordLength_8b;
    uart_config.parity          = USART_Parity_None;
    uart_config.stop_bits     = USART_StopBits_1;
    uart_config.flow_ctrl      = USART_HardwareFlowControl_None;
    uart_config.UART_RxFlowThresh = 120;
    uart_config.UART_InverseMask = UART_None_Inverse;
    
    UART_ConfigTypeDef uart_config1;
    uart_config1.baud_rate    = BIT_RATE_19200;
    uart_config1.data_bits     = UART_WordLength_8b;
    uart_config1.parity          = USART_Parity_None;
    uart_config1.stop_bits     = USART_StopBits_1;
    uart_config1.flow_ctrl      = USART_HardwareFlowControl_None;
    uart_config1.UART_RxFlowThresh = 120;
    uart_config1.UART_InverseMask = UART_None_Inverse;


    xQueueUart = xQueueCreate(256, sizeof(char));
    UART_ParamConfig(UART0, &uart_config);
    UART_ParamConfig(UART1, &uart_config1);
    
    UART_IntrConfTypeDef uart_intr;
    uart_intr.UART_IntrEnMask = UART_RXFIFO_TOUT_INT_ENA | UART_FRM_ERR_INT_ENA | UART_RXFIFO_FULL_INT_ENA | UART_TXFIFO_EMPTY_INT_ENA;
    uart_intr.UART_RX_FifoFullIntrThresh = 10;
    uart_intr.UART_RX_TimeOutIntrThresh = 2;
    uart_intr.UART_TX_FifoEmptyIntrThresh = 20;
    UART_IntrConfig(UART0, &uart_intr);


    UART_SetPrintPort(UART1);
    UART_intr_handler_register(uart0_rx_intr_handler);
    ETS_UART_INTR_ENABLE();

    //_xt_isr_unmask(1<<ETS_UART_INUM);

    //xTaskCreate(uart_task, (uint8 const *)"uTask", 512, NULL, tskIDLE_PRIORITY + 2, &xUartTaskHandle);

    /*
    UART_SetWordLength(UART0,UART_WordLength_8b);
    UART_SetStopBits(UART0,USART_StopBits_1);
    UART_SetParity(UART0,USART_Parity_None);
    UART_SetBaudrate(UART0,74880);
    UART_SetFlowCtrl(UART0,USART_HardwareFlowControl_None,0);
    */

}



/*
 *
 





 *
 * */
