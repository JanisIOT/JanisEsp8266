/*
	The obligatory blinky demo
	Blink an LED on GPIO pin 2
*/


// see eagle_soc.h for these definitions
#define LED_GPIO 2
#define LED_GPIO_MUX PERIPHS_IO_MUX_GPIO2_U
#define LED_GPIO_FUNC FUNC_GPIO2


#include "esp_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ws2812.h"
#include <gpio.h>


#include "freertos/semphr.h"
#include "freertos/portmacro.h"

#include "uart.h"

#define PORT 5683

uint8_t buf[1024];
uint8_t scratch_raw[1024];


/*
void ICACHE_FLASH_ATTR uart_init(void)
{
    uart_param_t uart_param;
    uart_param.uart_baud_rate = UART_BAUD_RATE_38400;
    uart_param.uart_xfer_bit = UART_XFER_8_BIT;
    uart_param.uart_parity_mode = UART_PARITY_NONE;
    uart_param.uart_stop_bit = UART_1_STOP_BIT;
    uart_param.uart_flow_ctrl = UART_NONE_FLOW_CTRL;
    uart0_init(&uart_param);
    uart1_init();
}
*/
/*
Connected to camera. Operating with 38400 baudrate.
Version: "VC0703 1.00"
Downsize: 17
Compression-level: 53
Image-size: 320x240
Motion detector state: disabled

LOCAL void ICACHE_FLASH_ATTR shell_task(void *pvParameters)
{
    os_event_t e;
    char ch;
    char debug[15];
    for (;;)
    {
        if (xQueueReceive(xQueueUart, (void *)&e, (portTickType)portMAX_DELAY))
        {
            switch (e.event)
            {
                case UART_EVENT_RX_CHAR:
                    ch = (char)e.param;
		    sprintf(debug, "debug v=%c\n", ch);
		    uart1_puts("tell me\n");
		    uart1_puts(debug);
                   // shell_process_char(ch);
		    
                    break;
                default:
		    uart1_puts("die");
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}
*/

void ICACHE_FLASH_ATTR
camUartCb(char *buf, int length) {
	// push the buffer into the microcontroller console
	int i=0;
	uart1_puts(buf);
}



/*
 * This is entry point for user code
 */
void ICACHE_FLASH_ATTR
user_init(void){
	//uart_init(38400, 19200);
	// init the wifi-serial transparent bridge (port 23)
	//uart_add_recv_cb(&camUartCb);

    //uart_set_baud(UART0,BIT_RATE_115200);
    //
    //uart_rx_init();
    //uart_init();
    //uart_init_new();
    wifi_init();
    coap_init();
    leds_init();

    //coap_server();
    //xTaskCreate(coap_server, "ap", 512, NULL, 2, NULL);
   //xTaskCreate(shell_task, "shell", 256, NULL, tskIDLE_PRIORITY + 2, NULL);

    //xTaskCreate(shell_task, "shell", 512, NULL, tskIDLE_PRIORITY + 2, NULL);
    //shell_do_draw();
   
}






