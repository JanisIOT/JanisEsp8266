/**
 * @brief       : coap server example
 *
 * @file        : coap_server.c
 * @author      : xukai
 * @version     : v0.0.1
 * @date        : 2015/5/11
 *
 * Change Logs  :
 *
 * Date        Version      Author      Notes
 * 2015/5/11    v0.0.1      xukai    some notes
 */


#include "esp_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <lwip/sockets.h> 
#include "coap_server.h"
//#include "coap.h"

#define PORT 5683

uint8_t buf[1024];
 uint8_t scratch_raw[1024];



static xTaskHandle xHandle = NULL;
static uint8_t serverUp;
static uint8_t curTask;

static ICACHE_FLASH_ATTR int fd;
static ICACHE_FLASH_ATTR struct sockaddr_in servaddr;

static coap_rw_buffer_t scratch_buf = {scratch_raw, sizeof(scratch_raw)};



ICACHE_FLASH_ATTR void setupSocket(void){
    fd= socket(AF_INET, SOCK_DGRAM, 0);
    printf("\nfile descriptor %d \r\n",fd);
    if (fd  == -1) {
        printf("\nSocket Error\r\n");
        return;
    }

}
ICACHE_FLASH_ATTR void bindSocket(void){
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(PORT);
    memset(&(servaddr.sin_zero), 0, sizeof(servaddr.sin_zero));
    if ((bind(fd, (struct sockaddr *)&servaddr, sizeof(servaddr))) == -1)
    {
       printf("Bind error\r\n");
       return;        
    }

}
ICACHE_FLASH_ATTR void  coap_server(void *pvParameters)
{
	printf("ami?");
    
	
    /*int fd;
    struct sockaddr_in servaddr, cliaddr;
    coap_rw_buffer_t scratch_buf = {scratch_raw, sizeof(scratch_raw)};

    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        printf("\nSocket Error\r\n");
        return;
    }
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(PORT);
    memset(&(servaddr.sin_zero), 0, sizeof(servaddr.sin_zero));
    if ((bind(fd, (struct sockaddr *)&servaddr, sizeof(servaddr))) == -1)
    {
       printf("Bind error\r\n");
 #ifdef MICROCOAP_DEBUG
        //shell_printf("\r\n--------------------\r\n");
        //shell_printf("Received Buffer: \r\n");
        //coap_dump(buf, n, true);
        //shell_printf("\r\n");
#endif

      return;        
    }*/
 	setupSocket();
	bindSocket();



        endpoint_setup();
    printf("Coap Server Start!%d\r\n",serverUp);
    while(serverUp)
    {
      // printf("im up");
        int n, rc;
        struct sockaddr_in  cliaddr;
        socklen_t len = sizeof(cliaddr);
        coap_packet_t pkt;

        n = recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *)&cliaddr, &len);
        //printf("\r\n--------------------\r\n");
        //printf("Received Buffer:%d \r\n",n);
        //coap_dump(buf, n, true);
        //printf("\r\n");
	if (0 != (rc = coap_parse(&pkt, buf, n)))
        {
            //printf("Bad packet rc\r\n");
	    //return;
            //shell_printf("Bad packet rc=%d\r\n", rc);
        }
        else
        {
            size_t rsplen = sizeof(buf);
            coap_packet_t rsppkt;
            //printf("Dump Packet: \r\n");
            //coap_dumpPacket(&pkt);
            coap_handle_req(&scratch_buf, &pkt, &rsppkt);

            if (0 != (rc = coap_build(buf, &rsplen, &rsppkt)))
            {
                 //printf("coap_build failed rc=%d\n", rc);
            }
            else
            {
                 //printf("coap_build success... sending rc=%d\n", rc);
                //shell_printf("--------------------\r\n");
        	//printf("Sending Buffer: \r\n");
               // coap_dump(buf, rsplen, true);
                //printf("\r\n");
                //coap_dumpPacket(&rsppkt);
                int ret=sendto(fd, buf, rsplen, 0, (struct sockaddr *)&cliaddr, sizeof(struct sockaddr));
		//printf(" send to results %d",ret);
            }
        }
    }
    
}
 ICACHE_FLASH_ATTR void coap_init(void){
	xHandle=NULL;
	serverUp=1;
	//	printf("starting coap server=%d, \n" ,serverUp);
       uint16_t t_id = rand();
        char tname[5];
	sprintf(tname, "coap%d",t_id );
        
	xTaskCreate(coap_server, tname, 512, NULL, 2, &xHandle);
}
 ICACHE_FLASH_ATTR void coap_restart( uint8_t killing )
 {
	if(killing){
	    serverUp=0;
	    //closesocket(fd);
	    printf("suspending...\n");
	    vTaskSuspend( xHandle );
	   //vTaskDelete(xHandle);
	}else{
	    //setupSocket();
           // bindSocket();
	    endpoint_setup();
	    serverUp=1;
	    printf("resuming...\n");
	    if(xHandle!=NULL){
            	vTaskResume( xHandle );
	    }
	    else{
	     	coap_init();
	    }
	}

     //if( xHandle != NULL )
     //{
//	serverUp=0;
//	printf("deleting task, server= %d, \n",serverUp);
       //  vTaskDelete( xHandle );
     //}
 }



void stopCoap(void *pvParameters){
//	if( xHandle != NULL )
  //   {
	serverUp=0;
	printf("deleting task, server= %d, \n",serverUp);
    //     vTaskDelete( xHandle );
    // }
//	vTaskDelete(NULL);

		
}
void  reSpawnCoap(void *pvParameters)
 {

     //if( xHandle != NULL )
     //{
	serverUp=1;
     //	coap_init();
	printf(" server UP= %d, \n",serverUp);
 //        vTaskDelete( NULL );
     //}
 }

