/*
 * wifi.c
 *
 *  Created on: Dec 14, 2014
 *      Author: Baoshi
 */

#include "esp_common.h"
#include "wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"


#define RED 0
#define BLUE 1
#define GREEN 2


#define WIFI_APSSID	"JANIS_105"
#define WIFI_APPASSWORD	"12345678"
#define PLATFORM_DEBUG	true

static char macaddr[6];

const portTickType xDelay = 100 / portTICK_RATE_MS;
typedef struct {
	char ssid[31];
	char pwd[31];
	uint32_t valid;
} user_config_t;
static ICACHE_FLASH_ATTR char ssid_name[30];
static ICACHE_FLASH_ATTR char password[30];
static ICACHE_FLASH_ATTR uint8_t restartActive;

void ICACHE_FLASH_ATTR config_wifi_new(void);
void ICACHE_FLASH_ATTR createTimerWifi(void);
#define PRIV_PARAM_START_SEC 0x3C
#define CONFIG_VALID 0xDEADBEEF
static ICACHE_FLASH_ATTR user_config_t user_config;
void  ICACHE_FLASH_ATTR wifi_event_cb(System_Event_t *evt) {
   static int serverinit=0;
   switch (evt->event_id) {
      case EVENT_SOFTAPMODE_STACONNECTED:
         printf("EVENT_SOFTAPMODE_STACONNECTED:\n");
	 break;
      case EVENT_SOFTAPMODE_STADISCONNECTED:
         printf("EVENT_SOFTAPMODE_STADISCONNECTED:\n");
         break;
      case EVENT_STAMODE_CONNECTED:
         printf("EVENT_STAMODE_CONNECTED:\n");
	 break;
      case EVENT_STAMODE_DISCONNECTED:
	 WS2812OutStatus(RED);
         printf("EVENT_STAMODE_DISCONNECTED:\n");
	 //erase_user_config();
	 //
 	 //wifi_init();
	if(!read_user_config(&user_config))
		config_wifi_new();
	 //	createTimerWifi();
	 
	break;
      case EVENT_STAMODE_SCAN_DONE:
         printf("EVENT_STAMODE_SCAN_DONE\n");
	 break;
      case EVENT_STAMODE_AUTHMODE_CHANGE:
         printf("EVENT_STAMODE_AUTHMODE_CHANGE:\n");
	 break;
      case EVENT_STAMODE_GOT_IP:
         printf("EVENT_STAMODE_GOT_IP:\n");
	 WS2812OutStatus(GREEN);
        save_user_config(&user_config);
	coap_restart(0);
        //coap_init();
	 break;
    case EVENT_MAX:
         //printf("EVENT_MAX:\n");
	break;
      default:
	//printf("Unknown event\n");
         break;
   }
}
void ICACHE_FLASH_ATTR vTimerSCallback( xTimerHandle pxTimer ){  
	configure(); 
}
void ICACHE_FLASH_ATTR vTimerSACallback( xTimerHandle pxTimer ){  
	config_wifi_new(); 
}
void ICACHE_FLASH_ATTR createTimerWifi(void){
	if(!restartActive){
	    restartActive=1;
             xTimerHandle wifiTimer2=xTimerCreate("s",200,pdFALSE, (void *)1,vTimerSACallback);
           if( xTimerStart( wifiTimer2,0) == pdPASS )  {
              printf("timer SOft AP babe!!!!! \n");  
           }


	}


}
void ICACHE_FLASH_ATTR createTimer(char * ssid, char * pwd){
	sprintf(ssid_name, "%s\0", ssid);
	sprintf(password,"%s\0",pwd);
     xTimerHandle wifiTimer=xTimerCreate("r",200,pdFALSE, (void *)1,vTimerSCallback);
       if( xTimerStart( wifiTimer,0) == pdPASS )  {
          printf("timer STATION babe!!!!! \n");  
     }



}

void ICACHE_FLASH_ATTR configure(void){
	restartActive=0;
        struct station_config st_config;
       	memset(st_config.ssid, 0, sizeof(st_config.ssid));
	memcpy(st_config.ssid, ssid_name, strlen(ssid_name));
	//st_config.ssid_len=strlen(ssid_name);
	memset(st_config.password, 0, sizeof(st_config.password));
	memcpy(st_config.password, password, strlen(password));
//	if(!isnew){
	     memset(user_config.ssid, 0, sizeof(user_config.ssid));
	     memcpy(user_config.ssid, ssid_name, strlen(ssid_name));
	     memset(user_config.pwd, 0, sizeof(user_config.pwd));
	     memcpy(user_config.pwd, password, strlen(password));
//	}
	printf("STA config: SSID: %s, PASSWORD: %s\r\n",st_config.ssid,st_config.password );
	printf("info: SSID: %s, PASSWORD: %s\r\n",ssid_name,password );
int ch;
        wifi_softap_dhcps_stop();
	wifi_set_event_handler_cb(wifi_event_cb);
	//vTaskDelay( xDelay );
         wifi_set_opmode(NULL_MODE);
	vTaskDelay( xDelay );
       //save_user_config(&user_config);
        wifi_set_opmode(STATION_MODE);
        //vTaskDelay( xDelay );
        wifi_station_set_config_current(&st_config);
	
//	int ret = wifi_station_set_auto_connect(1);
	//vTaskDelay( xDelay );
        wifi_station_connect();
	//vTaskDelay( xDelay );
        wifi_station_dhcpc_start();
	//vTaskDelay( 100 );
	//vTaskDelay( 100 );
	

	coap_restart(1);
}



ICACHE_FLASH_ATTR int
save_user_config(user_config_t *config)
{
	config->valid = CONFIG_VALID;
    spi_flash_erase_sector(PRIV_PARAM_START_SEC);
    uint32* data=(uint32*)config; 
    spi_flash_write((PRIV_PARAM_START_SEC) * SPI_FLASH_SEC_SIZE,
                        data, sizeof(user_config_t));
	return 0;
}
void ICACHE_FLASH_ATTR config_wifi_new(void){
        wifi_softap_dhcps_stop();
	//vTaskDelay( xDelay );
        wifi_set_opmode(NULL_MODE);
	//vTaskDelay( xDelay );
	wifi_set_opmode(SOFTAP_MODE);
	//vTaskDelay( xDelay );
	struct softap_config apConfig;
	char ssid[8];
	wifi_set_event_handler_cb(wifi_event_cb);
	//mode_info();
	memset(apConfig.ssid, 0, sizeof(apConfig.ssid));
	sprintf(ssid, "%s\0", WIFI_APSSID);
	memcpy(apConfig.ssid, ssid, strlen(ssid));
	printf("SSID %s\n",apConfig.ssid);
	apConfig.authmode = AUTH_OPEN;
	apConfig.channel = 5;
	apConfig.ssid_len=strlen(ssid);
	apConfig.max_connection = 255;
	apConfig.ssid_hidden = 0;
	wifi_softap_set_config(&apConfig);
	//vTaskDelay( xDelay );
	wifi_softap_dhcps_start();
	//vTaskDelay( xDelay*10 );
	coap_restart(0);
}

void ICACHE_FLASH_ATTR erase_user_config(void)
{
    user_config.valid=0x00;
    memset(user_config.ssid, 0, sizeof(user_config.ssid));
    memset(user_config.pwd, 0, sizeof(user_config.pwd));
    spi_flash_erase_sector(PRIV_PARAM_START_SEC);
    //wifi_init();
   // config_wifi_new();

}

ICACHE_FLASH_ATTR int
read_user_config(user_config_t *config)
{
	printf ("size to read is %d\n\r", sizeof(user_config_t));
    spi_flash_read((PRIV_PARAM_START_SEC) * SPI_FLASH_SEC_SIZE,
                        (uint32 *)config, sizeof(user_config_t));
	printf ("valid flag is 0x%x\r\n", config->valid);
	if (config->valid != CONFIG_VALID)
	{
		config->ssid[0] = '\0';
		config->pwd[0] = '\0';
		return 0;
	}
	else{
	   return 1; 
	}
	return 0;
}


void ICACHE_FLASH_ATTR mode_info(){
	int CUR_MODE=wifi_get_opmode();
	switch(CUR_MODE){
	    case NULL_MODE:
	       printf("NULL\n");
		break;
	    case STATION_MODE:
		printf("mode selected: STATION\n");
		break;
	    case SOFTAP_MODE:
		printf("mode selected SOFTAP\n");
		break;
	    case STATIONAP_MODE:
		printf("mode selected STATIONAP\n");
		break;
	    case MAX_MODE:
		printf("mode selected MAX\n");
	        break;
	}

}


void ICACHE_FLASH_ATTR wifi_softap(void){

	if(read_user_config(&user_config)){
		printf("user config restored\n");
		createTimer(user_config.ssid,user_config.pwd);
	}
	else{
	printf("wifi new\n");
             config_wifi_new();
	//	createTimerWifi();
	}
}

void ICACHE_FLASH_ATTR wifi_init(void)
{
    erase_user_config();

    vTaskDelay(xDelay);
 //xTaskCreate(wifi, "ap", 512, NULL, tskIDLE_PRIORITY+2, NULL);
    wifi_softap();
}
