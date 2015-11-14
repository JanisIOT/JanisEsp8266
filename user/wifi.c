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

#define RED 0
#define BLUE 1
#define GREEN 2


#define WIFI_APSSID	"JANIS103"
#define WIFI_APPASSWORD	"12345678"
#define PLATFORM_DEBUG	true

static char macaddr[6];

typedef struct {
	char ssid[31];
	char pwd[31];
	uint32_t valid;
} user_config_t;
#define PRIV_PARAM_START_SEC 0x3C
#define CONFIG_VALID 0xDEADBEEF
static ICACHE_FLASH_ATTR user_config_t user_config;
void  ICACHE_FLASH_ATTR wifi_event_cb(System_Event_t *evt) {
   static int serverinit=0;
   switch (evt->event_id) {
      case EVENT_SOFTAPMODE_STACONNECTED:
         //printf("EVENT_SOFTAPMODE_STACONNECTED:\n");
	 break;
      case EVENT_SOFTAPMODE_STADISCONNECTED:
         //printf("EVENT_SOFTAPMODE_STADISCONNECTED:\n");
         break;
      case EVENT_STAMODE_CONNECTED:
         //printf("EVENT_STAMODE_CONNECTED:\n");
	 break;
      case EVENT_STAMODE_DISCONNECTED:
	 //WS2812OutStatus(RED);
         printf("EVENT_STAMODE_DISCONNECTED:\n");
	 //erase_user_config();
 	 wifi_init();
	 break;
      case EVENT_STAMODE_AUTHMODE_CHANGE:
         //printf("EVENT_STAMODE_AUTHMODE_CHANGE:\n");
	 break;
      case EVENT_STAMODE_GOT_IP:
         printf("EVENT_STAMODE_GOT_IP:\n");
	 WS2812OutStatus(BLUE);
        save_user_config(&user_config);
	
	 break;
    case EVENT_MAX:
         //printf("EVENT_MAX:\n");
	break;
      default:
	//printf("Unknown event\n");
         break;
   }
}


void configure(char * ssid, char * pwd,uint8_t	isnew)
{
        char ssid_name[30];
	char password[30];
        struct station_config st_config;

	sprintf(ssid_name, "%s\0", ssid);
	sprintf(password,"%s\0",pwd);
	memset(st_config.ssid, 0, sizeof(st_config.ssid));
	memcpy(st_config.ssid, ssid_name, strlen(ssid_name));
	//st_config.ssid_len=strlen(ssid_name);
	memset(st_config.password, 0, sizeof(st_config.password));
	memcpy(st_config.password, password, strlen(password));
//	if(!isnew){
	     memset(user_config.ssid, 0, sizeof(user_config.ssid));
	     memcpy(user_config.ssid, ssid_name, strlen(ssid_name));
	     memset(user_config.pwd, 0, sizeof(user_config.pwd));
	     memcpy(user_config.pwd, password, strlen(pwd));
//	}
	printf("STA config: SSID: %s, PASSWORD: %s\r\n",st_config.ssid,st_config.password );
	printf("info: SSID: %s, PASSWORD: %s\r\n",ssid_name,password );
int ch;
	int ret = wifi_station_set_auto_connect(0);
        wifi_softap_dhcps_stop();
	vTaskDelay( 100 );
         wifi_set_opmode(NULL_MODE);
	vTaskDelay( 100 );
       //save_user_config(&user_config);
        wifi_set_opmode(STATION_MODE);
	wifi_set_event_handler_cb(wifi_event_cb);
        vTaskDelay( 100 );
        wifi_station_set_config_current(&st_config);
	vTaskDelay( 100 );
        wifi_station_connect();
	vTaskDelay( 100 );
        wifi_station_dhcpc_start();

	
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
void ICACHE_FLASH_ATTR erase_user_config(void)
{
    user_config.valid=0x00;
    memset(user_config.ssid, 0, sizeof(user_config.ssid));
    memset(user_config.pwd, 0, sizeof(user_config.pwd));
    spi_flash_erase_sector(PRIV_PARAM_START_SEC);
    //wifi_init();
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


void mode_info(){
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

void ICACHE_FLASH_ATTR config_wifi_new(void){
        wifi_softap_dhcps_stop();
	vTaskDelay( 100 );
        wifi_set_opmode(NULL_MODE);
	//vTaskDelay( 100 );
	wifi_set_opmode(SOFTAP_MODE);
	//vTaskDelay( 100 );
	struct softap_config apConfig;
	char ssid[8];
	//mode_info();
	memset(apConfig.ssid, 0, sizeof(apConfig.ssid));
	sprintf(ssid, "%s\0", WIFI_APSSID);
	memcpy(apConfig.ssid, ssid, strlen(ssid));
	printf("SSID %s\n",apConfig.ssid);
	wifi_set_event_handler_cb(wifi_event_cb);
	apConfig.authmode = AUTH_OPEN;
	apConfig.channel = 5;
	apConfig.ssid_len=strlen(ssid);
	apConfig.max_connection = 255;
	apConfig.ssid_hidden = 0;
	wifi_softap_set_config(&apConfig);
	//vTaskDelay( 100 );
	wifi_softap_dhcps_start();


}

void wifi_softap(void){

	if(read_user_config(&user_config)){
	    configure(user_config.ssid,user_config.pwd,0);
	}
	else{
	
             config_wifi_new();
	}
}

void wifi_init(void)
{
    erase_user_config();

    //vTaskDelay(100);
 //xTaskCreate(wifi, "ap", 512, NULL, tskIDLE_PRIORITY+2, NULL);
    wifi_softap();
}
