#include <lwip/netdb.h>
#include <lwip/sockets.h>
#include "esp_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "coap_cli.h"


// CoAP Message Setup
#define MSG_BUF_LEN 64
uint8_t msg_send_buf[MSG_BUF_LEN];
coap_pdu msg_send = {msg_send_buf, 0, 64};
uint8_t msg_recv_buf[MSG_BUF_LEN];
coap_pdu msg_recv = {msg_recv_buf, 0, 64};
// Helpers
void hex_dump(char* bytes, size_t len)
{
  size_t i, j;
  for (i = 0; i < len; i+=16){
    printf("  0x%.3zx    ", i);
    for (j = 0; j < 16; j++){
      if (i+j < len)
        printf("%c ", bytes[i+j]);
      else
        printf("%s ", "--");
    }
    printf("   %.*s\n", (int)(16 > len-i ? len-i : 16), bytes+i);
  }
}

void coap_pretty_print(coap_pdu *pdu)
{
  if (coap_validate_pkt(pdu) == 0){
    printf("Found Valid Coap Packet\n");
  }

  hex_dump((char*)pdu->buf, pdu->len);
}
void ICACHE_FLASH_ATTR coap_send(void *pvParameters)
//int coap_send(void)
{
    char *url = "coap.me";
    int port = 5683;
    
    uint16_t message_id_counter = rand();

    printf("%d,%d",message_id_counter, port);
    
    int sock;
    int bytes_sent;
    int bytes_recv;
    
    struct hostent *host;
    struct sockaddr_in server_addr;
    
    /* 通过函数入口参数url获得host地址*/
    host= (struct hostent *) gethostbyname(url);
    printf("here");
    /* 创建一个socket，类型是SOCK_DGRAM，UDP类型 */
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        printf("Socket Error\n");
    }
    printf("here2");
    /* 初始化预连接的服务端地址 */
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr = *((struct in_addr *)host->h_addr);
    memset(&(server_addr.sin_zero), 0, sizeof(server_addr.sin_zero));
    
    printf("--------------------\r\n");
    
    // Build Message
    coap_init_pdu(&msg_send);
    //memset(msg_send, 0, msg_send_len);
    coap_set_version(&msg_send, COAP_V1);
    coap_set_type(&msg_send, CT_CON);
    coap_set_code(&msg_send, CC_GET); // or POST to write
    coap_set_mid(&msg_send, message_id_counter++);
    // coap_set_token(&msg_send, rand(), 2);
    coap_add_option(&msg_send, CON_URI_PATH, (uint8_t*)"hello", strlen("hello"));
    // coap_add_option(&msg_send, CON_URI_PATH, (uint8_t*)alias, strlen(alias));
    // coap_add_option(&msg_send, CON_URI_QUERY, (uint8_t*)cik, strlen(cik));
    // to write, set payload:
    //coap_set_payload(msg_send, &msg_send_len, MSG_BUF_LEN, (uint8_t*)"99", 2);
    
    // Send Message
    if ((bytes_sent = sendto(sock, msg_send.buf, msg_send.len, 0, 
                             (struct sockaddr *)&server_addr, sizeof(struct sockaddr))) == -1)
    {
        printf("Failed to Send Message\r\n");
    }
    else{
	printf("bytes %d\n",bytes_sent);	
    }

     socklen_t len = sizeof(server_addr);
    
     bytes_recv = recvfrom(sock, (void *)msg_recv.buf, msg_recv.max, 0, (struct sockaddr *)&server_addr,&len);
   
    if (bytes_recv < 0) {
      printf("game over baby\n");
    }
    else{
	printf("yuhu\n");
    }

    msg_recv.len = bytes_recv;

    if(coap_validate_pkt(&msg_recv) == CE_NONE)
    {
      printf("Got Valid CoAP Packet\n");
      if(coap_get_mid(&msg_recv) == coap_get_mid(&msg_send) &&
         coap_get_token(&msg_recv) == coap_get_token(&msg_send))
      {
        printf("Is Response to Last Message\n");
        coap_pretty_print(&msg_recv);
      }
    }else{
      printf("Received %zi Bytes, Not Valid CoAP\n", msg_recv.len);
    }    
  vTaskDelete(NULL);	
}
void coap_client(void){
	xTaskCreate(coap_send, "send", 512, NULL, 100, NULL);
}
