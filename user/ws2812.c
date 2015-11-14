#include "esp_common.h"
#include "gpio.h"
#include "c_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static uint32_t WSGPIO = 12;

#define LED_GPIO_12 12
#define LED_GPIO_MUX_12 PERIPHS_IO_MUX_MTDI_U
#define LED_GPIO_FUNC_12 FUNC_GPIO12
#define OUT_ON 1
#define OUT_OFF 0

//static uint32_t WSGPIO = 1;

//#define LED_GPIO_1 1
//#define LED_GPIO_MUX_1 PERIPHS_IO_MUX_MTDI_U
//#define LED_GPIO_FUNC_1 FUNC_GPIO1
//#define OUT_ON 1
//#define OUT_OFF 0


static int nleds = 16;
static bool tainted=false;

static uint8_t r=255;
static uint8_t g=255;
static uint8_t b=255;

#define F_CPU CPU_CLK_FREQ
#define CYCLES_800_T0H (F_CPU / 2500000)  // 0.4us
#define CYCLES_800_T1H (F_CPU / 1250000)  // 0.8us
#define CYCLES_ERROR (F_CPU / 5000000)    // 0.2us
#define CYCLES_800 (F_CPU / 800000)       // 1.25us per bit



#define RED 0
#define BLUE 1
#define GREEN 2
#define YELOW 2
//#define GPIO_OUTPUT_SET(gpio_no, bit_value) \
	gpio_output_set(bit_value<<gpio_no, ((~bit_value)&0x01)<<gpio_no, 1<<gpio_no,0)


static uint32_t _getCycleCount(void) __attribute__((always_inline));

static inline uint32_t _getCycleCount(void) {
  uint32_t ccount;
  __asm__ __volatile__("rsr %0,ccount" : "=a"(ccount));
  return ccount;
}

// Read Set Interrupt Level
#define RSIL(r) __asm__ __volatile__("rsil %0,15 ; esync" : "=a"(r))

// Write Register Processor State
#define WSR_PS(w) __asm__ __volatile__("wsr %0,ps ; esync" ::"a"(w) : "memory")

static inline uint32_t esp8266_enter_critical() {
  uint32_t state;
  RSIL(state);
  return state;
}

static inline void esp8266_leave_critical(uint32_t state) {
	WSR_PS(state); 
}


static void  ICACHE_FLASH_ATTR SEND_WS_0()
{
	uint8_t time;
	time = 4; while(time--) GPIO_REG_WRITE( GPIO_ID_PIN(WSGPIO), 1 );
	time = 9; while(time--) GPIO_REG_WRITE( GPIO_ID_PIN(WSGPIO), 0 );

}

static void ICACHE_FLASH_ATTR SEND_WS_1()
{
	uint8_t time; 
        time = 8; while(time--) GPIO_REG_WRITE( GPIO_ID_PIN(WSGPIO), 1 );
	time = 6; while(time--) GPIO_REG_WRITE( GPIO_ID_PIN(WSGPIO), 0 );

}

static void ICACHE_FLASH_ATTR send_ws_0(uint8_t gpio){
     uint8_t i;
     i = 4; while (i--) GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1 << gpio);
     i = 9; while (i--) GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1 << gpio);
}
static void ICACHE_FLASH_ATTR send_ws_1(uint8_t gpio){
uint8_t i;
    i = 8; while (i--) GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1 << gpio);
    i = 6; while (i--) GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1 << gpio);
}


ICACHE_FLASH_ATTR void WS2812OutByte(uint8_t byte) {
  uint8_t mask = 0x80;
  while (mask) {
    if (byte & mask)
      send_ws_1(WSGPIO);
    else
      send_ws_0(WSGPIO);
    mask >>= 1;
  }
}

ICACHE_FLASH_ATTR void WS2812OutStatus( uint8_t color )
{
	uint8_t i;
	for (i=0;i<5;i++){
	   switch(color){
		case RED:
		//	WS2812OutBufferColor(0,255,0,16); 
			break;
		case GREEN:
		//	WS2812OutBufferColor(255,0,0,16); 
			break;
		case BLUE:
		//	WS2812OutBufferColor(0,0,255,16); 
			break;
	    }
	}
}


uint8_t ICACHE_FLASH_ATTR h2b(uint8_t l, uint8_t r){
	if(l<=57)l-=48;
	else l-=55;
	if(r<=57)r-=48;
	else r-=55;
	return (l<<4)+r;
}
uint8_t ICACHE_FLASH_ATTR unpackHex(uint8_t * buffer){
	uint8_t r1 = *buffer;
	buffer++;
	uint8_t r2 = *buffer;
        buffer++;
	return h2b(r1,r2);
}

void ICACHE_FLASH_ATTR ledRGB(uint8_t * buffer){
  uint8_t r1 = *buffer;
  buffer++;
  uint8_t r2 = *buffer;
  buffer++;
  uint8_t G=h2b(r1,r2);

  r1 = *buffer;
  buffer++;
  r2 = *buffer;
  buffer++;
  uint8_t R=h2b(r1,r2);

  r1 = *buffer;
  buffer++;
  r2 = *buffer;
  buffer++;
  uint8_t B=h2b(r1,r2);

  r=R;
  g=G;
  b=B;
  
  r1 = *buffer;
  buffer++;
  r2 = *buffer;
  uint8_t len=h2b(r1,r2);
  leds_set_nleds(len);
}
void ICACHE_FLASH_ATTR ledT(void *p){
  uint8_t * buffer=(uint8_t*)p;
  uint8_t r1 = *buffer;
  buffer++;
  uint8_t r2 = *buffer;
  buffer++;
  uint8_t g=h2b(r1,r2);

  r1 = *buffer;
  buffer++;
  r2 = *buffer;
  buffer++;
  uint8_t r=h2b(r1,r2);

  r1 = *buffer;
  buffer++;
  r2 = *buffer;
  buffer++;
  uint8_t b=h2b(r1,r2);

  r1 = *buffer;
  buffer++;
  r2 = *buffer;
  uint8_t len=h2b(r1,r2);
  PIN_FUNC_SELECT(LED_GPIO_MUX_12, LED_GPIO_FUNC_12);
  GPIO_OUTPUT_SET(GPIO_ID_PIN(WSGPIO), 0);
  taskENTER_CRITICAL();
  int i=0;
  while( i<len )
	{
	WS2812OutByte(g);
	WS2812OutByte(r);
	WS2812OutByte(b);
    i++;
  }
  taskEXIT_CRITICAL();
  vTaskDelete(NULL);	
  //tainted=true;
}

void ICACHE_FLASH_ATTR ledSingle(uint8_t* buffer){
  uint8_t r1 = *buffer;
  buffer++;
  uint8_t r2 = *buffer;
  buffer++;
  uint8_t g=h2b(r1,r2);

  r1 = *buffer;
  buffer++;
  r2 = *buffer;
  buffer++;
  uint8_t r=h2b(r1,r2);

  r1 = *buffer;
  buffer++;
  r2 = *buffer;
  buffer++;
  uint8_t b=h2b(r1,r2);

  r1 = *buffer;
  buffer++;
  r2 = *buffer;
  uint8_t len=h2b(r1,r2);
  PIN_FUNC_SELECT(LED_GPIO_MUX_12, LED_GPIO_FUNC_12);
  GPIO_OUTPUT_SET(GPIO_ID_PIN(WSGPIO), 0);
  taskENTER_CRITICAL();
  int i=0;
  while( i<len )
	{
	WS2812OutByte(g);
	WS2812OutByte(r);
	WS2812OutByte(b);
    i++;
  }
  taskEXIT_CRITICAL();
  //tainted=true;
}


static bool ICACHE_FLASH_ATTR send_pix_bit_banging(uint8_t pix, uint8_t pin) {
  const uint32_t pinRegister = 1 << pin;
  uint8_t mask=0x80;
  uint8_t subpix;
  uint32_t cyclesStart;
  uint32_t state;
  state = esp8266_enter_critical();
  cyclesStart = _getCycleCount() - CYCLES_800;
  while (mask) {
      uint32_t cyclesBit = ((pix & mask)) ? CYCLES_800_T1H : CYCLES_800_T0H;
      uint32_t cyclesNext = cyclesStart;
      uint32_t delta;
      uint32_t elapsed;
      do {
        cyclesStart = _getCycleCount();
        elapsed = cyclesStart - cyclesNext;

        if (elapsed >= (CYCLES_800 + CYCLES_ERROR))
          goto end_send_pixels;
        else if (elapsed >= CYCLES_800)
          break;
      } while (true);

      GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, pinRegister);

      do {
        cyclesNext = _getCycleCount();
        elapsed = cyclesNext - cyclesStart;

        if (elapsed >= (cyclesBit + CYCLES_ERROR))
          goto end_send_pixels;
        else if (elapsed >= cyclesBit)
          break;
      } while (true);

      GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, pinRegister);
    mask >>= 1;
  } 
  esp8266_leave_critical(state);
  return true;

end_send_pixels:
  esp8266_leave_critical(state);
  return false;
}



ICACHE_FLASH_ATTR bool WS2812OutPix() {
  
  taskENTER_CRITICAL();
  int i=0;
  bool res=true;
  while( i<nleds)
	{
	bool g_ret=send_pix_bit_banging(g,WSGPIO);
	bool r_ret=send_pix_bit_banging(r,WSGPIO);
	bool b_ret=send_pix_bit_banging(b,WSGPIO);

       GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1 << WSGPIO);
       if(!g_ret||!r_ret||!b_ret){
	       res=false;
	       break;
       }
    i++;
  }
  taskEXIT_CRITICAL();


  return res;
}


 

void  ledControllerTask(void *p) {
  int i;
  PIN_FUNC_SELECT(LED_GPIO_MUX_12, LED_GPIO_FUNC_12);
  GPIO_OUTPUT_SET(GPIO_ID_PIN(WSGPIO), 0);
  tainted = true;
  while (true) {
    if (tainted) tainted = !WS2812OutPix();
    vTaskDelay(1);
  }
}




int ICACHE_FLASH_ATTR leds_set_nleds(int n) {
  nleds = n ;
  tainted = true;
  return nleds;
}
  
void leds_init(void) {
  xTaskCreate(ledControllerTask, "LED", 512, NULL, 0, NULL);
} 


