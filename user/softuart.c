#include "esp_common.h"
#include "gpio.h"

#define DIRECT_WRITE(v)    (GPIO_OUTPUT_SET(GPIO_ID_PIN(4), v))

static inline u8 chbit(u8 data, u8 bit)
{
    if ((data & bit) != 0)
    {
    	return 1;
    }
    else
    {
    	return 0;
    }
}

// Function for printing individual characters
static int soft_uart_putchar_c(unsigned bit_time, char data)
{
	unsigned i;
	unsigned start_time = 0x7FFFFFFF & system_get_time();

	//Start Bit
	DIRECT_WRITE(0);
	for(i = 0; i <= 8; i ++ )
	{
		while ((0x7FFFFFFF & system_get_time()) < (start_time + (bit_time*(i+1))))
		{
			//If system timer overflow, escape from while loop
			if ((0x7FFFFFFF & system_get_time()) < start_time){break;}
		}
		DIRECT_WRITE(chbit(data,1<<i));
	}

	// Stop bit
	while ((0x7FFFFFFF & system_get_time()) < (start_time + (bit_time*9)))
	{
		//If system timer overflow, escape from while loop
		if ((0x7FFFFFFF & system_get_time()) < start_time){break;}
	}
	DIRECT_WRITE(1);

	// Delay after byte, for new sync
	os_delay_us(bit_time*6);

	return 1;
}

int softuart_write( const char* data, unsigned len )
{
    const unsigned baudrate = 19200;
    const unsigned bit_time = (1000000 / baudrate);

    DIRECT_WRITE(1);
    os_delay_us(bit_time*8);
    int s=0;
    for(; s < len; ++s )
    {
        soft_uart_putchar_c(bit_time, data[ s ]);
    }
    return 0;
}
