
//#include "main.h"

//#include "stm32f4xx_hal.h"




int main(void){

	(*(volatile unsigned int *) ( 0x40023800 + 0x30 ))|= (1U << 2);

	(*(volatile unsigned int *) ( 0x40020800 )) |= (0b01 << 26);


	while(1){

		(*(volatile unsigned int *) ( 0x40020800 + 0x14 )) ^= (1 << 13);

		for(int i = 0; i < 300000; i++);

	}
}
