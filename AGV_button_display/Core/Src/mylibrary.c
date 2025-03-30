/*
 * mylibrary.c
 *
 *  Created on: Feb 7, 2020
 *      Author: Viktor Vano
 */
#include "mylibrary.h"

char buffer[50];
uint8_t timer_count = 0, buffer_index = 0;

uint8_t string_compare(char array1[], char array2[], uint16_t length)
{
	 uint8_t comVAR=0, i;
	 for(i=0;i<length;i++)
	   	{
	   		  if(array1[i]==array2[i])
	   	  		  comVAR++;
	   	  	  else comVAR=0;
	   	}
	 if (comVAR==length)
		 	return 1;
	 else 	return 0;
}

