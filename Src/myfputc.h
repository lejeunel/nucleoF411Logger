/**
   @file fputc_debug.c
   @brief Trying to redirect printf() to debug port
   @date 2012/06/25
*/
 
#include "stdio.h"
#include "stm32f4xx_hal.h"

#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))

#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000
 
/* We need to implement own __FILE struct */
/* FILE struct is used from __FILE */
struct __FILE {
    int dummy;
};
 
/* You need this if you want use printf */
/* Struct FILE is implemented in stdio.h */
//FILE __stdout;

 int fputc(int c, FILE *stream)
{
   return(ITM_SendChar(c));
}