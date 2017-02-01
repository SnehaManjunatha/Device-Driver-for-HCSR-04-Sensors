//first device

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <time.h>
#include "gpio_ioctl.h"
#include <stdint.h>
#include <poll.h>

#define SIZE_OF_BUF  5															/*Ring buffer size*/
#define GPIO_TRIGGER_1 38														/*Trigger pin device 0*/
#define GPIO_ECHO_1 10															/*Echo pin device 0*/

#define GPIO_TRIGGER_2 40														/*Trigger pin device 1*/
#define GPIO_ECHO_2 0															/*Echo pin device 1*/

#define GPIO_MUX_1 74															/*Mux pin for echo pin of device 0*/
#define GPIO_MUX_2 66															/*Mux pin for echo pin of device 1*/


void IOSetup_init(void)
{
	int PinExport,pin_test;
	PinExport = open("/sys/class/gpio/export", O_WRONLY);
	if (PinExport < 0)
		printf("\n gpio export open failed");
	
	if(write(PinExport,"74",2)<0)													/*Pin number "74" can be changed to other pins*/
		printf("error PinExport 74");
	
	if (PinExport < 0)
		printf("\n gpio export open failed");
	else printf("\n gpio%d export successful",GPIO_MUX_1);

	if(write(PinExport,"66",2)<0)													/*Pin number "66" can be changed to other pins*/
		printf("error PinExport 66");
	else printf("\n gpio%d export successful",GPIO_MUX_2);
	
	close(PinExport);

//Set the value for the pins
	pin_test = open("/sys/class/gpio/gpio74/value", O_WRONLY);									/*Pin number "74" can be changed to other pins*/
	if (pin_test < 0)
		printf("\n gpio%d value open failed",GPIO_MUX_1);

	if(write(pin_test ,"0",1)<0)
		printf("error gpio%d value",GPIO_MUX_1 );

	pin_test = open("/sys/class/gpio/gpio66/value", O_WRONLY);									/*Pin number "66" can be changed to other pins*/
	if (pin_test < 0)		
		printf("\n gpio%d value open failed", GPIO_MUX_2);

	if(write(pin_test,"0",1)<0)
		printf("error gpio%d value", GPIO_MUX_2);

}

void IOSetup_exit(void)
{	int PinUnexport;
	PinUnexport = open("/sys/class/gpio/unexport", O_WRONLY);
	if (PinUnexport < 0)
		printf("\n gpio unexport open failed");
	
	if( write(PinUnexport,"74",2)<0)													/*Pin number "74" can be changed to other pins*/
		printf("error PinUnexport 74");
	
	if (PinUnexport < 0)
		printf("\n gpio unexport open failed");
	
	if( write(PinUnexport,"66",2)< 0)													/*Pin number "66" can be changed to other pins*/
		printf("error PinUnexport 66");
	close(PinUnexport);
}


int main(int argc, char **argv)
{
	

	IOSetup_init();	

return 0;	
}
