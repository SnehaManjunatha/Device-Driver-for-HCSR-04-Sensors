//first device

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <pthread.h>
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

int set_retval;

typedef struct {
	uint64_t TSC;
}data, *Pdata;

void IOSetup_init(void)
{
	int PinExport,pin_test;
	PinExport = open("/sys/class/gpio/export", O_WRONLY);
	if (PinExport < 0)
		printf("\n gpio export open failed");
	
	if(0< write(PinExport,"74",2))													/*Pin number "74" can be changed to other pins*/
		printf("error PinExport 74");
	
	if (PinExport < 0)
		printf("\n gpio export open failed");
	
	if(0< write(PinExport,"66",2))													/*Pin number "66" can be changed to other pins*/
		printf("error PinExport 66");
	close(PinExport);

//Set the value for the pins
	pin_test = open("/sys/class/gpio/gpio74/value", O_WRONLY);						/*Pin number "74" can be changed to other pins*/
	if (pin_test < 0)
		printf("\n gpio%d value open failed",GPIO_MUX_1);

	if(0< write(pin_test ,"0",1))
		printf("error gpio%d value",GPIO_MUX_1 );

	pin_test = open("/sys/class/gpio/gpio66/value", O_WRONLY);						/*Pin number "66" can be changed to other pins*/
	if (pin_test < 0)
		printf("\n gpio%d value open failed", GPIO_MUX_2);

	if(0< write(pin_test,"0",1))
		printf("error gpio%d value", GPIO_MUX_2);

}

void IOSetup_exit(void)
{	int PinUnexport;
	PinUnexport = open("/sys/class/gpio/unexport", O_WRONLY);
	if (PinUnexport < 0)
		printf("\n gpio unexport open failed");
	
	if(write(PinUnexport,"74",2)< 0)													/*Pin number "74" can be changed to other pins*/
		printf("error PinUnexport 74");
	
	if (PinUnexport < 0)
		printf("\n gpio unexport open failed");
	
	if( write(PinUnexport,"66",2)< 0)													/*Pin number "66" can be changed to other pins*/
		printf("error PinUnexport 66");
	close(PinUnexport);
}

/* Setting the trigger and echo pins
	@fd: file descriptor
	@trigger: gpio pin to be set as trigger pin
	@echo: gpio pin to be set as echo pin
*/
int hcsr_set_pins(int fd, int trigger, int echo)     								/*call to give input key to the read function*/
{
	SIOCTL_SETPIN Sioctl_setpin;
	Sioctl_setpin.in.in_trigger = trigger;
	Sioctl_setpin.in.in_echo = echo;
	
	if((set_retval = ioctl(fd,HCSR_IOCTL_SETPIN,(unsigned long)&Sioctl_setpin))!=0){
		printf("ioctl error %d\n", set_retval);}
	
	if(set_retval)
		printf("hcsr_dev_error: unable to perform hcsr_set_pins \n");
	
	return 0;
}

/* Setting the mode(one shot or periodic)
	@fd: file descriptor
	@mode: one shot=0 and periodic=1
	@freq: frequency should be <16hz 
*/
int hcsr_set_mode(int fd, int mode, int freq)     									/*call to give input key to the read function*/
{
	SIOCTL_SETMODE Sioctl_setmode;
	Sioctl_setmode.in.in_mode = mode;
	Sioctl_setmode.in.in_time = freq;
	
if((set_retval = ioctl(fd,HCSR_IOCTL_SETMODE,(unsigned long)&Sioctl_setmode))!=0){
	printf("ioctl error %d\n", set_retval);}
	
	if(set_retval)
		printf("hcsr_dev_error: unable to perform hcsr_set_mode \n");
	return 0;
}

int main(int argc, char **argv)
{
	int fd1, fd2, res;
	int new_buff1, new_buff2 ,i;
	int buf;

	IOSetup_init();	
/*Opening device 1*/
	fd1 = open("/dev/HCSRdevice1", O_RDWR);
	if (fd1 < 0 ){
		printf("Can not open device  hcsr1 file\n");		
		return 0;
	}
	else {
		printf("hcsr1 device is opened\n");
	}

/*Opening device 2*/
	fd2 = open("/dev/HCSRdevice2", O_RDWR);
	if (fd2 < 0 ){
		printf("Can not open device  hcsr2 file\n");		
		return 0;
	}
	else {
		printf("hcsr2 device is opened\n");
	}

/*Setting trigger and echo pins for device 1*/
	res = hcsr_set_pins(fd1,GPIO_TRIGGER_1,GPIO_ECHO_1) ;
	if(res<0)
	printf("hcsr_set_pins for hcsr1 failed\n");

/*Setting trigger and echo pins for device 2*/	
	res = hcsr_set_pins(fd2,GPIO_TRIGGER_2,GPIO_ECHO_2) ;
	if(res<0)
	printf("hcsr_set_pins for hcsr2 failed\n");

/*Setting one-shot mode for device 1*/
	res = hcsr_set_mode(fd1,0,12) ;  //one shot mode
	if(res<0)
	printf("hcsr_set_mode for hcsr1 failed\n");

/*Setting one-shot mode for device 2*/
	res = hcsr_set_mode(fd2,0,11) ;  //one shot mode
	if(res<0)
	printf("hcsr_set_mode for hcsr2 failed\n");
	
/*Block the read function in device 1 when the buffer is empty*/
	if ((res =read( fd1, &buf , sizeof(int)))==-1)     
	printf("no data is read\n");
	else
	printf("distance read for hcsr1 =  %d\n ", buf);

/*Performing one-shot without clearing the buffer */
	new_buff1 = 0;    
	res = write(fd1, &new_buff1, sizeof(int));
		if(res<0)
		printf("write operation for hcsr1  failed");
	
	new_buff2 = 0;    
	res = write(fd2, &new_buff2, sizeof(int));
		if(res<0)
		printf("write operation for hcsr2  failed");

/*Setting sampling mode for the devices*/
	res = hcsr_set_mode(fd1,1,10) ;    
        if(res<0)
        printf("hcsr_set_mode for hcsr1  failed\n");
	res = hcsr_set_mode(fd2,1,5) ;  
        if(res<0)
        printf("hcsr_set_mode for hcsr2  failed\n");

/*Thread to start the sampling and write process*/
	new_buff1 = 1;   
	res = write(fd1, &new_buff1, sizeof(int));
    if(res<0)
        printf("write operation for hcsr1  failed");
	
	new_buff2 = 1;   
	res = write(fd2, &new_buff2, sizeof(int));//sampling
    if(res<0)
		printf("write operation for hcsr2  failed");
			
	sleep(3);
/*Stopping of thread to write in device 2*/		
	new_buff2 = 0;  
	res = write(fd2, &new_buff2, sizeof(int));
    if(res<0)
        printf("write operation for hcsr2  failed");
			
/*Reading measurment from ring buffer of device 1*/			
	if ((res =read( fd1, &buf , sizeof(int)))==-1)     
		printf("no data is read form hcsr1 \n");
	else
		printf("distance read for hcsr1  =  %d\n ", buf);
	sleep(2);

/*Stopping sampling mode of device 1*/
	new_buff1 = 0;
	res = write(fd1, &new_buff1, sizeof(int));
    if(res<0)
		printf("write operation failed for hcsr1 ");
			
/*Reading measurment from ring buffer of device 2*/	
	if ((res =read( fd2, &buf , sizeof(int)))==-1)     
		printf("no data is read form hcsr2 \n");
	else
		printf("distance read for hcsr2  =  %d\n ", buf);

/*Setting one shot mode*/	
	res = hcsr_set_mode(fd1,0,6) ;  
	if(res<0)
        printf("hcsr_set_mode for hcsr1  failed\n");

/*One shot mode with on-going measurement*/
	new_buff1 = 1 ;   
    res = write(fd1, &new_buff1, sizeof(int));//one shot... print on going measurement
    if(res<0)
        printf("write operation failed for hcsr1 ");

/*Burst shot mode*/	
	res = hcsr_set_mode(fd1,1,5) ; 
    if(res<0)
        printf("hcsr_set_mode failed for hcsr1 \n");

/*Thread to start the sampling and write process*/
	new_buff1 = 1;   
	res = write(fd1, &new_buff1, sizeof(int));
    if(res<0)
        printf("write operation for hcsr1  failed");

	sleep(5);

/*Stopping sampling mode of device 1*/
	new_buff1 = 0;
	res = write(fd1, &new_buff1, sizeof(int));
    if(res<0)
		printf("write operation failed for hcsr1 ");
	
	
/*Reading the values of buffers of device 1 and 2*/	
	for(i=0 ; i< 5; i++)
	{
		if ((res =read( fd1, &buf , sizeof(int)))==-1)     //print meassuring
			printf("no data is read form hcsr1 \n");
        else
			printf("distance read  form hcsr1 =  %d\n ", buf);
	}

	for(i=0 ; i< 5; i++)
	{
		if ((res =read( fd2, &buf , sizeof(int)))==-1)     //print meassuring
			printf("no data is read form hcsr2 \n");
        else
			printf("distance read  form hcsr2 =  %d\n ", buf);
	}

	sleep(4);
	IOSetup_exit();
	close(fd1);
	close(fd2);
return 0;	
}
