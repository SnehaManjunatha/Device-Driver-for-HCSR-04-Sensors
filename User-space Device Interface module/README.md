Part 1: A Linux kernel module to enable user-space device interface for HC-SR04 sensors

In this assignment we have developed a loadable kernel module which initiates two instances of HC-SR04 sensors (named as HCSR_1 and HCSR_2) and allows the sensors being accessed as device files in user space.

In the test programm "main.c", the MUX GPIO pins are configured and in the kernel module the trigger and echo pins are configured.

The following are the steps to test this module-
1) To configure the MUX gpio pins:

	In main.c we have given 
	#define GPIO_MUX_1 74
	#define GPIO_MUX_2 66
	
   These values can be changed for the respective linux gpio MUX pins used.
   eg : #define GPIO_MUX_1 77    	if the linux gpio MUX is 77
	#define GPIO_MUX_2 70	        if the linux gpio MUX is 70

   Also replace the pin number in open and write call of IOSetup_init(void) as well as in IOSetup_exit(void) function
   
	eg :    if(0< write(PinExport,"66",2)) line has to be changed to
		if(0< write(PinExport,"70",2)) if the linux gpio MUX is 70

		if(0< write(PinUnexport,"66",2)) line has to be changed to
		if(0< write(PinUnexport,"70",2)) if the linux gpio MUX is 70

   Similarly the steps 
	if(0< write(PinExport,"66",2))

2) Change the echo and trigger pin number as per the gpio MUX pins used in the above step-
	#define GPIO_TRIGGER_2 40
	#define GPIO_ECHO_2 0
	#define GPIO_TRIGGER_1 38
	#define GPIO_ECHO_1 10


3) Change the frequency or the sampling rate by passing values less than 16hz in the freq argument( the third argument) of 
   hcsr_set_mode(int fd, int mode, int freq) function
   
	if mode = 0 (one shot) then give freq = 1
	if mode = 1 (burst mode) then give freq between 1hz to 16 hz

4) In command line run:
	make all

5) After the command "make all" gpio.ko and hcsr_tester files are gernerated.Load these files onto the board.These fils are present on      the executables
   Run these files on the galileo serial terminal by running the following command
	sudo screen /dev/ttyUSB0 115200
	
   inside the screen terminal enter: 
	 insmod gpio.ko
	 chmod 777 hcsr_tester
	 ./hcsr_tester


 	
