Part 2: Platform device driver and sysfs interface for HC-SR04

In this part a loadable module enables driver operations for two hard-coded HC-SR04 sensors.

In the test program "main.c", the MUX GPIO pins are configured and in the kernel module the trigger and echo pins are configured.

The following are the steps to test this module:

1) To configure the MUX gpio pins:
   The gpio mux pins specified in the F and G column of "gpio_io pins" excel sheet(attached in the folder) are virtual pins whose
   direction is by default "out".
   
	In main.c we have given 
	#define GPIO_MUX_1 74
	#define GPIO_MUX_2 66
	
	change these value for the respective linux gpio MUX pins used.	
	eg : #define GPIO_MUX_1 77        if the linux gpio MUX is 77
	     #define GPIO_MUX_2 56	  if the linux gpio MUX is 56

Also replace the pin number in open and write call of IOSetup_init(void) as well as in IOSetup_exit(void) function
eg :if(0< write(PinExport,"66",2)) line has to be changed to
	if(0< write(PinExport,"77",2)) if the linux gpio MUX is 77
	
	if(0< write(PinUnexport,"66",2)) line has to be changed to
	if(0< write(PinUnexport,"77",2)) if the linux gpio MUX is 77

	Similarly the steps 
	if(0< write(PinExport,"66",2))    line can be changed.

   2) Change the echo and trigger pin number as per the gpio MUX pins used in the above step:
	#define GPIO_TRIGGER_2 40
	#define GPIO_ECHO_2 0
	#define GPIO_TRIGGER_1 38
	#define GPIO_ECHO_1 10

   3) After configuring the gpio linux mux pins, follow the steps to set the platform device and driver for the HCSR sensor.

	Run the Makefile to obtain the executable and the relocable file:
			make all

	After the command "make all", .platform_device.ko, platform_driver.ko, hcsr_tester and hcsr_sys files are gernerated.Load these
	along with hcsr_sysfs.sh onto the board. These files are present in the executables folder.

	Run these files on the galileo serial termial by running the following command
			sudo screen /dev/ttyUSB0 115200
	
	inside the serial screen ternimal enter
	   $insmod platform_device.ko
	   $insmod platform driver.ko	(the steps 1 and 2 can be interchanged)

	After the platform device and the driver are matched, the driver is probed to intialise and register the device. Since there are
	two sensors, the driver probe function is called twice. simultaneously the class HCSR, the device subdirectories HCSRdevice1 and
	HCSRdevice2 and their respective attributes are created.
	
   4) now we can run our application using the dev interface by :
			
			chmod 777 hcsr_tester
			./hcsr_tester
			
			
   5) we can also run the sysfs interface by :
			
			chmod 777 hcsr_sysfs.sh
			./hcsr_sysfs.sh
			
			
			
   6) Since we have configured the mux pins at the user space to test the dev interface in step 4, we can run step 5 without
  setting the mux pins again, but if step 5 has to be tested first then we can run:
		
		$ chmod 777 hcsr_sys
		$ ./hcsr_sys                              (this is to set the gpio mux pins)
		$ chmod 777 hcsr_sysfs.sh
		$ ./hcsr_sysfs.sh			  (this is to run the script file)
			
			
			
			
