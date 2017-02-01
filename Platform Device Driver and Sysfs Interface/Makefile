IOT_HOME = /opt/iot-devkit/1.7.3/sysroots
PWD:= $(shell pwd)

KDIR:=$(IOT_HOME)/i586-poky-linux/usr/src/kernel
PATH := $(PATH):$(IOT_HOME)/x86_64-pokysdk-linux/usr/bin/i586-poky-linux

CC = i586-poky-linux-gcc
ARCH = x86
CROSS_COMPILE = i586-poky-linux-
SROOT=$(IOT_HOME)/i586-poky-linux/

APP1 = hcsr_tester
APP2 = hcsr_sys
CFLAGS+ = -g -Wall

obj-m = platform_device.o platform_driver.o

all:
	make EXTRA_FLAGS=$(CFLAGS) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) modules
	$(CC) -o $(APP1) main.c -Wall -g --sysroot=$(SROOT)
	$(CC) -o $(APP2) main_sys.c -Wall -g --sysroot=$(SROOT)
clean:
	make ARCH=x86 CROSS_COMPILE=i586-poky-linux- -C $(SROOT)/usr/src/kernel M=$(PWD) clean

