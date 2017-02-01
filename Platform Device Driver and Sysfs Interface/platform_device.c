/*
 * A sample program to show the binding of platform driver and device.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "platform_device.h"


#define CLASS_NAME "HCSR"
#define DRIVER_NAME "PLT_HCSR"
//#define DEVICE_NAME1 "hcsr0"
//#define DEVICE_NAME2 "hcsr1"
static void hcsrdevice_release(struct device *dev)
 {
	 
 }

static struct HCSRdevice hcsr_device0 = {
		.name	= DEVICE_NAME1,
		.dev_no 	= 1,
		.plf_dev = {
			.name	= DEVICE_NAME1,
			.id	= -1,
			.dev = {.release = hcsrdevice_release,}
		}
};

static struct HCSRdevice hcsr_device1 = {
		.name	= DEVICE_NAME2,
		.dev_no 	= 2,
		.plf_dev = {
			.name	= DEVICE_NAME2,
			.id	= -1,
			.dev = {.release = hcsrdevice_release,}
		}
};


/**
 * register the device when module is initiated
 */

static int p_device_init(void)
{
	int ret = 0;
	
	/* Register the device */
	platform_device_register(&hcsr_device0.plf_dev);
	
	printk(KERN_ALERT "Platform device 1 is registered in init \n");

	platform_device_register(&hcsr_device1.plf_dev);

	printk(KERN_ALERT "Platform device 2 is registered in init \n");
	
	return ret;
}

static void p_device_exit(void)
{
    	platform_device_unregister(&hcsr_device0.plf_dev);

	platform_device_unregister(&hcsr_device1.plf_dev);

	printk(KERN_ALERT "Goodbye, unregister the device\n");
}

module_init(p_device_init);
module_exit(p_device_exit);
MODULE_LICENSE("GPL");
