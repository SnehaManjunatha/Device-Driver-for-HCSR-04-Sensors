#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/list.h>
//#include <sys/ioctl.h>
#include <linux/semaphore.h>

#include <linux/kernel.h>    // kernel stuff
#include <linux/gpio.h>      // GPIO functions/macros
#include <linux/interrupt.h> // interrupt functions/macros
#include <linux/time.h>
#include <linux/unistd.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/spinlock.h>



#define SIZE_OF_BUF 5



#define GPIO_INT_NAME "gpio_int"


#define CLASS_NAME "HCSR"
#define DRIVER_NAME "PLT_HCSR"
#define DEVICE_NAME1 "HCSRdevice1"
#define DEVICE_NAME2 "HCSRdevice2"
#ifndef __SAMPLE_PLATFORM_H__

#define __SAMPLE_PLATFORM_H__



typedef struct HCSRdevice {

		char 			*name;

		int			dev_no;

		struct platform_device 	plf_dev;

}*Pchip_hcsrdevice;


typedef struct {
      int  /*uint64_t*/ TSC;
}data, *Pdata;

struct ring_buf {
        Pdata ptr;
        int   head;
        int   tail;
        unsigned long count;
        unsigned long loss;
};



 struct HCSR_DEV_OBJ

{
	Pchip_hcsrdevice phcsr_device;
	struct miscdevice my_misc_dev;  
	char *dev_name;
	//struct miscdevice my_misc_dev;             							/* The  miscdevice  structure */
	struct semaphore lock;
	int state;
	int mode;
	int time;
	int dev_num;
	struct ring_buf *Pring_buf;
	int thread_state;
	int trigger_pin;
	int echo_pin;
	int freq;
	spinlock_t m_Lock;
	int ongoing;
	int enable;
	struct list_head device_entry;
	struct task_struct *task;
	 //struct HCSR_DEV_OBJ next;
};
typedef struct HCSR_DEV_OBJ SHCSR_DEV_OBJ;
typedef struct HCSR_DEV_OBJ* PSHCSR_DEV_OBJ;




#endif /* __GPIO_FUNC_H__ */
