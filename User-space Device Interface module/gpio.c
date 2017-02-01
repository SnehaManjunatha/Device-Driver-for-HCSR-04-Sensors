#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/semaphore.h>
#include<linux/init.h>
#include<linux/moduleparam.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>    						
#include <linux/gpio.h>      						
#include <linux/interrupt.h> 						
#include <linux/time.h>
#include <linux/unistd.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/module.h> 
#include <linux/spinlock.h>
#include "gpio_ioctl.h"
#include <linux/spinlock_types.h>

#define NUM_DEVICES 2
#define GPIO_INT_NAME  "gpio_int"
#define SIZE_OF_BUF      5
#define DEVICE_NAME1    "HCSR_1"
#define DEVICE_NAME2    "HCSR_2"

/*global variales*/
int usDeviceNo=0;
int errno;
unsigned long long t1,t2;
short int irq_any_gpio    = 0;
int count =0;
uint64_t tsc1=0, tsc2=0;

/*argument structure*/
typedef struct
{
	int send_trigger;
	int send_time ;
}Sarg, *PSarg;

/*rdtsc timer structure*/
typedef struct {
	uint64_t TSC;
}data, *Pdata;

/*ring buffer structure*/
struct ring_buf {
	Pdata ptr;
	int   head;
	int   tail;
	unsigned long count;
	unsigned long loss;
};

/*device structure */
struct HCSR_DEV_OBJ {
	struct miscdevice my_misc_dev;             							/* The  miscdevice  structure */
	struct semaphore lock;
	int state;
	int mode;
	int time;
	int dev_num;
	struct ring_buf *Pring_buf;
	int thread_state;
	int trigger_pin;
	int echo_pin;
	spinlock_t m_Lock;
	int ongoing;
	struct task_struct *task;
	struct HCSR_DEV_OBJ *next;
} *PPSHCSR_DEV_OBJ[NUM_DEVICES];

typedef struct HCSR_DEV_OBJ SHCSR_DEV_OBJ;
typedef struct HCSR_DEV_OBJ* PSHCSR_DEV_OBJ;

struct HCSR_DRIVER_OBJ
{
	PSHCSR_DEV_OBJ  head;
}SHCSR_DRIVER_OBJ, *PSHCSR_DRIVER_OBJ;

/*Pin configuration function declaration*/
int configure(int in_pin, int in_dir, int in_val);


#if defined(__i386__)
static __inline__ unsigned long long get_tsc(void)

{
	unsigned long long int x;
     	__asm__ volatile (".byte 0x0f, 0x31" : "=A" (x));
     	return x;
}

#elif defined(__x86_64__)
static __inline__ unsigned long long get_tsc(void)

{
	unsigned hi, lo;
  	__asm__ __volatile__ ("rdtsc" : "=a"(lo), "=d"(hi));
  	return ( (unsigned long long)lo)|( ((unsigned long long)hi)<<32 );
}
#endif

/*IRQ handler
* handling the interrupts at rising and falling edge
*/
static irqreturn_t r_irq_handler(int irq, void *dev_id)
{
	PSHCSR_DEV_OBJ  pshcsr_dev_obj = (PSHCSR_DEV_OBJ)dev_id;
	unsigned long flags;
	int distance;	
	int check = gpio_get_value(pshcsr_dev_obj->echo_pin);
        count++;    
	printk( "interrupt received (irq: %d)\n", irq);
	
	if (irq == gpio_to_irq(pshcsr_dev_obj->echo_pin)) 
	{
		if (check ==0)
		{  
	        	printk("gpio pin is low\n");  
	 		tsc2=get_tsc();
			distance =  ((int)(tsc2-tsc1)/(139200)); 								// processor_freq * 58 = 2400 * 58= 139200 
			if (pshcsr_dev_obj->Pring_buf->ptr)
			{
				spin_lock_irqsave(&pshcsr_dev_obj->m_Lock, flags );
			       	pshcsr_dev_obj->Pring_buf->ptr[pshcsr_dev_obj->Pring_buf->tail].TSC = distance;
				pshcsr_dev_obj->Pring_buf->tail = (pshcsr_dev_obj->Pring_buf->tail + 1) % SIZE_OF_BUF;
				if (pshcsr_dev_obj->Pring_buf->tail == pshcsr_dev_obj->Pring_buf->head)
				{
					pshcsr_dev_obj->Pring_buf->loss++;
				}
				else
				{
					pshcsr_dev_obj->Pring_buf->count++;
				}
				pshcsr_dev_obj->ongoing = 0;
				spin_unlock_irqrestore(&pshcsr_dev_obj->m_Lock, flags);
			}
			
			if(pshcsr_dev_obj->state == 1)  
			{
				up(&pshcsr_dev_obj->lock);
				printk("releasing the semaphore\n");
				pshcsr_dev_obj->state =0;
			}
			irq_set_irq_type(irq, IRQ_TYPE_EDGE_RISING);				//change the pin to rising edge
		}
       		else
		{
			tsc1=get_tsc();
	    		printk("gpio pin is high\n");  
			irq_set_irq_type(irq, IRQ_TYPE_EDGE_FALLING);											//change the pin to falling edge
	
		}
	}
	return IRQ_HANDLED;  
}

/*One shot mode function
*handle -  the pointer to the device structure
*/
void one_shot(void* handle)
{
	PSHCSR_DEV_OBJ  pshcsr_dev_obj = (PSHCSR_DEV_OBJ)handle;
	gpio_set_value_cansleep(pshcsr_dev_obj->trigger_pin, 1);
   	pshcsr_dev_obj->ongoing = 1;
   	udelay(20);
   	gpio_set_value_cansleep(pshcsr_dev_obj->trigger_pin, 0);										//make the trigger pin low
   	mdelay(20);										
}

/*Thread function
* handle  - the pointer to the device structure
*/
static int thread_function(void *handle)
{
	PSHCSR_DEV_OBJ  pshcsr_dev_obj = (PSHCSR_DEV_OBJ)handle;
	printk("time =%d\n", pshcsr_dev_obj->time);
	printk("entered thread\n");
	do{
		gpio_set_value_cansleep(pshcsr_dev_obj->trigger_pin, 0);
		gpio_set_value_cansleep(pshcsr_dev_obj->trigger_pin, 1);
		pshcsr_dev_obj->ongoing = 1;
		udelay(20);
   		gpio_set_value_cansleep(pshcsr_dev_obj->trigger_pin, 0);									//make the trigger pin low
		mdelay(pshcsr_dev_obj->time);      															//in the user space get the frequecy convert it into time for kenel space usage
	}while(!kthread_should_stop());
	return 0;
}

/*HCSR driver write function*/
ssize_t hcsr_driver_write(struct file *file, const char *buf,
	size_t count, loff_t *ppos)
{
	PSHCSR_DEV_OBJ  pshcsr_dev_obj = file->private_data;
	int *write_int;
	int integer;
	int ret=0;
	printk("time2 = %d\n",pshcsr_dev_obj->time); 	
	if (!(write_int= kmalloc(sizeof(int), GFP_KERNEL)))
	{
		printk("Bad Kmalloc\n");
		return -ENOMEM;
	}
	memset(write_int, 0, sizeof(int));

	if (copy_from_user(write_int, buf, count))
		return -EFAULT;

	integer = *(write_int);
	if (pshcsr_dev_obj->mode == 0)
	{
		if (pshcsr_dev_obj->ongoing == 1) //current
		{
			printk("there is an ongoing measurement\n");
		}
		else
		{
			if (integer != 0)
			{
				//clear the buffer
				memset(pshcsr_dev_obj->Pring_buf->ptr, 0, sizeof(data)* SIZE_OF_BUF);
			}
			one_shot((void*)pshcsr_dev_obj);
		}
	}
	if (pshcsr_dev_obj->mode == 1)
        {
                if(integer !=0)
                {
                        if(pshcsr_dev_obj->thread_state ==0)
                        {
                                pshcsr_dev_obj->task = kthread_run(&thread_function,(void *)pshcsr_dev_obj, "sampling");
                                pshcsr_dev_obj->thread_state = 1;
                        }

                }
                else if(integer == 0)
                {
                if(pshcsr_dev_obj->thread_state ==1)
                        {       ret = kthread_stop(pshcsr_dev_obj->task);
                                pshcsr_dev_obj->thread_state = 0;
                                printk( "ret value of kthread_stop = %d", ret);
                        }
                }
        }

	kfree(write_int);
	return 0;
}

/*HCSR driver read function*/
ssize_t hcsr_driver_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	unsigned long flags;
	int var, ret;
	int bytes_read=0;
	
	PSHCSR_DEV_OBJ  pshcsr_dev_obj = file->private_data;
	if (pshcsr_dev_obj->Pring_buf->ptr == NULL || pshcsr_dev_obj->Pring_buf->count == 0)
	{
		if (pshcsr_dev_obj->ongoing == 1)
		{
			pshcsr_dev_obj->state = 1;   															//lock
			ret = down_interruptible(&pshcsr_dev_obj->lock);
			if(ret != 0)
			printk("not accquired the semaphore\n");
		}
		else
		{
			one_shot((void*)pshcsr_dev_obj);
		}
	}
	mdelay(2);	
	if(pshcsr_dev_obj->Pring_buf->count >= 1)
	{
		spin_lock_irqsave(&pshcsr_dev_obj->m_Lock, flags );
		var= pshcsr_dev_obj->Pring_buf->head;
		spin_unlock_irqrestore(&pshcsr_dev_obj->m_Lock, flags );
	 
		if (copy_to_user((Pdata)buf, &(pshcsr_dev_obj->Pring_buf->ptr[var]) , sizeof(data))) 
        	{
			printk("unable to copy to  the user");
			return -EFAULT;
		}
		bytes_read = sizeof(data);
		pshcsr_dev_obj->Pring_buf->head = (pshcsr_dev_obj->Pring_buf->head + 1)% SIZE_OF_BUF;
		return bytes_read;
	}
	else
	{
		printk(" invalid read operation from the ring buffer\n");
		errno = EINVAL;
		return -1;
	  
	}
	printk("read working\n");		
	return bytes_read;																			//return size
}

/*Interrupt configuration function
*int_gpio - the interrupt gpio pin 
*handle   - the pointer to the device structure
*/
void r_int_config(int int_gpio, void* handle) 
{
	PSHCSR_DEV_OBJ  pshcsr_dev_obj = (PSHCSR_DEV_OBJ)handle;
	if ( (irq_any_gpio = gpio_to_irq(int_gpio)) < 0 ) 
	{
		printk("GPIO to IRQ mapping failure %s\n",GPIO_INT_NAME );
      		return;
   	}
   	printk(KERN_NOTICE "Mapped int %d\n", irq_any_gpio);
   	if (request_irq(irq_any_gpio,(irq_handler_t ) r_irq_handler, IRQF_TRIGGER_RISING /*| IRQF_TRIGGER_FALLING*/, GPIO_INT_NAME, (void*)pshcsr_dev_obj)) 
   	{
     		printk("Irq Request failure\n");
		return;
   	}
   	return;
}

/*HCSR driver open function*/
int hcsr_driver_open(struct inode *inode, struct file *file)
{
	int dev_minor= iminor(inode);
	const struct file_operations *new_fops = NULL;
	PSHCSR_DEV_OBJ  ptempcontext = PSHCSR_DRIVER_OBJ->head;
	while (ptempcontext->next != NULL)
	{
		if(ptempcontext->my_misc_dev.minor == dev_minor)
		{
			new_fops = fops_get(ptempcontext->my_misc_dev.fops);
			break;
		}
		ptempcontext = ptempcontext->next;
	}
	printk("minor number is %d\n", dev_minor);
	sema_init(&ptempcontext->lock, 0);																/*semaphore lock*/
	spin_lock_init(&ptempcontext->m_Lock);															/*spin lock*/
	file->private_data = ptempcontext;
	return 0;
}

/*Interrupt release function*/
int r_int_release(void* handle)
{
	PSHCSR_DEV_OBJ  pshcsr_dev_obj = (PSHCSR_DEV_OBJ)handle;
	free_irq(gpio_to_irq(pshcsr_dev_obj->echo_pin), handle );
	return 0;
}

/*HCSR driver release function*/
int hcsr_driver_release(struct inode *inode, struct file *file)
{
	int ret;
	PSHCSR_DEV_OBJ  pshcsr_dev_obj = file->private_data;
	 if(pshcsr_dev_obj->thread_state ==1)
                        {       ret = kthread_stop(pshcsr_dev_obj->task);
                                pshcsr_dev_obj->thread_state = 0;
                                printk( "ret value of kthread_stop = %d", ret);
                        }
	
	ret =r_int_release((void*)pshcsr_dev_obj);
	if(ret !=0)
		printk("failed to free interrupt");
	gpio_set_value_cansleep(pshcsr_dev_obj->trigger_pin, 0);
	gpio_set_value_cansleep(pshcsr_dev_obj->echo_pin, 0);
	gpio_free(pshcsr_dev_obj->echo_pin); //free all the pins
    	gpio_free(pshcsr_dev_obj->trigger_pin); //free all the pins
	printk("freed");
	return 0;
}

/*Configuration of pins
* in_pin - the input pin (trigger or echo pin)
* in_dir - 0 if output, 1 if input
* in_val  - the value of that pin
*/
int configure(int in_pin, int in_dir, int in_val)
{
	gpio_request(in_pin, "sysfs");    							// Request a GPIO pin from the driver
   	if(in_dir == 1)
   	gpio_direction_input(in_pin);          							// Set GPIO as input
   	if(in_dir == 0)
   	gpio_direction_output(in_pin, in_val);           					// Set GPIO as input
   return 0;
}

int trigger_validation(int trig_pin)
{
	int  i;
	int flag = 1;
	int arr_pins[22]={0,1,4,5,6,7,10,11,12,13,14,15,38,40,48,50,52,54,56,58,61,62};

	for(i =0 ; i< 22 ; i++)
	{
		if(arr_pins[i] == trig_pin)
		{	flag = 0;
			break;
		}
	}
return flag;
}

int pin_validation(int echo_pin)
{
	int i, flag = 1;	
	int arr_pins[19]={0,1,4,5,6,7,10,11,12,13,14,15,48,50,52,54,56,58,62};

	 for(i =0 ; i< 19 ; i++)
        {
                if(arr_pins[i] == echo_pin)
                {       flag = 0;
                        break;
                }
        }
return flag;

}
/*setting the trigger and echo pins*/
int hcsr_config_setpin (void* handle, int in_trigger, int in_echo)
{
	int rtrig,recho;
	rtrig = trigger_validation(in_trigger);
	recho = pin_validation(in_echo);
	if(rtrig == 0 && recho==0)
	{
		PSHCSR_DEV_OBJ  pshcsr_dev_obj = (PSHCSR_DEV_OBJ)handle;

	printk("trigger pin = %d \n", in_trigger);
		pshcsr_dev_obj->trigger_pin = in_trigger;
		pshcsr_dev_obj->echo_pin = in_echo;
		configure(in_trigger, 0 , 0);
        configure(in_echo, 1 , 0);
		r_int_config(in_echo, (void*)pshcsr_dev_obj);
		printk("trigger is set to %d\n", pshcsr_dev_obj->trigger_pin);
	}
	else
	{
		printk("Enter a valid trigger or echo pin");
	}
	
	return 0;
}

/* Mode configuration in HCSR device*/
int hcsr_config_mode (void* handle, int in_mode, int in_freq)
{
	int in_time;
	PSHCSR_DEV_OBJ  pshcsr_dev_obj = (PSHCSR_DEV_OBJ)handle;

	/*validating the mode number number*/
	if (in_mode >1)
	{
		printk("\n invalid mode number in hcsr_config_setpin function\n ");
		return 0;
	}
	
	/*validating the sampling frequency in Khz*/
	if (in_freq > 16)					
	{
		printk("\n invalid in_freq in hcsr_config_setpin function\n frequency should be less than  16hz\n");
		return 0;
	}
	in_time = (int)(1000/in_freq);
	pshcsr_dev_obj->mode = in_mode; 								//lock
	pshcsr_dev_obj->time = in_time; 								//lock

	return 0;
}
	
/* IOCTL call for HCSR driver*/
long hcsr_driver_ioctl(struct file *file, unsigned int ioctl_num, unsigned long ioctl_param)
{
	PSHCSR_DEV_OBJ  pshcsr_dev_obj = file->private_data;
	void *pIoBuffer = NULL;
	usDeviceNo= iminor(file->f_path.dentry->d_inode);
	printk("ioctl minor = %d\n", usDeviceNo);	
	
	if (!(pIoBuffer = kmalloc(_IOC_SIZE(ioctl_num), GFP_KERNEL)))
	{
		printk("ht530_dev_error: unable to allocate io buffer memory for ioctl \n");
		return -ENOMEM;
	}
	if(copy_from_user(pIoBuffer,(unsigned long *)(ioctl_param), _IOC_SIZE(ioctl_num)))
	{
		printk("ht530_dev_error: unable to copy data from user space\n");
		kfree(pIoBuffer);
		return -ENOMEM;
	}
	
	switch(ioctl_num)
	{
	
		case HCSR_IOCTL_SETPIN:
			((PSIOCTL_SETPIN)pIoBuffer)->RetVal = hcsr_config_setpin( (void*)pshcsr_dev_obj,((PSIOCTL_SETPIN)pIoBuffer)->in.in_trigger, ((PSIOCTL_SETPIN)pIoBuffer)->in.in_echo);
		break;
		case HCSR_IOCTL_SETMODE:
			((PSIOCTL_SETMODE)pIoBuffer)->RetVal = hcsr_config_mode( (void*)pshcsr_dev_obj,((PSIOCTL_SETMODE)pIoBuffer)->in.in_mode, ((PSIOCTL_SETMODE)pIoBuffer)->in.in_time);
		break;
	
		kfree(pIoBuffer);
		pIoBuffer = NULL;
	}
	return 0;
}


/* File operations structure. Defined in linux/fs.h */
static struct file_operations hcsr_fops = {
    .owner		= THIS_MODULE,           /* Owner */
    .open		= hcsr_driver_open,        /* Open method */
    .release    	= hcsr_driver_release,     /* Release method */
    .write		= hcsr_driver_write,       /* Write method */
    .read		= hcsr_driver_read,        /* Read method */
   .unlocked_ioctl	= hcsr_driver_ioctl,	/* ioctl method */
};

/*HCSR init*/
int __init hrsc_init(void)
{
	int i,ret;
	PSHCSR_DRIVER_OBJ = kmalloc(sizeof( struct HCSR_DRIVER_OBJ), GFP_KERNEL);
	if (!PSHCSR_DRIVER_OBJ) 
	{
		printk("Bad Kmalloc\n"); 
		return -ENOMEM;
	}
	printk("driver kmalloc\n");
	for(i=0; i< NUM_DEVICES; i++)
	{
		/* Allocate memory for the device structure */
		PPSHCSR_DEV_OBJ[i] = kmalloc(sizeof( struct HCSR_DEV_OBJ), GFP_KERNEL);
		if (!PPSHCSR_DEV_OBJ[i]) 
		{
			printk("Bad Kmalloc\n"); 
			return -ENOMEM;
		}
		printk("device kmalloc\n");
		memset(PPSHCSR_DEV_OBJ[i], 0, sizeof (struct HCSR_DEV_OBJ));
		if(i==0)
		{
			PPSHCSR_DEV_OBJ[i]->my_misc_dev.minor = 1;
			PPSHCSR_DEV_OBJ[i]->my_misc_dev.name = DEVICE_NAME1;
		}
		if(i==1)
		{
			PPSHCSR_DEV_OBJ[i]->my_misc_dev.minor = 2;
			PPSHCSR_DEV_OBJ[i]->my_misc_dev.name = DEVICE_NAME2;
		}
	
		PPSHCSR_DEV_OBJ[i]->my_misc_dev.fops = &hcsr_fops;
		ret = misc_register(&PPSHCSR_DEV_OBJ[i]->my_misc_dev);
	
		//per device buffer
		PPSHCSR_DEV_OBJ[i]->Pring_buf = kmalloc(sizeof(struct ring_buf), GFP_KERNEL);
		printk("device kmalloc buf\n");
	
		if (!PPSHCSR_DEV_OBJ[i]->Pring_buf) 
		{
			printk("Bad Kmalloc\n");
			return -ENOMEM;
		}

		if (!(PPSHCSR_DEV_OBJ[i]->Pring_buf->ptr = kmalloc(sizeof(data)* SIZE_OF_BUF, GFP_KERNEL)))
		{
			printk("Bad Kmalloc\n");
			return -ENOMEM;
		}
		printk("device kmalloc buff\n");
		PPSHCSR_DEV_OBJ[i]->Pring_buf->head = 0;
		PPSHCSR_DEV_OBJ[i]->Pring_buf->tail = 0;
		PPSHCSR_DEV_OBJ[i]->Pring_buf->count = 0;
		PPSHCSR_DEV_OBJ[i]->Pring_buf->loss = 0;
	}

	//creating a list of two devices
	PSHCSR_DRIVER_OBJ->head = PPSHCSR_DEV_OBJ[0];
	PPSHCSR_DEV_OBJ[0]->next = PPSHCSR_DEV_OBJ[1];
	PPSHCSR_DEV_OBJ[1]->next = NULL;
	printk("HCSR_1 driver initialized.\n");
	printk("HCSR_2 driver initialized.\n");
        return 0;
}

/*HCSR exit*/
void __exit hrsc_exit(void)
{
	int i;
	printk("count =%d\n", count);
	printk("TSC1 = %llu \n", tsc1);
	printk("TSC2 = %llu \n", tsc2);
	 
	for(i=NUM_DEVICES-1; i>=0 ; i--)	
	{
		printk("start clear\n");
		PPSHCSR_DEV_OBJ[i]->next = NULL;
		kfree(PPSHCSR_DEV_OBJ[i]->Pring_buf->ptr);
   		printk("freed ptr\n");
		kfree(PPSHCSR_DEV_OBJ[i]->Pring_buf);
   		printk("freed buf\n");
		misc_deregister(&PPSHCSR_DEV_OBJ[i]->my_misc_dev);
   		printk("freed misc\n");
		kfree(PPSHCSR_DEV_OBJ[i]);
   		printk("freed obj\n");
	}
	PSHCSR_DRIVER_OBJ->head = NULL;
	kfree(PSHCSR_DRIVER_OBJ);
}

module_init(hrsc_init);
module_exit(hrsc_exit);
MODULE_LICENSE("GPL");

