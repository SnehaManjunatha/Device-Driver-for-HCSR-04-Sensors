/*
 * A sample program to show the binding of platform driver and device.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include "platform_device.h"
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/device.h>
#include "gpio_ioctl.h"
#include <linux/errno.h>



int usDeviceNo=0;
int errno;
unsigned long long t1,t2;
short int irq_any_gpio    = 0;
int count =0;
uint64_t tsc1=0, tsc2=0;


static struct device *gko_device;
static struct class *gko_class;
static dev_t gko_dev= 1;
static int base_minor_num;


static int first = 1;
static const struct platform_device_id P_id_table[] = {
         { DEVICE_NAME1, 0 },
         { DEVICE_NAME2, 0 },
};




static LIST_HEAD(device_list);



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
			distance =  ((int)(tsc2-tsc1)/(139200)); 				// processor_freq * 58 = 2400 * 58= 139200 
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
			irq_set_irq_type(irq, IRQ_TYPE_EDGE_FALLING);				//change the pin to falling edge
	
		}
	}
	return IRQ_HANDLED;  
}


/*One shot mode function
*handle  the pointer to the device structure
*/
void one_shot(void* handle)
{
	PSHCSR_DEV_OBJ  pshcsr_dev_obj = (PSHCSR_DEV_OBJ)handle;
	gpio_set_value_cansleep(pshcsr_dev_obj->trigger_pin, 1);
   	pshcsr_dev_obj->ongoing = 1;
   	udelay(20);
   	//usleep_range(20,30);
   	gpio_set_value_cansleep(pshcsr_dev_obj->trigger_pin, 0);				//make the trigger pin low
   	mdelay(20);										//change this value if possible and try with usleep
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
	 	
   		gpio_set_value_cansleep(pshcsr_dev_obj->trigger_pin, 0);			//make the trigger pin low
		mdelay(pshcsr_dev_obj->time);      						//in the user space get the frequecy value and convert it into time and send it into the kernel space
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
	pshcsr_dev_obj->enable = integer;
	if (pshcsr_dev_obj->mode == 0)
	{
		if (pshcsr_dev_obj->ongoing == 1) 
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
			pshcsr_dev_obj->task = kthread_run(&thread_function,(void *)pshcsr_dev_obj, "sampling");
			pshcsr_dev_obj->thread_state = 1;
		
		}
		else if(integer == 0 && pshcsr_dev_obj->thread_state==1)
		{
			ret = kthread_stop(pshcsr_dev_obj->task);
			pshcsr_dev_obj->thread_state = 0;
			printk("ret value of kthread_stop = %d", ret);
		}
	}
	kfree(write_int);
	printk("write freed\n");
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
			pshcsr_dev_obj->state = 1;   							
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
		//	printk(" in time diff = %llu ", pshcsr_dev_obj->Pring_buf -> ptr[var].TSC); 
	 
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
	return bytes_read;										//return size
}



int hcsr_driver_open(struct inode *inode, struct file *file)
{


	const struct file_operations *new_fops = NULL;
	int dev_minor= iminor(inode);
	PSHCSR_DEV_OBJ  ptempcontext; 
printk("minor number in open = %d\n", dev_minor);
	
	list_for_each_entry(ptempcontext, &device_list, device_entry)
	{
		if(ptempcontext->my_misc_dev.minor == dev_minor)
		{
			new_fops = fops_get(ptempcontext->my_misc_dev.fops);
			break;
		}
	}
	usDeviceNo++;
		printk("minor number is %d\n", dev_minor);
	
	sema_init(&ptempcontext->lock, 0);
	spin_lock_init(&ptempcontext->m_Lock);
	file->private_data = ptempcontext;
	
return 0;
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
   	if (request_irq(irq_any_gpio,(irq_handler_t ) r_irq_handler, IRQF_TRIGGER_RISING , GPIO_INT_NAME, (void*)pshcsr_dev_obj)) 
   	{
     		printk("Irq Request failure\n");
		return;
   	}
   	return;
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
			                    						// The GPIO will appear in /sys/class/gpio
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
	pshcsr_dev_obj->freq = in_freq;
	in_time = (int)(1000/pshcsr_dev_obj->freq);
	pshcsr_dev_obj->mode = in_mode; 								
	pshcsr_dev_obj->time = in_time; 								
printk("mode is set to %d\n", pshcsr_dev_obj->mode);
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

/*Interrupt release function*/
int r_int_release(void* handle)
{
	PSHCSR_DEV_OBJ  pshcsr_dev_obj = (PSHCSR_DEV_OBJ)handle;
	free_irq(gpio_to_irq(pshcsr_dev_obj->echo_pin), handle );
	return 0;
}

int hcsr_driver_release(struct inode *inode, struct file *file)
{
	int ret;
	PSHCSR_DEV_OBJ  pshcsr_dev_obj = file->private_data;
	printk("enter release on close");
	if(pshcsr_dev_obj->thread_state == 1)
	{
	    ret = kthread_stop(pshcsr_dev_obj->task);
			pshcsr_dev_obj->thread_state = 0;
	}
	ret =r_int_release((void*)pshcsr_dev_obj);
	if(ret !=0)
		printk("failed to free interrupt");
gpio_set_value_cansleep(pshcsr_dev_obj->trigger_pin, 0);

	gpio_set_value_cansleep(pshcsr_dev_obj->echo_pin, 0);

	gpio_free(pshcsr_dev_obj->echo_pin); //free all the pins
    	gpio_free(pshcsr_dev_obj->trigger_pin); //free all the pins
    printk("free the pins\n");	


	printk("freed\n");
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




static ssize_t trigger_pin_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	PSHCSR_DEV_OBJ pshcsr_dev_obj = dev_get_drvdata(dev);
        return snprintf(buf, PAGE_SIZE, "%d\n", pshcsr_dev_obj->trigger_pin);
}

static ssize_t echo_pin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	PSHCSR_DEV_OBJ pshcsr_dev_obj = dev_get_drvdata(dev);
        return snprintf(buf, PAGE_SIZE, "%d\n", pshcsr_dev_obj->echo_pin);
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
      PSHCSR_DEV_OBJ pshcsr_dev_obj = dev_get_drvdata(dev);
        return snprintf(buf, PAGE_SIZE, "%d\n", pshcsr_dev_obj->mode);
}

static ssize_t frequency_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        PSHCSR_DEV_OBJ pshcsr_dev_obj = dev_get_drvdata(dev);

        return snprintf(buf, PAGE_SIZE, "%d\n", pshcsr_dev_obj->freq);
}

static ssize_t Enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        PSHCSR_DEV_OBJ pshcsr_dev_obj = dev_get_drvdata(dev);
        return snprintf(buf, PAGE_SIZE, "%d\n", pshcsr_dev_obj->enable);
}

static ssize_t distance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	
       PSHCSR_DEV_OBJ pshcsr_dev_obj = dev_get_drvdata(dev);

	printk("distance = %d\n", pshcsr_dev_obj->Pring_buf->ptr[pshcsr_dev_obj->Pring_buf->tail].TSC);
	 return snprintf(buf, PAGE_SIZE, "%d\n",pshcsr_dev_obj->Pring_buf->ptr[pshcsr_dev_obj->Pring_buf->tail].TSC);
	
}

/*Store functions*/
static ssize_t trigger_pin_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf,
                                  size_t count)
{
	PSHCSR_DEV_OBJ pshcsr_dev_obj = dev_get_drvdata(dev);
		
        sscanf(buf, "%d", &pshcsr_dev_obj->trigger_pin);
	 configure(pshcsr_dev_obj->trigger_pin, 0 , 0);	
	return count;

}


static ssize_t echo_pin_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf,
                                  size_t count)
{
	PSHCSR_DEV_OBJ pshcsr_dev_obj = dev_get_drvdata(dev);
		
        sscanf(buf, "%d", &pshcsr_dev_obj->echo_pin);
	 configure(pshcsr_dev_obj->echo_pin, 1 , 0);	
	r_int_config(pshcsr_dev_obj->echo_pin, (void*)pshcsr_dev_obj);
	return count;

}

static ssize_t mode_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf,
                                  size_t count)
{
	PSHCSR_DEV_OBJ pshcsr_dev_obj = dev_get_drvdata(dev);
	 sscanf(buf, "%d", &pshcsr_dev_obj->mode);
	return count;
}

static ssize_t frequency_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf,
                                  size_t count)
{
	int freq;
	PSHCSR_DEV_OBJ pshcsr_dev_obj = dev_get_drvdata(dev);
	sscanf(buf, "%d", &freq);
        pshcsr_dev_obj->freq = freq;
	pshcsr_dev_obj->time = 1/freq;
	return count;
}

static ssize_t Enable_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf,
                                  size_t count)
{
int integer,ret;
PSHCSR_DEV_OBJ pshcsr_dev_obj = dev_get_drvdata(dev);
	sscanf(buf, "%d",&integer);

	if (pshcsr_dev_obj->mode == 0)
	{
		if (pshcsr_dev_obj->ongoing == 1) 
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
		else if(integer == 0 && pshcsr_dev_obj->thread_state==1)
		{
			ret = kthread_stop(pshcsr_dev_obj->task);
			pshcsr_dev_obj->thread_state = 0;
			printk("ret value of kthread_stop = %d", ret);
		}
	}
	
	printk("write freed\n");
	
	return count;
}


static DEVICE_ATTR(trigger_pin,S_IRWXU, trigger_pin_show, trigger_pin_store);
static DEVICE_ATTR(echo_pin,S_IRWXU, echo_pin_show, echo_pin_store);
static DEVICE_ATTR(mode,S_IRWXU, mode_show, mode_store);
static DEVICE_ATTR(frequency,S_IRWXU, frequency_show, frequency_store);
static DEVICE_ATTR(Enable,S_IRWXU, Enable_show, Enable_store);
static DEVICE_ATTR(distance, S_IRWXU, distance_show,NULL);






 PSHCSR_DEV_OBJ init_device(void* Pchip_handle)
{
	int ret;
	Pchip_hcsrdevice pdevice = (Pchip_hcsrdevice)Pchip_handle;
	PSHCSR_DEV_OBJ pdrv;
	
	pdrv = kmalloc(sizeof(SHCSR_DEV_OBJ), GFP_KERNEL);
	printk("device kmalloc buf\n");
	if (!pdrv) {
		printk("Bad Kmalloc\n");
		
	}
printk("malloc pdrv\n");
	
	memset(pdrv, 0, sizeof( SHCSR_DEV_OBJ));
	
	pdrv->phcsr_device = pdevice;
	
	if (pdevice->dev_no ==1)
	{
		pdrv->dev_name = DEVICE_NAME1;
		pdrv->my_misc_dev.name = DEVICE_NAME1;

	
	}
	
	if (pdevice->dev_no ==2)
	{
		pdrv->dev_name = DEVICE_NAME2;
		pdrv->my_misc_dev.name = DEVICE_NAME2;

	}
	
	pdrv->my_misc_dev.minor = ++base_minor_num;
	pdrv->my_misc_dev.fops = &hcsr_fops;
	ret = misc_register(&pdrv->my_misc_dev);
	
	printk("inside probe  minor = %d", pdrv->my_misc_dev.minor);
                        //per device buffer
        pdrv->Pring_buf = kmalloc(sizeof(struct ring_buf), GFP_KERNEL);
printk("device kmalloc ring buf\n");
        if (!pdrv->Pring_buf) {
                printk("Bad Kmalloc\n");
       
        }

        if (!(pdrv->Pring_buf->ptr = kmalloc(sizeof(data)* SIZE_OF_BUF, GFP_KERNEL)))
        {
                printk("Bad Kmalloc\n");
                
        }
printk("device kmalloc buff\n");
        pdrv->Pring_buf->head = 0;
        pdrv->Pring_buf->tail = 0;
        pdrv->Pring_buf->count = 0;
        pdrv->Pring_buf->loss = 0;

    

	
	printk("inside probe  minor = %d", pdrv->my_misc_dev.minor);
	printk("endof init\n");
	return pdrv;
	
}

static int P_driver_probe(struct platform_device *dev_found)
{
	
	int rval;	
	PSHCSR_DEV_OBJ pdrv;

	Pchip_hcsrdevice pdevice;
	
	pdevice = container_of(dev_found, struct HCSRdevice, plf_dev);
	
	printk(KERN_ALERT "Found the device -- %s  %d \n", pdevice->name, pdevice->dev_no);
if(first == 1)
{
 gko_class = class_create(THIS_MODULE, CLASS_NAME);
        if (IS_ERR(gko_class)) {
                printk( " cant create class %s\n", CLASS_NAME);
                goto class_err;

        }
first =0;
}
if(!(pdrv =init_device((void*)pdevice)))
{
	printk("device initialisation failed\n");
}


INIT_LIST_HEAD(&pdrv->device_entry) ;
list_add(&pdrv->device_entry, &device_list );




/* device */
        gko_device = device_create(gko_class, NULL, gko_dev, pdrv, pdrv->dev_name);
        if (IS_ERR(gko_device)) {
                
                printk( " cant create device %s\n", pdrv->dev_name);
                goto device_err;
        }

printk("device create\n");	
	rval = device_create_file(gko_device, &dev_attr_trigger_pin);
        if (rval < 0) {
                printk(" cant create device attribute %s %s\n", 
                       pdrv->dev_name, dev_attr_trigger_pin.attr.name);
        }
	
	rval = device_create_file(gko_device, &dev_attr_echo_pin);
        if (rval < 0) {
               printk(" cant create device attribute %s %s\n", 
                       pdrv->dev_name, dev_attr_echo_pin.attr.name);
        }

	rval = device_create_file(gko_device, &dev_attr_mode);
        if (rval < 0) {
               printk(" cant create device attribute %s %s\n", 
                       pdrv->dev_name, dev_attr_mode.attr.name);
        }

	rval = device_create_file(gko_device, &dev_attr_frequency);
        if (rval < 0) {
       printk(" cant create device attribute %s %s\n", 
                       pdrv->dev_name, dev_attr_frequency.attr.name);
        }

	rval = device_create_file(gko_device, &dev_attr_Enable);
        if (rval < 0) {
      printk(" cant create device attribute %s %s\n", 
                       pdrv->dev_name, dev_attr_Enable.attr.name);
        }

        rval = device_create_file(gko_device, &dev_attr_distance);
        if (rval < 0) {
   	 printk(" cant create device attribute %s %s\n", 
                       pdrv->dev_name, dev_attr_distance.attr.name);
        }
printk("probe done\n");

	return 0;
device_err:
        device_destroy(gko_class, gko_dev);
class_err:
        class_unregister(gko_class);
        class_destroy(gko_class);
return -EFAULT;			

};

static int P_driver_remove(struct platform_device *pdev)
{	
PSHCSR_DEV_OBJ  ptempcontext;	
printk("enter remove of the probe\n");

list_for_each_entry(ptempcontext, &device_list, device_entry)
	{
	    	printk("remove a list\n");
		if(ptempcontext->dev_name == pdev->name)
		{
		    --usDeviceNo;
		list_del(&ptempcontext->device_entry);
	    device_destroy(gko_class, ptempcontext->my_misc_dev.minor);
		kfree(ptempcontext->Pring_buf->ptr);
   	    printk("freed ptr\n");
	    kfree(ptempcontext->Pring_buf);
   	    printk("freed buf\n");
	    misc_deregister(&ptempcontext->my_misc_dev);
   	    printk("freed misc\n");
	    kfree(ptempcontext);
   	    printk("freed obj\n");
			break;
		}
	}






if (usDeviceNo == 0)
{
class_unregister(gko_class);
        class_destroy(gko_class);
}



	return 0;
};

static struct platform_driver P_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= P_driver_probe,
	.remove		= P_driver_remove,
	.id_table	= P_id_table,
};

module_platform_driver(P_driver);
MODULE_LICENSE("GPL");
