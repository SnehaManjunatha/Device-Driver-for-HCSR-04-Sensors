#include <linux/ioctl.h>


#define MAGIC_NUMBER 				'K'
#define HCSR_IOCTL_BASE				0x00



#define	HCSR_IOCTL_SETPIN							_IOWR(MAGIC_NUMBER, HCSR_IOCTL_BASE+1, struct _IOCTL_SETPIN)
#define HCSR_IOCTL_SETMODE								_IOWR(MAGIC_NUMBER, HCSR_IOCTL_BASE+2, struct _IOCTL_SETMODE)

typedef struct _IOCTL_SETPIN
{
	int RetVal;
	struct
	{
		int in_trigger;
		int in_echo;
	}in;
}SIOCTL_SETPIN, *PSIOCTL_SETPIN;

typedef struct _IOCTL_SETMODE
{
	int RetVal;
	struct
	{
		int in_mode;
		int in_time;
	}in;
}SIOCTL_SETMODE, *PSIOCTL_SETMODE;
