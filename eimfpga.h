#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/ioport.h>
#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

/**about FPGA_DATADEP**/
#define RB_CAP 	14400  

typedef struct
{	
	unsigned long size;       /* amount of data stored here */	
	u8 *buffer, *wr, *rd;	
	unsigned long currlen;	

	spinlock_t spinlock;
	struct semaphore sem;     
	/* mutual exclusion semaphore     */		
}ringbuffer;

typedef struct 
{
	unsigned long iomux_base;
	unsigned long ccm_base;
	unsigned long gpio_base;
	unsigned long gpio3_base;
	unsigned long gpio5_base;
	unsigned long gpio6_base;
	
	unsigned long gpio1_base;
	unsigned long gpio7_base;

	unsigned long mmdc1_base;
	unsigned long mmdc2_base;

	unsigned long eim_cs0_base;
	
	unsigned long ram_base;

	unsigned long buffer_base;
	unsigned long buffer_len;

	unsigned long dspimage_address;
	unsigned long dspimage_size;	
	unsigned long dsp_filled;

	ringbuffer rb;

	struct clk *clk;
}eimfpga_info;

struct imx_dma_data {
	int dma_request; /* DMA request line */
	int dma_request_p2p;
	unsigned int peripheral_type;
	int priority;
	void *private;
};

#define MAGIC_NUMBER 0x5a5b
#define FRAME_LEN	    0x000e
struct frame_header
{
	unsigned short magic_num;
	unsigned short frmlen;
	unsigned short payload[0];
};

typedef enum
{
	frame_none,
	frame_magic,
	frame_len
}search_frame_state;




