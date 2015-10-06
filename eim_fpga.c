#include "eim.h"
#include "eimfpga.h"

static struct class *eimfpga_class;



#define EIMFPGA_MAJOR 		197
#define EIMFPGA_DRVNAME 	"eimfpga"
#define MXC_INT_WEIM			46
#define MXC_INT_GPIO6			108
#define BUFFER_PAGE_ORDER		8

#define BUFFER_SIZE 0x400000

eimfpga_info fpga_info={0};

u32 rxdma_buffer_phys=0;
dma_cap_mask_t eim_dma_mask;
struct dma_chan *eim_chan=NULL;
struct imx_dma_data	eim_dma_data={0};
unsigned char *dmabuff=NULL;
struct completion dma_cmp;
struct dma_async_tx_descriptor *dma_rxdesc = NULL;
dma_cookie_t		cookie;
struct dma_device *dma_dev;
enum dma_ctrl_flags 	dma_flags;
struct dma_slave_config eimdma_slave_config;
enum dma_slave_buswidth eim_buswidth;
struct scatterlist *eim_dst_sg, *eim_src_sg;
struct scatterlist eim_rx_sg[2];

struct tasklet_struct rxtasklet;
struct work_struct	 eimrxwork;



ssize_t  eimfpga_read(struct file *filp, char __user *buf, size_t count,loff_t *f_pos);
int eimfpga_open(struct inode *inode, struct file *filp);
ssize_t eimfpga_write(struct file *filp, char *buf, size_t count,loff_t *f_pos);
int eimfpga_release(struct inode *inode, struct file *filp);
static int fpga_probe(struct platform_device *pdev);
static int  fpga_remove(struct platform_device *pdev);

#define DISTANCE(wr,rd,size) 	((rd>=wr)?(rd-wr):(rd+size-wr))

static const struct file_operations fpga_fops = {
	.owner		= THIS_MODULE,
	.open		= eimfpga_open,
	.read		= eimfpga_read,
	.write 		= eimfpga_write,
	.release 		= eimfpga_release,
};

static struct resource eim_fpga_resources[] = {
	{
		.start	= EIMRAM_BASE,
		.end	= EIMRAM_BASE + SZ_64K - 1,
		.flags	= IORESOURCE_MEM,
	}
};
	
static struct platform_device fpga_pfdev = {
	.name = "eimfpga",
	.id		= 0,
	.dev		={
				.release = eimfpga_release,
				.platform_data = &fpga_info,
		},
	.num_resources	= ARRAY_SIZE(eim_fpga_resources),
	.resource	= eim_fpga_resources,
};

static struct platform_driver fpga_pfdrv = {
	.driver = {
		.name = "eimfpga",
		.owner = THIS_MODULE,
	},
	.probe = fpga_probe,
	.remove =fpga_remove,
};

int eimfpga_open(struct inode *inode, struct file *filp)
{
	eimfpga_info *dev;
	unsigned int value;

	dev=filp->private_data;

	if(!dev)
	{
		filp->private_data=&fpga_info;
	}
	dev=&fpga_info;

	return 0;          
}
ssize_t  eimfpga_read(struct file *filp, char __user *buf, size_t count,loff_t *f_pos)
{
	eimfpga_info *pdata;
	int index=0;
	unsigned short *buffer=(unsigned short *)buf;
	ringbuffer *rb;
	unsigned short *rdptr=NULL;
	unsigned long irqflags;
	unsigned long currlen=0;

	//printk("%s 0x%x 0x%x\n", __func__, buf, count);
	
	pdata=(eimfpga_info *)filp->private_data;
	rb=&(pdata->rb);

	spin_lock_irqsave(&(rb->spinlock), irqflags);

	currlen=rb->currlen;
	count=(count<currlen)?count:currlen;

	if(count)
	{
		currlen=rb->buffer+rb->size-rb->rd;
		count=(count<currlen)?count:currlen;
		rdptr=rb->rd;	

		if((rb->rd+count) >= (rb->buffer+rb->size))
		{
			rb->rd=rb->buffer;		
		}
		else
			rb->rd+=count;

		rb->currlen-=count;
	}	

	spin_unlock_irqrestore(&(rb->spinlock), irqflags);
	
	if(count)
		copy_to_user(buf, rdptr, count);
	
	return count;

	
#if 0
	ringbuffer *rb=filp->private_data;
	unsigned long irqflags;
	int ret=0;
	u8 *read=NULL;
	
	mutex_lock(&uppmutex);	
	if(! rb)
		return 0;
	
	spin_lock_irqsave(&upp_rwlock, irqflags);

	if(rb->buffer+rb->size-rb->rd <count)
	{
		count=rb->buffer+rb->size-rb->rd;
	}
	
	read=rb->rd;
	if(rb->currlen < count)
	{
		ret=rb->currlen;
	}
	else
	{
		ret=count;
	}
	rb->rd+=ret;
	rb->currlen-=ret;
	
	if(rb->rd >=(rb->buffer+rb->size))
		rb->rd=rb->buffer;	
	//printk("3");

	spin_unlock_irqrestore(&upp_rwlock, irqflags);
	mutex_unlock(&uppmutex);
	
	//printk("4");
	copy_to_user(buf, read, ret);
#endif
}

ssize_t eimfpga_write(struct file *filp, char *buf, size_t count,loff_t *f_pos)
{
	eimfpga_info *pdata;
	volatile unsigned long value;

	pdata=(eimfpga_info *)filp->private_data;
	//reset FIFO
	value=__raw_readl(pdata->gpio6_base+GPIO6_DR);
	value |= 0x00000800;
	__raw_writel(value, pdata->gpio6_base+GPIO6_DR);
	msleep(500);
	value=__raw_readl(pdata->gpio6_base+GPIO6_DR);
	value &= (~0x00000800);
	__raw_writel(value, pdata->gpio6_base+GPIO6_DR);	
	printk("%s \n", __func__);
	return 0;
}

int eimfpga_release(struct inode *inode, struct file *filp)
{
}

#define FPGA_DATADEP 7800
search_frame_state search_state=frame_none;
static int fpga_isr_cnt=0;
unsigned short fpgadata[FPGA_DATADEP*2] __attribute__((aligned(64)));

extern unsigned long memcpy_neon(void *dst, void *src, unsigned long size);

static void eim_dma_callback(void *para)
{
	struct dma_tx_state state;
	enum dma_status dmastat;
	eimfpga_info *pdata=(eimfpga_info *)para;
	int *data=(int *)(pdata->buffer_base);
	int index=0;
	
	dmastat=eim_chan->device->device_tx_status(eim_chan,cookie, &state);

	for(index=0;index<4;index++)
	{
		printk("\t 0x%x", data[index]);
	}
	printk("\n & 0x%x\n", dmastat);

	return ;
}

#ifdef EIM_TASKLET
int eimfpga_rxfunc(unsigned long param)
#else
int eimfpga_rxfunc(struct work_struct *work)
#endif
{
#ifdef EIM_WORK
	eimfpga_info *pdata=&fpga_info;
#endif
#ifdef EIM_TASKLET
	eimfpga_info *pdata=(eimfpga_info *)param;
#endif
	ringbuffer *prb=&(pdata->rb);
	int index=0;
	unsigned long v1,v2,v3,v4;
	unsigned long frame_start=0;
	unsigned short prevalue=0;
	unsigned long err_cnt=0;
	unsigned long zero_cnt=0;
	struct dma_tx_state state;
	enum dma_status dmastat;
	unsigned short *dataptr=NULL;
	unsigned long end_splcnt=0,wr_splcnt;
	unsigned long irqflags;	
	unsigned short u16value;
	unsigned long u32value;
	unsigned long frmlen=0;
	unsigned long cap=0;
	unsigned long first=0;
	unsigned long end=0;
	//volatile unsigned long delay=0;
	int *data=(int *)(pdata->buffer_base);
	
#if 0
	spin_lock_irqsave(&(pdata->rb.spinlock), irqflags);	
	for(index=0;index<FPGA_DATADEP;index++)
	{
		u16value=*(u16 *)(pdata->ram_base);
		//if(index%128 == 0)
		//	printk("0x%x ", u16value);
	}
	spin_unlock_irqrestore(&(pdata->rb.spinlock), irqflags);	
#endif

#if 1
	search_state=frame_none;
	//end_splcnt=(pdata->rb.buffer+pdata->rb.size-pdata->rb.wr)/sizeof(u16);
	wr_splcnt=0;
	//spin_lock_irqsave(&(prb->spinlock), irqflags);
	dataptr=(u16 *)prb->wr;
	end=prb->buffer+prb->size;
	
	for(index=0;index<FPGA_DATADEP;)
	{
		//u16value=*(u16 *)(pdata->ram_base);
		u32value=*(u32 *)(pdata->ram_base);
		
		u16value=(unsigned short)u32value;
		if(search_state==frame_none && u16value != MAGIC_NUMBER)
		{
			printk("$");
			continue ;
		}
		//printk("0x%x \n", u32value);
		if( search_state==frame_none && u16value==MAGIC_NUMBER)
		{
			search_state=frame_magic;
		}
		else if(search_state==frame_magic && u16value!=0)
		{
			frmlen=(u16value-4)/sizeof(u16);
			search_state=frame_len;
		}
		else if(search_state==frame_len)
		{
			frmlen--;
			if(frmlen == 0)
				search_state=frame_none;
		}
		else
		{
			printk("state err: 0x%x 0x%x \n", u16value, search_state);
			continue ;
		}

		fpgadata[index]=u16value;
	#if 0	
		*dataptr++=u16value;
		//dataptr++;
		wr_splcnt++;
		if(dataptr>=end)
		{
			dataptr=prb->buffer;
		}	
	#endif
		index++;			
	}
#if 1

	#if 1
	spin_lock_irqsave(&(prb->spinlock), irqflags);

	wr_splcnt=FPGA_DATADEP;
	first=(prb->buffer+prb->size-prb->wr);
	if(first > FPGA_DATADEP*sizeof(short))
	{	
		memcpy_neon((u8 *)prb->wr, fpgadata, FPGA_DATADEP*sizeof(short));
		prb->wr+=(FPGA_DATADEP*sizeof(short));
	}
	else
	{
		memcpy_neon((u8 *)prb->wr, fpgadata, first);
		memcpy_neon((u8 *)prb->buffer, &(fpgadata[first/sizeof(short)]),FPGA_DATADEP*sizeof(short)-first);
		prb->wr=pdata->rb.buffer+FPGA_DATADEP*sizeof(short)-first;
	}	
	#endif

	//prb->wr+=(FPGA_DATADEP*sizeof(short));
	//if(prb->wr >=end)
	//	prb->wr-=prb->size;
	
	cap=DISTANCE((u32)(prb->wr), (u32)(prb->rd), (u32)(prb->size));
	//prb->wr=(u8 *)dataptr;
	//printk("cap 0x%x: 0x%x 0x%x \t", cap,prb->wr,prb->rd);
	if(cap < RB_CAP)
	{
		cap=RB_CAP-cap;
		//prb->currlen+=((wr_splcnt*sizeof(u16))-cap);
		prb->currlen+=((FPGA_DATADEP*sizeof(u16))-cap);
		prb->rd+=cap;
		if(prb->rd>=end)
			prb->rd-=prb->size;

		//printk(" **0x%x  0x%x\n", cap, prb->currlen);
	}
	else
	{
		//prb->currlen+=(wr_splcnt*sizeof(u16));
		prb->currlen+=(FPGA_DATADEP*sizeof(u16));
		//printk(" $$ 0x%x\n", prb->currlen);
	}
	spin_unlock_irqrestore(&(prb->spinlock), irqflags);
	//printk("0x%x\n", prb->currlen);
#endif

#endif
}


static int fpga_isr(int irq, void *dev_id)
{
	unsigned long value=0;
	struct platform_device *pdev=(struct platform_device *)dev_id;
	eimfpga_info *pdata=pdev->dev.platform_data;
	int index=0;
	unsigned long v1,v2,v3,v4;
	unsigned long frame_start=0;
	unsigned short prevalue=0;
	unsigned long err_cnt=0;
	unsigned long zero_cnt=0;
	struct dma_tx_state state;
	enum dma_status dmastat;
	unsigned short *dataptr=NULL;
	unsigned long end_splcnt=0,wr_splcnt;
	unsigned long irqflags;	
	unsigned short u16value;
	unsigned long frmlen=0;
	unsigned long cap=0;
	//volatile unsigned long delay=0;
	int *data=(int *)(pdata->buffer_base);

	//return IRQ_HANDLED;
	
	value=__raw_readl(pdata->gpio6_base+GPIO6_ISR);
	if((value & 0x00004000) !=0x0)
	{
		return IRQ_HANDLED;
	}

	value|=(0x00004000);
	//value|=0xffffffff;
	__raw_writel(value, pdata->gpio6_base+GPIO6_ISR);

	prevalue=0;
	frame_start=0;
	zero_cnt=0;	
	
	//for(index=0;index<FPGA_DATADEP;index++)
	{
	//	u16value=*(u32 *)(pdata->ram_base);
		//printk("0x%x ", u16value);
		//printk("$");
	}

#ifdef EIM_TASKLET
	tasklet_schedule(&rxtasklet);
	return IRQ_HANDLED;
#endif

#ifdef EIM_WORK
	schedule_work(&eimrxwork);
	return IRQ_HANDLED;
#endif

#ifdef FPGA_DMA	
#if 0
	memset(data, 0, 4096);
	dma_rxdesc->callback = eim_dma_callback;
	dma_rxdesc->callback_param = pdata;
	cookie = dma_rxdesc->tx_submit(dma_rxdesc);
	if(dma_submit_error(cookie))
	{
		printk("unable to submit FPGA DMA\n");
	}
	//dma_async_issue_pending(eim_chan);

	
	zero_cnt=0;
	#if 0
	while(1)
	{
		dmastat=eim_chan->device->device_tx_status(eim_chan,cookie, &state);
		if(dmastat == DMA_SUCCESS)
			break;
		zero_cnt++;
		//printk("0x%x\n", dmastat);			
		if(zero_cnt >= 0x1000000)
			break;
	}
	printk("%s dmastat:%d \n", __func__, dmastat);
	#endif
#endif	
#endif


//#ifndef FPGA_DMA
#if 0

	for(index=0;index<(FPGA_DATADEP);index++)
	{
		u16value=*(u32 *)(pdata->ram_base);
		printk("0x%x\t", u16value);
	}
#if 0	
	search_state=frame_none;
	//end_splcnt=(pdata->rb.buffer+pdata->rb.size-pdata->rb.wr)/sizeof(u16);
	wr_splcnt=0;
	spin_lock_irqsave(&(pdata->rb.spinlock), irqflags);
	dataptr=(u16 *)pdata->rb.wr;
	


	for(index=0;index<FPGA_DATADEP;)
	{
		//fpgadata[index]=*(u32 *)(pdata->ram_base);
		u16value=*(u32 *)(pdata->ram_base);

		//for(delay=0;delay<10000;delay++);

		if(search_state==frame_none && u16value != MAGIC_NUMBER)
			continue ;

		if( search_state==frame_none && u16value==MAGIC_NUMBER)
		{
			search_state=frame_magic;
		}
		else if(search_state==frame_magic && u16value!=0)
		{
			frmlen=(u16value-4)/sizeof(u16);
			search_state=frame_len;
		}
		else if(search_state==frame_len)
		{
			frmlen--;
			if(frmlen == 0)
				search_state=frame_none;
		}
		else
		{
			printk("state err: 0x%x 0x%x \n", u16value, search_state);
		}
		*dataptr=u16value;
		dataptr++;
		wr_splcnt++;
		if(dataptr>=(pdata->rb.buffer+pdata->rb.size))
		{
			dataptr=pdata->rb.buffer;
		}	
		index++;			
	}

	cap=DISTANCE((u32)(dataptr), (u32)(pdata->rb.rd), (u32)(pdata->rb.size));
	pdata->rb.wr=(u8 *)dataptr;

	if(cap < RB_CAP)
	{
		cap=RB_CAP-cap;
		pdata->rb.currlen+=((wr_splcnt*sizeof(u16))-cap);
		pdata->rb.rd+=cap;
		if(pdata->rb.rd>=(pdata->rb.buffer+pdata->rb.size))
			pdata->rb.rd-=pdata->rb.size;
	}
	else
	{
		pdata->rb.currlen+=(wr_splcnt*sizeof(u16));
	}
	spin_unlock_irqrestore(&(pdata->rb.spinlock), irqflags);
#endif
	
#endif


	return IRQ_HANDLED;
}


 int mxc_iomux_v3_setup_multiple_pads(iomux_v3_cfg_t *pad_list, unsigned count, unsigned long base)
{
	iomux_v3_cfg_t *p = pad_list;
	int i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = mxc_iomux_v3_setup_pad(*p, base);
		if (ret)
			return ret;
		p++;
	}
	return 0;
}
/*
 * configures a single pad in the iomuxer
 */
int mxc_iomux_v3_setup_pad(iomux_v3_cfg_t pad, unsigned long base)
{
	u32 mux_ctrl_ofs = (pad & MUX_CTRL_OFS_MASK) >> MUX_CTRL_OFS_SHIFT;
	u32 mux_mode = (pad & MUX_MODE_MASK) >> MUX_MODE_SHIFT;
	u32 sel_input_ofs = (pad & MUX_SEL_INPUT_OFS_MASK) >> MUX_SEL_INPUT_OFS_SHIFT;
	u32 sel_input = (pad & MUX_SEL_INPUT_MASK) >> MUX_SEL_INPUT_SHIFT;
	u32 pad_ctrl_ofs = (pad & MUX_PAD_CTRL_OFS_MASK) >> MUX_PAD_CTRL_OFS_SHIFT;
	u32 pad_ctrl = (pad & MUX_PAD_CTRL_MASK) >> MUX_PAD_CTRL_SHIFT;

	if (mux_ctrl_ofs)
	{
		__raw_writel(mux_mode, base + mux_ctrl_ofs);
		//printk("%s 0x%x -> 0x%x\n", __func__, base+mux_ctrl_ofs, mux_mode);
	}

	if (sel_input_ofs)
	{
		__raw_writel(sel_input, base + sel_input_ofs);
		//printk("%s 0x%x -> 0x%x\n", __func__, base+sel_input_ofs, sel_input);
	}

	if (!(pad_ctrl & NO_PAD_CTRL) && pad_ctrl_ofs)
	{
		__raw_writel(pad_ctrl, base + pad_ctrl_ofs);
		//printk("%s 0x%x -> 0x%x\n", __func__, base+pad_ctrl_ofs, pad_ctrl);
	}

	return 0;
}

static bool filter(struct dma_chan *chan, void *param)
{
	chan->private=param;
	return true;
}


#define FPGAINTR_GPIO IMX_GPIO_NR(6,14)

#define SG_NUM 2

static int fpga_probe(struct platform_device *pdev)
{
	volatile int index=0;
	eimfpga_info *pdata = pdev->dev.platform_data;
	volatile unsigned long value=0;
	volatile unsigned long v1,v2,v3,v4,v5,v6,v7;
	unsigned long eimfpga_pads_cnt;
	int retval;
	struct clk *clk;
	struct resource *resource;
	unsigned int *start=NULL;
	unsigned int cnt=0;
	unsigned long rate;
	int order=0;


	if (pdev != &fpga_pfdev)
		return -ENODEV;

#ifdef FPGA_DMA
#if 0
	dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
		
	pdata->buffer_len=BUFFER_SIZE;
	pdata->buffer_base=(u32)dma_alloc_coherent(NULL, BUFFER_SIZE, &rxdma_buffer_phys, GFP_KERNEL|GFP_DMA);
	printk("dma_alloc_coherent vaddr: 0x%x dma_addr:0x%x\n",pdata->buffer_base, rxdma_buffer_phys);
	//pdata->buffer_base=(u32)dma_alloc_coherent(NULL, BUFFER_SIZE/2, &rxdma_buffer_phys, GFP_KERNEL|GFP_DMA);
	//printk("dma_alloc_coherent vaddr: 0x%x dma_addr:0x%x\n",pdata->buffer_base, rxdma_buffer_phys);
	
	dma_cap_zero(eim_dma_mask);
	dma_cap_set(DMA_SLAVE, eim_dma_mask);

	eim_dma_data.priority =DMA_PRIO_MEDIUM;/* DMA_PRIO_HIGH;*/
	eim_dma_data.dma_request = MX6Q_DMA_REQ_EXT_DMA_REQ_1;
	eim_dma_data.peripheral_type = IMX_DMATYPE_MEMORY;/*IMX_DMATYPE_FIFO_MEMORY;*/
	eim_chan = dma_request_channel(eim_dma_mask, filter, &eim_dma_data);
	printk("dma chan: 0x%x\n", eim_chan);
	if(eim_chan == 0)
	{
		printk("dma_request_channel err\n");
		return -1;
	}
#endif
#endif

#if 1
	order=get_order(BUFFER_SIZE);
	pdata->buffer_base=__get_free_pages(GFP_KERNEL,order);	
	if(pdata->buffer_base == NULL)
	{
		printk("dma_buffer __get_free_pages failed\n");
	}
	//int len=0x1000*(1<<BUFFER_PAGE_ORDER);
	int len=BUFFER_SIZE;
	pdata->buffer_len=len;
	//rxdma_buffer_phys=dma_map_single(NULL, pdata->buffer_base,len,DMA_FROM_DEVICE);
	//printk("tx dma addr: vir:0x%x phys:0x%x\n", pdata->buffer_base, rxdma_buffer_phys);
	//if(eim_chan)
	//	dma_dev = eim_chan->device;

	//unsigned long virt_to_bus(volatile void *address);

	printk("virt_to_bus addr: 0x%x\n",virt_to_bus((void *)pdata->buffer_base));
#endif
	
#ifdef FPGA_DMA

#if 0
	//sg_init_one(&eim_rx_sg,  pdata->buffer_base,  0x8000);  
	//printk("chan:0x%x 0x%x 0x%x\n", eim_chan, eim_chan->device, eim_chan->device->device_prep_dma_memcpy);
	sg_init_table(eim_rx_sg, SG_NUM);
	for (index = 0; index < SG_NUM; index++) 
	{
			sg_set_buf(&eim_rx_sg[index],pdata->buffer_base + index * 0x8000,0x8000);
	}

	if (dma_map_sg(eim_chan->device->dev, eim_rx_sg, SG_NUM, DMA_DEV_TO_MEM) != SG_NUM) {
		printk("unable to map RX DMA\n");
		return -1;
	}	
	eim_rx_sg[0].dma_address=rxdma_buffer_phys;
	eim_rx_sg[1].dma_address=rxdma_buffer_phys+sg_dma_len(&eim_rx_sg[0]);
	
	for(index=0;index<SG_NUM;index++)
	{
		printk("sg %d addr&len: 0x%x 0x%x\n", index, sg_dma_address(&eim_rx_sg[index]),sg_dma_len(&eim_rx_sg[index]));
	}

	eimdma_slave_config.direction= DMA_DEV_TO_MEM;            
	eimdma_slave_config.src_addr = EIMRAM_BASE;
	eimdma_slave_config.src_addr_width= DMA_SLAVE_BUSWIDTH_4_BYTES;
	eimdma_slave_config.src_maxburst=1200;
	//eimdma_slave_config.dst_addr= rxdma_buffer_phys;        
	//eimdma_slave_config.dst_addr_width= DMA_SLAVE_BUSWIDTH_4_BYTES;
	//eimdma_slave_config.dst_maxburst=1200;
	retval=dmaengine_slave_config(eim_chan, &eimdma_slave_config);
	if(retval)
	{
		printk("dmaengine_slave_config err: %d\n", retval);
	}


  	//sg_dma_address(&eim_rx_sg) = rxdma_buffer_phys;
	//sg_dma_len(&eim_rx_sg) =  FPGA_DATADEP*4;
	//dma_flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT| 
	//	DMA_COMPL_SKIP_DEST_UNMAP |DMA_COMPL_SKIP_DEST_UNMAP|DMA_PREP_CONTINUE;
	//dma_flags=DMA_PREP_INTERRUPT|DMA_COMPL_SKIP_DEST_UNMAP|DMA_COMPL_SKIP_SRC_UNMAP;
	//dma_flags=DMA_CTRL_ACK|DMA_PREP_INTERRUPT|DMA_COMPL_SKIP_DEST_UNMAP|DMA_COMPL_SKIP_SRC_UNMAP;
	dma_flags=DMA_CTRL_ACK|DMA_PREP_INTERRUPT;
	if(eim_chan)
	{
		//printk("chan:0x%x 0x%x 0x%x\n", eim_chan, eim_chan->device, eim_chan->device->device_prep_dma_memcpy);
		//dma_rxdesc = eim_chan->device->device_prep_dma_memcpy(eim_chan,
		//	 rxdma_buffer_phys, EIMRAM_BASE, FPGA_DATADEP, dma_flags);
		dma_rxdesc=eim_chan->device->device_prep_slave_sg(eim_chan, eim_rx_sg, SG_NUM, DMA_DEV_TO_MEM, dma_flags);
	}
	else
	{
		printk("eim dma chan err\n");
	}

	if (!dma_rxdesc) {
		printk("unable to prep FPGA DMA\n");
	}
#endif

#endif

	memset(&(pdata->rb), 0, sizeof(ringbuffer));
	pdata->rb.buffer=(u8 *)pdata->buffer_base;
	pdata->rb.size=pdata->buffer_len;
	pdata->rb.wr=pdata->rb.rd=pdata->rb.buffer;
	sema_init(&(pdata->rb.sem), 1);
	printk("%s 0x%x 0x%x 0x%x order:0x%x\n", __func__, pdata->rb.buffer, pdata->rb.size, pdata->rb.wr, order);


	spin_lock_init(&(pdata->rb.spinlock));

	
	pdata->iomux_base=ioremap(IOMUX_BASE, SZ_4K);

	eimfpga_pads_cnt=ARRAY_SIZE(eimfpga_pads);
	mxc_iomux_v3_setup_multiple_pads(eimfpga_pads, eimfpga_pads_cnt, pdata->iomux_base);
	//printk("%s iomux:0x%x\n", __func__, pdata->iomux_base);	
	
	pdata->ccm_base=ioremap(CCM_BASE, SZ_4K);
	//printk("%s ccm_base:0x%x\n", __func__, pdata->ccm_base);

#ifdef NONMUX
	pdata->gpio_base=ioremap(GPIO2_BASE, SZ_4K);
	//printk("%s gpio_base:0x%x\n", __func__, pdata->gpio_base);
	value=__raw_readl(pdata->gpio_base+GPIO2_GDIR);
	//printk("pre GPIO2_GDIR :0x%x\n", value);
	value |= (0x00002000);
	__raw_writel(value, pdata->gpio_base+GPIO2_GDIR);

	value=__raw_readl(pdata->gpio_base+GPIO2_DR);
	//printk("pre GPIO2_DR :0x%x\n", value);
	value &= ~(0x00002000);
	//value |= 0x00002000;
	//value|=0x00004000;
	__raw_writel(value, pdata->gpio_base+GPIO2_DR);

	pdata->gpio1_base=ioremap(GPIO1_BASE, SZ_4K);
	//printk("%s gpio_base:0x%x\n", __func__, pdata->gpio1_base);
	value=__raw_readl(pdata->gpio1_base+GPIOx_GDIR);
	//printk("pre GPIO1_GDIR :0x%x\n", value);
	value |= (0x00000c00);
	__raw_writel(value, pdata->gpio1_base+GPIOx_GDIR);

	value=__raw_readl(pdata->gpio1_base+GPIOx_DR);
	//printk("pre GPIO1_DR :0x%x\n", value);
	value &= ~(0x00000c00);
	//value |= 0x00002000;
	//value|=0x00004000;
	__raw_writel(value, pdata->gpio1_base+GPIOx_DR);
#else
	pdata->gpio_base=ioremap(GPIO2_BASE, SZ_4K);
	//printk("%s gpio_base:0x%x\n", __func__, pdata->gpio_base);
	value=__raw_readl(pdata->gpio_base+GPIO2_GDIR);
	//printk("pre GPIO2_GDIR :0x%x\n", value);
	value |= (0x0000e000);
	__raw_writel(value, pdata->gpio_base+GPIO2_GDIR);

	value=__raw_readl(pdata->gpio_base+GPIO2_DR);
	//printk("pre GPIO2_DR :0x%x\n", value);
	value &= ~(0x0000e000);
	//value |= 0x00002000;
	//value|=0x00004000;
	__raw_writel(value, pdata->gpio_base+GPIO2_DR);
#endif


	pdata->gpio3_base=ioremap(GPIO3_BASE, SZ_4K);
	value=__raw_readl(pdata->gpio3_base+GPIO3_GDIR);
	//printk("pre GPIO3_GDIR :0x%x\n", value);
	//value |= (0x02000000);
	value |= (0x02000000);
	__raw_writel(value, pdata->gpio3_base+GPIO3_GDIR);

	value=__raw_readl(pdata->gpio3_base+GPIO3_DR);
	//printk("pre GPIO3_DR :0x%x\n", value);
	//value &= ~(0x0000e000);
	//value |= 0x02000000;
	value &= ~(0x02000000);
	__raw_writel(value, pdata->gpio3_base+GPIO3_DR);


	pdata->gpio6_base=ioremap(GPIO6_BASE, SZ_16K);
	value=__raw_readl(pdata->gpio6_base+GPIO6_GDIR);
	//printk("pre GPIO6_GDIR :0x%x\n", value);
	value &= ~(0x00004000);
	value |= 0x00000800; //GPIO6_11 for reset FIFO
	__raw_writel(value, pdata->gpio6_base+GPIO6_GDIR);
	
	value=__raw_readl(pdata->gpio6_base+GPIO6_ICR1);
	//printk("pre GPIO6_ICR1 :0x%x\n", value);
	value &= ~(0x30000000);
	//value |= (0x10000000);
	value |= (0x20000000);
	__raw_writel(value, pdata->gpio6_base+GPIO6_ICR1);
	
	value=__raw_readl(pdata->gpio6_base+GPIO6_IMR);
	//printk("pre GPIO6_IMR :0x%x\n", value);
	//value |= (0x00004000);
	value &= ~(0x00004000);
	//value=0xffffbfff;
	__raw_writel(value, pdata->gpio6_base+GPIO6_IMR);

	msleep(1000);

	//////add gpio API///
	retval=gpio_request(FPGAINTR_GPIO, "fpga_gpio");
	if (retval) {
		printk(KERN_ERR"failed to get GPIO fpga_gpio:"
			" %d\n", retval);
		return;
	}
	gpio_direction_input(FPGAINTR_GPIO);
		

	pdata->eim_cs0_base=ioremap(EIM_BASE, SZ_4K);
	//printk("%s eim_cs0_base:0x%x\n", __func__, pdata->eim_cs0_base);
	
	/* CLKCTL_CCGR6: Set emi_slow_clock to be on in all modes */
	value=__raw_readl(pdata->ccm_base+0x80);
	value |= 0x00000c00;
	__raw_writel(value, pdata->ccm_base+0x80);

	value=__raw_readl(pdata->ccm_base+0x80);
	//printk("%s ccm 0x80 :0x%x\n", __func__, value);
	
	/* Timing settings below based upon datasheet for M29W256GL7AN6E
	   These setting assume that the EIM_SLOW_CLOCK is set to 132 MHz */
	clk = clk_get(NULL, "emi_slow_clk");
	if (IS_ERR(clk))
		printk(KERN_ERR "emi_slow_clk not found\n");

	pdata->clk=clk;
	rate = clk_get_rate(clk);
	if (rate != 132000000)
		printk(KERN_ERR "Warning: emi_slow_clk not set to 132 MHz!" 	" WEIM NOR timing may be incorrect!\n");

	/*
	 * For EIM General Configuration registers.
	 *
	 * CS0GCR1:
	 *	PSZ=0
	 * 	GBC = 0; CSREC = 6; DSZ = 2; BL = 0;
	 *	CREP = 1; CSEN = 1;
	 *	
	 *	EIM Operation Mode: MUM = SRD = SWR = 0.
	 *		(Async write/Async page read, none multiplexed)
	 *
	 * CS0GCR2:
	 *	ADH = 1
	 */
	//__raw_writel(0x07f1f001, pdata->eim_cs0_base+EIM_CSn_OFFSET(0)+EIM_GCR1_OFFSET);
#ifdef NONMUX
	//0x0191009f
	__raw_writel(0x019100bf, pdata->eim_cs0_base+EIM_CSn_OFFSET(0)+EIM_GCR1_OFFSET);
    	__raw_writel(0x00000000, pdata->eim_cs0_base+EIM_CSn_OFFSET(0)+EIM_GCR2_OFFSET);
#else
	__raw_writel(0x07f1f007, pdata->eim_cs0_base+EIM_CSn_OFFSET(0)+EIM_GCR1_OFFSET);
    	__raw_writel(0x00001800, pdata->eim_cs0_base+EIM_CSn_OFFSET(0)+EIM_GCR2_OFFSET);
#endif		
	/*
	 * For EIM Read Configuration registers.
	 *
	 * CS0RCR1:
	 *	RWSC = 1C;
	 *	RADVA = 0; RADVN = 2;
	 *	OEA = 2; OEN = 0;
	 *	RCSA = 0; RCSN = 0
	 *
	 * CS0RCR2:
	 *	APR = 1 (Async Page Read);
	 *	PAT = 4 (6 EIM clock sycles)
	 */
	__raw_writel(0x01000000, pdata->eim_cs0_base+EIM_CSn_OFFSET(0)+EIM_RCR1_OFFSET);
    	__raw_writel(0x00000008, pdata->eim_cs0_base+EIM_CSn_OFFSET(0)+EIM_RCR2_OFFSET);
		
	/*
	 * For EIM Write Configuration registers.
	 *
	 * CS0WCR1:
	 *	WWSC = 20;
	 *	WADVA = 0; WADVN = 1;
	 *	WBEA = 1; WBEN = 2;
	 *	WEA = 1; WEN = 6;
	 *	WCSA = 1; WCSN = 2;
	 *
	 * CS0WCR2:
	 *	WBCDD = 0
	 */
	__raw_writel(0x01000000, pdata->eim_cs0_base+EIM_CSn_OFFSET(0)+EIM_WCR1_OFFSET);
    	__raw_writel(0x00000008, pdata->eim_cs0_base+EIM_CSn_OFFSET(0)+EIM_WCR2_OFFSET);
	
	__raw_writel(0x00000020, pdata->eim_cs0_base+EIM_WCR);


	value=__raw_readl(pdata->gpio6_base+GPIO6_IMR);
	//printk("pre GPIO6_IMR :0x%x\n", value);
	value &= ~(0x00004000);
	//value |= (0x00004000);
	__raw_writel(value, pdata->gpio6_base+GPIO6_IMR);



	value = __raw_readl(pdata->iomux_base+0x4);/*IOMUX_GPR1*/
	//printk("gpr1: 0x%x\n", value);
	value &=~0x00000fff;
	value|=0x000000249;
	__raw_writel(value, pdata->iomux_base+0x4);

	pdata->ram_base=devm_ioremap(&pdev->dev, EIMRAM_BASE , SZ_16M);
	/*pdata->ram_base=ioremap(EIMRAM_BASE, SZ_64K);*/
	//printk("ram base:0x%x\n", pdata->ram_base);	



	//retval = request_irq(pdata->irq_num, fpga_isr, IRQF_TRIGGER_LOW, "upp_intr", pdev);
	//retval = request_irq(pdata->irq_num, fpga_isr, IRQF_SHARED, "upp_intr", pdev);
	retval=0;
	//IRQ_TYPE_EDGE_RISING
	retval = request_irq(gpio_to_irq(FPGAINTR_GPIO), fpga_isr, IRQF_SHARED, "fpga_intr", pdev);
	if(retval != 0)
	{
		printk("request_irq: 0x%x failed\n", retval);
		return -1;
	}
	else
	{
		printk("request_irq 0x%x \n", gpio_to_irq(FPGAINTR_GPIO));
	}
	//int cpumask=0x8;
	//irq_set_affinity(gpio_to_irq(FPGAINTR_GPIO),cpumask_of(0x4));

#ifdef EIM_TASKLET
	tasklet_init(&rxtasklet, eimfpga_rxfunc,(unsigned long)pdata);
#endif

#ifdef EIM_WORK
	INIT_WORK(&eimrxwork, eimfpga_rxfunc);
#endif

	value=__raw_readl(pdata->gpio6_base+GPIO6_IMR);
	//printk("pre GPIO6_IMR :0x%x\n", value);
	value |= (0x00004000);
	//value &= ~(0x00004000);
	__raw_writel(value, pdata->gpio6_base+GPIO6_IMR);

	//reset FIFO
	value=__raw_readl(pdata->gpio6_base+GPIO6_DR);
	value |= 0x00000800;
	__raw_writel(value, pdata->gpio6_base+GPIO6_DR);
	msleep(500);
	value=__raw_readl(pdata->gpio6_base+GPIO6_DR);
	value &= (~0x00000800);
	__raw_writel(value, pdata->gpio6_base+GPIO6_DR);	
	
#if 0
	msleep(1000);

	value=__raw_readl(pdata->gpio6_base+GPIO6_DR);
	value |= 0x00000800;
	__raw_writel(value, pdata->gpio6_base+GPIO6_DR);
	msleep(5);
	value=__raw_readl(pdata->gpio6_base+GPIO6_DR);
	value &= (~0x00000800);
	__raw_writel(value, pdata->gpio6_base+GPIO6_DR);	
#endif

	return 0;
}


static int   fpga_remove(struct platform_device *pdev)
{
	volatile int index=0;
	eimfpga_info *pdata = pdev->dev.platform_data;
	volatile unsigned long value=0;
	int retval;

	disable_irq(gpio_to_irq(FPGAINTR_GPIO));
	free_irq(gpio_to_irq(FPGAINTR_GPIO), pdev);
	if(eim_chan)
		dma_release_channel(eim_chan);
	if(rxdma_buffer_phys)
		dma_unmap_single(NULL, rxdma_buffer_phys,pdata->buffer_len, DMA_FROM_DEVICE);

	free_pages(pdata->buffer_base, BUFFER_PAGE_ORDER);

	//free_irq(gpio_to_irq(FPGAINTR_GPIO), pdev);
	tasklet_disable(&rxtasklet);
	tasklet_kill(&rxtasklet);

	devm_iounmap(&pdev->dev, pdata->ram_base);
	clk_put(pdata->clk);
	gpio_free(FPGAINTR_GPIO);
	
	iounmap(pdata->iomux_base);
	iounmap(pdata->ccm_base);
	iounmap(pdata->eim_cs0_base);
	return ;
}


static int __init eimfpga_init(void)
{
	if (register_chrdev(EIMFPGA_MAJOR, EIMFPGA_DRVNAME, &fpga_fops)) 
	{
		printk(KERN_ERR,"adb: unable to get major %d\n", EIMFPGA_MAJOR);
		return;
	}

	eimfpga_class = class_create(THIS_MODULE, EIMFPGA_DRVNAME);
	if (IS_ERR(eimfpga_class))
		return;
	
	device_create(eimfpga_class, NULL, MKDEV(EIMFPGA_MAJOR, 0), NULL, EIMFPGA_DRVNAME);
	
	platform_device_register(&fpga_pfdev);
	platform_driver_probe(&fpga_pfdrv, fpga_probe);

}


static void __exit  eimfpga_exit(void)
{
	platform_driver_unregister(&fpga_pfdrv);
	platform_device_unregister(&fpga_pfdev);
	
	device_destroy(eimfpga_class, MKDEV(EIMFPGA_MAJOR, 0));
	class_destroy(eimfpga_class);
	unregister_chrdev(EIMFPGA_MAJOR, EIMFPGA_DRVNAME);
	return 0;	
}

module_init(eimfpga_init);
module_exit(eimfpga_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("FANG SA");

