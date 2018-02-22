#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kernel.h> 
#include <linux/slab.h>
#include <linux/errno.h> 
#include <linux/types.h>  
#include <linux/interrupt.h>
#include <linux/in.h>
#include <linux/netdevice.h>   
#include <linux/etherdevice.h>
#include <linux/ip.h>        
#include <linux/tcp.h>         
#include <linux/skbuff.h>
#include <linux/if_ether.h>
#include <linux/in6.h>
#include <linux/irq.h>
#include <mach/regs-mem.h>

#include <asm/uaccess.h>
#include <asm/checksum.h>

#define DMA0_BASE_ADDR  0x4B000000
#define DMA1_BASE_ADDR  0x4B000040
#define DMA2_BASE_ADDR  0x4B000080
#define DMA3_BASE_ADDR  0x4B0000C0

//static DECLARE_WAIT_QUEUE_HEAD(dma0_waitq);
//static DECLARE_WAIT_QUEUE_HEAD(dma1_waitq);


#define IO_BASE			0x08000000
#define MTU 2048

static volatile  int ev_dma_0=0;
static volatile  int ev_dma_1=0;

int watchdog = 5000;

 struct fpga_priv {
    struct net_device_stats stats;//有用的统计信息
    int status;//网络设备的状态信息，是发完数据包，还是接收到网络数据包
    int rx_packetlen;//接收到的数据包长度
    u8 *rx_packetdata;//接收到的数据
    int tx_packetlen;//发送的数据包长度
    u8 *tx_packetdata;//发送的数据
    spinlock_t lock;
    struct sk_buff *skb;//socket buffer结构体，网络各层之间传送数据都是通过这个结构体来实现的
};
 struct s3c_dma_regs {  
	unsigned long disrc;  
	unsigned long disrcc;  
	unsigned long didst;  
	unsigned long didstc;  
	unsigned long dcon;  
	unsigned long dstat;  
	unsigned long dcsrc;  
	unsigned long dcdst;  
	unsigned long dmasktrig;  
};

 struct net_device* fpga_devs;

 volatile struct s3c_dma_regs *dma0_regs;
 volatile struct s3c_dma_regs *dma1_regs;

 unsigned int  recv_len_reg;
 unsigned int  send_len_reg;

 dma_addr_t send_src; 
 unsigned char *src_1 = NULL; 
 dma_addr_t send_dest=0x08000004;

 dma_addr_t recv_dest; 
 unsigned char *dest = NULL; 
 dma_addr_t recv_src=0x08000004;

//接包函数
static unsigned char* fpga_read(int len)
{
	ev_dma_1 = 0;
	
	memset(dest,0,MTU);
	
    dma1_regs->disrc =recv_src; /* 源的物理地址 */  
    dma1_regs->disrcc =(0<<1) | (0<<0); /* 源位于AHB总线, 源地址递增 */  
	
    dma1_regs->didst =recv_dest; /* 目的的物理地址 */  
    dma1_regs->didstc=(0<<2) | (0<<1) | (0<<0); /* 目的位于AHB总线, 目的地址递增 */  
    dma1_regs->dcon  = (1<<30)|(1<<29)|(0<<28)|(1<<27)|(0<<23)|(1<<20)|((len/2)<<0); /* 使能中断,单个传输,软件触发, */  

    /* 启动DMA */  
    dma1_regs->dmasktrig = (1<<1) |(1<<0);  

//	printk("recv finish\n");
//	wait_event_interruptible(dma1_waitq, ev_dma_1);

	while(ev_dma_1==0);

	return dest;

}

static int fpga_write(char *buf, int len)
{

	ev_dma_0 = 0; 
	
	if(len>60)
		len=len+1;
	
	memset(src_1,0,MTU);
	
	memcpy(src_1, buf, len);

	dma0_regs->disrc =send_src; 
	dma0_regs->disrcc =(0<<1) | (0<<0); 
	
	dma0_regs->didst =send_dest; 
	dma0_regs->didstc =(0<<2) | (0<<1) | (0<<0); 

	dma0_regs->dcon	=(1<<30)|(1<<29)|(0<<28)|(1<<27)|(0<<23)|(1<<20)|((len/2)<<0);
	dma0_regs->dmasktrig = (1<<1) |(1<<0);  

//	printk("send finish\n");
//	wait_event_interruptible(dma0_waitq, ev_dma_0);
	
	while(ev_dma_0==0);

	return 0;
}

/*还需要从网卡知道要读的长度*/
static void fpga_rx(struct net_device *dev)
{
	int len;
    struct sk_buff *skb;

	len=ioread16(recv_len_reg);
	
    skb = dev_alloc_skb(len+2);//分配一个socket buffer,并且初始化skb->data,skb->tail和skb->head
    if (!skb) 
	{
        printk("fpga rx: low on mem - packet dropped/n");
		return ;
    }
	
    skb_reserve(skb, 2); /* align IP on 16B boundary */ 
	skb->dev = dev;
	
	skb->data=fpga_read(len);
/*
	int i;
	for(i=0;i<len;i++)
		printk("%x",*(skb->data+i));
	
	printk("\n");
*/
//	if(len%2!=0)
//	else
		
	memcpy(skb_put(skb, len),skb->data,len);

    /* Write metadata, and then pass to the receive level */
    skb->protocol = eth_type_trans(skb, dev);//返回的是协议号
    skb->ip_summed = CHECKSUM_UNNECESSARY; //此处不校验
	
    if(NET_RX_SUCCESS!=netif_rx(skb))//通知内核已经接收到包，并且封装成socket buffer传到上层
    {
		printk("rx fail\n");
	}

	dev->last_rx=jiffies;
	dev->stats.rx_bytes +=len;
	dev->stats.rx_packets ++;

}

static irqreturn_t fpga_interrupt(int irq, void *dev_id)
{
	struct net_device *dev;

	dev=dev_id;

	fpga_rx(dev);

    return IRQ_HANDLED;
}

//dma数据结束产生中断
static irqreturn_t fpga_dma0_irq(int irq, void *devid)
{
	
//	wake_up_interruptible(&dma0_waitq);
	ev_dma_0 = 1;
	
	return IRQ_HANDLED;
}

static irqreturn_t fpga_dma1_irq(int irq, void *devid)
{
	
//	wake_up_interruptible(&dma1_waitq);
	ev_dma_1 = 1;
	
	return IRQ_HANDLED;
}

static void fpga_tx_timeout (struct net_device *dev)
{
	struct fpga_priv* priv;
	
	priv = netdev_priv(dev);

    printk("Transmit timeout at %ld, latency %ld \n", jiffies,jiffies - dev->trans_start);
    
    priv->stats.tx_errors++;//发送的错误数
    
    netif_wake_queue(dev); //为了再次发送数据，调用此函数，重新启动发送队列
}


static int fpga_open(struct net_device *dev)
 {
	int ret;
		
	/* 这里注册一个中断，当DMA数据传输完毕之后会发生此中断 */
	ret=request_irq(IRQ_DMA0, fpga_dma0_irq, 0, "dma0_fpga",(void *)1);
	if (ret<0)
	{
		printk("can't request_irq for DMA\n");
		return ret;
	}
	ret=request_irq(IRQ_DMA1, fpga_dma1_irq, 0, "dma1_fpga",(void *)1);
	if (ret<0)
	{
		printk("can't request_irq for DMA\n");
		return ret;
	}
	
	memcpy(dev->dev_addr, "Px08.x", ETH_ALEN);//分配一个硬件地址，ETH＿ALEN是网络设备硬件地址的长度

	dest= dma_alloc_coherent(NULL,MTU, &recv_dest, GFP_DMA);	
	if(NULL==dest)
	{
		printk("can't dma_alloc_coherent\n");
		return -1;
	}
	printk("dest = 0x%x, recv_dest = 0x%x\n",(unsigned int)dest, recv_dest);	

	src_1= dma_alloc_coherent(NULL,MTU, &send_src, GFP_DMA);	
	if(NULL==src_1)
	{
		printk("can't dma_alloc_coherent\n");
		return -1;
	}
	printk("src_1 = 0x%x, send_src = 0x%x\n",(unsigned int)src_1, send_src);	
		//申请中断
	ret = request_irq(IRQ_EINT9,fpga_interrupt,IRQ_TYPE_EDGE_FALLING,"int_fpga",dev);
	if(ret<0)
	{
		printk("can't request irq for int_fpga\n");
	    return ret;
	}

	dev->trans_start = 0;
	netif_start_queue(dev);

    return 0;
}

static int fpga_stop(struct net_device *dev)
{
 	printk("%s\n",__FUNCTION__);

	free_irq(IRQ_EINT9,dev);
	
	free_irq(IRQ_DMA0,(void *)1);
	free_irq(IRQ_DMA1,(void *)1);


	dma_free_coherent(NULL, MTU, dest, recv_dest);  
	dma_free_coherent(NULL, MTU, src_1, send_src);
	
    netif_stop_queue(dev); //当网络接口关闭的时候，调用stop方法，这个函数表示不能再发送数据
    
    return 0;
}


//发包函数
static int hard_start_xmit(struct sk_buff *skb, struct net_device *dev)
{	
	int len;
	unsigned char *data, shortpkt[ETH_ZLEN];

	netif_stop_queue(dev);

/* 获得有效数据指针和长度 */
	data = skb->data;
	len = skb->len;
	if(len < ETH_ZLEN)
	{
		/* 如果帧长小于以太网帧最小长度,补0 */
		memset(shortpkt,0,ETH_ZLEN);
		memcpy(shortpkt,skb->data,skb->len);
		len = ETH_ZLEN;
		data = shortpkt;
	}

	dev->trans_start = jiffies; 	//记录发送时间戳

	/*写给fpga,让其发出 */
	 fpga_write(data,len);
	
	 iowrite16(len,send_len_reg);

	 dev->trans_start=jiffies;
	 dev->stats.tx_packets++;
	 dev->stats.tx_bytes +=len;
	 
	 		
	dev_kfree_skb(skb);
	netif_wake_queue(dev);
	
    return 0; /* Our simple device can not fail */
}

static const struct net_device_ops fpga_netdev_ops = {
	.ndo_open			= fpga_open,
	.ndo_stop			= fpga_stop,
	.ndo_start_xmit		= hard_start_xmit,
	.ndo_tx_timeout		= fpga_tx_timeout,
	//.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address= eth_mac_addr,
	.ndo_change_mtu		= eth_change_mtu,
};


//设备初始化函数
static __init int fpga_eth_init(void)
{
	int ret;
	struct fpga_priv* priv;
	unsigned int oldval_bwscon;

	/* 设置S3C2410_BWSCON: 数据总线宽度16bits ，使用UB/LB，禁止WAIT */
	oldval_bwscon = *(volatile unsigned int *)S3C2410_BWSCON;

	*((volatile unsigned int *)S3C2410_BWSCON) = (oldval_bwscon & ~(3<<5)) |
	S3C2410_BWSCON_DW1_16 | S3C2410_BWSCON_ST1;

//	*((volatile unsigned int *)S3C2410_BANKCON1) = 0x0300;
	*((volatile unsigned int *)S3C2410_BANKCON1) = 0x1f7c;


	fpga_devs = alloc_netdev(sizeof(struct fpga_priv), "vnet%d", ether_setup);
	if(NULL==fpga_devs)
	{
		printk("error:alloc_netdev\n");
		return -1;
	}

	fpga_devs->netdev_ops = &fpga_netdev_ops;
	fpga_devs->watchdog_timeo	= msecs_to_jiffies(watchdog);

	priv = netdev_priv(fpga_devs);

	ret=register_netdev(fpga_devs);
	if(ret)//注册设备
	{
		printk("fpga: error %i registering device %s \n",ret, fpga_devs->name);
		return -1;
	}

	dma0_regs = ioremap(DMA0_BASE_ADDR, sizeof(struct s3c_dma_regs));
	dma1_regs = ioremap(DMA1_BASE_ADDR, sizeof(struct s3c_dma_regs));

//从0x00读取长度
	recv_len_reg 	= ioremap(IO_BASE, 2);

	send_len_reg	= recv_len_reg+2;

	return 0;
}

static __exit void fpga_eth_exit(void)
{
 	printk("%s\n",__FUNCTION__);
		
	iounmap(dma0_regs);
	iounmap(dma1_regs);

	iounmap((volatile void *)recv_len_reg);
    unregister_netdev(fpga_devs);

}

module_init(fpga_eth_init);
module_exit(fpga_eth_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("fpga_eth driver");
MODULE_AUTHOR(" QF&&2018.1.15");
