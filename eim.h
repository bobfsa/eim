#include "iomux-mx6q.h"
#include "iomux-v3.h"

#define NONMUX 

#define FPGA_DMA

#define EIM_TASKLET 
//#define EIM_WORK

/*static iomux_v3_cfg_t eimfpga_pads[] __initdata = {*/
static iomux_v3_cfg_t eimfpga_pads[] = {
	MX6Q_PAD_EIM_OE__WEIM_WEIM_OE,
	MX6Q_PAD_EIM_RW__WEIM_WEIM_RW,
	MX6Q_PAD_EIM_WAIT__WEIM_WEIM_WAIT,
	MX6Q_PAD_EIM_CS0__WEIM_WEIM_CS_0,
	MX6Q_PAD_NANDF_CS2__WEIM_WEIM_CRE,
	/*Control NOR reset using gpio mode*/
	/*MX6Q_PAD_DISP0_DAT8__GPIO_4_29,*/

	MX6Q_PAD_EIM_LBA__WEIM_WEIM_LBA,
	MX6Q_PAD_EIM_BCLK__WEIM_WEIM_BCLK,
/*
	MX6Q_PAD_EIM_D16__WEIM_WEIM_D_16,
	MX6Q_PAD_EIM_D17__WEIM_WEIM_D_17,
	MX6Q_PAD_EIM_D18__WEIM_WEIM_D_18,
	MX6Q_PAD_EIM_D19__WEIM_WEIM_D_19,
	MX6Q_PAD_EIM_D20__WEIM_WEIM_D_20,
	MX6Q_PAD_EIM_D21__WEIM_WEIM_D_21,
	MX6Q_PAD_EIM_D22__WEIM_WEIM_D_22,
	MX6Q_PAD_EIM_D23__WEIM_WEIM_D_23,
*/
	
/*
	MX6Q_PAD_EIM_D24__WEIM_WEIM_D_24,
	MX6Q_PAD_EIM_D25__WEIM_WEIM_D_25,
	MX6Q_PAD_EIM_D26__WEIM_WEIM_D_26,
	MX6Q_PAD_EIM_D27__WEIM_WEIM_D_27,
	MX6Q_PAD_EIM_D28__WEIM_WEIM_D_28,
	MX6Q_PAD_EIM_D29__WEIM_WEIM_D_29,
	MX6Q_PAD_EIM_D30__WEIM_WEIM_D_30,
	MX6Q_PAD_EIM_D31__WEIM_WEIM_D_31,
*/
	/* Parallel Nor 26 bit Address Bus */
	//MX6Q_PAD_EIM_A24__GPIO_5_4,
/*
	MX6Q_PAD_EIM_A25__WEIM_WEIM_A_25,
	MX6Q_PAD_EIM_A24__WEIM_WEIM_A_24,
	MX6Q_PAD_EIM_A23__WEIM_WEIM_A_23,
	MX6Q_PAD_EIM_A22__WEIM_WEIM_A_22,
	MX6Q_PAD_EIM_A21__WEIM_WEIM_A_21,
	MX6Q_PAD_EIM_A20__WEIM_WEIM_A_20,
	MX6Q_PAD_EIM_A19__WEIM_WEIM_A_19,
	MX6Q_PAD_EIM_A18__WEIM_WEIM_A_18,
	MX6Q_PAD_EIM_A17__WEIM_WEIM_A_17,
	MX6Q_PAD_EIM_A16__WEIM_WEIM_A_16,
*/
	MX6Q_PAD_EIM_DA15__WEIM_WEIM_DA_A_15,
	MX6Q_PAD_EIM_DA14__WEIM_WEIM_DA_A_14,
	MX6Q_PAD_EIM_DA13__WEIM_WEIM_DA_A_13,
	MX6Q_PAD_EIM_DA12__WEIM_WEIM_DA_A_12,
	MX6Q_PAD_EIM_DA11__WEIM_WEIM_DA_A_11,
	MX6Q_PAD_EIM_DA10__WEIM_WEIM_DA_A_10,
	MX6Q_PAD_EIM_DA9__WEIM_WEIM_DA_A_9,
	MX6Q_PAD_EIM_DA8__WEIM_WEIM_DA_A_8,
	MX6Q_PAD_EIM_DA7__WEIM_WEIM_DA_A_7,
	MX6Q_PAD_EIM_DA6__WEIM_WEIM_DA_A_6,
	MX6Q_PAD_EIM_DA5__WEIM_WEIM_DA_A_5,
	MX6Q_PAD_EIM_DA4__WEIM_WEIM_DA_A_4,
	MX6Q_PAD_EIM_DA3__WEIM_WEIM_DA_A_3,
	MX6Q_PAD_EIM_DA2__WEIM_WEIM_DA_A_2,
	MX6Q_PAD_EIM_DA1__WEIM_WEIM_DA_A_1,
	MX6Q_PAD_EIM_DA0__WEIM_WEIM_DA_A_0,

	MX6Q_PAD_SD4_DAT5__GPIO_2_13,
	MX6Q_PAD_SD2_CLK__GPIO_1_10,
	MX6Q_PAD_SD2_CMD__GPIO_1_11,
/*

	MX6Q_PAD_CSI0_DATA_EN__WEIM_WEIM_D_0,
	MX6Q_PAD_CSI0_VSYNC__WEIM_WEIM_D_1,
	MX6Q_PAD_CSI0_DAT4__WEIM_WEIM_D_2,
	MX6Q_PAD_CSI0_DAT5__WEIM_WEIM_D_3,
	//MX6Q_PAD_CSI0_DAT5__GPIO_5_23,
	MX6Q_PAD_CSI0_DAT6__WEIM_WEIM_D_4,
	MX6Q_PAD_CSI0_DAT7__WEIM_WEIM_D_5,
	MX6Q_PAD_CSI0_DAT8__WEIM_WEIM_D_6,
	MX6Q_PAD_CSI0_DAT9__WEIM_WEIM_D_7,
	MX6Q_PAD_CSI0_DAT12__WEIM_WEIM_D_8,
	MX6Q_PAD_CSI0_DAT13__WEIM_WEIM_D_9,
	MX6Q_PAD_CSI0_DAT14__WEIM_WEIM_D_10,
	MX6Q_PAD_CSI0_DAT15__WEIM_WEIM_D_11,
	MX6Q_PAD_CSI0_DAT16__WEIM_WEIM_D_12,
	MX6Q_PAD_CSI0_DAT17__WEIM_WEIM_D_13,
	MX6Q_PAD_CSI0_DAT18__WEIM_WEIM_D_14,
	MX6Q_PAD_CSI0_DAT19__WEIM_WEIM_D_15,

	
	MX6Q_PAD_SD4_DAT5__GPIO_2_13,
	MX6Q_PAD_SD4_DAT6__GPIO_2_14,
	MX6Q_PAD_SD4_DAT7__GPIO_2_15,
*/
	MX6Q_PAD_NANDF_CS0__GPIO_6_11, /*for reset FIFO*/
	MX6Q_PAD_NANDF_CS1__GPIO_6_14,
	
	MX6Q_PAD_EIM_D25__GPIO_3_25,
};
enum sdma_peripheral_type {
	IMX_DMATYPE_SSI,	/* MCU domain SSI */
	IMX_DMATYPE_SSI_SP,	/* Shared SSI */
	IMX_DMATYPE_MMC,	/* MMC */
	IMX_DMATYPE_SDHC,	/* SDHC */
	IMX_DMATYPE_UART,	/* MCU domain UART */
	IMX_DMATYPE_UART_SP,	/* Shared UART */
	IMX_DMATYPE_FIRI,	/* FIRI */
	IMX_DMATYPE_CSPI,	/* MCU domain CSPI */
	IMX_DMATYPE_CSPI_SP,	/* Shared CSPI */
	IMX_DMATYPE_SIM,	/* SIM */
	IMX_DMATYPE_ATA,	/* ATA */
	IMX_DMATYPE_CCM,	/* CCM */
	IMX_DMATYPE_EXT,	/* External peripheral */
	IMX_DMATYPE_MSHC,	/* Memory Stick Host Controller */
	IMX_DMATYPE_MSHC_SP,	/* Shared Memory Stick Host Controller */
	IMX_DMATYPE_DSP,	/* DSP */
	IMX_DMATYPE_MEMORY,	/* Memory */
	IMX_DMATYPE_FIFO_MEMORY,/* FIFO type Memory */
	IMX_DMATYPE_SPDIF,	/* SPDIF */
	IMX_DMATYPE_IPU_MEMORY,	/* IPU Memory */
	IMX_DMATYPE_ASRC,	/* ASRC */
	IMX_DMATYPE_ESAI,	/* ESAI */
	IMX_DMATYPE_HDMI,
};
enum imx_dma_prio {
	DMA_PRIO_HIGH = 0,
	DMA_PRIO_MEDIUM = 1,
	DMA_PRIO_LOW = 2
};

/*
 * DMA request assignments
 */
#define MX6Q_DMA_REQ_VPU		0
#define MX6Q_DMA_REQ_GPC		1
#define MX6Q_DMA_REQ_IPU1		2
#define MX6Q_DMA_REQ_EXT_DMA_REQ_0	2
#define MX6Q_DMA_REQ_CSPI1_RX		3
#define MX6Q_DMA_REQ_I2C3_A		3
#define MX6Q_DMA_REQ_CSPI1_TX		4
#define MX6Q_DMA_REQ_I2C2_A		4
#define MX6Q_DMA_REQ_CSPI2_RX		5
#define MX6Q_DMA_REQ_I2C1_A		5
#define MX6Q_DMA_REQ_CSPI2_TX		6
#define MX6Q_DMA_REQ_CSPI3_RX		7
#define MX6Q_DMA_REQ_CSPI3_TX		8
#define MX6Q_DMA_REQ_CSPI4_RX		9
#define MX6Q_DMA_REQ_EPIT2		9
#define MX6Q_DMA_REQ_CSPI4_TX		10
#define MX6Q_DMA_REQ_I2C1_B		10
#define MX6Q_DMA_REQ_CSPI5_RX		11
#define MX6Q_DMA_REQ_CSPI5_TX		12
#define MX6Q_DMA_REQ_GPT		13
#define MX6Q_DMA_REQ_SPDIF_RX		14
#define MX6Q_DMA_REQ_EXT_DMA_REQ_1	14
#define MX6Q_DMA_REQ_SPDIF_TX		15
#define MX6Q_DMA_REQ_EPIT1		16
#define MX6Q_DMA_REQ_ASRC_RX1		17
#define MX6Q_DMA_REQ_ASRC_RX2		18
#define MX6Q_DMA_REQ_ASRC_RX3		19
#define MX6Q_DMA_REQ_ASRC_TX1		20
#define MX6Q_DMA_REQ_ASRC_TX2		21
#define MX6Q_DMA_REQ_ASRC_TX3		22
#define MX6Q_DMA_REQ_ESAI_RX		23
#define MX6Q_DMA_REQ_I2C3_B		23
#define MX6Q_DMA_REQ_ESAI_TX		24
#define MX6Q_DMA_REQ_UART1_RX		25
#define MX6Q_DMA_REQ_UART1_TX		26
#define MX6Q_DMA_REQ_UART2_RX		27
#define MX6Q_DMA_REQ_UART2_TX		28
#define MX6Q_DMA_REQ_UART3_RX		29
#define MX6Q_DMA_REQ_UART3_TX		30
#define MX6Q_DMA_REQ_UART4_RX		31
#define MX6Q_DMA_REQ_UART4_TX		32
#define MX6Q_DMA_REQ_UART5_RX		33
#define MX6Q_DMA_REQ_UART5_TX		34
#define MX6Q_DMA_REQ_SSI1_RX1		35
#define MX6Q_DMA_REQ_SSI1_TX1		36
#define MX6Q_DMA_REQ_SSI1_RX0		37
#define MX6Q_DMA_REQ_SSI1_TX0		38
#define MX6Q_DMA_REQ_SSI2_RX1		39
#define MX6Q_DMA_REQ_SSI2_TX1		40
#define MX6Q_DMA_REQ_SSI2_RX0		41
#define MX6Q_DMA_REQ_SSI2_TX0		42
#define MX6Q_DMA_REQ_SSI3_RX1		43
#define MX6Q_DMA_REQ_SSI3_TX1		44
#define MX6Q_DMA_REQ_SSI3_RX0		45
#define MX6Q_DMA_REQ_SSI3_TX0		46
#define MX6Q_DMA_REQ_DTCP		47


#define IOMUX_BASE 	0x020e0000
#define CCM_BASE	0x020c4000
#define EIM_BASE	0x021b8000
#define GPIO1_BASE	0x0209c000
#define GPIO2_BASE	0x020a0000
#define GPIO3_BASE	0x020a4000
#define GPIO5_BASE	0x020ac000
#define GPIO6_BASE	0x020b0000
#define GPIO7_BASE	0x020b4000

#define MMDC1_BASE	0x021b0000
#define MMDC2_BASE	0x021b4000

#define EIM_CSn_OFFSET(n) 	(n*0x18)
#define EIM_GCR1_OFFSET 	0x00
#define EIM_GCR2_OFFSET	0x04
#define EIM_RCR1_OFFSET 	0x08
#define EIM_RCR2_OFFSET 	0x0c
#define EIM_WCR1_OFFSET 	0x10
#define EIM_WCR2_OFFSET 	0x14
#define EIM_WCR 			0x90
#define EIM_WIAR 			0x94
#define EIM_EAR 				0x98

#define GPIO2_DR	0x0
#define GPIO2_GDIR	0x4
#define GPIO2_PSR	0x8
#define GPIO3_DR	0x0
#define GPIO3_GDIR	0x4
#define GPIO3_PSR	0x8

#define GPIO6_DR	0x0
#define GPIO6_GDIR	0x4
#define GPIO6_PSR	0x8
#define GPIO6_ICR1	0xc
#define GPIO6_ICR2	0x10
#define GPIO6_IMR	0x14
#define GPIO6_ISR	0x18

#define GPIOx_DR	0x0
#define GPIOx_GDIR	0x4
#define GPIOx_PSR	0x8
#define GPIOx_ICR1	0xc
#define GPIOx_ICR2	0x10
#define GPIOx_IMR	0x14
#define GPIOx_ISR	0x18
#define GPIOx_EDS	0x1c


#define EIMRAM_BASE 0x08000000

