##EXTRA_CFLAGS = " -O2 "
WFLAGS :=  -Wunused-variable
EXTRA_CFLAGS += -O2

MODULE_NAME := eim
RESMAIN_CORE_OBJS = eim_fpga.o
RESMAIN_GLUE_OBJS = memcpy_neon.o

$(MODULE_NAME)-objs = $(RESMAIN_GLUE_OBJS) $(RESMAIN_CORE_OBJS)

obj-m = eim.o 


##KDIR = /home/bobfsa/fsa_work/code/ti-sdk-omapl138-lcdk/board-support/linux-3.1.10
##KDIR = /home/bobfsa/fsa_work/code/ti-sdk-omapl138-lcdk/board-support/linux-2.6.33-rc4/
##KDIR = /usr/src/linux-headers-2.6.32-21-generic
##KDIR = /home/bobfsa/fsa_work/code/ti2.6.33sdk/DaVinci-PSP-SDK-03.20.00.13/src/kernel/linux-03.20.00.13/

##KDIR =/home/bobfsa/fsa_work/code/imx6_3035sdk/i_MX6QSABRELite/kernel-source/linux-3.0
##KDIR=/home/bobfsa/fsa_work/code/imx6_3035sdk/ltib/linux-3.0.35
KBUILD_EXTRA_SYMBOLS = /home/bobfsa/fsa_work/code/imx6_3035sdk/imx3015/linux-2.6-imx/linux-2.6-imx/Module.symvers
KDIR=/home/bobfsa/fsa_work/code/imx6_3035sdk/imx3015/linux-2.6-imx/linux-2.6-imx

all:

	##EXTRA_CFLAGS = " -O2"
	##$(AS) *.s
	$(MAKE) -C $(KDIR)  M=$(PWD)

.PHONY: clean
clean:
	rm -f *.mod.c *.mod.o .*.o.cmd *.ko *.o *.tmp_versions
