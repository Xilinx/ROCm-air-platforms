# https://www.kernel.org/doc/html/latest/kbuild/modules.html

obj-m := amdair.o
amdair-y := amdair_chardev.o amdair_device.o amdair_doorbell.o amdair_drv.o \
	    amdair_mem_manager.o amdair_process.o amdair_queue.o vck5000.o

ifeq ($(KERNELRELEASE),)
KDIR ?= /usr/src/linux-headers-`uname -r`

default:
	make -C $(KDIR) M=$$PWD
endif # KERNELRELEASE

clean:
	rm -f *.o *.ko *.a *.order *.mod.c *.symvers *.mod

.PHONY: tags

tags:
	ctags -R -f tags .
