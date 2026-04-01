KERNELRELEASE ?= `uname -r`
KERNEL_DIR	  ?= /lib/modules/$(KERNELRELEASE)/build

# SPDX-License-Identifier: GPL-2.0-only
snd-usb-zoom-objs := driver.o pcm.o
#snd-usb-zoom-objs := test.o
obj-m += snd-usb-zoom.o

.PHONY: build
build:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules
clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
