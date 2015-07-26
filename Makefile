#
# Makefile for the USB serial device drivers.
#

# Object file lists.

obj-m			+= option.o
obj-m			+= huawei_voice.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	
install:
	cp huawei_voice.ko /lib/modules/3.18.6+/kernel/drivers/usb/serial
	cp option.ko /lib/modules/3.18.6+/kernel/drivers/usb/serial
