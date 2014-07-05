obj-m = lirc-gpio.o
KVERSION=$(shell uname -r)
INCLUDEDIR = /usr/src/linux-headers-$(KVERSION)/arch/arm/plat-omap/include

all:
	make -I $(INCLUDEDIR) -C /lib/modules/$(KVERSION)/build M=$(PWD) modules
clean:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) clean
