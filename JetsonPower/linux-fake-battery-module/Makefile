
obj-m += change_battery_volt.o
obj-m += fake_battery.o


KERN_VER=4.9.140-tegra

all:
	make -C /lib/modules/$(KERN_VER)/build M=$(shell pwd) modules

clean:
	rm -f *.cmd *.ko *.o Module.symvers modules.order *.mod.c
