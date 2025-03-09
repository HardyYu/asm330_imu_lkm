obj-m += asm330_q_driver.o 
asm330_q_driver-objs := asm330lhh_reg.o asm330_quick_driver.o 
 
PWD := $(CURDIR)
 
all: module dt
	echo Builded Device Tree Overlay and Kernel Module


module:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
dt: asm330_dt.dts
	dtc -@ -I dts -O dtb -o asm330_dt.dtbo asm330_dt.dts
clean: 
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
