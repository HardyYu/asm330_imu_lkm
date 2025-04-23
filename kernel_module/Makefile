obj-m += asm330_kmodule.o 
asm330_kmodule-objs := asm330lhh_reg.o asm330_module.o 
 
PWD := $(CURDIR)
 
all: module dt
	echo Builded Device Tree Overlay and Kernel Module


module:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
dt: asm330_dt.dts
	dtc -@ -I dts -O dtb -o asm330_dt.dtbo asm330_dt.dts
clean: 
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
