obj-m += f75111.o

CURRENT = $(shell uname -r)
MDIR = drivers/input/
DEST = /lib/modules/$(CURRENT)/kernel/$(MDIR)


all:
		make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
		make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean 



install: 
	install f75111.ko $(DEST) 
	depmod -a

