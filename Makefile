obj-m += ovis.o

all:
	make -C /home/sadeem/kernel/linux-6.17 M=$(PWD) modules
clean:
	make -C /home/sadeem/kernel/linux-6.17 M=$(PWD) clean
