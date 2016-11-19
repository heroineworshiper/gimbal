GXX := g++
GCC_ARM := /opt/arm/bin/arm-elf-gcc
OBJCOPY := /opt/arm/bin/arm-elf-objcopy
ARM_CFLAGS := -O2 -c -I. -Irtlwifi -Iarm -Istm32f4 -fno-builtin -mthumb -mcpu=cortex-m4 -mlittle-endian -ffreestanding 
ARM_LFLAGS := -fno-builtin -mthumb -mcpu=cortex-m4 -mlittle-endian -ffreestanding -nostdlib -nostdinc -L/opt/arm/lib/gcc/arm-elf/4.1.1/

AVR_DIR := /amazon/root/arduino-1.6.7/hardware/tools/avr/bin/
AVR_GCC := $(AVR_DIR)avr-gcc
AVR_OBJCOPY := $(AVR_DIR)avr-objcopy -j .text -j .data -O ihex
AVR_DUDE := avrdude -v -patmega328p -cstk500v1 -P/dev/ttyACM0 -b19200
AVR_CFLAGS := -O2 -mmcu=atmega328p
AVR_LFLAGS := -O2 -mmcu=atmega328p -Wl,--section-start=.text=0x0000 -nostdlib

ARM_OBJS := \
	stm32f4/startup_main.o \
	stm32f4/usb_hcd_int.o \
	hardi2c.o \
	gimbal.o \
	idg3200.o \
	imu.o \
	uart.o \
	stm32f4/misc.o \
	stm32f4/stm32f4xx_rcc.o \
	stm32f4/stm32f4xx_usart.o \
	stm32f4/stm32f4xx_gpio.o \
	stm32f4/stm32f4xx_dcmi.o \
	stm32f4/stm32f4xx_dma.o \
	stm32f4/stm32f4xx_i2c.o \
	stm32f4/stm32f4xx_it.o \
	stm32f4/stm32f4xx_iwdg.o \
	stm32f4/stm32f4xx_tim.o \
	stm32f4/stm32f4xx_adc.o \
	stm32f4/stm32f4xx_flash.o \
	stm32f4/stm32f4xx_spi.o \
	stm32f4/usb_bsp.o \
	stm32f4/usb_core.o \
	stm32f4/usb_dcd.o \
	stm32f4/usb_hcd.o \
	stm32f4/usbd_core.o \
	stm32f4/usbd_req.o \
	stm32f4/usbd_ioreq.o \
	stm32f4/usb_dcd_int.o \
	stm32f4/usbh_core.o \
	stm32f4/usbh_hcs.o \
	stm32f4/usbh_ioreq.o \
	stm32f4/usbh_stdreq.o \
	mpu9150.o \
	math.o \
	linux.o


ARM_BOOTLOADER_OBJS := \
	bootloader.o \
	stm32f4/startup_boot.o \
	stm32f4/system_stm32f4xx.o \
	uart.o \
	stm32f4/misc.o \
	stm32f4/stm32f4xx_flash.o \
	stm32f4/stm32f4xx_gpio.o \
	stm32f4/stm32f4xx_rcc.o \
	stm32f4/stm32f4xx_tim.o \
	stm32f4/stm32f4xx_usart.o


GCC_FEIYU := /opt/gcc-arm-none-eabi-4_9-2015q1/bin/arm-none-eabi-gcc
OBJCOPY_FEIYU := /opt/gcc-arm-none-eabi-4_9-2015q1/bin/arm-none-eabi-objcopy
FEIYU_CFLAGS := -O2 -fno-builtin -mthumb -mcpu=cortex-m3 -mlittle-endian -ffreestanding -DSTM32F103xB -Istm32f103
FEIYU_LFLAGS := -fno-builtin -mthumb -mcpu=cortex-m3 -mlittle-endian -ffreestanding

FEIYU_OBJS := \
	startup_stm32f103xb.o \
	system_stm32f1xx.o \
	stm32f1xx_hal_cortex.o \
	stm32f1xx_hal_flash.o \
	stm32f1xx_hal_flash_ex.o \
	stm32f1xx_hal_gpio.o \
	stm32f1xx_hal_rcc.o \
	stm32f1xx_hal_uart.o \
	stm32f1xx_hal.o \
	feiyu_bootloader.o \
	arm_linux.o \
	feiyu_uart.o

FEIYU_MANE_OBJS := \
	arm_math.o \
	feiyu_adc.o \
	feiyu_feedback.o \
	feiyu_imu.o \
	feiyu_hall.o \
	feiyu_mane.o \
	feiyu_motor.o \
	stm32f1xx_hal_adc.o \
	stm32f1xx_hal_i2c.o \
	stm32f1xx_hal_rcc_ex.o \
	stm32f1xx_hal_spi.o \
	stm32f1xx_hal_tim.o \
	stm32f1xx_hal_tim_ex.o

FEIYU_COMMON_OBJS := \
	startup_stm32f103xb.o \
	system_stm32f1xx.o \
	stm32f1xx_hal_cortex.o \
	stm32f1xx_hal_gpio.o \
	stm32f1xx_hal_rcc.o \
	stm32f1xx_hal_uart.o \
	stm32f1xx_hal.o \
	arm_linux.o \
	feiyu_uart.o
	
FEIYU_SRCS := \
	arm_linux.c \
	arm_linux.h \
	arm_math.c \
	arm_math.h \
	avr_debug.c \
	avr_debug.h \
	feiyu2.c \
	feiyu_adc.c \
	feiyu_adc.h \
	feiyu_bootloader.c \
	feiyu_feedback.c \
	feiyu_feedback.h \
	feiyu.h \
	feiyu_hall.c \
	feiyu_hall.h \
	feiyu_imu.c \
	feiyu_imu.h \
	feiyu_mane.c \
	feiyu_mane.h \
	feiyu_mane.ld \
	feiyu_motor.c \
	feiyu_motor.h \
	feiyu_pid.h \
	feiyu_program.c \
	feiyu_uart.c \
	feiyu_uart.h \
	Makefile \
	startup_stm32f103xb.s \
	stm32f103 \
	STM32F103XB_FLASH.ld \
	stm32f103xb.h \
	stm32f1xx.h \
	stm32f1xx_hal_adc.c \
	stm32f1xx_hal_adc_ex.h \
	stm32f1xx_hal_adc.h \
	stm32f1xx_hal.c \
	stm32f1xx_hal_can_ex.h \
	stm32f1xx_hal_can.h \
	stm32f1xx_hal_cec.h \
	stm32f1xx_hal_conf.h \
	stm32f1xx_hal_conf_template.h \
	stm32f1xx_hal_cortex.c \
	stm32f1xx_hal_cortex.h \
	stm32f1xx_hal_crc.h \
	stm32f1xx_hal_dac_ex.h \
	stm32f1xx_hal_dac.h \
	stm32f1xx_hal_def.h \
	stm32f1xx_hal_dma_ex.h \
	stm32f1xx_hal_dma.h \
	stm32f1xx_hal_eth.h \
	stm32f1xx_hal_flash.c \
	stm32f1xx_hal_flash_ex.c \
	stm32f1xx_hal_flash_ex.h \
	stm32f1xx_hal_flash.h \
	stm32f1xx_hal_gpio.c \
	stm32f1xx_hal_gpio_ex.h \
	stm32f1xx_hal_gpio.h \
	stm32f1xx_hal.h \
	stm32f1xx_hal_hcd.h \
	stm32f1xx_hal_i2c.c \
	stm32f1xx_hal_i2c.h \
	stm32f1xx_hal_i2s.h \
	stm32f1xx_hal_irda.h \
	stm32f1xx_hal_iwdg.h \
	stm32f1xx_hal_nand.h \
	stm32f1xx_hal_nor.h \
	stm32f1xx_hal_pccard.h \
	stm32f1xx_hal_pcd_ex.h \
	stm32f1xx_hal_pcd.h \
	stm32f1xx_hal_pwr.h \
	stm32f1xx_hal_rcc.c \
	stm32f1xx_hal_rcc_ex.c \
	stm32f1xx_hal_rcc_ex.h \
	stm32f1xx_hal_rcc.h \
	stm32f1xx_hal_rtc_ex.h \
	stm32f1xx_hal_rtc.h \
	stm32f1xx_hal_sd.h \
	stm32f1xx_hal_smartcard.h \
	stm32f1xx_hal_spi.c \
	stm32f1xx_hal_spi.h \
	stm32f1xx_hal_sram.h \
	stm32f1xx_hal_tim.c \
	stm32f1xx_hal_tim_ex.c \
	stm32f1xx_hal_tim_ex.h \
	stm32f1xx_hal_tim.h \
	stm32f1xx_hal_uart.c \
	stm32f1xx_hal_uart.h \
	stm32f1xx_hal_usart.h \
	stm32f1xx_hal_wwdg.h \
	stm32f1xx_ll_fsmc.h \
	stm32f1xx_ll_sdmmc.h \
	stm32f1xx_ll_usb.h \
	system_stm32f1xx.c \
	system_stm32f1xx.h \
	tables.c




JPEG_OBJS := \
	jpeg/cdjpeg.o \
	jpeg/jcapimin.o \
	jpeg/jcapistd.o \
	jpeg/jccoefct.o \
	jpeg/jccolor.o \
	jpeg/jcdctmgr.o \
	jpeg/jchuff.o \
	jpeg/jcinit.o \
	jpeg/jcmainct.o \
	jpeg/jcmarker.o \
	jpeg/jcmaster.o \
	jpeg/jcomapi.o \
	jpeg/jcparam.o \
	jpeg/jcphuff.o \
	jpeg/jcprepct.o \
	jpeg/jcsample.o \
	jpeg/jctrans.o \
	jpeg/jdapimin.o \
	jpeg/jdapistd.o \
	jpeg/jdatadst.o \
	jpeg/jdatasrc.o \
	jpeg/jdcoefct.o \
	jpeg/jdcolor.o \
	jpeg/jddctmgr.o \
	jpeg/jdhuff.o \
	jpeg/jdinput.o \
	jpeg/jdmainct.o \
	jpeg/jdmarker.o \
	jpeg/jdmaster.o \
	jpeg/jdmerge.o \
	jpeg/jdphuff.o \
	jpeg/jdpostct.o \
	jpeg/jdsample.o \
	jpeg/jdtrans.o \
	jpeg/jerror.o \
	jpeg/jfdctflt.o \
	jpeg/jfdctfst.o \
	jpeg/jfdctint.o \
	jpeg/jidctflt.o \
	jpeg/jidctfst.o \
	jpeg/jidctint.o \
	jpeg/jidctred.o \
	jpeg/jmemmgr.o \
	jpeg/jmemnobs.o \
	jpeg/jquant1.o \
	jpeg/jquant2.o \
	jpeg/jutils.o \
	jpeg/rdbmp.o \
	jpeg/rdcolmap.o \
	jpeg/rdgif.o \
	jpeg/rdppm.o \
	jpeg/rdrle.o \
	jpeg/rdswitch.o \
	jpeg/rdtarga.o \
	jpeg/transupp.o \
	jpeg/wrbmp.o \
	jpeg/wrgif.o \
	jpeg/wrppm.o \
	jpeg/wrrle.o \
	jpeg/wrtarga.o



CAM_OBJS := \
	cam.o \
	motion.o \
	$(JPEG_OBJS)

#all: feiyu_bootloader.bin tables gimbal.bin bootloader.bin imu.hex
all: feiyu_bootloader.bin feiyu_mane.bin feiyu_program feiyu.hex

feiyu_program: feiyu_program.c
	gcc -o feiyu_program feiyu_program.c

feiyu_mane.bin: feiyu_bootloader.bin $(FEIYU_MANE_OBJS)
	$(GCC_FEIYU) -o feiyu_mane.elf \
		$(FEIYU_COMMON_OBJS) \
		$(FEIYU_MANE_OBJS) \
		$(FEIYU_LFLAGS) \
		-Tfeiyu_mane.ld
	$(OBJCOPY_FEIYU) -O binary feiyu_mane.elf feiyu_mane.bin

feiyu_bootloader.bin: $(FEIYU_OBJS)
	$(GCC_FEIYU) -o feiyu_bootloader.elf \
		$(FEIYU_OBJS) \
		$(FEIYU_LFLAGS) \
		-TSTM32F103XB_FLASH.ld
	$(OBJCOPY_FEIYU) -O binary feiyu_bootloader.elf feiyu_bootloader.bin


# compile feiyu.hex for the ATMega328
feiyu.hex: feiyu2.c avr_debug.c
	$(AVR_GCC) $(AVR_CFLAGS) -o feiyu.o feiyu2.c avr_debug.c
	$(AVR_GCC) $(AVR_LFLAGS) -o feiyu.elf feiyu.o
	$(AVR_OBJCOPY) feiyu.elf feiyu.hex

# program standalone atmega328 fuses
# might need to program 1 fuse at a time
feiyu_fuse:
#	$(AVR_DUDE) -Ulock:w:0x3F:m -Uefuse:w:0x05:m -Uhfuse:w:0xDA:m -Ulfuse:w:0xE2:m 
	$(AVR_DUDE) -Ulock:w:0x3F:m -Ulfuse:w:0xE2:m 
	$(AVR_DUDE) -Ulock:w:0x3F:m -Uhfuse:w:0xDA:m
	$(AVR_DUDE) -Ulock:w:0x3F:m -Uefuse:w:0x05:m

# program feiyu.hex
feiyu_isp: feiyu.hex
	$(AVR_DUDE) -Uflash:w:feiyu.hex:i -Ulock:w:0x0F:m

feiyu_archive:
	mkdir /tmp/feiyu
	cp -a $(FEIYU_SRCS) /tmp/feiyu
	cd /tmp && tar Jcf feiyu.tar.xz feiyu


imu.hex: imu.s hardi2c.s hardi2c.inc
	gpasm -o imu.hex imu.s

tables: tables.c
	gcc -O -o tables tables.c -lm

gimbal.bin: $(ARM_OBJS)
	$(GCC_ARM) -o gimbal.elf \
		$(ARM_OBJS) \
		$(ARM_LFLAGS) \
		-Tstm32f4/main.ld
	$(OBJCOPY) -O binary gimbal.elf gimbal.bin

bootloader.bin: $(ARM_BOOTLOADER_OBJS)
	$(GCC_ARM) -o bootloader.elf $(ARM_BOOTLOADER_OBJS) $(ARM_BOOTLOADER_OBJS2) \
		-Tstm32f4/bootloader.ld $(ARM_LFLAGS)
	$(OBJCOPY) -O binary bootloader.elf bootloader.bin


bootloader.o stm32f4/startup_boot.o stm32f4/system_stm32f4xx.o: 
	$(GCC_ARM) $(ARM_CFLAGS) -c $< -o $*.o

cam_lidar: cam_lidar.c
	g++ -O2 cam_lidar.c -o cam_lidar $(JPEG_OBJS) -lm -lX11 -lXext -lpthread -Ijpeg

cam: $(CAM_OBJS)
	$(GXX) -DX86 -O2 -g -o cam $(CAM_OBJS) -lpthread -lm

$(CAM_OBJS):
	$(GXX) -O2 -g -c $*.c -o $*.o -DX86 -Ijpeg


$(ARM_OBJS):
	$(GCC_ARM) $(ARM_CFLAGS) -c $< -o $*.o

$(FEIYU_OBJS) $(FEIYU_MANE_OBJS):
	$(GCC_FEIYU) $(FEIYU_CFLAGS) -c $< -o $*.o

clean:
	rm -f cam_lidar *.o *.elf *.bin *.hex stm32f4/*.o 


# Feiyu objs
startup_stm32f103xb.o: startup_stm32f103xb.s
system_stm32f1xx.o: system_stm32f1xx.c
feiyu_bootloader.o: feiyu_bootloader.c
feiyu_adc.o: feiyu_adc.c
feiyu_feedback.o: feiyu_feedback.c
feiyu_imu.o: feiyu_imu.c
feiyu_hall.o: feiyu_hall.c
feiyu_mane.o: feiyu_mane.c
feiyu_motor.o: feiyu_motor.c
arm_linux.o: arm_linux.c
arm_math.o: arm_math.c
feiyu_uart.o: feiyu_uart.c
stm32f1xx_hal_adc.o: stm32f1xx_hal_adc.c
stm32f1xx_hal_i2c.o: stm32f1xx_hal_i2c.c
stm32f1xx_hal_uart.o: stm32f1xx_hal_uart.c
stm32f1xx_hal_gpio.o: stm32f1xx_hal_gpio.c
stm32f1xx_hal_rcc.o: stm32f1xx_hal_rcc.c
stm32f1xx_hal.o: stm32f1xx_hal.c
stm32f1xx_hal_dma.o: stm32f1xx_hal_dma.c
stm32f1xx_hal_cortex.o: stm32f1xx_hal_cortex.c
stm32f1xx_hal_rcc_ex.o: stm32f1xx_hal_rcc_ex.c
stm32f1xx_hal_spi.o: stm32f1xx_hal_spi.c
stm32f1xx_hal_tim.o: stm32f1xx_hal_tim.c
stm32f1xx_hal_tim_ex.o: stm32f1xx_hal_tim_ex.c
stm32f1xx_hal_flash.o: stm32f1xx_hal_flash.c
stm32f1xx_hal_flash_ex.o: stm32f1xx_hal_flash_ex.c




stm32f4/startup_main.o:       stm32f4/startup_main.s
stm32f4/usb_hcd_int.o:	      stm32f4/usb_hcd_int.c
hardi2c.o:		  hardi2c.c
gimbal.o:		  gimbal.c
idg3200.o:                idg3200.c
imu.o:                    imu.c
uart.o: 		  uart.c
stm32f4/misc.o: 	      stm32f4/misc.c
stm32f4/stm32f4xx_rcc.o:      stm32f4/stm32f4xx_rcc.c
stm32f4/stm32f4xx_usart.o:    stm32f4/stm32f4xx_usart.c
stm32f4/stm32f4xx_gpio.o:     stm32f4/stm32f4xx_gpio.c
stm32f4/stm32f4xx_dcmi.o:     stm32f4/stm32f4xx_dcmi.c
stm32f4/stm32f4xx_dma.o:      stm32f4/stm32f4xx_dma.c
stm32f4/stm32f4xx_i2c.o:      stm32f4/stm32f4xx_i2c.c
stm32f4/stm32f4xx_it.o:       stm32f4/stm32f4xx_it.c
stm32f4/stm32f4xx_iwdg.o:     stm32f4/stm32f4xx_iwdg.c
stm32f4/stm32f4xx_tim.o:      stm32f4/stm32f4xx_tim.c
stm32f4/stm32f4xx_adc.o:      stm32f4/stm32f4xx_adc.c
stm32f4/stm32f4xx_flash.o:    stm32f4/stm32f4xx_flash.c
stm32f4/stm32f4xx_spi.o:      stm32f4/stm32f4xx_spi.c
stm32f4/usb_bsp.o:	      stm32f4/usb_bsp.c
stm32f4/usb_core.o:	      stm32f4/usb_core.c
stm32f4/usb_dcd.o:	      stm32f4/usb_dcd.c
stm32f4/usb_hcd.o:	      stm32f4/usb_hcd.c
stm32f4/usbd_core.o:	      stm32f4/usbd_core.c
stm32f4/usbd_req.o:	      stm32f4/usbd_req.c
stm32f4/usbd_ioreq.o:	      stm32f4/usbd_ioreq.c
stm32f4/usb_dcd_int.o:	      stm32f4/usb_dcd_int.c
stm32f4/usbh_core.o:	      stm32f4/usbh_core.c
stm32f4/usbh_hcs.o:	      stm32f4/usbh_hcs.c
stm32f4/usbh_ioreq.o:	      stm32f4/usbh_ioreq.c
stm32f4/usbh_stdreq.o:	      stm32f4/usbh_stdreq.c
mpu9150.o:		  mpu9150.c
math.o: 		  math.c
linux.o:		  linux.c


bootloader.o:		  bootloader.c
stm32f4/startup_boot.o: stm32f4/startup_boot.s
stm32f4/system_stm32f4xx.o: stm32f4/system_stm32f4xx.c
cam.o: cam.c
motion.o: motion.c



