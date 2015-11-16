; 18f14k50
; ../copter/usb_programmer -p 18f14k50 -e
; internal + PLL
; ../copter/usb_programmer -p 18f14k50 -c 0x300000 0011100000000000
; watchdog & brownout
; ../copter/usb_programmer -p 18f14k50 -c 0x300002 0001111100011110
; disable PGM & extended instructions
; ../copter/usb_programmer -p 18f14k50 -c 0x300006 0000000000000000
; rm imu.hex.orig;../copter/usb_programmer -p 18f14k50 imu.hex
; ../copter/usb_programmer -p 18f14k50 -r imu.hex



; IDG3200 + KXTF9 driver on a standalone board
; requires power cycling for the I2C peripherals to work


PROCESSOR 18f14k50
#include "p18f14k50.inc"
#include "util.inc"
#include "hardi2c.inc"


#define SYNC_CODE 0xe5


#define PIC_USE_MPU9150



#define CLOCKSPEED 32000000
; must be faster than I2C
#define BAUD 500000
#define BAUD_RATE_CODE (CLOCKSPEED / (BAUD * 4))
#define PACKET_SIZE 16
#define STARTUP_DELAY 32

; FLAG bits
cblock 0
	SERIAL_BUF_FULL : 1
endc



	VARSTART H'00', H'100'
	VARADD FLAGS, 1
	VARADD IMU_STATE, 2
	VARADD RESULT, 1
	VARADD RAW_DATA, 32
	VARADD SERIAL_BUF, PACKET_SIZE
	VARADD SERIAL_BUF2, PACKET_SIZE
	VARADD SERIAL_COUNTER, 1
	VARADD TEMP0, 4
	I2C_VARS
	




	ORG RESETVECTOR
	goto start

	ORG INTVECTORHI
	goto start

	ORG INTVECTORLO
	goto start


	ORG 0x200
start:
	BANKSEL H'00'
	
	SET_REGISTER OSCCON, B'01100000'
	SET_REGISTER OSCTUNE, B'01000000'

	clrf FLAGS
	clrf ANSEL
	clrf ANSELH

	SET_REGISTER TXSTA, B'00100100'
	SET_REGISTER BAUDCON, B'00001000'
	SET_REGISTER16 SPBRG, BAUD_RATE_CODE
	SET_REGISTER RCSTA, B'10000000'



; delay for peripherals
	CLEAR_REGISTER32 TEMP0
	SET_REGISTER TEMP0 + 2, -STARTUP_DELAY
start_delay:
	clrwdt
	incfsz TEMP0, F
	bra start_delay
		incfsz TEMP0 + 1, F
		bra start_delay
			incfsz TEMP0 + 2, F
			bra start_delay


	SET_REGISTER16 IMU_STATE, init1
	SET_REGISTER SERIAL_COUNTER, PACKET_SIZE




	call init_i2c
; I2C baud rate divider
	SET_REGISTER SSPADD, 20

	bra loop


#include "hardi2c.s"




loop:
	clrwdt

	call i2c_loop
	call handle_imu

ifndef PIC_USE_MPU9150

	btfss PIR1, TXIF
	bra loop

		SKIP_LESS_LITERAL SERIAL_COUNTER, PACKET_SIZE
		bra get_next_packet

			SET_POINTER0_LITERAL SERIAL_BUF2
			ADD POINTER0, SERIAL_COUNTER
			COPY_REGISTER TXREG, INDF0
			incf SERIAL_COUNTER, f
			bra loop

get_next_packet:
	btfss FLAGS, SERIAL_BUF_FULL
	bra loop
	
		COPY_REGISTER SERIAL_BUF2 + 0,  SERIAL_BUF + 0
		COPY_REGISTER SERIAL_BUF2 + 1,  SERIAL_BUF + 1
		COPY_REGISTER SERIAL_BUF2 + 2,  SERIAL_BUF + 2
		COPY_REGISTER SERIAL_BUF2 + 3,  SERIAL_BUF + 3
		COPY_REGISTER SERIAL_BUF2 + 4,  SERIAL_BUF + 4
		COPY_REGISTER SERIAL_BUF2 + 5,  SERIAL_BUF + 5
		COPY_REGISTER SERIAL_BUF2 + 6,  SERIAL_BUF + 6
		COPY_REGISTER SERIAL_BUF2 + 7,  SERIAL_BUF + 7
		COPY_REGISTER SERIAL_BUF2 + 8,  SERIAL_BUF + 8
		COPY_REGISTER SERIAL_BUF2 + 9,  SERIAL_BUF + 9
		COPY_REGISTER SERIAL_BUF2 + 10, SERIAL_BUF + 10
		COPY_REGISTER SERIAL_BUF2 + 11, SERIAL_BUF + 11
		COPY_REGISTER SERIAL_BUF2 + 12, SERIAL_BUF + 12
		COPY_REGISTER SERIAL_BUF2 + 13, SERIAL_BUF + 13
		COPY_REGISTER SERIAL_BUF2 + 14, SERIAL_BUF + 14
		COPY_REGISTER SERIAL_BUF2 + 15, SERIAL_BUF + 15
		clrf SERIAL_COUNTER
		bcf FLAGS, SERIAL_BUF_FULL

endif ; !PIC_USE_MPU9150

		bra loop



























handle_imu:
	SET_PC_REG IMU_STATE



send_results:
	I2C_READY

;	SET_REGISTER16 IMU_STATE, gyro_status1
	SET_REGISTER16 IMU_STATE, gyro_status2
	
ifndef PIC_USE_MPU9150
; create packet if enough room
	btfsc FLAGS, SERIAL_BUF_FULL
endif


	return





ifdef PIC_USE_MPU9150
#define GYRO_ADDRESS  B'1101000' << 1


send_raw_i2c:

	SKIP_EQUAL_LITERAL16 I2C_STATE, i2c_idle
	bra send_raw_i2c2

; debug
	bcf TRISA, 5
	btg LATA, 5

		SET_REGISTER16 IMU_STATE, send_results
		return

send_raw_i2c2:
	btfss PIR1, TXIF
	return
		btfss I2C_HAVE_DATA, 0
		return
			clrf I2C_HAVE_DATA
			COPY_REGISTER TXREG, I2C_VALUE
			incf SERIAL_COUNTER, F
			SKIP_GREATEREQUAL_LITERAL SERIAL_COUNTER, 14
			return
				SET_REGISTER16 IMU_STATE, send_results
				return


send_header2:
	btfss PIR1, TXIF
	return
		SET_REGISTER TXREG, SYNC_CODE
		SET_REGISTER16 IMU_STATE, send_raw_i2c
		return


send_header1:
	btfss PIR1, TXIF
	return
		SET_REGISTER TXREG, 0xff
		SET_REGISTER16 IMU_STATE, send_header2
		return


gyro_status2:
	I2C_READY
;	btfss RESULT, 0
;	bra gyro_status1
; accel, temp, gyros
		I2C_READ_DEVICE GYRO_ADDRESS, 0x3b, RAW_DATA, 14
		clrf SERIAL_COUNTER
;		SET_REGISTER16 IMU_STATE, send_results
		SET_REGISTER16 IMU_STATE, send_header1
		return

;gyro_status1:
;	I2C_READY
;	I2C_READ_DEVICE GYRO_ADDRESS, 0x3a, RESULT, 1
;	SET_REGISTER16 IMU_STATE, gyro_status2
;	return



init6:
	I2C_READY
; sample rate divider
; high enough to keep i2c from dropping samples
;	I2C_WRITE_DEVICE GYRO_ADDRESS, 0x19, 0x5
	I2C_WRITE_DEVICE GYRO_ADDRESS, 0x19, 0x0
;	SET_REGISTER16 IMU_STATE, gyro_status1
	SET_REGISTER16 IMU_STATE, gyro_status2
	return


init5:
	I2C_READY
; accel config
	I2C_WRITE_DEVICE GYRO_ADDRESS, 0x1c, 0x00
	SET_REGISTER16 IMU_STATE, init6
	return



init4:
	I2C_READY
; gyro full scale range
; 250 deg/sec
	I2C_WRITE_DEVICE GYRO_ADDRESS, 0x1b, 0x00
	SET_REGISTER16 IMU_STATE, init5
	return


init3:
	I2C_READY
; I2C master disable
	I2C_WRITE_DEVICE GYRO_ADDRESS, 0x6a, 0x00
	SET_REGISTER16 IMU_STATE, init4
	return



init2:
	I2C_READY
; passthrough mode
	I2C_WRITE_DEVICE GYRO_ADDRESS, 0x37, 0x02
	SET_REGISTER16 IMU_STATE, init3
	return


init1:
	I2C_READY
; sleep mode & clock
	I2C_WRITE_DEVICE GYRO_ADDRESS, 0x6b, 0x01
	SET_REGISTER16 IMU_STATE, init2
	return





else ; PIC_USE_MPU9150

#define GYRO_ADDRESS  B'1101001' << 1
#define ACCEL_ADDRESS B'0001111' << 1


; send_results 
		SET_REGISTER SERIAL_BUF + 0, 0xff
		SET_REGISTER SERIAL_BUF + 1, SYNC_CODE
		COPY_REGISTER SERIAL_BUF + 2, RAW_DATA + 0
		COPY_REGISTER SERIAL_BUF + 3, RAW_DATA + 1
		COPY_REGISTER SERIAL_BUF + 4, RAW_DATA + 2
		COPY_REGISTER SERIAL_BUF + 5, RAW_DATA + 3
		COPY_REGISTER SERIAL_BUF + 6, RAW_DATA + 4
		COPY_REGISTER SERIAL_BUF + 7, RAW_DATA + 5
		COPY_REGISTER SERIAL_BUF + 8, RAW_DATA + 6
		COPY_REGISTER SERIAL_BUF + 9, RAW_DATA + 7
		COPY_REGISTER SERIAL_BUF + 10, RAW_DATA + 8
		COPY_REGISTER SERIAL_BUF + 11, RAW_DATA + 9
		COPY_REGISTER SERIAL_BUF + 12, RAW_DATA + 10
		COPY_REGISTER SERIAL_BUF + 13, RAW_DATA + 11
		COPY_REGISTER SERIAL_BUF + 14, RAW_DATA + 12
		COPY_REGISTER SERIAL_BUF + 15, RAW_DATA + 13
		bsf FLAGS, SERIAL_BUF_FULL
		return


; accelerometer only goes at 100Hz
accel_status2:

	I2C_READY


	btfss RESULT, 4
	bra send_results
		I2C_READ_DEVICE ACCEL_ADDRESS, 0x6, RAW_DATA + 8, 6
		SET_REGISTER16 IMU_STATE, send_results
		return


accel_status1:
	I2C_READY
	I2C_READ_DEVICE ACCEL_ADDRESS, 0x18, RESULT, 1
	SET_REGISTER16 IMU_STATE, accel_status2
	return


; gyro goes at 1000Hz
gyro_status2:

	I2C_READY


	btfss RESULT, 0
	bra gyro_status1
		I2C_READ_DEVICE GYRO_ADDRESS, 0x1b, RAW_DATA, 8
		SET_REGISTER16 IMU_STATE, accel_status1
		return



gyro_status1:

	I2C_READY
	I2C_READ_DEVICE GYRO_ADDRESS, 0x1a, RESULT, 1
	SET_REGISTER16 IMU_STATE, gyro_status2
	return





init5:
	I2C_READY
; accel CTRL_REG1
	I2C_WRITE_DEVICE ACCEL_ADDRESS, 0x1b, 0xe0
	SET_REGISTER16 IMU_STATE, gyro_status1
	return





init4:
	I2C_READY
; DATA_CTRL_REG 
	I2C_WRITE_DEVICE ACCEL_ADDRESS, 0x21, 0x03
	SET_REGISTER16 IMU_STATE, init5
	return




init3:
	I2C_READY

; accel CTRL_REG1
	I2C_WRITE_DEVICE ACCEL_ADDRESS, 0x1b, 0x00
	SET_REGISTER16 IMU_STATE, init4
	return





init2:

	I2C_READY
; 2000 deg/sec
; DLPF
	I2C_WRITE_DEVICE GYRO_ADDRESS, 0x16, 0x18
	SET_REGISTER16 IMU_STATE, init3
	return




init1:

	I2C_READY
; sample rate divider
; high enough to keep i2c from dropping samples
	I2C_WRITE_DEVICE GYRO_ADDRESS, 0x15, 0x7
	SET_REGISTER16 IMU_STATE, init2
	return


endif ; !PIC_USE_MPU9150


END











