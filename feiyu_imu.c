// MPU6050 driver for the Feiyu

#include "feiyu_mane.h"
#include "feiyu_imu.h"
#include "feiyu_hall.h"
#include "arm_linux.h"
#include "feiyu_uart.h"
#include "stm32f1xx_hal_i2c.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"



#ifdef BOARD2


#define I2C_ADDRESS (0x68 << 1)

I2C_HandleTypeDef I2cHandle;
imu_t imu;




void i2c_read_device9()
{
	if(__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_RXNE) != RESET)
	{
		imu.i2c_buffer[imu.current_byte] = I2cHandle.Instance->DR;


//TRACE
//print_hex2(&uart, imu.i2c_buffer[imu.current_byte]);
		imu.current_byte++;
		
		if(imu.current_byte < imu.bytes)
		{
			/* Enable Acknowledge */
			SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_ACK);
		}
		else
		{
    		/* Disable Acknowledge */
    		CLEAR_BIT(I2cHandle.Instance->CR1, I2C_CR1_ACK);
			/* Generate Stop */
    		SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_STOP);
			imu.i2c_function = 0;
		}
	}
}

void i2c_read_device8()
{
	if(imu.bytes == 1)
	{
    	/* Disable Acknowledge */
    	CLEAR_BIT(I2cHandle.Instance->CR1, I2C_CR1_ACK);

    	/* Clear ADDR flag */
    	__HAL_I2C_CLEAR_ADDRFLAG(&I2cHandle);

    	/* Generate Stop */
    	SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_STOP);

	}
	else
	{
    	/* Enable Acknowledge */
    	SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_ACK);

    	/* Clear ADDR flag */
    	__HAL_I2C_CLEAR_ADDRFLAG(&I2cHandle);
	}


	imu.i2c_function = i2c_read_device9;
}

// returns 1 when the address flag is set
int wait_address_flag()
{
// wait for ADDR flag
	if(__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_ADDR) != RESET)
	{
//TRACE
		  /* Clear ADDR flag */
	  	__HAL_I2C_CLEAR_ADDRFLAG(&I2cHandle);

		return 1;
	}
	else
	{
		if(__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_AF) == SET)
		{
//TRACE
    		/* Generate Stop */
    		SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_STOP);

    		/* Clear AF Flag */
    		__HAL_I2C_CLEAR_FLAG(&I2cHandle, I2C_FLAG_AF);
		}
	}
	
	return 0;
}


void i2c_read_device7()
{
// wait for ADDR flag
	if(wait_address_flag())
	{
		imu.i2c_function = i2c_read_device8;
	}
}

void i2c_read_device6()
{
	if(__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_SB) != RESET)
	{
  		/* Send slave address */
  		I2cHandle.Instance->DR = I2C_7BIT_ADD_READ(I2C_ADDRESS);
		imu.i2c_function = i2c_read_device7;
	}
}

void i2c_read_device5()
{
	/* Wait until TXE flag is set */
	if(__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_TXE) != RESET)
	{
  		/* Generate Restart */
  		SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_START);
		imu.i2c_function = i2c_read_device6;
	}
}



void i2c_read_device4()
{
	/* Wait until TXE flag is set */
	if(__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_TXE) != RESET)
	{
//TRACE
    	/* Send Memory Address */
    	I2cHandle.Instance->DR = I2C_MEM_ADD_LSB(imu.reg);
		imu.i2c_function = i2c_read_device5;
	}
}

void i2c_read_device3()
{
/*
 * if(UART_EMPTY(&uart))
 * {
 * TRACE
 * print_hex8(&uart, I2C2->CR1);
 * print_hex8(&uart, I2C2->SR1);
 * print_hex8(&uart, I2C2->SR2);
 * }
 */
// wait for ADDR flag
	if(wait_address_flag())
	{
//TRACE
		imu.i2c_function = i2c_read_device4;
	}
}


void i2c_read_device2()
{
// wait for SB flag

	if(__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_SB) == SET)
	{
//TRACE
	  /* Send slave address */
 	 	I2cHandle.Instance->DR = I2C_7BIT_ADD_WRITE(I2C_ADDRESS);
		imu.i2c_function = i2c_read_device3;
	}


}

void i2c_read_device1()
{
	if(__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_BUSY) == RESET)
	{
//TRACE
  		/* Disable Pos */
 	   	CLEAR_BIT(I2cHandle.Instance->CR1, I2C_CR1_POS);

		/* Enable Acknowledge */
		SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_ACK);

		/* Generate Start */
		SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_START);

		imu.i2c_function = i2c_read_device2;
	}
}

void i2c_read_device(unsigned char reg, int bytes)
{
	int i;
	for(i = 0; i < bytes; i++)
	{
		imu.i2c_buffer[i] = 0xff;
	}

	imu.reg = reg;
	imu.bytes = bytes;
	imu.current_byte = 0;
	imu.i2c_function = i2c_read_device1;
}

void i2c_write_device6()
{
	/* Wait until TXE flag is set */
    if(__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_TXE) != RESET)
	{
		/* Generate Stop */
    	SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_STOP);
		imu.i2c_function = 0;
	}
}

void i2c_write_device5()
{
	/* Wait until TXE flag is set */
	if(__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_TXE) != RESET)
	{
		/* Write data to DR */
      	I2cHandle.Instance->DR = imu.i2c_buffer[imu.current_byte];
		imu.current_byte++;
  		imu.i2c_function = i2c_write_device6;
	}
}

void i2c_write_device4()
{
	/* Wait until TXE flag is set */
	if(__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_TXE) != RESET)
	{
    	/* Send Memory Address */
    	I2cHandle.Instance->DR = I2C_MEM_ADD_LSB(imu.reg);
		imu.i2c_function = i2c_write_device5;
	}
}


void i2c_write_device3()
{
// wait for ADDR flag
	if(wait_address_flag())
	{
		imu.i2c_function = i2c_write_device4;
	}
}

void i2c_write_device2()
{
	// wait for SB flag
	if(__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_SB) != RESET)
	{
		/* Send slave address */
  		I2cHandle.Instance->DR = I2C_7BIT_ADD_WRITE(I2C_ADDRESS);
		imu.i2c_function = i2c_write_device3;
	}
}


void i2c_write_device1()
{
	if(__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_BUSY) == RESET)
	{
	    /* Disable Pos */
    	CLEAR_BIT(I2cHandle.Instance->CR1, I2C_CR1_POS);
 	 	/* Generate Start */
 		SET_BIT(I2cHandle.Instance->CR1, I2C_CR1_START);
		imu.i2c_function = i2c_write_device2;
	}
}


void i2c_write_device(unsigned char reg, unsigned char value)
{
	imu.reg = reg;
	imu.bytes = 1;
	imu.current_byte = 0;
	imu.i2c_buffer[0] = value;
	imu.i2c_function = i2c_write_device1;
}




void imu_idle()
{
}

void test_status();
void send_result()
{
	int i;
	if(UART_EMPTY(&uart))
	{

// send data to BOARD1
		if(UART_EMPTY(&uart))
		{
			int size = 20;
			unsigned char buffer[size];
			buffer[0] = 0xff;
			buffer[1] = SYNC_CODE;
			memcpy(buffer + 2, imu.i2c_buffer, 14);
			buffer[16] = hall.value & 0xff;
			buffer[17] = (hall.value >> 8) & 0xff;
			buffer[18] = 0;
			buffer[19] = 0;
			send_uart(&uart, buffer, size);
		}
		
//		TRACE
/*
 * 		for(i = 0; i < 14; i++)
 * 		{
 * 			print_hex2(&uart, imu.i2c_buffer[i]);
 * 		}
 */
/*
 *  		int offset = 0;
 *  		print_number(&uart, READ_INT16BE(imu.i2c_buffer, offset));
 *  		print_number(&uart, READ_INT16BE(imu.i2c_buffer, offset));
 *  		print_number(&uart, READ_INT16BE(imu.i2c_buffer, offset));
 * 		offset += 2;
 *  		print_number(&uart, READ_INT16BE(imu.i2c_buffer, offset));
 *  		print_number(&uart, READ_INT16BE(imu.i2c_buffer, offset));
 *  		print_number(&uart, READ_INT16BE(imu.i2c_buffer, offset));
 */
	}

	imu.count++;
	if(mane_time - imu.time2 > HZ)
	{
//		TRACE
//		print_number(&uart, imu.count);
		imu.count = 0;
		imu.time2 = mane_time;
	}

	test_status();
}

void test_status3()
{
	if(mane_time - imu.time >= 0)
	{
		i2c_read_device(0x3b, 14);
		imu.imu_function = send_result;
	}
}

void test_status2()
{
//TRACE
//print_hex2(&uart, imu.i2c_buffer[0]);
	if((imu.i2c_buffer[0] & 0x1))
	{
		imu.time = mane_time;
		imu.imu_function = test_status3;
	}
	else
	{
		test_status();
	}
}

void test_status1()
{
	if(mane_time - imu.time >= 0)
	{
		i2c_read_device(0x3a, 1);
		imu.imu_function = test_status2;
	}
}

void test_status()
{
	imu.time = mane_time;
	imu.imu_function = test_status1;
}

void imu_init3()
{
// sample rate divider
	i2c_write_device(0x19, 19);
	imu.imu_function = test_status;
}

void imu_init2()
{
// accel config accel scale
	i2c_write_device(0x1c, 0x00);
	imu.imu_function = imu_init3;
}

void imu_init1()
{
// gyro config gyro scale
	i2c_write_device(0x1b, 0x18);
	imu.imu_function = imu_init2;
}

void imu_init0()
{
	if(mane_time - imu.time > HZ / 10)
	{
// sleep mode & clock
		i2c_write_device(0x6b, 1);
		imu.imu_function = imu_init1;
	}
}



// callback from HAL_I2C_Init
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

/*##-1- Enable peripherals and GPIO Clocks #################################*/
/* Enable GPIO TX/RX clock */
	__HAL_RCC_GPIOB_CLK_ENABLE();
/* Enable I2Cx clock */
	__HAL_RCC_I2C2_CLK_ENABLE();

/*##-2- Configure peripheral GPIO ##########################################*/  
/* I2C CLK GPIO pin configuration  */
	GPIO_InitStruct.Pin       = GPIO_PIN_10;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* I2C DATA GPIO pin configuration  */
	GPIO_InitStruct.Pin       = GPIO_PIN_11;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/*##-3- Configure the NVIC for I2C ########################################*/   
/* NVIC for I2Cx */
// 	HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 1);
// 	HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
// 	HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 2);
// 	HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
}

void init_imu()
{
	I2cHandle.Instance             = I2C2;
	I2cHandle.Init.ClockSpeed      = 100000;
	I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
	I2cHandle.Init.OwnAddress1     = I2C_ADDRESS;
	I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2cHandle.Init.OwnAddress2     = 0xFF;
	I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;  

	HAL_I2C_Init(&I2cHandle);

// read 0x68
//	i2c_read_device(0x75, 1);
	imu.time = mane_time;
	imu.time2 = mane_time;
	imu.imu_function = imu_init0;
//	imu.imu_function = imu_idle;
}





#else // BOARD2

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
}

#endif // !BOARD2




