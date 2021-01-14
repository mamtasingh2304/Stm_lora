

#include "ds18b20_mflib.h"

extern volatile uint32_t time_us;

/**
 * Inserts a delay (in us).
 *
 * @param 	t: Time in us.
 * @return 	None.
 */


//--------------------------------------------------
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
micros *= (SystemCoreClock / 1000000) / 9;
/* Wait till done */
while (micros--) ;
}
//--------------------------------------------------

/**
 * Initialises communication with the DS18B20 device.
 *
 * @param 	None.
 * @return 	1 if succeed, 0 if failed.
 */
////
uint8_t get_resolution(){
	ds18b20_write_byte(READ_SCRATCHPAD_CMD);
	uint8_t conf;
	ds18b20_read_byte();
	ds18b20_read_byte();
	ds18b20_read_byte();
	ds18b20_read_byte();
	conf = ds18b20_read_byte();
	/* Return 9 - 12 value according to number of bits */
	return ((conf & 0x60) >> 5) + 9;	
}
uint8_t ds18b20_init_seq(void)
{
	GPIO_WriteBit(TXRX_PORT, TXRX_PIN, Bit_RESET); //HAL_GPIO_WritePin(TXRX_PORT, TXRX_PIN, GPIO_PIN_RESET);
	DelayMicro(600);
	GPIO_WriteBit(TXRX_PORT, TXRX_PIN, Bit_SET); ;//HAL_GPIO_WritePin(TXRX_PORT, TXRX_PIN, GPIO_PIN_SET);
	DelayMicro(100);
	if (GPIO_ReadInputDataBit(TXRX_PORT, TXRX_PIN)== Bit_RESET){   //(HAL_GPIO_ReadPin(TXRX_PORT, TXRX_PIN) == GPIO_PIN_RESET) {
		DelayMicro(500);
		return 1;
	} else {
		DelayMicro(500);
		return 0;
	}
}
/**
 * Sends the read rom command.
 *
 * @param 	None.
 * @return 	The lasered rom code.
 */
uint64_t ds18b20_read_rom_cmd(void)
{
	uint8_t crc, family_code;
	uint64_t lasered_rom_code, serial_num;

	ds18b20_write_byte(READ_ROM_CMD_BYTE);

	family_code = ds18b20_read_byte();
	serial_num = ds18b20_read_byte();
	serial_num |= (ds18b20_read_byte() << 8);
	serial_num |= (ds18b20_read_byte() << 16);
	crc = ds18b20_read_byte();

	lasered_rom_code = (crc << 24) | (serial_num << 8) |  family_code;

	return lasered_rom_code;

}

/**
 * Sends the function command.
 *
 * @param 	cmd: Number of the function command
 * @return  None.
 */
void ds18b20_send_function_cmd(uint8_t cmd)
{
	ds18b20_write_byte(cmd);

	if (cmd == CONVERT_T_CMD) {
		while(GPIO_ReadInputDataBit(TXRX_PORT, TXRX_PIN)== Bit_RESET) {
		/* wait for end of conversion */
		}
	}
		
		
}

/**
 * Sends the rom command.
 *
 * @param 	cmd: Number of the rom command
 * @return  None.
 */
void ds18b20_send_rom_cmd(uint8_t cmd)
{
	ds18b20_write_byte(cmd);
}

/**
 * Read the temperature.
 *
 * @param 	None.
 * @return  Float value of the last measured temperature.
 */
float ds18b20_read_temp(void)
{
	int8_t k;
	uint8_t temp_LSB, temp_MSB;
	uint16_t u16_temp, mask = 1;
	float temperature = 0.0;

	temp_LSB = ds18b20_read_byte();
	temp_MSB = ds18b20_read_byte();

	u16_temp = ((temp_MSB << 8) | temp_LSB);

	for (k = -4; k < 7; k++) {
		if (u16_temp & mask) {
			temperature += powf(2,k);
		}
		mask = mask << 1;
	}
	return temperature;
}

/**
 * Write byte to DS18B20.
 *
 * @param 	data: Data to be written.
 * @return  None.
 */
void ds18b20_write_byte(uint8_t data)
{
	uint8_t i;
  uint8_t 	mask = 0x01;
	uint8_t data_bit = data & mask;
	for (i = 0; i < 8; i++) {
		if (data_bit) {
			GPIO_WriteBit(TXRX_PORT, TXRX_PIN, Bit_RESET);//HAL_GPIO_WritePin(TXRX_PORT, TXRX_PIN, GPIO_PIN_RESET);
			DelayMicro(3);
			GPIO_WriteBit(TXRX_PORT, TXRX_PIN, Bit_SET); //HAL_GPIO_WritePin(TXRX_PORT, TXRX_PIN, GPIO_PIN_SET);
			DelayMicro(90);
		} else {
			GPIO_WriteBit(TXRX_PORT, TXRX_PIN, Bit_RESET);  // HAL_GPIO_WritePin(TXRX_PORT, TXRX_PIN, GPIO_PIN_RESET);
			DelayMicro(90);
		  GPIO_WriteBit(TXRX_PORT, TXRX_PIN, Bit_SET);	//HAL_GPIO_WritePin(TXRX_PORT, TXRX_PIN, GPIO_PIN_SET);
			DelayMicro(3);
		}
		mask = mask << 1;
		data_bit = data & mask;
	}
}

/**
 * Read one bit from DS18B20.
 *
 * @param 	None.
 * @return  1 if succeed, 0 if failed.
 */
uint8_t ds18b20_read_bit(void)
{
	GPIO_WriteBit(TXRX_PORT, TXRX_PIN, Bit_RESET);//HAL_GPIO_WritePin(TXRX_PORT, TXRX_PIN, GPIO_PIN_RESET);
	DelayMicro(2);
	GPIO_WriteBit(TXRX_PORT, TXRX_PIN, Bit_SET);  //HAL_GPIO_WritePin(TXRX_PORT, TXRX_PIN, GPIO_PIN_SET);
	DelayMicro(10);
	if (GPIO_ReadInputDataBit(TXRX_PORT, TXRX_PIN)== Bit_RESET) {
		DelayMicro(58);
		return 0;
	} else {
		DelayMicro(58);
		return 1;
	}
}

/**
 * Read one byte from DS18B20.
 *
 * @param 	None.
 * @return  One byte of data read from DS18B20.
 */
uint8_t ds18b20_read_byte(void)
{
	uint8_t i, data = 0;
	for (i = 0; i < 8; i++)
		data |= (ds18b20_read_bit() << i);
	return data;
}
