/*
 * sx1278.c
 *
 *  Created on: Nov 18, 2023
 *      Author: Patrick
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "main.h"
#include "sx1278.h"

//Change This When Changing uC
//Chip Select Pin for F0 --> GPIOA , GPIO_PIN_4
#define cs_group GPIOA
#define cs_pin GPIO_PIN_4

#define WRITE_MASK 0b10000000
#define READ_MASK 0b01111111

#define DATA_SIZE 0x40

//Gets the IRQ2 Register Status
uint8_t get_irq1_register(SPI_HandleTypeDef *hspi)
{
	return spi_single_read(hspi, REG_IRQFLAGS1);;
}


//Gets the IRQ2 Register Status
uint8_t get_irq2_register(SPI_HandleTypeDef *hspi)
{
	return spi_single_read(hspi, REG_IRQFLAGS2);;
}


//This Writes to a single register
void spi_single_write(SPI_HandleTypeDef *hspi, uint8_t address, uint8_t data)
{
	address |= WRITE_MASK;
	HAL_GPIO_WritePin(cs_group, cs_pin, 0);
	HAL_SPI_Transmit(hspi, &address, sizeof(address), 100);
	HAL_SPI_Transmit(hspi, &data, sizeof(data), 100);
	HAL_GPIO_WritePin(cs_group, cs_pin, 1);
}

//This reads a single register
uint8_t spi_single_read(SPI_HandleTypeDef *hspi, uint8_t address)
{
	uint8_t rx_data;
	address &= READ_MASK;
	HAL_GPIO_WritePin(cs_group, cs_pin, 0);
	HAL_SPI_Transmit(hspi, &address, sizeof(address), 100);
	HAL_SPI_Receive(hspi, &rx_data, sizeof(rx_data), 100);
	HAL_GPIO_WritePin(cs_group, cs_pin, 1);
	return rx_data;
}

//Only Change Below if the Value is different
//From the default setting in Datasheet
void sx1278_struct_init(SX1278 *radio)
{
	//Common Settings
	radio->RegOpMode |= RF_OPMODE_STANDBY | RF_OPMODE_FREQMODE_ACCESS_LF;
	radio->RegBitrateMsb |= RF_BITRATEMSB_250000_BPS;
	radio->RegBitrateLsb |= RF_BITRATELSB_250000_BPS;
	//You Have to Calculate with Eqs on Datasheet
//	radio->RegFrfMsb = 0x6c;
//	radio->RegFrfMid = 0x80;
//	radio->RegFrfLsb = 0x00;
	//TX/RX Settings
	radio->RegPaConfig |= RF_PACONFIG_PASELECT_RFO | 0x04 | 0x0f;
	radio->RegPaRamp |= 0x00;
	radio->RegLna |= RF_LNA_GAIN_G6;
//	radio->RegFdevMsb |= 0b00111010;
	//TCXO Settings:
	radio->RegTcxo = RF_TCXO_TCXOINPUT_ON;
	radio->RegFifoThresh |=  0x0b00111111;
}

//This gets the status of all registers.
//Mainly for init purposes
uint8_t sx1278_read_all_registers(SX1278 *radio, SPI_HandleTypeDef *hspi)
{
	uint8_t *struct_ptr = &(radio->RegOpMode);
	for(uint8_t reg = 1; reg <= REGISTER_COUNT; reg++)
	{
		*(struct_ptr + ((reg-1))) = spi_single_read(hspi, reg);
	}
	uint8_t temp = spi_single_read(hspi, REG_VERSION);
	if(temp == CHIP_VERSION)
	{
		return 1;
	}
	return 0;
}

//This Function updates all registers with the desired configuration
//Probably will be only used for init and major function changes.
uint8_t sx1278_write_all_registers(SX1278 *radio, SPI_HandleTypeDef *hspi)
{
	uint8_t *struct_ptr = &(radio->RegOpMode);
	for(uint8_t reg = 1; reg <= REGISTER_COUNT; reg++)
		{
			spi_single_write(hspi, reg, *(struct_ptr + ((reg-1))));
		}
	uint8_t temp = spi_single_read(hspi, REG_OPMODE);
	if(radio->RegOpMode == temp)
	{
		return 1;
	}
	return 0;
}

void sx1278_mem_init(SPI_HandleTypeDef *hspi, radio *radio)
{
	// Set for the SX App
	radio->tx_flags.tx_init = 0;
	radio->tx_flags.tx_inp= 0;
	radio->tx_buffer_prog = 0;
}

//General Init Function for the Module.
uint8_t sx1278_init(radio *radio, SPI_HandleTypeDef *hspi)
{
	uint8_t timeout_counter = 0;
	uint8_t stat = 0;
	while(stat == 0 && timeout_counter < TIMEOUT_COUNTER_LIM)
	{
		stat = sx1278_read_all_registers(&(radio->radio), hspi);
		timeout_counter++;
		if(timeout_counter == TIMEOUT_COUNTER_LIM-1)
		{
			return 0;
		}
	}
	timeout_counter = 0;
	stat = 0;
	sx1278_struct_init(&(radio->radio));
	radio->sx_state = STANDBY;
	while(stat == 0 && timeout_counter < TIMEOUT_COUNTER_LIM)
	{
		timeout_counter++;
		stat = sx1278_write_all_registers(&(radio->radio), hspi);
		if(timeout_counter == TIMEOUT_COUNTER_LIM-1)
		{
			return 0;
		}
	}
	sx1278_mem_init(hspi, radio);
	return 1;
}

uint8_t sx1278_fifo_fill(SPI_HandleTypeDef *hspi, uint8_t* data)
{
	uint8_t address_packet = WRITE_MASK | REG_FIFO;
 	for(uint8_t i = 0; i < DATA_SIZE; i++)
	{
 		spi_single_write(hspi, address_packet, data[i]);
	}
 	if((get_irq2_register(hspi) & 0x00100000 )== 0x00100000)
	{
 		//If Fifo is filled tell the higher level and adjust the tx_buffer.
 		return 1;
	}
	return 0;
}

uint8_t change_opmode(radio *radio, SPI_HandleTypeDef *hspi, radio_state new_mode)
{
	uint8_t timeout_counter = 0;
	uint8_t stat = 0;
	while(stat == 0 && timeout_counter < TIMEOUT_COUNTER_LIM)
	{
		stat = sx1278_read_all_registers(&(radio->radio), hspi);
		timeout_counter++;
		if(timeout_counter == TIMEOUT_COUNTER_LIM-1)
		{
			return 0;
		}
	}
	uint8_t temp_mode = RF_OPMODE_MODULATIONTYPE_FSK |
			RF_OPMODE_FREQMODE_ACCESS_LF |new_mode;
	radio->radio.RegOpMode = temp_mode;
	radio->sx_state = new_mode;
	spi_single_write(hspi, REG_OPMODE, (radio->radio.RegOpMode));
	return 1;
}

void packet(radio* radio, uint8_t *dat)
{
	uint8_t packet_to_send[DATA_SIZE];
	uint8_t remaining = radio->tx_buffer_size - radio->tx_buffer_prog;
	if(remaining > DATA_SIZE)
	{
		memcpy(packet_to_send, &(radio->tx_buffer[radio->tx_buffer_prog]), DATA_SIZE);
		radio->tx_buffer_prog += DATA_SIZE;
	}
	else if(remaining == DATA_SIZE)
	{
		//If there are 64 bytes of data left in the buffer just return the buffer
		memcpy(packet_to_send, &(radio->tx_buffer[radio->tx_buffer_prog]), DATA_SIZE);
		//This is last packet in buffer take out of tx
		radio->tx_buffer_size = 0;
		radio->tx_buffer_prog = 0;
		radio->tx_flags.tx_inp = 0;
	}
	else if(remaining < DATA_SIZE)
	{
		memcpy(packet_to_send, &(radio->tx_buffer[radio->tx_buffer_prog]), DATA_SIZE - remaining);
		for(uint8_t place = remaining; place < DATA_SIZE; place++)
		{
			packet_to_send[place] = '\0';
		}
		//This is last packet in buffer take out of tx
		radio->tx_buffer_size = 0;
		radio->tx_buffer_prog = 0;
		radio->tx_flags.tx_inp = 0;
	}
	memcpy(dat, packet_to_send, DATA_SIZE);
}
//This Function fills the FIFO with the input data and sets the opmode to transmit.
//It will be up to the app to check when the tx is done.
void SX1278_APP(radio *radio, SPI_HandleTypeDef *hspi)
{
	switch(radio->sx_state)
	{
	case SLEEP:
		break;
	case STANDBY:
		break;
	case TRANSMITTER:
		if(radio->tx_flags.tx_init == 0)
		{
			// If the Radio has not been initialized for TX Set DIO  & FiFo thresh
			//The Following Sets the TX Start to condition to anything above 0 & the thresh to datasize.
			radio->radio.RegFifoThresh = RF_FIFOTHRESH_TXSTARTCONDITION_FIFONOTEMPTY | (DATA_SIZE-1);
			spi_single_write(hspi, REG_FIFOTHRESH, radio->radio.RegFifoThresh);
			change_opmode(radio, hspi, TRANSMITTER);
			radio->tx_flags.tx_init = 1;
			//How my brain works fuck you
			radio->tx_flags.tx_inp = 1;
		}
		else if(radio->tx_flags.tx_inp == 1)
		{
			uint8_t packet_to_send[64];
			if(radio->tx_flags.tx_fifo_full == 0)
			{
				packet(radio, packet_to_send);
				if(sx1278_fifo_fill(hspi, packet_to_send) == 1)
				{
					radio->tx_flags.tx_fifo_full = 1;
				}
				//Check if fifo is full
			}
			else if(radio->tx_flags.tx_fifo_full == 1)
			{
				//Check if fifo is empty and is ready to be written to
				if((get_irq2_register(hspi) & 0x01000000 )== 0x01000000)
				{
					radio->tx_flags.tx_fifo_full = 0;
				}
			}
		}
		else
		{
			radio->tx_buffer_size = 0;
			radio->tx_buffer_prog = 0;
			radio->tx_flags.tx_init = 0;
			radio->tx_flags.tx_inp = 0;
			radio->tx_flags.tx_fifo_full = 0;
			radio->sx_state = STANDBY;
			change_opmode(radio, hspi, STANDBY);
		}
		break;
	case RECEIVER:
		break;
	}

}

/*
 * sx1278.c
 *
 *  Created on: Nov 18, 2023
 *      Author: Patrick
 */
