/*
 * mcp_2515.h
 *
 * Created: 7/5/2018 21:58:30 PM
 * Author : Matt
 */

#include "mcp_2515.h"

void mcp2515_reset(void){
	SLAVE_SELECT;
	SPI_SendByte(MCP_RESET);
	SLAVE_DESELECT;
}

uint8_t mcp2515_read(uint8_t address){
	uint8_t r_data;
	
	SLAVE_SELECT;
	SPI_SendByte(MCP_READ);
	SPI_SendByte(address);
	r_data = SPI_ReadByte();
	SLAVE_DESELECT;
	
	return r_data;
}

void mcp2515_write(uint8_t address, uint8_t data){
	SLAVE_SELECT;
	SPI_SendByte(MCP_WRITE);
	SPI_SendByte(address);
	SPI_SendByte(data);
	SLAVE_DESELECT;
}

void mcp2515_bitModify(uint8_t address, uint8_t maskByte, uint8_t dataByte){
	SLAVE_SELECT;
	SPI_SendByte(MCP_BITMOD);
	SPI_SendByte(address);
	SPI_SendByte(maskByte);
	SPI_SendByte(dataByte);
	SLAVE_DESELECT;
}

uint8_t mcp2515_readStatus(void){
	uint8_t status;
	
	SLAVE_SELECT;
	SPI_SendByte(MCP_READ_STATUS);
	status = SPI_ReadByte();
	SLAVE_DESELECT;
	
	return status;
}

uint8_t* mcp2515_readArray(uint8_t address, uint8_t length){
	uint8_t i;
	uint8_t *data_out = malloc(length);
	
	SLAVE_SELECT;
	SPI_SendByte(MCP_READ);
	SPI_SendByte(address);
	
	for(i=0; i < length; i++){
		data_out[i] = SPI_ReadByte();
	}
	
	SLAVE_DESELECT;
	
	return data_out;
}

void mcp2515_writeArray(uint8_t address, uint8_t *data, uint8_t length){
	uint8_t i;
	
	SLAVE_SELECT
	SPI_SendByte(MCP_WRITE);
	SPI_SendByte(address);
	
	for(i=0; i < length; i++){
		SPI_SendByte(data[i]);
	}
	
	SLAVE_DESELECT;
}

void can_init(void){
		
	// Reset the chip, set it to loopback mode
	mcp2515_reset();
	
	// Set interrupts to be enabled when message received (RXB0) for testing
	mcp2515_bitModify(MCP_CANINTE, 0x03, 0x01);
	
	// Disable masks and filters to receive any message and disable rollover
	mcp2515_bitModify(MCP_RXB0CTRL, 0x64, 0x60);
	
	// Disable transmit buffer 0 empty interrupt enable bit
	mcp2515_bitModify(MCP_CANINTE, 0x04, 0x00);
	
	// Set baud rate prescaler, phase segments and other bit timing settings
	// Target baud rate is 500kbps with 16 MHz clk
	mcp2515_bitModify(MCP_CNF1, 0xFF, 0x00);
	mcp2515_bitModify(MCP_CNF2, 0xFF, 0xF0);
	mcp2515_bitModify(MCP_CNF3, 0xFF, 0x86);
	
	//mcp2515_bitModify(MCP_CANCTRL, 0xE0, MODE_LOOPBACK);
	
	// Uncomment the line below to swap to normal mode
	mcp2515_bitModify(MCP_CANCTRL, 0xE0, MODE_NORMAL);

}

void can_message_send(struct can_message *message){
	uint8_t message_length = (0x0F & message->length);
	
	mcp2515_bitModify(MCP_TXB0SIDH, 0xFF, message->idH);
	mcp2515_bitModify(MCP_TXB0SIDL, 0xE0, message->idL);
	
	mcp2515_bitModify(MCP_TXB0DLC, 0x40, 0x00);
	mcp2515_bitModify(MCP_TXB0DLC, 0x0F, message_length);
	
	mcp2515_writeArray(MCP_TXB0D0, message->data, message_length);
	mcp2515_bitModify(MCP_TXB0CTRL, 0x0B, 0x0B);
	
}

void can_message_receive(struct can_message *received_message){
	received_message->length = mcp2515_read(MCP_RXB0DLC) & 0x0F;
	received_message->data = mcp2515_readArray(MCP_RXB0D0, received_message->length);
	received_message->idH = mcp2515_read(MCP_RXB0SIDH);
	received_message->idL = mcp2515_read(MCP_RXB0SIDL);
	
}