/*
 * mcp_2515.h
 *
 * Created: 7/5/2018 21:48:50 PM
 * Author : Matt
 */

#ifndef __MCP2515_CFG_H
#define __MCP2515_CFG_H

#include "MCP2515_defines.h"
#include "SPI.h"
#include <stdlib.h>
#define MCP_CNF1 0x2a
#define MCP_CNF2 0x29
#define MCP_CNF3 0x28
// MCP2515 Function Declarations

// Send reset command to MCP2515
void mcp2515_reset(void);

// Read specific register address and return the data
uint8_t mcp2515_read(uint8_t address);

// Write specific register address
void mcp2515_write(uint8_t address, uint8_t data);

// Modify specific masks and bits for select registers w/ this functionality
void mcp2515_bitModify(uint8_t address, uint8_t maskByte, uint8_t dataByte);

// Return MCP2515 status command byte
uint8_t mcp2515_readStatus(void);

// allocates space and returns a pointer to the data, caller's responsibility to free after use
uint8_t* mcp2515_readArray(uint8_t address, uint8_t length);

// User provides data to send and passes pointer to data, function then sends data
void mcp2515_writeArray(uint8_t address, uint8_t *data, uint8_t length);

// Initializes CAN bus and sets up controller to send and receive messages
void can_init(void);

// Initializes a message struct for the can bus to create predefined messages
struct can_message{
	uint8_t idH;
	uint8_t idL;
	uint8_t *data;
	uint8_t length;
	};

// Sets the id, the data length code, the data and sets flag to send message
void can_message_send(struct can_message *message);

// When message is received and available, function writes received data to passed struct pointer
void can_message_receive(struct can_message *received_message);

#endif