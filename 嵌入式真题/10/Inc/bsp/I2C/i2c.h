#ifndef __I2C_H
#define __I2C_H

#include "main.h"

void I2CStart(void);
void I2CStop(void);
unsigned char I2CWaitAck(void);
void I2CSendAck(void);
void I2CSendNotAck(void);
void I2CSendByte(unsigned char cSendByte);
unsigned char I2CReceiveByte(void);
void I2CInit(void);
void EEPROM_Write(unsigned char add,unsigned char dat);
unsigned char EEPROM_Read(unsigned char add);
void MCP4017_Write(unsigned char val);
unsigned char MCP4017_Read(void);
#endif
