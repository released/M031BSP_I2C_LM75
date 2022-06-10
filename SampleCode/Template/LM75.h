

#ifndef LM75_H
#define LM75_H

#include <stdio.h>

typedef unsigned char bool;

/* list of I2C addresses */
#define LM75_DEVADDR_7BIT					(0x48)		// 7 bit
#define LM75_DEVADDR_8BIT					(LM75_DEVADDR_7BIT << 1)


signed int readIntegerTemperatureRegister(unsigned char register_index);
void writeIntegerTemperatureRegister(unsigned char register_index, signed int value);
unsigned char readConfigurationRegister(void);
void writeConfigurationRegister(unsigned char configuration);
void setConfigurationBits(unsigned char bits);
void clearConfigurationBits(unsigned char bits);
void setConfigurationBitValue(unsigned char value, unsigned char start, unsigned char width);
float readTemperatureC(void);
  
void LM75_WriteReg(unsigned char RegAddr, unsigned char* txData , unsigned char length);
int LM75_ReadReg(unsigned char RegAddr , unsigned char* rxData , unsigned char length);

void LM75_Init(void);

#endif /* LM75_H */

