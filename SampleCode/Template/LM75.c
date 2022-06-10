
#include <stdio.h>
#include <stdlib.h>
#include "LM75.h"

#include "i2c_driver.h"
#include <math.h>
#include "project_config.h"



#ifndef bit
#define bit(x)								(uint32_t)(1<<x)
#endif

// The layout of registers accessed through the I2C protocol.
typedef struct  {
	unsigned char temperature;
	unsigned char configuration;
	unsigned char temperature_low;
	unsigned char temperature_high;
}RegisterLayout;

// Attributes about a device or family of devices (if the attributes are shared).
typedef struct  {
	unsigned char temperature_width;
	unsigned char default_temperature_resolution;
	unsigned char default_temperature_frac_width;
	unsigned char max_temperature_resolution;
	RegisterLayout *registers;
}Attributes;


// The standard register layout for most devices, based on LM75.
RegisterLayout LM75_Compatible_Registers = {
	 0x00,
	 0x01,
	 0x02,
	 0x03,
};

Attributes Generic_LM75_Attributes = {
	 16,
	 9,
	 8,
	 9,
	 &LM75_Compatible_Registers,
};

enum ConfigurationBits {
	FaultQueueLength  = 3, // mask 0x18, length 2 bits
	AlertPolarity     = 2, // mask 0x04, length 1 bit
	ThermostatMode    = 1, // mask 0x02, length 1 bit
	Shutdown          = 0, // mask 0x01, length 1 bit
};

enum FaultQueueLength {
	FaultQueueLength_1_fault   = 0,
	FaultQueueLength_2_faults  = 1,
	FaultQueueLength_4_faults  = 2,
	FaultQueueLength_6_faults  = 3,
};
  

enum Resolution {
	Resolution_9_bits   = 0,
	Resolution_10_bits  = 1,
	Resolution_11_bits  = 2,
	Resolution_12_bits  = 3,
};


unsigned char i2c_address;
unsigned char resolution = 0;
unsigned int resolution_mask = 0;
unsigned char temperature_frac_width;
float temperature_frac_factor;
Attributes *attributes;

 
void LM75_Delay(unsigned int nCount)
{
    #if 1 // ms
    CLK_SysTickDelay(nCount*1000);
    #else
    /* Decrement nCount value */
    while (nCount != 0)
    {
      nCount--;
    }
    #endif
}

void LM75_WriteReg(unsigned char RegAddr, unsigned char* txData , unsigned char length)
{
    i2c_reg_write(LM75_DEVADDR_7BIT ,RegAddr ,txData ,length);
}

int LM75_ReadReg(unsigned char RegAddr , unsigned char* rxData , unsigned char length)
{
    i2c_reg_read(LM75_DEVADDR_7BIT ,RegAddr ,rxData ,length);	
    return 0;
}

 float convertCtoF(float c) {
return c * 1.8 + 32;
}

float convertFtoC(float f) {
return (f - 32) / 1.8;
}

// Set the internal resolution of the temperature sensor, which affects
// conversions and which bits are discarded.
void setInternalResolution(unsigned char input_resolution) {
	resolution = input_resolution;
	resolution_mask = ~(unsigned int)((1 << (attributes->temperature_width - input_resolution)) - 1);
}

void setInternalTemperatureFracWidth(unsigned char input_temperature_frac_width) {
	temperature_frac_width = input_temperature_frac_width;
	temperature_frac_factor = 1.0 / (float)(1 << input_temperature_frac_width);
}

signed int readIntegerTemperatureRegister(unsigned char register_index) {
	unsigned char* rxData ;	// or just use 	unsigned char rxData[2] = {0} ;
	unsigned int t = 0;
	unsigned char len = 0;

	len = (resolution <= 8 ? 1 : 2 );
	
	// rxData = (unsigned char*) malloc(len * sizeof(unsigned char));	// array not initial
	rxData = calloc(len , sizeof(unsigned char) );                	// initial arrray size as 0/NULL	
	// printf("rxData size : %2d/%2d (%2d)\r\n" , sizeof(rxData) , len , sizeof(rxData) / sizeof(rxData[0]));

	LM75_ReadReg(register_index , rxData ,len );	

	t = rxData[0] << 8;

	if (resolution > 8) {
		t |= rxData[1];
	}	
  
	// Mask out unused/reserved bit from the full 16-bit register.
	t &= resolution_mask;

	free(rxData);

	// Read the raw memory as a 16-bit signed integer and return.
	return *(signed int *)(&t);
}

void writeIntegerTemperatureRegister(unsigned char register_index, signed int value) {

	unsigned char txData[2] = {0} ;
	
	txData[0] = (unsigned char)(value & 0xff00) >> 8;
	txData[1] = (unsigned char)(value & 0x00ff);	
	LM75_WriteReg(register_index , txData , 2);	

}

unsigned char readConfigurationRegister(void) {
  
	unsigned char rxData[2] = {0} ;
	unsigned char register_index = attributes->registers->configuration;
	
	LM75_ReadReg(register_index , rxData , 1);	
   
	return rxData[0];
}

void writeConfigurationRegister(unsigned char configuration) {
  
	// unsigned char txData[2] = {0} ;
	unsigned char register_index = attributes->registers->configuration;
	
	LM75_WriteReg(register_index , &configuration , 1);	  
    
}

void setConfigurationBits(unsigned char bits) {
  unsigned char configuration = readConfigurationRegister();

  configuration |= bits;

  writeConfigurationRegister(configuration);
}

void clearConfigurationBits(unsigned char bits) {
  unsigned char configuration = readConfigurationRegister();

  configuration &= ~bits;

  writeConfigurationRegister(configuration);
}


void setConfigurationBitValue(unsigned char value, unsigned char start, unsigned char width) {
  unsigned char configuration = readConfigurationRegister();

  unsigned char mask = ((1 << width) - 1) << start;

  configuration &= ~mask;
  configuration |= value << start;

  writeConfigurationRegister(configuration);
}

 float convertIntegerTemperature(signed int value) {
    return (float)value * temperature_frac_factor;
  }

  signed int convertFloatTemperature(float value) {
    return (signed int)(value / temperature_frac_factor);
  }

float readTemperatureC(void) {
	return convertIntegerTemperature(readIntegerTemperatureRegister(attributes->registers->temperature));
}

float readTemperatureF(void) {
	return convertCtoF(readTemperatureC());
}

float readTemperatureLowC(void) {
	return convertIntegerTemperature(readIntegerTemperatureRegister(attributes->registers->temperature_low));
}

float readTemperatureLowF(void) {
	return convertCtoF(readTemperatureLowC());
}

void setTemperatureLowC(float value) {
	writeIntegerTemperatureRegister(attributes->registers->temperature_low, convertFloatTemperature(value));
}

void setTemperatureLowF(float value) {
	setTemperatureLowC(convertFtoC(value));
}

float readTemperatureHighC(void) {
	return convertIntegerTemperature(readIntegerTemperatureRegister(attributes->registers->temperature_high));
}

float readTemperatureHighF(void) {
	return convertCtoF(readTemperatureHighC());
}

void setTemperatureHighC(float value) {
	writeIntegerTemperatureRegister(attributes->registers->temperature_high, convertFloatTemperature(value));
}

void setTemperatureHighF(float value) {
	setTemperatureHighC(convertFtoC(value));
}


unsigned char readConfigurationBits(unsigned char bits) {
	return readConfigurationRegister() & bits;
}

bool checkConfigurationBits(unsigned char bits) {
	return (readConfigurationRegister() & bits) == bits;
}

void setFaultQueueLength(enum FaultQueueLength faults) {
	setConfigurationBitValue(faults, FaultQueueLength, 2);
}

void setAlertActiveLow(void) {
	clearConfigurationBits(bit(AlertPolarity));
}

void setAlertActiveHigh(void) {
	setConfigurationBits(bit(AlertPolarity));
}

void setThermostatComparatorMode(void) {
	clearConfigurationBits(bit(ThermostatMode));
}

void setThermostatInterruptMode(void) {
	setConfigurationBits(bit(ThermostatMode));
}

void enableShutdownMode(void) {
	setConfigurationBits(bit(Shutdown));
}

void disableShutdownMode(void) {
	clearConfigurationBits(bit(Shutdown));
}

void LM75_Init(void)
{
	attributes = &Generic_LM75_Attributes;
    setInternalResolution(attributes->default_temperature_resolution);
    setInternalTemperatureFracWidth(attributes->default_temperature_frac_width);

}

