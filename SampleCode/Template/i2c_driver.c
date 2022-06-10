/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "i2c_driver.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/
volatile uint8_t g_u8DeviceAddr_m;
volatile uint8_t g_u8DataLen_m;
volatile uint8_t rawlenth;
volatile uint8_t g_au8Reg;
volatile uint8_t g_u8EndFlag = 0;
uint8_t *g_au8Buffer;

typedef void (*I2C_FUNC)(uint32_t u32Status);

I2C_FUNC __IO I2Cx_Master_HandlerFn = NULL;

uint8_t u8RxStopAndStart = 0;
/*_____ M A C R O S ________________________________________________________*/


/*_____ F U N C T I O N S __________________________________________________*/


void I2Cx_Master_LOG(uint32_t u32Status)
{
	#if defined (DEBUG_LOG_MASTER_LV1)
    printf("%s  : 0x%2x \r\n", __FUNCTION__ , u32Status);
	#endif
}

void I2Cx_Master_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(MASTER_I2C);

    if (I2C_GET_TIMEOUT_FLAG(MASTER_I2C))
    {
        /* Clear I2C Timeout Flag */
        I2C_ClearTimeoutFlag(MASTER_I2C);                   
    }    
    else
    {
        if (I2Cx_Master_HandlerFn != NULL)
            I2Cx_Master_HandlerFn(u32Status);
    }
}

void I2Cx_MasterRx_multi(uint32_t u32Status)
{
    if(u32Status == MASTER_START_TRANSMIT) //0x08                       	/* START has been transmitted and prepare SLA+W */
    {
		if (u8RxStopAndStart == 0)
		{
			I2C_SET_DATA(MASTER_I2C, ((g_u8DeviceAddr_m << 1) | I2C_WR));    				/* Write SLA+W to Register I2CDAT */
			I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI);
		}
		else
		{
			I2C_SET_DATA(MASTER_I2C, ((g_u8DeviceAddr_m << 1) | I2C_RD));   		/* Write SLA+R to Register I2CDAT */
			I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI);
		}

		I2Cx_Master_LOG(u32Status);
    }
    else if(u32Status == MASTER_TRANSMIT_ADDRESS_ACK) //0x18        			/* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(MASTER_I2C, g_au8Reg);
        I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI);
		
		I2Cx_Master_LOG(u32Status);
    }
    else if(u32Status == MASTER_TRANSMIT_ADDRESS_NACK) //0x20            	/* SLA+W has been transmitted and NACK has been received */
    {
        I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI | I2C_CTL_STA | I2C_CTL_STO);

//        I2C_STOP(MASTER_I2C);
//        I2C_START(MASTER_I2C);
		
		I2Cx_Master_LOG(u32Status);
    }
    else if(u32Status == MASTER_TRANSMIT_DATA_ACK) //0x28                  	/* DATA has been transmitted and ACK has been received */
    {
        if (rawlenth > 0)
			I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI | I2C_CTL_STA);				//repeat start
		else
		{
			I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI | I2C_CTL_STO);
			g_u8EndFlag = 1;
		}
		
		I2Cx_Master_LOG(u32Status);
    }
    else if(u32Status == MASTER_REPEAT_START) //0x10                  		/* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(MASTER_I2C, ((g_u8DeviceAddr_m << 1) | I2C_RD));   		/* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI);
		
		I2Cx_Master_LOG(u32Status);
    }
    else if(u32Status == MASTER_RECEIVE_ADDRESS_ACK) //0x40                	/* SLA+R has been transmitted and ACK has been received */
    {
		if (rawlenth > 1)
			I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI | I2C_CTL_AA);
		else
			I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI);

		I2Cx_Master_LOG(u32Status);
    }
	else if(u32Status == MASTER_RECEIVE_DATA_ACK) //0x50                 	/* DATA has been received and ACK has been returned */
    {
        g_au8Buffer[g_u8DataLen_m++] = (unsigned char) I2C_GetData(MASTER_I2C);
        if (g_u8DataLen_m < (rawlenth-1))
		{
			I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI | I2C_CTL_AA);
		}
		else
		{
			I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI);
		}
		
		I2Cx_Master_LOG(u32Status);
    }
    else if(u32Status == MASTER_RECEIVE_DATA_NACK) //0x58                  	/* DATA has been received and NACK has been returned */
    {
        g_au8Buffer[g_u8DataLen_m++] = (unsigned char) I2C_GetData(MASTER_I2C);
        I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI | I2C_CTL_STO);
        g_u8EndFlag = 1;

		
		I2Cx_Master_LOG(u32Status);
    }
    else
    {
		#if defined (DEBUG_LOG_MASTER_LV1)
        /* TO DO */
        printf("I2Cx_MasterRx_multi Status 0x%x is NOT processed\n", u32Status);
		#endif
    }
}

void I2Cx_MasterTx_multi(uint32_t u32Status)
{
    if(u32Status == MASTER_START_TRANSMIT)  //0x08                     	/* START has been transmitted */
    {
        I2C_SET_DATA(MASTER_I2C, ((g_u8DeviceAddr_m << 1) | I2C_WR));    			/* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI);

		I2Cx_Master_LOG(u32Status);
		
    }
    else if(u32Status == MASTER_TRANSMIT_ADDRESS_ACK)  //0x18           	/* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(MASTER_I2C, g_au8Reg);
        I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI);
		
		I2Cx_Master_LOG(u32Status);	
    }
    else if(u32Status == MASTER_TRANSMIT_ADDRESS_NACK) //0x20           /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI | I2C_CTL_STA | I2C_CTL_STO);

//        I2C_STOP(MASTER_I2C);
//        I2C_START(MASTER_I2C);

		I2Cx_Master_LOG(u32Status);	
    }
    else if(u32Status == MASTER_TRANSMIT_DATA_ACK) //0x28              	/* DATA has been transmitted and ACK has been received */
    {
        if(g_u8DataLen_m < rawlenth)
        {
            I2C_SET_DATA(MASTER_I2C, g_au8Buffer[g_u8DataLen_m++]);
            I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI | I2C_CTL_STO);
            g_u8EndFlag = 1;
        }

		I2Cx_Master_LOG(u32Status);		
    }
    else if(u32Status == MASTER_ARBITRATION_LOST) //0x38
    {
		I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_STA_SI_AA);

		I2Cx_Master_LOG(u32Status);		
    }
    else if(u32Status == BUS_ERROR) //0x00
    {
		I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_STO_SI_AA);
		I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI_AA);
		
		I2Cx_Master_LOG(u32Status);		
    }		
    else
    {
		#if defined (DEBUG_LOG_MASTER_LV1)
        /* TO DO */
        printf("I2Cx_MasterTx_multi Status 0x%x is NOT processed\n", u32Status);
		#endif
    }
}

void I2Cx_WriteMultiToSlaveIRQ(unsigned char address,unsigned char reg,unsigned char *data,unsigned int len)
{		
	g_u8DeviceAddr_m = address;
	rawlenth = len;
	g_au8Reg = reg;
	g_au8Buffer = data;

	g_u8DataLen_m = 0;
	g_u8EndFlag = 0;

	/* I2C function to write data to slave */
	I2Cx_Master_HandlerFn = (I2C_FUNC)I2Cx_MasterTx_multi;

//	printf("I2Cx_MasterTx_multi finish\r\n");

	/* I2C as master sends START signal */
	I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_STA);

	/* Wait I2C Tx Finish */
	while(g_u8EndFlag == 0);
	g_u8EndFlag = 0;
}

void I2Cx_ReadMultiFromSlaveIRQ(unsigned char address,unsigned char reg,unsigned char *data,unsigned int len)
{ 
	g_u8DeviceAddr_m = address;
	rawlenth = len;
	g_au8Reg = reg ;
	g_au8Buffer = data;

	g_u8EndFlag = 0;
	g_u8DataLen_m = 0;

	u8RxStopAndStart = 0;
	/* I2C function to read data from slave */
	I2Cx_Master_HandlerFn = (I2C_FUNC)I2Cx_MasterRx_multi;

//	printf("I2Cx_MasterRx_multi finish\r\n");
	
	I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_STA);

	/* Wait I2C Rx Finish */
	while(g_u8EndFlag == 0);
	
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Write I2C                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void I2Cx_WriteSingleToSlaveIRQ(unsigned char address,unsigned int reg, unsigned char *data)
{
	g_u8DeviceAddr_m = address;
	rawlenth = 1;
	g_au8Reg = reg;
	g_au8Buffer = data;

	g_u8DataLen_m = 0;
	g_u8EndFlag = 0;
	
	/* I2C function to write data to slave */
	I2Cx_Master_HandlerFn = (I2C_FUNC)I2Cx_MasterTx_multi;
	
	/* I2C as master sends START signal */
	I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_STA);

	/* Wait I2C Tx Finish */
	while (g_u8EndFlag == 0);	
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Read I2C                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void I2Cx_ReadSingleToSlaveIRQ(unsigned char address, unsigned int reg,unsigned char *data)
{
	g_u8DeviceAddr_m = address;
	rawlenth = 1;
	g_au8Reg = reg ;
	g_au8Buffer = data;

	g_u8DataLen_m = 0;
	g_u8EndFlag = 0;
	
	/* I2C function to write data to slave */
	I2Cx_Master_HandlerFn = (I2C_FUNC)I2Cx_MasterRx_multi;
	
	/* I2C as master sends START signal */
	I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_STA);

	/* Wait I2C Tx Finish */
	while (g_u8EndFlag == 0);	
}

char i2c_reg_write(unsigned char i2c_addr, unsigned char reg_addr, unsigned char *reg_data, unsigned int length)
{
	#if defined (ENABLE_I2C_POLLING_DISCRETE)
	unsigned char i = 0;	
	unsigned char tmp = 0;	
	I2C_T *i2c = MASTER_I2C;	
	
	I2C_START(i2c);                    											//Start
	I2C_WAIT_READY(i2c);

	I2C_SET_DATA(i2c, (i2c_addr << 1) | I2C_WR );        						//send slave address
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_WAIT_READY(i2c);

	// if (reg_addr != NULL)	// due to this device use 0x00 as first register
	{
		I2C_SET_DATA(i2c, (reg_addr & 0xFF));             							//send index
		I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
		I2C_WAIT_READY(i2c);	

		for (i = 0; i < length; i++)
		{
			tmp = reg_data[i];
			I2C_SET_DATA(i2c, tmp);            										//send Data
			I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
			I2C_WAIT_READY(i2c);
		}	
	}

	I2C_STOP(i2c);																//Stop
	#else
	I2Cx_WriteMultiToSlaveIRQ(i2c_addr , reg_addr , reg_data  , length);
	#endif

	return 0 ;

    /* Implement the I2C write routine according to the target machine. */
//    return -1;
}

char i2c_reg_read(unsigned char i2c_addr, unsigned char reg_addr, unsigned char *reg_data, unsigned int length)
{
	#if defined (ENABLE_I2C_POLLING_DISCRETE)
	unsigned char i = 0;	
	unsigned char tmp = 0;
	I2C_T *i2c = MASTER_I2C;

	// if (reg_addr != NULL)	// due to this device use 0x00 as first register
	{
		I2C_START(i2c);                         									//Start
		I2C_WAIT_READY(i2c);

		I2C_SET_DATA(i2c, (i2c_addr << 1) | I2C_WR );           					//send slave address+W
		I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
		I2C_WAIT_READY(i2c);

		I2C_SET_DATA(i2c, (reg_addr & 0xFF));             							//send index
		I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
		I2C_WAIT_READY(i2c);	
	}

//	I2C_STOP(i2c);																//Stop
	////////////////////////////////////////////
	// I2C_START(i2c);                         									//Start
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_STA_SI);
	I2C_WAIT_READY(i2c);

	I2C_SET_DATA(i2c, (i2c_addr << 1) | I2C_RD );    							//send slave address+R
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_WAIT_READY(i2c);

	// refer to BMP280 datasheet , Figure 8: I2C multiple byte read 
	for (i = 0; i < length; i++)
	{
		if (i == (length -1))														//last byte : NACK
		{
//			I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO);
			I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
		}
		else			// ACK
		{
			I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
		}
		I2C_WAIT_READY(i2c);
		tmp = I2C_GET_DATA(i2c);           				//read data
		reg_data[i]=tmp;
	}

//	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);
	
	#else
	I2Cx_ReadMultiFromSlaveIRQ(i2c_addr , reg_addr , reg_data  , length);
	#endif
	
	return 0 ;

    /* Implement the I2C read routine according to the target machine. */
//    return -1;
}


void I2C_WriteData(unsigned char SlaveAddr,unsigned char RegAddr,unsigned char wdata, unsigned int Len)
{
	unsigned char _slaveAddr = SlaveAddr >> 1;	
	unsigned char _value = wdata;

	i2c_reg_write(_slaveAddr , RegAddr , &_value  , Len);
}

void I2C_ReadData(unsigned char SlaveAddr,unsigned char RegAddr,unsigned char* rdata, unsigned int Len)
{
	unsigned char _slaveAddr = SlaveAddr >> 1;	

	i2c_reg_read(_slaveAddr , RegAddr , rdata  , Len);
}

