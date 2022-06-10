/*_____ I N C L U D E S ____________________________________________________*/


/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/


#define ENABLE_I2C_POLLING_DISCRETE
// #define ENABLE_I2C_IRQ


#define MASTER_I2C						  								(I2C0)
#define MASTER_I2C_IRQn						  							(I2C0_IRQn)
#define I2Cx_Master_IRQHandler											(I2C0_IRQHandler)
#define MASTER_I2C_SPEED				  								(400000)

#define I2C_WR			  												(0x00)
#define I2C_RD			  												(0x01)

#define MASTER_START_TRANSMIT			  								(0x08)
#define MASTER_REPEAT_START               								(0x10)
#define MASTER_TRANSMIT_ADDRESS_ACK       								(0x18)
#define MASTER_TRANSMIT_ADDRESS_NACK      								(0x20)
#define MASTER_TRANSMIT_DATA_ACK          								(0x28)
#define MASTER_TRANSMIT_DATA_NACK         								(0x30)
#define MASTER_ARBITRATION_LOST           								(0x38)
#define MASTER_RECEIVE_ADDRESS_ACK        								(0x40)
#define MASTER_RECEIVE_ADDRESS_NACK       								(0x48)
#define MASTER_RECEIVE_DATA_ACK           								(0x50)
#define MASTER_RECEIVE_DATA_NACK          								(0x58)
#define BUS_ERROR                         								(0x00)

/*_____ M A C R O S ________________________________________________________*/
// #define DEBUG_LOG_MASTER_LV1

/*_____ F U N C T I O N S __________________________________________________*/

char i2c_reg_write(unsigned char i2c_addr, unsigned char reg_addr, unsigned char *reg_data, unsigned int length);
char i2c_reg_read(unsigned char i2c_addr, unsigned char reg_addr, unsigned char *reg_data, unsigned int length);


void I2C_WriteData(unsigned char SlaveAddr,unsigned char RegAddr,unsigned char wdata, unsigned int Len);
void I2C_ReadData(unsigned char SlaveAddr,unsigned char RegAddr,unsigned char* rdata, unsigned int Len);


