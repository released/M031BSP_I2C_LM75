/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "project_config.h"

#include "LM75.h"
#include "i2c_driver.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/
volatile uint32_t BitFlag = 0;
volatile uint32_t counter_tick = 0;

#define NUM_AVG_TRAGET 								(8)
#define NUM_SAMPLE_DELETE 							(4ul)
#define NUM_AVG_POW	 								(3)

uint32_t convert_Humidity_data = 0;
uint32_t convert_Temperature_data = 0;

enum
{
	read_Humidity , 
	read_Temperature ,

};

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void tick_counter(void)
{
	counter_tick++;
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void compare_buffer(uint8_t *src, uint8_t *des, int nBytes)
{
    uint16_t i = 0;	
	
    for (i = 0; i < nBytes; i++)
    {
        if (src[i] != des[i])
        {
            printf("error idx : %4d : 0x%2X , 0x%2X\r\n", i , src[i],des[i]);
			set_flag(flag_error , ENABLE);
        }
    }

	if (!is_flag_set(flag_error))
	{
    	printf("%s finish \r\n" , __FUNCTION__);	
		set_flag(flag_error , DISABLE);
	}

}

void reset_buffer(void *dest, unsigned int val, unsigned int size)
{
    uint8_t *pu8Dest;
//    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;

	#if 1
	while (size-- > 0)
		*pu8Dest++ = val;
	#else
	memset(pu8Dest, val, size * (sizeof(pu8Dest[0]) ));
	#endif
	
}

void copy_buffer(void *dest, void *src, unsigned int size)
{
    uint8_t *pu8Src, *pu8Dest;
    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;
    pu8Src  = (uint8_t *)src;


	#if 0
	  while (size--)
	    *pu8Dest++ = *pu8Src++;
	#else
    for (i = 0; i < size; i++)
        pu8Dest[i] = pu8Src[i];
	#endif
}

void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}

void  dump_buffer_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            printf("%02X ", pucBuff[nIdx + i]);
        printf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}

void delay(uint16_t dly)
{
/*
	delay(100) : 14.84 us
	delay(200) : 29.37 us
	delay(300) : 43.97 us
	delay(400) : 58.5 us	
	delay(500) : 73.13 us	
	
	delay(1500) : 0.218 ms (218 us)
	delay(2000) : 0.291 ms (291 us)	
*/

	while( dly--);
}


void delay_ms(uint16_t ms)
{
	TIMER_Delay(TIMER0, 1000*ms);
}

void Sort_tab(uint32_t tab[], uint8_t length)
{
	uint8_t l = 0x00, exchange = 0x01; 
	float tmp = 0x00;

	/* Sort tab */
	while(exchange==1) 
	{ 
		exchange=0; 
		for(l=0; l<length-1; l++) 
		{
			if( tab[l] > tab[l+1] ) 
			{ 
				tmp = tab[l]; 
				tab[l] = tab[l+1]; 
				tab[l+1] = tmp; 
				exchange=1; 
			}
		}
	} 
}

uint32_t Filter_GetNDelXAvg(uint8_t cmd , uint16_t GetN , uint16_t DelX)
{
	uint16_t n = 0;
	uint16_t total = GetN + DelX;
	uint32_t data_sample[NUM_AVG_TRAGET + NUM_SAMPLE_DELETE]= {0};
	uint32_t data_target = 0;
	uint32_t data_sum_target = 0;

	data_sum_target = 0;

	for ( n = 0 ; n < total ; n++)
	{
		if (cmd == read_Humidity)
		{
	
		}
		else if (cmd == read_Temperature)
		{
			data_sample[n]= (uint32_t) readTemperatureC();
		}				
	}

	/* Sort the samples */
	Sort_tab(data_sample,total);
	
	/* Add the samples */
	for (n = NUM_SAMPLE_DELETE/2; n < total - NUM_SAMPLE_DELETE/2; n++)
	{
		data_sum_target += (uint32_t) data_sample[n];
	}

	/* get avg */
	data_target = data_sum_target >> NUM_AVG_POW ;
	
	data_sum_target = 0;

	#if 0	// debug
	printf("GetNDelXAvg:data_target:0x%4X\r\n" , data_target );
	#endif

	return data_target;
}


void I2C0_Init(void)	//PC1 : SCL , PC0 : SDA
{
	uint8_t i = 0;

	// I2C bus recover
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC0MFP_Msk)) |
                    (SYS_GPC_MFPL_PC0MFP_GPIO | SYS_GPC_MFPL_PC1MFP_GPIO);

	PC1 = ENABLE;
	PC0 = ENABLE;
		
	for (i = 0 ; i < 9 ; i++)
	{
		PC1 = ENABLE;	//SET_SCL;
		CLK_SysTickDelay(10);
		PC1 = DISABLE;	//RESET_SCL;
		CLK_SysTickDelay(10);		
	}

	// STOP
	PC0 = DISABLE;	//RESET_SDA;
	CLK_SysTickDelay(10);
	PC1 = ENABLE;	//SET_SCL;
	CLK_SysTickDelay(10);
	PC0 = ENABLE;	//SET_SDA;
	CLK_SysTickDelay(10);

    SYS_ResetModule(I2C0_RST);

    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC0MFP_Msk)) |
                    (SYS_GPC_MFPL_PC0MFP_I2C0_SDA | SYS_GPC_MFPL_PC1MFP_I2C0_SCL);

    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, MASTER_I2C_SPEED);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

	#if defined (ENABLE_I2C_IRQ)
    I2C_EnableInt(MASTER_I2C);
    NVIC_EnableIRQ(MASTER_I2C_IRQn);
	#endif  

}

void custom_loop(void)
{
	if (is_flag_set(flag_LM75_GetData))
	{
		set_flag(flag_LM75_GetData , DISABLE);
		
		#if 1	// display float

		printf("Temperature:%4.6f\r\n" , readTemperatureC());

		#else	// display unsigned int
		convert_Temperature_data = Filter_GetNDelXAvg(read_Temperature , NUM_AVG_TRAGET , NUM_SAMPLE_DELETE);

		printf("Temperature:%6d\r\n" , convert_Temperature_data);
		#endif	
	}
}

void GPIO_Init (void)
{
    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);

    GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT);	
}

void TMR1_IRQHandler(void)
{
//	static uint32_t LOG = 0;
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
//        	printf("%s : %4d\r\n",__FUNCTION__,LOG++);
			PB14 ^= 1;			
		}

		if ((get_tick() % 1000) == 0)
		{
			set_flag(flag_LM75_GetData , ENABLE);
		}		
    }
}


void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res == 'x' || res == 'X')
	{
		NVIC_SystemReset();
	}

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
			case '1':
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
				NVIC_SystemReset();		
				break;
		}
	}
}

void UART02_IRQHandler(void)
{

    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
            UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(UART02_IRQn);
	
	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
	#endif	

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_HXTSTB_Msk);

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk|CLK_PWRCTL_LXTEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk|CLK_STATUS_LXTSTB_Msk);	

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    CLK_EnableModuleClock(TMR1_MODULE);
  	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    CLK_EnableModuleClock(I2C0_MODULE);

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);


    // SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC0MFP_Msk)) |
    //                 (SYS_GPC_MFPL_PC0MFP_I2C0_SDA | SYS_GPC_MFPL_PC1MFP_I2C0_SCL);

   /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M031 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
	SYS_Init();

	UART0_Init();
	GPIO_Init();
	TIMER1_Init();

    I2C0_Init();

	LM75_Init();

    /* Got no where to go, just loop forever */
    while(1)
    {
    	custom_loop();

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
