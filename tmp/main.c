/*
 * version 0.91 date:16-4-93
 * version 0.92 date:16-4-93
 *
 * version 0.94 date:24-4-93
 *git 
 * version 0.95 date:29-4-93
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_exti.h"

#include "misc.h"
#include "lcd4bit.h"
#include "delay.h"
#include "ds18b20.h"
#include <stdio.h>

#define LCD_PORT        GPIOA
#define saveitem        10
#define endmainmenu     2
#define password        11
#define wrongpassword   12
#define manual          2
/*
 * map:
 * PA0		backlight
 * PA1-7	lcd
 * PB0		<
 * PB1		>
 * PB6		/\
 * PB5		\/
 * PB13		pump
 * PB14		fan
 * PB15		lamp
 * PB7		1wire
 */

#define	Bl		GPIO_Pin_0
#define	Esc		GPIO_Pin_0
#define	Ok		GPIO_Pin_1
#define	Up		GPIO_Pin_6
#define	Down	GPIO_Pin_5
#define	Pump	GPIO_Pin_13
#define	Fan		GPIO_Pin_14
#define	Lamp	GPIO_Pin_15
#define	Key_Mask	GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2
#define BI		1

#define	endTask	20
#define	Time_Mask	0x000fffff
#define	Pump_Mask	0x00100000
#define	Fan_Mask	0x00200000
#define	Lamp_Mask	0x00400000

/***************************************************************************//**
 * Global variables, private define and typedef
 ******************************************************************************/
//#define RTCClockOutput_Enable  /* RTC Clock/64 is output on tamper pin(PC.13) */

__IO uint32_t  TimeDisplay = 0;
char *menu[20]={
		"Set Time",
		"Tasks",
		"Manual",
		"0",
		"0",
		"0",
		"0",
		"0",
		"0",
		"0",
		" item saved..",
		" Enter Pass...",
		" Wrong Pass..."
};


int wait=0;
volatile int num=1;
volatile char daily=1;
char Lcd_On=0;
int  Index[3]={3,6,9};
int  Indexf[3]={4,7,10};
char Time[16];
int time[3]={0,0,0};
uint32_t THH = 0, TMM = 0, TSS = 0,Task[42];
int KeyTemp;
/***************************************************************************//**
 * Declare function prototypes
 ******************************************************************************/
void GPIO_Configuration(void);
void RTC_Configuration(void);
void NVIC_Configuration(void);
void EXTI_Configuration(void);
uint16_t Key_Pressed(void);
uint32_t Modify_Task(int n);
uint32_t Time_Set(void);
char *Time_Display(uint32_t TimeVar);
//void Set_Time(void);
void Time_Show(void);
void Temp_Control(void);
void Time_Control(void);
void Show_Task(void);
void View_Task(int i);
void print(int i,int j,int a,int b,int c);

void RTC_IRQHandler(void);
void EXTI0_IRQHandler(void);
void RTCAlarm_IRQHandler(void);

 uint16_t BKPDataReg[42] =
   {
    BKP_DR2, BKP_DR3, BKP_DR4, BKP_DR5, BKP_DR6, BKP_DR7, BKP_DR8,
     BKP_DR9, BKP_DR10, BKP_DR11, BKP_DR12, BKP_DR13, BKP_DR14, BKP_DR15, BKP_DR16,
     BKP_DR17, BKP_DR18, BKP_DR19, BKP_DR20, BKP_DR21, BKP_DR22, BKP_DR23, BKP_DR24,
     BKP_DR25, BKP_DR26, BKP_DR27, BKP_DR28, BKP_DR29, BKP_DR30, BKP_DR31, BKP_DR32,
     BKP_DR33, BKP_DR34, BKP_DR35, BKP_DR36, BKP_DR37, BKP_DR38, BKP_DR39, BKP_DR40,
     BKP_DR41, BKP_DR42
   };
 uint16_t  Pass[4]={Up,Up,Up,Ok};
/**
  * @brief  The Low Speed External (LSE) clock is used as RTC clock source.
  * The RTC clock can be output on the Tamper pin (PC.13). To enable this functionality,
  * uncomment the corresponding line: #define RTCClockOutput_Enable.
  * The RTC is in the backup (BKP) domain, still powered by VBAT when VDD is switched off,
  * so the RTC configuration is not lost if a battery is connected to the VBAT pin.
  * A key value is written in backup data register1 (BKP_DR1) to indicate if the RTC
  * is already configured.
  * @param  None
  * @retval None
  */

 uint32_t tim2int(char h,char m,char s){
	 return (h*3600+m*60+s);
}
int main(void)
{
	GPIO_Configuration();
    /* NVIC configuration */
	NVIC_Configuration();
	EXTI_Configuration();
    /* Enable PWR and BKP clock */
	RTC_Configuration();	
	
	SysTick_Config(SystemCoreClock/1000000);

	  // Init vom LC-Display
	lcd_init();
	delay_ms(50);
	lcd_clear();
	lcd_gotoxy(1,1);
    lcd_putsf("wake up...");
    delay_ms(5000);
	GPIO_WriteBit(LCD_PORT, Bl, Bit_SET);

    if (BKP_ReadBackupRegister(BKP_DR1) != 7753)
    {
        /* Backup data register value is not correct or not yet programmed (when
           the first time the program is executed) */
        /* RTC Configuration */
    	//Lcd_On=1;
    	if(Lcd_On){
			GPIO_WriteBit(LCD_PORT, Bl, Bit_SET);
    	}

        /* Adjust time by values entred by the user on the hyperterminal */
        Time_Set();
        BKP_WriteBackupRegister(BKP_DR1, 7753);
    }
    else
    {
        /* Check if the Power On Reset flag is set
        if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
        {
            //printf("\r\n\n Power On Reset occurred....");
        }
        // Check if the Pin Reset flag is set
        else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
        {
           // printf("\r\n\n External Reset occurred....");
        }*/

      //  printf("\r\n No need to configure RTC....");
        /* Wait for RTC registers synchronization */
         RTC_WaitForSynchro();
        /* Enable the RTC Second */
        RTC_ITConfig(RTC_IT_SEC, ENABLE);
        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
        int j;
        for(j=0;j<endTask;j++){
            Task[j]=BKP_ReadBackupRegister(BKPDataReg[j]);
            delay_us(20);
        }
    }

#ifdef RTCClockOutput_Enable
    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    /* Allow access to BKP Domain */
   // PWR_BackupAccessCmd(ENABLE);

    /* Disable the Tamper Pin */
    BKP_TamperPinCmd(DISABLE); /* To output RTCCLK/64 on Tamper pin, the tamper
                                 functionality must be disabled */

    /* Enable RTC Clock Output on Tamper Pin */
    BKP_RTCOutputConfig(BKP_RTCOutputSource_CalibClock);
#endif

    /* Clear reset flags */
    RCC_ClearFlag();
    //RTC_SetAlarm(1000);
	while(1){
		/* Display time in infinite loop */
		Time_Show();
		while(Lcd_On){
            GPIO_WriteBit(LCD_PORT, Bl, Bit_SET);
            lcd_clear();
            lcd_gotoxy(1,1);
            char pass=0;
            while(Lcd_On && (pass<3)){				
                lcd_putsf(menu[password]);
                int i;
                for(i=0; i<4;i++){
					lcd_gotoxy(2,(i+6));
                    lcd_putsf("*");
                    if(Key_Pressed()==Pass[i]) {
                        pass++;
                    }
                }
                if(pass==3) pass=4;
                else {
                    pass=0;
                    lcd_clear();
					lcd_gotoxy(1,1);
                    lcd_putsf(menu[wrongpassword]);
                    delay_ms(250);
                   }
            }
			int i=0;
			while(Lcd_On){
                lcd_clear();
				lcd_gotoxy(1,1);
				lcd_putsf(menu[i]);
				lcd_cursor_blink();
				lcd_gotoxy(2,1);
				lcd_putsf(menu[i+1]);
				switch (Key_Pressed()) {
					case Up:
						i--;
						if (i<0){
							i=endmainmenu;
						}

						break;
					case Down:
						i++;
						if (i>endmainmenu){
								i=0;
							}
						break;
					case Esc:
                        Lcd_On=0;
						break;
					case Ok:
                        if (i==0)
                            Time_Set();
                        else if (i==1)
                            Show_Task();

						break;
					default:
						break;
				}
            }
        }
        GPIO_WriteBit(LCD_PORT, Bl, Bit_RESET);
        Lcd_On=0;
		EXTI_Configuration();
        if (BKP_ReadBackupRegister(BKP_DR1) != 0x7753){
            Temp_Control();
        }
        else if (BKP_ReadBackupRegister(BKP_DR1) == 0x7753){
            Time_Control();
        }
  	}
return 0;
}
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    /* Configure the GPIO_key pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_5|GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //configure output ac
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //
    // Configure PA0 as balight
    //
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
/*
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
*/

}

//**************************************************************************
//
//This function handles External lines 9 to 5 interrupt request.
//
//**************************************************************************


/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    /* Enable the RTC Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
void EXTI_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN, ENABLE);

	//
	 // Connect EXTI Line to PB0
	 //
	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

	 //
	 // Configure EXTI Line0 to generate an interrupt on rising or falling edge
	 //

	 EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	 EXTI_Init(&EXTI_InitStructure);

      //
     // Enable the EXTI0 Interrupt
     //
     NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);


}

/**09133589609
  * @brief  Configures the RTC.
  * @param  None
  * @retval None
  */
void RTC_Configuration(void)
{
    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);

    /* Reset Backup Domain */
    //BKP_DeInit();

    /* Enable LSE */
    RCC_LSEConfig(RCC_LSE_ON);
    /* Wait till LSE is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {}

    /* Select LSE as RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    /* Enable RTC Clock */
    RCC_RTCCLKCmd(ENABLE);

    /* Wait for RTC registers synchronization */
    RTC_WaitForSynchro();

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Enable the RTC Second */
    RTC_ITConfig(RTC_IT_SEC, ENABLE);

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Set RTC prescaler: set RTC period to 1sec */
    RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
}
/**
 * @brief Check for key pressed or no
 */
uint16_t Key_Pressed(void){
	//return Up;
	delay_ms(150);
	while(Lcd_On){
		if(GPIO_ReadInputDataBit(GPIOB, Esc)==RESET){
			wait=0;
			return Esc;
		}
		if(GPIO_ReadInputDataBit(GPIOB, Ok)==RESET){
			wait=0;
			return Ok;
		}
		if(GPIO_ReadInputDataBit(GPIOB, Up)==RESET){
			wait=0;
			return Up;
		}
		if(GPIO_ReadInputDataBit(GPIOB, Down)==RESET){
			wait=0;
			return Down;
		}
	}
	return 0;
}
/**
  * @brief  Returns the time entered by user, using Hyperterminal.
  * @param  None
  * @retval Current time RTC counter value
  */
uint32_t Modify_Task(int n)
{
// uint32_t Tmp_HH = 0xFF, Tmp_MM = 0xFF, Tmp_SS = 0xFF;

    Time_Display(Task[n]&Time_Mask);
	time[0]=THH;
	time[1]=TMM;
	time[2]=TSS;
	int i=0 ;
	char show=1;
	while(i<4&&Lcd_On){
		lcd_gotoxy(1,Index[i]);
		lcd_cursor_blink();
		if(show){
			show=0;
            lcd_clear();
			lcd_gotoxy(1,1);
            //lcd_putsf(menu);
            print(0,0,(int)THH, (int)TMM,(int)TSS);
            lcd_gotoxy(1,1);
		}

		switch (Key_Pressed()) {
			case Up:
				show=1;
				time[i]++;
				if (i==0){
					if (time[i]>24)
						time[i]=0;
				}
				else if(time[i]>59){
					time[i]=0;
				}
				break;
			case Down:
				show=1;
				time[i]--;
				if (i==0){
					if (time[i]<0)
						time[i]=23;
				}
				else if(time[i]<0){
					time[i]=59;
				}
				break;
			case Esc:
				return 0;
				break;
			case Ok:
				i++;
				if(i==4){
					Task[n]=time[1]*3600+time[2]*60+time[3];
					lcd_clear();
					lcd_gotoxy(1,1);
				    lcd_putsf(menu[saveitem]);
				}
				break;
			default:
				break;
		}
	}
	i=0;
    while(i<3&&Lcd_On){
		lcd_gotoxy(1,Indexf[i]);
		lcd_cursor_blink();
        char Tmp[3];
        Tmp[0]=(Task[n]&0x001>>5);
        Tmp[1]=(Task[n]&0x002>>5);
        Tmp[2]=(Task[n]&0x004>>5);
		if(show){
			show=0;
            lcd_clear();
			lcd_gotoxy(1,1);
            //lcd_putsf(menu);
            print(0,0,(int)time[0],(int)time[1],(int)time[2]);
            print(1,0,(int)Tmp[0],(int)Tmp[1],(int)Tmp[2]);
		}

		switch (Key_Pressed()) {
			case Up:
			case Down:
				show=1;
				if (Tmp[i]==0){
                    Tmp[i]=1;
				}
                else if (Tmp[i]==1){
                    Tmp[i]=0;
				}
				break;
			case Esc:
				return 0;
				break;
			case Ok:
				i++;
				if(i==3){
					Task[n]=Task[n]+((Tmp[0]+Tmp[1]+Tmp[2])<<5);
				    lcd_clear();
					lcd_gotoxy(1,1);
				    lcd_putsf(menu[saveitem]);
				    BKP_WriteBackupRegister(BKPDataReg[n], Task[n]);
				}
				break;
			default:
				break;
		}
	}
	return 0;
}

uint32_t Time_Set(void)
{
// uint32_t Tmp_HH = 0xFF, Tmp_MM = 0xFF, Tmp_SS = 0xFF;
	time[0]=THH;
	time[1]=TMM;
	time[2]=TSS;
	int i=0 ;
	char show=1;
	while(i<4&&Lcd_On){
		if(show){
			show=0;
			lcd_clear();
			lcd_gotoxy(1,1);
			print(0,0,(int)time[0], (int)time[1],(int)time[2]);
		}
		switch (Key_Pressed()) {
			case Up:
				show=1;
				time[i]++;
				if (i==0){
					if (time[i]>23)
						time[i]=0;
				}
				else if(time[i]>59){
					time[i]=0;
				}
				break;
			case Down:
				show=1;
				time[i]--;
				if (i==0){
					if (time[i]<0)
						time[i]=23;
				}
				else if(time[i]<0){
					time[i]=59;
				}
				break;
			case Esc:
                i--;
                if(i<0)
                    return 0;
				break;
			case Ok:
				i++;
				if(i==4){
					THH=time[0];
					TMM=time[1];
					TSS=time[2];
					RTC_WaitForLastTask();

				    RTC_SetCounter(time[0]*3600+time[1]*60+time[2]);
				    /* Wait until last write operation on RTC registers has finished */
				    RTC_WaitForLastTask();
				    lcd_clear();
					lcd_gotoxy(1,1);
					lcd_putsf(menu[saveitem]);
				    return 1;
				}
				break;
			default:
				break;
		}
	}


    /* //printf("\r\n==============Time Settings=====================================");
    //printf("\r\n  Please Set Hours");

    while (Tmp_HH == 0xFF)
    {
        //Tmp_HH = USART_Scanf(23);
    }
    //printf(":  %d", Tmp_HH);
    //printf("\r\n  Please Set Minutes");
    while (Tmp_MM == 0xFF)
    {
    }
    //printf(":  %d", Tmp_MM);
   // printf("\r\n  Please Set Seconds");
    while (Tmp_SS == 0xFF)
    {
    }
    //printf(":  %d", Tmp_SS);

    // Return the value to store in RTC counter register
    return((Tmp_HH*3600 + Tmp_MM*60 + Tmp_SS));
    */
	return 0;
}

/**
  * @brief  Displays the current time.
  * @param  TimeVar: RTC counter value.
  * @retval None
  */
char *Time_Display(uint32_t TimeVar)
{
    /* Compute  hours */
    THH = TimeVar / 3600;
    /* Compute minutes */
    TMM = (TimeVar % 3600) / 60;
    /* Compute seconds */
    TSS = (TimeVar % 3600) % 60;
    //print(0,0, (int)THH, (int)TMM,(int)TSS);
    sprintf(Time,"%2d:%2d:%2d",(int)THH, (int)TMM,(int)TSS);
    return Time;
}

/**
  * @brief  Shows the current time (HH:MM:SS) on the Hyperterminal.
  * @param  None
  * @retval None
  */
void Time_Show(void)
{
        /* Reset RTC Counter when Time is 23:59:59 */
        if (RTC_GetCounter() == 0x0001517F)
        {
            RTC_SetCounter(0x0);
            /* Wait until last write operation on RTC registers has finished */
            RTC_WaitForLastTask();
            daily=1;
        }
        /* If 1s has been elapased */
        if (TimeDisplay == 1)
        {
            /* Display current time */
            lcd_clear();
			lcd_gotoxy(1,1);
            delay_us(100);
			lcd_gotoxy(1,5);
			lcd_putsf(Time_Display(RTC_GetCounter()));
            TimeDisplay = 0;
        }
}
void Temp_Control(void){
    while(DS18B20_Init());
    while(!Lcd_On ){
        Time_Show();
        lcd_gotoxy(2,2);
        lcd_puts((int)DS18B20_Get_Temp);
        char t=10;
        if((int)DS18B20_Get_Temp>50){
            GPIO_SetBits(GPIOB,Pump);
            while(t){
                delay_ms(60000);
                t--;
            }
            GPIO_SetBits(GPIOB,Fan);
        }
        else if((int)DS18B20_Get_Temp < 45){
            GPIO_ResetBits(GPIOB,Fan);
            while(t){
                delay_ms(60000);
                t--;
            }
            GPIO_ResetBits(GPIOB,Pump);
        }
    }
}
void Time_Control(void){
    while(!Lcd_On ){
		if(daily&&TimeDisplay){
			if(((Task[num]&Time_Mask)<RTC_GetCounter())&&((Task[num]&Time_Mask)<(RTC_GetCounter()+60))){
				if(Task[num]&&Pump_Mask)
					GPIO_SetBits(GPIOB,Pump);
				else
					GPIO_ResetBits(GPIOB,Pump);
				if(Task[num]&&Fan_Mask)
					GPIO_SetBits(GPIOB,Fan);
				else
					GPIO_ResetBits(GPIOB,Fan);
				if(Task[num]&&Lamp_Mask)
					GPIO_SetBits(GPIOB,Lamp);
				else
					GPIO_ResetBits(GPIOB,Lamp);
				num++;
				if(num>20)num=1;
			}
			else if((Task[num]&Time_Mask)<RTC_GetCounter())
				while((Task[num]&Time_Mask)>(RTC_GetCounter())&&daily){
					num++;
					if(num==21){
						daily=0;
						num=1;
					}
				}
        Time_Show();
			}
    }
	return;
}
void Show_Task(void){
    lcd_clear();
	lcd_gotoxy(1,1);
    lcd_cursor_home();
    char Tmp[32],show=1;
    int j=0;
	while(Lcd_On&& j<(endTask+1)){
		if(show){
			Time_Display((uint32_t)(Task[j]&Time_Mask));
            print(1,j,(int)(Task[j]&Pump_Mask)&1,(int)(Task[j]&Fan_Mask)&2,(int)(Task[j]&Lamp_Mask)&4);
            lcd_putsf(Tmp);
            //lcd_gotoxy(0,1);
			show=0;
		}
		switch (Key_Pressed()) {
			case Up:
				show=1;
				j--;
				if (j==0){
                    j=endTask;
				}
				break;
			case Down:
				show=1;
				j++;
				if (j==endTask){
                    j=0;
				}
				break;
            case Esc:
				return ;
				break;
			case Ok:
                View_Task(j);
				break;
			default:
				break;
		}
	}

}
void View_Task(int i){
    char show=1;
	while(Lcd_On){
		if(show){
            lcd_clear();
			lcd_gotoxy(1,1);
            lcd_putsf("/\\:modify Task");
            lcd_gotoxy(1,2);
            lcd_putsf("\\/:delet Task");
        }
        switch(Key_Pressed()) {
            case Up:
                show=1;
                Modify_Task(i);
                break;
            case Down:
                show=1;
                lcd_clear();
				lcd_gotoxy(1,1);
                lcd_putsf("Task Delet...");
                Task[i]=0;
                break;
            default:
                return;
                break;
        }
    }
}
void print(int i,int j,int a,int b,int c){
	char tmp[16];
	if (!i){
		lcd_clear();
		delay_ms(100);
		lcd_gotoxy(1,1);
		lcd_putsf("Set Time:");
		delay_ms(100);
		//LCDGotoXY(2,7);
		lcd_gotoxy(1,4);
		sprintf(tmp,"%2d:%2d:%2d",a,b,c);
		lcd_putsf(tmp);
	}
	else if (i){
		lcd_clear();
		delay_ms(100);
		lcd_gotoxy(1,1);
		sprintf(tmp,"Task(%2d):",j);
		lcd_putsf(tmp);
		lcd_gotoxy(1,7);
		sprintf(tmp,"p:%1d*f:%1d*l:%1d",a,b,c);
		lcd_putsf(tmp);
	}
}
/**
  * @brief  This function handles RTC global interrupt request.
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{
    if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
    {
        /* Clear the RTC Second interrupt */
        RTC_ClearITPendingBit(RTC_IT_SEC);


        /* Toggle LED1 */
        //STM_EVAL_LEDToggle(LED1);
        //GPIOB->ODR ^= GPIO_Pin_8;

        /* Enable time update */
        TimeDisplay = 1;
        wait++;
        if (wait>120){
        	wait=0;
			Lcd_On=0;
        }

        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
    }
}
void RTCAlarm_IRQHandler(void){
    if (RTC_GetITStatus( RTC_FLAG_ALR)!= RESET){
		GPIO_WriteBit(LCD_PORT, Bl, Bit_SET);
		delay_ms(500);
		GPIO_WriteBit(LCD_PORT, Bl, Bit_RESET);
		delay_ms(1500);
    	RTC_ClearITPendingBit(RTC_IT_ALR);

    }

	return;
}
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)

    {
        delay_us(10);
        GPIO_WriteBit(LCD_PORT, Bl, Bit_SET);
        Lcd_On=1;

        //
        //Clear the EXTI line 0 pending bit
        //
        EXTI_ClearITPendingBit(EXTI0_IRQn);
    }
    EXTI_DeInit();
}
/**
  * @brief  Gets numeric values from the hyperterminal.
  * @param  None
  * @retval None
  */

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif
