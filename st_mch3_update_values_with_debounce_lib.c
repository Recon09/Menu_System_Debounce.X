/***************************************************
Name:- Tommy Gartlan
Date last modified:- Jan 2016
Updated: Update to use xc8 compiler within Mplabx

Filename:- st_mch3_2_debounce2b.c
Program Description:- Using a simple state machine
to loop through a menu system.
 
Rule of thumb: Always read inputs from PORTx and write outputs to LATx. 
  If you need to read what you set an output to, read LATx.
 * 
 * This version combines st_mch3 and debounce2b so that buttons are debounced
 * properly for the menu system
 * 
 * this follow on version uses two buttons'UP and 'DOWN' and and an 'Enter, button to update values 
 * Good example of a menu system where values can be updated.
*****************************************************************************************************/

/****************************************************
		Libraries included
*****************************************************/
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../Buttons_Debounce_State_Mch/Buttons_Debounce.X/Buttons_Debounce.h"
#include "../../LCD_library/lcdlib_2016.h"

#include <plib/timers.h>
//#include <plib/usart.h>
//#include <plib/can2510.h>

/***********************************************
		Configuration Selection bits
************************************************/

/*************************************************
					Clock
*************************************************/
#define _XTAL_FREQ 8000000

/************************************************
			Global Variables
*************************************************/
const unsigned char msg_ary[10][16] = { "Desired> ","Actual> ", 
                                        "Pot_Value> ", "Desired> ",};

const unsigned char * problem = "Problem";

const unsigned char * startup = "Ready to go";
										  

/************************************************
			Function Prototypes
*************************************************/
void Initial(void);
void Window(unsigned char num);
void delay_s(unsigned char secs);
void Set(void);
void flash(void);
void Controller_Func(void);



/************************************************
 Interrupt Function 
*************************************************/
unsigned char count_test =0;
unsigned char TICK_E;
unsigned char counter=0;
unsigned char num;
unsigned int captured=0;

typedef struct{
        unsigned char Desired;
        unsigned char Actual;
    }Motor;
    
    Motor Motor1;

    typedef struct{
        float kp;               //Proportional Gain
        float ki;               //Integral Gain
        float kpki;             //kp+Ki
        signed char en;         //error
        signed char en_1;       //e[n]-1 previous error in 
        unsigned char un;       //u[n] speed is sent to CCPR2L
        unsigned char un_1;     //u[n]-1 previous speed
    }Controller;

    
 Controller control1;
 
void __interrupt myIsr(void)
{
    //Timer overflows every 10mS
    // only process timer-triggered interrupts
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF) {
        
        Find_Button_Press();       //check the buttons every 10mS
        WriteTimer0(45536); 
        INTCONbits.TMR0IF = 0;  // clear this interrupt condition
        
        //Heartbeat signal
        count_test++;
        if(count_test == 100){
            PORTEbits.RE1 = ~PORTEbits.RE1;   //check the timer overflow rate
            count_test = 0;                   //Toggle every 1 second (heartbeat))
        }
        
        counter++;
        if(counter >10)
        {
            Controller_Func();
            CCPR2L= control1.un;
            TICK_E= 1;
            counter=0;
        }
        
        if(PIR1bits.CCP1IF && PIE1bits.CCP1IE)   //Capture Flag Bit
      {
          
        PIR1bits.CCP1IF=0;      //clear flag bit
        captured=(CCPR1H<< 8);
        captured= captured+CCPR1L;
        
        TMR1H=0;                
        TMR1L=0;
      } 

    }
}


//declare Button
Bit_Mask Button_Press;	


/************************************************
			Macros
*************************************************/
#define MENU_E Button_Press.B0
#define ENTER_E Button_Press.B1
#define UP_E Button_Press.B2
#define DOWN_E Button_Press.B3
/*****************************************
 			Main Function
******************************************/





void main ( void ) 
{
    unsigned short long t1clk =250000;
    unsigned int test=0;

    unsigned char ADC_Result = 0;
    
    typedef  enum {MENU_0 = 0,MENU_1} states;
    states  my_mch_state = MENU_1;
    
    
    Motor1.Desired=0;
    Motor1.Actual=0;
    
    Initial();
    flash();
    Set();
    flash();
    lcd_start ();
    lcd_cursor ( 0, 0 ) ;
    lcd_print ( startup ) ;
    
    delay_s(2);
    //Initial LCD Display
    Window(0);

    
    while(1)
    {
		
		//wait for a button Event
		//while(!MENU_E && !ENTER_E && !UP_E && !DOWN_E);  
		while(!Got_Button_E && !TICK_E);
        
		switch(my_mch_state)	
		{
			case MENU_0: 
				if (MENU_E){
                    my_mch_state = MENU_1; //state transition        
                    Window(1);             //OnEntry action
                    PIR1bits.ADIF=0;
                    ADCON0bits.ADON=1;
                    
                }
                
				break;
			case MENU_1: 
				if (MENU_E){
                    my_mch_state = MENU_0;   //state transition
                    Window(0);              //OnEntry action
                    ADCON0bits.ADON=0;
                }
				break;


			default: 
				if (MENU_E){
                    my_mch_state = MENU_0;
                    Window(0);
                }
				break;
		}
		
		
		switch(my_mch_state)	
		{
			case MENU_0: 
				
                lcd_cursor(10,0);
                lcd_display_value(Motor1.Desired);
                test = captured;
                Motor1.Actual= t1clk/test;
                lcd_cursor(10, 1);
                lcd_display_value(Motor1.Actual);
                
                
                //LATC = 0x01;
				
				break;
                
                
                
			case MENU_1: 
                if (ENTER_E)          //state actions with guard
                {
                    Motor1.Desired=ADC_Result;
                }
                
                ADC_Result=ADRESH>>2;
                if (ADC_Result>51)
                {
                    ADC_Result=50;
                }
                ADCON0bits.GO_nDONE=1;
                
				lcd_cursor ( 10, 0 ) ;
                lcd_display_value(ADC_Result);
				lcd_cursor ( 10, 1 ) ;
                lcd_display_value(Motor1.Desired);
                //LATC= 0x02;
				break;

			default: 
				lcd_cursor ( 0, 0 ) ;
                lcd_clear();
				lcd_print ( problem );
                //LATC = 0x05;
				break;
		}
		
        Button_Press.Full = 0;  //Clear all events since only allowing one button event at a time
                                //which will be dealt with immediately
        TICK_E=0;
    }
}   


void Initial(void){
    ADCON0 = 0x01;
    ADCON1 = 0x0E;
    ADCON2 = 0x17;
    TRISA = 0x01;
	TRISB = 0xFF;        //Buttons
	TRISC = 0x00;        //Motor
    TRISE = 0x00;        //LED Block
    TRISCbits.RC2=1;     
    
    
    
	LATE = 0xff;
	delay_s(3);
	LATE = 0x00;
    
    //0n, 16bit, internal clock(Fosc/4), prescale by 2)
    OpenTimer0( TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_1);
    WriteTimer0(45536);  //65,536 - 24,576  //overflows every 10mS
    ei();

     //Setting UP PWM Signal
    T2CONbits.TMR2ON=1; //Timer 2 ON
    T2CONbits.T2OUTPS3=0;//prescale 1:1
    T2CONbits.T2OUTPS2=0;
    T2CONbits.T2OUTPS1=0;
    T2CONbits.T2OUTPS0=0;
    
    CCP2CONbits.CCP2M3=1;  // PWM mode 11xx;
    CCP2CONbits.CCP2M2=1;
    CCP2CONbits.CCP2M1=0;
    CCP2CONbits.CCP2M0=1;
    
    PR2=100;
    
    //Enabling CCP1
    CCP1CONbits.CCP1M3=0;//Capture Mode, Every rising edge
    CCP1CONbits.CCP1M2=1; 
    CCP1CONbits.CCP1M1=0; 
    CCP1CONbits.CCP1M0=1; 
    PIE1bits.CCP1IE=1;//Enables the CCP1 Interrupt
    
    
    //Enabling Timer 1
    T1CONbits.RD16=1;           //Enables Timer 1 in 16 bit operation
    T1CONbits.TMR1CS=0;         //Internal clock (Fosc/4)
    T1CONbits.TMR1ON=1;         //Enable Timer1
    T1CONbits.T1CKPS1=1;
    T1CONbits.T1CKPS0=1;
       
    control1.kp=0.4;
    control1.ki=0.3;
    control1.kpki=0.7;
    control1.en=0;
    control1.en_1=0;
    control1.un=30;
    control1.un_1=30;
    
    Motor1.Actual=0;
    Motor1.Desired=20;
}

void Set(void)
{
    OSCCONbits.IRCF2=1;         //8MHz configuration(IRCF2:IRCF0)
    OSCCONbits.IRCF1=1;
    OSCCONbits.IRCF0=1;
    OSCCONbits.SCS1=1;          //Internal Oscillator Block (SCS1:SCS0)
    OSCCONbits.SCS0=0;

}

void flash(void)
{
    PORTE = 0x01;           //Flash LED
    delay_s(5);             //Delay 5 seconds
    PORTE = 0x00;
    delay_s(5);
}

void Window(unsigned char num)
{
    lcd_clear();
    lcd_cursor ( 0, 0 ) ;	
	lcd_print ( msg_ary[num*2]);
    lcd_cursor ( 0, 1 ) ;
    lcd_print ( msg_ary[(num*2)+1]);
}


void delay_s(unsigned char secs)
{
    num=25;
    
    unsigned char i,j;
    for(j=0;j<secs;j++)
    {
        for (i=0;i<num;i++)
                __delay_ms(40);  //max value is 40 since this depends on the _delay() function which has a max number of cycles
        
    }
}

void Controller_Func(void)
{
    float temp=0;
    control1.en_1= control1.en;
    control1.en= Motor1.Desired-Motor1.Actual;
    control1.un_1= control1.un;
    
    temp=((control1.kpki)*(control1.en))-((control1.kp*control1.en_1));
    control1.un=control1.un_1+temp;
}