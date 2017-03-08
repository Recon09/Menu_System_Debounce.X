/***************************************************
Name:- Tommy Gartlan
Date last modified:- Jan 2016
Updated: Update to use xc8 compiler within Mplabx

Filename:- st_mch3.c
Program Description:- Using a simple state machine
to loop through a menu system.
 
Rule of thumb: Always read inputs from PORTx and write outputs to LATx. 
  If you need to read what you set an output to, read LATx.
 * 
 * This version combines st_mch3 and debounce libraary so that buttons are debounced
 * properly for the menu system
 * LCD on PORTD
 * Buttons on PORTB
 * LEDS on PORTC
****************************************************/

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
#define _XTAL_FREQ 19660800



/************************************************
			Function Prototypes
*************************************************/
void Initial(void);
void delay_s(unsigned char secs);

/************************************************
			Interrupt Function
*************************************************/
unsigned char count_test =0;
void __interrupt myIsr(void)
{
    //Timer overflows every 10mS
    // only process timer-triggered interrupts
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF) {
        
        Find_Button_Press();       //check the buttons every 10mS
        WriteTimer0(40960); 
        INTCONbits.TMR0IF = 0;  // clear this interrupt condition
        
        count_test++;
        if(count_test == 100){
            PORTCbits.RC7 = ~PORTCbits.RC7;   //check the timer overflow rate
            count_test = 0;                   //Toggle every 1 second (heartbeat))
        }
    }
}
/************************************************
			Global Variables
*************************************************/

const unsigned char * msg1 = "Top Menu 1";     //puts into ROM during download
const unsigned char * msg2 = "Top Menu 2";
const unsigned char * msg3 = "Top Menu 3"; //Puts into RAM during startup of program
											//CompGblVar62  is location in RAM. gbl_msg3 holds address of this location
const unsigned char * msg4 = "Top Menu 4";
const unsigned char * problem = "Problem";

const unsigned char * startup = "Ready to go";  //USes up RAM. Also initialised during startup as opposed to download. 
										  //However faster during runtime since don't have to copy to RAM first.
										  
unsigned char test[10] = "help";		//string stored in RAM !!!
unsigned char mess[20];					//uses up RAM
//unsigned char *mess;                  //could have done this either but need to investigate how and when linker decides to allocate RAM
                                         //looks like memory allocated during runtime as linker report shows smaller Heap for this option


Bit_Mask Button_Press;	

/************************************************
			Macros
*************************************************/
#define PUSH1_E Button_Press.B0

/*****************************************
 			Main Function
******************************************/

void main ( void ) 
{
    
    typedef  enum {MENU_1 = 0,MENU_2,MENU_3, MENU_4} states;
    states  my_mch_state = MENU_4;
    
    Initial();
    
    lcd_start () ;
    
    lcd_cursor ( 5, 0 ) ;
    
    lcd_print ( msg1 ) ;
    
    delay_s(2);
    
    while(1)
    {
		
		//wait for event
		while(!PUSH1_E);  
		
		//update the states
		my_mch_state = (my_mch_state+1)% 4;  
		
		//update the outputs
		lcd_clear();	//applies in all cases so can put it here
		switch(my_mch_state)	
		{
			case MENU_1: 
				lcd_cursor ( 5, 0 ) ;	
				lcd_print ( msg1);
                LATC = 0x01;
				
				break;
			case MENU_2: 
				lcd_cursor ( 5, 0 ) ;
				lcd_print ( msg2 );
                LATC= 0x02;
				break;
			case MENU_3: 
				lcd_cursor ( 5, 0 ) ;
				lcd_print ( msg3 );
                LATC = 0x03;
				break;
			case MENU_4: 
				lcd_cursor ( 5, 0 ) ;
				lcd_print ( msg4 );
                LATC = 0x04;
				break;
			default: 
				lcd_cursor ( 0, 0 ) ;
				lcd_print ( problem );
                LATC = 0x05;
				break;
		}
		
		//delay_s(1);		//again I shouldn't be really do delay
                        //it's for debounce
		//while((PUSH1_E));
        PUSH1_E = 0;   //clear the event
    
    }
}   


void Initial(void){
    
    ADCON1 = 0x0F;
	TRISB = 0xFF; //Buttons
	TRISC = 0x00;   //LEDS

	LATC = 0x00;
	delay_s(1);
	LATC = 0xff;
	delay_s(1);
	LATC = 0x00;
    
	for(unsigned int i =0; i<1000;i++)
	{
		LATC = PORTB;  //test switches here
		__delay_ms(10);	
	}

	LATC = 0xff;
	delay_s(1);
	LATC = 0x00;
	delay_s(1); 
    //0n, 16bit, internal clock(Fosc/4), prescale by 2)
    OpenTimer0( TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_2);
    WriteTimer0(40960);  //65,536 - 24,576  //overflows every 10mS
    ei();
    
}

void delay_s(unsigned char secs)
{
    unsigned char i,j;
    for(j=0;j<secs;j++)
    {
        for (i=0;i<25;i++)
                __delay_ms(40);  //max value is 40 since this depends on the _delay() function which has a max number of cycles
        
    }
}
