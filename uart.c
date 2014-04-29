/*
 * UART.c this file watch on the RX pin for DMX frames using UART's interrupt(RCIF)
 * The UART interrupt feed a buffer (RXBuffer)
 * The Main loop watch for new data available in RXBuffer and set the appropriate RGB values using
 * the hardware PWM CCP1,2,3.
 * Greetings : neomilium every advice he gave me (like using UART's interrupt)
*/


#define __16F737

#include <pic16f737.h>
#include <stdio.h>

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
/*
// CONFIG1
#pragma config FOSC = HS
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // MCLR/VPP/RE3 Pin Function Select bit (MCLR/VPP/RE3 pin function is MCLR)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (Enabled)
#pragma config BORV = 20        // Brown-out Reset Voltage bits (VBOR set to 2.0V)
#pragma config CCP2MX = RC1     // CCP2 Multiplex bit (CCP2 is on RC1)
#pragma config CP = OFF         // Flash Program Memory Code Protection bits (Code protection off)

// CONFIG2
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal External Switchover mode enabled)
#pragma config BORSEN = ON      // Brown-out Reset Software Enable bit (Enabled)
*/
__code uint8_t __at(_CONFIG1) cfg1 = ( _FOSC_HS  & _WDTE_OFF  & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _BOREN_ON & _BORV_20 & _CCP2MX_RC1 & _CP_OFF );
__code uint8_t __at(_CONFIG2) cfg2 = ( _FCMEN_ON & _IESO_ON & _BORSEN_ON );


#define PWM_ON 0b00001100

#define NUMBER_OF_CHANNELS 5   // R, G, B, Dimmer, Strobe   The Device we are programming is using 5 DMX channels
#define DMX_STATE_WAIT_BREAK 1 // Differents States for DMX reception
#define DMX_STATE_WAIT_START 2 //
#define DMX_STATE_GET_DATA 3   //

// Using Internal Clock of 8 Mhz
#define FOSC 8000000L
// Delay Function
#define _delay_us(x) { uint8_t us; \
us = (x)/(12000000/FOSC)|1; \
while(--us != 0) continue; }



// Global Variables Definition
uint8_t STATE = DMX_STATE_WAIT_BREAK;
uint8_t tmp, Red, Green, Blue;
uint16_t dmxFirstAddress = 1;
uint16_t dmxCurrentAddress;
uint16_t dmxLastAddress;
volatile uint8_t dmxNewData = 0; // When data is available, we set thie variable to 1 (could be a boolean but i havn't found a way use bool values under sdcc
volatile uint8_t RXbuffer[NUMBER_OF_CHANNELS]; // Buffer that will store the DMX Values used by our device




void init_clock();
void init_timer2();
void init_usart();
void update_pwm1(uint8_t );
void update_pwm2(uint8_t);
void update_pwm3(uint8_t );
void _delay_ms(uint16_t ms);






void main (void)
{
    
    init_clock();
    init_timer2();
    init_usart();
    TRISA = 0xFF;
    TRISB = 0xFF;
    NOT_RBPU = 0; //. port B internall pull-ups
    
    //PWM
    TRISC2 = 0; // Set CCP1 as Output   (PIN12)
    TRISC1 = 0; // Set CCP2 as Output  (PIN13)
    TRISB5 = 0; // Set CCP3 as Output  (PIN26)
    ADCON1 = 0x0F; // Disable Analog mode to enable reading values of the 8bit Dip Switch
    CVROE = 0; // need to check if necessary or can be removed
    
    // Read dmxAddress of the device from the 8bit Dip Switch
    dmxFirstAddress = (RB0  | (RB1 << 1) | (RB2 << 2) | (RB3 << 3) | (RB4 << 4) | (RA2 << 5) | (RB6 << 6) | (RB7 << 7)) ^ 0xFF;
    dmxLastAddress = dmxFirstAddress + NUMBER_OF_CHANNELS - 1;

    
    update_pwm1(0); //vert
    update_pwm2(0); //rouge 
    update_pwm3(0); // bleu
    
    while (1)
    {
        
        //getDMXTrame();
    
        if (dmxNewData)
        {
            Red = RXbuffer[1] * ((float)RXbuffer[3] / 255);
            Green = RXbuffer[0] * ((float)RXbuffer[3] / 255);
            Blue = RXbuffer[2] * ((float)RXbuffer[3] / 255);
            update_pwm1(Green);//vert
            update_pwm2(Red);//rouge
            update_pwm3(Blue);//bleu
            
            if (RXbuffer[4] > 20)
            {
                //delay ON
                _delay_ms(40);                
                update_pwm1(0);//rouge
                update_pwm2(0);//vert
                update_pwm3(0);//bleu
                //delay OFF
                _delay_ms((int)((float)RXbuffer[4] * 200 / 255));   
            }
            
            dmxNewData = 0;            
        }
        if (OERR)
        {
            CREN = 0;
            CREN = 1;
        }
    };

}

void init_clock()
{    // Configuration of the Internal Oscillator at 8MHz
    IRCF0 = 1;
    IRCF1 = 1;
    IRCF2 = 1;
}

static void isr(void) __interrupt 0 { // inspired from the APP NOTE from microchip available at : 
                                        // http://ww1.microchip.com/downloads/en/AppNotes/01076A.pdf
    if(RCIF){  // Data Available in UART's RCREG register
        switch (STATE)
        {
            case DMX_STATE_WAIT_BREAK : // we are waiting for the break sequence (which will generate framing errors)
                
                tmp = RCREG; // we discard data until a framing error occur
                if (FERR) { STATE = DMX_STATE_WAIT_START; } 
            break;
            case DMX_STATE_WAIT_START : // Now we know we are in the Break part, we need to wait 
                                        // until the line return th the idle state (mark after break)
                                        // we need to poll the uart until a valid frame is received
                tmp = RCREG; // we discard data until a no framing error occurs
                if(!FERR) 
                {
                    STATE = DMX_STATE_GET_DATA;
                    dmxCurrentAddress = 1;
                } 
            
            break;
            case DMX_STATE_GET_DATA :
                /*if (OERR) 
                {
                    error(1);
                    CREN = 0;
                    CREN = 1; // doing this will Clean the OERR bit
                }  
                */
                tmp = RCREG; // we get the dmx value
                if (dmxCurrentAddress >= dmxFirstAddress) 
                {   
                    RXbuffer[dmxCurrentAddress - dmxFirstAddress] = tmp; 
                }
                dmxCurrentAddress++;
                if (dmxCurrentAddress > dmxLastAddress) 
                {
                    STATE = DMX_STATE_WAIT_BREAK;
                    dmxNewData = 1; // Inform the Main Loop that there is data available in RXBuffer
                }
            break;
        }
    }
}






/*
  From  http://dics.voicecontrol.ro/tutorials/brgh/calculate.php
    BRGH = 1 - High Speed
    The calculated value for spbrg is : 1.000
    The value you will use for spbrg is : 1
    The calculated baudrate for this value is : 250000 bps
    The error is : 0.00%
*/
void init_usart()
{   
    TRISC7 = 1;// allow the EUSART RX to control pin RC7
    TRISC6 = 0 ;// allow the EUSART TX to control pin RC6
    // configuration de la vitesse de transmission
    // activation du mode Asynchrone en mettant SYNC = 0 et SPEN = 1 
    SYNC = 0;// = 0<< SYNC | 1 << BRGH;   // <<4 | <<2     VOIR PAGE 135
    BRGH = 1;
    SPBRG = 1;//

    // Interrupt Enable
    GIE = 1; // global interrupt
    PEIE = 1; // interrupt peripheral
    RCIE = 1;
    
    // UART : ON
    SPEN = 1;

    //activation de la reception
    CREN = 1;
    // si interruption desiree, RCIE = 1
    // RCIF indique qu'une reception est complete et une interruption sera générée si RCIE=1
}
void init_timer2()
{ // on configure pour avoir 256 etat possibles de duty cycle d'apres le site ci-dessous
    // calculator http://eng-serve.com/pic/pic_pwm.html
    T2CON = 0b00000100; // prescaler + turn on TMR2;
    PR2 = 0b00111111;
}
// Les 3 fonctions update_pwmX sont identiques mais travaillent sur les 3 modules CCP
void update_pwm1(uint8_t duty)
{
   // le duty doit être stocké dans 2 variables :
   // les 2 bits de poid faible (lsb) vont ici dans CCPxCON : 0b 00XX 1100    (ici 1100 indique que l'on configuer le CCPx en mode PWM)
   // les autres bits du duty (6 bits restants) sont stockés dans CCPRxL
   // on shift donc de 2 bit vers la droite le duty et on le stock dans CCPRxL
   // et on met dans CCP1CON les 2 bits de poid faible du duty cycle au rang 4 et 5 dans CCPxCON
   //    duty = 0bwxyzabcd en binaire
   //    on doit mettre CCPRxL = 00wxyzab
   //    et ensuite CCPxCON = 00cd1100
   CCPR1L = duty >> 2; // on décale dont de 2 bit duty pour avoir CCPRxL = 00wxyzab
   CCP1CON = PWM_ON | ((duty & (uint8_t)3) << 4); // et on conserve les 2 bits de poid faible en faisant un ET avec 3 (0b11)
                                                     // on décalle le tout de 4 vers la gauche pour obtenir  0b00cdxxxx (où xxxx est PWM_ON qui est égal à 0b1100)
}
void update_pwm2(uint8_t duty)
{
   CCPR2L = duty >> 2;;
   CCP2CON = PWM_ON | ((duty & (uint8_t)3) << 4);
}
void update_pwm3(uint8_t duty)
{
   CCPR3L = duty >> 2;;
   CCP3CON = PWM_ON | ((duty & (uint8_t)3) << 4);

}

void _delay_ms(uint16_t ms)
{
    uint8_t i;
    do {
        i = 4;
        do {
            //_delay_us(164);
            _delay_us(75);
        } while(--i);
    } while(--ms);
}


