/****** ASEN 4/5519 Lab 6 ******************************************************
 * Author: Dylan Reed
 * Date  : 11/8/17
 *
 *
 * Description
 * On power up exceute the following sequence:
 *      RD5 ON for 0.5s +/- 10ms then off
 *      RD6 ON for 0.5s +/- 10ms then off
 *      RD7 ON for 0.5s +/- 10ms then off
 * The following then occurs forever:
 *      RD4 blinks: 100ms +/- 10ms ON, then 900ms +/- 10ms OFF
 *      LCD Displays the following lines:
 *          'T=xx.x C'
 *          'PT=x.xxV'
 *      Where the 'x' is replaced by a digit in the measurement.
 *          Temperature data must be calculated / displayed with one digit to
 *          the right of the decimal as shown.  The sensor itself can have
 *          errors up to +/- 5 degrees Celsius.
 *          Potentiometer data must be calculated / displayed with two digits
 *          to the right of the decimal as shown.
 *          These measurements must be refreshed at LEAST at a frequency of 5Hz.
 *      USART Commands are read / executed properly. '\n' is a Line Feed char (0x0A)
 *          ASEN 4519:
 *              'TEMP\n'     - Transmits temperature data in format: 'XX.XC'
 *              'POT\n'      - Transmits potentiometer data in format: X.XXV'
 *          ASEN 5519: Same as ASEN 4519, plus two additional commands
 *              'CONT_ON\n'  - Begins continuous transmission of data over USART
 *              'CONT_OFF\n' - Ends continuous transmission of data over USART
 *
 *              Continuous transmission should output in the following format:
 *                  'T=XX.XC; PT = X.XXV\n'
 *      DAC is used to output analog signal onto RA5 with jumper cables. 
 *          ASEN 4519:
 *              Potentiometer voltage is converted from a digital value to analog 
 *              and output on the DAC. 
 *          ASEN 5519: 
 *              A 0.5 Hz 0-3.3V triangle wave is output on the DAC. 
 *******************************************************************************
 *
 * Program hierarchy 
 *
 * Mainline
 *   Initial
 *
 * HiPriISR (included just to show structure)
 *
 * LoPriISR
 *   TMR0handler
 ******************************************************************************/

 
#include <p18cxxx.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <timers.h>
#include <delays.h>
#include <spi.h>
#include "LCDroutinesEasyPic.h"
#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF

/******************************************************************************
 * Global variables
 ******************************************************************************/
const rom far char LCDRow1[] = {0x80,'T','=',0x00};
const rom far char LCDRow2[] = {0xC0,'P','T','=',0x00};
char LCDRow1v[] = {0x82,'0','0','.','0','C',0x00};
char LCDRow2v[] = {0xC3,'0','.','0','0','V',0x00};
unsigned int Alive_count = 0;
unsigned int lightcnt = 0;
char x[3];
char y, dummy;
char dataH, dataL;
int temp, volt, volt2;
int type, num,j;
int i=0;
char r = '\r';
char Buffer[];

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts
void HiPriISR(void);
void LoPriISR(void);
void TMR0handler(void);     // Interrupt handler for TMR0, typo in main
void disptemp(void);    // function for display the temperature
void disppt(void);      // function for displaying the potentiometer
void wait500ms(void);   //function for a delay of 500ms
void outread(void);     //function that does the lcd output and reads in values from sensors
void Uartread(void);    //ouputs values vie uart
void uarthandler(void); //reads in usart values in interrupt
void spistuff(void);   // does spi stuff with dac

#pragma code highVector=0x08
void atHighVector(void)
{
 _asm GOTO HiPriISR _endasm
}
#pragma code

#pragma code lowVector=0x18
void atLowVector(void)
{
 _asm GOTO LoPriISR _endasm
}
#pragma code

/******************************************************************************
 * main()
 ******************************************************************************/
void main() {
     Initial();                 // Initialize everything
      while(1) 
      {
          //read and output stuff to lcd.
          outread();
          
          Delay100TCYx(10);
          //Output using USART
          if (j==1){
              Uartread();
              j=0;
          } 
          
          //Send to DAC using SPI
          spistuff();
          
      }// Sit here for ever
     
}
/******************************************************************************
 * Initial()
 *
 * This subroutine performs all initializations of variables and registers.
 * It enables TMR0 and sets CCP1 for compare, and enables LoPri interrupts for
 * both.
 ******************************************************************************/
void Initial() {
    // Configure the IO ports
    TRISA = 0b11101111;
    LATA = 0;
    TRISD  = 0b00001111;
    LATD = 0;
    TRISC  = 0b11111111;
    LATC = 0;
    
    // Configure the LCD pins for output. Defined in LCDRoutines.h
    LCD_RS_TRIS   = 0;              // 
    LCD_E_TRIS    = 0;
    LCD_DATA_TRIS = 0b11000000;     // Note the LCD is only on the upper nibble
                                    // The lower nibble is all inputs
    LCD_DATA_LAT = 0;           // Initialize LCD data LAT to zero

    // Initialize the LCD and print to it
    InitLCD();
    DisplayC(LCDRow1);
    DisplayC(LCDRow2);
    DisplayV(LCDRow1v);
    DisplayV(LCDRow2v);
    
    // Initializing TMR0
    T0CON = 0b00000100;             // 8-bit, Fosc / 4, no pre/post scale timer
    
    //Turn on LEDS.
    wait500ms();
    LATDbits.LATD5=~LATDbits.LATD5;
    wait500ms();
    LATDbits.LATD5=~LATDbits.LATD5;
    LATDbits.LATD6=~LATDbits.LATD6;
    wait500ms();
    LATDbits.LATD6=~LATDbits.LATD6;
    LATDbits.LATD7=~LATDbits.LATD7;
    wait500ms();
    LATDbits.LATD7=~LATDbits.LATD7;
    
    // Initializing TMR0
    T0CON = 0b01001000;             // 8-bit, Fosc / 4, no pre/post scale timer
    TMR0L = 0;                      // Clearing TMR0 registers
    TMR0H = 0;

    // Configure the A/D for thermometer
    ADCON0 = 0b00001101;
    ADCON1 = 0b00000000;
    ADCON2 = 0b10010101;
    
    // Configuring Interrupts
    RCONbits.IPEN = 1;              // Enable priority levels
    //CCP1CON = 0b01001011;           //setup capture mode 
    INTCON2bits.TMR0IP = 0;         // Assign low priority to TMR0 interrupt
    
    IPR1bits.RC1IP = 0;
    PIE1bits.RC1IE = 1;

    INTCONbits.TMR0IE = 1;          // Enable TMR0 interrupts
    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts

    T0CONbits.TMR0ON = 1;           // Turning on TMR0
    
    //start adc for thermometer.
    ADCON0bits.GO=1;
    
    //initialize Usart
    BAUDCON1=0b00000000;
    TXSTA1=0b00100100;
    RCSTA1=0b10010000;
    SPBRG1=51;
    
    //rcreg1=0;
    
    //initialize random numbers
    num=0;
    type=0;
    
    OpenSPI1(SPI_FOSC_4,MODE_01,SMPEND);

}

/*******************************************************************************
 *outread
 * subroutine that handles the reading in of sensors and potentiometer and outputs
 to lcd.*/
void outread(){
    if (ADCON0bits.GO==0) 
        {
            if (num==1){ // this loop throws away first value that is wrong.
                if (type==0){
                    disptemp();//display the temperature
                    // Configure the A/D for potentiometer
                    ADCON0 = 0b00000001;
                    ADCON1 = 0b00000000;
                    ADCON2 = 0b10010101;
                    type=1; //sets loop to display potentiometer next time
                }else{
                    disppt();//display the potentiometer
                    // Configure the A/D for thermometer
                    ADCON0 = 0b00001101;
                    ADCON1 = 0b00000000;
                    ADCON2 = 0b10010101;
                    type=0;//sets program to display temperature next time
                }
                num=0;
            }else{
                num++;
            }
            ADCON0bits.GO=1;
            
            
        }
}

/*******************************************************************************
 *wait500ms
 *subroutine to wait 5000ms*/

void wait500ms(){
        WriteTimer0(65536-62500);//delay for total loop time of 1 ms
		T0CONbits.TMR0ON = 1;           // Turning on TMR0
        while(ReadTimer0()>0){
        }
        INTCONbits.TMR0IF = 0;      //Clear flag and return to polling routine
    
}

/*******************************************************************************
 * disptemp
 * subroutine to display the temperature
 */
void disptemp(){
        temp=ADRES*0.806; //read in thermometer and convert.
        itoa(temp,x);
        if (temp>100){
            LCDRow1v[1]=x[0];
            LCDRow1v[2]=x[1];
            LCDRow1v[4]=x[2];
        }else if (temp>10){
            LCDRow1v[1]='0';
            LCDRow1v[2]=x[0];
            LCDRow1v[4]=x[1];
        }else{
            LCDRow1v[1]='0';
            LCDRow1v[2]='0';
            LCDRow1v[4]=x[1];
        }
        DisplayV(LCDRow1v); //disp to lcd
    
    
}

/*******************************************************************************
 * disppt
 * subroutine to display the potentiometer.
 */
void disppt( ){
        volt=ADRES*0.806; //read in thermometer and convert.
        itoa(volt,x);
        if (volt>1000){
            LCDRow2v[1]=x[0];
            LCDRow2v[3]=x[1];
            LCDRow2v[4]=x[2];
        }
        else if(volt>100){
            LCDRow2v[1]='0';
            LCDRow2v[3]=x[0];
            LCDRow2v[4]=x[1];
        }else {
            LCDRow2v[1]='0';
            LCDRow2v[3]='0';
            LCDRow2v[4]=x[0];
        }
        DisplayV(LCDRow2v); //disp to lcd
    
}
//Function to output values from board via uart.
void Uartread(void){
        if ( PIR1bits.TXIF != 0 ) {   // do 
            char TEMP[]={'T','E','M','P','\r'};
            char VOLT[]={'P','O','T','\r'};
            if (memcmp(&Buffer,&TEMP,5)==0){
                itoa(temp*9/5+320,x);
                if(temp>100){
                    TXREG1 = x[0];
                } else{
                    TXREG1='0';
                }
                Delay10TCYx(1000);
                if(temp>10){
                    TXREG1 = x[1];
                } else{
                    TXREG1='0';
                }
                Delay10TCYx(1000);
                TXREG1='.';
                Delay10TCYx(1000);
                TXREG=x[2];
                Delay10TCYx(1000);
                TXREG='F';
                Buffer[4]='n'; //stops code from outputting value every time key is pressed
            }else if(memcmp(&Buffer,&VOLT,4)==0){
                itoa(volt,x);
                if(volt>1000){
                    TXREG1 = x[0];
                } else{
                    TXREG1='0';
                }
                Delay10TCYx(1000);
                TXREG1='.';
                Delay10TCYx(1000);
                if(volt>100){
                    TXREG1 = x[1];
                } else{
                    TXREG1 = '0';
                }
                Delay10TCYx(1000);
                TXREG=x[2];
                Delay10TCYx(1000);
                TXREG='V';
                Buffer[3]='n'; //stops code from outputting value every time key is pressed
            }
            Delay10TCYx(1000);
            TXREG=0b00001101;
        }
}


//subroutine to handle spi output to dac
void spistuff(void){
    itoa(volt,dataL);
    volt2=volt;
    volt2 >> 8;   //get hi bit            
    itoa(volt2,dataH);         
    dataH | 0b00110000;   //add 0011
    dataH & 0b00111111;   // add amplification.  
    
    WriteSPI1(dataL);               
    while (!DataRdySPI1());           
    ReadSPI1();                  
    
    WriteSPI1(dataH);        
    while (!DataRdySPI1());     
    ReadSPI1();                 
}
/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * Included to show form, does nothing
 ******************************************************************************/
#pragma interrupt HiPriISR
void HiPriISR() {
}	// Supports retfie FAST automatically

/******************************************************************************
 * LoPriISR interrupt service routine
 *
 * Calls the individual interrupt routines. It sits in a loop calling the required
 * handler functions until until TMR0IF and CCP1IF are clear.
 ******************************************************************************/
#pragma interruptlow LoPriISR nosave=TBLPTR, TBLPTRU, TABLAT, PCLATH, PCLATU, PROD, section("MATH_DATA")//, section(".tmpdata")
void LoPriISR() {
    while(1) {
        if( INTCONbits.TMR0IF ) {
            TMR0handler();
            continue;
        }
        if (PIR1bits.RC1IF){
            uarthandler();   
        }
        break;
    }
}

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * Change the temporary data section for all code following this #pragma
 * statement. Normally the compiler/linker uses .tmpdata section for all non
 * ISR code and a separate tempdata section for each ISR (Hi and Lo). However,
 * when your primary ISR calls other functions the called functions share the
 * .tmpdata section with the main code. This can cause issues, just like not
 * saving WREG, STATUS and BSR. There are two options:
 *
 *   1. have the ISR save the .tmpdata section. This may have large effects on
 *      latency
 *   2. Force the functions called by the ISR to have their own temp data
 *      section.
 *
 * We are using option 2. However, this means that no main code functions can
 * appear after the following pragma, unless you change the temp data section
 * back.
 *
 * The temp data section is used for complex math operations such as 
 * a = b*c + d*e so you may not need a temp data section depending on what
 * you are doing.
 *
 * Keep in mind you may have the same issue with the MATH_DATA section.
 * MATH_DATA is used for arguments, return values and temporary locations
 * for math library functions.
 *!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
#pragma tmpdata handler_temp

/******************************************************************************
 * TMR0handler interrupt service routine.
 *
 * Handles Alive LED Blinking via counter
 ******************************************************************************/
void TMR0handler() {
    if( Alive_count < 1570 ) { Alive_count++; }
    else {
        if(lightcnt==8){
            LATDbits.LATD4 = ~LATDbits.LATD4;
            lightcnt=50;
        } else if(lightcnt==50){
            LATDbits.LATD4 = ~LATDbits.LATD4;
            lightcnt=0;
        } else{
            lightcnt++;
        }
        Alive_count = 0;
    }
    INTCONbits.TMR0IF = 0;      //Clear flag and return to polling routine
}

void uarthandler(){
    if (LATDbits.LATD7==1){
        LATDbits.LATD7=0;
    } else{
        LATDbits.LATD7=1;
    }
        if( RCSTA1bits.FERR == 1 ) {
            temp = RCREG1;    // Clear out RCREG1 to temp register & ignore       
        return;}
        if( RCSTA1bits.OERR == 1) {
            RCSTA1bits.CREN = 0;
            RCSTA1bits.CREN = 1;// Reset CREN will reset USART1
        return;}
        Buffer[i] = RCREG1;// Move RCREG1 data into an array
        if (i < 4){
            i++;
        }
        else {
            i=0;
        }
        if (memcmp(RCREG1,r,1)==0){ //changes the index to zero if enter is pressed
            i=0;
        }
        j=1;
        PIR1bits.RC1IF = 0;
}
