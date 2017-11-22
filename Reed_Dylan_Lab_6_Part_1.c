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
#include <math.h>
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
char *x;
float temp;
float volt;
int type;
int num;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts
void HiPriISR(void);
void LoPriISR(void);
void TMR0handler(void);     // Interrupt handler for TMR0, typo in main
void disptemp(void);    // function for display the temperature
void disppt(void);      // function for displaying the potentiometer

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
        }// Sit here for ever
     
      }
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
    TRISA = 0b000001001;
    LATA = 0;
    TRISD  = 0b00001111;
    LATD = 0;
    TRISC  = 0b10010011;
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
    T0CON = 0b01001000;             // 8-bit, Fosc / 4, no pre/post scale timer
    TMR0L = 0;                      // Clearing TMR0 registers
    TMR0H = 0;

    // Configure the A/D for thermometer
    ADCON0 = 0b00001101;
    ADCON1 = 0b00000000;
    ADCON2 = 0b10010101;
    
    // Configuring Interrupts
    RCONbits.IPEN = 1;              // Enable priority levels
    INTCON2bits.TMR0IP = 0;         // Assign low priority to TMR0 interrupt

    INTCONbits.TMR0IE = 1;          // Enable TMR0 interrupts
    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts

    T0CONbits.TMR0ON = 1;           // Turning on TMR0
    
    //start adc for thermometer.
    ADCON0bits.GO=1;
    
    //initialize Eusart
    SPEN.RCSTA1=1;
    
    //initialize random numbers
    num=0;
    type=0;
}

/*******************************************************************************
 * disptemp
 * subroutine to display the temperature
 */
void disptemp(){
        temp=ADRES*0.806*10; //read in thermometer and convert.
        if(temp>999){//check to see if 1st digit exists
            itoa(temp,x);
            LCDRow1v[1]=*x;//displays 1st digit
            //calculate 2nd digit if 1st existed
            if (temp<2000)
            {
                temp=temp-1000;
            } else if (temp<3000)
            {
                temp=temp-2000;
            }else if (temp<4000)
            {
                temp=temp-3000;
            }else if (temp<5000)
            {
                temp=temp-4000;
            }else if (temp<6000)
            {
                temp=temp-5000;
            }else if (temp<7000)
            {
                temp=temp-6000;
            }else if (temp<8000)
            {
                temp=temp-7000;
            }else if (temp<9000)
            {
                temp=temp-8000;
            }else if (temp<10000){
                temp=temp-9000;
            }
            if(temp>99){//check to see if 2nd digit exists(in case where 1st digit existed)
                itoa(temp,x);
                LCDRow1v[2]=*x;
            }else if(temp>9.99){ //sets value to 0 if second digit exists
                LCDRow1v[2]='0';
            }
        }else if(temp>99){ // checks to see if 2nd digit exists
            LCDRow1v[1]='0';
            itoa(temp,x);
            LCDRow1v[2]=*x;    
        }else{ //sets everything to 0, if the values are 0
           LCDRow1v[1]='0';
            LCDRow1v[3]='0'; 
        }
        if(temp>99){ //check to see if 2nd digit existed, so we can display 3rd digit
            //calculate 3rd digit
            if (temp<200)
            {
                temp=temp-100;
            } else if (temp<300)
            {
                temp=temp-200;
            }else if (temp<400)
            {
                temp=temp-300;
            }else if (temp<500)
            {
                temp=temp-400;
            }else if (temp<600)
            {
                temp=temp-500;
            }else if (temp<700)
            {
                temp=temp-600;
            }else if (temp<800)
            {
                temp=temp-700;
            }else if (temp<900)
            {
                temp=temp-800;
            }else if (temp<1000){
                temp=temp-900;
            }
            itoa(temp,x);
            LCDRow1v[4]=*x;
        }else if(temp>9.99){//check to see if third digit exists, and outputs if it does(case where 1st digit exists, but not hte second)
            itoa(temp,x);
            LCDRow1v[4]=*x;
        }else //sets value to zero if 3rd digit is to small to exist
        {
            LCDRow1v[4]='0';
        }
        DisplayV(LCDRow1v); //disp to lcd
    
    
}

/*******************************************************************************
 * disppt
 * subroutine to display the potentiometer.
 */
void disppt( ){
        volt=ADRES*0.806; //read in thermometer and convert.
        if(volt>1000){ //check to see if value exists in the ones place.
            itoa(volt,x); //if it does, export the value to lcd
            LCDRow2v[1]=*x;
            //find what value is, so we can display the next digit
            if (volt<2000)
            {
                volt=volt-1000;
            } else if (volt<3000)
            {
                volt=volt-2000;
            }else if (volt<4000)
            {
                volt=volt-3000;
            }else if (volt<5000)
            {
                volt=volt-4000;
            }else if (volt<6000)
            {
                volt=volt-5000;
            }else if (volt<7000)
            {
                volt=volt-6000;
            }else if (volt<8000)
            {
                volt=volt-7000;
            }else if (volt<9000)
            {
                volt=volt-8000;
            }else if (volt<10000){
                volt=volt-9000;
            }
            //check to see if the 2nd digit exists after subtraction
            if(volt>99){
                itoa(volt,x);
                LCDRow2v[3]=*x;
            }else if (volt>9.99){
                LCDRow2v[3]='0';
            }
        }else if(volt>99){//check to see if 2nd digit exists(assuming that first digit didnt)
            LCDRow2v[1]='0';
            itoa(volt,x);
            LCDRow2v[3]=*x;
        }else{ //case if onle thrid digit exits
           LCDRow2v[1]='0';
            LCDRow2v[3]='0'; 
        }
        if(volt>99){//check to see if 3rd digit exists
            //calculate 3rd digit
            if (volt<200)
            {
                volt=volt-100;
            } else if (volt<300)
            {
                volt=volt-200;
            }else if (volt<400)
            {
                volt=volt-300;
            }else if (volt<500)
            {
                volt=volt-400;
            }else if (volt<600)
            {
                volt=volt-500;
            }else if (volt<700)
            {
                volt=volt-600;
            }else if (volt<800)
            {
                volt=volt-700;
            }else if (volt<900)
            {
                volt=volt-800;
            }else if (volt<1000){
                volt=volt-900;
            }
            itoa(volt,x);
            LCDRow2v[4]=*x;//disp value to lcd
        }else if(volt>9.99){//check to see if 3rd digit is nonzero(in case where 2nd digit didnt exist) 
            itoa(volt,x);
            LCDRow2v[4]=*x;
        }else // set value to zero if it is less than 0.01
        {
            LCDRow2v[4]='0';
        }
        DisplayV(LCDRow2v); //disp to lcd
    
    
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
    if( Alive_count < 4880 ) { Alive_count++; }
    else {
        LATDbits.LATD4 = ~LATDbits.LATD4;
        Alive_count = 0;
    }
    INTCONbits.TMR0IF = 0;      //Clear flag and return to polling routine
}
