/******************************************************************************
 *
 * Author: Gabriel LoDolce
 * Author of last change: Trudy Schwartz
 * Date of last change: 5/16/2017
 * Revision: 1.0
 *
 *******************************************************************************
 *
 * FileName:        LCDroutinesEasyPic.h
 * Dependencies:    Delays.h and p18cxxx.h
 * Processor:       PIC18F
 * Compiler:        C18
 *
 *******************************************************************************
 * File Description: This library contains a set of functions for the 
 ******************************************************************************/

#include <p18cxxx.h>

#ifndef _LCD_ROUTINES_B_
#define _LCD_ROUTINES_B_

/*------------------------------------------------------------------------------
 * Definitions for this LCD interface for EasyPic Pro v7
 -----------------------------------------------------------------------------*/

/* RS Pin Assignments */
#define LCD_RS_TRIS     TRISBbits.TRISB4
#define LCD_RS_LAT      LATBbits.LATB4

/* E Pin Assignments */
#define LCD_E_TRIS	TRISBbits.TRISB5
#define LCD_E_LAT	LATBbits.LATB5

/* Data Pin Assignments. Note we only need the upper nibble
** but it is hard to break apart by nibbles. We have to break apart for new LCD since RS and E
** are also on PORTB!!
*/
#define LCD_DATA_TRIS TRISB
#define LCD_DATA_LAT LATB
#define LCD_DATA_PORT PORTB

/*------------------------------------------------------------------------------
 * Public Library Functions
 -----------------------------------------------------------------------------*/

/******************************************************************************
 *     Function Name:	InitLCD
 *     Parameters:      None
 *     Description:		This function initializes the Optrex 8x2 character LCD.
 *						This function generates a 0.1s and 0.01s delay using the
 *						c18 Delay library
 *
 ******************************************************************************/
void InitLCD( void );

/******************************************************************************
 *     Function Name:	DisplayC
 *     Parameters:      Pointer to a character array in program memory
 *     Description:		This function sends a character array in program memory
 *						to the LCD display. Note the first character of the
 *						string is the positioning commmand. The string must
 *						also be terminated by the null character
 *
 *						This function generates a 40us delay using the c18
 *						Delay library
 *
 ******************************************************************************/
void DisplayC( const rom far char * LCDStr );

/******************************************************************************
 *     Function Name:	DisplayV
 *     Parameters:      Pointer to a character array in data memory
 *     Description:		This function sends a character array in data memory
 *						to the LCD display. Note the first character of the
 *						string is the positioning command. The string must
 *						also be terminated by the null character
 *
 *						This function generates a 40us delay using the c18
 *						Delay library
 *
 ******************************************************************************/
void DisplayV( const char * LCDStr );

#endif


