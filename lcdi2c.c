/*	----------------------------------------------------------------------------
	File:		lcdi2c.c
	Purpose:		PIC18F Driver library for HD44780-compatible LCD displays
                    through I2C PCF8574 I/O expander.
	Author:		Jonatan Yam <jonatan.yam@gmail.com>.
                    Based on routines of lcdi2c.c/.h by Regis Blanchot.
     Version:       v1.0.0
	First release:	17/May/2018
	Last release:
     Note:          SDCC compiler v3.7.0 #10232 (GNU/Linux)
	----------------------------------------------------------------------------

     Copyright (C) 2018  Jonatan I. Yam Cabrera

     This program is free software: you can redistribute it and/or
     modify it under the terms of the GNU Lesser General Public License
     as published by the Free Software Foundation, either version 3
     of the License, or (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
     General Public License for more details.

     You should have received a copy of the GNU Lesser General Public License
     along with this program.  If not, see <https://www.gnu.org/licenses/>.
     --------------------------------------------------------------------------*/


/*----------------------    PCF8574 Pin relation    ----------------------------


          +5V       A0        -|o      |-        VDD     +5V
          +5V       A1        -|       |-        SDA
          +5V       A2        -|       |-        SCL
          LCD_BL    P0        -|       |-        INT
          LCD_RS    P1        -|       |-        P7      LCD_D7
          LCD_RW    P2        -|       |-        P6      LCD_D6
          LCD_EN    P3        -|       |-        P5      LCD_D5
          GRND      VSS       -|       |-        P4      LCD_D4

------------------------------------------------------------------------------*/

#ifndef __LCDI2C_C
#define __LCDI2C_C

#include <i2c.h>
#include <delay.h>
#include "lcdi2c.h"


/*******************************************************************************

          						Global variables

*******************************************************************************/

	__PCF8574Data_t PCF8574Data;
	unsigned char max_lcd_x;			// from 0 to 15 = 16
	unsigned char max_lcd_y;			// from 0 to 1  = 2
	unsigned char PCF8574_address;
	unsigned char lcd_backlight_status = 0;


/*******************************************************************************
*   Function Name:  lcdi2c_backlight                                           *
*   Return Value:   void                                                       *
*   Parameters:     @param light_status : bool variable to set de status of bl *
*                   light_status = 0 -> Turns OFF the backlight                *
*                   light_status = 1 -> Turns ON the backlight                 *
*   Description:    This routine affects de backliht cntrol bit BL, but sends  *
*                   the hole byte PCF8574Data.byte                             *
*   Note                                                                       *
*                                                                              *
*******************************************************************************/
#ifdef I2CLCD_BACKLIGHT
void lcdi2c_backlight(unsigned char light_status)
{
     lcd_backlight_status = light_status;
     PCF8574Data.BL = lcd_backlight_status;
	i2c_start();
	i2c_idle();
	i2c_writechar((PCF8574_address << 1) & 0xFE);
	i2c_writechar(PCF8574Data.byte);
	i2c_stop();
	i2c_idle();


}
#endif


/*******************************************************************************
*   Function Name:  lcdi2c_send_nibble                                         *
*   Return Value:   void                                                       *
*   Parameters:     @param nibble : Nibble to send to LCD                      *
*                   @param mode: Command or Data.                              *
*                   Macros for mode: LCD_CMD & LCD_DATA                        *
*   Description:    This routine sends a nibble/quartet to the Hitachi         *
*                   HD44780 LCD controller.                                    *
*  Note:            The nibble it's a byte, but we only use the four most      *
*                   significative bits.                                        *
*                                                                              *
*                   Nibble format:  // D7 D6 D5 D4 BL EN RW RS                 *
*******************************************************************************/


void lcdi2c_send_nibble(unsigned char high_nibble, unsigned char mode)
{


     PCF8574Data.byte = high_nibble;
	PCF8574Data.EN = LOW;
	PCF8574Data.RW = LCD_WRITE;
	PCF8574Data.RS = mode;
	PCF8574Data.BL = lcd_backlight_status;


     i2c_start();            // send start condition
     i2c_idle();             // waits for i2c becomes idle
     //the micro calls PCF8574 with Write mode
     i2c_writechar((PCF8574_address << 1) & 0xFE);
     PCF8574Data.EN = 1;
     i2c_writechar(PCF8574Data.byte);     //sends the data byte
     PCF8574Data.EN = 0;
     i2c_writechar(PCF8574Data.byte);     //sends the data byte
     i2c_stop();                          //send stop confition
     i2c_idle();
}



/*******************************************************************************
*   Function Name:  lcdi2c_send_byte                                           *
*   Return Value:   void                                                       *
*   Parameters:     @param byte : byte to send to LCD in 4bits mode            *
*                   @param mode: Command or Data.                              *
*                   Macros for mode: LCD_CMD & LCD_DATA                        *
*   Description:    This routine sends a byte to the Hitachi HD44780 LCD       *
*                   controller. The byte its transfered in two 4 bits transfers*
*                   Most sinificative nibble is transfered first.              *
*   Note:           The data is sended to the D7 ... D4 data pins of LCD.      *
*******************************************************************************/

void lcdi2c_send_byte(unsigned char byte, unsigned char mode)
{
	lcdi2c_send_nibble(byte & LCD_MASK, mode);			// send upper 4 bits
	lcdi2c_send_nibble((byte << 4) & LCD_MASK, mode);	// send lower 4 bits
	//Delayus(46);			 // Wait for instruction excution time (more than 46us)
}


/*******************************************************************************
*   Function Name:  lcdi2c_goto_xy                                             *
*   Return Value:   void                                                       *
*   Parameters:     @param x: x its equivalent to column                       *
*                   @param y: y its equivalent to lines                        *
*   Description:    This routine sets the cursos to specified X,Y coordinate   *
*   Note:                                                                      *
*******************************************************************************/


void lcdi2c_goto_xy(unsigned char x, unsigned char y)
{
     int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };

     if ( y > max_lcd_y )
          y = max_lcd_y - 1;    // we count rows starting w/0
     if ( x > max_lcd_x )
          x = max_lcd_x - 1;    // we count rows starting w/0

     lcdi2c_send_byte(LCD_SETDDRAMADDR | (x + row_offsets[y]), LCD_CMD);
}


/*******************************************************************************
*   Function Name:  lcdi2c_clear_line                                          *
*   Return Value:   void                                                       *
*   Parameters:     @param line: Line number to erase                          *
*   Description:    This function erases an entire lcd line                    *
*   Note:                                                                      *
*******************************************************************************/

void lcdi2c_clear_line(unsigned char line)
{
     unsigned char i;

     lcdi2c_goto_xy(0, line);
     for (i = 0; i < max_lcd_x; i++)
          lcdi2c_write_char(' ');
}


/*******************************************************************************
*   Function Name:  lcdi2c_write_char                                          *
*   Return Value:   void                                                       *
*   Parameters:     @param c: ASCII code character to write to LCD             *
*   Description:    This routine sends a character in ASCII code to LCD.       *
*   Note:           This uses the lcdi2c_send_byte function in mode data, this *
*                   it's indicated with th macro LCD_DATA                      *
*******************************************************************************/

void lcdi2c_write_char(unsigned char c)
{
	lcdi2c_send_byte(c, LCD_DATA);
}


/*******************************************************************************
*   Function Name:  lcdi2c_write_str                                           *
*   Return Value:   void                                                       *
*   Parameters:     @param str: str variable with text to write to LCD         *
*   Description:    This routine sends a series of characters to LCD.          *
*   Note:           This uses the lcdi2c_send_byte function in mode data, this *
*                   it's indicated with th macro LCD_DATA                      *
*                                                                              *
*                   Usage:    char text[12] = "Hello World";                   *
*                             lcdi2c_write_str(text);                          *
*******************************************************************************/

void lcdi2c_write_str(char * str) {
    unsigned char i = 0;

    while (str[i] != '\0')
        lcdi2c_send_byte(str[i++],LCD_DATA);
}

/*******************************************************************************
*   Function Name:  lcdi2c_write_cstr                                          *
*   Return Value:   void                                                       *
*   Parameters:     @param cstr: constant str var with text to write to LCD    *
*   Description:    This routine sends a series of constant characters to LCD. *
*   Note:           This uses the lcdi2c_send_byte function in mode data, this *
*                   it's indicated with th macro LCD_DATA                      *
*                                                                              *
*                   Usage:    lcdi2c_write_cstr("Hello World");                *
*******************************************************************************/

void lcdi2c_write_cstr(const char * cstr) {
    unsigned char i = 0;

    while (cstr[i] != '\0')
        lcdi2c_send_byte(cstr[i++],LCD_DATA);
}

/*******************************************************************************
*   Function Name:  lcdi2c_init                                                *
*   Return Value:   void                                                       *
*   Parameters:     @param usr_def_x: provided lcd max. number of columns      *
*                   @param usr_def_y: provided lcd max. number of lines        *
*                   @param i2c_address: PCF8574 addres:                        *
*                   @param i2c_sspadd: SSPADD = (Fosc/BitRate)/4-1             *
*   Description:    This routine initializes the I2C protcol, then initalizes  *
*                   the Hitachi HD44780 LCD controller                         *
*   Note:           This function must be called before any other LCD function *
*                   PCF8574 adress format is [0 1 0 0 A2 A1 A0 0]              *
*                   Default addres for PCF8574 is 0b01001110 or 0x4E           *
*                                                                              *
*******************************************************************************/


void lcdi2c_init(unsigned char usr_def_x, unsigned char usr_def_y,
	 unsigned char i2c_address, unsigned char i2c_sspadd)
{

	PCF8574_address = i2c_address;
	PCF8574Data.byte = 0x00;
	lcd_backlight_status = 0;

	max_lcd_x  = usr_def_x - 1;
	max_lcd_y = usr_def_y - 1;

    i2c_open(I2C_MASTER,I2C_SLEW_OFF,i2c_sspadd);
	// Wait  +15ms after VDD rises to 4.5V
	lcdi2c_send_nibble(0x30, LCD_CMD);           // 0x30 - Mode 8 bits
	delay10ktcy(5);                              // Wait for more than 4.1 ms
	lcdi2c_send_nibble(0x30, LCD_CMD);           // 0x30 - Mode 8 bits
	lcdi2c_send_nibble(0x30, LCD_CMD);           // 0x30 - Mode 8 bits
	lcdi2c_send_nibble(0x20, LCD_CMD);           // 0x20 - Mode 4 bits
	lcdi2c_send_byte(LCD_SYSTEM_SET_4BITS, LCD_CMD);  // 4 bits + 2 lines + 5x7 chars
	lcdi2c_send_byte(LCD_DISPLAY_ON, LCD_CMD);        // 0x0C - Display ON + Cursor OFF + Blinking OFF
	lcdi2c_send_byte(LCD_DISPLAY_CLEAR, LCD_CMD);     // 0x01 - erases the lcd + init. DDRAM
	lcdi2c_send_byte(LCD_ENTRY_MODE_SET, LCD_CMD);    // 0x06 - Increment + Display not shifted (automatic cursor chisft)
     delay10ktcy(5);

}

#endif
