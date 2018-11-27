/*	----------------------------------------------------------------------------
	File:		lcdi2c.h
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


#ifndef __LCDI2C_H
#define __LCDI2C_H

/*******************************************************************************

                            Bitfield Definitions

*******************************************************************************/

typedef union
{
    unsigned char byte;
    struct
    {
        unsigned bit0 :1;
        unsigned bit1 :1;
        unsigned bit2 :1;
        unsigned bit3 :1;
        unsigned bit4 :1;
        unsigned bit5 :1;
        unsigned bit6 :1;
        unsigned bit7 :1;
    };
    struct
    {
        unsigned RS :1;
        unsigned RW :1;
        unsigned EN :1;
        unsigned BL :1;
        unsigned D4 :1;
        unsigned D5 :1;
        unsigned D6 :1;
        unsigned D7 :1;
    };
} __PCF8574Data_t;

extern __PCF8574Data_t PCF8574Data;

//extern unsigned char lcd_backlight_status;


/*******************************************************************************

                                     Macros

*******************************************************************************/

//----------     Hardware Configuration      ----------//
#define I2CLCD_BACKLIGHT        // Uncomment this if lcd has backlight.

//----------     Constants      ----------//
#define LOW                        0
#define HIGH                       1
#define LCD_MASK		          0b11110000
#define LCD_WRITE		          0
#define LCD_READ		          1
#define LCD_DATA    		     1
#define LCD_CMD	         	     0


//----------     Commands      ----------//
#define LCD_CLEARDISPLAY		     0x01
#define LCD_RETURNHOME	       	0x02
#define LCD_ENTRYMODESET		     0x04
#define LCD_DISPLAYCONTROL	     0x08
#define LCD_CURSORSHIFT	 	     0x10
#define LCD_FUNCTIONSET		     0x20
#define LCD_SETCGRAMADDR           0x40
#define LCD_SETDDRAMADDR           0x80

//----------     Flags for display entry mode     -----------//
#define LCD_ENTRYRIGHT             0x00
#define LCD_ENTRYLEFT              0x02
#define LCD_ENTRYSHIFTINCREMENT    0x01
#define LCD_ENTRYSHIFTDECREMENT    0x00

//----------     Flags for display on/off control     -----------//
#define LCD_DISPLAYON			0x04
#define LCD_DISPLAYOFF			0x00
#define LCD_CURSORON			0x02
#define LCD_CURSOROFF			0x00
#define LCD_BLINKON				0x01
#define LCD_BLINKOFF			0x00

//-----------    Flags for display/cursor shift     ------------//
#define LCD_DISPLAYMOVE			0x08
#define LCD_CURSORMOVE			0x00
#define LCD_MOVERIGHT			0x04
#define LCD_MOVELEFT			0x00

//----------     Flags for function set     ----------//
#define LCD_8BITMODE			0x10
#define LCD_4BITMODE			0x00
#define LCD_2LINE				0x08
#define LCD_1LINE				0x00
#define LCD_5x10DOTS			0x04
#define LCD_5x8DOTS				0x00

#define LCD_CENTER				101
#define LCD_RIGHT				102
#define LCD_LEFT				103


#define LCD_SYSTEM_SET_4BITS    0b00101000 	// 4 bits + 2 lines + 5x7 chars
#define LCD_DISPLAY_CLEAR		0b00000001 	// Clears then display
#define LCD_CURSOR_HOME         0b00000010  // Cursor to home
#define LCD_DISPLAY_ON			0b00001100 	// Display ON + Cursor OFF + Blinking OFF
#define LCD_ENTRY_MODE_SET		0b00000110 	// Increment + Display not shifted


/*******************************************************************************

                                Prototypes

*******************************************************************************/


//----------     Public     ----------//

void lcdi2c_init(unsigned char, unsigned char, unsigned char, unsigned char);
void lcdi2c_clear_line(unsigned char);
void lcdi2c_goto_xy(unsigned char, unsigned char);
void lcdi2c_write_char(unsigned char);
void lcdi2c_write_str(char *);
void lcdi2c_write_cstr(const char *);

#ifdef I2CLCD_BACKLIGHT                      //Declares function if LCD has BACKLIGHT
void lcdi2c_backlight(unsigned char light_status);
#endif

//----------     Private     ----------//
void lcdi2c_send_nibble(unsigned char nibble, unsigned char mode);
void lcdi2c_send_byte(unsigned char, unsigned char);


#endif


//----------     Inline Functions     ----------//
#define lcdi2c_clear() do { lcdi2c_send_byte(LCD_DISPLAY_CLEAR, LCD_CMD); delay10ktcy(20); } while(0)
#define lcdi2c_home()                 do { lcdi2c_send_byte(LCD_CURSOR_HOME, LCD_CMD); delay10ktcy(20); } while(0)
#define lcdi2c_scroll_display_right() lcdi2c_send_byte(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT, LCD_CMD)
#define lcdi2c_scroll_display_left()  lcdi2c_send_byte(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT, LCD_CMD)
