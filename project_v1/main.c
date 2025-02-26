#define F_CPU 1000000UL // 1MHz internal clock
#define RFID_CS   PB2  // RFID module Chip Select
#define LCD_CS    PB1  // LCD module Chip Select
# include <avr/io.h>
# include <stdio.h>
# include <util/delay.h>
# include "lcd.h" // library file is in the same directory
#include <utils.h>
#include <spi.h>
#include <mfrc522.h>


uint8_t SelfTestBuffer[64];

void LCD_init (void) // collect hardware initializations here
{
PORTD = 0b00000100 ; // pull -up resistor active PD2 (S1)
lcd_init ( LCD_DISP_ON ); // initialize LCD
lcd_clrscr ();
lcd_puts ("Hello world"); // put text string on display
}

////////////////////////////////////////////////
/*
 * main.c
 * 
 * Copyright 2013 Shimon <shimon@monistit.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */
 

 

  
 int main()
 {
     uint8_t byte;
     uint8_t str[MAX_LEN];
     _delay_ms(50);
     LCD_init();
     lcd_gotoxy (0 ,0); // first character first row
     lcd_puts ("RFID Reader");
     _delay_ms(5000);
     lcd_clrscr (); // clear screen
          
     spi_init();
     //init reader

     mfrc522_init();
     
     lcd_gotoxy (0 ,0); // first character first row
     PORTB |= (1 << RFID_CS);  // Deselect RFID
     PORTB &= ~(1 << LCD_CS);  // Select LCD
     lcd_puts ("Checkpoint");
     PORTB |= (1 << LCD_CS);   // Deselect LCD
     
     //check version of the reader
     byte = mfrc522_read(VersionReg);
     char debug_buf[20];
        sprintf(debug_buf, "Ver: %02X", byte);
        lcd_clrscr();
        lcd_puts(debug_buf);
        _delay_ms(2000);

     if(byte == 0x92)
     {
        lcd_gotoxy (0 ,0); // first character first row
        lcd_puts ("MIFARE RC522v2");
        lcd_gotoxy (0 ,1); // first character second row
        lcd_puts ("Detected");
     }else if(byte == 0x91 || byte==0x90)
     {
        lcd_gotoxy (0 ,0); // first character first row
        lcd_puts ("MIFARE RC522v1");
        lcd_gotoxy (0 ,1); // first character second row
        lcd_puts ("Detected");
     }else
     {
        lcd_gotoxy (0 ,0); // first character first row
        lcd_puts ("No reader found");
     }
     
     byte = mfrc522_read(ComIEnReg);
     mfrc522_write(ComIEnReg,byte|0x20);
     byte = mfrc522_read(DivIEnReg);
     mfrc522_write(DivIEnReg,byte|0x80);
     
     _delay_ms(1500);
     lcd_clrscr (); // clear screen
     
     while(1){
        char buffer [40];
        byte = mfrc522_request(PICC_REQALL,str);
        char buffer1 [40];
        lcd_gotoxy (0 ,0); // first character second row
        sprintf (buffer , "byte : %3x", byte);
        lcd_puts (buffer);
         
         if(byte == CARD_FOUND)
         {
             byte = mfrc522_get_card_serial(str);
             if(byte == CARD_FOUND)
             {
                 for(byte=0;byte<8;byte++)
                 lcd_gotoxy (byte*2 ,0); // first character second row
                 sprintf (buffer1 , "byte : %3d", str[byte]);
                 lcd_puts (buffer1);
                     //LCDHexDumpXY(byte*2,0,str[byte]);
                 
                 _delay_ms(2500);
             }
             else
             {
                lcd_gotoxy (0 ,0); // first character second row
                lcd_puts("Error");
             }
         }
         
         _delay_ms(1000);
     } 
 }
 