#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include "lcd.h"

// RFID Commands
#define MFRC522_IDLE 0x00
#define MFRC522_AUTHENT 0x0E
#define MFRC522_RECEIVE 0x08
#define MFRC522_TRANSMIT 0x04
#define MFRC522_TRANSCEIVE 0x0C
#define MFRC522_RESET 0x0F
#define MFRC522_CALCCRC 0x03

// RFID Registers
#define CommandReg 0x01
#define ComIEnReg 0x02
#define FIFODataReg 0x09
#define FIFOLevelReg 0x0A
#define ControlReg 0x0C
#define TModeReg 0x2A
#define TPrescalerReg 0x2B
#define TReloadRegH 0x2C
#define TReloadRegL 0x2D

void SPI_Init(void) 
{
    // Set MOSI, SCK as output
    DDRB |= (1<<PB3)|(1<<PB5);
    // Set MISO as input
    DDRB &= ~(1<<PB4);
    // Enable SPI, Master mode, set clock rate
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}

uint8_t SPI_Transmit(uint8_t data) 
{
    // Start transmission
    SPDR = data;
    // Wait for transmission complete
    while(!(SPSR & (1<<SPIF)));
    // Return received data
    return SPDR;
}

void MFRC522_WriteRegister(uint8_t addr, uint8_t val) 
{
    // CS low
    PORTB &= ~(1<<PB2);
    // Address format: 0XXXXXX0 where XXXXXX is address
    SPI_Transmit((addr<<1)&0x7E);
    SPI_Transmit(val);
    // CS high
    PORTB |= (1<<PB2);
}

uint8_t MFRC522_ReadRegister(uint8_t addr) 
{
    uint8_t val;
    // CS low
    PORTB &= ~(1<<PB2);
    // Address format: 1XXXXXX0 where XXXXXX is address
    SPI_Transmit(((addr<<1)&0x7E) | 0x80);
    val = SPI_Transmit(0x00);
    // CS high
    PORTB |= (1<<PB2);
    return val;
}

void MFRC522_Reset(void) 
{
    MFRC522_WriteRegister(CommandReg, MFRC522_RESET);
    _delay_ms(50);
    
    // Set timer
    MFRC522_WriteRegister(TModeReg, 0x8D);
    MFRC522_WriteRegister(TPrescalerReg, 0x3E);
    MFRC522_WriteRegister(TReloadRegL, 30);
    MFRC522_WriteRegister(TReloadRegH, 0);
}

void MFRC522_Init(void) 
{
    // Set SS pin as output
    DDRB |= (1<<PB2);
    // SS high (inactive)
    PORTB |= (1<<PB2);
    
    SPI_Init();
    MFRC522_Reset();
}

void LCD_Init(void)
{
    PORTD = 0b00000100; // pull -up resistor active PD2 (S1)
    lcd_init(LCD_DISP_ON); // initialize LCD
    lcd_clrscr();
    lcd_puts("Hello world"); // put text string on display
}

int main(void) 
{
    // Initialize RFID & LCD
    // MFRC522_Init();
    LCD_Init();
    _delay_ms(100);
    lcd_puts("Hello world"); // put text string on display
    
    while(1) {
        // Read version register - should return 0x91 or 0x92
        // uint8_t version = MFRC522_ReadRegister(0x37);
        
        // You can add LED indicators here to show the version was read
        
        _delay_ms(1000);
        lcd_puts("Hello world"); // put text string on display
    }
    
    return 0;
}