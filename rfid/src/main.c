#define F_CPU 1000000UL // 1MHz internal clock
#define NUM_OF_CARDS 10

# include <avr/io.h>
# include <stdio.h>
# include <util/delay.h>
#include <avr/interrupt.h>
# include "lcd.h" 
#include <utils.h>
#include <spi.h>
#include <mfrc522.h>
#include <string.h> // Include for strcpy

uint8_t SelfTestBuffer[64];
volatile static uint8_t cntr = 0;
volatile int last_card_i;

typedef struct carddata {
   char snr[9];  // Serial number of card
   int auth;   // Authentication level of card ( 1: opens one relay, 2: opens both relays)
} carddata;

carddata cards_array[NUM_OF_CARDS]; // Can store maximum 10 cards - statically defined

void card_data_init(carddata* carray) {   // Add some cards with auth level, set the left to empty snr, 0 auth level
   strcpy(carray[0].snr, "b9e802c2"); // Blue tag
   carray[0].auth = 2;

   strcpy(carray[1].snr, "e4c7ef4e"); // Green
   carray[1].auth = 2;

   strcpy(carray[2].snr, "b4f5e421"); // Red
   carray[2].auth = 1;

   strcpy(carray[3].snr, "871e2442"); // White
   carray[3].auth = 1;

   for (int i = 4; i<NUM_OF_CARDS; i++) {
      strcpy(carray[3].snr, ""); 
      carray[3].auth = 0;
   }
}


void init (void) 
{
DDRC = 0b00110000 ; // PC5 and PC4 active -> GPIOs controlling relays
PORTD = 0b00001100 ; // pull-up resistors active -> PD2 (S1) and PD3 (S2)

lcd_init (LCD_DISP_ON); // initialize LCD
lcd_clrscr ();
/* Setting up TIMER0 */
TCCR0A = (0 << COM0A1) | (0 << COM0A0) 
         | (0 << COM0B1) | (0 << COM0B0) 
         | (0 << WGM01) | (0 << WGM00); 

TCCR0B = (0 << WGM02) 
         | (1 << CS02) | (0 << CS01) | (1 << CS00); //prescaler = 1024, ~1000 ticks/second

  // Enable the TIMER0 OVERFLOW INTERRUPT
  TIMSK0 = (0 << OCIE0B) | (0 << OCIE0A) | (1 << TOIE0);

  // Config and enable INT0 nad INT1 interrupt pins
  EICRA = (1 << ISC11 ) | (0 << ISC10 ) | (1 << ISC01 ) | (0 << ISC00 );
  EIMSK = (1 << INT1) | (1 << INT0 );

  sei();
}

// Timer0, runs every ~0.256 second
ISR(TIMER0_OVF_vect)
{
   cntr ++;

   if (cntr > 20) {  // If 0.256*20 = ~5s has passed
      PORTC &= ~(1 << PC5); //Turn off both relays
      PORTC &= ~(1 << PC4); //Turn off both relays
      cntr = 0;
      lcd_clrscr (); 
   }
}

// INT0, activated when S1 button is pushed, sets the last card's authentication level to 1
ISR(INT0_vect)
{
   cards_array[last_card_i].auth = 1;
   lcd_gotoxy (0 ,0); 
   lcd_clrscr (); 
   lcd_puts ("auth set to 1!");
   _delay_ms(1000);
   lcd_clrscr (); 
}

// INT1, activated when S2 button is pushed, sets the last card's authentication level to 2
ISR(INT1_vect)
{
   cards_array[last_card_i].auth = 2;
   lcd_gotoxy (0 ,0); 
   lcd_clrscr (); 
   lcd_puts ("auth set to 2!");
   _delay_ms(1000);
   lcd_clrscr (); 
}

// Function to convert 4-byte array to a hex string
void convert_serial_to_string(uint8_t *serial, char *serial_str) {
   sprintf(serial_str, "%02x%02x%02x%02x", serial[0], serial[1], serial[2], serial[3]);
}

 int main()
 {
     card_data_init(cards_array); // Init all cards in the array
     uint8_t byte;
     uint8_t str[MAX_LEN];
     char serial_str[9];   // Stores a serial number
     char auth_buf[2];  // Stores an authentication level number

     _delay_ms(50);
     init();
     lcd_gotoxy (0 ,0); 
     lcd_puts ("RFID Reader");
     _delay_ms(4000);
     lcd_clrscr (); 
          
     spi_init();

     mfrc522_init();
     
     // Check version of the reader
     byte = mfrc522_read(VersionReg);

     if(byte == 0x92)
     {
        lcd_gotoxy (0 ,0); 
        lcd_puts ("MIFARE RC522v2");
        lcd_gotoxy (0 ,1); 
        lcd_puts ("Detected");
     } else if(byte == 0x91 || byte==0x90) {
        lcd_gotoxy (0 ,0); 
        lcd_puts ("MIFARE RC522v1");
        lcd_gotoxy (0 ,1); 
        lcd_puts ("Detected");
     } else {
        lcd_gotoxy (0 ,0); 
        lcd_puts ("No reader found");
     }
     _delay_ms(2000);
     byte = mfrc522_read(ComIEnReg);
     mfrc522_write(ComIEnReg,byte|0x20);
     byte = mfrc522_read(DivIEnReg);
     mfrc522_write(DivIEnReg,byte|0x80);
     
     lcd_clrscr (); 
     
     while(1){
        byte = mfrc522_request(PICC_REQALL,str);
         if(byte == CARD_FOUND)
         {
             byte = mfrc522_get_card_serial(str); // Get serial number of the card, returns status
             if (byte == CARD_FOUND)
             {
               lcd_gotoxy(0,0);
               lcd_puts ("card found!");
               _delay_ms(1000);
               lcd_gotoxy(0, 0);
               lcd_puts("snr:");
               convert_serial_to_string(str, serial_str);
               lcd_gotoxy(4, 0);
               lcd_puts(serial_str);
               _delay_ms(100);
               
               int found = 0; // Flag to check if the card is in the array
               int empty_slot = -1; // Index of an empty slot

               for (int i=0; i<NUM_OF_CARDS; i++) {
                  if (strcmp(serial_str, (cards_array[i].snr)) == 0) { // If card exists between the stored cards
                     found = 1;
                     last_card_i = i;  // Remember position of the card in the cards' array
                     lcd_gotoxy(0, 1);
                     sprintf(auth_buf, "auth:%d", (cards_array[i].auth));  
                     lcd_puts(auth_buf);
                     switch (cards_array[i].auth) {   // Check which authentication level it has
                        case 1:  // If auth lvl = 1, switch one relay on, restart timer
                           PORTC |= (1 << PC5); 
                           cntr = 0; 
                           break;
                        case 2:  // If auth lvl = 2, switch both relays on, restart timer
                           PORTC |= (1 << PC5); 
                           PORTC |= (1 << PC4); 
                           cntr = 0; 
                           break;
                        default:
                           break;
                     }
                     break;
                  }
                  if (cards_array[i].snr[0] == '\0' && empty_slot == -1) {  // If card is not in the array, and there is an empty slot, add it
                     empty_slot = i;
                  }
                 
               }

               if (!found) { 
                  if (empty_slot != -1) { // If there is space to store the card
                      strcpy(cards_array[empty_slot].snr, serial_str);
                      cards_array[empty_slot].auth = 0; // Default auth level is 0
                      last_card_i = empty_slot;
  
                      lcd_gotoxy(0, 1);
                      lcd_puts("New card added");
                      _delay_ms(2000);
                      lcd_clrscr();
                  } else { // No space in the array
                      lcd_gotoxy(0, 1);
                      lcd_puts("Memory full!");
                  }
              }
               
             }
             else
             {
                lcd_gotoxy (0 ,0); 
                lcd_puts("Error");
             }
         }
         _delay_ms(1000);
         
     } 
 }
 