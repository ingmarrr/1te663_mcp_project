
build:
	avr-gcc -Os -DF_CPU=1000000UL -mmcu=ATmega328p -c main.c lcd.c
	avr-gcc -mmcu=ATmega328p main.o lcd.o -o main.elf
	avr-objcopy -O ihex -R .eeprom main.elf main.hex
	avrdude -B 5 -c usbasp -p m328p -U flash:w:main.hex:i
