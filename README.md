 Reverese engineering a Konnwei KW206 OBD2 https://www.konnwei.com/product/430.html to read Tesla Model S data from the canbus.
 
 CANBUS packets for Model S/X decoded by Jason Hughes  https://skie.net/ used in the sketch.



MCU PINS
AT32F415

Programming
SWDIO → PA13 → QFN48 pin 34 (pin2)
SWCLK → PA14 → QFN48 pin 37 (pin3)
NRST → QFN48 pin 7
BOOT0 → QFN48 pin 44 for recovery/boot-mode work if needed.

PB6 pin 42 - down button
PA13 pin 34 - up button
PA14 pin 37 - esc button
PB2 pin 20 - ok button
PB3 pin 39 - buzzer
PB1 pin 19 - photocell (analog input)

EN25Q64 - eeprom
PB12 pin 25 - CS# pin 1
PB13 pin 26 -  CLK pin 6
PB14 pin 27 - DO (DQ1) pin 2
PB15 pin 28 - DI (DQ0) pin 5

CA-IF1051H - can transceiver
PA12 pin 33 - TXD pin 1
PA11 pin 32 - RXD pin 4


JLT35002A-PSS - 320x480 8-bit parallel LCD
MCU pin - LCD pin mapping
PA15 pin 38 - 3 PWM backlight brightness
PB4 pin 40 - 8 reset lcd
PC13 pin 2 - pin 34 WR/SCL
PC15 pin 4 - pin 35 RS
LCD DATA
PA7 pin 17 - pin 23
PA6 pin 16 - pin 24
PA5 pin 15 - pin 25
PA4 pin 14 - pin 26
PA3 pin 13 - pin 27
PA2 pin 12 - pin 28
PA1 pin 11 - pin 29
PA0 pin 10 - pin 30
