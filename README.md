# nrf24l01

Bare-bone AVR nRF24L01+ driver, written for avr-gcc.

This library is designed to be small:  
* Purposely omits convenience functions (e.g., to set radio address registers).  
* Does not use Arduino library (e.g., SPI).

It is tested on ATmega328p.  It will likely require modifications to compile for other AVR MCUs.
