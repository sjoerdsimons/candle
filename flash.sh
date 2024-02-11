#!/bin/sh
#
# read fuses: avrdude -c ft232h -p t85   -U hfuse:r:-:h -U lfuse:r:-:h -U efuse:r:-:h
#
# default hi: 0xdf -> 0xdd => brown-outlevel = 2.7v
# write fuses: avrdude -c ft232h -p t85   -U hfuse:w:0xdd:m

avrdude -c ft232h -p t85 -U flash:w:./target/avr-attiny85/release/candle.elf:e
