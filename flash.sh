#!/bin/sh

avrdude -c ft232h -p t85 -U flash:w:./target/avr-attiny85/release/candle.elf:e
