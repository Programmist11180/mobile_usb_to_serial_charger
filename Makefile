# Makefile for mobile_usb_to_serial_charger

CFLAGS= -Wall -Wextra 

all: charger.o
	gcc $(CFLAGS) -o mobile_usb_to_serial_charger charger.o

charger.o:
	gcc $(CFLAGS) -c charger.c

clean:
	rm -f mobile_usb_to_serial_charger *.o *.a
