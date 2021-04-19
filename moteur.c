//avr-gcc -Wall -g -Os -mmcu=atmega328p -o moteur.bin moteur.c
//avr-objcopy -j .text -j .data -O ihex moteur.bin moteur.hex
//avrdude -p atmega328p -c usbasp -U flash:w:moteur.hex:i -F -P usb

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define ROUGE   (1 << PD2) // A+          A+ -- A-
#define BLEU    (1 << PD3) // A-          B+ -- B-
#define VERT    (1 << PD5) // B+       ROUGE -- BLEU
#define NOIR    (1 << PD6) // B-       VERT  -- NOIR

#define millisecondes 4
        
              
int sensHoraire[]     = {ROUGE,NOIR ,BLEU ,VERT ,ROUGE};
int sensAntiHoraire[] = {VERT ,BLEU ,NOIR ,ROUGE,VERT };


void halfStepping(int direction[])
{    
    for (uint8_t i=0; i<4; i++ )    // step through the phases
    {	    
        PORTD = direction[i];	    // single-coil part
        _delay_ms(millisecondes);
        
        PORTD |= direction[i+1];	// add in half-step
        _delay_ms(millisecondes);
    }  
}


int main(void)
{       
    DDRD |= ROUGE | NOIR | BLEU | VERT;
    
    while(1)
    {
        for(uint8_t i=0; i<=10 ; i++) halfStepping(sensHoraire    );
        //for(uint8_t i=0; i<=10; i++) halfStepping(sensAntiHoraire);
    }
    return 0;
}


