#define F_CPU 1000000UL
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>


// ATtiny       HM10
// PB0        → TX 
// PB2        → STATE
// PB2 OUTPUT → LED to switch


void sleep() 
{   
    GIMSK  |= 1<<PCIE;                          // Enable Pin Change Interrupts
    PCMSK  |= 1<<PCINT2;                        // Use PB2 as interrupt pin
    ADCSRA &= ~(1 << ADEN);                     // ADC off
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);        // replaces above statement
    
    sleep_enable();                             // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
    sei();                                      // Enable interrupts
    
    sleep_cpu();                                // sleep
    
    cli();                                      // Disable interrupts
    PCMSK &= ~_BV(PCINT2);                      // Turn off PB2 as interrupt pin
   
    sleep_disable();                            // Clear SE bit
    ADCSRA |= _BV(ADEN);                        // ADC on

    sei();                                      // Enable interrupts
}


unsigned char ReverseByte (unsigned char x) 
{
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return x;    
}


void InitialiseUSI (void)                       // Initialise USI for UART reception
{  
    DDRB  &= ~(1 << PB0);                       // PB0 INPUT (change pin interrupt)
    PORTB |=  (1 << PB0);                       // Enable internal pull-up on pin PB0
    USICR  =   0         ;                      // Disable USI
    GIFR   =   1<<PCIF   ;                      // Clear pin change interrupt flag
    GIMSK |=   1<<PCIE   ;                      // Enable pin change interrupts
    PCMSK |=   1<<PCINT0 ;                      // Enable pin change on pin 0
}


ISR (PCINT0_vect)                               // Pin change interrupt detects start of UART reseption
{
    if (!(PINB & 1<<PINB0))                     // Ignore if PB0 is HIGH
    {       
        GIMSK &= ~(1<<PCIE)  ;                  // Disable pin change interrupts
        TCCR0A =  (1<<WGM01) ;                  // CTC mode
        TCCR0B =  (1<<CS00)  ;                  // Set no prescaler
        TCNT0  =  0          ;                  // Count up from 0
        OCR0A  =  51         ;                  // Delay (51+1)
        TIFR  |=  (1<<OCF0A) ;                  // Clear output compare flag
        TIMSK |=  (1<<OCIE0A);                  // Enable output compare interrupt
    }
}


void using_Byte_Received(char c) 
{ 
    if     (c=='a'){OCR1A= 0 ;}                 // LED switch off
    else if(c=='b'){OCR1A=255;}                 // LED switch on
    //else if(c=='c'){OCR1A-=10;} 
    //else if(c=='d'){OCR1A+=10;}
    _delay_ms(15); 
}                                               


ISR (TIMER0_COMPA_vect)                         // COMPA interrupt indicates middle of start bit
{
    TIMSK &= ~(1<<OCIE0A);                      // Disable COMPA interrupt
    TCNT0  = 0;                                 // Count up from 0
    OCR0A  = 103;                               // Shift every (103+1)
        
    USICR  = 1<<USIOIE | 0<<USIWM0 | 1<<USICS0; // Enable USI OVF interrupt, and select Timer0 compare match as USI Clock source
    USISR  = 1<<USIOIF | 8;                     // Clear USI OVF flag, and set counter
}


ISR (USI_OVF_vect)                              // USI overflow interrupt indicates we've received a byte
{
    USICR = 0;                                  // Disable USI         
    int temp = USIDR;
    using_Byte_Received(ReverseByte(temp));
    GIFR   = 1<<PCIF;                           // Clear pin change interrupt flag.
    GIMSK |= 1<<PCIE;                           // Enable pin change interrupts again  
}


void flashLed(char flash)
{
    while(flash>0)
    {
        OCR1A=255;
        _delay_ms(200);
        OCR1A=0  ; 
        _delay_ms(200);
        flash--;
    }
}


int main(void)
{
    DDRB  |=  (1 << PB1) ;                      //PB1 OUTPUT → LED to switch
    DDRB  &= ~(1 << PB2) ;                      //PB2 INPUT (change pin interrupt) 
    PORTB |=  (1 << PB2) ;                      //PB2 Enable internal pull-up
    
    TCCR1 =  (1<<PWM1A) | (1<<COM1A0) | (1<<CTC1) | (1<<CS12);
    OCR1C =  255; 
    OCR1A =  0  ;
    
    InitialiseUSI(); 
    
    flashLed(20);
    while(1)
    {
        //if   (!(PINB & 1<<PINB2)) {sleep();}
        //while(!(PINB & 1<<PINB2));
    }
}


