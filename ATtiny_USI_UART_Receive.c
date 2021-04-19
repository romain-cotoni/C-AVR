#define F_CPU 1000000UL
#include <avr/interrupt.h>
#include <avr/io.h>

void using_Byte_Received(unsigned char c) 
{
    if     ( c == 'b' ){ PORTB |=  (1<< PB1); }
    else if( c == 'a' ){ PORTB &= ~(1<< PB1); }
}

unsigned char ReverseByte (unsigned char x) 
{
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return x;    
}


void InitialiseUSI()             // Initialise USI for UART reception
{  
    DDRB  &= ~(1 << PB0)   ;     // PB0 INPUT MODE (change pin interrupt)
    PORTB |=  (1 << PB0)   ;     // PB0 Enable internal pull-up
    USICR  =   0           ;     // Disable USI.
    GIFR   =  (1 << PCIF)  ;     // Clear pin change interrupt flag.
    GIMSK |=  (1 << PCIE)  ;     // Enable pin change interrupts
    PCMSK |=  (1 << PCINT0);     // Enable pin change on PB0
}


ISR (PCINT0_vect)                // Pin change interrupt detects start of UART reseption
{
    if (!(PINB & 1 << PINB0))    // Ignore if PB0 is HIGH
    {       
        GIMSK &= ~(1 << PCIE)  ; // Disable pin change interrupts
        TCCR0A =  (1 << WGM01) ; // CTC mode
        TCCR0B =  (1 << CS00)  ; // Set no prescaler
        TCNT0  =  0            ; // Count up from 0
        OCR0A  =  51           ; // Delay (51+1)
        TIFR  |=  (1 << OCF0A) ; // Clear output compare flag
        TIMSK |=  (1 << OCIE0A); // Enable output compare interrupt
    }
}


ISR (TIMER0_COMPA_vect)          // COMPA interrupt indicates middle of start bit
{
    TIMSK &= ~(1<<OCIE0A);       // Disable COMPA interrupt
    TCNT0  =   0         ;       // Count up from 0
    OCR0A  =   103       ;       // Shift every (103+1)
    
    USICR  =  (1<<USIOIE) |      // Enable USI OVF interrupt
              (0<<USIWM0) |        
              (1<<USICS0) ;      // select Timer0 compare match as USI Clock source

    USISR  =  (1<<USIOIF) | 8;   // Clear USI OVF flag, and set counter
}  


ISR (USI_OVF_vect)               // USI overflow interrupt indicates we've received a byte
{
    USICR    = 0    ;            // Disable USI 
        
    int temp = USIDR;
    using_Byte_Received(ReverseByte(temp));

    GIFR     = 1<<PCIF;          // Clear pin change interrupt flag.
    GIMSK   |= 1<<PCIE;          // Enable pin change interrupts again
}





int main(void) 
{
    DDRB |=  (1 << PB1) ;        //PB1 OUTPUT 
    InitialiseUSI();

    return(1);
}



