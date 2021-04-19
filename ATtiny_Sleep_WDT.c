#define F_CPU 1000000UL
#include <avr/io.h> 
#include <util/delay.h>
#include <avr/wdt.h> 
#include <avr/sleep.h>
#include <avr/interrupt.h>


int count;

void flashLed(int n)
{
    int i = 0;
    while(i<n)
    {
        PORTB |=  (1 << PB2);
        _delay_ms(1000);
        PORTB &= ~(1 << PB2);
        _delay_ms(1000);
        i++;
    }   
}    

int main(void) 
{
    DDRB |= (1 << PB2);      // PB2 OUTPUT MODE (OUTPUT=1)
    
    MCUSR  &= ~(1 << WDRF);                 // RESET THE WATCHDOG RESET FLAG    
	WDTCR  |=  (1 << WDCE) | (1 << WDE);    // SET WATCHDOG CHANGE ENABLE & WATCHDOG ENABLE
    WDTCR   =  (1 << WDP3) | (1 << WDP0);   // SET THE WATCHDOG TIMEOUT PRESCALER
    WDTCR  |=  (1 << WDIE);                 // ENABLE THE WATCHDOG INTERRUPT (NOT RESET)    

    while(1)
    {        
        //ADCSRA |= (1 << ADEN);            // ADC ON               
        flashLed(4);

        count = 0;
        //ADCSRA &= ~(1 << ADEN);           // ADC OFF
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);    
        sleep_enable();
        sei();                              // ENABLE INTERRUPTS                       
        while(count<2) sleep_mode();        // SLEEP
         
        cli();                              // DISABLE INTERRUPTS
        sleep_disable();                    // WAKE UP FROM SLEEP
        
    }
    
    return 0;
}

ISR(WDT_vect) 
{
    // DON'T DO ANYTHING HERE BUT THIS BLOCK OF CODE MUST BE INCLUDED OTHERWISE THE INTERRUPT CALLS AN UNINITIALIZED INTERRUPT HANDLER
    count++;
}

