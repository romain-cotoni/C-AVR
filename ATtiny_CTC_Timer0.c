#define F_CPU 1000000UL

ISR(TIMER0_COMPA_vect)
{   
    PORTB |=  (1 << PB0);    // PB0 HIGH
}


ISR(TIMER0_COMPB_vect)
{
    PORTB &= ~(1 << PB0);    // PB0 LOW
    //if      (OCR0B == 6) OCR0B = 4;
    //else if (OCR0B == 4) OCR0B = 8;
    //else if (OCR0B == 8) OCR0B = 6;
}


int main(void)
{
    DDRB |= (1 << PB0);      // PB0 OUTPUT MODE (OUTPUT=1)        

    TCNT0 =  0;              // compteur qui augmente automatiquement
    OCR0A = 78 - OCR0A;              // 0.020s = 20
    OCR0B =  4;              // 0.001s=4 //0.0015s=6 //0.002s=8   
    
    TCCR0A |= (1 << WGM01);  // mode CTC (Clear Timer on Counter)   
    TCCR0B |= (1 << CS02);   // prescaler 256     //(1 << CS02) | (1 << CS00) ;  → Prescaler 1024
    TIMSK  |= (1 << OCIE0A); // Output Compare Match A Interrupt Enable
    TIMSK  |= (1 << OCIE0B); // Output Compare Match B Interrupt Enable
    
    sei();
    while(1){}
}
