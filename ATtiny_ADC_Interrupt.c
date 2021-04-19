#include <avr/io.h>
#include <avr/interrupt.h>
 
 
int adc_result;
 
ISR(ADC_vect)
{
    adc_result  = ADCL;                                                 // récupère la valeur du registre bas  (les 8 bits les moins forts)
    adc_result |= ((int) ADCH) << 8;                                    // récupère la valeur du registre haut (les 2 bits les plus forts)


    if     ( adc_result < 500 ) { PORTB &= ~(1<< PB1); }                // PB1 LOW  si la valeur est inférieur a 500
    else if( adc_result > 800 ) { PORTB &= ~(1<< PB1); }                // PB1 LOW  si la valeur est supérieur à 800
    else                        { PORTB |=  (1<< PB1); }                // PB1 HIGH si la valeur est comprise entre 500 et 800

    ADCSRA |= (1 << ADSC);                                              // redémarre la conversion ADC
}
 
int main()
{
    DDRB  |= (1<<PB1);                                                  // PB1 OUTPUT MODE (OUTPUT=1)

    ADMUX  = (0 << REFS2) | (0 << REFS1) | (0 << REFS0) |               // définit la tension de référence à VCC
             (0 << ADLAR) |                                             // determine l'alignement des bits (0 pour droite, 1 pour gauche)
             (0 << MUX3)  | (0 << MUX2)  | (1 << MUX1)  | (1 << MUX0);  // choisit le canal ADC3
 
    ADCSRA = (1 << ADEN)  |                                             // active l'ADC
             (1 << ADIE)  |                                             // active les interruptions de fin de conversion
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);                // choisit un diviseur : 128
 
    sei();                                                              // active les interruptions

    ADCSRA |= (1 << ADSC);                                              // démarre la conversion ADC

    while(1);                                                           // boucle principale
}
