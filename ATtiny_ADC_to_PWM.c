#include <avr/io.h>
#include <avr/interrupt.h>
 
int adc_result;
 
ISR(ADC_vect)
{
    adc_result  = ADCL;                                                // récupère la valeur du registre bas (les 8 bits les moins forts)
    adc_result |= ((int)ADCH) << 8;                                    // récupère la valeur du registre haut (les 2 bits les plus forts)

    OCR1A = adc_result/4 ;                                             // modifie la valeur de référence et divise par 4 pour convertir la résolution de 1024 à 255

    ADCSRA |= (1 << ADSC);                                             // redémarre la conversion ADC
}
 
int main()
{
    DDRB |= (1 << PB1);                                                // PB1 OUTPUT MODE (OUTPUT=1)

    TCCR1 = (1<<PWM1A)  |                                              // active la sortie PWM sur le compteur 1, avec le registre A en référence(OCR1A)
            (1<<COM1A0) |                                              // définit l'ordre de comparaison (0 si compteur < référence, 1 sinon)
            (1<<CTC1)   |                                              // permet d'utiliser le registre OCR1C comme TOP
            (1<<CS12)   ;                                              // définit un diviseur : 8                                                           
                                                           
    OCR1C = 255;                                                       // valeur TOP (maximum 255 [2^8bits])
    OCR1A = 0  ;                                                       // valeur de référence

    ADMUX = (0 << REFS2) | (0 << REFS1) | (0 << REFS0) |               // définit la tension de référence à VCC
            (0 << ADLAR) |                                             // détermine l'alignement des bits (0 pour droite, 1 pour gauche)
            (0 << MUX3)  | (0 << MUX2)  | (1 << MUX1)  | (1 << MUX0);  // selection canal ADC3
 
    ADCSRA = (1 << ADEN)  |                                            // active l'ADC
             (1 << ADIE)  |                                            // active les interruptions de fin de conversion
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);               // choisit un diviseur : 128
 
    sei();                                                             // active les interruptions

    ADCSRA |= (1 << ADSC);                                             // démarre la conversion ADC

    while(1);                                                          // boucle principale
}
