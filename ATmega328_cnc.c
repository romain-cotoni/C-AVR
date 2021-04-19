//avr-gcc -Wall -g -Os -mmcu=atmega328p -o ATmega328_cnc.bin ATmega328_cnc.c
//avr-objcopy -j .text -j .data -O ihex ATmega328_cnc.bin ATmega328_cnc.hex
//avrdude -p atmega328p -c usbasp -U flash:w:ATmega328_cnc.hex:i -F -P usb


#define F_CPU 1000000UL // 1MHz
#define BAUD 9600

#define ROUGE   (1 << PB3) // A+         A+ -- A-
#define BLEU    (1 << PB1) // A-         B+ -- B-
#define VERT    (1 << PB4) // B+      ROUGE -- BLEU
#define NOIR    (1 << PB2) // B-      VERT  -- NOIR
#define millisecondes 4

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


volatile char data[25] = {0};
uint8_t count = 0;
                   
int sensHoraire[]     = {ROUGE,NOIR,BLEU,VERT,ROUGE};
int sensAntiHoraire[] = {VERT,BLEU,NOIR,ROUGE,VERT };


void init_uart(void)
{
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

    #if USE_2X
    UCSR0A |= (1<<U2X0);
    #else
    UCSR0A &= ~(1<<U2X0);
    #endif

    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);             // 8bit data
    UCSR0B = (1<<RXEN0) | (1<<RXCIE0) | (1<<TXEN0); // Enable RX - RX INTERRUPT - Enable TX
}


/*
char usart_getchar(void)
{   
    while( !(UCSR0A & (1<<RXC0)) ) ; // Wait for data to be received
    return UDR0;                     // Get and return received data from buffer
}*/


void usart_transmit(char data)
{
    while( !( UCSR0A & (1<<UDRE0)) ); // Wait for empty transmit buffer
    UDR0 = data;                      // Put data into buffer, sends the data
}


void uart_flush(void) 
{
	while(UCSR0A & (1<<RXC0)) UDR0;
    return;
}


void halfStepping(int direction[])
{    
    for (uint8_t i=0; i<4; i++ )    // step through the phases
    {	    
        PORTB = direction[i];	    // single-coil part
        _delay_ms(millisecondes);
    
        PORTB |= direction[i+1];	// add in half-step
        _delay_ms(millisecondes);
    }  
}


int delay(int n)
{
    for(int i=0;i<n;i++) _delay_ms(1);
    return 0;
}


/*void parse()
{
    unsigned char strX[6], strY[6], strZ[6];    
    unsigned char *token = strtok(data," ");
    uint8_t i;
    while( token != NULL )        //WHILE THERE ARE TOKENS IN STRING
    {   
        if     (token[0] == 'X') strcpy(strX,&token[0]);
        else if(token[0] == 'Y') strcpy(strY,&token[0]);
        else if(token[0] == 'Z') strcpy(strZ,&token[0]);        
        token = strtok(NULL," "); //GET NEXT TOKEN
    }
    int nx = (((strX[1]-'0')*1000)) + ((strX[2]-'0')*100) + ((strX[3]-'0')*10) + ((strX[4]-'0')*1) ;
    int ny = (((strY[1]-'0')*1000)) + ((strY[2]-'0')*100) + ((strY[3]-'0')*10) + ((strY[4]-'0')*1) ;
    int nz = (((strZ[1]-'0')*1000)) + ((strZ[2]-'0')*100) + ((strZ[3]-'0')*10) + ((strZ[4]-'0')*1) ;
    int n[5] = {0,0,nx,ny,nz};
    for(i=2;i<5;i++) {PORTB |= (1 << i); delay(n[i]); PORTB &= ~(1 << i);}
    count = 0; 
    memset(data,'\0',sizeof(data));
    uart_flush();
    return;
}*/


void parse_motor()
{   
    char strX[10], strY[10], strZ[10];    
    char *token = strtok((char *)data," ");    
    while( token != NULL)        //WHILE THERE ARE TOKENS IN STRING
    {   
        if     (token[0] == 'X') strcpy(strX,&token[0]);
        else if(token[0] == 'Y') strcpy(strY,&token[0]);
        else if(token[0] == 'Z') strcpy(strZ,&token[0]);        
        token = strtok(NULL," "); //GET NEXT TOKEN
    }
    uint16_t n = ((strX[2]-'0') * 100) + ((strX[3]-'0')*10) + (strX[4]-'0');
    if     (strX[1]=='-') for(uint16_t i=0; i<=n ; i++) halfStepping(sensAntiHoraire);    
    else if(strX[1]=='0') for(uint16_t i=0; i<=n ; i++) halfStepping(sensHoraire    );     
    PORTB &= ~ROUGE & ~BLEU & ~VERT & ~NOIR;
        
    count = 0;
    memset((void *)data,'\0',sizeof(data));
    memset((void *)strX,'\0',sizeof(strX));
    memset((void *)strY,'\0',sizeof(strY));
    memset((void *)strZ,'\0',sizeof(strZ));
    uart_flush();
    return;
}


ISR(USART_RX_vect)
{            
    data[count] = UDR0;
    if(data[count++]=='#') { parse_motor();}
}


int main(void)
{       
    DDRB |= ROUGE | BLEU | VERT | NOIR | (1 << PB5);    
    init_uart();
    sei(); //enable all interrupts
    char txt[] = "hello";
    while(1)
    {
        usart_transmit('h');
    }
    
    return 0;
}
 



