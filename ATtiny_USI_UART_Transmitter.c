//ATTiny85 PB1/MISO/DO = Serial UART Tx -> connect to Rx of serial output device

//Supported combinations:
//F_CPU 1000000   BAUDRATE 1200, 2400 
//F_CPU 8000000   BAUDRATE 9600, 19200
//F_CPU 16000000  BAUDRATE 9600, 19200, 28800, 38400

#define F_CPU 1000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#define BAUDRATE            9600                       // Set baud rate 
#define STOPBITS               1                       // Set number of stop bits

// Calculate prescaler setting
#define CYCLES_PER_BIT      ( (F_CPU) / (BAUDRATE) )   
#if (CYCLES_PER_BIT > 255)                             // If bit width in cpu cycles is greater than 255 then divide by 8 to fit in timer
    #define DIVISOR            8
    #define CLOCKSELECT        2
#else
    #define DIVISOR            1
    #define CLOCKSELECT        1
#endif

#define FULL_BIT_TICKS      ( (CYCLES_PER_BIT) / (DIVISOR) )


// USISerial send state variable and accessors
enum USISERIAL_SEND_STATE{ AVAILABLE, FIRST, SECOND };  // USISerial send state variable and accessors

static volatile enum USISERIAL_SEND_STATE usiserial_send_state = AVAILABLE;

static inline enum USISERIAL_SEND_STATE usiserial_send_get_state()
{
    return usiserial_send_state;
}

static inline void usiserial_send_set_state(enum USISERIAL_SEND_STATE state)
{
    usiserial_send_state=state;
}

char usiserial_send_available()
{
    return usiserial_send_get_state()==AVAILABLE;
}

// Transmit data persistent between USI OVF interrupts
static volatile char usiserial_tx_data;

static inline char usiserial_get_tx_data()
{
    return usiserial_tx_data;
}

static inline void usiserial_set_tx_data(char tx_data)
{
    usiserial_tx_data = tx_data;
}


static char reverse_byte (char x) {
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return x;
}


void usiserial_send_byte(char data)
{
    while (usiserial_send_get_state() != AVAILABLE)
    {
        // Spin until we finish sending previous packet
    };

    usiserial_send_set_state(FIRST);
    usiserial_set_tx_data(reverse_byte(data));
 
    // Configure Timer0
    TCCR0A = 2 << WGM00;                                    // CTC mode
    TCCR0B = CLOCKSELECT;                                   // Set prescaler to clk or clk /8
    GTCCR |= 1 << PSR0;                                     // Reset prescaler
    OCR0A  = FULL_BIT_TICKS;                                // Trigger every full bit width
    TCNT0  = 0;                                             // Count up from 0 

    // Configure USI to send high start bit and 7 bits of data
    USIDR = 0x00 | (usiserial_get_tx_data() >> 1);          // Start bit (low) & first 7 bits of serial data
                         
    USICR = (1 << USIOIE) |                                 // Enable USI Counter OVF interrupt
            (0 << USIWM1) | (1 << USIWM0) |                 // Select three wire mode to ensure USI written to PB1
            (0 << USICS1) | (1 << USICS0) | (0 << USICLK);  // Select Timer0 Compare match as USI Clock source

    DDRB |= (1 << PB1);                                     // Configure USI_DO as output
    USISR = (1 << USIOIF) | (16 - 8);                       // Clear USI overflow interrupt flag & set USI counter to count 8 bits
                                      
}


ISR(USI_OVF_vect)                                           // USI overflow interrupt indicates we have sent a buffer
{
    if (usiserial_send_get_state() == FIRST)
    {
        usiserial_send_set_state(SECOND);
        USIDR = (usiserial_get_tx_data() << 7) | 0x7F;      // Send last 1 bit of data & stop bits (high)
                                                   
        USISR = (1<<USIOIF) | (16 - (1 + (STOPBITS)));      // Clear USI overflow interrupt flag & Set USI counter to send last data bit and stop bits
    }
    else
    {
        PORTB |= (1 << PB1);                                // Ensure output is high
        DDRB  |= (1 << PB1);                                // Configure USI_DO as output.
        USICR  =  0;                                        // Disable USI.
        USISR |= 1 << USIOIF;                               // clear interrupt flag

        usiserial_send_set_state(AVAILABLE);
    }
}

void send_message(char message[])
{
    for(char i=0; i<sizeof(message)-1; i++)
    {
        while (!usiserial_send_available())
        {
            // Wait for last send to complete 
        }    
        usiserial_send_byte(message[i]);
    }
    _delay_ms(1000);
}


int main() 
{
    DDRB  |= (1 << PB1);                        // PB1 OUTPUT
    PORTB |= (1 << PB1);                        // Ensure serial output is high when idle
       
    send_message("AT+DEFAULT\r\n");
    send_message("AT+NAMEREMOTE\r\n");
    send_message("AT+ROLE0\r\n");

    _delay_ms(1000);

    return(1);
}

