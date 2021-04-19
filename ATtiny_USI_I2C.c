#include <avr/io.h>
#include <util/delay.h>

// Minimal Tiny I2C Routines **********************************************

int I2Ccount;

uint8_t transfer (uint8_t data) 
{
    USISR = data;                               // Set USISR according to data.
                                                // Prepare clocking.
    data = 0<<USISIE | 0<<USIOIE |              // Interrupts disabled
           1<<USIWM1 | 0<<USIWM0 |              // Set USI in Two-wire mode.
           1<<USICS1 | 0<<USICS0 | 1<<USICLK |  // Software clock strobe as source.
           1<<USITC;                            // Toggle Clock Port.
    do 
    {
        _delay_us(5);
        USICR = data;                           // Generate positive SCL edge.
        while (!(PINB & 1<<PB2));               // Wait for SCL to go high.
        _delay_us(4);
        USICR = data;                           // Generate negative SCL edge.
    }while (!(USISR & 1<<USIOIF));              // Check for transfer complete.

    _delay_us(5);
    data = USIDR;                               // Read out data.
    USIDR = 0xFF;                               // Release SDA.
    DDRB |= (1<<PB0);                           // Enable SDA as output.

    return data;                                // Return the data from the USIDR
}

void USI_I2C_init() 
{
    PORTB |= 1<<PB0;                                      // Enable pullup on SDA.
    PORTB |= 1<<PB2;                                      // Enable pullup on SCL.

    DDRB |= 1<<PB2;                                       // Enable SCL as output.
    DDRB |= 1<<PB0;                                       // Enable SDA as output.

    USIDR = 0xFF;                                         // Preload data register with "released level" data.
    USICR = 0<<USISIE | 0<<USIOIE |                       // Disable Interrupts.
            1<<USIWM1 | 0<<USIWM0 |                       // Set USI in Two-wire mode.
            1<<USICS1 | 0<<USICS0 | 1<<USICLK |           // Software stobe as counter clock source
            0<<USITC;
    USISR = 1<<USISIF | 1<<USIOIF | 1<<USIPF | 1<<USIDC | // Clear flags,
            0x0<<USICNT0;                                 // and reset counter.
}

uint8_t read(void) 
{
    if ((I2Ccount != 0) && (I2Ccount != -1)) I2Ccount--;
  
    /* Read a byte */
    DDRB &= ~1<<PB0;                                       // Enable SDA as input.
    uint8_t data = transfer(USISR_8bit);

    /* Prepare to generate ACK (or NACK in case of End Of Transmission) */
    if (I2Ccount == 0) USIDR = 0xFF; 
    else USIDR = 0x00;
    transfer(USISR_1bit);                                  // Generate ACK/NACK.

    return data;                                           // Read successfully completed
}



bool write (uint8_t data) 
{
    /* Write a byte */
    PORTB &= ~(1<<PB2);                                       // Pull SCL LOW.
    USIDR = data;                                             // Setup data.
    transfer(USISR_8bit);                                     // Send 8 bits on bus.

    /* Clock and verify (N)ACK from slave */
    DDRB &= ~(1<<PB0);                                     // Enable SDA as input.
    if (transfer(USISR_1bit) & 1<<TWI_NACK_BIT) return false;

    return true;                                              // Write successfully completed
}


// Start transmission by sending address
bool start (uint8_t address, int readcount) 
{
    if (readcount != 0) { I2Ccount = readcount; readcount = 1; }
    uint8_t addressRW = address<<1 | readcount;

    /* Release SCL to ensure that (repeated) Start can be performed */
    PORTB |= 1<<PB2;                                          // Release SCL.
    while (!(PINB & 1<<PB2));                                 // Verify that SCL becomes high.
    _delay_us(5);


    /* Generate Start Condition */
    PORTB &= ~(1<<PB0);                                       // Force SDA LOW.
    _delay_us(4);
    PORTB &= ~(1<<PB2);                                       // Pull SCL LOW.
    PORTB |= 1<<PB0;                                          // Release SDA.

    if (!(USISR & 1<<USISIF)) return false;

    /*Write address */
    PORTB &= ~(1<<PB2);                                       // Pull SCL LOW.
    USIDR = addressRW;                                        // Setup data.
    transfer(USISR_8bit);                                     // Send 8 bits on bus.

    /* Clock and verify (N)ACK from slave */
    DDRB &= ~(1<<PB0);                                     // Enable SDA as input.
    if (transfer(USISR_1bit) & 1<<TWI_NACK_BIT) return false; // No ACK

    return true;                                              // Start successfully completed
}


void stop (void) 
{
    PORTB &= ~(1<<PB0);           // Pull SDA low.
    PORTB |= 1<<PB2;              // Release SCL.
    while (!(PINB & 1<<PB2));     // Wait for SCL to go high.
    _delay_us(4);
    PORTB |= 1<<PB0;              // Release SDA.
    _delay_us(5);
}

