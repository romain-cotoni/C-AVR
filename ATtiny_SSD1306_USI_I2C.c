#include <avr/io.h>
#include <util/delay.h>

#define TWI_NACK_BIT 0     // Bit position for (N)ACK bit.

const unsigned char USISR_8bit = 1<<USISIF | 1<<USIOIF | 1<<USIPF | 1<<USIDC | 0x0<<USICNT0; // Prepare register value to: Clear flags, and set USI to shift 8 bits

const unsigned char USISR_1bit = 1<<USISIF | 1<<USIOIF | 1<<USIPF | 1<<USIDC | 0xE<<USICNT0; // Prepare register value to: Clear flags, and set USI to shift 1 bit 

int I2Ccount;

// Slave adress of the SSD1306
#define SSD1306_ADDR                       0x3C

// Control byte
#define SSD1306_CONTROL_BYTE_CMD_SINGLE    0x80
#define SSD1306_CONTROL_BYTE_CMD_STREAM    0x00
#define SSD1306_CONTROL_BYTE_DATA_STREAM   0x40

// Fundamental commands
#define SSD1306_CMD_SET_CONTRAST           0x81    // follow with 0x7F
#define SSD1306_CMD_DISPLAY_RAM            0xA4
#define SSD1306_CMD_DISPLAY_ALLON          0xA5
#define SSD1306_CMD_DISPLAY_NORMAL         0xA6
#define SSD1306_CMD_DISPLAY_INVERTED       0xA7
#define SSD1306_CMD_DISPLAY_OFF            0xAE
#define SSD1306_CMD_DISPLAY_ON             0xAF

// Addressing Command Table
#define SSD1306_CMD_SET_MEMORY_ADDR_MODE   0x20    // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define SSD1306_CMD_SET_COLUMN_RANGE       0x21    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
#define SSD1306_CMD_SET_PAGE_RANGE         0x22    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7

// Hardware Config
#define SSD1306_CMD_SET_DISPLAY_START_LINE 0x40
#define SSD1306_CMD_SET_SEGMENT_REMAP      0xA1    
#define SSD1306_CMD_SET_MUX_RATIO          0xA8    // follow with 0x3F = 64 MUX
#define SSD1306_CMD_SET_COM_SCAN_MODE      0xC8    
#define SSD1306_CMD_SET_DISPLAY_OFFSET     0xD3    // follow with 0x00
#define SSD1306_CMD_SET_COM_PIN_MAP        0xDA    // follow with 0x12
#define SSD1306_CMD_NOP                    0xE3    // NOP

// Timing and Driving Scheme
#define SSD1306_CMD_SET_DISPLAY_CLOCK_DIV  0xD5    // follow with 0x80
#define SSD1306_CMD_SET_PRECHARGE          0xD9    // follow with 0xF1
#define SSD1306_CMD_SET_VCOMH_DESELCT      0xDB    // follow with 0x30

// Charge Pump
#define SSD1306_CMD_SET_CHARGE_PUMP        0x8D    // follow with 0x14


void setup()
{
    USI_I2C_init();
    SSD1306_init();    
}

void loop()
{
    start(SSD1306_ADDR, 0);

	write(SSD1306_CONTROL_BYTE_CMD_STREAM);

	write(SSD1306_CMD_SET_COLUMN_RANGE);
	write(0x00);
	write(0x7F);

	write(SSD1306_CMD_SET_PAGE_RANGE);
	write(0);
	write(0x07);

	stop();

	for(uint16_t i=0;i<1024;i++)
    {
	    start(SSD1306_ADDR, 0);
		write(SSD1306_CONTROL_BYTE_DATA_STREAM);
	    for (uint8_t x=0; x<16; x++) 
        {	
		    // write(0b11000001);	
			// write(0x81);	
			// write(0x02);
			write(pattern1[x]);
			// write(pattern2[x]);	
			i++;
	    }
	    i--;
	    stop();   
	}

    _delay_ms(5000);

}

void SSD1306_init()
{
    start(SSD1306_ADDR, 0);                 // 0 = I2C write mode

    write(SSD1306_CONTROL_BYTE_CMD_STREAM); // Tell the SSD1306 that a command stream is incoming

    write(SSD1306_CMD_DISPLAY_OFF);         // Turn the Display OFF

    // Set multiplex ratio to 63
    write(SSD1306_CMD_SET_MUX_RATIO);
    write(0x3F);

    // Set the display offset to 0
	write(SSD1306_CMD_SET_DISPLAY_OFFSET); 
	write(0x00);   
    
	Wire.write(SSD1306_CMD_SET_DISPLAY_START_LINE); // Display start line to 0

    write(SSD1306_CMD_SET_SEGMENT_REMAP); // Set segment Re-map → 0xA1 in case pins are north - 0xA0 in case pins are south (default) 

    write(SSD1306_CMD_SET_COM_SCAN_MODE); // Set COM Output Scan Direction → 0xC8 in case pins are north - 0xC0 in case pins are south(default)

    // Default - alternate COM pin map
	write(SSD1306_CMD_SET_COM_PIN_MAP);
	write(0x12);

	// set contrast
	write(SSD1306_CMD_SET_CONTRAST);
	write(0x7F);

	// Set display to enable rendering from GDDRAM (Graphic Display Data RAM)
	write(SSD1306_CMD_DISPLAY_RAM);

	// Set display Normal mode
	write(SSD1306_CMD_DISPLAY_NORMAL);

	// Default oscillator clock
	write(SSD1306_CMD_SET_DISPLAY_CLK_DIV);
	write(0x80);

	// Enable the charge pump
	write(SSD1306_CMD_SET_CHARGE_PUMP);
	write(0x14);

	// Set precharge cycles to high cap type
	write(SSD1306_CMD_SET_PRECHARGE);
	write(0x22);

	// Set the V_COMH deselect volatage to max
	write(SSD1306_CMD_SET_VCOMH_DESELCT);
	write(0x30);

	// Horizonatal addressing mode - same as the KS108 GLCD
	write(SSD1306_CMD_SET_MEMORY_ADDR_MODE);
	write(0x00);
	
	write(SSD1306_CMD_DISPLAY_ON); // Turn the Display ON
	
	stop(); // End the I2C communication with the SSD1306
}


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

