//avr-gcc -Wall -g -Os -mmcu=atmega328p -o ATmega328_SSD1306_I2C.bin ATmega328_SSD1306_I2C.c
//avr-objcopy -j .text -j .data -O ihex ATmega328_SSD1306_I2C.bin ATmega328_SSD1306_I2C.hex
//avrdude -p atmega328p -c usbasp -U flash:w:ATmega328_SSD1306_I2C.hex:i -F -P usb

#define F_CPU 1000000UL    // 1MHz
#define SCL_CLOCK 50000L   // I2C clock 50KHz
#define BAUD 2400         //
#define FOSC 1000000       // Clock Speed
#define MYUBRR FOSC/16/(BAUD-1)

#define I2C_READ    1      // defines the data direction (reading from I2C device) in i2c_start(),i2c_rep_start()
#define I2C_WRITE   0      // defines the data direction (writing to I2C device) in i2c_start(),i2c_rep_start()
     
#include <avr/io.h> 
#include <util/delay.h>
#include <util/twi.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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

// Set column and row position
#define SSD1306_SETLOWCOLUMN               0x00    // Set Lower Column Start Address for Page Addressing Mode
#define SSD1306_SETHIGHCOLUMN              0x10    // Set Higher Column Start Address for Page Addressing Mode

#define SSD1306_SETSTARTPAGE               0XB0    // Set GDDRAM Page Start Address

const char BasicFont[][8] PROGMEM = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x5F,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x07,0x00,0x07,0x00,0x00,0x00},
	{0x00,0x14,0x7F,0x14,0x7F,0x14,0x00,0x00},
	{0x00,0x24,0x2A,0x7F,0x2A,0x12,0x00,0x00},
	{0x00,0x23,0x13,0x08,0x64,0x62,0x00,0x00},
	{0x00,0x36,0x49,0x55,0x22,0x50,0x00,0x00},
	{0x00,0x00,0x05,0x03,0x00,0x00,0x00,0x00},
	{0x00,0x1C,0x22,0x41,0x00,0x00,0x00,0x00},
	{0x00,0x41,0x22,0x1C,0x00,0x00,0x00,0x00},
	{0x00,0x08,0x2A,0x1C,0x2A,0x08,0x00,0x00},
	{0x00,0x08,0x08,0x3E,0x08,0x08,0x00,0x00},
	{0x00,0xA0,0x60,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x08,0x08,0x08,0x08,0x08,0x00,0x00},
	{0x00,0x60,0x60,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x20,0x10,0x08,0x04,0x02,0x00,0x00},
	{0x00,0x3E,0x51,0x49,0x45,0x3E,0x00,0x00},
	{0x00,0x00,0x42,0x7F,0x40,0x00,0x00,0x00},
	{0x00,0x62,0x51,0x49,0x49,0x46,0x00,0x00},
	{0x00,0x22,0x41,0x49,0x49,0x36,0x00,0x00},
	{0x00,0x18,0x14,0x12,0x7F,0x10,0x00,0x00},
	{0x00,0x27,0x45,0x45,0x45,0x39,0x00,0x00},
	{0x00,0x3C,0x4A,0x49,0x49,0x30,0x00,0x00},
	{0x00,0x01,0x71,0x09,0x05,0x03,0x00,0x00},
	{0x00,0x36,0x49,0x49,0x49,0x36,0x00,0x00},
	{0x00,0x06,0x49,0x49,0x29,0x1E,0x00,0x00},
	{0x00,0x00,0x36,0x36,0x00,0x00,0x00,0x00},
	{0x00,0x00,0xAC,0x6C,0x00,0x00,0x00,0x00},
	{0x00,0x08,0x14,0x22,0x41,0x00,0x00,0x00},
	{0x00,0x14,0x14,0x14,0x14,0x14,0x00,0x00},
	{0x00,0x41,0x22,0x14,0x08,0x00,0x00,0x00},
	{0x00,0x02,0x01,0x51,0x09,0x06,0x00,0x00},
	{0x00,0x32,0x49,0x79,0x41,0x3E,0x00,0x00},
	{0x00,0x7E,0x09,0x09,0x09,0x7E,0x00,0x00},
	{0x00,0x7F,0x49,0x49,0x49,0x36,0x00,0x00},
	{0x00,0x3E,0x41,0x41,0x41,0x22,0x00,0x00},
	{0x00,0x7F,0x41,0x41,0x22,0x1C,0x00,0x00},
	{0x00,0x7F,0x49,0x49,0x49,0x41,0x00,0x00},
	{0x00,0x7F,0x09,0x09,0x09,0x01,0x00,0x00},
	{0x00,0x3E,0x41,0x41,0x51,0x72,0x00,0x00},
	{0x00,0x7F,0x08,0x08,0x08,0x7F,0x00,0x00},
	{0x00,0x41,0x7F,0x41,0x00,0x00,0x00,0x00},
	{0x00,0x20,0x40,0x41,0x3F,0x01,0x00,0x00},
	{0x00,0x7F,0x08,0x14,0x22,0x41,0x00,0x00},
	{0x00,0x7F,0x40,0x40,0x40,0x40,0x00,0x00},
	{0x00,0x7F,0x02,0x0C,0x02,0x7F,0x00,0x00},
	{0x00,0x7F,0x04,0x08,0x10,0x7F,0x00,0x00},
	{0x00,0x3E,0x41,0x41,0x41,0x3E,0x00,0x00},
	{0x00,0x7F,0x09,0x09,0x09,0x06,0x00,0x00},
	{0x00,0x3E,0x41,0x51,0x21,0x5E,0x00,0x00},
	{0x00,0x7F,0x09,0x19,0x29,0x46,0x00,0x00},
	{0x00,0x26,0x49,0x49,0x49,0x32,0x00,0x00},
	{0x00,0x01,0x01,0x7F,0x01,0x01,0x00,0x00},
	{0x00,0x3F,0x40,0x40,0x40,0x3F,0x00,0x00},
	{0x00,0x1F,0x20,0x40,0x20,0x1F,0x00,0x00},
	{0x00,0x3F,0x40,0x38,0x40,0x3F,0x00,0x00},
	{0x00,0x63,0x14,0x08,0x14,0x63,0x00,0x00},
	{0x00,0x03,0x04,0x78,0x04,0x03,0x00,0x00},
	{0x00,0x61,0x51,0x49,0x45,0x43,0x00,0x00},
	{0x00,0x7F,0x41,0x41,0x00,0x00,0x00,0x00},
	{0x00,0x02,0x04,0x08,0x10,0x20,0x00,0x00},
	{0x00,0x41,0x41,0x7F,0x00,0x00,0x00,0x00},
	{0x00,0x04,0x02,0x01,0x02,0x04,0x00,0x00},
	{0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00},
	{0x00,0x01,0x02,0x04,0x00,0x00,0x00,0x00},
	{0x00,0x20,0x54,0x54,0x54,0x78,0x00,0x00},
	{0x00,0x7F,0x48,0x44,0x44,0x38,0x00,0x00},
	{0x00,0x38,0x44,0x44,0x28,0x00,0x00,0x00},
	{0x00,0x38,0x44,0x44,0x48,0x7F,0x00,0x00},
	{0x00,0x38,0x54,0x54,0x54,0x18,0x00,0x00},
	{0x00,0x08,0x7E,0x09,0x02,0x00,0x00,0x00},
	{0x00,0x18,0xA4,0xA4,0xA4,0x7C,0x00,0x00},
	{0x00,0x7F,0x08,0x04,0x04,0x78,0x00,0x00},
	{0x00,0x00,0x7D,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x80,0x84,0x7D,0x00,0x00,0x00,0x00},
	{0x00,0x7F,0x10,0x28,0x44,0x00,0x00,0x00},
	{0x00,0x41,0x7F,0x40,0x00,0x00,0x00,0x00},
	{0x00,0x7C,0x04,0x18,0x04,0x78,0x00,0x00},
	{0x00,0x7C,0x08,0x04,0x7C,0x00,0x00,0x00},
	{0x00,0x38,0x44,0x44,0x38,0x00,0x00,0x00},
	{0x00,0xFC,0x24,0x24,0x18,0x00,0x00,0x00},
	{0x00,0x18,0x24,0x24,0xFC,0x00,0x00,0x00},
	{0x00,0x00,0x7C,0x08,0x04,0x00,0x00,0x00},
	{0x00,0x48,0x54,0x54,0x24,0x00,0x00,0x00},
	{0x00,0x04,0x7F,0x44,0x00,0x00,0x00,0x00},
	{0x00,0x3C,0x40,0x40,0x7C,0x00,0x00,0x00},
	{0x00,0x1C,0x20,0x40,0x20,0x1C,0x00,0x00},
	{0x00,0x3C,0x40,0x30,0x40,0x3C,0x00,0x00},
	{0x00,0x44,0x28,0x10,0x28,0x44,0x00,0x00},
	{0x00,0x1C,0xA0,0xA0,0x7C,0x00,0x00,0x00},
	{0x00,0x44,0x64,0x54,0x4C,0x44,0x00,0x00},
	{0x00,0x08,0x36,0x41,0x00,0x00,0x00,0x00},
	{0x00,0x00,0x7F,0x00,0x00,0x00,0x00,0x00},
	{0x00,0x41,0x36,0x08,0x00,0x00,0x00,0x00},
	{0x00,0x02,0x01,0x01,0x02,0x01,0x00,0x00},
	{0x00,0x02,0x05,0x05,0x02,0x00,0x00,0x00} 
};


//----------I2C---------------------------------------

void i2c_init()                       // I2C initialize function
{
    TWSR = 0x00;                      // status register prescaler value 1 → TWPS1 = 0 and TWPS0 = 0
    TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  // prescale to 100000kHz
}

unsigned char i2c_start(unsigned char address)
{    
    uint8_t twst;

	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);               // send START condition

	while(!(TWCR & (1<<TWINT)));                              // wait until transmission completed

	twst = TW_STATUS & 0xF8;                                  // check value of TWI Status Register. Mask prescaler bits.
	if ( (twst != TW_START) && (twst != TW_REP_START))        // 
        return 1;                                             //    

	TWDR = address;                                           // send device address
	TWCR = (1<<TWINT) | (1<<TWEN);	                          // Enable TWI and clear interrupt flag

	while(!(TWCR & (1<<TWINT)));                              // wail until transmission completed and ACK/NACK has been received    

	twst = TW_STATUS & 0xF8;                                  // check value of TWI Status Register. Mask prescaler bits
	if((twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK)) 
        return 1;
    
	return 0;
}

void i2c_start_wait(unsigned char address)
{
    uint8_t twst;
    while(1)
    {
	    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);              // send START condition
    	while(!(TWCR & (1<<TWINT)));                             // wait until transmission completed        	
    	twst = TW_STATUS & 0xF8;                                 // check value of TWI Status Register. Mask prescaler bits.
    	if ( (twst != TW_START) && (twst != TW_REP_START))       //
            continue;                                            //        	
    	TWDR = address;                                          // send device address
    	TWCR = (1<<TWINT) | (1<<TWEN);    	                     // Enable TWI and clear interrupt flag
    	while(!(TWCR & (1<<TWINT)));                             // wail until transmission completed        	
    	twst = TW_STATUS & 0xF8;                                 // check value of TWI Status Register. Mask prescaler bits
    	if((twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK)) 
    	{          	    
	        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);          // device busy, send stop condition to terminate write operation	        	        
	        while(TWCR & (1<<TWSTO));                            // wait until stop condition is executed and bus released	        
    	    continue;
    	}
    	//if( twst != TW_MT_SLA_ACK) return 1;
        
        break;
    }   
}

void i2c_stop(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO); // send stop condition
	while(TWCR & (1<<TWSTO));                   // wait until stop condition is executed and bus released
}

unsigned char i2c_write(unsigned char data)
{	
    uint8_t   twst;
	TWDR = data;                                // send data to the previously addressed device
    TWCR = (1<<TWINT) | (1<<TWEN);	            // Enable TWI and clear interrupt flag
	while(!(TWCR & (1<<TWINT)));                // wait until transmission completed
	twst = TW_STATUS & 0xF8;                    // check value of TWI Status Register. Mask prescaler bits
	if( twst != TW_MT_DATA_ACK)                 //
        return 1;                               //
    
	return 0;
}

//----------SSD1306---------------------------------------

void setRow(uint8_t row) 
{
    i2c_start_wait((SSD1306_ADDR<<1) + I2C_WRITE);
	i2c_write(SSD1306_CONTROL_BYTE_CMD_SINGLE);
    i2c_write(SSD1306_SETSTARTPAGE | row);           // set page address
    //i2c_stop();
}

void setCol(uint8_t col)
{  
    i2c_start_wait((SSD1306_ADDR<<1) + I2C_WRITE);
	i2c_write(SSD1306_CONTROL_BYTE_CMD_STREAM);
    i2c_write(SSD1306_SETLOWCOLUMN  | (col &  0XF));   // set column lower address
    i2c_write(SSD1306_SETHIGHCOLUMN | (col >>   4));   // set column higher address
    i2c_stop();
}

void setCursor(char row, char col)
{
	setRow(row); // row = 1 page(8 pixel rows) 
    setCol(col); // col = 8 pixel columns
}

void printChar(char C)
{
    if(C<32 || C>127) C = '*';                         // Ignore unused ASCII characters - * indicate characters that can't be displayed
    i2c_start_wait((SSD1306_ADDR<<1) + I2C_WRITE);
	i2c_write(SSD1306_CONTROL_BYTE_DATA_STREAM);
    for(int i=0; i<8; i++) 
    {	
        i2c_write(pgm_read_byte(&BasicFont[C-32][i])); // font array starts at 0, ASCII starts at 32. Hence the translation       
    }
    i2c_stop();
}

void printString(const char *String)
{	
	uint8_t count = 0;
    uint8_t numChar = strlen(String);
    while(String[count] && count<numChar) printChar(String[count++]);
}

void clearScreen()
{    
    i2c_start_wait((SSD1306_ADDR<<1) + I2C_WRITE);
    i2c_write(SSD1306_CONTROL_BYTE_DATA_STREAM);
    
    for(uint16_t i=0;i<64;i++)
    {		
	    for (uint8_t j=0; j<128; j++) 
        {	
            i2c_write(0x00);
		    j++;
	    }
	    i++;	      
	}
    i2c_stop();
}

void clearLine()
{
    i2c_start_wait((SSD1306_ADDR<<1) + I2C_WRITE);
    i2c_write(SSD1306_CONTROL_BYTE_DATA_STREAM);
    for(uint8_t i=0;i<128;i++) i2c_write(0x00);
    i2c_stop(); 
}

void drawHLine(uint8_t x,uint8_t y,uint8_t length,uint8_t draw) //draw: 1=draw - 0=erase
{
    setCursor(y,x);
    i2c_start_wait((SSD1306_ADDR<<1) + I2C_WRITE);
    i2c_write(SSD1306_CONTROL_BYTE_DATA_STREAM);
    for(uint8_t i=0; i<length; i++) 
    {	
        i2c_write(1*draw);
	}
    i2c_stop();
}

void drawVLine(uint8_t x,uint8_t y,uint8_t draw)
{
    setCursor(y,x);
    i2c_start_wait((SSD1306_ADDR<<1) + I2C_WRITE);
    i2c_write(SSD1306_CONTROL_BYTE_DATA_STREAM);
    i2c_write(255*draw);
    i2c_stop();
}
           
void drawRect(uint8_t x1,uint8_t x2,uint8_t y1,uint8_t y2,uint8_t pen) //pen: 1=draw - 0=erase
{    
    setCursor(y1,x1);
    i2c_start_wait((SSD1306_ADDR<<1) + I2C_WRITE);
    i2c_write(SSD1306_CONTROL_BYTE_DATA_STREAM);
    for(uint8_t j=x1; j<=x2; j++) 
    {	
        i2c_write(1*pen);
	}
    i2c_stop();
    
    for(uint8_t j=y1; j<y2; j++) 
    {	
        setCursor(j,x1);
        i2c_start_wait((SSD1306_ADDR<<1) + I2C_WRITE);
        i2c_write(SSD1306_CONTROL_BYTE_DATA_STREAM);
        i2c_write(255*pen);
        i2c_stop();		
	}

    for(uint8_t j=y1; j<y2; j++) 
    {	
        setCursor(j,x2);
        i2c_start_wait((SSD1306_ADDR<<1) + I2C_WRITE);
        i2c_write(SSD1306_CONTROL_BYTE_DATA_STREAM);
        i2c_write(255*pen);
        i2c_stop();		
	}

    setCursor(y2,x1);
    i2c_start_wait((SSD1306_ADDR<<1) + I2C_WRITE);
    i2c_write(SSD1306_CONTROL_BYTE_DATA_STREAM);
    for(uint8_t j=x1; j<=x2; j++) 
    {	
        i2c_write(1*pen);
	}
    i2c_stop();
}

void dashboard()
{        
    int t = 1;
    /*while(1)
    {    
                      drawRect( 17, 24,0,6,1); drawRect( 59, 66,1,6,1); drawRect(101,108,2,6,1); 
        _delay_ms(t); drawRect( 17, 24,0,6,0); drawRect( 59, 66,1,6,0); drawRect(101,108,2,6,0);
                      setCursor(7,9); printString("X100"); setCursor(7,50); printString("Y200"); setCursor(7,93); printString("Z300");

                      drawRect( 17, 24,3,6,1); drawRect( 59, 66,2,6,1); drawRect(101,108,4,6,1);
        _delay_ms(t); drawRect( 17, 24,3,6,0); drawRect( 59, 66,2,6,0); drawRect(101,108,4,6,0);
                      setCursor(7,9); printString("X110"); setCursor(7,50); printString("Y220"); setCursor(7,93); printString("Z330");

                      drawRect( 17, 24,2,6,1); drawRect( 59, 66,4,6,1); drawRect(101,108,3,6,1);
        _delay_ms(t); drawRect( 17, 24,2,6,0); drawRect( 59, 66,4,6,0); drawRect(101,108,3,6,0);
                      setCursor(7,9); printString("X111"); setCursor(7,50); printString("Y222"); setCursor(7,93); printString("Z333");
    }*/
    /*while(1)
    {    
                      drawRect(  4,5,0,6,1); drawRect(  60,61,2,6,1); drawRect(  100,101,4,6,1); 
        _delay_ms(t); drawRect(  4,5,0,6,0); drawRect(  60,61,2,6,0); drawRect(  100,101,4,6,0);
                      setCursor(7,0); printString("X100"); setCursor(7,40); printString("Y200"); setCursor(7,80); printString("Z300");

                      drawRect(  4,5,4,6,1); drawRect(  60,61,0,6,1); drawRect(  100,101,2,6,1); 
        _delay_ms(t); drawRect(  4,5,4,6,0); drawRect(  60,61,0,6,0); drawRect(  100,101,2,6,0);
                      setCursor(7,9); printString("X19"); setCursor(7,50); printString("Y86"); setCursor(7,93); printString("Z67");

                      drawRect(  4,5,5,6,1); drawRect(  60,61,4,6,1); drawRect(  100,101,3,6,1); 
        _delay_ms(t); drawRect(  4,5,5,6,0); drawRect(  60,61,4,6,0); drawRect(  100,101,3,6,0);
                      setCursor(7,9); printString("X89"); setCursor(7,50); printString("Y94"); setCursor(7,93); printString("Z34");
    }*/
    /*while(1)
    {
        
                      drawHLine(1,1,120,1); drawHLine(1,3, 10,1); drawHLine(1,5, 80,1);
        _delay_ms(t); drawHLine(1,1,120,0); drawHLine(1,3, 10,0); drawHLine(1,5, 80,0);
                      setCursor(0,0); printString("X120"); setCursor(7,40); printString("Y10"); setCursor(7,80); printString("Z80");

                      drawHLine(1,1, 40,1); drawHLine(1,3, 70,1); drawHLine(1,5,120,1);
        _delay_ms(t); drawHLine(1,1, 40,0); drawHLine(1,3, 70,0); drawHLine(1,5,120,0);
                      setCursor(0,0); printString("X 40"); setCursor(7,40); printString("Y70"); setCursor(7,80); printString("Z120");

                      drawHLine(1,1, 80,1); drawHLine(1,3,120,1); drawHLine(1,5, 25,1);
        _delay_ms(t); drawHLine(1,1, 80,0); drawHLine(1,3,120,0); drawHLine(1,5, 25,0);
                      setCursor(0,0); printString("X 80"); setCursor(7,40); printString("Y120"); setCursor(7,80); printString("Z80");
    }*/
    while(1)
    {
    for(uint8_t i=0; i<120; i++)
    {        
        drawVLine(i,2,1);
        drawVLine(i,3,1);
        drawVLine(i,4,1);
        drawVLine(i,5,1);
        _delay_ms(t);                      
    }
    for(uint8_t i=120; i>0; i--)
    {        
        drawVLine(i,2,0);
        drawVLine(i,3,0);
        drawVLine(i,4,0);
        drawVLine(i,5,0);
        _delay_ms(t);                      
    }
    }                  
}

void SSD1306_Init() 
{
    i2c_start((SSD1306_ADDR<<1) + I2C_WRITE);      // I2C_WRITE = 0 = I2C write mode

    i2c_write(SSD1306_CONTROL_BYTE_CMD_STREAM);    // Tell the SSD1306 that a command stream is incoming

    i2c_write(SSD1306_CMD_DISPLAY_OFF);            // Turn the Display OFF
    
    i2c_write(SSD1306_CMD_SET_MUX_RATIO);          // Set multiplex ratio to 63
    i2c_write(0x3F);                               //
    
	i2c_write(SSD1306_CMD_SET_DISPLAY_OFFSET);     // Set the display offset to 0
	i2c_write(0x00);                               //
    
	i2c_write(SSD1306_CMD_SET_DISPLAY_START_LINE); // Display start line to 0

    i2c_write(SSD1306_CMD_SET_SEGMENT_REMAP);      // Set segment Re-map → 0xA1 in case pins are north - 0xA0 in case pins are south (default) 

    i2c_write(SSD1306_CMD_SET_COM_SCAN_MODE);      // Set COM Output Scan Direction → 0xC8 in case pins are north - 0xC0 in case pins are south(default)
    
	i2c_write(SSD1306_CMD_SET_COM_PIN_MAP);        // Default - alternate COM pin map
	i2c_write(0x12);                               //
	
	i2c_write(SSD1306_CMD_SET_CONTRAST);           // set contrast
	i2c_write(0x7F);                               //

	i2c_write(SSD1306_CMD_DISPLAY_RAM);            // Set display to enable rendering from GDDRAM (Graphic Display Data RAM)

	i2c_write(SSD1306_CMD_DISPLAY_NORMAL);         // Set display Normal mode

	i2c_write(SSD1306_CMD_SET_DISPLAY_CLOCK_DIV);  // Default oscillator clock
	i2c_write(0x80);                               //

	i2c_write(SSD1306_CMD_SET_CHARGE_PUMP);        // Enable the charge pump
	i2c_write(0x14);                               //
	
	i2c_write(SSD1306_CMD_SET_PRECHARGE);          // Set precharge cycles to high cap type
	i2c_write(0x22);                               //

	i2c_write(SSD1306_CMD_SET_VCOMH_DESELCT);      // Set the V_COMH deselect volatage to max 
	i2c_write(0x30);                               //

	i2c_write(SSD1306_CMD_SET_MEMORY_ADDR_MODE);   // Horizonatal addressing mode - same as the KS108 GLCD
	i2c_write(0x00);                               //
	
	i2c_write(SSD1306_CMD_DISPLAY_ON);             // Turn the Display ON

	i2c_write(SSD1306_CMD_SET_COLUMN_RANGE);       // Set column range
	i2c_write(0x00);                               //
	i2c_write(0x7F);                               //
	
    i2c_write(SSD1306_CMD_SET_PAGE_RANGE);         // Set Row (Page) range
	i2c_write(0);                                  //
	i2c_write(0x07);                               //
	
	i2c_stop();                                    // End the I2C communication with the SSD1306
}

//----------UART---------------------------------------

void init_uart(void)
{
   
   	unsigned int ubrr = 25;                         // Set baud rate 2400
    UBRR0H = (ubrr>>8);
	UBRR0L = (ubrr);
    
    UCSR0B = (1<<TXEN0) | (1<<RXEN0) | (1<<RXCIE0); // Enable TX - Enable RX - Enable RX INTERRUPT	
	              
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);             // Set frame format: 8data, 1stop bit
}


char usart_getchar(void)
{   
    while( !(UCSR0A & (1<<RXC0)) ) ; // Wait for data to be received
    return UDR0;                     // Get and return received data from buffer    
}


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

volatile char data[25];
uint8_t count = 0;
unsigned char c;

/*ISR(USART_RX_vect)
{                   
    c = UDR0;    
    if(c=='>'){UDR0=c; printChar(c); clearLine(); setCursor(0,0);}
    else      {UDR0=c; printChar(c);}
}*/

ISR(USART_RX_vect)
{                   
    data[count] = UDR0;    
    if(data[count]=='>'){UDR0=data[count]; printChar(data[count]); clearLine(); setCursor(0,0); count = 0;}
    else                {UDR0=data[count]; printChar(data[count]);                              count++  ;}    
}   

//-------------------------------------------
char strN[6], strG[6], strX[6], strY[6], strZ[6], strF[6], strS[6], strT[6], strM[6];

void parseGcode(char buf[25])
{
    //N## G## X## Y## Z## F## S## T## M##
    char *token = strtok(buf," ");
    while(token != NULL ) //WHILE THERE ARE TOKENS IN STRING
    {   
        if     (token[0] == 'N') strcpy(strN, &token[1]);
        else if(token[0] == 'G') strcpy(strG, &token[1]);
        else if(token[0] == 'X') strcpy(strX, &token[1]);
        else if(token[0] == 'Y') strcpy(strY, &token[1]);         
        else if(token[0] == 'Z') strcpy(strZ, &token[1]);
        else if(token[0] == 'F') strcpy(strF, &token[1]);
        else if(token[0] == 'S') strcpy(strS, &token[1]);         
        else if(token[0] == 'T') strcpy(strT, &token[1]);
        else if(token[0] == 'M') strcpy(strM, &token[1]);
        
        token = strtok(NULL," "); //GET NEXT TOKEN
    }

    printf("%d\n",atoi(strX));    

    //VIDER LES BUFFERS        
    memset((void *)buf,'\0',sizeof(data));
    memset((void *)strX,'\0',sizeof(strX));
    memset((void *)strY,'\0',sizeof(strY));
    memset((void *)strZ,'\0',sizeof(strZ));
}

void useGcode()
{

}

void GcodeToMotor()
{           
    uint16_t n = ((strX[2]-'0') * 100) + ((strX[3]-'0')*10) + (strX[4]-'0');

    if     (strX[1]=='-') for(uint16_t i=0; i<=n ; i++) /*halfStepping(sensAntiHoraire)*/;    
    else if(strX[1]=='0') for(uint16_t i=0; i<=n ; i++) /*halfStepping(sensHoraire    )*/;                    
}

int main(void)
{
    i2c_init();
    SSD1306_Init();    
    init_uart();
    clearScreen();    
    sei(); 
    
    parseGcode("X123 Y456 Z789");
    while(1){}
        
    //dashboard();       

    return 0;
}




