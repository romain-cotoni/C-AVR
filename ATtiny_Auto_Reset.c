#include <avr/wdt.h>

int main(void) 
{
    MCUSR = 0;
    wdt_disable();

    /* faire quelque chose */
    
    wdt_enable(WDTO_15MS);
    while(true){}             //la boucle va tourner pendant 15ms 
}


