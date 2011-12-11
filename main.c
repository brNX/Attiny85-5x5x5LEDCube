#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include "usbdrv/usbdrv.h"
#include "usbdrv/oddebug.h"

#define __LATCH_LOW PORTB &= ~(1 << PB0)
#define __LATCH_HIGH PORTB |= (1 << PB0)

void setup(void);
uint8_t spi_transfer(uint8_t data);
int main(void);
//uchar usbFunctionRead(uchar *data, uchar len);
//uchar usbFunctionWrite(uchar *data, uchar len);
//usbMsgLen_t usbFunctionSetup(uchar data[8]);
void    usbEventResetReady(void);



void setup(void) {

  // USI stuff
  DDRB |= _BV(PB1); // as output (DO)
  DDRB |= _BV(PB2); // as output (USISCK)
  DDRB |= _BV(PB0); // as output (DI)

//  //USB Stuff
//  uchar   i;
//  uchar   calibrationValue;
//
//  calibrationValue = eeprom_read_byte(0); /* calibration value from last time */
//  if(calibrationValue != 0xff){
//      OSCCAL = calibrationValue;
//  }
//  odDebugInit();
//  usbDeviceDisconnect();
//  for(i=0;i<20;i++){  /* 300 ms disconnect */
//      _delay_ms(15);
//  }
//  usbDeviceConnect();
//
//  wdt_enable(WDTO_1S);
//
//  usbInit();
  sei();

}

int main(void) {
	setup();
	for(;;) {
        /*wdt_reset();
        usbPoll();*/

  	  __LATCH_LOW;
  	    spi_transfer(0x01); // channel 1 active (red)
  	    spi_transfer(0x04); // channel 1 active (red)
  	  __LATCH_HIGH;
  	  _delay_ms(500);

  	  __LATCH_LOW;
  	    spi_transfer(0x02); // channel 2 active (green)
  	    spi_transfer(0x02); // channel 2 active (green)
  	  __LATCH_HIGH;
  	  _delay_ms(500);

  	  __LATCH_LOW;
  	    spi_transfer(0x04); // channel 3 active (blue)
  	    spi_transfer(0x01); // channel 3 active (blue)
  	  __LATCH_HIGH;
  	  _delay_ms(500);

  	  __LATCH_LOW;
  	    spi_transfer(0x07); // channels 1,2,3 active (white)
  	    spi_transfer(0x07); // channels 1,2,3 active (white)
  	  __LATCH_HIGH;
  	  _delay_ms(500);


  	  __LATCH_LOW;
  	    spi_transfer(0x00); // all outputs off
  	    spi_transfer(0x00); // all outputs off
  	  __LATCH_HIGH;
  	  _delay_ms(500);

	}

}

uint8_t spi_transfer(uint8_t data) {
  USIDR = data;
  USISR = _BV(USIOIF); // clear flag

  while ( (USISR & _BV(USIOIF)) == 0 ) {
   USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC);
  }
  return USIDR;
}

//// ----------------------------------------------------------------------
//// Handle an IN packet.
//// ----------------------------------------------------------------------
//uchar usbFunctionRead(uchar *data, uchar len)
//{
//	uchar	i;
//	return len;
//}
//
//// ----------------------------------------------------------------------
//// Handle an OUT packet.
//// ----------------------------------------------------------------------
//uchar usbFunctionWrite(uchar *data, uchar len)
//{
//	uchar	i;
//	unsigned	usec;
//	uchar	r;
//
//	//return last;
//	return 1;
//}
//
//
//usbMsgLen_t usbFunctionSetup(uchar data[8])
//{
//	return 0;
//}

/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */

/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
static void calibrateOscillator(void)
{
uchar       step = 128;
uchar       trialValue = 0, optimumValue;
int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

    /* do a binary search: */
    do{
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    /* proportional to current real frequency */
        if(x < targetValue)             /* frequency still too low */
            trialValue += step;
        step >>= 1;
    }while(step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; /* this is certainly far away from optimum */
    for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
        x = usbMeasureFrameLength() - targetValue;
        if(x < 0)
            x = -x;
        if(x < optimumDev){
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}

/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/

void    usbEventResetReady(void)
{
    calibrateOscillator();
    eeprom_write_byte(0, OSCCAL);   /* store the calibrated value in EEPROM */
}

