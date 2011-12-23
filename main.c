#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <string.h>

#include "usbdrv/usbdrv.h"
#include "usbdrv/oddebug.h"

PROGMEM char usbHidReportDescriptor[22] = {    /* USB report descriptor */
    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x14,                    //   REPORT_COUNT (20)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0xc0                           // END_COLLECTION
};

#define __LATCH_LOW PORTB &= ~(1 << PB0)
#define __LATCH_HIGH PORTB |= (1 << PB0)

void setup(void);
uint8_t spi_transfer(uint8_t data);
int main(void);

uchar usbFunctionRead(uchar *data, uchar len);
uchar usbFunctionWrite(uchar *data, uchar len);
usbMsgLen_t usbFunctionSetup(uchar data[8]);
void    usbEventResetReady(void);

/* The following variables store the status of the current data transfer */
static uchar    currentAddress;
static uchar    bytesRemaining;

//framebuffer
volatile uchar buffer[21]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile uint8_t currentbuffer=0;

// What layer the interrupt routine is currently showing.
volatile uchar current_layer=0;



static void initTimers(){
	//enable interrupt TimerCounter0 compare match A
	TIMSK = _BV(OCIE0A);

	//setting CTC
	TCCR0A = _BV(WGM01);

	//Timer0 Settings: Timer Prescaler /1024,
	TCCR0B = _BV(CS00) | _BV(CS02);

	//top of the counters (1239hz)
	OCR0A=0x1A;
}


//timer interrupt 0
ISR(TIMER0_COMPA_vect)
{

	unsigned char layer = current_layer;
	int shift = (4*layer);

	__LATCH_LOW;
	spi_transfer(buffer[3+shift]);
	spi_transfer(buffer[2+shift]);
	spi_transfer(buffer[1+shift]);
	spi_transfer(buffer[0+shift]);
	__LATCH_HIGH;

	if (layer++ == 4){
		layer=0;
	}
	current_layer=layer;

}

void setup(void) {

	// USI stuff
	DDRB |= _BV(PB1); // as output (DO)
	DDRB |= _BV(PB2); // as output (USISCK)
	DDRB |= _BV(PB0); // as output (DI)

	// USB Stuff
	uchar   i;
	uchar   calibrationValue;

	calibrationValue = eeprom_read_byte(0); /* calibration value from last time */
	if(calibrationValue != 0xff){
		OSCCAL = calibrationValue;
	}

	wdt_enable(WDTO_1S);

	odDebugInit();
	DBG1(0x00, 0, 0);       // debug output: main starts
	usbInit();
	usbDeviceDisconnect();  // enforce re-enumeration, do this while interrupts are disabled!
	i = 0;
	while(--i){         /* fake USB disconnect for > 250 ms */
		wdt_reset();
		_delay_ms(1);
	}
	usbDeviceConnect();

	//TIMERS
	initTimers();
	sei();

}

uint8_t spi_transfer(uint8_t data) {
  USIDR = data;
  USISR = _BV(USIOIF); // clear flag

  while ( (USISR & _BV(USIOIF)) == 0 ) {
   USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC);
  }
  return USIDR;
}

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

/* usbFunctionWrite() is called when the host sends a chunk of data to the
 * device. For more information see the documentation in usbdrv/usbdrv.h.
 */
uchar   usbFunctionWrite(uchar *data, uchar len)
{
	if(bytesRemaining == 0){
		currentbuffer^=1;//toggle
		return 1;               /* end of transfer */
	}

	if(len > bytesRemaining)
		len = bytesRemaining;
	memcpy(buffer+currentAddress,data,len);
	currentAddress += len;
	bytesRemaining -= len;

	if (bytesRemaining == 0){
		currentbuffer^=1;//toggle
		return 1;               /* end of transfer */
	}
	return 0; /* return 1 if this was the last chunk */
}

/* usbFunctionRead() is called when the host requests a chunk of data from
 * the device. For more information see the documentation in usbdrv/usbdrv.h.
 */
uchar   usbFunctionRead(uchar *data, uchar len)
{
    if(len > bytesRemaining)
        len = bytesRemaining;
    memcpy(data,buffer+currentAddress,len);
    currentAddress += len;
    bytesRemaining -= len;
    return len;
}


usbMsgLen_t usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* HID class request */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* since we have only one report type, we can ignore the report-ID */
            bytesRemaining = 20;
            currentAddress = 0;
            return USB_NO_MSG;  /* use usbFunctionRead() to obtain data */
        }else if(rq->bRequest == USBRQ_HID_SET_REPORT){
            /* since we have only one report type, we can ignore the report-ID */
            bytesRemaining = 20;
            currentAddress = 0;
            return USB_NO_MSG;  /* use usbFunctionWrite() to receive data from host */
        }
    }else{
        /* ignore vendor type requests, we don't use any */
    }
    return 0;
}

int __attribute__((noreturn)) main(void){

	setup();

    DBG1(0x01, 0, 0);       // debug output: main loop starts
    for(;;){    /* main event loop */
    	wdt_reset();
        usbPoll();
    }
}

