/*********************************************************************
 * main.c - Main firmware (ATmega16 version)                         *
 * $Id: $
 * Version 1.98ﾟ                                                     *
 *********************************************************************
 * c64key is Copyright (C) 2006-2007 Mikkel Holm Olsen               *
 * based on HID-Test by Christian Starkjohann, Objective Development *
 *********************************************************************
 * Spaceman Spiff's Commodire 64 USB Keyboard (c64key for short) is  *
 * is free software; you can redistribute it and/or modify it under  *
 * the terms of the OBDEV license, as found in the licence.txt file. *
 *                                                                   *
 * c64key is distributed in the hope that it will be useful,         *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of    *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the     *
 * OBDEV license for further details.                                *
 *********************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>

/* Now included from the makefile */
//#include "keycodes.h"

#include "usbdrv.h"
//#define DEBUG_LEVEL 0
//#include "oddebug.h"
#include "key_dk_us.h"

/* Hardware documentation:
 * ATmega-16 @12.000 MHz
 *
 * XT1..XT2: 12MHz X-tal
 * PB0..PB7: Keyboard matrix Row0..Row7 (pins 12,11,10,5,8,7,6,9 on C64 kbd)
 * PC0..PC7: Keyboard matrix Col0..Col7 (pins 13,19,18,17,16,15,14,20 on C64 kbd)
 * PD0     : D- USB negative (needs appropriate zener-diode and resistors)
 * PD1     : UART TX
 * PD2/INT0: D+ USB positive (needs appropriate zener-diode and resistors)
 * PD7     : Keyboard matrix Row8 (Restore key)
 *
 * USB Connector:
 * -------------
 *  1 (red)    +5V
 *  2 (white)  DATA-
 *  3 (green)  DATA+
 *  4 (black)  GND
 *    
 *
 *
 *                                     VCC
 *                  +--[4k7]--+--[2k2]--+
 *      USB        GND        |                     ATmega-16
 *                            |
 *      (D-)-------+----------+--------[82r]------- PD0
 *                 |
 *      (D+)-------|-----+-------------[82r]------- PD2/INT0
 *                 |     |
 *                 _     _
 *                 ^     ^  2 x 3.6V 
 *                 |     |  zener to GND
 *                 |     |
 *                GND   GND
 */

/* The LED states */
#define LED_NUM     0x01
#define LED_CAPS    0x02
#define LED_SCROLL  0x04
#define LED_COMPOSE 0x08
#define LED_KANA    0x10


/* Originally used as a mask for the modifier bits, but now also
   used for other x -> 2^x conversions (lookup table). */
const char modmask[8] PROGMEM = {
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80
  };


/* USB report descriptor (length is defined in usbconfig.h)
   This has been changed to conform to the USB keyboard boot
   protocol */
const char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] 
  PROGMEM = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x03,                    //   REPORT_SIZE (3)
    0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs)
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0xff,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0xff,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION  
};

/* This buffer holds the last values of the scanned keyboard matrix */
static uchar bitbuf[NUMROWS]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};

/* The ReportBuffer contains the USB report sent to the PC */
static uchar reportBuffer[8];    /* buffer for HID reports */
static uchar idleRate;           /* in 4 ms units */
static uchar protocolVer=1;      /* 0 is the boot protocol, 1 is report protocol */

static void hardwareInit(void) {
  PORTB = 0xFF;   /* Port B are row drivers - enable pull-up */
  DDRB  = 0x00;   /* Port B is input */

  PORTA = 0xFF;   /* activate all pull-ups */
  DDRA  = 0x00;   /* all pins input */
  
  PORTC = 0xFF;   /* activate all pull-ups */
  DDRC  = 0x00;   /* all pins input */
  
  PORTD= 0b11110011;   /* 1111 0011 bin: activate pull-ups except on USB lines */
  DDRD=  0b00001100;   /* 0000 1100 bin: all pins input except USB (-> USB reset) */

  /* USB Reset by device only required on Watchdog Reset */
  _delay_us(11);   /* delay >10ms for USB reset */ 

  DDRD = 0x02;    /* 0000 0010 bin: remove USB reset condition */ 
  /* configure timer 0 for a rate of 12M/(1024 * 256) = 45.78 Hz (~22ms) */
  TCCR0A = 5;      /* timer 0 prescaler: 1024 */
}

/* This function scans the entire keyboard, debounces the keys, and
   if a key change has been found, a new report is generated, and the
   function returns true to signal the transfer of the report. */
static uchar scankeys(void) {   
  uchar reportIndex=1; /* First available report entry is 2 */
  uchar retval=0;
  uchar row,data,key, modkeys;
  volatile uchar col, mask;
  static uchar debounce=5;

	/* reset the PC7 beforehand */
	DDRC&=~(0xF8);
	PORTC|=0xF8;
   
  /* PDD4-5,PA3-7,PC0-2 these 10 pins are Row pins used for selecting rows to be checked. */
  for (row=0;row<NUMROWS;++row) { /* Scan all rows */
     
		if(row<=6){    /* row0-6,PB1-PB7 */
	      DDRB&=~(0xFE); /* AND 0b0000 0001 */
	      DDRB|=(1<<(1+row));
	      PORTB|=0xFE;   /* AND 0b1111 1110 */
	      PORTB&=~(1<<(1+row));
		  
		  if(row==1){ /* PD5 Output Low */
			  DDRD|=~(0xDF); /*0b0010 0000 */
			  PORTD&=0xDF;   /*0b1101 1111 */
		  }
		  if (row==2){ /* Reset PD5*/
			  DDRD&=0xDF;
			  PORTD|=~(0xDF);
		  }

		  
		}else if(row==7){ /* row7 PA0 */
			if(row==7){ /* Reset PortB first*/
				DDRB&=~(0xFE);  
				PORTB|=0xFE;  
			}
			
		  DDRA&=~(0x01);
		  DDRA|=(1<<(row-7));
		  PORTA|=0x01;
		  PORTA&=~(1<<(row-7));

			
		}else if(row>=8 && row<=9){ /*row8-9 PD0-1 */
			if(row==8){ /* Reset PortA Rows */
				DDRA&=~(0x01);  
				PORTA|=0x01;  
			}

			DDRD&=~(0x03);
			DDRD|=(1<<(row-8));
			PORTD|=0x03; /* 0b0000 0011*/
			PORTD&=~(1<<(row-8));


		}else if(row==10){ /*row10 PD4 */
			if(row==10){ /* Reset PortA Rows */
				DDRD&=~(0x03);  
				PORTD|=0x03;  
			}

			DDRD&=~(0x10);
			DDRD|=(1<<(row-10+4));
			PORTD|=0x10; /* 0b0001 0000*/
			PORTD&=~(1<<(row-10+4));
			
		}else if(row==11){ /*row11 PD6 */
			if(row==11){ /* Reset PortA Rows */
				DDRD&=~(0x10);  
				PORTD|=0x10;  
			}

			DDRD&=~(0x40);
			DDRD|=(1<<(row-11+6));
			PORTD|=0x40; /* 0b0100 0000*/
			PORTD&=~(1<<(row-11+6));
			
		}else if(row>=12 && row<=16){ /*row12-16 PC3-7 */
			if(row==12){ /* Reset PortA Rows */
				DDRD&=~(0x40);  
				PORTD|=0x040;  
			}

			DDRC&=~(0xF8);
			DDRC|=(1<<(row-12+3));
			PORTC|=0xF8; /* 0b1111 1000*/
			PORTC&=~(1<<(row-12+3));
			
		}
    
    _delay_us(30); /* Used to be small loop, but the compiler optimized it away ;-) default 30*/
    
    /*PA3-PA7 and PC0-PC2 are Column pins which are used for read*/

    /*PA0,PB1-PB7 are data1 	Bit Order is 'PA7,PA6,PA5,PA4,PA3,PC2,PC1,PC0' */
    data = PINC & 0x07;
    data|= PINA & 0xF8;
	
	/* for debug */
	//if(row>=20 || row<=14){
	//	data =0xFF;
	//}
	/* End debug */

    if (data^bitbuf[row]) { 
      debounce=10; /* If a change was detected, activate debounce counter */
    }
    bitbuf[row]=data; /* Store the data into [x][0]  */
  }

  if (debounce==1) { /* Debounce counter expired */
    modkeys=0;
    memset(reportBuffer,0,sizeof(reportBuffer)); /* Clear report buffer */
    for (row=0;row<NUMROWS;++row) { /* Process all rows for key-codes */
         data=bitbuf[row]; /* Restore buffer */
         if (data!=0xFF) { /* Anything on this row? - optimization */
           for (col=0,mask=1;col<8;++col,mask<<=1) { /* yes - check individual bits */
             if (!(data&mask)) { /* Key detected */
               key=pgm_read_byte(&keymap[row][col]); /* Read keyboard map */
               if (key>KEY_Special) { /* Special handling of shifted keys */
				   
				   if(key==KEY_Romaji){
					   key=0x90; //Romaji key:HIDcode 0x90   
				   }else if(key==KEY_Muhenkan){
					   key=0x8B; //Muhenkan key:HIDcode 0x8B
				   }else if(key==KEY_Henkan){
					   key=0x8A; //Henkan key:HID code 0x8A   
				   }else if(key==KEY_Yen){
					   key=0x89; //YEN Mark key:HID code 0x89
				   }else if(key==KEY_INT1){
					   key=0x87; //(\ _) Mark key:HID code 0x87   
				   }else if(key==KEY_Win){ //WindowsKey equals to CTRL+ESC
					   reportBuffer[0]|=0x10; //CTRL key
					   key=0x29;
				   }else if(key==KEY_APP){ //WindowsKey equals to CTRL+ESC
				  	   reportBuffer[0]|=0x02; //LSHIFT key
				  	   key=KEY_F10;
				   }

               }else if (key>KEY_Modifiers) { /* Is this a modifier key? */
                 reportBuffer[0]|=pgm_read_byte(&modmask[key-(KEY_Modifiers+1)]);
                 key=0;
               }
               if (key) { /* Normal keycode should be added to report */
                 if (++reportIndex>=sizeof(reportBuffer)) { /* Too many keycodes - rollOver */
                   if (!retval&0x02) { /* Only fill buffer once */
                     memset(reportBuffer+2, KEY_errorRollOver, sizeof(reportBuffer)-2);
                     retval|=2; /* continue decoding to get modifiers */
		           //Do nothing while investigating codes
                   }
                 } else {
                   reportBuffer[reportIndex]=key; /* Set next available entry */
                 }
               }
             }
           }
      }
    }
    if (modkeys&0x80) { /* Clear RSHIFT */
      reportBuffer[0]&=~0x20;
    }
    if (modkeys&0x08) { /* Clear LSHIFT */
      reportBuffer[0]&=~0x02;
    }
    reportBuffer[0]|=modkeys&0x77; /* Set other modifiers */

    retval|=1; /* Must have been a change at some point, since debounce is done */
  }
  if (debounce) debounce--; /* Count down, but avoid underflow */
  return retval;
}

uchar expectReport=0;
uchar LEDstate=0;

uchar usbFunctionSetup(uchar data[8]) {
  usbRequest_t *rq = (void *)data;
  usbMsgPtr = reportBuffer;
  if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
    if(rq->bRequest == USBRQ_HID_GET_REPORT){  
      /* wValue: ReportType (highbyte), ReportID (lowbyte) */
      /* we only have one report type, so don't look at wValue */
      return sizeof(reportBuffer);
    }else if(rq->bRequest == USBRQ_HID_SET_REPORT){
      if (rq->wLength.word == 1) { /* We expect one byte reports */
        expectReport=1;
        return 0xFF; /* Call usbFunctionWrite with data */
      }  
    }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
      usbMsgPtr = &idleRate;
      return 1;
    }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
      idleRate = rq->wValue.bytes[1];
    }else if(rq->bRequest == USBRQ_HID_GET_PROTOCOL) {
      if (rq->wValue.bytes[1] < 1) {
        protocolVer = rq->wValue.bytes[1];
      }
    }else if(rq->bRequest == USBRQ_HID_SET_PROTOCOL) {
      usbMsgPtr = &protocolVer;
      return 1;
    }
  }
  return 0;
}

uchar usbFunctionWrite(uchar *data, uchar len) {
  if ((expectReport)&&(len==1)) {
    LEDstate=data[0]; /* Get the state of all 5 LEDs */
    if (LEDstate&LED_CAPS) { /* Check state of CAPS lock LED */
       DDRB|=0x01;
	   PORTB&=~(0x01);   /* PB0 IS LED I/O PORT and SET it OUTPUT-LOW*/
    } else {
       DDRB&=~(0x01);
       PORTB|=0x01;   /* PB0 IS LED I/O PORT and SET it INPUT-PULLUP ENABLE*/  
    }
	
    if (LEDstate&LED_NUM) { /* Check state of NUM lock LED */
	    DDRA|=0x02;
	    PORTA&=~(0x02);   /* PA1 IS LED I/O PORT and SET it OUTPUT-LOW*/
	    } else {
	    DDRA&=~(0x02);
	    PORTA|=0x02;   /* PA1 IS LED I/O PORT and SET it INPUT-PULLUP ENABLE*/
    }
	
    if (LEDstate&LED_SCROLL) { /* Check state of SCROLL lock LED */
	    DDRA|=0x04;
	    PORTA&=~(0x04);   /* PA2 IS LED I/O PORT and SET it OUTPUT-LOW*/
	    } else {
	    DDRA&=~(0x04);
	    PORTA|=0x04;   /* PA2 IS LED I/O PORT and SET it INPUT-PULLUP ENABLE*/
    }		
	
	expectReport=0;
    return 1;
  }
  expectReport=0;
  return 0x01;
}

int main(void) {
  uchar   updateNeeded = 0;
  uchar   idleCounter = 0;

  wdt_enable(WDTO_2S); /* Enable watchdog timer 2s */
  hardwareInit(); /* Initialize hardware (I/O) */
  
  //odDebugInit();

  usbInit(); /* Initialize USB stack processing */
  sei(); /* Enable global interrupts */
  
  for(;;){  /* Main loop */
    wdt_reset(); /* Reset the watchdog */
    usbPoll(); /* Poll the USB stack */

    updateNeeded|=scankeys(); /* Scan the keyboard for changes */
    
    /* Check timer if we need periodic reports */
    if(TIFR0 & (1<<TOV0)){
      TIFR0 = 1<<TOV0; /* Reset flag */
      if(idleRate != 0){ /* Do we need periodic reports? */
        if(idleCounter > 4){ /* Yes, but not yet */
          idleCounter -= 5;   /* 22 ms in units of 4 ms */
        }else{ /* Yes, it is time now */
          updateNeeded = 1;
          idleCounter = idleRate;
        }
      }
    }
    
    /* If an update is needed, send the report */
    if(updateNeeded && usbInterruptIsReady()){
      updateNeeded = 0;
      usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
    }
  }
  return 0;
}
