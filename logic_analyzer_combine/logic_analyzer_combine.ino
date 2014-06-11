/*
 *
 * SUMP Protocol Implementation for Arduino boards.
 *
 * Copyright (c) 2011,2012,2013 Andrew Gillham
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY ANDREW GILLHAM ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIESf
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANDREW GILLHAM BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * NOTE: v0.09 switched the channels BACK to pins 8-13 for trigger reliability.
 *       Please report any issues.  Uncomment USE_PORTD for pins 2-7.
 *
 * This Arduino sketch implements a SUMP protocol compatible with the standard
 * SUMP client as well as the alternative client from here:
 *	http://www.lxtreme.nl/ols/
 *
 * This SUMP protocol compatible logic analyzer for the Arduino board supports
 * 6 channels consisting of digital pins 2-7, which are the last 6 bits (2-7)
 * of PORTD.  Bits 0 & 1 are the UART RX/TX pins.
 *
 * On the Arduino Mega board 8 channels are supported and 7k of samples.
 * Pins 22-29 (Port A) are used by default, you can change the 'CHANPIN' below
 * if something else works better for you.
 *
 * NOTE:
 * If you are using the original SUMP client, or using the alternative client
 * without the device profiles, then you will get a "device not found" error.
 * You must DISABLE the Arduino auto reset feature to use this logic analyzer
 * code. There are various methods to do this, some boards have a jumper,
 * others require you to cut a trace.  You may also install a *precisely*
 * 120 Ohm resistor between the reset & 5V piins.  Make sure it is really
 * 120 Ohm or you may damage your board.
 * It is much easier to use the alternative SUMP client from here:
 *      http://www.lxtreme.nl/ols/
 *
 * The device profiles should be included with this code.  Copy them to the
 * 'plugins' directory of the client.  The location varies depending on the
 * platform, but on the mac it is here by default:
 * /Applications/LogicSniffer.app/Contents/Resources/Java/plugins
 *
 * To use this with the original or alternative SUMP clients,
 * use these settings:
 * 
 * Sampling rate: 4MHz (or lower)
 * Channel Groups: 0 (zero) only
 * Recording Size:
 *    ATmega168:  532 (or lower)
 *    ATmega328:  1024 (or lower)
 *    ATmega2560: 7168 (or lower)
 * Noise Filter: doesn't matter
 * RLE: disabled (unchecked)
 *    NOTE: Preliminary RLE support for 50Hz or less exists, please test it.
 *
 * Triggering is still a work in progress, but generally works for samples
 * below 1MHz.  1MHz works for a basic busy wait trigger that doesn't store
 * until after the trigger fires.
 * Please try it out and report back.
 *
 * Release: v0.12 September 6, 2013.
 *
 */

/*
 * Function prototypes so this can compile from the cli.
 * You'll need the 'arduino-core' package and to check the paths in the
 * Makefile.
 */

void triggerMicro(void);
void captureMicro(void);
void captureMilli(void);
void getCmd(void);
void setupDelay(void);
void blinkled(void);
void get_metadata(void);
void debugprint(void);
void debugdump(void);


/*
 * Should we use PORTD or PORTB?  (default is PORTB)
 * PORTD support with triggers seems to work but needs more testing.
 */
//#define USE_PORTD 1

/*
 * Arduino device profile:      ols.profile-agla.cfg
 * Arduino Mega device profile: ols.profile-aglam.cfg
 */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define CHANPIN PINA
#define CHAN0 22
#define CHAN1 23
#define CHAN2 24
#define CHAN3 25
#define CHAN4 26
#define CHAN5 27
#define CHAN6 28
#define CHAN7 29
#else
#if defined(USE_PORTD)
#define CHANPIN PIND
#define CHAN0 2
#define CHAN1 3
#define CHAN2 4
#define CHAN3 5
#define CHAN4 6
#define CHAN5 7
#else
#define CHANPIN PINB
#define CHAN0 8
#define CHAN1 9
#define CHAN2 10
#define CHAN3 11
#define CHAN4 12
/* Comment out CHAN5 if you don't want to use the LED pin for an input */
#define CHAN5 13
#endif /* USE_PORTD */
#endif
#define ledPin 13

/* XON/XOFF are not supported. */
#define SUMP_RESET 0x00
#define SUMP_ARM   0x01
#define SUMP_QUERY 0x02
#define SUMP_XON   0x11
#define SUMP_XOFF  0x13

/* mask & values used, config ignored. only stage0 supported */
#define SUMP_TRIGGER_MASK 0xC0
#define SUMP_TRIGGER_VALUES 0xC1
#define SUMP_TRIGGER_CONFIG 0xC2

/* Most flags (except RLE) are ignored. */
#define SUMP_SET_DIVIDER 0x80
#define SUMP_SET_READ_DELAY_COUNT 0x81
#define SUMP_SET_FLAGS 0x82
#define SUMP_SET_RLE 0x0100

/* extended commands -- self-test unsupported, but metadata is returned. */
#define SUMP_SELF_TEST 0x03
#define SUMP_GET_METADATA 0x04

/* ATmega168:  532 (or lower)
 * ATmega328:  1024 (or lower)
 * ATmega2560: 7168 (or lower)
 */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define DEBUG_CAPTURE_SIZE 7168
#define CAPTURE_SIZE 7168
#elif defined(__AVR_ATmega328P__)
#define DEBUG_CAPTURE_SIZE 1024
#define CAPTURE_SIZE 1024
#else
#define DEBUG_CAPTURE_SIZE 532
#define CAPTURE_SIZE 532
#endif

#ifdef USE_PORTD
#define DEBUG_ENABLE DDRB = DDRB | B00000001
#define DEBUG_ON PORTB = B00000001
#define DEBUG_OFF PORTB = B00000000
#else
#define DEBUG_ENABLE DDRD = DDRD | B10000000
#define DEBUG_ON PORTD = B10000000
#define DEBUG_OFF PORTD = B00000000
#endif
#define DEBUG
#ifdef DEBUG
#define MAX_CAPTURE_SIZE DEBUG_CAPTURE_SIZE
#else
#define MAX_CAPTURE_SIZE CAPTURE_SIZE
#endif /* DEBUG */

/*
 * SUMP command from host (via serial)
 * SUMP commands are either 1 byte, or for the extended commands, 5 bytes.
 */
int cmdByte = 0;
byte cmdBytes[5];

#ifdef DEBUG
byte savebytes[128];
int savecount = 0;
#endif /* DEBUG */

byte logicdata[MAX_CAPTURE_SIZE];
unsigned int logicIndex = 0;
unsigned int triggerIndex = 0;
unsigned int readCount = MAX_CAPTURE_SIZE;
unsigned int delayCount = 0;
unsigned int trigger = 0;
unsigned int trigger_values = 0;
unsigned int useMicro = 0;
unsigned int delayTime = 0;
unsigned long divider = 0;
boolean rleEnabled = 0;

void setup()
{
  Serial.begin(115200);

  /*
   * set debug pin (digital pin 8) to output right away so it settles.
   * this gets toggled during sampling as a way to measure
   * the sample time.  this is used during development to
   * properly pad out the sampling routines.
   */
  DEBUG_ENABLE; /* debug measurement pin */

  pinMode(CHAN0, INPUT);
  pinMode(CHAN1, INPUT);
  pinMode(CHAN2, INPUT);
  pinMode(CHAN3, INPUT);
  pinMode(CHAN4, INPUT);
#ifdef CHAN5
  pinMode(CHAN5, INPUT);
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  pinMode(CHAN6, INPUT);
  pinMode(CHAN7, INPUT);
#else
#ifndef CHAN5
  pinMode(ledPin, OUTPUT);
#endif
#endif /* Mega */

#if 0

  /*
   * This sets up timer2 at 100KHz to toggle a pin.  This is useful
   * for debugging as it gives an internally precise signal source.
   * This doesn't work on the Arduino Mega.  Use on the Uno or older.
   * We're using the same clock source for the timer & our sampling.
   */

  /* Set OC2A (digital pin 11) to output so we can toggle it. */
  pinMode(11, OUTPUT);

  /* reset timer to zero */
  TCNT2 = 0;
  TCCR2A = 0;
  TCCR2B = 0;
  OCR2A = 0;

  /* Set CTC mode and toggle on compare. */
  TCCR2A = _BV (COM2A0) | _BV (WGM21);
  /* 79 = 100KHz, 15 = 500KHz, 7 = 1MHz */
  OCR2A = 79;
  TCCR2B = _BV (CS20);
#endif
}

void loop()
{
  int i;

  if (Serial.available() > 0) {
    cmdByte = Serial.read();
    switch(cmdByte) {
    case SUMP_RESET:
      /*
       * We don't do anything here as some unsupported extended commands have
       * zero bytes and are mistaken as resets.  This can trigger false resets
       * so we don't erase the data or do anything for a reset.
       */
      break;
    case SUMP_QUERY:
      /* return the expected bytes. */
      Serial.write('1');
      Serial.write('A');
      Serial.write('L');
      Serial.write('S');
      break;
    case SUMP_ARM:
      /*
       * Zero out any previous samples before arming.
       * Done here instead via reset due to spurious resets.
       */
      for (i = 0 ; i < MAX_CAPTURE_SIZE; i++) {
        logicdata[i] = 0;
      }
      /*
       * depending on the sample rate we need to delay in microseconds
       * or milliseconds.  We can't do the complex trigger at 1MHz
       * so in that case (delayTime == 1 and triggers enabled) use
       * captureMicro() instead of triggerMicro().
       */

      if (divider == 24) {
        /* 4.0MHz */
        captureInline4mhz();
      } 
      else if (divider == 49) {
        /* 2.0MHz */
        captureInline2mhz();
      } 
      else if (useMicro) {
        if (trigger && (delayTime != 1)) {
          triggerMicro();
        } 
        else {
          captureMicro();
        }
      } 
      else {
        captureMilli();
      }
      break;
    case SUMP_TRIGGER_MASK:
      /*
       * the trigger mask byte has a '1' for each enabled trigger so
       * we can just use it directly as our trigger mask.
       */
      getCmd();
#ifdef USE_PORTD
      trigger = cmdBytes[0] << 2;
#else
      trigger = cmdBytes[0];
#endif
      break;
    case SUMP_TRIGGER_VALUES:
      /*
       * trigger_values can be used directly as the value of each bit
       * defines whether we're looking for it to be high or low.
       */
      getCmd();
#ifdef USE_PORTD
      trigger_values = cmdBytes[0] << 2;
#else
      trigger_values = cmdBytes[0];
#endif
      break;
    case SUMP_TRIGGER_CONFIG:
      /* read the rest of the command bytes, but ignore them. */
      getCmd();
      break;
    case SUMP_SET_DIVIDER:
      /*
       * the shifting needs to be done on the 32bit unsigned long variable
       * so that << 16 doesn't end up as zero.
       */
      getCmd();
      divider = cmdBytes[2];
      divider = divider << 8;
      divider += cmdBytes[1];
      divider = divider << 8;
      divider += cmdBytes[0];
      setupDelay();
      break;
    case SUMP_SET_READ_DELAY_COUNT:
      /*
       * this just sets up how many samples there should be before
       * and after the trigger fires.  The readCount is total samples
       * to return and delayCount number of samples after the trigger.
       * this sets the buffer splits like 0/100, 25/75, 50/50
       * for example if readCount == delayCount then we should
       * return all samples starting from the trigger point.
       * if delayCount < readCount we return (readCount - delayCount) of
       * samples from before the trigger fired.
       */
      getCmd();
      readCount = 4 * (((cmdBytes[1] << 8) | cmdBytes[0]) + 1);
      if (readCount > MAX_CAPTURE_SIZE)
        readCount = MAX_CAPTURE_SIZE;
      delayCount = 4 * (((cmdBytes[3] << 8) | cmdBytes[2]) + 1);
      if (delayCount > MAX_CAPTURE_SIZE)
        delayCount = MAX_CAPTURE_SIZE;
      break;
    case SUMP_SET_FLAGS:
      /* read the rest of the command bytes and check if RLE is enabled. */
      getCmd();
      rleEnabled = ((cmdBytes[1] & B1000000) != 0);
      break;
    case SUMP_GET_METADATA:
      /*
       * We return a description of our capabilities.
       * Check the function's comments below.
       */
      get_metadata();
      break;
    case SUMP_SELF_TEST:
      /* ignored. */
      break;
#ifdef DEBUG
      /*
       * a couple of debug commands used during development.
       */
    case '0':
      /*
       * This resets the debug buffer pointer, effectively clearing the
       * previous commands out of the buffer. Clear the sample data as well.
       * Just send a '0' from the Arduino IDE's Serial Monitor.
       */
      savecount=0;
      for (i = 0 ; i < MAX_CAPTURE_SIZE; i++) {
        logicdata[i] = 0;
      }
      break;
    case '1':
      /*
       * This is used to see what commands were sent to the device.
       * you can use the Arduino serial monitor and send a '1' and get
       * a debug printout.  useless except for development.
       */
      blinkled();
      debugprint();
      break;
    case '2':
      /*
       * This dumps the sample data to the serial port.  Used for debugging.
       */
      debugdump();
      break;
#endif /* DEBUG */
    default:
      /* ignore any unrecognized bytes. */
      break;
    }
  }
}

void blinkled() {
  digitalWrite(ledPin, HIGH);
  delay(200);
  digitalWrite(ledPin, LOW);
  delay(200);
}

/*
 * Extended SUMP commands are 5 bytes.  A command byte followed by 4 bytes
 * of options. We already read the command byte, this gets the remaining
 * 4 bytes of the command.
 * If we're debugging we save the received commands in a debug buffer.
 * We need to make sure we don't overrun the debug buffer.
 */
void getCmd() {
  delay(10);
  cmdBytes[0] = Serial.read();
  cmdBytes[1] = Serial.read();
  cmdBytes[2] = Serial.read();
  cmdBytes[3] = Serial.read();

#ifdef DEBUG
  if (savecount < 120 ) {
    savebytes[savecount++] = ' ';
    savebytes[savecount++] = cmdByte;
    savebytes[savecount++] = cmdBytes[0];
    savebytes[savecount++] = cmdBytes[1];
    savebytes[savecount++] = cmdBytes[2];
    savebytes[savecount++] = cmdBytes[3];
  }
#endif /* DEBUG */
}

/*
 * This function samples data using a microsecond delay function.
 * It also has rudimentary trigger support where it will just sit in
 * a busy loop waiting for the trigger conditions to occur.
 *
 * This loop is not clocked to the sample rate in any way, it just
 * reads the port as fast as possible waiting for a trigger match.
 * Multiple channels can have triggers enabled and can have different
 * trigger values.  All conditions must match to trigger.
 *
 * After the trigger fires (if it is enabled) the pins are sampled
 * at the appropriate rate.
 *
 */

void captureMicro() {
  unsigned int i;

  /*
   * basic trigger, wait until all trigger conditions are met on port.
   * this needs further testing, but basic tests work as expected.
   */
  if (trigger) {
    while ((trigger_values ^ CHANPIN) & trigger);
  }

  /*
   * disable interrupts during capture to maintain precision.
   * we're hand padding loops with NOP instructions so we absolutely
   * cannot have any interrupts firing.
   */
  cli();

  /*
   * toggle pin a few times to activate trigger for debugging.
   * this is used during development to measure the sample intervals.
   * it is best to just leave the toggling in place so we don't alter
   * any timing unexpectedly.
   * Arduino digital pin 8 is being used here.
   */
  DEBUG_ENABLE;
#ifdef DEBUG
  DEBUG_ON;
  delayMicroseconds(20);
  DEBUG_OFF;
  delayMicroseconds(20);
  DEBUG_ON;
  delayMicroseconds(20);
  DEBUG_OFF;
  delayMicroseconds(20);
#endif

  if (delayTime == 1) {
    /*
     * 1MHz sample rate = 1 uS delay so we can't use delayMicroseconds
     * since our loop takes some time.  The delay is padded out by hand.
     */
    DEBUG_ON; /* debug timing measurement */
    for (i = 0 ; i < readCount; i++) {
      logicdata[i] = CHANPIN;
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    }
    DEBUG_OFF; /* debug timing measurement */
  } 
  else if (delayTime == 2) {
    /*
     * 500KHz sample rate = 2 uS delay, still pretty fast so we pad this
     * one by hand too.
     */
    DEBUG_ON; /* debug timing measurement */
    for (i = 0 ; i < readCount; i++) {
      logicdata[i] = CHANPIN;
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    }
    DEBUG_OFF; /* debug timing measurement */
  } 
  else {
    /*
     * not 1MHz or 500KHz; delayMicroseconds(delay - 1) works fine here
     * with two NOPs of padding. (based on measured debug pin toggles with
     * a better logic analyzer)
     * start of real measurement
     */
    DEBUG_ON; /* debug timing measurement */
    for (i = 0 ; i < readCount; i++) {
      logicdata[i] = CHANPIN;
      delayMicroseconds(delayTime - 1);
      __asm__("nop\n\t""nop\n\t");
    }
    DEBUG_OFF; /* debug timing measurement */
  }

  /* re-enable interrupts now that we're done sampling. */
  sei();

  /*
   * dump the samples back to the SUMP client.  nothing special
   * is done for any triggers, this is effectively the 0/100 buffer split.
   */
  for (i = 0 ; i < readCount; i++) {
#ifdef USE_PORTD
    Serial.write(logicdata[i] >> 2);
#else
    Serial.write(logicdata[i]);
#endif
  }
}

/*
 * This function does straight sampling with basic triggering.  It is
 * for those sample rates that can't be done via the 'delayMicrosecond()' call
 * which is limited to 16383 microseconds max delay.  That is about 62Hz max.
 * This is only used for sample rates < 100Hz.
 *
 * The basic triggering in this function will be replaced by a 'triggerMillis'
 * function eventually that uses the circular trigger buffer.
 *
 * Since we're using delay() and 20ms/50ms/100ms sample rates we're not
 * worried that the sample loops take a few microseconds more than we're
 * supposed to.
 * We could measure the sample loop and use delay(delayTime - 1), then
 * delayMicroseconds() and possibly a bit of NOP padding to ensure our
 * samples our a precise multiple of milliseconds, but for now we'll use
 * this basic functionality.
 */

 
void captureInline2mhz() {
  unsigned int i;

  /*
   * basic trigger, wait until all trigger conditions are met on port.
   * this needs further testing, but basic tests work as expected.
   */
  if (trigger) {
    while ((trigger_values ^ CHANPIN) & trigger);
  }

  /*
   * disable interrupts during capture to maintain precision.
   * we cannot afford any timing interference so we absolutely
   * cannot have any interrupts firing.
   */
  cli();

  /*
   * toggle pin a few times to activate trigger for debugging.
   * this is used during development to measure the sample intervals.
   * it is best to just leave the toggling in place so we don't alter
   * any timing unexpectedly.
   */
  DEBUG_ENABLE;
  DEBUG_ON;
  delayMicroseconds(1);
  DEBUG_OFF;
  delayMicroseconds(1);
  DEBUG_ON;
  delayMicroseconds(1);
  DEBUG_OFF;
  delayMicroseconds(1);

  DEBUG_ON; /* debug timing measurement */

  /*
   * Unroll loop to maximize capture speed.
   * Pad with 5 cycles to make this run at 2MHz.
   * Using the RJMP instructions (instead of NOP) reduces flash usage by 4096 bytes.
   *
   */

#undef INLINE_NOP
#define INLINE_NOP		__asm__("nop\n\t""rjmp 1f\n\t""1:\n\t""rjmp 2f\n\t""2:\n\t");

  logicdata[0] = CHANPIN;
  INLINE_NOP;
  logicdata[1] = CHANPIN;
  INLINE_NOP;
  logicdata[2] = CHANPIN;
  INLINE_NOP;
  logicdata[3] = CHANPIN;
  INLINE_NOP;
  logicdata[4] = CHANPIN;
  INLINE_NOP;
  logicdata[5] = CHANPIN;
  INLINE_NOP;
  logicdata[6] = CHANPIN;
  INLINE_NOP;
  logicdata[7] = CHANPIN;
  INLINE_NOP;
  logicdata[8] = CHANPIN;
  INLINE_NOP;
  logicdata[9] = CHANPIN;
  INLINE_NOP;
  logicdata[10] = CHANPIN;
  INLINE_NOP;
  logicdata[11] = CHANPIN;
  INLINE_NOP;
  logicdata[12] = CHANPIN;
  INLINE_NOP;
  logicdata[13] = CHANPIN;
  INLINE_NOP;
  logicdata[14] = CHANPIN;
  INLINE_NOP;
  logicdata[15] = CHANPIN;
  INLINE_NOP;
  logicdata[16] = CHANPIN;
  INLINE_NOP;
  logicdata[17] = CHANPIN;
  INLINE_NOP;
  logicdata[18] = CHANPIN;
  INLINE_NOP;
  logicdata[19] = CHANPIN;
  INLINE_NOP;
  logicdata[20] = CHANPIN;
  INLINE_NOP;
  logicdata[21] = CHANPIN;
  INLINE_NOP;
  logicdata[22] = CHANPIN;
  INLINE_NOP;
  logicdata[23] = CHANPIN;
  INLINE_NOP;
  logicdata[24] = CHANPIN;
  INLINE_NOP;
  logicdata[25] = CHANPIN;
  INLINE_NOP;
  logicdata[26] = CHANPIN;
  INLINE_NOP;
  logicdata[27] = CHANPIN;
  INLINE_NOP;
  logicdata[28] = CHANPIN;
  INLINE_NOP;
  logicdata[29] = CHANPIN;
  INLINE_NOP;
  logicdata[30] = CHANPIN;
  INLINE_NOP;
  logicdata[31] = CHANPIN;
  INLINE_NOP;
  logicdata[32] = CHANPIN;
  INLINE_NOP;
  logicdata[33] = CHANPIN;
  INLINE_NOP;
  logicdata[34] = CHANPIN;
  INLINE_NOP;
  logicdata[35] = CHANPIN;
  INLINE_NOP;
  logicdata[36] = CHANPIN;
  INLINE_NOP;
  logicdata[37] = CHANPIN;
  INLINE_NOP;
  logicdata[38] = CHANPIN;
  INLINE_NOP;
  logicdata[39] = CHANPIN;
  INLINE_NOP;
  logicdata[40] = CHANPIN;
  INLINE_NOP;
  logicdata[41] = CHANPIN;
  INLINE_NOP;
  logicdata[42] = CHANPIN;
  INLINE_NOP;
  logicdata[43] = CHANPIN;
  INLINE_NOP;
  logicdata[44] = CHANPIN;
  INLINE_NOP;
  logicdata[45] = CHANPIN;
  INLINE_NOP;
  logicdata[46] = CHANPIN;
  INLINE_NOP;
  logicdata[47] = CHANPIN;
  INLINE_NOP;
  logicdata[48] = CHANPIN;
  INLINE_NOP;
  logicdata[49] = CHANPIN;
  INLINE_NOP;
  logicdata[50] = CHANPIN;
  INLINE_NOP;
  logicdata[51] = CHANPIN;
  INLINE_NOP;
  logicdata[52] = CHANPIN;
  INLINE_NOP;
  logicdata[53] = CHANPIN;
  INLINE_NOP;
  logicdata[54] = CHANPIN;
  INLINE_NOP;
  logicdata[55] = CHANPIN;
  INLINE_NOP;
  logicdata[56] = CHANPIN;
  INLINE_NOP;
  logicdata[57] = CHANPIN;
  INLINE_NOP;
  logicdata[58] = CHANPIN;
  INLINE_NOP;
  logicdata[59] = CHANPIN;
  INLINE_NOP;
  logicdata[60] = CHANPIN;
  INLINE_NOP;
  logicdata[61] = CHANPIN;
  INLINE_NOP;
  logicdata[62] = CHANPIN;
  INLINE_NOP;
  logicdata[63] = CHANPIN;
  INLINE_NOP;
  logicdata[64] = CHANPIN;
  INLINE_NOP;
  logicdata[65] = CHANPIN;
  INLINE_NOP;
  logicdata[66] = CHANPIN;
  INLINE_NOP;
  logicdata[67] = CHANPIN;
  INLINE_NOP;
  logicdata[68] = CHANPIN;
  INLINE_NOP;
  logicdata[69] = CHANPIN;
  INLINE_NOP;
  logicdata[70] = CHANPIN;
  INLINE_NOP;
  logicdata[71] = CHANPIN;
  INLINE_NOP;
  logicdata[72] = CHANPIN;
  INLINE_NOP;
  logicdata[73] = CHANPIN;
  INLINE_NOP;
  logicdata[74] = CHANPIN;
  INLINE_NOP;
  logicdata[75] = CHANPIN;
  INLINE_NOP;
  logicdata[76] = CHANPIN;
  INLINE_NOP;
  logicdata[77] = CHANPIN;
  INLINE_NOP;
  logicdata[78] = CHANPIN;
  INLINE_NOP;
  logicdata[79] = CHANPIN;
  INLINE_NOP;
  logicdata[80] = CHANPIN;
  INLINE_NOP;
  logicdata[81] = CHANPIN;
  INLINE_NOP;
  logicdata[82] = CHANPIN;
  INLINE_NOP;
  logicdata[83] = CHANPIN;
  INLINE_NOP;
  logicdata[84] = CHANPIN;
  INLINE_NOP;
  logicdata[85] = CHANPIN;
  INLINE_NOP;
  logicdata[86] = CHANPIN;
  INLINE_NOP;
  logicdata[87] = CHANPIN;
  INLINE_NOP;
  logicdata[88] = CHANPIN;
  INLINE_NOP;
  logicdata[89] = CHANPIN;
  INLINE_NOP;
  logicdata[90] = CHANPIN;
  INLINE_NOP;
  logicdata[91] = CHANPIN;
  INLINE_NOP;
  logicdata[92] = CHANPIN;
  INLINE_NOP;
  logicdata[93] = CHANPIN;
  INLINE_NOP;
  logicdata[94] = CHANPIN;
  INLINE_NOP;
  logicdata[95] = CHANPIN;
  INLINE_NOP;
  logicdata[96] = CHANPIN;
  INLINE_NOP;
  logicdata[97] = CHANPIN;
  INLINE_NOP;
  logicdata[98] = CHANPIN;
  INLINE_NOP;
  logicdata[99] = CHANPIN;
  INLINE_NOP;
  logicdata[100] = CHANPIN;
  INLINE_NOP;
  logicdata[101] = CHANPIN;
  INLINE_NOP;
  logicdata[102] = CHANPIN;
  INLINE_NOP;
  logicdata[103] = CHANPIN;
  INLINE_NOP;
  logicdata[104] = CHANPIN;
  INLINE_NOP;
  logicdata[105] = CHANPIN;
  INLINE_NOP;
  logicdata[106] = CHANPIN;
  INLINE_NOP;
  logicdata[107] = CHANPIN;
  INLINE_NOP;
  logicdata[108] = CHANPIN;
  INLINE_NOP;
  logicdata[109] = CHANPIN;
  INLINE_NOP;
  logicdata[110] = CHANPIN;
  INLINE_NOP;
  logicdata[111] = CHANPIN;
  INLINE_NOP;
  logicdata[112] = CHANPIN;
  INLINE_NOP;
  logicdata[113] = CHANPIN;
  INLINE_NOP;
  logicdata[114] = CHANPIN;
  INLINE_NOP;
  logicdata[115] = CHANPIN;
  INLINE_NOP;
  logicdata[116] = CHANPIN;
  INLINE_NOP;
  logicdata[117] = CHANPIN;
  INLINE_NOP;
  logicdata[118] = CHANPIN;
  INLINE_NOP;
  logicdata[119] = CHANPIN;
  INLINE_NOP;
  logicdata[120] = CHANPIN;
  INLINE_NOP;
  logicdata[121] = CHANPIN;
  INLINE_NOP;
  logicdata[122] = CHANPIN;
  INLINE_NOP;
  logicdata[123] = CHANPIN;
  INLINE_NOP;
  logicdata[124] = CHANPIN;
  INLINE_NOP;
  logicdata[125] = CHANPIN;
  INLINE_NOP;
  logicdata[126] = CHANPIN;
  INLINE_NOP;
  logicdata[127] = CHANPIN;
  INLINE_NOP;
  logicdata[128] = CHANPIN;
  INLINE_NOP;
  logicdata[129] = CHANPIN;
  INLINE_NOP;
  logicdata[130] = CHANPIN;
  INLINE_NOP;
  logicdata[131] = CHANPIN;
  INLINE_NOP;
  logicdata[132] = CHANPIN;
  INLINE_NOP;
  logicdata[133] = CHANPIN;
  INLINE_NOP;
  logicdata[134] = CHANPIN;
  INLINE_NOP;
  logicdata[135] = CHANPIN;
  INLINE_NOP;
  logicdata[136] = CHANPIN;
  INLINE_NOP;
  logicdata[137] = CHANPIN;
  INLINE_NOP;
  logicdata[138] = CHANPIN;
  INLINE_NOP;
  logicdata[139] = CHANPIN;
  INLINE_NOP;
  logicdata[140] = CHANPIN;
  INLINE_NOP;
  logicdata[141] = CHANPIN;
  INLINE_NOP;
  logicdata[142] = CHANPIN;
  INLINE_NOP;
  logicdata[143] = CHANPIN;
  INLINE_NOP;
  logicdata[144] = CHANPIN;
  INLINE_NOP;
  logicdata[145] = CHANPIN;
  INLINE_NOP;
  logicdata[146] = CHANPIN;
  INLINE_NOP;
  logicdata[147] = CHANPIN;
  INLINE_NOP;
  logicdata[148] = CHANPIN;
  INLINE_NOP;
  logicdata[149] = CHANPIN;
  INLINE_NOP;
  logicdata[150] = CHANPIN;
  INLINE_NOP;
  logicdata[151] = CHANPIN;
  INLINE_NOP;
  logicdata[152] = CHANPIN;
  INLINE_NOP;
  logicdata[153] = CHANPIN;
  INLINE_NOP;
  logicdata[154] = CHANPIN;
  INLINE_NOP;
  logicdata[155] = CHANPIN;
  INLINE_NOP;
  logicdata[156] = CHANPIN;
  INLINE_NOP;
  logicdata[157] = CHANPIN;
  INLINE_NOP;
  logicdata[158] = CHANPIN;
  INLINE_NOP;
  logicdata[159] = CHANPIN;
  INLINE_NOP;
  logicdata[160] = CHANPIN;
  INLINE_NOP;
  logicdata[161] = CHANPIN;
  INLINE_NOP;
  logicdata[162] = CHANPIN;
  INLINE_NOP;
  logicdata[163] = CHANPIN;
  INLINE_NOP;
  logicdata[164] = CHANPIN;
  INLINE_NOP;
  logicdata[165] = CHANPIN;
  INLINE_NOP;
  logicdata[166] = CHANPIN;
  INLINE_NOP;
  logicdata[167] = CHANPIN;
  INLINE_NOP;
  logicdata[168] = CHANPIN;
  INLINE_NOP;
  logicdata[169] = CHANPIN;
  INLINE_NOP;
  logicdata[170] = CHANPIN;
  INLINE_NOP;
  logicdata[171] = CHANPIN;
  INLINE_NOP;
  logicdata[172] = CHANPIN;
  INLINE_NOP;
  logicdata[173] = CHANPIN;
  INLINE_NOP;
  logicdata[174] = CHANPIN;
  INLINE_NOP;
  logicdata[175] = CHANPIN;
  INLINE_NOP;
  logicdata[176] = CHANPIN;
  INLINE_NOP;
  logicdata[177] = CHANPIN;
  INLINE_NOP;
  logicdata[178] = CHANPIN;
  INLINE_NOP;
  logicdata[179] = CHANPIN;
  INLINE_NOP;
  logicdata[180] = CHANPIN;
  INLINE_NOP;
  logicdata[181] = CHANPIN;
  INLINE_NOP;
  logicdata[182] = CHANPIN;
  INLINE_NOP;
  logicdata[183] = CHANPIN;
  INLINE_NOP;
  logicdata[184] = CHANPIN;
  INLINE_NOP;
  logicdata[185] = CHANPIN;
  INLINE_NOP;
  logicdata[186] = CHANPIN;
  INLINE_NOP;
  logicdata[187] = CHANPIN;
  INLINE_NOP;
  logicdata[188] = CHANPIN;
  INLINE_NOP;
  logicdata[189] = CHANPIN;
  INLINE_NOP;
  logicdata[190] = CHANPIN;
  INLINE_NOP;
  logicdata[191] = CHANPIN;
  INLINE_NOP;
  logicdata[192] = CHANPIN;
  INLINE_NOP;
  logicdata[193] = CHANPIN;
  INLINE_NOP;
  logicdata[194] = CHANPIN;
  INLINE_NOP;
  logicdata[195] = CHANPIN;
  INLINE_NOP;
  logicdata[196] = CHANPIN;
  INLINE_NOP;
  logicdata[197] = CHANPIN;
  INLINE_NOP;
  logicdata[198] = CHANPIN;
  INLINE_NOP;
  logicdata[199] = CHANPIN;
  INLINE_NOP;
  logicdata[200] = CHANPIN;
  INLINE_NOP;
  logicdata[201] = CHANPIN;
  INLINE_NOP;
  logicdata[202] = CHANPIN;
  INLINE_NOP;
  logicdata[203] = CHANPIN;
  INLINE_NOP;
  logicdata[204] = CHANPIN;
  INLINE_NOP;
  logicdata[205] = CHANPIN;
  INLINE_NOP;
  logicdata[206] = CHANPIN;
  INLINE_NOP;
  logicdata[207] = CHANPIN;
  INLINE_NOP;
  logicdata[208] = CHANPIN;
  INLINE_NOP;
  logicdata[209] = CHANPIN;
  INLINE_NOP;
  logicdata[210] = CHANPIN;
  INLINE_NOP;
  logicdata[211] = CHANPIN;
  INLINE_NOP;
  logicdata[212] = CHANPIN;
  INLINE_NOP;
  logicdata[213] = CHANPIN;
  INLINE_NOP;
  logicdata[214] = CHANPIN;
  INLINE_NOP;
  logicdata[215] = CHANPIN;
  INLINE_NOP;
  logicdata[216] = CHANPIN;
  INLINE_NOP;
  logicdata[217] = CHANPIN;
  INLINE_NOP;
  logicdata[218] = CHANPIN;
  INLINE_NOP;
  logicdata[219] = CHANPIN;
  INLINE_NOP;
  logicdata[220] = CHANPIN;
  INLINE_NOP;
  logicdata[221] = CHANPIN;
  INLINE_NOP;
  logicdata[222] = CHANPIN;
  INLINE_NOP;
  logicdata[223] = CHANPIN;
  INLINE_NOP;
  logicdata[224] = CHANPIN;
  INLINE_NOP;
  logicdata[225] = CHANPIN;
  INLINE_NOP;
  logicdata[226] = CHANPIN;
  INLINE_NOP;
  logicdata[227] = CHANPIN;
  INLINE_NOP;
  logicdata[228] = CHANPIN;
  INLINE_NOP;
  logicdata[229] = CHANPIN;
  INLINE_NOP;
  logicdata[230] = CHANPIN;
  INLINE_NOP;
  logicdata[231] = CHANPIN;
  INLINE_NOP;
  logicdata[232] = CHANPIN;
  INLINE_NOP;
  logicdata[233] = CHANPIN;
  INLINE_NOP;
  logicdata[234] = CHANPIN;
  INLINE_NOP;
  logicdata[235] = CHANPIN;
  INLINE_NOP;
  logicdata[236] = CHANPIN;
  INLINE_NOP;
  logicdata[237] = CHANPIN;
  INLINE_NOP;
  logicdata[238] = CHANPIN;
  INLINE_NOP;
  logicdata[239] = CHANPIN;
  INLINE_NOP;
  logicdata[240] = CHANPIN;
  INLINE_NOP;
  logicdata[241] = CHANPIN;
  INLINE_NOP;
  logicdata[242] = CHANPIN;
  INLINE_NOP;
  logicdata[243] = CHANPIN;
  INLINE_NOP;
  logicdata[244] = CHANPIN;
  INLINE_NOP;
  logicdata[245] = CHANPIN;
  INLINE_NOP;
  logicdata[246] = CHANPIN;
  INLINE_NOP;
  logicdata[247] = CHANPIN;
  INLINE_NOP;
  logicdata[248] = CHANPIN;
  INLINE_NOP;
  logicdata[249] = CHANPIN;
  INLINE_NOP;
  logicdata[250] = CHANPIN;
  INLINE_NOP;
  logicdata[251] = CHANPIN;
  INLINE_NOP;
  logicdata[252] = CHANPIN;
  INLINE_NOP;
  logicdata[253] = CHANPIN;
  INLINE_NOP;
  logicdata[254] = CHANPIN;
  INLINE_NOP;
  logicdata[255] = CHANPIN;
  INLINE_NOP;
  logicdata[256] = CHANPIN;
  INLINE_NOP;
  logicdata[257] = CHANPIN;
  INLINE_NOP;
  logicdata[258] = CHANPIN;
  INLINE_NOP;
  logicdata[259] = CHANPIN;
  INLINE_NOP;
  logicdata[260] = CHANPIN;
  INLINE_NOP;
  logicdata[261] = CHANPIN;
  INLINE_NOP;
  logicdata[262] = CHANPIN;
  INLINE_NOP;
  logicdata[263] = CHANPIN;
  INLINE_NOP;
  logicdata[264] = CHANPIN;
  INLINE_NOP;
  logicdata[265] = CHANPIN;
  INLINE_NOP;
  logicdata[266] = CHANPIN;
  INLINE_NOP;
  logicdata[267] = CHANPIN;
  INLINE_NOP;
  logicdata[268] = CHANPIN;
  INLINE_NOP;
  logicdata[269] = CHANPIN;
  INLINE_NOP;
  logicdata[270] = CHANPIN;
  INLINE_NOP;
  logicdata[271] = CHANPIN;
  INLINE_NOP;
  logicdata[272] = CHANPIN;
  INLINE_NOP;
  logicdata[273] = CHANPIN;
  INLINE_NOP;
  logicdata[274] = CHANPIN;
  INLINE_NOP;
  logicdata[275] = CHANPIN;
  INLINE_NOP;
  logicdata[276] = CHANPIN;
  INLINE_NOP;
  logicdata[277] = CHANPIN;
  INLINE_NOP;
  logicdata[278] = CHANPIN;
  INLINE_NOP;
  logicdata[279] = CHANPIN;
  INLINE_NOP;
  logicdata[280] = CHANPIN;
  INLINE_NOP;
  logicdata[281] = CHANPIN;
  INLINE_NOP;
  logicdata[282] = CHANPIN;
  INLINE_NOP;
  logicdata[283] = CHANPIN;
  INLINE_NOP;
  logicdata[284] = CHANPIN;
  INLINE_NOP;
  logicdata[285] = CHANPIN;
  INLINE_NOP;
  logicdata[286] = CHANPIN;
  INLINE_NOP;
  logicdata[287] = CHANPIN;
  INLINE_NOP;
  logicdata[288] = CHANPIN;
  INLINE_NOP;
  logicdata[289] = CHANPIN;
  INLINE_NOP;
  logicdata[290] = CHANPIN;
  INLINE_NOP;
  logicdata[291] = CHANPIN;
  INLINE_NOP;
  logicdata[292] = CHANPIN;
  INLINE_NOP;
  logicdata[293] = CHANPIN;
  INLINE_NOP;
  logicdata[294] = CHANPIN;
  INLINE_NOP;
  logicdata[295] = CHANPIN;
  INLINE_NOP;
  logicdata[296] = CHANPIN;
  INLINE_NOP;
  logicdata[297] = CHANPIN;
  INLINE_NOP;
  logicdata[298] = CHANPIN;
  INLINE_NOP;
  logicdata[299] = CHANPIN;
  INLINE_NOP;
  logicdata[300] = CHANPIN;
  INLINE_NOP;
  logicdata[301] = CHANPIN;
  INLINE_NOP;
  logicdata[302] = CHANPIN;
  INLINE_NOP;
  logicdata[303] = CHANPIN;
  INLINE_NOP;
  logicdata[304] = CHANPIN;
  INLINE_NOP;
  logicdata[305] = CHANPIN;
  INLINE_NOP;
  logicdata[306] = CHANPIN;
  INLINE_NOP;
  logicdata[307] = CHANPIN;
  INLINE_NOP;
  logicdata[308] = CHANPIN;
  INLINE_NOP;
  logicdata[309] = CHANPIN;
  INLINE_NOP;
  logicdata[310] = CHANPIN;
  INLINE_NOP;
  logicdata[311] = CHANPIN;
  INLINE_NOP;
  logicdata[312] = CHANPIN;
  INLINE_NOP;
  logicdata[313] = CHANPIN;
  INLINE_NOP;
  logicdata[314] = CHANPIN;
  INLINE_NOP;
  logicdata[315] = CHANPIN;
  INLINE_NOP;
  logicdata[316] = CHANPIN;
  INLINE_NOP;
  logicdata[317] = CHANPIN;
  INLINE_NOP;
  logicdata[318] = CHANPIN;
  INLINE_NOP;
  logicdata[319] = CHANPIN;
  INLINE_NOP;
  logicdata[320] = CHANPIN;
  INLINE_NOP;
  logicdata[321] = CHANPIN;
  INLINE_NOP;
  logicdata[322] = CHANPIN;
  INLINE_NOP;
  logicdata[323] = CHANPIN;
  INLINE_NOP;
  logicdata[324] = CHANPIN;
  INLINE_NOP;
  logicdata[325] = CHANPIN;
  INLINE_NOP;
  logicdata[326] = CHANPIN;
  INLINE_NOP;
  logicdata[327] = CHANPIN;
  INLINE_NOP;
  logicdata[328] = CHANPIN;
  INLINE_NOP;
  logicdata[329] = CHANPIN;
  INLINE_NOP;
  logicdata[330] = CHANPIN;
  INLINE_NOP;
  logicdata[331] = CHANPIN;
  INLINE_NOP;
  logicdata[332] = CHANPIN;
  INLINE_NOP;
  logicdata[333] = CHANPIN;
  INLINE_NOP;
  logicdata[334] = CHANPIN;
  INLINE_NOP;
  logicdata[335] = CHANPIN;
  INLINE_NOP;
  logicdata[336] = CHANPIN;
  INLINE_NOP;
  logicdata[337] = CHANPIN;
  INLINE_NOP;
  logicdata[338] = CHANPIN;
  INLINE_NOP;
  logicdata[339] = CHANPIN;
  INLINE_NOP;
  logicdata[340] = CHANPIN;
  INLINE_NOP;
  logicdata[341] = CHANPIN;
  INLINE_NOP;
  logicdata[342] = CHANPIN;
  INLINE_NOP;
  logicdata[343] = CHANPIN;
  INLINE_NOP;
  logicdata[344] = CHANPIN;
  INLINE_NOP;
  logicdata[345] = CHANPIN;
  INLINE_NOP;
  logicdata[346] = CHANPIN;
  INLINE_NOP;
  logicdata[347] = CHANPIN;
  INLINE_NOP;
  logicdata[348] = CHANPIN;
  INLINE_NOP;
  logicdata[349] = CHANPIN;
  INLINE_NOP;
  logicdata[350] = CHANPIN;
  INLINE_NOP;
  logicdata[351] = CHANPIN;
  INLINE_NOP;
  logicdata[352] = CHANPIN;
  INLINE_NOP;
  logicdata[353] = CHANPIN;
  INLINE_NOP;
  logicdata[354] = CHANPIN;
  INLINE_NOP;
  logicdata[355] = CHANPIN;
  INLINE_NOP;
  logicdata[356] = CHANPIN;
  INLINE_NOP;
  logicdata[357] = CHANPIN;
  INLINE_NOP;
  logicdata[358] = CHANPIN;
  INLINE_NOP;
  logicdata[359] = CHANPIN;
  INLINE_NOP;
  logicdata[360] = CHANPIN;
  INLINE_NOP;
  logicdata[361] = CHANPIN;
  INLINE_NOP;
  logicdata[362] = CHANPIN;
  INLINE_NOP;
  logicdata[363] = CHANPIN;
  INLINE_NOP;
  logicdata[364] = CHANPIN;
  INLINE_NOP;
  logicdata[365] = CHANPIN;
  INLINE_NOP;
  logicdata[366] = CHANPIN;
  INLINE_NOP;
  logicdata[367] = CHANPIN;
  INLINE_NOP;
  logicdata[368] = CHANPIN;
  INLINE_NOP;
  logicdata[369] = CHANPIN;
  INLINE_NOP;
  logicdata[370] = CHANPIN;
  INLINE_NOP;
  logicdata[371] = CHANPIN;
  INLINE_NOP;
  logicdata[372] = CHANPIN;
  INLINE_NOP;
  logicdata[373] = CHANPIN;
  INLINE_NOP;
  logicdata[374] = CHANPIN;
  INLINE_NOP;
  logicdata[375] = CHANPIN;
  INLINE_NOP;
  logicdata[376] = CHANPIN;
  INLINE_NOP;
  logicdata[377] = CHANPIN;
  INLINE_NOP;
  logicdata[378] = CHANPIN;
  INLINE_NOP;
  logicdata[379] = CHANPIN;
  INLINE_NOP;
  logicdata[380] = CHANPIN;
  INLINE_NOP;
  logicdata[381] = CHANPIN;
  INLINE_NOP;
  logicdata[382] = CHANPIN;
  INLINE_NOP;
  logicdata[383] = CHANPIN;
  INLINE_NOP;
  logicdata[384] = CHANPIN;
  INLINE_NOP;
  logicdata[385] = CHANPIN;
  INLINE_NOP;
  logicdata[386] = CHANPIN;
  INLINE_NOP;
  logicdata[387] = CHANPIN;
  INLINE_NOP;
  logicdata[388] = CHANPIN;
  INLINE_NOP;
  logicdata[389] = CHANPIN;
  INLINE_NOP;
  logicdata[390] = CHANPIN;
  INLINE_NOP;
  logicdata[391] = CHANPIN;
  INLINE_NOP;
  logicdata[392] = CHANPIN;
  INLINE_NOP;
  logicdata[393] = CHANPIN;
  INLINE_NOP;
  logicdata[394] = CHANPIN;
  INLINE_NOP;
  logicdata[395] = CHANPIN;
  INLINE_NOP;
  logicdata[396] = CHANPIN;
  INLINE_NOP;
  logicdata[397] = CHANPIN;
  INLINE_NOP;
  logicdata[398] = CHANPIN;
  INLINE_NOP;
  logicdata[399] = CHANPIN;
  INLINE_NOP;
  logicdata[400] = CHANPIN;
  INLINE_NOP;
  logicdata[401] = CHANPIN;
  INLINE_NOP;
  logicdata[402] = CHANPIN;
  INLINE_NOP;
  logicdata[403] = CHANPIN;
  INLINE_NOP;
  logicdata[404] = CHANPIN;
  INLINE_NOP;
  logicdata[405] = CHANPIN;
  INLINE_NOP;
  logicdata[406] = CHANPIN;
  INLINE_NOP;
  logicdata[407] = CHANPIN;
  INLINE_NOP;
  logicdata[408] = CHANPIN;
  INLINE_NOP;
  logicdata[409] = CHANPIN;
  INLINE_NOP;
  logicdata[410] = CHANPIN;
  INLINE_NOP;
  logicdata[411] = CHANPIN;
  INLINE_NOP;
  logicdata[412] = CHANPIN;
  INLINE_NOP;
  logicdata[413] = CHANPIN;
  INLINE_NOP;
  logicdata[414] = CHANPIN;
  INLINE_NOP;
  logicdata[415] = CHANPIN;
  INLINE_NOP;
  logicdata[416] = CHANPIN;
  INLINE_NOP;
  logicdata[417] = CHANPIN;
  INLINE_NOP;
  logicdata[418] = CHANPIN;
  INLINE_NOP;
  logicdata[419] = CHANPIN;
  INLINE_NOP;
  logicdata[420] = CHANPIN;
  INLINE_NOP;
  logicdata[421] = CHANPIN;
  INLINE_NOP;
  logicdata[422] = CHANPIN;
  INLINE_NOP;
  logicdata[423] = CHANPIN;
  INLINE_NOP;
  logicdata[424] = CHANPIN;
  INLINE_NOP;
  logicdata[425] = CHANPIN;
  INLINE_NOP;
  logicdata[426] = CHANPIN;
  INLINE_NOP;
  logicdata[427] = CHANPIN;
  INLINE_NOP;
  logicdata[428] = CHANPIN;
  INLINE_NOP;
  logicdata[429] = CHANPIN;
  INLINE_NOP;
  logicdata[430] = CHANPIN;
  INLINE_NOP;
  logicdata[431] = CHANPIN;
  INLINE_NOP;
  logicdata[432] = CHANPIN;
  INLINE_NOP;
  logicdata[433] = CHANPIN;
  INLINE_NOP;
  logicdata[434] = CHANPIN;
  INLINE_NOP;
  logicdata[435] = CHANPIN;
  INLINE_NOP;
  logicdata[436] = CHANPIN;
  INLINE_NOP;
  logicdata[437] = CHANPIN;
  INLINE_NOP;
  logicdata[438] = CHANPIN;
  INLINE_NOP;
  logicdata[439] = CHANPIN;
  INLINE_NOP;
  logicdata[440] = CHANPIN;
  INLINE_NOP;
  logicdata[441] = CHANPIN;
  INLINE_NOP;
  logicdata[442] = CHANPIN;
  INLINE_NOP;
  logicdata[443] = CHANPIN;
  INLINE_NOP;
  logicdata[444] = CHANPIN;
  INLINE_NOP;
  logicdata[445] = CHANPIN;
  INLINE_NOP;
  logicdata[446] = CHANPIN;
  INLINE_NOP;
  logicdata[447] = CHANPIN;
  INLINE_NOP;
  logicdata[448] = CHANPIN;
  INLINE_NOP;
  logicdata[449] = CHANPIN;
  INLINE_NOP;
  logicdata[450] = CHANPIN;
  INLINE_NOP;
  logicdata[451] = CHANPIN;
  INLINE_NOP;
  logicdata[452] = CHANPIN;
  INLINE_NOP;
  logicdata[453] = CHANPIN;
  INLINE_NOP;
  logicdata[454] = CHANPIN;
  INLINE_NOP;
  logicdata[455] = CHANPIN;
  INLINE_NOP;
  logicdata[456] = CHANPIN;
  INLINE_NOP;
  logicdata[457] = CHANPIN;
  INLINE_NOP;
  logicdata[458] = CHANPIN;
  INLINE_NOP;
  logicdata[459] = CHANPIN;
  INLINE_NOP;
  logicdata[460] = CHANPIN;
  INLINE_NOP;
  logicdata[461] = CHANPIN;
  INLINE_NOP;
  logicdata[462] = CHANPIN;
  INLINE_NOP;
  logicdata[463] = CHANPIN;
  INLINE_NOP;
  logicdata[464] = CHANPIN;
  INLINE_NOP;
  logicdata[465] = CHANPIN;
  INLINE_NOP;
  logicdata[466] = CHANPIN;
  INLINE_NOP;
  logicdata[467] = CHANPIN;
  INLINE_NOP;
  logicdata[468] = CHANPIN;
  INLINE_NOP;
  logicdata[469] = CHANPIN;
  INLINE_NOP;
  logicdata[470] = CHANPIN;
  INLINE_NOP;
  logicdata[471] = CHANPIN;
  INLINE_NOP;
  logicdata[472] = CHANPIN;
  INLINE_NOP;
  logicdata[473] = CHANPIN;
  INLINE_NOP;
  logicdata[474] = CHANPIN;
  INLINE_NOP;
  logicdata[475] = CHANPIN;
  INLINE_NOP;
  logicdata[476] = CHANPIN;
  INLINE_NOP;
  logicdata[477] = CHANPIN;
  INLINE_NOP;
  logicdata[478] = CHANPIN;
  INLINE_NOP;
  logicdata[479] = CHANPIN;
  INLINE_NOP;
  logicdata[480] = CHANPIN;
  INLINE_NOP;
  logicdata[481] = CHANPIN;
  INLINE_NOP;
  logicdata[482] = CHANPIN;
  INLINE_NOP;
  logicdata[483] = CHANPIN;
  INLINE_NOP;
  logicdata[484] = CHANPIN;
  INLINE_NOP;
  logicdata[485] = CHANPIN;
  INLINE_NOP;
  logicdata[486] = CHANPIN;
  INLINE_NOP;
  logicdata[487] = CHANPIN;
  INLINE_NOP;
  logicdata[488] = CHANPIN;
  INLINE_NOP;
  logicdata[489] = CHANPIN;
  INLINE_NOP;
  logicdata[490] = CHANPIN;
  INLINE_NOP;
  logicdata[491] = CHANPIN;
  INLINE_NOP;
  logicdata[492] = CHANPIN;
  INLINE_NOP;
  logicdata[493] = CHANPIN;
  INLINE_NOP;
  logicdata[494] = CHANPIN;
  INLINE_NOP;
  logicdata[495] = CHANPIN;
  INLINE_NOP;
  logicdata[496] = CHANPIN;
  INLINE_NOP;
  logicdata[497] = CHANPIN;
  INLINE_NOP;
  logicdata[498] = CHANPIN;
  INLINE_NOP;
  logicdata[499] = CHANPIN;
  INLINE_NOP;
  logicdata[500] = CHANPIN;
  INLINE_NOP;
  logicdata[501] = CHANPIN;
  INLINE_NOP;
  logicdata[502] = CHANPIN;
  INLINE_NOP;
  logicdata[503] = CHANPIN;
  INLINE_NOP;
  logicdata[504] = CHANPIN;
  INLINE_NOP;
  logicdata[505] = CHANPIN;
  INLINE_NOP;
  logicdata[506] = CHANPIN;
  INLINE_NOP;
  logicdata[507] = CHANPIN;
  INLINE_NOP;
  logicdata[508] = CHANPIN;
  INLINE_NOP;
  logicdata[509] = CHANPIN;
  INLINE_NOP;
  logicdata[510] = CHANPIN;
  INLINE_NOP;
  logicdata[511] = CHANPIN;
  INLINE_NOP;
  logicdata[512] = CHANPIN;
  INLINE_NOP;
  logicdata[513] = CHANPIN;
  INLINE_NOP;
  logicdata[514] = CHANPIN;
  INLINE_NOP;
  logicdata[515] = CHANPIN;
  INLINE_NOP;
  logicdata[516] = CHANPIN;
  INLINE_NOP;
  logicdata[517] = CHANPIN;
  INLINE_NOP;
  logicdata[518] = CHANPIN;
  INLINE_NOP;
  logicdata[519] = CHANPIN;
  INLINE_NOP;
  logicdata[520] = CHANPIN;
  INLINE_NOP;
  logicdata[521] = CHANPIN;
  INLINE_NOP;
  logicdata[522] = CHANPIN;
  INLINE_NOP;
  logicdata[523] = CHANPIN;
  INLINE_NOP;
  logicdata[524] = CHANPIN;
  INLINE_NOP;
  logicdata[525] = CHANPIN;
  INLINE_NOP;
  logicdata[526] = CHANPIN;
  INLINE_NOP;
  logicdata[527] = CHANPIN;
  INLINE_NOP;
  logicdata[528] = CHANPIN;
  INLINE_NOP;
  logicdata[529] = CHANPIN;
  INLINE_NOP;
  logicdata[530] = CHANPIN;
  INLINE_NOP;
  logicdata[531] = CHANPIN;
  INLINE_NOP;
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  logicdata[532] = CHANPIN;
  INLINE_NOP;
  logicdata[533] = CHANPIN;
  INLINE_NOP;
  logicdata[534] = CHANPIN;
  INLINE_NOP;
  logicdata[535] = CHANPIN;
  INLINE_NOP;
  logicdata[536] = CHANPIN;
  INLINE_NOP;
  logicdata[537] = CHANPIN;
  INLINE_NOP;
  logicdata[538] = CHANPIN;
  INLINE_NOP;
  logicdata[539] = CHANPIN;
  INLINE_NOP;
  logicdata[540] = CHANPIN;
  INLINE_NOP;
  logicdata[541] = CHANPIN;
  INLINE_NOP;
  logicdata[542] = CHANPIN;
  INLINE_NOP;
  logicdata[543] = CHANPIN;
  INLINE_NOP;
  logicdata[544] = CHANPIN;
  INLINE_NOP;
  logicdata[545] = CHANPIN;
  INLINE_NOP;
  logicdata[546] = CHANPIN;
  INLINE_NOP;
  logicdata[547] = CHANPIN;
  INLINE_NOP;
  logicdata[548] = CHANPIN;
  INLINE_NOP;
  logicdata[549] = CHANPIN;
  INLINE_NOP;
  logicdata[550] = CHANPIN;
  INLINE_NOP;
  logicdata[551] = CHANPIN;
  INLINE_NOP;
  logicdata[552] = CHANPIN;
  INLINE_NOP;
  logicdata[553] = CHANPIN;
  INLINE_NOP;
  logicdata[554] = CHANPIN;
  INLINE_NOP;
  logicdata[555] = CHANPIN;
  INLINE_NOP;
  logicdata[556] = CHANPIN;
  INLINE_NOP;
  logicdata[557] = CHANPIN;
  INLINE_NOP;
  logicdata[558] = CHANPIN;
  INLINE_NOP;
  logicdata[559] = CHANPIN;
  INLINE_NOP;
  logicdata[560] = CHANPIN;
  INLINE_NOP;
  logicdata[561] = CHANPIN;
  INLINE_NOP;
  logicdata[562] = CHANPIN;
  INLINE_NOP;
  logicdata[563] = CHANPIN;
  INLINE_NOP;
  logicdata[564] = CHANPIN;
  INLINE_NOP;
  logicdata[565] = CHANPIN;
  INLINE_NOP;
  logicdata[566] = CHANPIN;
  INLINE_NOP;
  logicdata[567] = CHANPIN;
  INLINE_NOP;
  logicdata[568] = CHANPIN;
  INLINE_NOP;
  logicdata[569] = CHANPIN;
  INLINE_NOP;
  logicdata[570] = CHANPIN;
  INLINE_NOP;
  logicdata[571] = CHANPIN;
  INLINE_NOP;
  logicdata[572] = CHANPIN;
  INLINE_NOP;
  logicdata[573] = CHANPIN;
  INLINE_NOP;
  logicdata[574] = CHANPIN;
  INLINE_NOP;
  logicdata[575] = CHANPIN;
  INLINE_NOP;
  logicdata[576] = CHANPIN;
  INLINE_NOP;
  logicdata[577] = CHANPIN;
  INLINE_NOP;
  logicdata[578] = CHANPIN;
  INLINE_NOP;
  logicdata[579] = CHANPIN;
  INLINE_NOP;
  logicdata[580] = CHANPIN;
  INLINE_NOP;
  logicdata[581] = CHANPIN;
  INLINE_NOP;
  logicdata[582] = CHANPIN;
  INLINE_NOP;
  logicdata[583] = CHANPIN;
  INLINE_NOP;
  logicdata[584] = CHANPIN;
  INLINE_NOP;
  logicdata[585] = CHANPIN;
  INLINE_NOP;
  logicdata[586] = CHANPIN;
  INLINE_NOP;
  logicdata[587] = CHANPIN;
  INLINE_NOP;
  logicdata[588] = CHANPIN;
  INLINE_NOP;
  logicdata[589] = CHANPIN;
  INLINE_NOP;
  logicdata[590] = CHANPIN;
  INLINE_NOP;
  logicdata[591] = CHANPIN;
  INLINE_NOP;
  logicdata[592] = CHANPIN;
  INLINE_NOP;
  logicdata[593] = CHANPIN;
  INLINE_NOP;
  logicdata[594] = CHANPIN;
  INLINE_NOP;
  logicdata[595] = CHANPIN;
  INLINE_NOP;
  logicdata[596] = CHANPIN;
  INLINE_NOP;
  logicdata[597] = CHANPIN;
  INLINE_NOP;
  logicdata[598] = CHANPIN;
  INLINE_NOP;
  logicdata[599] = CHANPIN;
  INLINE_NOP;
  logicdata[600] = CHANPIN;
  INLINE_NOP;
  logicdata[601] = CHANPIN;
  INLINE_NOP;
  logicdata[602] = CHANPIN;
  INLINE_NOP;
  logicdata[603] = CHANPIN;
  INLINE_NOP;
  logicdata[604] = CHANPIN;
  INLINE_NOP;
  logicdata[605] = CHANPIN;
  INLINE_NOP;
  logicdata[606] = CHANPIN;
  INLINE_NOP;
  logicdata[607] = CHANPIN;
  INLINE_NOP;
  logicdata[608] = CHANPIN;
  INLINE_NOP;
  logicdata[609] = CHANPIN;
  INLINE_NOP;
  logicdata[610] = CHANPIN;
  INLINE_NOP;
  logicdata[611] = CHANPIN;
  INLINE_NOP;
  logicdata[612] = CHANPIN;
  INLINE_NOP;
  logicdata[613] = CHANPIN;
  INLINE_NOP;
  logicdata[614] = CHANPIN;
  INLINE_NOP;
  logicdata[615] = CHANPIN;
  INLINE_NOP;
  logicdata[616] = CHANPIN;
  INLINE_NOP;
  logicdata[617] = CHANPIN;
  INLINE_NOP;
  logicdata[618] = CHANPIN;
  INLINE_NOP;
  logicdata[619] = CHANPIN;
  INLINE_NOP;
  logicdata[620] = CHANPIN;
  INLINE_NOP;
  logicdata[621] = CHANPIN;
  INLINE_NOP;
  logicdata[622] = CHANPIN;
  INLINE_NOP;
  logicdata[623] = CHANPIN;
  INLINE_NOP;
  logicdata[624] = CHANPIN;
  INLINE_NOP;
  logicdata[625] = CHANPIN;
  INLINE_NOP;
  logicdata[626] = CHANPIN;
  INLINE_NOP;
  logicdata[627] = CHANPIN;
  INLINE_NOP;
  logicdata[628] = CHANPIN;
  INLINE_NOP;
  logicdata[629] = CHANPIN;
  INLINE_NOP;
  logicdata[630] = CHANPIN;
  INLINE_NOP;
  logicdata[631] = CHANPIN;
  INLINE_NOP;
  logicdata[632] = CHANPIN;
  INLINE_NOP;
  logicdata[633] = CHANPIN;
  INLINE_NOP;
  logicdata[634] = CHANPIN;
  INLINE_NOP;
  logicdata[635] = CHANPIN;
  INLINE_NOP;
  logicdata[636] = CHANPIN;
  INLINE_NOP;
  logicdata[637] = CHANPIN;
  INLINE_NOP;
  logicdata[638] = CHANPIN;
  INLINE_NOP;
  logicdata[639] = CHANPIN;
  INLINE_NOP;
  logicdata[640] = CHANPIN;
  INLINE_NOP;
  logicdata[641] = CHANPIN;
  INLINE_NOP;
  logicdata[642] = CHANPIN;
  INLINE_NOP;
  logicdata[643] = CHANPIN;
  INLINE_NOP;
  logicdata[644] = CHANPIN;
  INLINE_NOP;
  logicdata[645] = CHANPIN;
  INLINE_NOP;
  logicdata[646] = CHANPIN;
  INLINE_NOP;
  logicdata[647] = CHANPIN;
  INLINE_NOP;
  logicdata[648] = CHANPIN;
  INLINE_NOP;
  logicdata[649] = CHANPIN;
  INLINE_NOP;
  logicdata[650] = CHANPIN;
  INLINE_NOP;
  logicdata[651] = CHANPIN;
  INLINE_NOP;
  logicdata[652] = CHANPIN;
  INLINE_NOP;
  logicdata[653] = CHANPIN;
  INLINE_NOP;
  logicdata[654] = CHANPIN;
  INLINE_NOP;
  logicdata[655] = CHANPIN;
  INLINE_NOP;
  logicdata[656] = CHANPIN;
  INLINE_NOP;
  logicdata[657] = CHANPIN;
  INLINE_NOP;
  logicdata[658] = CHANPIN;
  INLINE_NOP;
  logicdata[659] = CHANPIN;
  INLINE_NOP;
  logicdata[660] = CHANPIN;
  INLINE_NOP;
  logicdata[661] = CHANPIN;
  INLINE_NOP;
  logicdata[662] = CHANPIN;
  INLINE_NOP;
  logicdata[663] = CHANPIN;
  INLINE_NOP;
  logicdata[664] = CHANPIN;
  INLINE_NOP;
  logicdata[665] = CHANPIN;
  INLINE_NOP;
  logicdata[666] = CHANPIN;
  INLINE_NOP;
  logicdata[667] = CHANPIN;
  INLINE_NOP;
  logicdata[668] = CHANPIN;
  INLINE_NOP;
  logicdata[669] = CHANPIN;
  INLINE_NOP;
  logicdata[670] = CHANPIN;
  INLINE_NOP;
  logicdata[671] = CHANPIN;
  INLINE_NOP;
  logicdata[672] = CHANPIN;
  INLINE_NOP;
  logicdata[673] = CHANPIN;
  INLINE_NOP;
  logicdata[674] = CHANPIN;
  INLINE_NOP;
  logicdata[675] = CHANPIN;
  INLINE_NOP;
  logicdata[676] = CHANPIN;
  INLINE_NOP;
  logicdata[677] = CHANPIN;
  INLINE_NOP;
  logicdata[678] = CHANPIN;
  INLINE_NOP;
  logicdata[679] = CHANPIN;
  INLINE_NOP;
  logicdata[680] = CHANPIN;
  INLINE_NOP;
  logicdata[681] = CHANPIN;
  INLINE_NOP;
  logicdata[682] = CHANPIN;
  INLINE_NOP;
  logicdata[683] = CHANPIN;
  INLINE_NOP;
  logicdata[684] = CHANPIN;
  INLINE_NOP;
  logicdata[685] = CHANPIN;
  INLINE_NOP;
  logicdata[686] = CHANPIN;
  INLINE_NOP;
  logicdata[687] = CHANPIN;
  INLINE_NOP;
  logicdata[688] = CHANPIN;
  INLINE_NOP;
  logicdata[689] = CHANPIN;
  INLINE_NOP;
  logicdata[690] = CHANPIN;
  INLINE_NOP;
  logicdata[691] = CHANPIN;
  INLINE_NOP;
  logicdata[692] = CHANPIN;
  INLINE_NOP;
  logicdata[693] = CHANPIN;
  INLINE_NOP;
  logicdata[694] = CHANPIN;
  INLINE_NOP;
  logicdata[695] = CHANPIN;
  INLINE_NOP;
  logicdata[696] = CHANPIN;
  INLINE_NOP;
  logicdata[697] = CHANPIN;
  INLINE_NOP;
  logicdata[698] = CHANPIN;
  INLINE_NOP;
  logicdata[699] = CHANPIN;
  INLINE_NOP;
  logicdata[700] = CHANPIN;
  INLINE_NOP;
  logicdata[701] = CHANPIN;
  INLINE_NOP;
  logicdata[702] = CHANPIN;
  INLINE_NOP;
  logicdata[703] = CHANPIN;
  INLINE_NOP;
  logicdata[704] = CHANPIN;
  INLINE_NOP;
  logicdata[705] = CHANPIN;
  INLINE_NOP;
  logicdata[706] = CHANPIN;
  INLINE_NOP;
  logicdata[707] = CHANPIN;
  INLINE_NOP;
  logicdata[708] = CHANPIN;
  INLINE_NOP;
  logicdata[709] = CHANPIN;
  INLINE_NOP;
  logicdata[710] = CHANPIN;
  INLINE_NOP;
  logicdata[711] = CHANPIN;
  INLINE_NOP;
  logicdata[712] = CHANPIN;
  INLINE_NOP;
  logicdata[713] = CHANPIN;
  INLINE_NOP;
  logicdata[714] = CHANPIN;
  INLINE_NOP;
  logicdata[715] = CHANPIN;
  INLINE_NOP;
  logicdata[716] = CHANPIN;
  INLINE_NOP;
  logicdata[717] = CHANPIN;
  INLINE_NOP;
  logicdata[718] = CHANPIN;
  INLINE_NOP;
  logicdata[719] = CHANPIN;
  INLINE_NOP;
  logicdata[720] = CHANPIN;
  INLINE_NOP;
  logicdata[721] = CHANPIN;
  INLINE_NOP;
  logicdata[722] = CHANPIN;
  INLINE_NOP;
  logicdata[723] = CHANPIN;
  INLINE_NOP;
  logicdata[724] = CHANPIN;
  INLINE_NOP;
  logicdata[725] = CHANPIN;
  INLINE_NOP;
  logicdata[726] = CHANPIN;
  INLINE_NOP;
  logicdata[727] = CHANPIN;
  INLINE_NOP;
  logicdata[728] = CHANPIN;
  INLINE_NOP;
  logicdata[729] = CHANPIN;
  INLINE_NOP;
  logicdata[730] = CHANPIN;
  INLINE_NOP;
  logicdata[731] = CHANPIN;
  INLINE_NOP;
  logicdata[732] = CHANPIN;
  INLINE_NOP;
  logicdata[733] = CHANPIN;
  INLINE_NOP;
  logicdata[734] = CHANPIN;
  INLINE_NOP;
  logicdata[735] = CHANPIN;
  INLINE_NOP;
  logicdata[736] = CHANPIN;
  INLINE_NOP;
  logicdata[737] = CHANPIN;
  INLINE_NOP;
  logicdata[738] = CHANPIN;
  INLINE_NOP;
  logicdata[739] = CHANPIN;
  INLINE_NOP;
  logicdata[740] = CHANPIN;
  INLINE_NOP;
  logicdata[741] = CHANPIN;
  INLINE_NOP;
  logicdata[742] = CHANPIN;
  INLINE_NOP;
  logicdata[743] = CHANPIN;
  INLINE_NOP;
  logicdata[744] = CHANPIN;
  INLINE_NOP;
  logicdata[745] = CHANPIN;
  INLINE_NOP;
  logicdata[746] = CHANPIN;
  INLINE_NOP;
  logicdata[747] = CHANPIN;
  INLINE_NOP;
  logicdata[748] = CHANPIN;
  INLINE_NOP;
  logicdata[749] = CHANPIN;
  INLINE_NOP;
  logicdata[750] = CHANPIN;
  INLINE_NOP;
  logicdata[751] = CHANPIN;
  INLINE_NOP;
  logicdata[752] = CHANPIN;
  INLINE_NOP;
  logicdata[753] = CHANPIN;
  INLINE_NOP;
  logicdata[754] = CHANPIN;
  INLINE_NOP;
  logicdata[755] = CHANPIN;
  INLINE_NOP;
  logicdata[756] = CHANPIN;
  INLINE_NOP;
  logicdata[757] = CHANPIN;
  INLINE_NOP;
  logicdata[758] = CHANPIN;
  INLINE_NOP;
  logicdata[759] = CHANPIN;
  INLINE_NOP;
  logicdata[760] = CHANPIN;
  INLINE_NOP;
  logicdata[761] = CHANPIN;
  INLINE_NOP;
  logicdata[762] = CHANPIN;
  INLINE_NOP;
  logicdata[763] = CHANPIN;
  INLINE_NOP;
  logicdata[764] = CHANPIN;
  INLINE_NOP;
  logicdata[765] = CHANPIN;
  INLINE_NOP;
  logicdata[766] = CHANPIN;
  INLINE_NOP;
  logicdata[767] = CHANPIN;
  INLINE_NOP;
  logicdata[768] = CHANPIN;
  INLINE_NOP;
  logicdata[769] = CHANPIN;
  INLINE_NOP;
  logicdata[770] = CHANPIN;
  INLINE_NOP;
  logicdata[771] = CHANPIN;
  INLINE_NOP;
  logicdata[772] = CHANPIN;
  INLINE_NOP;
  logicdata[773] = CHANPIN;
  INLINE_NOP;
  logicdata[774] = CHANPIN;
  INLINE_NOP;
  logicdata[775] = CHANPIN;
  INLINE_NOP;
  logicdata[776] = CHANPIN;
  INLINE_NOP;
  logicdata[777] = CHANPIN;
  INLINE_NOP;
  logicdata[778] = CHANPIN;
  INLINE_NOP;
  logicdata[779] = CHANPIN;
  INLINE_NOP;
  logicdata[780] = CHANPIN;
  INLINE_NOP;
  logicdata[781] = CHANPIN;
  INLINE_NOP;
  logicdata[782] = CHANPIN;
  INLINE_NOP;
  logicdata[783] = CHANPIN;
  INLINE_NOP;
  logicdata[784] = CHANPIN;
  INLINE_NOP;
  logicdata[785] = CHANPIN;
  INLINE_NOP;
  logicdata[786] = CHANPIN;
  INLINE_NOP;
  logicdata[787] = CHANPIN;
  INLINE_NOP;
  logicdata[788] = CHANPIN;
  INLINE_NOP;
  logicdata[789] = CHANPIN;
  INLINE_NOP;
  logicdata[790] = CHANPIN;
  INLINE_NOP;
  logicdata[791] = CHANPIN;
  INLINE_NOP;
  logicdata[792] = CHANPIN;
  INLINE_NOP;
  logicdata[793] = CHANPIN;
  INLINE_NOP;
  logicdata[794] = CHANPIN;
  INLINE_NOP;
  logicdata[795] = CHANPIN;
  INLINE_NOP;
  logicdata[796] = CHANPIN;
  INLINE_NOP;
  logicdata[797] = CHANPIN;
  INLINE_NOP;
  logicdata[798] = CHANPIN;
  INLINE_NOP;
  logicdata[799] = CHANPIN;
  INLINE_NOP;
  logicdata[800] = CHANPIN;
  INLINE_NOP;
  logicdata[801] = CHANPIN;
  INLINE_NOP;
  logicdata[802] = CHANPIN;
  INLINE_NOP;
  logicdata[803] = CHANPIN;
  INLINE_NOP;
  logicdata[804] = CHANPIN;
  INLINE_NOP;
  logicdata[805] = CHANPIN;
  INLINE_NOP;
  logicdata[806] = CHANPIN;
  INLINE_NOP;
  logicdata[807] = CHANPIN;
  INLINE_NOP;
  logicdata[808] = CHANPIN;
  INLINE_NOP;
  logicdata[809] = CHANPIN;
  INLINE_NOP;
  logicdata[810] = CHANPIN;
  INLINE_NOP;
  logicdata[811] = CHANPIN;
  INLINE_NOP;
  logicdata[812] = CHANPIN;
  INLINE_NOP;
  logicdata[813] = CHANPIN;
  INLINE_NOP;
  logicdata[814] = CHANPIN;
  INLINE_NOP;
  logicdata[815] = CHANPIN;
  INLINE_NOP;
  logicdata[816] = CHANPIN;
  INLINE_NOP;
  logicdata[817] = CHANPIN;
  INLINE_NOP;
  logicdata[818] = CHANPIN;
  INLINE_NOP;
  logicdata[819] = CHANPIN;
  INLINE_NOP;
  logicdata[820] = CHANPIN;
  INLINE_NOP;
  logicdata[821] = CHANPIN;
  INLINE_NOP;
  logicdata[822] = CHANPIN;
  INLINE_NOP;
  logicdata[823] = CHANPIN;
  INLINE_NOP;
  logicdata[824] = CHANPIN;
  INLINE_NOP;
  logicdata[825] = CHANPIN;
  INLINE_NOP;
  logicdata[826] = CHANPIN;
  INLINE_NOP;
  logicdata[827] = CHANPIN;
  INLINE_NOP;
  logicdata[828] = CHANPIN;
  INLINE_NOP;
  logicdata[829] = CHANPIN;
  INLINE_NOP;
  logicdata[830] = CHANPIN;
  INLINE_NOP;
  logicdata[831] = CHANPIN;
  INLINE_NOP;
  logicdata[832] = CHANPIN;
  INLINE_NOP;
  logicdata[833] = CHANPIN;
  INLINE_NOP;
  logicdata[834] = CHANPIN;
  INLINE_NOP;
  logicdata[835] = CHANPIN;
  INLINE_NOP;
  logicdata[836] = CHANPIN;
  INLINE_NOP;
  logicdata[837] = CHANPIN;
  INLINE_NOP;
  logicdata[838] = CHANPIN;
  INLINE_NOP;
  logicdata[839] = CHANPIN;
  INLINE_NOP;
  logicdata[840] = CHANPIN;
  INLINE_NOP;
  logicdata[841] = CHANPIN;
  INLINE_NOP;
  logicdata[842] = CHANPIN;
  INLINE_NOP;
  logicdata[843] = CHANPIN;
  INLINE_NOP;
  logicdata[844] = CHANPIN;
  INLINE_NOP;
  logicdata[845] = CHANPIN;
  INLINE_NOP;
  logicdata[846] = CHANPIN;
  INLINE_NOP;
  logicdata[847] = CHANPIN;
  INLINE_NOP;
  logicdata[848] = CHANPIN;
  INLINE_NOP;
  logicdata[849] = CHANPIN;
  INLINE_NOP;
  logicdata[850] = CHANPIN;
  INLINE_NOP;
  logicdata[851] = CHANPIN;
  INLINE_NOP;
  logicdata[852] = CHANPIN;
  INLINE_NOP;
  logicdata[853] = CHANPIN;
  INLINE_NOP;
  logicdata[854] = CHANPIN;
  INLINE_NOP;
  logicdata[855] = CHANPIN;
  INLINE_NOP;
  logicdata[856] = CHANPIN;
  INLINE_NOP;
  logicdata[857] = CHANPIN;
  INLINE_NOP;
  logicdata[858] = CHANPIN;
  INLINE_NOP;
  logicdata[859] = CHANPIN;
  INLINE_NOP;
  logicdata[860] = CHANPIN;
  INLINE_NOP;
  logicdata[861] = CHANPIN;
  INLINE_NOP;
  logicdata[862] = CHANPIN;
  INLINE_NOP;
  logicdata[863] = CHANPIN;
  INLINE_NOP;
  logicdata[864] = CHANPIN;
  INLINE_NOP;
  logicdata[865] = CHANPIN;
  INLINE_NOP;
  logicdata[866] = CHANPIN;
  INLINE_NOP;
  logicdata[867] = CHANPIN;
  INLINE_NOP;
  logicdata[868] = CHANPIN;
  INLINE_NOP;
  logicdata[869] = CHANPIN;
  INLINE_NOP;
  logicdata[870] = CHANPIN;
  INLINE_NOP;
  logicdata[871] = CHANPIN;
  INLINE_NOP;
  logicdata[872] = CHANPIN;
  INLINE_NOP;
  logicdata[873] = CHANPIN;
  INLINE_NOP;
  logicdata[874] = CHANPIN;
  INLINE_NOP;
  logicdata[875] = CHANPIN;
  INLINE_NOP;
  logicdata[876] = CHANPIN;
  INLINE_NOP;
  logicdata[877] = CHANPIN;
  INLINE_NOP;
  logicdata[878] = CHANPIN;
  INLINE_NOP;
  logicdata[879] = CHANPIN;
  INLINE_NOP;
  logicdata[880] = CHANPIN;
  INLINE_NOP;
  logicdata[881] = CHANPIN;
  INLINE_NOP;
  logicdata[882] = CHANPIN;
  INLINE_NOP;
  logicdata[883] = CHANPIN;
  INLINE_NOP;
  logicdata[884] = CHANPIN;
  INLINE_NOP;
  logicdata[885] = CHANPIN;
  INLINE_NOP;
  logicdata[886] = CHANPIN;
  INLINE_NOP;
  logicdata[887] = CHANPIN;
  INLINE_NOP;
  logicdata[888] = CHANPIN;
  INLINE_NOP;
  logicdata[889] = CHANPIN;
  INLINE_NOP;
  logicdata[890] = CHANPIN;
  INLINE_NOP;
  logicdata[891] = CHANPIN;
  INLINE_NOP;
  logicdata[892] = CHANPIN;
  INLINE_NOP;
  logicdata[893] = CHANPIN;
  INLINE_NOP;
  logicdata[894] = CHANPIN;
  INLINE_NOP;
  logicdata[895] = CHANPIN;
  INLINE_NOP;
  logicdata[896] = CHANPIN;
  INLINE_NOP;
  logicdata[897] = CHANPIN;
  INLINE_NOP;
  logicdata[898] = CHANPIN;
  INLINE_NOP;
  logicdata[899] = CHANPIN;
  INLINE_NOP;
  logicdata[900] = CHANPIN;
  INLINE_NOP;
  logicdata[901] = CHANPIN;
  INLINE_NOP;
  logicdata[902] = CHANPIN;
  INLINE_NOP;
  logicdata[903] = CHANPIN;
  INLINE_NOP;
  logicdata[904] = CHANPIN;
  INLINE_NOP;
  logicdata[905] = CHANPIN;
  INLINE_NOP;
  logicdata[906] = CHANPIN;
  INLINE_NOP;
  logicdata[907] = CHANPIN;
  INLINE_NOP;
  logicdata[908] = CHANPIN;
  INLINE_NOP;
  logicdata[909] = CHANPIN;
  INLINE_NOP;
  logicdata[910] = CHANPIN;
  INLINE_NOP;
  logicdata[911] = CHANPIN;
  INLINE_NOP;
  logicdata[912] = CHANPIN;
  INLINE_NOP;
  logicdata[913] = CHANPIN;
  INLINE_NOP;
  logicdata[914] = CHANPIN;
  INLINE_NOP;
  logicdata[915] = CHANPIN;
  INLINE_NOP;
  logicdata[916] = CHANPIN;
  INLINE_NOP;
  logicdata[917] = CHANPIN;
  INLINE_NOP;
  logicdata[918] = CHANPIN;
  INLINE_NOP;
  logicdata[919] = CHANPIN;
  INLINE_NOP;
  logicdata[920] = CHANPIN;
  INLINE_NOP;
  logicdata[921] = CHANPIN;
  INLINE_NOP;
  logicdata[922] = CHANPIN;
  INLINE_NOP;
  logicdata[923] = CHANPIN;
  INLINE_NOP;
  logicdata[924] = CHANPIN;
  INLINE_NOP;
  logicdata[925] = CHANPIN;
  INLINE_NOP;
  logicdata[926] = CHANPIN;
  INLINE_NOP;
  logicdata[927] = CHANPIN;
  INLINE_NOP;
  logicdata[928] = CHANPIN;
  INLINE_NOP;
  logicdata[929] = CHANPIN;
  INLINE_NOP;
  logicdata[930] = CHANPIN;
  INLINE_NOP;
  logicdata[931] = CHANPIN;
  INLINE_NOP;
  logicdata[932] = CHANPIN;
  INLINE_NOP;
  logicdata[933] = CHANPIN;
  INLINE_NOP;
  logicdata[934] = CHANPIN;
  INLINE_NOP;
  logicdata[935] = CHANPIN;
  INLINE_NOP;
  logicdata[936] = CHANPIN;
  INLINE_NOP;
  logicdata[937] = CHANPIN;
  INLINE_NOP;
  logicdata[938] = CHANPIN;
  INLINE_NOP;
  logicdata[939] = CHANPIN;
  INLINE_NOP;
  logicdata[940] = CHANPIN;
  INLINE_NOP;
  logicdata[941] = CHANPIN;
  INLINE_NOP;
  logicdata[942] = CHANPIN;
  INLINE_NOP;
  logicdata[943] = CHANPIN;
  INLINE_NOP;
  logicdata[944] = CHANPIN;
  INLINE_NOP;
  logicdata[945] = CHANPIN;
  INLINE_NOP;
  logicdata[946] = CHANPIN;
  INLINE_NOP;
  logicdata[947] = CHANPIN;
  INLINE_NOP;
  logicdata[948] = CHANPIN;
  INLINE_NOP;
  logicdata[949] = CHANPIN;
  INLINE_NOP;
  logicdata[950] = CHANPIN;
  INLINE_NOP;
  logicdata[951] = CHANPIN;
  INLINE_NOP;
  logicdata[952] = CHANPIN;
  INLINE_NOP;
  logicdata[953] = CHANPIN;
  INLINE_NOP;
  logicdata[954] = CHANPIN;
  INLINE_NOP;
  logicdata[955] = CHANPIN;
  INLINE_NOP;
  logicdata[956] = CHANPIN;
  INLINE_NOP;
  logicdata[957] = CHANPIN;
  INLINE_NOP;
  logicdata[958] = CHANPIN;
  INLINE_NOP;
  logicdata[959] = CHANPIN;
  INLINE_NOP;
  logicdata[960] = CHANPIN;
  INLINE_NOP;
  logicdata[961] = CHANPIN;
  INLINE_NOP;
  logicdata[962] = CHANPIN;
  INLINE_NOP;
  logicdata[963] = CHANPIN;
  INLINE_NOP;
  logicdata[964] = CHANPIN;
  INLINE_NOP;
  logicdata[965] = CHANPIN;
  INLINE_NOP;
  logicdata[966] = CHANPIN;
  INLINE_NOP;
  logicdata[967] = CHANPIN;
  INLINE_NOP;
  logicdata[968] = CHANPIN;
  INLINE_NOP;
  logicdata[969] = CHANPIN;
  INLINE_NOP;
  logicdata[970] = CHANPIN;
  INLINE_NOP;
  logicdata[971] = CHANPIN;
  INLINE_NOP;
  logicdata[972] = CHANPIN;
  INLINE_NOP;
  logicdata[973] = CHANPIN;
  INLINE_NOP;
  logicdata[974] = CHANPIN;
  INLINE_NOP;
  logicdata[975] = CHANPIN;
  INLINE_NOP;
  logicdata[976] = CHANPIN;
  INLINE_NOP;
  logicdata[977] = CHANPIN;
  INLINE_NOP;
  logicdata[978] = CHANPIN;
  INLINE_NOP;
  logicdata[979] = CHANPIN;
  INLINE_NOP;
  logicdata[980] = CHANPIN;
  INLINE_NOP;
  logicdata[981] = CHANPIN;
  INLINE_NOP;
  logicdata[982] = CHANPIN;
  INLINE_NOP;
  logicdata[983] = CHANPIN;
  INLINE_NOP;
  logicdata[984] = CHANPIN;
  INLINE_NOP;
  logicdata[985] = CHANPIN;
  INLINE_NOP;
  logicdata[986] = CHANPIN;
  INLINE_NOP;
  logicdata[987] = CHANPIN;
  INLINE_NOP;
  logicdata[988] = CHANPIN;
  INLINE_NOP;
  logicdata[989] = CHANPIN;
  INLINE_NOP;
  logicdata[990] = CHANPIN;
  INLINE_NOP;
  logicdata[991] = CHANPIN;
  INLINE_NOP;
  logicdata[992] = CHANPIN;
  INLINE_NOP;
  logicdata[993] = CHANPIN;
  INLINE_NOP;
  logicdata[994] = CHANPIN;
  INLINE_NOP;
  logicdata[995] = CHANPIN;
  INLINE_NOP;
  logicdata[996] = CHANPIN;
  INLINE_NOP;
  logicdata[997] = CHANPIN;
  INLINE_NOP;
  logicdata[998] = CHANPIN;
  INLINE_NOP;
  logicdata[999] = CHANPIN;
  INLINE_NOP;
  logicdata[1000] = CHANPIN;
  INLINE_NOP;
  logicdata[1001] = CHANPIN;
  INLINE_NOP;
  logicdata[1002] = CHANPIN;
  INLINE_NOP;
  logicdata[1003] = CHANPIN;
  INLINE_NOP;
  logicdata[1004] = CHANPIN;
  INLINE_NOP;
  logicdata[1005] = CHANPIN;
  INLINE_NOP;
  logicdata[1006] = CHANPIN;
  INLINE_NOP;
  logicdata[1007] = CHANPIN;
  INLINE_NOP;
  logicdata[1008] = CHANPIN;
  INLINE_NOP;
  logicdata[1009] = CHANPIN;
  INLINE_NOP;
  logicdata[1010] = CHANPIN;
  INLINE_NOP;
  logicdata[1011] = CHANPIN;
  INLINE_NOP;
  logicdata[1012] = CHANPIN;
  INLINE_NOP;
  logicdata[1013] = CHANPIN;
  INLINE_NOP;
  logicdata[1014] = CHANPIN;
  INLINE_NOP;
  logicdata[1015] = CHANPIN;
  INLINE_NOP;
  logicdata[1016] = CHANPIN;
  INLINE_NOP;
  logicdata[1017] = CHANPIN;
  INLINE_NOP;
  logicdata[1018] = CHANPIN;
  INLINE_NOP;
  logicdata[1019] = CHANPIN;
  INLINE_NOP;
  logicdata[1020] = CHANPIN;
  INLINE_NOP;
  logicdata[1021] = CHANPIN;
  INLINE_NOP;
  logicdata[1022] = CHANPIN;
  INLINE_NOP;
  logicdata[1023] = CHANPIN;
  INLINE_NOP;
#endif


  DEBUG_OFF; /* debug timing measurement */

  /* re-enable interrupts now that we're done sampling. */
  sei();

  /*
   * dump the samples back to the SUMP client.  nothing special
   * is done for any triggers, this is effectively the 0/100 buffer split.
   */
  for (i = 0 ; i < readCount; i++) {
#ifdef USE_PORTD
    Serial.write(logicdata[i] >> 2);
#else
    Serial.write(logicdata[i]);
#endif
  }
}

void captureInline4mhz() {
  unsigned int i;

  /*
   * basic trigger, wait until all trigger conditions are met on port.
   * this needs further testing, but basic tests work as expected.
   */
  if (trigger) {
    while ((trigger_values ^ CHANPIN) & trigger);
  }

  /*
   * disable interrupts during capture to maintain precision.
   * we cannot afford any timing interference so we absolutely
   * cannot have any interrupts firing.
   */
  cli();

  /*
   * toggle pin a few times to activate trigger for debugging.
   * this is used during development to measure the sample intervals.
   * it is best to just leave the toggling in place so we don't alter
   * any timing unexpectedly.
   */
  DEBUG_ENABLE;
  DEBUG_ON;
  delayMicroseconds(1);
  DEBUG_OFF;
  delayMicroseconds(1);
  DEBUG_ON;
  delayMicroseconds(1);
  DEBUG_OFF;
  delayMicroseconds(1);

  DEBUG_ON; /* debug timing measurement */

  /*
   * Unroll loop to maximize capture speed.
   * Pad with 1 NOP (1 cycle) to make this run at 4MHz.
   *
   *
   */

#undef INLINE_NOP
#define INLINE_NOP		__asm__("nop\n\t");

  logicdata[0] = CHANPIN;
  INLINE_NOP;
  logicdata[1] = CHANPIN;
  INLINE_NOP;
  logicdata[2] = CHANPIN;
  INLINE_NOP;
  logicdata[3] = CHANPIN;
  INLINE_NOP;
  logicdata[4] = CHANPIN;
  INLINE_NOP;
  logicdata[5] = CHANPIN;
  INLINE_NOP;
  logicdata[6] = CHANPIN;
  INLINE_NOP;
  logicdata[7] = CHANPIN;
  INLINE_NOP;
  logicdata[8] = CHANPIN;
  INLINE_NOP;
  logicdata[9] = CHANPIN;
  INLINE_NOP;
  logicdata[10] = CHANPIN;
  INLINE_NOP;
  logicdata[11] = CHANPIN;
  INLINE_NOP;
  logicdata[12] = CHANPIN;
  INLINE_NOP;
  logicdata[13] = CHANPIN;
  INLINE_NOP;
  logicdata[14] = CHANPIN;
  INLINE_NOP;
  logicdata[15] = CHANPIN;
  INLINE_NOP;
  logicdata[16] = CHANPIN;
  INLINE_NOP;
  logicdata[17] = CHANPIN;
  INLINE_NOP;
  logicdata[18] = CHANPIN;
  INLINE_NOP;
  logicdata[19] = CHANPIN;
  INLINE_NOP;
  logicdata[20] = CHANPIN;
  INLINE_NOP;
  logicdata[21] = CHANPIN;
  INLINE_NOP;
  logicdata[22] = CHANPIN;
  INLINE_NOP;
  logicdata[23] = CHANPIN;
  INLINE_NOP;
  logicdata[24] = CHANPIN;
  INLINE_NOP;
  logicdata[25] = CHANPIN;
  INLINE_NOP;
  logicdata[26] = CHANPIN;
  INLINE_NOP;
  logicdata[27] = CHANPIN;
  INLINE_NOP;
  logicdata[28] = CHANPIN;
  INLINE_NOP;
  logicdata[29] = CHANPIN;
  INLINE_NOP;
  logicdata[30] = CHANPIN;
  INLINE_NOP;
  logicdata[31] = CHANPIN;
  INLINE_NOP;
  logicdata[32] = CHANPIN;
  INLINE_NOP;
  logicdata[33] = CHANPIN;
  INLINE_NOP;
  logicdata[34] = CHANPIN;
  INLINE_NOP;
  logicdata[35] = CHANPIN;
  INLINE_NOP;
  logicdata[36] = CHANPIN;
  INLINE_NOP;
  logicdata[37] = CHANPIN;
  INLINE_NOP;
  logicdata[38] = CHANPIN;
  INLINE_NOP;
  logicdata[39] = CHANPIN;
  INLINE_NOP;
  logicdata[40] = CHANPIN;
  INLINE_NOP;
  logicdata[41] = CHANPIN;
  INLINE_NOP;
  logicdata[42] = CHANPIN;
  INLINE_NOP;
  logicdata[43] = CHANPIN;
  INLINE_NOP;
  logicdata[44] = CHANPIN;
  INLINE_NOP;
  logicdata[45] = CHANPIN;
  INLINE_NOP;
  logicdata[46] = CHANPIN;
  INLINE_NOP;
  logicdata[47] = CHANPIN;
  INLINE_NOP;
  logicdata[48] = CHANPIN;
  INLINE_NOP;
  logicdata[49] = CHANPIN;
  INLINE_NOP;
  logicdata[50] = CHANPIN;
  INLINE_NOP;
  logicdata[51] = CHANPIN;
  INLINE_NOP;
  logicdata[52] = CHANPIN;
  INLINE_NOP;
  logicdata[53] = CHANPIN;
  INLINE_NOP;
  logicdata[54] = CHANPIN;
  INLINE_NOP;
  logicdata[55] = CHANPIN;
  INLINE_NOP;
  logicdata[56] = CHANPIN;
  INLINE_NOP;
  logicdata[57] = CHANPIN;
  INLINE_NOP;
  logicdata[58] = CHANPIN;
  INLINE_NOP;
  logicdata[59] = CHANPIN;
  INLINE_NOP;
  logicdata[60] = CHANPIN;
  INLINE_NOP;
  logicdata[61] = CHANPIN;
  INLINE_NOP;
  logicdata[62] = CHANPIN;
  INLINE_NOP;
  logicdata[63] = CHANPIN;
  INLINE_NOP;
  logicdata[64] = CHANPIN;
  INLINE_NOP;
  logicdata[65] = CHANPIN;
  INLINE_NOP;
  logicdata[66] = CHANPIN;
  INLINE_NOP;
  logicdata[67] = CHANPIN;
  INLINE_NOP;
  logicdata[68] = CHANPIN;
  INLINE_NOP;
  logicdata[69] = CHANPIN;
  INLINE_NOP;
  logicdata[70] = CHANPIN;
  INLINE_NOP;
  logicdata[71] = CHANPIN;
  INLINE_NOP;
  logicdata[72] = CHANPIN;
  INLINE_NOP;
  logicdata[73] = CHANPIN;
  INLINE_NOP;
  logicdata[74] = CHANPIN;
  INLINE_NOP;
  logicdata[75] = CHANPIN;
  INLINE_NOP;
  logicdata[76] = CHANPIN;
  INLINE_NOP;
  logicdata[77] = CHANPIN;
  INLINE_NOP;
  logicdata[78] = CHANPIN;
  INLINE_NOP;
  logicdata[79] = CHANPIN;
  INLINE_NOP;
  logicdata[80] = CHANPIN;
  INLINE_NOP;
  logicdata[81] = CHANPIN;
  INLINE_NOP;
  logicdata[82] = CHANPIN;
  INLINE_NOP;
  logicdata[83] = CHANPIN;
  INLINE_NOP;
  logicdata[84] = CHANPIN;
  INLINE_NOP;
  logicdata[85] = CHANPIN;
  INLINE_NOP;
  logicdata[86] = CHANPIN;
  INLINE_NOP;
  logicdata[87] = CHANPIN;
  INLINE_NOP;
  logicdata[88] = CHANPIN;
  INLINE_NOP;
  logicdata[89] = CHANPIN;
  INLINE_NOP;
  logicdata[90] = CHANPIN;
  INLINE_NOP;
  logicdata[91] = CHANPIN;
  INLINE_NOP;
  logicdata[92] = CHANPIN;
  INLINE_NOP;
  logicdata[93] = CHANPIN;
  INLINE_NOP;
  logicdata[94] = CHANPIN;
  INLINE_NOP;
  logicdata[95] = CHANPIN;
  INLINE_NOP;
  logicdata[96] = CHANPIN;
  INLINE_NOP;
  logicdata[97] = CHANPIN;
  INLINE_NOP;
  logicdata[98] = CHANPIN;
  INLINE_NOP;
  logicdata[99] = CHANPIN;
  INLINE_NOP;
  logicdata[100] = CHANPIN;
  INLINE_NOP;
  logicdata[101] = CHANPIN;
  INLINE_NOP;
  logicdata[102] = CHANPIN;
  INLINE_NOP;
  logicdata[103] = CHANPIN;
  INLINE_NOP;
  logicdata[104] = CHANPIN;
  INLINE_NOP;
  logicdata[105] = CHANPIN;
  INLINE_NOP;
  logicdata[106] = CHANPIN;
  INLINE_NOP;
  logicdata[107] = CHANPIN;
  INLINE_NOP;
  logicdata[108] = CHANPIN;
  INLINE_NOP;
  logicdata[109] = CHANPIN;
  INLINE_NOP;
  logicdata[110] = CHANPIN;
  INLINE_NOP;
  logicdata[111] = CHANPIN;
  INLINE_NOP;
  logicdata[112] = CHANPIN;
  INLINE_NOP;
  logicdata[113] = CHANPIN;
  INLINE_NOP;
  logicdata[114] = CHANPIN;
  INLINE_NOP;
  logicdata[115] = CHANPIN;
  INLINE_NOP;
  logicdata[116] = CHANPIN;
  INLINE_NOP;
  logicdata[117] = CHANPIN;
  INLINE_NOP;
  logicdata[118] = CHANPIN;
  INLINE_NOP;
  logicdata[119] = CHANPIN;
  INLINE_NOP;
  logicdata[120] = CHANPIN;
  INLINE_NOP;
  logicdata[121] = CHANPIN;
  INLINE_NOP;
  logicdata[122] = CHANPIN;
  INLINE_NOP;
  logicdata[123] = CHANPIN;
  INLINE_NOP;
  logicdata[124] = CHANPIN;
  INLINE_NOP;
  logicdata[125] = CHANPIN;
  INLINE_NOP;
  logicdata[126] = CHANPIN;
  INLINE_NOP;
  logicdata[127] = CHANPIN;
  INLINE_NOP;
  logicdata[128] = CHANPIN;
  INLINE_NOP;
  logicdata[129] = CHANPIN;
  INLINE_NOP;
  logicdata[130] = CHANPIN;
  INLINE_NOP;
  logicdata[131] = CHANPIN;
  INLINE_NOP;
  logicdata[132] = CHANPIN;
  INLINE_NOP;
  logicdata[133] = CHANPIN;
  INLINE_NOP;
  logicdata[134] = CHANPIN;
  INLINE_NOP;
  logicdata[135] = CHANPIN;
  INLINE_NOP;
  logicdata[136] = CHANPIN;
  INLINE_NOP;
  logicdata[137] = CHANPIN;
  INLINE_NOP;
  logicdata[138] = CHANPIN;
  INLINE_NOP;
  logicdata[139] = CHANPIN;
  INLINE_NOP;
  logicdata[140] = CHANPIN;
  INLINE_NOP;
  logicdata[141] = CHANPIN;
  INLINE_NOP;
  logicdata[142] = CHANPIN;
  INLINE_NOP;
  logicdata[143] = CHANPIN;
  INLINE_NOP;
  logicdata[144] = CHANPIN;
  INLINE_NOP;
  logicdata[145] = CHANPIN;
  INLINE_NOP;
  logicdata[146] = CHANPIN;
  INLINE_NOP;
  logicdata[147] = CHANPIN;
  INLINE_NOP;
  logicdata[148] = CHANPIN;
  INLINE_NOP;
  logicdata[149] = CHANPIN;
  INLINE_NOP;
  logicdata[150] = CHANPIN;
  INLINE_NOP;
  logicdata[151] = CHANPIN;
  INLINE_NOP;
  logicdata[152] = CHANPIN;
  INLINE_NOP;
  logicdata[153] = CHANPIN;
  INLINE_NOP;
  logicdata[154] = CHANPIN;
  INLINE_NOP;
  logicdata[155] = CHANPIN;
  INLINE_NOP;
  logicdata[156] = CHANPIN;
  INLINE_NOP;
  logicdata[157] = CHANPIN;
  INLINE_NOP;
  logicdata[158] = CHANPIN;
  INLINE_NOP;
  logicdata[159] = CHANPIN;
  INLINE_NOP;
  logicdata[160] = CHANPIN;
  INLINE_NOP;
  logicdata[161] = CHANPIN;
  INLINE_NOP;
  logicdata[162] = CHANPIN;
  INLINE_NOP;
  logicdata[163] = CHANPIN;
  INLINE_NOP;
  logicdata[164] = CHANPIN;
  INLINE_NOP;
  logicdata[165] = CHANPIN;
  INLINE_NOP;
  logicdata[166] = CHANPIN;
  INLINE_NOP;
  logicdata[167] = CHANPIN;
  INLINE_NOP;
  logicdata[168] = CHANPIN;
  INLINE_NOP;
  logicdata[169] = CHANPIN;
  INLINE_NOP;
  logicdata[170] = CHANPIN;
  INLINE_NOP;
  logicdata[171] = CHANPIN;
  INLINE_NOP;
  logicdata[172] = CHANPIN;
  INLINE_NOP;
  logicdata[173] = CHANPIN;
  INLINE_NOP;
  logicdata[174] = CHANPIN;
  INLINE_NOP;
  logicdata[175] = CHANPIN;
  INLINE_NOP;
  logicdata[176] = CHANPIN;
  INLINE_NOP;
  logicdata[177] = CHANPIN;
  INLINE_NOP;
  logicdata[178] = CHANPIN;
  INLINE_NOP;
  logicdata[179] = CHANPIN;
  INLINE_NOP;
  logicdata[180] = CHANPIN;
  INLINE_NOP;
  logicdata[181] = CHANPIN;
  INLINE_NOP;
  logicdata[182] = CHANPIN;
  INLINE_NOP;
  logicdata[183] = CHANPIN;
  INLINE_NOP;
  logicdata[184] = CHANPIN;
  INLINE_NOP;
  logicdata[185] = CHANPIN;
  INLINE_NOP;
  logicdata[186] = CHANPIN;
  INLINE_NOP;
  logicdata[187] = CHANPIN;
  INLINE_NOP;
  logicdata[188] = CHANPIN;
  INLINE_NOP;
  logicdata[189] = CHANPIN;
  INLINE_NOP;
  logicdata[190] = CHANPIN;
  INLINE_NOP;
  logicdata[191] = CHANPIN;
  INLINE_NOP;
  logicdata[192] = CHANPIN;
  INLINE_NOP;
  logicdata[193] = CHANPIN;
  INLINE_NOP;
  logicdata[194] = CHANPIN;
  INLINE_NOP;
  logicdata[195] = CHANPIN;
  INLINE_NOP;
  logicdata[196] = CHANPIN;
  INLINE_NOP;
  logicdata[197] = CHANPIN;
  INLINE_NOP;
  logicdata[198] = CHANPIN;
  INLINE_NOP;
  logicdata[199] = CHANPIN;
  INLINE_NOP;
  logicdata[200] = CHANPIN;
  INLINE_NOP;
  logicdata[201] = CHANPIN;
  INLINE_NOP;
  logicdata[202] = CHANPIN;
  INLINE_NOP;
  logicdata[203] = CHANPIN;
  INLINE_NOP;
  logicdata[204] = CHANPIN;
  INLINE_NOP;
  logicdata[205] = CHANPIN;
  INLINE_NOP;
  logicdata[206] = CHANPIN;
  INLINE_NOP;
  logicdata[207] = CHANPIN;
  INLINE_NOP;
  logicdata[208] = CHANPIN;
  INLINE_NOP;
  logicdata[209] = CHANPIN;
  INLINE_NOP;
  logicdata[210] = CHANPIN;
  INLINE_NOP;
  logicdata[211] = CHANPIN;
  INLINE_NOP;
  logicdata[212] = CHANPIN;
  INLINE_NOP;
  logicdata[213] = CHANPIN;
  INLINE_NOP;
  logicdata[214] = CHANPIN;
  INLINE_NOP;
  logicdata[215] = CHANPIN;
  INLINE_NOP;
  logicdata[216] = CHANPIN;
  INLINE_NOP;
  logicdata[217] = CHANPIN;
  INLINE_NOP;
  logicdata[218] = CHANPIN;
  INLINE_NOP;
  logicdata[219] = CHANPIN;
  INLINE_NOP;
  logicdata[220] = CHANPIN;
  INLINE_NOP;
  logicdata[221] = CHANPIN;
  INLINE_NOP;
  logicdata[222] = CHANPIN;
  INLINE_NOP;
  logicdata[223] = CHANPIN;
  INLINE_NOP;
  logicdata[224] = CHANPIN;
  INLINE_NOP;
  logicdata[225] = CHANPIN;
  INLINE_NOP;
  logicdata[226] = CHANPIN;
  INLINE_NOP;
  logicdata[227] = CHANPIN;
  INLINE_NOP;
  logicdata[228] = CHANPIN;
  INLINE_NOP;
  logicdata[229] = CHANPIN;
  INLINE_NOP;
  logicdata[230] = CHANPIN;
  INLINE_NOP;
  logicdata[231] = CHANPIN;
  INLINE_NOP;
  logicdata[232] = CHANPIN;
  INLINE_NOP;
  logicdata[233] = CHANPIN;
  INLINE_NOP;
  logicdata[234] = CHANPIN;
  INLINE_NOP;
  logicdata[235] = CHANPIN;
  INLINE_NOP;
  logicdata[236] = CHANPIN;
  INLINE_NOP;
  logicdata[237] = CHANPIN;
  INLINE_NOP;
  logicdata[238] = CHANPIN;
  INLINE_NOP;
  logicdata[239] = CHANPIN;
  INLINE_NOP;
  logicdata[240] = CHANPIN;
  INLINE_NOP;
  logicdata[241] = CHANPIN;
  INLINE_NOP;
  logicdata[242] = CHANPIN;
  INLINE_NOP;
  logicdata[243] = CHANPIN;
  INLINE_NOP;
  logicdata[244] = CHANPIN;
  INLINE_NOP;
  logicdata[245] = CHANPIN;
  INLINE_NOP;
  logicdata[246] = CHANPIN;
  INLINE_NOP;
  logicdata[247] = CHANPIN;
  INLINE_NOP;
  logicdata[248] = CHANPIN;
  INLINE_NOP;
  logicdata[249] = CHANPIN;
  INLINE_NOP;
  logicdata[250] = CHANPIN;
  INLINE_NOP;
  logicdata[251] = CHANPIN;
  INLINE_NOP;
  logicdata[252] = CHANPIN;
  INLINE_NOP;
  logicdata[253] = CHANPIN;
  INLINE_NOP;
  logicdata[254] = CHANPIN;
  INLINE_NOP;
  logicdata[255] = CHANPIN;
  INLINE_NOP;
  logicdata[256] = CHANPIN;
  INLINE_NOP;
  logicdata[257] = CHANPIN;
  INLINE_NOP;
  logicdata[258] = CHANPIN;
  INLINE_NOP;
  logicdata[259] = CHANPIN;
  INLINE_NOP;
  logicdata[260] = CHANPIN;
  INLINE_NOP;
  logicdata[261] = CHANPIN;
  INLINE_NOP;
  logicdata[262] = CHANPIN;
  INLINE_NOP;
  logicdata[263] = CHANPIN;
  INLINE_NOP;
  logicdata[264] = CHANPIN;
  INLINE_NOP;
  logicdata[265] = CHANPIN;
  INLINE_NOP;
  logicdata[266] = CHANPIN;
  INLINE_NOP;
  logicdata[267] = CHANPIN;
  INLINE_NOP;
  logicdata[268] = CHANPIN;
  INLINE_NOP;
  logicdata[269] = CHANPIN;
  INLINE_NOP;
  logicdata[270] = CHANPIN;
  INLINE_NOP;
  logicdata[271] = CHANPIN;
  INLINE_NOP;
  logicdata[272] = CHANPIN;
  INLINE_NOP;
  logicdata[273] = CHANPIN;
  INLINE_NOP;
  logicdata[274] = CHANPIN;
  INLINE_NOP;
  logicdata[275] = CHANPIN;
  INLINE_NOP;
  logicdata[276] = CHANPIN;
  INLINE_NOP;
  logicdata[277] = CHANPIN;
  INLINE_NOP;
  logicdata[278] = CHANPIN;
  INLINE_NOP;
  logicdata[279] = CHANPIN;
  INLINE_NOP;
  logicdata[280] = CHANPIN;
  INLINE_NOP;
  logicdata[281] = CHANPIN;
  INLINE_NOP;
  logicdata[282] = CHANPIN;
  INLINE_NOP;
  logicdata[283] = CHANPIN;
  INLINE_NOP;
  logicdata[284] = CHANPIN;
  INLINE_NOP;
  logicdata[285] = CHANPIN;
  INLINE_NOP;
  logicdata[286] = CHANPIN;
  INLINE_NOP;
  logicdata[287] = CHANPIN;
  INLINE_NOP;
  logicdata[288] = CHANPIN;
  INLINE_NOP;
  logicdata[289] = CHANPIN;
  INLINE_NOP;
  logicdata[290] = CHANPIN;
  INLINE_NOP;
  logicdata[291] = CHANPIN;
  INLINE_NOP;
  logicdata[292] = CHANPIN;
  INLINE_NOP;
  logicdata[293] = CHANPIN;
  INLINE_NOP;
  logicdata[294] = CHANPIN;
  INLINE_NOP;
  logicdata[295] = CHANPIN;
  INLINE_NOP;
  logicdata[296] = CHANPIN;
  INLINE_NOP;
  logicdata[297] = CHANPIN;
  INLINE_NOP;
  logicdata[298] = CHANPIN;
  INLINE_NOP;
  logicdata[299] = CHANPIN;
  INLINE_NOP;
  logicdata[300] = CHANPIN;
  INLINE_NOP;
  logicdata[301] = CHANPIN;
  INLINE_NOP;
  logicdata[302] = CHANPIN;
  INLINE_NOP;
  logicdata[303] = CHANPIN;
  INLINE_NOP;
  logicdata[304] = CHANPIN;
  INLINE_NOP;
  logicdata[305] = CHANPIN;
  INLINE_NOP;
  logicdata[306] = CHANPIN;
  INLINE_NOP;
  logicdata[307] = CHANPIN;
  INLINE_NOP;
  logicdata[308] = CHANPIN;
  INLINE_NOP;
  logicdata[309] = CHANPIN;
  INLINE_NOP;
  logicdata[310] = CHANPIN;
  INLINE_NOP;
  logicdata[311] = CHANPIN;
  INLINE_NOP;
  logicdata[312] = CHANPIN;
  INLINE_NOP;
  logicdata[313] = CHANPIN;
  INLINE_NOP;
  logicdata[314] = CHANPIN;
  INLINE_NOP;
  logicdata[315] = CHANPIN;
  INLINE_NOP;
  logicdata[316] = CHANPIN;
  INLINE_NOP;
  logicdata[317] = CHANPIN;
  INLINE_NOP;
  logicdata[318] = CHANPIN;
  INLINE_NOP;
  logicdata[319] = CHANPIN;
  INLINE_NOP;
  logicdata[320] = CHANPIN;
  INLINE_NOP;
  logicdata[321] = CHANPIN;
  INLINE_NOP;
  logicdata[322] = CHANPIN;
  INLINE_NOP;
  logicdata[323] = CHANPIN;
  INLINE_NOP;
  logicdata[324] = CHANPIN;
  INLINE_NOP;
  logicdata[325] = CHANPIN;
  INLINE_NOP;
  logicdata[326] = CHANPIN;
  INLINE_NOP;
  logicdata[327] = CHANPIN;
  INLINE_NOP;
  logicdata[328] = CHANPIN;
  INLINE_NOP;
  logicdata[329] = CHANPIN;
  INLINE_NOP;
  logicdata[330] = CHANPIN;
  INLINE_NOP;
  logicdata[331] = CHANPIN;
  INLINE_NOP;
  logicdata[332] = CHANPIN;
  INLINE_NOP;
  logicdata[333] = CHANPIN;
  INLINE_NOP;
  logicdata[334] = CHANPIN;
  INLINE_NOP;
  logicdata[335] = CHANPIN;
  INLINE_NOP;
  logicdata[336] = CHANPIN;
  INLINE_NOP;
  logicdata[337] = CHANPIN;
  INLINE_NOP;
  logicdata[338] = CHANPIN;
  INLINE_NOP;
  logicdata[339] = CHANPIN;
  INLINE_NOP;
  logicdata[340] = CHANPIN;
  INLINE_NOP;
  logicdata[341] = CHANPIN;
  INLINE_NOP;
  logicdata[342] = CHANPIN;
  INLINE_NOP;
  logicdata[343] = CHANPIN;
  INLINE_NOP;
  logicdata[344] = CHANPIN;
  INLINE_NOP;
  logicdata[345] = CHANPIN;
  INLINE_NOP;
  logicdata[346] = CHANPIN;
  INLINE_NOP;
  logicdata[347] = CHANPIN;
  INLINE_NOP;
  logicdata[348] = CHANPIN;
  INLINE_NOP;
  logicdata[349] = CHANPIN;
  INLINE_NOP;
  logicdata[350] = CHANPIN;
  INLINE_NOP;
  logicdata[351] = CHANPIN;
  INLINE_NOP;
  logicdata[352] = CHANPIN;
  INLINE_NOP;
  logicdata[353] = CHANPIN;
  INLINE_NOP;
  logicdata[354] = CHANPIN;
  INLINE_NOP;
  logicdata[355] = CHANPIN;
  INLINE_NOP;
  logicdata[356] = CHANPIN;
  INLINE_NOP;
  logicdata[357] = CHANPIN;
  INLINE_NOP;
  logicdata[358] = CHANPIN;
  INLINE_NOP;
  logicdata[359] = CHANPIN;
  INLINE_NOP;
  logicdata[360] = CHANPIN;
  INLINE_NOP;
  logicdata[361] = CHANPIN;
  INLINE_NOP;
  logicdata[362] = CHANPIN;
  INLINE_NOP;
  logicdata[363] = CHANPIN;
  INLINE_NOP;
  logicdata[364] = CHANPIN;
  INLINE_NOP;
  logicdata[365] = CHANPIN;
  INLINE_NOP;
  logicdata[366] = CHANPIN;
  INLINE_NOP;
  logicdata[367] = CHANPIN;
  INLINE_NOP;
  logicdata[368] = CHANPIN;
  INLINE_NOP;
  logicdata[369] = CHANPIN;
  INLINE_NOP;
  logicdata[370] = CHANPIN;
  INLINE_NOP;
  logicdata[371] = CHANPIN;
  INLINE_NOP;
  logicdata[372] = CHANPIN;
  INLINE_NOP;
  logicdata[373] = CHANPIN;
  INLINE_NOP;
  logicdata[374] = CHANPIN;
  INLINE_NOP;
  logicdata[375] = CHANPIN;
  INLINE_NOP;
  logicdata[376] = CHANPIN;
  INLINE_NOP;
  logicdata[377] = CHANPIN;
  INLINE_NOP;
  logicdata[378] = CHANPIN;
  INLINE_NOP;
  logicdata[379] = CHANPIN;
  INLINE_NOP;
  logicdata[380] = CHANPIN;
  INLINE_NOP;
  logicdata[381] = CHANPIN;
  INLINE_NOP;
  logicdata[382] = CHANPIN;
  INLINE_NOP;
  logicdata[383] = CHANPIN;
  INLINE_NOP;
  logicdata[384] = CHANPIN;
  INLINE_NOP;
  logicdata[385] = CHANPIN;
  INLINE_NOP;
  logicdata[386] = CHANPIN;
  INLINE_NOP;
  logicdata[387] = CHANPIN;
  INLINE_NOP;
  logicdata[388] = CHANPIN;
  INLINE_NOP;
  logicdata[389] = CHANPIN;
  INLINE_NOP;
  logicdata[390] = CHANPIN;
  INLINE_NOP;
  logicdata[391] = CHANPIN;
  INLINE_NOP;
  logicdata[392] = CHANPIN;
  INLINE_NOP;
  logicdata[393] = CHANPIN;
  INLINE_NOP;
  logicdata[394] = CHANPIN;
  INLINE_NOP;
  logicdata[395] = CHANPIN;
  INLINE_NOP;
  logicdata[396] = CHANPIN;
  INLINE_NOP;
  logicdata[397] = CHANPIN;
  INLINE_NOP;
  logicdata[398] = CHANPIN;
  INLINE_NOP;
  logicdata[399] = CHANPIN;
  INLINE_NOP;
  logicdata[400] = CHANPIN;
  INLINE_NOP;
  logicdata[401] = CHANPIN;
  INLINE_NOP;
  logicdata[402] = CHANPIN;
  INLINE_NOP;
  logicdata[403] = CHANPIN;
  INLINE_NOP;
  logicdata[404] = CHANPIN;
  INLINE_NOP;
  logicdata[405] = CHANPIN;
  INLINE_NOP;
  logicdata[406] = CHANPIN;
  INLINE_NOP;
  logicdata[407] = CHANPIN;
  INLINE_NOP;
  logicdata[408] = CHANPIN;
  INLINE_NOP;
  logicdata[409] = CHANPIN;
  INLINE_NOP;
  logicdata[410] = CHANPIN;
  INLINE_NOP;
  logicdata[411] = CHANPIN;
  INLINE_NOP;
  logicdata[412] = CHANPIN;
  INLINE_NOP;
  logicdata[413] = CHANPIN;
  INLINE_NOP;
  logicdata[414] = CHANPIN;
  INLINE_NOP;
  logicdata[415] = CHANPIN;
  INLINE_NOP;
  logicdata[416] = CHANPIN;
  INLINE_NOP;
  logicdata[417] = CHANPIN;
  INLINE_NOP;
  logicdata[418] = CHANPIN;
  INLINE_NOP;
  logicdata[419] = CHANPIN;
  INLINE_NOP;
  logicdata[420] = CHANPIN;
  INLINE_NOP;
  logicdata[421] = CHANPIN;
  INLINE_NOP;
  logicdata[422] = CHANPIN;
  INLINE_NOP;
  logicdata[423] = CHANPIN;
  INLINE_NOP;
  logicdata[424] = CHANPIN;
  INLINE_NOP;
  logicdata[425] = CHANPIN;
  INLINE_NOP;
  logicdata[426] = CHANPIN;
  INLINE_NOP;
  logicdata[427] = CHANPIN;
  INLINE_NOP;
  logicdata[428] = CHANPIN;
  INLINE_NOP;
  logicdata[429] = CHANPIN;
  INLINE_NOP;
  logicdata[430] = CHANPIN;
  INLINE_NOP;
  logicdata[431] = CHANPIN;
  INLINE_NOP;
  logicdata[432] = CHANPIN;
  INLINE_NOP;
  logicdata[433] = CHANPIN;
  INLINE_NOP;
  logicdata[434] = CHANPIN;
  INLINE_NOP;
  logicdata[435] = CHANPIN;
  INLINE_NOP;
  logicdata[436] = CHANPIN;
  INLINE_NOP;
  logicdata[437] = CHANPIN;
  INLINE_NOP;
  logicdata[438] = CHANPIN;
  INLINE_NOP;
  logicdata[439] = CHANPIN;
  INLINE_NOP;
  logicdata[440] = CHANPIN;
  INLINE_NOP;
  logicdata[441] = CHANPIN;
  INLINE_NOP;
  logicdata[442] = CHANPIN;
  INLINE_NOP;
  logicdata[443] = CHANPIN;
  INLINE_NOP;
  logicdata[444] = CHANPIN;
  INLINE_NOP;
  logicdata[445] = CHANPIN;
  INLINE_NOP;
  logicdata[446] = CHANPIN;
  INLINE_NOP;
  logicdata[447] = CHANPIN;
  INLINE_NOP;
  logicdata[448] = CHANPIN;
  INLINE_NOP;
  logicdata[449] = CHANPIN;
  INLINE_NOP;
  logicdata[450] = CHANPIN;
  INLINE_NOP;
  logicdata[451] = CHANPIN;
  INLINE_NOP;
  logicdata[452] = CHANPIN;
  INLINE_NOP;
  logicdata[453] = CHANPIN;
  INLINE_NOP;
  logicdata[454] = CHANPIN;
  INLINE_NOP;
  logicdata[455] = CHANPIN;
  INLINE_NOP;
  logicdata[456] = CHANPIN;
  INLINE_NOP;
  logicdata[457] = CHANPIN;
  INLINE_NOP;
  logicdata[458] = CHANPIN;
  INLINE_NOP;
  logicdata[459] = CHANPIN;
  INLINE_NOP;
  logicdata[460] = CHANPIN;
  INLINE_NOP;
  logicdata[461] = CHANPIN;
  INLINE_NOP;
  logicdata[462] = CHANPIN;
  INLINE_NOP;
  logicdata[463] = CHANPIN;
  INLINE_NOP;
  logicdata[464] = CHANPIN;
  INLINE_NOP;
  logicdata[465] = CHANPIN;
  INLINE_NOP;
  logicdata[466] = CHANPIN;
  INLINE_NOP;
  logicdata[467] = CHANPIN;
  INLINE_NOP;
  logicdata[468] = CHANPIN;
  INLINE_NOP;
  logicdata[469] = CHANPIN;
  INLINE_NOP;
  logicdata[470] = CHANPIN;
  INLINE_NOP;
  logicdata[471] = CHANPIN;
  INLINE_NOP;
  logicdata[472] = CHANPIN;
  INLINE_NOP;
  logicdata[473] = CHANPIN;
  INLINE_NOP;
  logicdata[474] = CHANPIN;
  INLINE_NOP;
  logicdata[475] = CHANPIN;
  INLINE_NOP;
  logicdata[476] = CHANPIN;
  INLINE_NOP;
  logicdata[477] = CHANPIN;
  INLINE_NOP;
  logicdata[478] = CHANPIN;
  INLINE_NOP;
  logicdata[479] = CHANPIN;
  INLINE_NOP;
  logicdata[480] = CHANPIN;
  INLINE_NOP;
  logicdata[481] = CHANPIN;
  INLINE_NOP;
  logicdata[482] = CHANPIN;
  INLINE_NOP;
  logicdata[483] = CHANPIN;
  INLINE_NOP;
  logicdata[484] = CHANPIN;
  INLINE_NOP;
  logicdata[485] = CHANPIN;
  INLINE_NOP;
  logicdata[486] = CHANPIN;
  INLINE_NOP;
  logicdata[487] = CHANPIN;
  INLINE_NOP;
  logicdata[488] = CHANPIN;
  INLINE_NOP;
  logicdata[489] = CHANPIN;
  INLINE_NOP;
  logicdata[490] = CHANPIN;
  INLINE_NOP;
  logicdata[491] = CHANPIN;
  INLINE_NOP;
  logicdata[492] = CHANPIN;
  INLINE_NOP;
  logicdata[493] = CHANPIN;
  INLINE_NOP;
  logicdata[494] = CHANPIN;
  INLINE_NOP;
  logicdata[495] = CHANPIN;
  INLINE_NOP;
  logicdata[496] = CHANPIN;
  INLINE_NOP;
  logicdata[497] = CHANPIN;
  INLINE_NOP;
  logicdata[498] = CHANPIN;
  INLINE_NOP;
  logicdata[499] = CHANPIN;
  INLINE_NOP;
  logicdata[500] = CHANPIN;
  INLINE_NOP;
  logicdata[501] = CHANPIN;
  INLINE_NOP;
  logicdata[502] = CHANPIN;
  INLINE_NOP;
  logicdata[503] = CHANPIN;
  INLINE_NOP;
  logicdata[504] = CHANPIN;
  INLINE_NOP;
  logicdata[505] = CHANPIN;
  INLINE_NOP;
  logicdata[506] = CHANPIN;
  INLINE_NOP;
  logicdata[507] = CHANPIN;
  INLINE_NOP;
  logicdata[508] = CHANPIN;
  INLINE_NOP;
  logicdata[509] = CHANPIN;
  INLINE_NOP;
  logicdata[510] = CHANPIN;
  INLINE_NOP;
  logicdata[511] = CHANPIN;
  INLINE_NOP;
  logicdata[512] = CHANPIN;
  INLINE_NOP;
  logicdata[513] = CHANPIN;
  INLINE_NOP;
  logicdata[514] = CHANPIN;
  INLINE_NOP;
  logicdata[515] = CHANPIN;
  INLINE_NOP;
  logicdata[516] = CHANPIN;
  INLINE_NOP;
  logicdata[517] = CHANPIN;
  INLINE_NOP;
  logicdata[518] = CHANPIN;
  INLINE_NOP;
  logicdata[519] = CHANPIN;
  INLINE_NOP;
  logicdata[520] = CHANPIN;
  INLINE_NOP;
  logicdata[521] = CHANPIN;
  INLINE_NOP;
  logicdata[522] = CHANPIN;
  INLINE_NOP;
  logicdata[523] = CHANPIN;
  INLINE_NOP;
  logicdata[524] = CHANPIN;
  INLINE_NOP;
  logicdata[525] = CHANPIN;
  INLINE_NOP;
  logicdata[526] = CHANPIN;
  INLINE_NOP;
  logicdata[527] = CHANPIN;
  INLINE_NOP;
  logicdata[528] = CHANPIN;
  INLINE_NOP;
  logicdata[529] = CHANPIN;
  INLINE_NOP;
  logicdata[530] = CHANPIN;
  INLINE_NOP;
  logicdata[531] = CHANPIN;
  INLINE_NOP;
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  logicdata[532] = CHANPIN;
  INLINE_NOP;
  logicdata[533] = CHANPIN;
  INLINE_NOP;
  logicdata[534] = CHANPIN;
  INLINE_NOP;
  logicdata[535] = CHANPIN;
  INLINE_NOP;
  logicdata[536] = CHANPIN;
  INLINE_NOP;
  logicdata[537] = CHANPIN;
  INLINE_NOP;
  logicdata[538] = CHANPIN;
  INLINE_NOP;
  logicdata[539] = CHANPIN;
  INLINE_NOP;
  logicdata[540] = CHANPIN;
  INLINE_NOP;
  logicdata[541] = CHANPIN;
  INLINE_NOP;
  logicdata[542] = CHANPIN;
  INLINE_NOP;
  logicdata[543] = CHANPIN;
  INLINE_NOP;
  logicdata[544] = CHANPIN;
  INLINE_NOP;
  logicdata[545] = CHANPIN;
  INLINE_NOP;
  logicdata[546] = CHANPIN;
  INLINE_NOP;
  logicdata[547] = CHANPIN;
  INLINE_NOP;
  logicdata[548] = CHANPIN;
  INLINE_NOP;
  logicdata[549] = CHANPIN;
  INLINE_NOP;
  logicdata[550] = CHANPIN;
  INLINE_NOP;
  logicdata[551] = CHANPIN;
  INLINE_NOP;
  logicdata[552] = CHANPIN;
  INLINE_NOP;
  logicdata[553] = CHANPIN;
  INLINE_NOP;
  logicdata[554] = CHANPIN;
  INLINE_NOP;
  logicdata[555] = CHANPIN;
  INLINE_NOP;
  logicdata[556] = CHANPIN;
  INLINE_NOP;
  logicdata[557] = CHANPIN;
  INLINE_NOP;
  logicdata[558] = CHANPIN;
  INLINE_NOP;
  logicdata[559] = CHANPIN;
  INLINE_NOP;
  logicdata[560] = CHANPIN;
  INLINE_NOP;
  logicdata[561] = CHANPIN;
  INLINE_NOP;
  logicdata[562] = CHANPIN;
  INLINE_NOP;
  logicdata[563] = CHANPIN;
  INLINE_NOP;
  logicdata[564] = CHANPIN;
  INLINE_NOP;
  logicdata[565] = CHANPIN;
  INLINE_NOP;
  logicdata[566] = CHANPIN;
  INLINE_NOP;
  logicdata[567] = CHANPIN;
  INLINE_NOP;
  logicdata[568] = CHANPIN;
  INLINE_NOP;
  logicdata[569] = CHANPIN;
  INLINE_NOP;
  logicdata[570] = CHANPIN;
  INLINE_NOP;
  logicdata[571] = CHANPIN;
  INLINE_NOP;
  logicdata[572] = CHANPIN;
  INLINE_NOP;
  logicdata[573] = CHANPIN;
  INLINE_NOP;
  logicdata[574] = CHANPIN;
  INLINE_NOP;
  logicdata[575] = CHANPIN;
  INLINE_NOP;
  logicdata[576] = CHANPIN;
  INLINE_NOP;
  logicdata[577] = CHANPIN;
  INLINE_NOP;
  logicdata[578] = CHANPIN;
  INLINE_NOP;
  logicdata[579] = CHANPIN;
  INLINE_NOP;
  logicdata[580] = CHANPIN;
  INLINE_NOP;
  logicdata[581] = CHANPIN;
  INLINE_NOP;
  logicdata[582] = CHANPIN;
  INLINE_NOP;
  logicdata[583] = CHANPIN;
  INLINE_NOP;
  logicdata[584] = CHANPIN;
  INLINE_NOP;
  logicdata[585] = CHANPIN;
  INLINE_NOP;
  logicdata[586] = CHANPIN;
  INLINE_NOP;
  logicdata[587] = CHANPIN;
  INLINE_NOP;
  logicdata[588] = CHANPIN;
  INLINE_NOP;
  logicdata[589] = CHANPIN;
  INLINE_NOP;
  logicdata[590] = CHANPIN;
  INLINE_NOP;
  logicdata[591] = CHANPIN;
  INLINE_NOP;
  logicdata[592] = CHANPIN;
  INLINE_NOP;
  logicdata[593] = CHANPIN;
  INLINE_NOP;
  logicdata[594] = CHANPIN;
  INLINE_NOP;
  logicdata[595] = CHANPIN;
  INLINE_NOP;
  logicdata[596] = CHANPIN;
  INLINE_NOP;
  logicdata[597] = CHANPIN;
  INLINE_NOP;
  logicdata[598] = CHANPIN;
  INLINE_NOP;
  logicdata[599] = CHANPIN;
  INLINE_NOP;
  logicdata[600] = CHANPIN;
  INLINE_NOP;
  logicdata[601] = CHANPIN;
  INLINE_NOP;
  logicdata[602] = CHANPIN;
  INLINE_NOP;
  logicdata[603] = CHANPIN;
  INLINE_NOP;
  logicdata[604] = CHANPIN;
  INLINE_NOP;
  logicdata[605] = CHANPIN;
  INLINE_NOP;
  logicdata[606] = CHANPIN;
  INLINE_NOP;
  logicdata[607] = CHANPIN;
  INLINE_NOP;
  logicdata[608] = CHANPIN;
  INLINE_NOP;
  logicdata[609] = CHANPIN;
  INLINE_NOP;
  logicdata[610] = CHANPIN;
  INLINE_NOP;
  logicdata[611] = CHANPIN;
  INLINE_NOP;
  logicdata[612] = CHANPIN;
  INLINE_NOP;
  logicdata[613] = CHANPIN;
  INLINE_NOP;
  logicdata[614] = CHANPIN;
  INLINE_NOP;
  logicdata[615] = CHANPIN;
  INLINE_NOP;
  logicdata[616] = CHANPIN;
  INLINE_NOP;
  logicdata[617] = CHANPIN;
  INLINE_NOP;
  logicdata[618] = CHANPIN;
  INLINE_NOP;
  logicdata[619] = CHANPIN;
  INLINE_NOP;
  logicdata[620] = CHANPIN;
  INLINE_NOP;
  logicdata[621] = CHANPIN;
  INLINE_NOP;
  logicdata[622] = CHANPIN;
  INLINE_NOP;
  logicdata[623] = CHANPIN;
  INLINE_NOP;
  logicdata[624] = CHANPIN;
  INLINE_NOP;
  logicdata[625] = CHANPIN;
  INLINE_NOP;
  logicdata[626] = CHANPIN;
  INLINE_NOP;
  logicdata[627] = CHANPIN;
  INLINE_NOP;
  logicdata[628] = CHANPIN;
  INLINE_NOP;
  logicdata[629] = CHANPIN;
  INLINE_NOP;
  logicdata[630] = CHANPIN;
  INLINE_NOP;
  logicdata[631] = CHANPIN;
  INLINE_NOP;
  logicdata[632] = CHANPIN;
  INLINE_NOP;
  logicdata[633] = CHANPIN;
  INLINE_NOP;
  logicdata[634] = CHANPIN;
  INLINE_NOP;
  logicdata[635] = CHANPIN;
  INLINE_NOP;
  logicdata[636] = CHANPIN;
  INLINE_NOP;
  logicdata[637] = CHANPIN;
  INLINE_NOP;
  logicdata[638] = CHANPIN;
  INLINE_NOP;
  logicdata[639] = CHANPIN;
  INLINE_NOP;
  logicdata[640] = CHANPIN;
  INLINE_NOP;
  logicdata[641] = CHANPIN;
  INLINE_NOP;
  logicdata[642] = CHANPIN;
  INLINE_NOP;
  logicdata[643] = CHANPIN;
  INLINE_NOP;
  logicdata[644] = CHANPIN;
  INLINE_NOP;
  logicdata[645] = CHANPIN;
  INLINE_NOP;
  logicdata[646] = CHANPIN;
  INLINE_NOP;
  logicdata[647] = CHANPIN;
  INLINE_NOP;
  logicdata[648] = CHANPIN;
  INLINE_NOP;
  logicdata[649] = CHANPIN;
  INLINE_NOP;
  logicdata[650] = CHANPIN;
  INLINE_NOP;
  logicdata[651] = CHANPIN;
  INLINE_NOP;
  logicdata[652] = CHANPIN;
  INLINE_NOP;
  logicdata[653] = CHANPIN;
  INLINE_NOP;
  logicdata[654] = CHANPIN;
  INLINE_NOP;
  logicdata[655] = CHANPIN;
  INLINE_NOP;
  logicdata[656] = CHANPIN;
  INLINE_NOP;
  logicdata[657] = CHANPIN;
  INLINE_NOP;
  logicdata[658] = CHANPIN;
  INLINE_NOP;
  logicdata[659] = CHANPIN;
  INLINE_NOP;
  logicdata[660] = CHANPIN;
  INLINE_NOP;
  logicdata[661] = CHANPIN;
  INLINE_NOP;
  logicdata[662] = CHANPIN;
  INLINE_NOP;
  logicdata[663] = CHANPIN;
  INLINE_NOP;
  logicdata[664] = CHANPIN;
  INLINE_NOP;
  logicdata[665] = CHANPIN;
  INLINE_NOP;
  logicdata[666] = CHANPIN;
  INLINE_NOP;
  logicdata[667] = CHANPIN;
  INLINE_NOP;
  logicdata[668] = CHANPIN;
  INLINE_NOP;
  logicdata[669] = CHANPIN;
  INLINE_NOP;
  logicdata[670] = CHANPIN;
  INLINE_NOP;
  logicdata[671] = CHANPIN;
  INLINE_NOP;
  logicdata[672] = CHANPIN;
  INLINE_NOP;
  logicdata[673] = CHANPIN;
  INLINE_NOP;
  logicdata[674] = CHANPIN;
  INLINE_NOP;
  logicdata[675] = CHANPIN;
  INLINE_NOP;
  logicdata[676] = CHANPIN;
  INLINE_NOP;
  logicdata[677] = CHANPIN;
  INLINE_NOP;
  logicdata[678] = CHANPIN;
  INLINE_NOP;
  logicdata[679] = CHANPIN;
  INLINE_NOP;
  logicdata[680] = CHANPIN;
  INLINE_NOP;
  logicdata[681] = CHANPIN;
  INLINE_NOP;
  logicdata[682] = CHANPIN;
  INLINE_NOP;
  logicdata[683] = CHANPIN;
  INLINE_NOP;
  logicdata[684] = CHANPIN;
  INLINE_NOP;
  logicdata[685] = CHANPIN;
  INLINE_NOP;
  logicdata[686] = CHANPIN;
  INLINE_NOP;
  logicdata[687] = CHANPIN;
  INLINE_NOP;
  logicdata[688] = CHANPIN;
  INLINE_NOP;
  logicdata[689] = CHANPIN;
  INLINE_NOP;
  logicdata[690] = CHANPIN;
  INLINE_NOP;
  logicdata[691] = CHANPIN;
  INLINE_NOP;
  logicdata[692] = CHANPIN;
  INLINE_NOP;
  logicdata[693] = CHANPIN;
  INLINE_NOP;
  logicdata[694] = CHANPIN;
  INLINE_NOP;
  logicdata[695] = CHANPIN;
  INLINE_NOP;
  logicdata[696] = CHANPIN;
  INLINE_NOP;
  logicdata[697] = CHANPIN;
  INLINE_NOP;
  logicdata[698] = CHANPIN;
  INLINE_NOP;
  logicdata[699] = CHANPIN;
  INLINE_NOP;
  logicdata[700] = CHANPIN;
  INLINE_NOP;
  logicdata[701] = CHANPIN;
  INLINE_NOP;
  logicdata[702] = CHANPIN;
  INLINE_NOP;
  logicdata[703] = CHANPIN;
  INLINE_NOP;
  logicdata[704] = CHANPIN;
  INLINE_NOP;
  logicdata[705] = CHANPIN;
  INLINE_NOP;
  logicdata[706] = CHANPIN;
  INLINE_NOP;
  logicdata[707] = CHANPIN;
  INLINE_NOP;
  logicdata[708] = CHANPIN;
  INLINE_NOP;
  logicdata[709] = CHANPIN;
  INLINE_NOP;
  logicdata[710] = CHANPIN;
  INLINE_NOP;
  logicdata[711] = CHANPIN;
  INLINE_NOP;
  logicdata[712] = CHANPIN;
  INLINE_NOP;
  logicdata[713] = CHANPIN;
  INLINE_NOP;
  logicdata[714] = CHANPIN;
  INLINE_NOP;
  logicdata[715] = CHANPIN;
  INLINE_NOP;
  logicdata[716] = CHANPIN;
  INLINE_NOP;
  logicdata[717] = CHANPIN;
  INLINE_NOP;
  logicdata[718] = CHANPIN;
  INLINE_NOP;
  logicdata[719] = CHANPIN;
  INLINE_NOP;
  logicdata[720] = CHANPIN;
  INLINE_NOP;
  logicdata[721] = CHANPIN;
  INLINE_NOP;
  logicdata[722] = CHANPIN;
  INLINE_NOP;
  logicdata[723] = CHANPIN;
  INLINE_NOP;
  logicdata[724] = CHANPIN;
  INLINE_NOP;
  logicdata[725] = CHANPIN;
  INLINE_NOP;
  logicdata[726] = CHANPIN;
  INLINE_NOP;
  logicdata[727] = CHANPIN;
  INLINE_NOP;
  logicdata[728] = CHANPIN;
  INLINE_NOP;
  logicdata[729] = CHANPIN;
  INLINE_NOP;
  logicdata[730] = CHANPIN;
  INLINE_NOP;
  logicdata[731] = CHANPIN;
  INLINE_NOP;
  logicdata[732] = CHANPIN;
  INLINE_NOP;
  logicdata[733] = CHANPIN;
  INLINE_NOP;
  logicdata[734] = CHANPIN;
  INLINE_NOP;
  logicdata[735] = CHANPIN;
  INLINE_NOP;
  logicdata[736] = CHANPIN;
  INLINE_NOP;
  logicdata[737] = CHANPIN;
  INLINE_NOP;
  logicdata[738] = CHANPIN;
  INLINE_NOP;
  logicdata[739] = CHANPIN;
  INLINE_NOP;
  logicdata[740] = CHANPIN;
  INLINE_NOP;
  logicdata[741] = CHANPIN;
  INLINE_NOP;
  logicdata[742] = CHANPIN;
  INLINE_NOP;
  logicdata[743] = CHANPIN;
  INLINE_NOP;
  logicdata[744] = CHANPIN;
  INLINE_NOP;
  logicdata[745] = CHANPIN;
  INLINE_NOP;
  logicdata[746] = CHANPIN;
  INLINE_NOP;
  logicdata[747] = CHANPIN;
  INLINE_NOP;
  logicdata[748] = CHANPIN;
  INLINE_NOP;
  logicdata[749] = CHANPIN;
  INLINE_NOP;
  logicdata[750] = CHANPIN;
  INLINE_NOP;
  logicdata[751] = CHANPIN;
  INLINE_NOP;
  logicdata[752] = CHANPIN;
  INLINE_NOP;
  logicdata[753] = CHANPIN;
  INLINE_NOP;
  logicdata[754] = CHANPIN;
  INLINE_NOP;
  logicdata[755] = CHANPIN;
  INLINE_NOP;
  logicdata[756] = CHANPIN;
  INLINE_NOP;
  logicdata[757] = CHANPIN;
  INLINE_NOP;
  logicdata[758] = CHANPIN;
  INLINE_NOP;
  logicdata[759] = CHANPIN;
  INLINE_NOP;
  logicdata[760] = CHANPIN;
  INLINE_NOP;
  logicdata[761] = CHANPIN;
  INLINE_NOP;
  logicdata[762] = CHANPIN;
  INLINE_NOP;
  logicdata[763] = CHANPIN;
  INLINE_NOP;
  logicdata[764] = CHANPIN;
  INLINE_NOP;
  logicdata[765] = CHANPIN;
  INLINE_NOP;
  logicdata[766] = CHANPIN;
  INLINE_NOP;
  logicdata[767] = CHANPIN;
  INLINE_NOP;
  logicdata[768] = CHANPIN;
  INLINE_NOP;
  logicdata[769] = CHANPIN;
  INLINE_NOP;
  logicdata[770] = CHANPIN;
  INLINE_NOP;
  logicdata[771] = CHANPIN;
  INLINE_NOP;
  logicdata[772] = CHANPIN;
  INLINE_NOP;
  logicdata[773] = CHANPIN;
  INLINE_NOP;
  logicdata[774] = CHANPIN;
  INLINE_NOP;
  logicdata[775] = CHANPIN;
  INLINE_NOP;
  logicdata[776] = CHANPIN;
  INLINE_NOP;
  logicdata[777] = CHANPIN;
  INLINE_NOP;
  logicdata[778] = CHANPIN;
  INLINE_NOP;
  logicdata[779] = CHANPIN;
  INLINE_NOP;
  logicdata[780] = CHANPIN;
  INLINE_NOP;
  logicdata[781] = CHANPIN;
  INLINE_NOP;
  logicdata[782] = CHANPIN;
  INLINE_NOP;
  logicdata[783] = CHANPIN;
  INLINE_NOP;
  logicdata[784] = CHANPIN;
  INLINE_NOP;
  logicdata[785] = CHANPIN;
  INLINE_NOP;
  logicdata[786] = CHANPIN;
  INLINE_NOP;
  logicdata[787] = CHANPIN;
  INLINE_NOP;
  logicdata[788] = CHANPIN;
  INLINE_NOP;
  logicdata[789] = CHANPIN;
  INLINE_NOP;
  logicdata[790] = CHANPIN;
  INLINE_NOP;
  logicdata[791] = CHANPIN;
  INLINE_NOP;
  logicdata[792] = CHANPIN;
  INLINE_NOP;
  logicdata[793] = CHANPIN;
  INLINE_NOP;
  logicdata[794] = CHANPIN;
  INLINE_NOP;
  logicdata[795] = CHANPIN;
  INLINE_NOP;
  logicdata[796] = CHANPIN;
  INLINE_NOP;
  logicdata[797] = CHANPIN;
  INLINE_NOP;
  logicdata[798] = CHANPIN;
  INLINE_NOP;
  logicdata[799] = CHANPIN;
  INLINE_NOP;
  logicdata[800] = CHANPIN;
  INLINE_NOP;
  logicdata[801] = CHANPIN;
  INLINE_NOP;
  logicdata[802] = CHANPIN;
  INLINE_NOP;
  logicdata[803] = CHANPIN;
  INLINE_NOP;
  logicdata[804] = CHANPIN;
  INLINE_NOP;
  logicdata[805] = CHANPIN;
  INLINE_NOP;
  logicdata[806] = CHANPIN;
  INLINE_NOP;
  logicdata[807] = CHANPIN;
  INLINE_NOP;
  logicdata[808] = CHANPIN;
  INLINE_NOP;
  logicdata[809] = CHANPIN;
  INLINE_NOP;
  logicdata[810] = CHANPIN;
  INLINE_NOP;
  logicdata[811] = CHANPIN;
  INLINE_NOP;
  logicdata[812] = CHANPIN;
  INLINE_NOP;
  logicdata[813] = CHANPIN;
  INLINE_NOP;
  logicdata[814] = CHANPIN;
  INLINE_NOP;
  logicdata[815] = CHANPIN;
  INLINE_NOP;
  logicdata[816] = CHANPIN;
  INLINE_NOP;
  logicdata[817] = CHANPIN;
  INLINE_NOP;
  logicdata[818] = CHANPIN;
  INLINE_NOP;
  logicdata[819] = CHANPIN;
  INLINE_NOP;
  logicdata[820] = CHANPIN;
  INLINE_NOP;
  logicdata[821] = CHANPIN;
  INLINE_NOP;
  logicdata[822] = CHANPIN;
  INLINE_NOP;
  logicdata[823] = CHANPIN;
  INLINE_NOP;
  logicdata[824] = CHANPIN;
  INLINE_NOP;
  logicdata[825] = CHANPIN;
  INLINE_NOP;
  logicdata[826] = CHANPIN;
  INLINE_NOP;
  logicdata[827] = CHANPIN;
  INLINE_NOP;
  logicdata[828] = CHANPIN;
  INLINE_NOP;
  logicdata[829] = CHANPIN;
  INLINE_NOP;
  logicdata[830] = CHANPIN;
  INLINE_NOP;
  logicdata[831] = CHANPIN;
  INLINE_NOP;
  logicdata[832] = CHANPIN;
  INLINE_NOP;
  logicdata[833] = CHANPIN;
  INLINE_NOP;
  logicdata[834] = CHANPIN;
  INLINE_NOP;
  logicdata[835] = CHANPIN;
  INLINE_NOP;
  logicdata[836] = CHANPIN;
  INLINE_NOP;
  logicdata[837] = CHANPIN;
  INLINE_NOP;
  logicdata[838] = CHANPIN;
  INLINE_NOP;
  logicdata[839] = CHANPIN;
  INLINE_NOP;
  logicdata[840] = CHANPIN;
  INLINE_NOP;
  logicdata[841] = CHANPIN;
  INLINE_NOP;
  logicdata[842] = CHANPIN;
  INLINE_NOP;
  logicdata[843] = CHANPIN;
  INLINE_NOP;
  logicdata[844] = CHANPIN;
  INLINE_NOP;
  logicdata[845] = CHANPIN;
  INLINE_NOP;
  logicdata[846] = CHANPIN;
  INLINE_NOP;
  logicdata[847] = CHANPIN;
  INLINE_NOP;
  logicdata[848] = CHANPIN;
  INLINE_NOP;
  logicdata[849] = CHANPIN;
  INLINE_NOP;
  logicdata[850] = CHANPIN;
  INLINE_NOP;
  logicdata[851] = CHANPIN;
  INLINE_NOP;
  logicdata[852] = CHANPIN;
  INLINE_NOP;
  logicdata[853] = CHANPIN;
  INLINE_NOP;
  logicdata[854] = CHANPIN;
  INLINE_NOP;
  logicdata[855] = CHANPIN;
  INLINE_NOP;
  logicdata[856] = CHANPIN;
  INLINE_NOP;
  logicdata[857] = CHANPIN;
  INLINE_NOP;
  logicdata[858] = CHANPIN;
  INLINE_NOP;
  logicdata[859] = CHANPIN;
  INLINE_NOP;
  logicdata[860] = CHANPIN;
  INLINE_NOP;
  logicdata[861] = CHANPIN;
  INLINE_NOP;
  logicdata[862] = CHANPIN;
  INLINE_NOP;
  logicdata[863] = CHANPIN;
  INLINE_NOP;
  logicdata[864] = CHANPIN;
  INLINE_NOP;
  logicdata[865] = CHANPIN;
  INLINE_NOP;
  logicdata[866] = CHANPIN;
  INLINE_NOP;
  logicdata[867] = CHANPIN;
  INLINE_NOP;
  logicdata[868] = CHANPIN;
  INLINE_NOP;
  logicdata[869] = CHANPIN;
  INLINE_NOP;
  logicdata[870] = CHANPIN;
  INLINE_NOP;
  logicdata[871] = CHANPIN;
  INLINE_NOP;
  logicdata[872] = CHANPIN;
  INLINE_NOP;
  logicdata[873] = CHANPIN;
  INLINE_NOP;
  logicdata[874] = CHANPIN;
  INLINE_NOP;
  logicdata[875] = CHANPIN;
  INLINE_NOP;
  logicdata[876] = CHANPIN;
  INLINE_NOP;
  logicdata[877] = CHANPIN;
  INLINE_NOP;
  logicdata[878] = CHANPIN;
  INLINE_NOP;
  logicdata[879] = CHANPIN;
  INLINE_NOP;
  logicdata[880] = CHANPIN;
  INLINE_NOP;
  logicdata[881] = CHANPIN;
  INLINE_NOP;
  logicdata[882] = CHANPIN;
  INLINE_NOP;
  logicdata[883] = CHANPIN;
  INLINE_NOP;
  logicdata[884] = CHANPIN;
  INLINE_NOP;
  logicdata[885] = CHANPIN;
  INLINE_NOP;
  logicdata[886] = CHANPIN;
  INLINE_NOP;
  logicdata[887] = CHANPIN;
  INLINE_NOP;
  logicdata[888] = CHANPIN;
  INLINE_NOP;
  logicdata[889] = CHANPIN;
  INLINE_NOP;
  logicdata[890] = CHANPIN;
  INLINE_NOP;
  logicdata[891] = CHANPIN;
  INLINE_NOP;
  logicdata[892] = CHANPIN;
  INLINE_NOP;
  logicdata[893] = CHANPIN;
  INLINE_NOP;
  logicdata[894] = CHANPIN;
  INLINE_NOP;
  logicdata[895] = CHANPIN;
  INLINE_NOP;
  logicdata[896] = CHANPIN;
  INLINE_NOP;
  logicdata[897] = CHANPIN;
  INLINE_NOP;
  logicdata[898] = CHANPIN;
  INLINE_NOP;
  logicdata[899] = CHANPIN;
  INLINE_NOP;
  logicdata[900] = CHANPIN;
  INLINE_NOP;
  logicdata[901] = CHANPIN;
  INLINE_NOP;
  logicdata[902] = CHANPIN;
  INLINE_NOP;
  logicdata[903] = CHANPIN;
  INLINE_NOP;
  logicdata[904] = CHANPIN;
  INLINE_NOP;
  logicdata[905] = CHANPIN;
  INLINE_NOP;
  logicdata[906] = CHANPIN;
  INLINE_NOP;
  logicdata[907] = CHANPIN;
  INLINE_NOP;
  logicdata[908] = CHANPIN;
  INLINE_NOP;
  logicdata[909] = CHANPIN;
  INLINE_NOP;
  logicdata[910] = CHANPIN;
  INLINE_NOP;
  logicdata[911] = CHANPIN;
  INLINE_NOP;
  logicdata[912] = CHANPIN;
  INLINE_NOP;
  logicdata[913] = CHANPIN;
  INLINE_NOP;
  logicdata[914] = CHANPIN;
  INLINE_NOP;
  logicdata[915] = CHANPIN;
  INLINE_NOP;
  logicdata[916] = CHANPIN;
  INLINE_NOP;
  logicdata[917] = CHANPIN;
  INLINE_NOP;
  logicdata[918] = CHANPIN;
  INLINE_NOP;
  logicdata[919] = CHANPIN;
  INLINE_NOP;
  logicdata[920] = CHANPIN;
  INLINE_NOP;
  logicdata[921] = CHANPIN;
  INLINE_NOP;
  logicdata[922] = CHANPIN;
  INLINE_NOP;
  logicdata[923] = CHANPIN;
  INLINE_NOP;
  logicdata[924] = CHANPIN;
  INLINE_NOP;
  logicdata[925] = CHANPIN;
  INLINE_NOP;
  logicdata[926] = CHANPIN;
  INLINE_NOP;
  logicdata[927] = CHANPIN;
  INLINE_NOP;
  logicdata[928] = CHANPIN;
  INLINE_NOP;
  logicdata[929] = CHANPIN;
  INLINE_NOP;
  logicdata[930] = CHANPIN;
  INLINE_NOP;
  logicdata[931] = CHANPIN;
  INLINE_NOP;
  logicdata[932] = CHANPIN;
  INLINE_NOP;
  logicdata[933] = CHANPIN;
  INLINE_NOP;
  logicdata[934] = CHANPIN;
  INLINE_NOP;
  logicdata[935] = CHANPIN;
  INLINE_NOP;
  logicdata[936] = CHANPIN;
  INLINE_NOP;
  logicdata[937] = CHANPIN;
  INLINE_NOP;
  logicdata[938] = CHANPIN;
  INLINE_NOP;
  logicdata[939] = CHANPIN;
  INLINE_NOP;
  logicdata[940] = CHANPIN;
  INLINE_NOP;
  logicdata[941] = CHANPIN;
  INLINE_NOP;
  logicdata[942] = CHANPIN;
  INLINE_NOP;
  logicdata[943] = CHANPIN;
  INLINE_NOP;
  logicdata[944] = CHANPIN;
  INLINE_NOP;
  logicdata[945] = CHANPIN;
  INLINE_NOP;
  logicdata[946] = CHANPIN;
  INLINE_NOP;
  logicdata[947] = CHANPIN;
  INLINE_NOP;
  logicdata[948] = CHANPIN;
  INLINE_NOP;
  logicdata[949] = CHANPIN;
  INLINE_NOP;
  logicdata[950] = CHANPIN;
  INLINE_NOP;
  logicdata[951] = CHANPIN;
  INLINE_NOP;
  logicdata[952] = CHANPIN;
  INLINE_NOP;
  logicdata[953] = CHANPIN;
  INLINE_NOP;
  logicdata[954] = CHANPIN;
  INLINE_NOP;
  logicdata[955] = CHANPIN;
  INLINE_NOP;
  logicdata[956] = CHANPIN;
  INLINE_NOP;
  logicdata[957] = CHANPIN;
  INLINE_NOP;
  logicdata[958] = CHANPIN;
  INLINE_NOP;
  logicdata[959] = CHANPIN;
  INLINE_NOP;
  logicdata[960] = CHANPIN;
  INLINE_NOP;
  logicdata[961] = CHANPIN;
  INLINE_NOP;
  logicdata[962] = CHANPIN;
  INLINE_NOP;
  logicdata[963] = CHANPIN;
  INLINE_NOP;
  logicdata[964] = CHANPIN;
  INLINE_NOP;
  logicdata[965] = CHANPIN;
  INLINE_NOP;
  logicdata[966] = CHANPIN;
  INLINE_NOP;
  logicdata[967] = CHANPIN;
  INLINE_NOP;
  logicdata[968] = CHANPIN;
  INLINE_NOP;
  logicdata[969] = CHANPIN;
  INLINE_NOP;
  logicdata[970] = CHANPIN;
  INLINE_NOP;
  logicdata[971] = CHANPIN;
  INLINE_NOP;
  logicdata[972] = CHANPIN;
  INLINE_NOP;
  logicdata[973] = CHANPIN;
  INLINE_NOP;
  logicdata[974] = CHANPIN;
  INLINE_NOP;
  logicdata[975] = CHANPIN;
  INLINE_NOP;
  logicdata[976] = CHANPIN;
  INLINE_NOP;
  logicdata[977] = CHANPIN;
  INLINE_NOP;
  logicdata[978] = CHANPIN;
  INLINE_NOP;
  logicdata[979] = CHANPIN;
  INLINE_NOP;
  logicdata[980] = CHANPIN;
  INLINE_NOP;
  logicdata[981] = CHANPIN;
  INLINE_NOP;
  logicdata[982] = CHANPIN;
  INLINE_NOP;
  logicdata[983] = CHANPIN;
  INLINE_NOP;
  logicdata[984] = CHANPIN;
  INLINE_NOP;
  logicdata[985] = CHANPIN;
  INLINE_NOP;
  logicdata[986] = CHANPIN;
  INLINE_NOP;
  logicdata[987] = CHANPIN;
  INLINE_NOP;
  logicdata[988] = CHANPIN;
  INLINE_NOP;
  logicdata[989] = CHANPIN;
  INLINE_NOP;
  logicdata[990] = CHANPIN;
  INLINE_NOP;
  logicdata[991] = CHANPIN;
  INLINE_NOP;
  logicdata[992] = CHANPIN;
  INLINE_NOP;
  logicdata[993] = CHANPIN;
  INLINE_NOP;
  logicdata[994] = CHANPIN;
  INLINE_NOP;
  logicdata[995] = CHANPIN;
  INLINE_NOP;
  logicdata[996] = CHANPIN;
  INLINE_NOP;
  logicdata[997] = CHANPIN;
  INLINE_NOP;
  logicdata[998] = CHANPIN;
  INLINE_NOP;
  logicdata[999] = CHANPIN;
  INLINE_NOP;
  logicdata[1000] = CHANPIN;
  INLINE_NOP;
  logicdata[1001] = CHANPIN;
  INLINE_NOP;
  logicdata[1002] = CHANPIN;
  INLINE_NOP;
  logicdata[1003] = CHANPIN;
  INLINE_NOP;
  logicdata[1004] = CHANPIN;
  INLINE_NOP;
  logicdata[1005] = CHANPIN;
  INLINE_NOP;
  logicdata[1006] = CHANPIN;
  INLINE_NOP;
  logicdata[1007] = CHANPIN;
  INLINE_NOP;
  logicdata[1008] = CHANPIN;
  INLINE_NOP;
  logicdata[1009] = CHANPIN;
  INLINE_NOP;
  logicdata[1010] = CHANPIN;
  INLINE_NOP;
  logicdata[1011] = CHANPIN;
  INLINE_NOP;
  logicdata[1012] = CHANPIN;
  INLINE_NOP;
  logicdata[1013] = CHANPIN;
  INLINE_NOP;
  logicdata[1014] = CHANPIN;
  INLINE_NOP;
  logicdata[1015] = CHANPIN;
  INLINE_NOP;
  logicdata[1016] = CHANPIN;
  INLINE_NOP;
  logicdata[1017] = CHANPIN;
  INLINE_NOP;
  logicdata[1018] = CHANPIN;
  INLINE_NOP;
  logicdata[1019] = CHANPIN;
  INLINE_NOP;
  logicdata[1020] = CHANPIN;
  INLINE_NOP;
  logicdata[1021] = CHANPIN;
  INLINE_NOP;
  logicdata[1022] = CHANPIN;
  INLINE_NOP;
  logicdata[1023] = CHANPIN;
  INLINE_NOP;
#endif


  DEBUG_OFF; /* debug timing measurement */

  /* re-enable interrupts now that we're done sampling. */
  sei();

  /*
   * dump the samples back to the SUMP client.  nothing special
   * is done for any triggers, this is effectively the 0/100 buffer split.
   */
  for (i = 0 ; i < readCount; i++) {
#ifdef USE_PORTD
    Serial.write(logicdata[i] >> 2);
#else
    Serial.write(logicdata[i]);
#endif
  }
}



void captureMilli() {
  unsigned int i = 0;

  if(rleEnabled) {
    /*
     * very basic trigger, just like in captureMicros() above.
     */
    if (trigger) {
      while ((trigger_values ^ (CHANPIN & B01111111)) & trigger);
    }

    byte lastSample = 0;
    byte sampleCount = 0;

    while(i < readCount) {
      /*
       * Implementation of the RLE unlimited protocol: timings might be off a little
       */
      if(lastSample == (CHANPIN & B01111111) && sampleCount < 127) {
        sampleCount++;
        delay(delayTime);
        continue;
      }
      if(sampleCount != 0) {
        logicdata[i] = B10000000 | sampleCount;
        sampleCount = 0;
        i++;
        continue;
      }
      logicdata[i] = (CHANPIN & B01111111);
      lastSample = (CHANPIN & B01111111);
      delay(delayTime);

      i++;
    }
  } 
  else {
    /*
     * very basic trigger, just like in captureMicros() above.
     */
    if (trigger) {
      while ((trigger_values ^ CHANPIN) & trigger);
    }

    for (i = 0 ; i < readCount; i++) {
      logicdata[i] = CHANPIN;
      delay(delayTime);
    }
  }
  for (i = 0 ; i < readCount; i++) {
#ifdef USE_PORTD
    Serial.write(logicdata[i] >> 2);
#else
    Serial.write(logicdata[i]);
#endif
  }
}

/*
 * This function provides sampling with triggering and a circular trigger
 * buffer.
 * This works ok at 500KHz and lower sample rates.  We don't have enough time
 * with a 16MHz clock to sample at 1MHz into the circular buffer.  A 20MHz
 * clock might be ok but all of the timings would have to be redone.
 * 
 */
void triggerMicro() {
  unsigned int i = 0;

  logicIndex = 0;
  triggerIndex = 0;

  /*
   * disable interrupts during capture to maintain precision.
   * we're hand padding loops with NOP instructions so we absolutely
   * cannot have any interrupts firing.
   */
  cli();

  /*
   * toggle pin a few times to activate trigger for debugging.
   * this is used during development to measure the sample intervals.
   * it is best to just leave the toggling in place so we don't alter
   * any timing unexpectedly.
   * Arduino digital pin 8 is being used here.
   */
  DEBUG_ENABLE;
#ifdef DEBUG
  DEBUG_ON;
  delayMicroseconds(20);
  DEBUG_OFF;
  delayMicroseconds(20);
  DEBUG_ON;
  delayMicroseconds(20);
  DEBUG_OFF;
  delayMicroseconds(20);
#endif

  if (delayTime == 1) {
    /*
     * 1MHz case.  We can't really do it at the moment.  Timing is too tight.
     * We can fake it, or skip it, or rework it....
     * This should be retested on a 20MHz clocked microcontroller.
     * The data is flat out wrong for the 1MHz case.
     */

    /*
     * return for now, the client will timeout eventually or the user will
     * click stop.
     */
    return;
  } 
  else if (delayTime == 2) {
    /*
     * 500KHz case.  We should be able to manage this in time.
     *
     * busy loop reading CHANPIN until we trigger.
     * we always start capturing at the start of the buffer
     * and use it as a circular buffer
     */
    DEBUG_ON; /* debug timing measurement */
    while ((trigger_values ^ (logicdata[logicIndex] = CHANPIN)) & trigger) {
      /* DEBUG_OFF; */
      /* increment index. */
      logicIndex++;
      if (logicIndex >= readCount) {
        logicIndex = 0;
      }
      /*
       * pad loop to 2.0 uS (with pin toggles it is 3 x nop)
       * without pin toggles, will try 1 nop.
       * __asm__("nop\n\t""nop\n\t""nop\n\t");
       */
      __asm__("nop\n\t");
      /* DEBUG_ON; */
    }
    /* this pads the immediate trigger case to 2.0 uS, just as an example. */
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    DEBUG_OFF; /* debug timing measurement */

    /* 
     * One sample size delay. ends up being 2 uS combined with assignment
     * below.  This padding is so we have a consistent timing interval
     * between the trigger point and the subsequent samples.
     */
    delayMicroseconds(1);
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");

    /* 'logicIndex' now points to trigger sample, keep track of it */
    triggerIndex = logicIndex;

    /* keep sampling for delayCount after trigger */
    DEBUG_ON; /* debug timing measurement */
    /*
     * this is currently taking:
     * 1025.5 uS for 512 samples. (512 samples, 0/100 split)
     *  513.5 uS for 256 samples. (512 samples, 50/50 split)
     */
    for (i = 0 ; i < delayCount; i++) {
      if (logicIndex >= readCount) {
        logicIndex = 0;
      }
      logicdata[logicIndex++] = CHANPIN;
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    }
    DEBUG_OFF; /* debug timing measurement */
    delayMicroseconds(100);
  } 
  else {
    /*
     * Less than 500KHz case.  This uses delayMicroseconds() and some padding
     * to get precise timing, at least for the after trigger samples.
     *
     * busy loop reading CHANPIN until we trigger.
     * we always start capturing at the start of the buffer
     * and use it as a circular buffer
     *
     */
    DEBUG_ON; /* debug timing measurement */
    while ((trigger_values ^ (logicdata[logicIndex] = CHANPIN)) & trigger) {
      /* DEBUG_OFF; */
      /* increment index. */
      logicIndex++;
      if (logicIndex >= readCount) {
        logicIndex = 0;
      }
      else {
        /* pad the same number of cycles as the above assignment (needs verification) */
        __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      }
      delayMicroseconds(delayTime - 3);
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      /* DEBUG_ON; */
    }
    DEBUG_OFF; /* debug timing measurement */

    /* 'logicIndex' now points to trigger sample, keep track of it */
    triggerIndex = logicIndex;

    /*
     * This needs adjustment so that we have the right spacing between the
     * before trigger samples and the after trigger samples.
     */
    delayMicroseconds(delayTime - 2);
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    __asm__("nop\n\t""nop\n\t""nop\n\t");

    /* keep sampling for delayCount after trigger */
    DEBUG_ON; /* debug timing measurement */
    for (i = 0 ; i < delayCount; i++) {
      if (logicIndex >= readCount) {
        logicIndex = 0;
      }
      logicdata[logicIndex++] = CHANPIN;
      delayMicroseconds(delayTime - 3);
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t");
    }
    DEBUG_OFF; /* debug timing measurement */
    delayMicroseconds(100);
  }

  /* re-enable interrupts */
  sei();

  /*
   * trigger has fired and we have read delayCount of samples after the
   * trigger fired.  triggerIndex now points to the trigger sample
   * logicIndex now points to the last sample taken and logicIndex + 1
   * is where we should start dumping since it is circular.
   *
   * our buffer starts one entry above the last read entry.
   */
  logicIndex++;

  for (i = 0 ; i < readCount; i++) {
    if (logicIndex >= readCount) {
      logicIndex = 0;
    }
#ifdef USE_PORTD
    Serial.write(logicdata[logicIndex++] >> 2);
#else
    Serial.write(logicdata[logicIndex++]);
#endif
  }
}

/*
 * This function calculates what delay we need for the specific sample rate.
 * The dividers are based on SUMP's 100Mhz clock.
 * For example, a 1MHz sample rate has a divider of 99 (0x63 in the command
 * byte).
 * rate = clock / (divider + 1)
 * rate = 100,000,000 / (99 + 1)
 * result is 1,000,000 saying we want a 1MHz sample rate.
 * We calculate our inter sample delay from the divider and the delay between
 * samples gives us the sample rate per second.
 * So for 1MHz, delay = (99 + 1) / 100 which gives us a 1 microsecond delay.
 * For 500KHz, delay = (199 + 1) / 100 which gives us a 2 microsecond delay.
 *
 */
void setupDelay() {
  if (divider >= 1500000) {
    useMicro = 0;
    delayTime = (divider + 1) / 100000;
  } 
  else {
    useMicro = 1;
    delayTime = (divider + 1) / 100;
  }
}

/*
 * This function returns the metadata about our capabilities.  It is sent in
 * response to the  OpenBench Logic Sniffer extended get metadata command.
 *
 */
void get_metadata() {
  /* device name */
  Serial.write((uint8_t)0x01);
  Serial.write('A');
  Serial.write('G');
  Serial.write('L');
  Serial.write('A');
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  Serial.write('M');
#endif /* Mega */
  Serial.write('v');
  Serial.write('0');
  Serial.write((uint8_t)0x00);

  /* firmware version */
  Serial.write((uint8_t)0x02);
  Serial.write('0');
  Serial.write('.');
  Serial.write('1');
  Serial.write('2');
  Serial.write((uint8_t)0x00);

  /* sample memory */
  Serial.write((uint8_t)0x21);
  Serial.write((uint8_t)0x00);
  Serial.write((uint8_t)0x00);
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  /* 7168 bytes */
  Serial.write((uint8_t)0x1C);
  Serial.write((uint8_t)0x00);
#elif defined(__AVR_ATmega328P__)
  /* 1024 bytes */
  Serial.write((uint8_t)0x04);
  Serial.write((uint8_t)0x00);
#else
  /* 532 bytes */
  Serial.write((uint8_t)0x02);
  Serial.write((uint8_t)0x14);
#endif /* Mega */

  /* sample rate (4MHz) */
  Serial.write((uint8_t)0x23);
  Serial.write((uint8_t)0x00);
  Serial.write((uint8_t)0x3D);
  Serial.write((uint8_t)0x09);
  Serial.write((uint8_t)0x00);

  /* number of probes (6 by default on Arduino, 8 on Mega) */
  Serial.write((uint8_t)0x40);
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  Serial.write((uint8_t)0x08);
#else
#ifdef CHAN5
  Serial.write((uint8_t)0x06);
#else
  Serial.write((uint8_t)0x05);
#endif /* CHAN5 */
#endif /* Mega */

  /* protocol version (2) */
  Serial.write((uint8_t)0x41);
  Serial.write((uint8_t)0x02);

  /* end of data */
  Serial.write((uint8_t)0x00);  
}

/*
 * This is used by the '0' debug command to dump the contents of some
 * interesting variables and the debug buffer.
 *
 */
#ifdef DEBUG
void debugprint() {
  int i;

#if 0
  Serial.print("divider = ");
  Serial.println(divider, DEC);
  Serial.print("delayTime = ");
  Serial.println(delayTime, DEC);
  Serial.print("trigger_values = ");
  Serial.println(trigger_values, BIN);
#endif
  Serial.print("readCount = ");
  Serial.println(readCount, DEC);
  Serial.print("delayCount = ");
  Serial.println(delayCount, DEC);
  Serial.print("logicIndex = ");
  Serial.println(logicIndex, DEC);
  Serial.print("triggerIndex = ");
  Serial.println(triggerIndex, DEC);
  Serial.print("rleEnabled = ");
  Serial.println(rleEnabled, DEC);

  Serial.println("Bytes:");

  for (i = 0 ; i < savecount; i++) {
    if (savebytes[i] == 0x20) {
      Serial.println();
    } 
    else {
      Serial.print(savebytes[i], HEX);
      Serial.write(' ');
    }
  }
  Serial.println("done...");
}

/*
 * This is used by the '2' debug command to dump the contents
 * of the sample buffer.
 */
void debugdump() {
  int i;
  int j = 1;

  Serial.print("\r\n");

  for (i = 0 ; i < MAX_CAPTURE_SIZE; i++) {
#ifdef USE_PORTD
    Serial.print(logicdata[i] >> 2, HEX);
#else
    Serial.print(logicdata[i], HEX);
#endif
    Serial.print(" ");
    if (j == 32) {
      Serial.print("\r\n");
      j = 0;
    }
    j++;
  }
}
#endif /* DEBUG */











