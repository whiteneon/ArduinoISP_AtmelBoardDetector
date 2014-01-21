// ArduinoISP version 04m3
// Copyright (c) 2008-2011 Randall Bohn
// If you require a license, see 
//     http://www.opensource.org/licenses/bsd-license.php
//
// This sketch turns the Arduino into a AVRISP
// using the following arduino pins:
//
// pin name:    not-mega:         mega(1280 and 2560)
// slave reset: 10:               53 
// MOSI:        11:               51 
// MISO:        12:               50 
// SCK:         13:               52 
//
// Put an LED (with resistor) on the following pins:
// 9: Heartbeat   - shows the programmer is running
// 8: Error       - Lights up if something goes wrong (use red if that makes sense)
// 7: Programming - In communication with the slave
//

#include <SPI.h> //New Program Data
#include "pins_arduino.h"
#define RESET     SS

#define LED_HB    9
#define LED_ERR   8
#define LED_PMODE 7
#define CLOCKOUT 5
#define SELECTOR 6
#define PROG_FLICKER true

#define HWVER 2
#define SWMAJ 1
#define SWMIN 18

// STK Definitions
#define STK_OK      0x10
#define STK_FAILED  0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC  0x14
#define STK_NOSYNC  0x15
#define CRC_EOP     0x20 //ok it is a space...

//New Program Data

//const byte CLOCKOUT = 6;
//const byte RESET = 10;  // --> goes to reset on the target board

// number of items in an array
#define NUMITEMS(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0])))

// programming commands to send via SPI to the chip
enum {
    progamEnable = 0xAC,
    programAcknowledge = 0x53,
    
    readSignatureByte = 0x30,
    readCalibrationByte = 0x38,
    
    readLowFuseByte = 0x50,       readLowFuseByteArg2 = 0x00,
    readExtendedFuseByte = 0x50,  readExtendedFuseByteArg2 = 0x08,
    readHighFuseByte = 0x58,      readHighFuseByteArg2 = 0x08,  
    readLockByte = 0x58,          readLockByteArg2 = 0x00,  
    
    readProgramMemory = 0x20,  
    loadExtendedAddressByte = 0x4D,
    
};  // end of enum

typedef struct {
   byte sig [3];
   char * desc;
   unsigned long flashSize;
   unsigned int baseBootSize;
} signatureType;

const unsigned long kb = 1024;

// see Atmega328 datasheet page 298
const signatureType signatures [] = 
  {
//     signature          description   flash size  bootloader size

  // Attiny84 family
  { { 0x1E, 0x91, 0x0B }, "ATtiny24",   2 * kb,         0 },
  { { 0x1E, 0x92, 0x07 }, "ATtiny44",   4 * kb,         0 },
  { { 0x1E, 0x93, 0x0C }, "ATtiny84",   8 * kb,         0 },

  // Attiny85 family
  { { 0x1E, 0x91, 0x08 }, "ATtiny25",   2 * kb,         0 },
  { { 0x1E, 0x92, 0x06 }, "ATtiny45",   4 * kb,         0 },
  { { 0x1E, 0x93, 0x0B }, "ATtiny85",   8 * kb,         0 },

  // Atmega328 family
  { { 0x1E, 0x92, 0x0A }, "ATmega48PA",   4 * kb,         0 },
  { { 0x1E, 0x93, 0x0F }, "ATmega88PA",   8 * kb,       256 },
  { { 0x1E, 0x94, 0x0B }, "ATmega168PA", 16 * kb,       256 },
  { { 0x1E, 0x95, 0x0F }, "ATmega328P",  32 * kb,       512 },

  // Atmega644 family
  { { 0x1E, 0x94, 0x0A }, "ATmega164P",   16 * kb,      256 },
  { { 0x1E, 0x95, 0x08 }, "ATmega324P",   32 * kb,      512 },
  { { 0x1E, 0x96, 0x0A }, "ATmega644P",   64 * kb,   1 * kb },

  // Atmega2560 family
  { { 0x1E, 0x96, 0x08 }, "ATmega640",    64 * kb,   1 * kb },
  { { 0x1E, 0x97, 0x03 }, "ATmega1280",  128 * kb,   1 * kb },
  { { 0x1E, 0x97, 0x04 }, "ATmega1281",  128 * kb,   1 * kb },
  { { 0x1E, 0x98, 0x01 }, "ATmega2560",  256 * kb,   1 * kb },
  { { 0x1E, 0x98, 0x02 }, "ATmega2561",  256 * kb,   1 * kb },
  
  // Atmega32U2 family
  { { 0x1E, 0x93, 0x89 }, "ATmega8U2",    8 * kb,   512 },
  { { 0x1E, 0x94, 0x89 }, "ATmega16U2",  16 * kb,   512 },
  { { 0x1E, 0x95, 0x8A }, "ATmega32U2",  32 * kb,   512 },

  // Atmega32U4 family
  { { 0x1E, 0x94, 0x88 }, "ATmega16U4",  16 * kb,   512 },
  { { 0x1E, 0x95, 0x87 }, "ATmega32U4",  32 * kb,   512 },
  
  // ATmega1284P family
  { { 0x1E, 0x97, 0x05 }, "ATmega1284P", 128 * kb,   1 * kb },
  
  // ATtiny4313 family
  { { 0x1E, 0x91, 0x0A }, "ATtiny2313A", 2 * kb,   0 },
  { { 0x1E, 0x92, 0x0D }, "ATtiny4313",  4 * kb,   0 },
  
  // ATtiny13 family
  { { 0x1E, 0x90, 0x07 }, "ATtiny13A",   1 * kb,   0 },
  
  // Atmega8A family
  { { 0x1E, 0x93, 0x07 }, "ATmega8A",    8 * kb, 256 },
  
  };  // end of signatures

// if signature found in above table, this is its index
int foundSig = -1;
byte lastAddressMSB = 0;
word programLengthToRead = 0; //word == unsigned int
// execute one programming instruction ... b1 is command, b2, b3, b4 are arguments
//  processor may return a result on the 4th transfer, this is returned.
byte program (const byte b1, const byte b2 = 0, const byte b3 = 0, const byte b4 = 0)
  {
  SPI.transfer (b1);  
  SPI.transfer (b2);  
  SPI.transfer (b3);  
  return SPI.transfer (b4);  
  } // end of program
  
byte readFlash (unsigned long addr)
  {
  
  // set the extended (most significant) address byte if necessary
  byte MSB = (addr >> 16) & 0xFF;
  if (MSB != lastAddressMSB)
    {
    program (loadExtendedAddressByte, 0, MSB); 
    lastAddressMSB = MSB;
    }  // end if different MSB
     
  byte high = (addr & 1) ? 0x08 : 0;  // set if high byte wanted
  addr >>= 1;  // turn into word address
  return program (readProgramMemory | high, highByte (addr), lowByte (addr));
  } // end of readFlash
  
void showHex (const byte b, const boolean newline = false)
  {
  // try to avoid using sprintf
  char buf [4] = { ((b >> 4) & 0x0F) | '0', (b & 0x0F) | '0', ' ' , 0 };
  if (buf [0] > '9')
    buf [0] += 7;
  if (buf [1] > '9')
    buf [1] += 7;
  Serial.print (buf);
  if (newline)
    Serial.println ();
  }  // end of showHex 
  
void showYesNo (const boolean b, const boolean newline = false)
  {
  if (b)
    Serial.print ("Yes");
  else
    Serial.print ("No");
  if (newline)
    Serial.println ();
  }  // end of showYesNo 
  
void readBootloader ()
  {
  unsigned long addr;
  unsigned int  len;
  byte hFuse = program (readHighFuseByte, readHighFuseByteArg2);
  
  if (signatures [foundSig].baseBootSize == 0)
    {
    Serial.println ("No bootloader support.");
    return;  
    }
    
  addr = signatures [foundSig].flashSize;
  len = signatures [foundSig].baseBootSize;
  
  Serial.print ("Bootloader in use: ");
  showYesNo ((hFuse & _BV (0)) == 0, true);
  Serial.print ("EEPROM preserved through erase: ");
  showYesNo ((hFuse & _BV (3)) == 0, true);
  Serial.print ("Watchdog timer always on: ");
  showYesNo ((hFuse & _BV (4)) == 0, true);
  
  // work out bootloader size
  // these 2 bits basically give a base bootloader size multiplier
  switch ((hFuse >> 1) & 3)
    {
    case 0: len *= 8; break;  
    case 1: len *= 4; break;  
    case 2: len *= 2; break;  
    case 3: len *= 1; break;  
    }  // end of switch

  // where bootloader starts
  addr -= len;
  
  Serial.print ("Bootloader is ");
  Serial.print (len);
  Serial.print (" bytes starting at ");
  Serial.println (addr, HEX);
  Serial.println ();
  Serial.println ("Bootloader:");
  Serial.println ();
 
  for (int i = 0; i < len; i++)
    {
    // show address
    if (i % 16 == 0)
      {
      Serial.print (addr + i, HEX);
      Serial.print (": ");
      }
    showHex (readFlash (addr + i));
    // new line every 16 bytes
    if (i % 16 == 15)
      Serial.println ();
    }  // end of for
    
  Serial.println ();

  Serial.println ();  

  } // end of readBootloader
  
void readProgram ()
  {
  unsigned long addr = 0;
  unsigned int  len = 0;
  Serial.println ();
  if (digitalRead(SELECTOR) == LOW) {
    len = programLengthToRead;
    Serial.print("Printing flash memory ");
    Serial.print(len);
    Serial.println("KB");    
  } else {
    Serial.println ("First 256 bytes of program memory:");
    len = 256;
  }
  Serial.println ();
 
  for (int i = 0; i < len; i++)
    {
    // show address
    if (i % 16 == 0)
      {
      Serial.print (addr + i, HEX);
      Serial.print (": ");
      }
    showHex (readFlash (addr + i));
    // new line every 16 bytes
    if (i % 16 == 15)
      Serial.println ();
    }  // end of for
    
  Serial.println ();

  } // end of readProgram
  
void startProgramming ()
  {
  byte confirm;
  pinMode (SS, OUTPUT);
  pinMode (SCK, OUTPUT);
  
  // we are in sync if we get back programAcknowledge on the third byte
  do 
    {
    delay (100);
    // ensure SCK low
    digitalWrite (SCK, LOW);
    // then pulse reset, see page 309 of datasheet
    digitalWrite (SS, HIGH);
    delay (1);  // pulse for at least 2 clock cycles
    digitalWrite (SS, LOW);
    delay (25);  // wait at least 20 mS
    SPI.transfer (progamEnable);  
    SPI.transfer (programAcknowledge);  
    confirm = SPI.transfer (0);  
    SPI.transfer (0);  
    } while (confirm != programAcknowledge);
    
  Serial.println ("Entered programming mode OK.");
  }  // end of startProgramming

void getSignature ()
  {
  byte sig [3];
  Serial.print ("Signature = ");
  for (byte i = 0; i < 3; i++)
    {
    sig [i] = program (readSignatureByte, 0, i); 
    showHex (sig [i]);
    }
  Serial.println ();
  
  for (int j = 0; j < NUMITEMS (signatures); j++)
    {
    if (memcmp (sig, signatures [j].sig, sizeof sig) == 0)
      {
      foundSig = j;
      Serial.print ("Processor = ");
      Serial.println (signatures [j].desc);
      Serial.print ("Flash memory size = ");
      Serial.println (signatures [j].flashSize, DEC);
      programLengthToRead = signatures[j].flashSize;
      return;
      }
    }

  Serial.println ("Unrecogized signature.");  
  }  // end of getSignature

void getFuseBytes ()
  {
  Serial.print ("LFuse = ");
  showHex (program (readLowFuseByte, readLowFuseByteArg2), true);
  Serial.print ("HFuse = ");
  showHex (program (readHighFuseByte, readHighFuseByteArg2), true);
  Serial.print ("EFuse = ");
  showHex (program (readExtendedFuseByte, readExtendedFuseByteArg2), true);
  Serial.print ("Lock byte = ");
  showHex (program (readLockByte, readLockByteArg2), true);
  Serial.print ("Clock calibration = ");
  showHex (program (readCalibrationByte), true);
  }  // end of getFuseBytes
//****************************************//


void pulse(int pin, int times);

void setup() {
  pinMode(SELECTOR, INPUT_PULLUP);
  if (digitalRead(SELECTOR) == LOW) {
    //Input activated to enable Atmel chip Detector
    detectBoard();
  } else {
    Serial.begin(19200);
    pinMode(LED_PMODE, OUTPUT);
    pulse(LED_PMODE, 2);
    pinMode(LED_ERR, OUTPUT);
    pulse(LED_ERR, 2);
    pinMode(LED_HB, OUTPUT);
    pulse(LED_HB, 2);
  }
}
///////////////////////////////
//Begin ISP Functions
///////////////////////////////
int error=0;
int pmode=0;
// address for reading and writing, set by 'U' command
int here;
uint8_t buff[256]; // global block storage

#define beget16(addr) (*addr * 256 + *(addr+1) )
typedef struct param {
  uint8_t devicecode;
  uint8_t revision;
  uint8_t progtype;
  uint8_t parmode;
  uint8_t polling;
  uint8_t selftimed;
  uint8_t lockbytes;
  uint8_t fusebytes;
  int flashpoll;
  int eeprompoll;
  int pagesize;
  int eepromsize;
  int flashsize;
} 
parameter;

parameter param;

// this provides a heartbeat on pin 9, so you can tell the software is running.
uint8_t hbval=128;
int8_t hbdelta=8;
void heartbeat() {
  if (hbval > 192) hbdelta = -hbdelta;
  if (hbval < 32) hbdelta = -hbdelta;
  hbval += hbdelta;
  analogWrite(LED_HB, hbval);
  delay(20);
}


void loop(void) {
  // is pmode active?
  if (pmode) digitalWrite(LED_PMODE, HIGH); 
  else digitalWrite(LED_PMODE, LOW);
  // is there an error?
  if (error) digitalWrite(LED_ERR, HIGH); 
  else digitalWrite(LED_ERR, LOW);

  // light the heartbeat LED
  heartbeat();
  if (Serial.available()) {
    avrisp();
  }
  if (digitalRead(SELECTOR) == LOW) {
    //Selector button pushed, do reset so can initialize in
    //Atmel Chip Detector mode
    software_Reset();
  }
}

uint8_t getch() {
  while(!Serial.available());
  return Serial.read();
}
void fill(int n) {
  for (int x = 0; x < n; x++) {
    buff[x] = getch();
  }
}

#define PTIME 30
void pulse(int pin, int times) {
  do {
    digitalWrite(pin, HIGH);
    delay(PTIME);
    digitalWrite(pin, LOW);
    delay(PTIME);
  } 
  while (times--);
}

void prog_lamp(int state) {
  if (PROG_FLICKER)
    digitalWrite(LED_PMODE, state);
}

void spi_init() {
  uint8_t x;
  SPCR = 0x53;
  x=SPSR;
  x=SPDR;
}

void spi_wait() {
  do {
  } 
  while (!(SPSR & (1 << SPIF)));
}

uint8_t spi_send(uint8_t b) {
  uint8_t reply;
  SPDR=b;
  spi_wait();
  reply = SPDR;
  return reply;
}

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  uint8_t n;
  spi_send(a); 
  n=spi_send(b);
  //if (n != a) error = -1;
  n=spi_send(c);
  return spi_send(d);
}

void empty_reply() {
  if (CRC_EOP == getch()) {
    Serial.print((char)STK_INSYNC);
    Serial.print((char)STK_OK);
  } 
  else {
    error++;
    Serial.print((char)STK_NOSYNC);
  }
}

void breply(uint8_t b) {
  if (CRC_EOP == getch()) {
    Serial.print((char)STK_INSYNC);
    Serial.print((char)b);
    Serial.print((char)STK_OK);
  } 
  else {
    error++;
    Serial.print((char)STK_NOSYNC);
  }
}

void get_version(uint8_t c) {
  switch(c) {
  case 0x80:
    breply(HWVER);
    break;
  case 0x81:
    breply(SWMAJ);
    break;
  case 0x82:
    breply(SWMIN);
    break;
  case 0x93:
    breply('S'); // serial programmer
    break;
  default:
    breply(0);
  }
}

void set_parameters() {
  // call this after reading paramter packet into buff[]
  param.devicecode = buff[0];
  param.revision   = buff[1];
  param.progtype   = buff[2];
  param.parmode    = buff[3];
  param.polling    = buff[4];
  param.selftimed  = buff[5];
  param.lockbytes  = buff[6];
  param.fusebytes  = buff[7];
  param.flashpoll  = buff[8]; 
  // ignore buff[9] (= buff[8])
  // following are 16 bits (big endian)
  param.eeprompoll = beget16(&buff[10]);
  param.pagesize   = beget16(&buff[12]);
  param.eepromsize = beget16(&buff[14]);

  // 32 bits flashsize (big endian)
  param.flashsize = buff[16] * 0x01000000
    + buff[17] * 0x00010000
    + buff[18] * 0x00000100
    + buff[19];

}

void start_pmode() {
  spi_init();
  // following delays may not work on all targets...
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, HIGH);
  pinMode(SCK, OUTPUT);
  digitalWrite(SCK, LOW);
  delay(50);
  digitalWrite(RESET, LOW);
  delay(50);
  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
  spi_transaction(0xAC, 0x53, 0x00, 0x00);
  pmode = 1;
}

void end_pmode() {
  pinMode(MISO, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(RESET, INPUT);
  pmode = 0;
}

void universal() {
  int w;
  uint8_t ch;

  fill(4);
  ch = spi_transaction(buff[0], buff[1], buff[2], buff[3]);
  breply(ch);
}

void flash(uint8_t hilo, int addr, uint8_t data) {
  spi_transaction(0x40+8*hilo, 
  addr>>8 & 0xFF, 
  addr & 0xFF,
  data);
}
void commit(int addr) {
  if (PROG_FLICKER) prog_lamp(LOW);
  spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
  if (PROG_FLICKER) {
    delay(PTIME);
    prog_lamp(HIGH);
  }
}

//#define _current_page(x) (here & 0xFFFFE0)
int current_page(int addr) {
  if (param.pagesize == 32)  return here & 0xFFFFFFF0;
  if (param.pagesize == 64)  return here & 0xFFFFFFE0;
  if (param.pagesize == 128) return here & 0xFFFFFFC0;
  if (param.pagesize == 256) return here & 0xFFFFFF80;
  return here;
}


void write_flash(int length) {
  fill(length);
  if (CRC_EOP == getch()) {
    Serial.print((char) STK_INSYNC);
    Serial.print((char) write_flash_pages(length));
  } 
  else {
    error++;
    Serial.print((char) STK_NOSYNC);
  }
}

uint8_t write_flash_pages(int length) {
  int x = 0;
  int page = current_page(here);
  while (x < length) {
    if (page != current_page(here)) {
      commit(page);
      page = current_page(here);
    }
    flash(LOW, here, buff[x++]);
    flash(HIGH, here, buff[x++]);
    here++;
  }

  commit(page);

  return STK_OK;
}

#define EECHUNK (32)
uint8_t write_eeprom(int length) {
  // here is a word address, get the byte address
  int start = here * 2;
  int remaining = length;
  if (length > param.eepromsize) {
    error++;
    return STK_FAILED;
  }
  while (remaining > EECHUNK) {
    write_eeprom_chunk(start, EECHUNK);
    start += EECHUNK;
    remaining -= EECHUNK;
  }
  write_eeprom_chunk(start, remaining);
  return STK_OK;
}
// write (length) bytes, (start) is a byte address
uint8_t write_eeprom_chunk(int start, int length) {
  // this writes byte-by-byte,
  // page writing may be faster (4 bytes at a time)
  fill(length);
  prog_lamp(LOW);
  for (int x = 0; x < length; x++) {
    int addr = start+x;
    spi_transaction(0xC0, (addr>>8) & 0xFF, addr & 0xFF, buff[x]);
    delay(45);
  }
  prog_lamp(HIGH); 
  return STK_OK;
}

void program_page() {
  char result = (char) STK_FAILED;
  int length = 256 * getch();
  length += getch();
  char memtype = getch();
  // flash memory @here, (length) bytes
  if (memtype == 'F') {
    write_flash(length);
    return;
  }
  if (memtype == 'E') {
    result = (char)write_eeprom(length);
    if (CRC_EOP == getch()) {
      Serial.print((char) STK_INSYNC);
      Serial.print(result);
    } 
    else {
      error++;
      Serial.print((char) STK_NOSYNC);
    }
    return;
  }
  Serial.print((char)STK_FAILED);
  return;
}

uint8_t flash_read(uint8_t hilo, int addr) {
  return spi_transaction(0x20 + hilo * 8,
  (addr >> 8) & 0xFF,
  addr & 0xFF,
  0);
}

char flash_read_page(int length) {
  for (int x = 0; x < length; x+=2) {
    uint8_t low = flash_read(LOW, here);
    Serial.print((char) low);
    uint8_t high = flash_read(HIGH, here);
    Serial.print((char) high);
    here++;
  }
  return STK_OK;
}

char eeprom_read_page(int length) {
  // here again we have a word address
  int start = here * 2;
  for (int x = 0; x < length; x++) {
    int addr = start + x;
    uint8_t ee = spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
    Serial.print((char) ee);
  }
  return STK_OK;
}

void read_page() {
  char result = (char)STK_FAILED;
  int length = 256 * getch();
  length += getch();
  char memtype = getch();
  if (CRC_EOP != getch()) {
    error++;
    Serial.print((char) STK_NOSYNC);
    return;
  }
  Serial.print((char) STK_INSYNC);
  if (memtype == 'F') result = flash_read_page(length);
  if (memtype == 'E') result = eeprom_read_page(length);
  Serial.print(result);
  return;
}

void read_signature() {
  if (CRC_EOP != getch()) {
    error++;
    Serial.print((char) STK_NOSYNC);
    return;
  }
  Serial.print((char) STK_INSYNC);
  uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);
  Serial.print((char) high);
  uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
  Serial.print((char) middle);
  uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);
  Serial.print((char) low);
  Serial.print((char) STK_OK);
}
//////////////////////////////////////////
//////////////////////////////////////////


////////////////////////////////////
////////////////////////////////////
int avrisp() { 
  uint8_t data, low, high;
  uint8_t ch = getch();
  switch (ch) {
  case '0': // signon
    error = 0;
    empty_reply();
    break;
  case '1':
    if (getch() == CRC_EOP) {
      Serial.print((char) STK_INSYNC);
      Serial.print("AVR ISP");
      Serial.print((char) STK_OK);
    }
    break;
  case 'A':
    get_version(getch());
    break;
  case 'B':
    fill(20);
    set_parameters();
    empty_reply();
    break;
  case 'E': // extended parameters - ignore for now
    fill(5);
    empty_reply();
    break;

  case 'P':
    start_pmode();
    empty_reply();
    break;
  case 'U': // set address (word)
    here = getch();
    here += 256 * getch();
    empty_reply();
    break;

  case 0x60: //STK_PROG_FLASH
    low = getch();
    high = getch();
    empty_reply();
    break;
  case 0x61: //STK_PROG_DATA
    data = getch();
    empty_reply();
    break;

  case 0x64: //STK_PROG_PAGE
    program_page();
    break;

  case 0x74: //STK_READ_PAGE 't'
    read_page();    
    break;

  case 'V': //0x56
    universal();
    break;
  case 'Q': //0x51
    error=0;
    end_pmode();
    empty_reply();
    break;

  case 0x75: //STK_READ_SIGN 'u'
    read_signature();
    break;

    // expecting a command, not CRC_EOP
    // this is how we can get back in sync
  case CRC_EOP:
    error++;
    Serial.print((char) STK_NOSYNC);
    break;

    // anything else we will return STK_UNKNOWN
  default:
    error++;
    if (CRC_EOP == getch()) 
      Serial.print((char)STK_UNKNOWN);
    else
      Serial.print((char)STK_NOSYNC);
  }
}
/////////////////////////////////////
//End ISP Functions
/////////////////////////////////////
void software_Reset() { // Restarts program from beginning but does not reset the peripherals and registers
  asm volatile ("  jmp 0");  
}  

void detectBoard() {
    Serial.begin (115000);
    while (!Serial) ;  // for Leonardo, Micro etc.
    Serial.println ();
    Serial.println ("Atmega chip detector.");
 
    digitalWrite(SS, HIGH);  // ensure SS stays high for now
    SPI.begin ();
  
    // slow down SPI for benefit of slower processors like the Attiny
    SPI.setClockDivider(SPI_CLOCK_DIV64);

    pinMode (CLOCKOUT, OUTPUT);
  
    // set up Timer 1
    TCCR1A = _BV (COM1A0);  // toggle OC1A on Compare Match
    TCCR1B = _BV(WGM12) | _BV(CS10);   // CTC, no prescaling
    OCR1A =  0;       // output every cycle
  
    startProgramming ();
    getSignature ();
    getFuseBytes ();
  
    if (foundSig != -1)
      {
        readBootloader ();
      }
    readProgram ();
    digitalWrite(SS, HIGH); //Disables reset line to secondary processor...
    SPI.end();
    software_Reset();
}
