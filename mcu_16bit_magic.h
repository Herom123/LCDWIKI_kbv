#ifndef _mcu_16bit_magic_
#define _mcu_16bit_magic_

#define DELAY7        \
  asm volatile(       \
    "rjmp .+0" "\n\t" \
    "rjmp .+0" "\n\t" \
    "rjmp .+0" "\n\t" \
    "nop"      "\n"   \
    ::);

#if defined(ESP32)
  #define GPIO_MASK32(pin) ( 1UL  << (( pin > 31 )? (pin - 32) : pin) )
  #define GPIO_MASK64(pin) ( 1ULL << pin )

  #define GPIO_SLCOUTS(pin) (( pin > 31 )? GPIO.out1_w1ts.val:GPIO.out_w1ts)
  #define GPIO_SLCOUTC(pin) (( pin > 31 )? GPIO.out1_w1tc.val:GPIO.out_w1tc)

  #define BUS_GPIO_MASK set_mask16d(0xFF,0xFF)

  #define  CS_GPIO_MASK GPIO_MASK32( TFT_CS)
  #define  RS_GPIO_MASK GPIO_MASK32( TFT_RS)
  #define  WR_GPIO_MASK GPIO_MASK32( TFT_WR)
  #define  RD_GPIO_MASK GPIO_MASK32( TFT_RD)
  #define RST_GPIO_MASK GPIO_MASK32(TFT_RST)

  #define INIT_TFT_DATA_BUS           \
  for (int32_t c = 0; c<256; c++)     \
  {                                   \
  dbv_maskl[c] = 0;                   \
  dbv_maskh[c] = 0;                   \
  if ( c & 0x01 ) {/*00000001*/       \
    dbv_maskl[c]  = (1UL << TFT_D0);  \
    dbv_maskh[c]  = (1UL << TFT_D8);} \
  if ( c & 0x02 ) {/*00000010*/       \
    dbv_maskl[c] |= (1UL << TFT_D1);  \
    dbv_maskh[c] |= (1UL << TFT_D9);} \
  if ( c & 0x04 ) {/*00000100*/       \
    dbv_maskl[c] |= (1UL << TFT_D2);  \
    dbv_maskh[c] |= (1UL << TFT_D10);}\
  if ( c & 0x08 ) {/*00001000*/       \
    dbv_maskl[c] |= (1UL << TFT_D3);  \
    dbv_maskh[c] |= (1UL << TFT_D11);}\
  if ( c & 0x10 ) {/*00010000*/       \
    dbv_maskl[c] |= (1UL << TFT_D4);  \
    dbv_maskh[c] |= (1UL << TFT_D12);}\
  if ( c & 0x20 ) {/*00100000*/       \
    dbv_maskl[c] |= (1UL << TFT_D5);  \
    dbv_maskh[c] |= (1UL << TFT_D13);}\
  if ( c & 0x40 ) {/*01000000*/       \
    dbv_maskl[c] |= (1UL << TFT_D6);  \
    dbv_maskh[c] |= (1UL << TFT_D14);}\
  if ( c & 0x80 ) {/*10000000*/       \
    dbv_maskl[c] |= (1UL << TFT_D7);  \
    dbv_maskh[c] |= (1UL << TFT_D15);}\
  }
  
  #define SET_PIN_DIRECTION(bitmask, mode_)    \
  io_conf.intr_type    = GPIO_INTR_DISABLE;    \
  io_conf.mode         = mode_;                \
  io_conf.pin_bit_mask = bitmask;              \
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;\
  io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;  \
  gpio_config(&io_conf)

  #define SET_BUS_DIRECTION(mode) SET_PIN_DIRECTION(0xFFFFFFFFFFFFFFFF&BUS_GPIO_MASK, mode)
  #define SET_CP_DIRECTION(mode)  SET_PIN_DIRECTION((GPIO_MASK64(TFT_CS)|GPIO_MASK64(TFT_RS)|GPIO_MASK64(TFT_RST)|GPIO_MASK64(TFT_WR)), mode)

  #define set_mask8_low(C)  dbv_maskl[C]
  #define set_mask8_high(C) dbv_maskh[C] 

  #define set_mask16d(C,D) ((dbv_maskl[C])|dbv_maskh[D])
  #define set_mask16(C)    (dbv_maskl[C&0xFF]|dbv_maskh[C>>8])
  
  #define write8(data)  WR_ACTIVE;GPIO.out_w1tc = BUS_GPIO_MASK;GPIO.out_w1ts = set_mask8_low(data);WR_IDLE
  #define write_16(data) WR_ACTIVE;GPIO.out_w1tc = BUS_GPIO_MASK;GPIO.out_w1ts =    set_mask16(data);WR_IDLE 
//  #define write16(d) { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
  #define read_16(dst) {                                    \
    RD_ACTIVE;                                              \
    uint32_t rdbusgpioinputstemp_10354;                     \
    rdbusgpioinputstemp_10354 = gpio_input_get();           \
    rdbusgpioinputstemp_10354 = gpio_input_get();           \
    rdbusgpioinputstemp_10354 = gpio_input_get();           \
    RD_IDLE;                                                \
    dst  = (((rdbusgpioinputstemp_10354>>TFT_D0)&1) << 0);  \
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D1)&1) << 1);  \
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D2)&1) << 2);  \
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D3)&1) << 3);  \
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D4)&1) << 4);  \
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D5)&1) << 5);  \
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D6)&1) << 6);  \
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D7)&1) << 7);  \
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D8)&1) << 8);  \
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D9)&1) << 9);  \
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D10)&1) << 10);\
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D11)&1) << 11);\
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D12)&1) << 12);\
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D13)&1) << 13);\
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D14)&1) << 14);\
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D15)&1) << 15);\
  }
  #define read8(dst) {                                    \
    RD_ACTIVE;                                            \
    uint32_t rdbusgpioinputstemp_10354;                   \
    rdbusgpioinputstemp_10354 = gpio_input_get();         \
    rdbusgpioinputstemp_10354 = gpio_input_get();         \
    rdbusgpioinputstemp_10354 = gpio_input_get();         \
    RD_IDLE;                                              \
    dst  = (((rdbusgpioinputstemp_10354>>TFT_D0)&1) << 0);\
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D1)&1) << 1);\
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D2)&1) << 2);\
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D3)&1) << 3);\
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D4)&1) << 4);\
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D5)&1) << 5);\
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D6)&1) << 6);\
    dst |= (((rdbusgpioinputstemp_10354>>TFT_D7)&1) << 7);\
  }
//  #define read16(dst) { uint8_t hi; read8(hi); read8(dst); dst |= (hi << 8); }
  #define setWriteDir() { SET_BUS_DIRECTION(GPIO_MODE_OUTPUT); }
  #define setReadDir()  { SET_BUS_DIRECTION( GPIO_MODE_INPUT); }

  unsigned long dbv_maskh[256];
  unsigned long dbv_maskl[256];
  
  gpio_config_t io_conf;

#elif defined(__AVR_ATmega2560__)
   #ifdef USE_ADAFRUIT_SHIELD_PIN
	 #define RD_PORT PORTL
  	 #define WR_PORT PORTG
     #define CD_PORT PORTD
     #define CS_PORT PORTG
	 #define RD_MASK B01000000
  	 #define WR_MASK B00000010
  	 #define CD_MASK B10000000
  	 #define CS_MASK B00000001
	 
	 #define write_16(x)    { PORTA = (x) >> 8; PORTC = x; WR_STROBE;}
	 #define write8(x)     { PORTC = x; WR_STROBE;}
	 #define read_16(dst)   { RD_STROBE;dst = (PINA<<8) | (PINC); RD_IDLE;}
	 #define read8(dst)    { read16(dst);dst &= 0xFFFF;}
	 #define setWriteDir() { DDRC = 0xFF; DDRA = 0xFF; }
	 #define setReadDir()  { DDRC = 0x00; DDRA = 0x00; }
	#else
	 #define write_16(x)   { PORTA = (x) >> 8; PORTC = x; WR_STROBE;}
	 #define write8(x)    { PORTC = x; WR_STROBE;}
	 #define read_16(dst)   { RD_STROBE;dst = (PINA<<8) | (PINC); RD_IDLE;}
	 #define read8(dst)    { read_16(dst);dst &= 0xFFFF;}
	 #define setWriteDir() { DDRC = 0xFF; DDRA = 0xFF; }
	 #define setReadDir()  { DDRC = 0x00; DDRA = 0x00; }
	
	#endif	
#elif defined(__SAM3X8E__)
  #ifdef USE_ADAFRUIT_SHIELD_PIN
	 #define RD_PORT PIOA
	 #define WR_PORT PIOC
	 #define CD_PORT PIOC
	 #define CS_PORT PIOC
	 #define RD_MASK 0x00100000
	 #define WR_MASK 0x00000080
	 #define CD_MASK 0x00000040
	 #define CS_MASK 0x00000100
	// configure macros for data bus 
	// 
	#define AMASK         ((1<<7)|(3<<14))          //PA7, PA14-PA15
	#define BMASK         (1<<26)                   //PB26
	#define CMASK         (31<<1)                   //PC1-PC5
	#define DMASK         ((15<<0)|(1<<6)|(3<<9))   //PD0-PD3, PD6, PD9-PD10
	
	#define write_16(x)   { PIOA->PIO_CODR = AMASK; PIOB->PIO_CODR = BMASK; PIOC->PIO_CODR = CMASK; PIOD->PIO_CODR = DMASK; \
							PIOA->PIO_SODR = (((x)&(1<<6))<<1)|(((x)&(3<<9))<<5); \
							PIOB->PIO_SODR = (((x)&(1<<8))<<18); \
							PIOC->PIO_SODR = (((x)&(1<<0))<<5); \
							PIOC->PIO_SODR = (((x)&(1<<1))<<3); \
							PIOC->PIO_SODR = (((x)&(1<<2))<<1); \
							PIOC->PIO_SODR = (((x)&(1<<3))>>1); \
							PIOC->PIO_SODR = (((x)&(1<<4))>>3); \
							PIOD->PIO_SODR = (((x)&(1<<7))<<2)|(((x)&(1<<5))<<5)|(((x)&(15<<11))>>11)|(((x)&(1<<15))>>9); WR_STROBE; \
						  }
	
    #define write8(x)     { write_16(x & 0xFF); WR_STROBE;}
    #define read_16(dst)  { RD_STROBE;dst=(0\
							|((PIOC->PIO_PDSR & (1<<5))>>5)\
							|((PIOC->PIO_PDSR & (1<<4))>>3)\
							|((PIOC->PIO_PDSR & (1<<3))>>1)\
							|((PIOC->PIO_PDSR & (1<<2))<<1)\
							|((PIOC->PIO_PDSR & (1<<1))<<3)\
							|((PIOD->PIO_PDSR & (1<<10))>>5)\
							|((PIOA->PIO_PDSR & (1<<7))>>1)\
							|((PIOD->PIO_PDSR & (1<<9))>>2)\
							|((PIOB->PIO_PDSR & (1<<26))>>18)\
							|((PIOA->PIO_PDSR & (3<<14))>>5)\
							|((PIOD->PIO_PDSR & (15<<0))<<11)\
							|((PIOD->PIO_PDSR & (1<<6))<<9))\
    	                 }
	#define read8(dst)      {read_16(dst);dst &= 0xFFFF;}
	#define setWriteDir() {\
							PIOA->PIO_OER = AMASK; PIOA->PIO_PER = AMASK; \
							PIOB->PIO_OER = BMASK; PIOB->PIO_PER = BMASK; \
							PIOC->PIO_OER = CMASK; PIOC->PIO_PER = CMASK; \
							PIOD->PIO_OER = DMASK; PIOD->PIO_PER = DMASK; \
						  }
	#define setReadDir()  { \
							PMC->PMC_PCER0 = (1 << ID_PIOA)|(1 << ID_PIOB)|(1 << ID_PIOC)|(1 << ID_PIOD); \
							PIOA->PIO_ODR = AMASK; \
							PIOB->PIO_ODR = BMASK; \
							PIOC->PIO_ODR = CMASK; \
							PIOD->PIO_ODR = DMASK; \
						  }
   #define RD_ACTIVE  RD_PORT->PIO_CODR = RD_MASK
   #define RD_IDLE    RD_PORT->PIO_SODR = RD_MASK
   #define WR_ACTIVE  WR_PORT->PIO_CODR = WR_MASK
   #define WR_IDLE    WR_PORT->PIO_SODR = WR_MASK
   #define CD_COMMAND CD_PORT->PIO_CODR = CD_MASK
   #define CD_DATA    CD_PORT->PIO_SODR = CD_MASK
   #define CS_ACTIVE  CS_PORT->PIO_CODR = CS_MASK
   #define CS_IDLE    CS_PORT->PIO_SODR = CS_MASK
    #else
		#define write_16(x)   { PIOA->PIO_CODR = AMASK; PIOB->PIO_CODR = BMASK; PIOC->PIO_CODR = CMASK; PIOD->PIO_CODR = DMASK; \
							PIOA->PIO_SODR = (((x)&(1<<6))<<1)|(((x)&(3<<9))<<5); \
							PIOB->PIO_SODR = (((x)&(1<<8))<<18); \
							PIOC->PIO_SODR = (((x)&(1<<0))<<5); \
							PIOC->PIO_SODR = (((x)&(1<<1))<<3); \
							PIOC->PIO_SODR = (((x)&(1<<2))<<1); \
							PIOC->PIO_SODR = (((x)&(1<<3))>>1); \
							PIOC->PIO_SODR = (((x)&(1<<4))>>3); \
							PIOD->PIO_SODR = (((x)&(1<<7))<<2)|(((x)&(1<<5))<<5)|(((x)&(15<<11))>>11)|(((x)&(1<<15))>>9); WR_STROBE; \
						  }
	
    #define write8(x)     { write_16(x & 0xFF); WR_STROBE;}
    #define read_16(dst)  { RD_STROBE;dst=(0\
							|((PIOC->PIO_PDSR & (1<<5))>>5)\
							|((PIOC->PIO_PDSR & (1<<4))>>3)\
							|((PIOC->PIO_PDSR & (1<<3))>>1)\
							|((PIOC->PIO_PDSR & (1<<2))<<1)\
							|((PIOC->PIO_PDSR & (1<<1))<<3)\
							|((PIOD->PIO_PDSR & (1<<10))>>5)\
							|((PIOA->PIO_PDSR & (1<<7))>>1)\
							|((PIOD->PIO_PDSR & (1<<9))>>2)\
							|((PIOB->PIO_PDSR & (1<<26))>>18)\
							|((PIOA->PIO_PDSR & (3<<14))>>5)\
							|((PIOD->PIO_PDSR & (15<<0))<<11)\
							|((PIOD->PIO_PDSR & (1<<6))<<9))\
    	                 }
	#define read8(dst)      {read_16(dst);dst &= 0xFFFF;}
	#define setWriteDir() {\
							PIOA->PIO_OER = AMASK; PIOA->PIO_PER = AMASK; \
							PIOB->PIO_OER = BMASK; PIOB->PIO_PER = BMASK; \
							PIOC->PIO_OER = CMASK; PIOC->PIO_PER = CMASK; \
							PIOD->PIO_OER = DMASK; PIOD->PIO_PER = DMASK; \
						  }
	#define setReadDir()  { \
							PMC->PMC_PCER0 = (1 << ID_PIOA)|(1 << ID_PIOB)|(1 << ID_PIOC)|(1 << ID_PIOD); \
							PIOA->PIO_ODR = AMASK; \
							PIOB->PIO_ODR = BMASK; \
							PIOC->PIO_ODR = CMASK; \
							PIOD->PIO_ODR = DMASK; \
						  }
	#define RD_ACTIVE	rdPort->PIO_CODR = rdPinSet		//PIO_Clear(rdPort, rdPinSet)
    #define RD_IDLE		rdPort->PIO_SODR = rdPinSet		//PIO_Set(rdPort, rdPinSet)	
    #define WR_ACTIVE	wrPort->PIO_CODR = wrPinSet		//PIO_Clear(wrPort, wrPinSet)
    #define WR_IDLE		wrPort->PIO_SODR = wrPinSet		//PIO_Set(wrPort, wrPinSet)
    #define CD_COMMAND	cdPort->PIO_CODR = cdPinSet		//PIO_Clear(cdPort, cdPinSet)
    #define CD_DATA		cdPort->PIO_SODR = cdPinSet		//PIO_Set(cdPort, cdPinSet)
    #define CS_ACTIVE	csPort->PIO_CODR = csPinSet		//PIO_Clear(csPort, csPinSet)
    #define CS_IDLE		csPort->PIO_SODR = csPinSet
	#endif	
#else
#ifndef ESP32
 #error "Board type unsupported / not recognized"
#endif
#endif

#if !defined(__SAM3X8E__)
// Stuff common to all Arduino AVR board types:
#ifdef ESP32

#define WR_ACTIVE  GPIO.out_w1tc = WR_GPIO_MASK
#define WR_IDLE    GPIO.out_w1ts = WR_GPIO_MASK
#define RD_ACTIVE  GPIO.out_w1tc = RD_GPIO_MASK
#define RD_IDLE    GPIO.out_w1ts = RD_GPIO_MASK
#define CD_COMMAND GPIO_SLCOUTC(TFT_RS)  = RS_GPIO_MASK
#define CD_DATA    GPIO_SLCOUTS(TFT_RS)  = RS_GPIO_MASK
#define CS_ACTIVE  GPIO_SLCOUTC(TFT_CS)  = CS_GPIO_MASK
#define CS_IDLE    GPIO_SLCOUTS(TFT_CS)  = CS_GPIO_MASK
#define RST_ACTIVE GPIO_SLCOUTC(TFT_RST) = RST_GPIO_MASK
#define RST_IDLE   GPIO_SLCOUTS(TFT_RST) = RST_GPIO_MASK

#elif USE_ADAFRUIT_SHIELD_PIN

 // Control signals are ACTIVE LOW (idle is HIGH)
 // Command/Data: LOW = command, HIGH = data
 // These are single-instruction operations and always inline
 #define RD_ACTIVE  RD_PORT &= ~RD_MASK
 #define RD_IDLE    RD_PORT |=  RD_MASK
 #define WR_ACTIVE  WR_PORT &= ~WR_MASK
 #define WR_IDLE    WR_PORT |=  WR_MASK
 #define CD_COMMAND CD_PORT &= ~CD_MASK
 #define CD_DATA    CD_PORT |=  CD_MASK
 #define CS_ACTIVE  CS_PORT &= ~CS_MASK
 #define CS_IDLE    CS_PORT |=  CS_MASK
#else // Breakout board

 // When using the TFT breakout board, control pins are configurable.
 #define RD_ACTIVE  *rdPort &=  rdPinUnset
 #define RD_IDLE    *rdPort |=  rdPinSet
 #define WR_ACTIVE  *wrPort &=  wrPinUnset
 #define WR_IDLE    *wrPort |=  wrPinSet
 #define CD_COMMAND *cdPort &=  cdPinUnset
 #define CD_DATA    *cdPort |=  cdPinSet
 #define CS_ACTIVE  *csPort &=  csPinUnset
 #define CS_IDLE    *csPort |=  csPinSet

#endif
#endif

// Data write strobe, ~2 instructions and always inline
#define WR_STROBE { WR_ACTIVE; WR_IDLE; }
#define RD_STROBE {RD_IDLE; RD_ACTIVE;RD_ACTIVE;RD_ACTIVE;}  
#define write16(x) { write_16(x); }
#define read16(dst) { read_16(dst); }
#define writeCmd8(x){ CD_COMMAND; write8(x); CD_DATA;  }
#define writeData8(x){  write8(x); }
#define writeCmd16(x){ CD_COMMAND; write16(x); CD_DATA; }
#define writeData16(x){ write16(x); }



// These higher-level operations are usually functionalized,
// except on Mega where's there's gobs and gobs of program space.

// Set value of TFT register: 8-bit address, 8-bit value
#define writeCmdData8(a, d) { CD_COMMAND; write8(a); CD_DATA; write8(d); }

// Set value of TFT register: 16-bit address, 16-bit value
// See notes at top about macro expansion, hence hi & lo temp vars
#define writeCmdData16(a, d) { \
  CD_COMMAND; write16(a); \
  CD_DATA   ; write16(d);  }

// Set value of 2 TFT registers: Two 8-bit addresses (hi & lo), 16-bit value
//#define writeRegisterPairInline(aH, aL, d) { \
//  uint8_t hi = (d) >> 8, lo = (d); \
//  CD_COMMAND; write8(aH); CD_DATA; write8(hi); \
//  CD_COMMAND; write8(aL); CD_DATA; write8(lo); }

#endif // _mcu_16bit_magic_
