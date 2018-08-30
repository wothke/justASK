#include <justASK.h>

#include <stdlib.h>     /* malloc, free, rand */

#define NDEBUG
#include <assert.h>

//#define MEM_BEFORE_SPEED				// use to reduce RAM use

#define AVAILABLE_TIMEOUT 2000			// timeout used to wait for transmitter to become available for next transmission

#ifndef RADIOHEAD_COMPAT_MODE
// original "tstrt" preamble from DK200A.ASM example: start symbol (4-bit representation): ..,8,3,11,
// i.e. "1000 0011 1011", i.e. as two 6-bit: 0x20, 0x3b
static uint8_t PREAMBLE[PREAMBLE_LEN<<1]= {0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x3b, 0x20};
#else
// for some reason RadioHead implementation uses a different "start symbol" (0x38: 111000 0x2c: 101100)
static uint8_t PREAMBLE[PREAMBLE_LEN<<1]= {0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x38, 0x2c};
#define BROADCAST_ADDR 0xff
#endif

static uint16_t START_SYMBOL= PREAMBLE[6] | (PREAMBLE[7] << 6); // 12-bit "start symbol"

// original "smbl" table from DK200A.ASM example:
#define REV_SYMBOL_LOOKUP_LEN 64
static uint8_t s6bitSymbols[]= { 13,14,19,21,22,25,26,28,35,37,38,41,42,44,50,52 };
static uint8_t *s4bitLookup= 0;

// --------------------------- interrupt handling -------------------------

// note: on ESP8266 the ICACHE_RAM_ATTR must be used for all the functions/methods that are called from interrupt
// handler or the device will crash with one of those cryptic error dumps..

// CAUTION: DO NOT statically allocate ASK_Receiver/ASK_Transmitter instances or you may still get respective crashes!

static ASK_DriverBase *sSingleton= 0;	// last one wins - implementation supports exactly one instance

// interrupt handler is called RX_PLL_SAMPLES times per sent/received bit
#ifdef ESP8266
// for some reason the code crashes when using instance vars with setter/getter (even with ICACHE_RAM_ATTR)
// better keep it as simple as possible for the ESP8266 crap..
static uint32_t sCyclesOffset;	// global

// wrapper used to reduce visibility of method
void ICACHE_RAM_ATTR handleInterrupt() {
	sSingleton->doHandleInterrupt();
}

void ICACHE_RAM_ATTR handler_esp8266() {
	timer0_write(ESP.getCycleCount() + sCyclesOffset);

	handleInterrupt();
}
#else
ISR(TIMER1_COMPA_vect) {		// Timer/Counter1 Compare Match A
	handleInterrupt();
}
#endif


#ifndef ESP8266
static const uint16_t PRESCALER_TABLE[]= {1, 8, 64, 256, 1024};

/* Calculates optimal "prescaler" & "latch" timer settings for the selected
* "bps" transmission speed:
* Timer would normally count system clock cycles but it can be slowed down
* using "pre-scaler" 1= *1, 2= *8, 3= *64, 4= *256, 5= *1024, e.g. with the
* highest pre-scaler (5), 1024 cycle-steps are counted (i.e. precision goes down
* and preferably the lowest "pre-scaler" should be used that still creates a
* "latch" that fits into 16-bits)
*/
static bool getTimerSettings(uint16_t bps, bool isReceiver, uint8_t *prescaler, uint16_t *latch) {
	for (uint8_t p=0; p < (sizeof(PRESCALER_TABLE) / sizeof( uint16_t)); p++) {
		uint32_t prescaledClock= F_CPU / ((uint32_t)PRESCALER_TABLE[p]);
		uint32_t tmpLatch= prescaledClock / (((uint32_t)bps) * (isReceiver ? RX_PLL_SAMPLES : 1));

		if ((tmpLatch > 0) && (tmpLatch <= 0xffff)) {
			(*prescaler)= p+1;		// numbering starts with 1
			(*latch)= tmpLatch;

			return true;			// found viable candidate
		}
	}
	return false;
}
#endif

static void setupInterruptHandler(uint16_t bps, bool isReceiver) {
	// note: timer interval of the transmitter is RX_PLL_SAMPLES times longer than the
	// one of a receiver (which need to perform the sampling for each received bit).

	// Timer timer0 is used on ESP8266 and TIMER1 on ATmega

#ifdef ESP8266		// Wemos D1 mini Pro (and possibly other ESP8266 based modules)

	// Example: For 1000 bps receiver, interrupt must fire every 20000 cycles on 160Mhz to trigger 8000 calls per second.
	// But millis() based measurements suggest that 7950 calls are triggered instead (probably due to some
	// delay added by the interrupt handling overhead) when using the "mathematically correct" sCyclesOffset
	// (actually messages will still be received even with this flaw):
	sCyclesOffset= (F_CPU / (isReceiver ? RX_PLL_SAMPLES : 1) / bps);

	// apply correction: in the above example a correction of -125 would seem plausible to
	// get from 7950 to 8000 calls (20000*7950/8000- 20000). But a somewhat higher correction
	// actually seems to create even better results (this might need to be changed for other ESP8266 models
	// and/or CPU frequencies):
	sCyclesOffset-= 129;

    noInterrupts();
    timer0_isr_init();
    timer0_attachInterrupt(handler_esp8266);
    timer0_write(ESP.getCycleCount() + sCyclesOffset);
    interrupts();

#else			// ATMega127/328P
    cli();          // disable global interrupts

	uint8_t prescaler; 	// 3 least significant bits used for TCCR1B (aka Clock Select Bits)
    uint16_t latch; 	// timer latch to use with the above prescaler

	if (getTimerSettings(bps, isReceiver, &prescaler, &latch)) {
		TCCR1A= 0;

		TCCR1B= (1 << WGM12); 	// turn on CTC mode (count up to OCR1A)
		TCCR1B|= prescaler;

		OCR1A= latch;			// set match register

		// enable CompareA Match interrupt
#ifdef TIMSK1					// ATmega328P
		TIMSK1|= (1 << OCIE1A);
#else							// ATmega128
		TIMSK|= (1 << OCIE1A);
#endif
	}
    sei();
#endif
}


// ----------------------- CRC/FCS (Frame Check Sequence) calculation -----------------------

// error rate in ASK transmissions is rather high and use of some kind of checksum is essential to verify message
// integrity.. in principle any kind of CRC implementation could be used (e.g. see
// http://reveng.sourceforge.net/crc-catalogue/16.htm) but to avoid duplication while
// retaining compatibility with existing VirtualWire/RadioHead libraries the same KERMIT
// CRC (also used in RFC 1331) is used here. Instead of the implementation used in RadioHead
// a pre-calculated table lookup approach is used by default (see https://tools.ietf.org/html/rfc1331#page-60).

#define	CRC_POLY	0x8408	// HDLC polynomial: x^16 + x^12 + x^5 + 1
#define CRC_SEED    0xffff  // initial FCS value
#define CRC_GOOD    0xf0b8  // good final FCS value

#define CRC_TAB_SIZE 256

static uint16_t *sFcstab= 0;

static void assertFcsTable() {
#ifndef MEM_BEFORE_SPEED
	if (!sFcstab) {
		// if RAM was limited this table might be put into PROGMEM..
		sFcstab= (uint16_t*)malloc(CRC_TAB_SIZE*2);

		for (uint16_t i=0; i<CRC_TAB_SIZE; i++) {
			uint32_t  v= i;

			for (uint16_t j=0; j<8; j++) {
				 v= v & 1 ? (v >> 1) ^ CRC_POLY : v >> 1;
			}
			sFcstab[i]= v & 0xffff;
		}
	}
#endif
}

#ifndef MEM_BEFORE_SPEED
static uint16_t crc_kermit (uint16_t crc, uint8_t data) {
	// note: in CCITT the shift direction would be different...
    return (crc >> 8) ^ sFcstab[(crc ^ (uint16_t)data) & 0x00ff]; // KERMIT
}
#else
// see https://www.microchip.com/webdoc/AVRLibcReferenceManual/group__util__crc_1ga1c1d3ad875310cbc58000e24d981ad20.html
static uint16_t crc_kermit(uint16_t crc, uint8_t data) {
    data ^= (crc & 0xff);
    data ^= data << 4;

    return ((((uint16_t)data << 8) | (crc>>8)) ^ (uint8_t)(data >> 4)
            ^ ((uint16_t)data << 3));
}
#endif
// ------------------------------------ utilities ----------------------------------------

static void writeMsgByte(uint8_t **bufPtr, uint16_t *crc, uint8_t data) {
	// encode the two "data" nibbles as two 6-bit symbols
	(*bufPtr)[0]= s6bitSymbols[data >> 4];
	(*bufPtr)[1]= s6bitSymbols[data & 0xf];

	(*bufPtr)+= 2;								// advance buffer position for next byte

	if (crc != 0) {
		(*crc)= crc_kermit(*crc, data);	// update CRC
	}
}

ASK_DriverBase::ASK_DriverBase(uint16_t bps)  {
	assert(bps != 0);

	assertFcsTable();

	_bps= bps;

	sSingleton= this;
}

bool ICACHE_RAM_ATTR ASK_DriverBase::isIdle() {
	return (_mode == ASKIdle);
}
void ICACHE_RAM_ATTR ASK_DriverBase::setIdle() {
	_mode= ASKIdle;
}
void ICACHE_RAM_ATTR ASK_DriverBase::setRunning() {
	_mode= ASKRunning;
}
void ICACHE_RAM_ATTR ASK_DriverBase::doHandleInterrupt() {
}


// ---------------------------- RECEIVER -------------------------------------

ASK_Receiver::ASK_Receiver(uint16_t bps, uint8_t rxPin) : ASK_DriverBase(bps),
		_rxPin(rxPin),  _rxBufferLen(0), _rxID(0), _rxRamp(0), _rxSymbolFlag(false), _rxBit(0), _rxBitCount(0),
		_rxDataAvailable(false), _rxValidMsgAvailable(false) {

	pinMode(_rxPin, INPUT);
}

bool ASK_Receiver::init() {
	assertReverseLookupTable();

	setRunning();					// ready to receive
	setupInterruptHandler(_bps, true);

	return true;
}

bool ASK_Receiver::validateMsg() {
	uint16_t fcs= CRC_SEED;
	for (uint8_t i= 0; i < _rxBufferLen; i++) {
		fcs= crc_kermit(fcs, _rxBuffer[i]);
	}
	if (fcs == CRC_GOOD) {
		// note: the default implementation does not use any additional headers here
		// since those are handled in higher level protocols anyway (in my use-case)

#ifdef RADIOHEAD_COMPAT_MODE
    // if anybody cared he could get the 4 RadioHead specific headers: TO/FROM/MsgID/Flags
    // here:

    //	to    = _rxBuffer[1];
    //	from  = _rxBuffer[2];
    //  msgId = _rxBuffer[3];
    //  flags = _rxBuffer[4];
#endif
		return true;
	}
	// note: even in "favorable" test conditions the error rate is rather high (at 80cm distance
	// and 1000 bps my superheterodyne receiver still got corrupted messages about 20% of the time..)
	return false;
}

bool ASK_Receiver::hasReceivedMsg() {
	if (_rxDataAvailable) {
		_rxValidMsgAvailable= validateMsg();

		// get ready for next message
		_rxDataAvailable= false;
		_rxSymbolFlag= false;
		if (!_rxValidMsgAvailable) { setRunning(); } // otherwise wait until message has been claimed
	}
	return _rxValidMsgAvailable;
}

bool ASK_Receiver::waitReceive(uint16_t timeoutMs) {
	uint32_t t= millis();
	while (!timeoutMs || ((millis() - t) < timeoutMs)) {
		yield();		// make sure this cannot crash the ESP8266!
		if (hasReceivedMsg()) {
			return true;
		}
	}
	return false;
}

void ASK_Receiver::assertReverseLookupTable() {
	if (!s4bitLookup) {
		s4bitLookup= (uint8_t*)calloc(REV_SYMBOL_LOOKUP_LEN, 1);

		for (uint16_t nibble= 0; nibble < sizeof(s6bitSymbols); nibble++) {
			uint8_t symbol= s6bitSymbols[nibble];
			s4bitLookup[symbol]= nibble;
		}
	}
}

uint8_t ICACHE_RAM_ATTR ASK_Receiver::decode(uint8_t symbol6bit) {
	return s4bitLookup[symbol6bit & 0b111111];	// recover original 4-bit nibble from 6-bit symbol
}

bool ASK_Receiver::receiveMsg(uint8_t *buf, uint8_t *len) {
	assert(buf != 0);
	assert(len != 0);

	bool success= false;

	if (hasReceivedMsg()) {
		uint8_t messageLen= _rxBufferLen - PROTOCOL_WRAPPER_LEN;
		if (*len > messageLen) {
			*len= messageLen;

			memcpy(buf, _rxBuffer + MSG_HEADER_LEN, *len);
			success = true;
		}
		_rxValidMsgAvailable= false;	// clear for next reception
		setRunning();
	}
	return success;
}

bool ICACHE_RAM_ATTR ASK_Receiver::read() {
	return digitalRead(_rxPin);
}

// low level PLL-based message reception logic (interrupt driven) - see "p11" in original DK200A.ASM implementation
void ICACHE_RAM_ATTR ASK_Receiver::doHandleInterrupt() {
	if (isIdle()) { return; }	// do not receive new message while previous message has not been claimed

	bool rxsmp= read();

	_rxID += (rxsmp ? 1 : 0);	// count "set" samples

	if (rxsmp == _rxPrevSample) {
		_rxRamp+= RX_PLL_RAMP_INC;
	} else {
		_rxPrevSample= rxsmp;

		if (_rxRamp < RX_PLL_RAMP_SWITCH) {
			_rxRamp+= RX_PLL_RAMP_INC_RETARD;
		} else {
			_rxRamp+= RX_PLL_RAMP_INC_ADVANCE;
		}
	}


	if (_rxRamp >= RX_PLL_RAMP_TOP) {	// see "p116" -> wrap
		_rxBit >>= 1;
		_rxBit|= (_rxID >= 5) ? 0b100000000000: 0x0;	// store received as MS 12th bit
		_rxID= 0; 										// restart for next bit

		_rxRamp -= RX_PLL_RAMP_TOP;		// todo: would seem more logical to wrap at RX_PLL_RAMP_TOP+1! (bug in the ASM implementation?)

		if (!_rxSymbolFlag) {
			// still in the PREABLE..
			if (_rxBit == START_SYMBOL) {
				// from now on it is the actual message
				_rxSymbolFlag= true;
				_rxBitCount= _rxBufferLen= 0;
			}
		} else {
			// processing actual message bits
			_rxBitCount+= 1;

			if (_rxBitCount == 12) {	// enough data to decode one byte
				uint8_t data= ((decode(_rxBit & 0x3f)) << 4) | decode(_rxBit >> 6);

				// message starts with "length" field
				if (!_rxBufferLen) { _rxExpectedLen= data;	}

				if (_rxBufferLen >= (MAX_ASK_PAYLOAD_LEN + PROTOCOL_WRAPPER_LEN)) {	// guard against buffer overflow
					_rxSymbolFlag= false;	// abort, start from scratch
				} else {
					_rxBuffer[_rxBufferLen]= data;
					_rxBufferLen+= 1;

					if (_rxBufferLen == _rxExpectedLen) {
						setIdle();	// block reception of new messages until this one has been claimed
						_rxDataAvailable= true;	// signal that unverified message is available
					}
				}
				_rxBitCount= 0;
			}
		}
	}
}

// ---------------------------- TRANSMITTER -------------------------------------

ASK_Transmitter::ASK_Transmitter(uint16_t bps, uint8_t txPin, uint8_t txEnablePin) : ASK_DriverBase(bps),
		_txPin(txPin),
		_txEnablePin(txEnablePin) {

	pinMode(_txPin, OUTPUT);
	if (_txEnablePin != TX_ALWAYS_ENABLED) { pinMode(_txEnablePin, OUTPUT); }

	memcpy(_txBuffer, PREAMBLE, sizeof(PREAMBLE));
}

void ICACHE_RAM_ATTR ASK_Transmitter::disableTx() {
	if (_txEnablePin != TX_ALWAYS_ENABLED) {
		digitalWrite(_txEnablePin, LOW);
		write(LOW);
	}
}

void ICACHE_RAM_ATTR ASK_Transmitter::enableTx() {
	if (_txEnablePin != TX_ALWAYS_ENABLED) {
		digitalWrite(_txEnablePin, HIGH);
	}
}

bool ASK_Transmitter::init() {
	disableTx();
	setIdle();

    setupInterruptHandler(_bps, false);
    return true;
}

void ICACHE_RAM_ATTR ASK_Transmitter::write(bool value) {
    digitalWrite(_txPin, value);
}

bool ASK_Transmitter::waitMsgSent(uint16_t timeout) {
	uint32_t t= millis();
	while (!timeout || ((millis() - t) < timeout)) {
		yield();		// make sure this cannot crash the ESP8266!
		if (isIdle()) { // wait for end of previous transmission
			return true;
		}
	}
	return false;
}

bool ASK_Transmitter::sendMsg(const uint8_t *data, uint8_t len) {
	if ((len > MAX_ASK_PAYLOAD_LEN) || !waitMsgSent(AVAILABLE_TIMEOUT)) { return false; }

	uint8_t *msgPos, *msgStart;
	msgPos= msgStart= _txBuffer + (PREAMBLE_LEN<<1);

	uint16_t fcs= CRC_SEED;
	writeMsgByte(&msgPos, &fcs, len + PROTOCOL_WRAPPER_LEN);	// use same calculation as RadioHead for compatibility

#ifdef RADIOHEAD_COMPAT_MODE
	// whatever RadioHead might be using these field for ISN'T supported in this implementation
	// (it is just a placeholder)
	writeMsgByte(&msgPos, &fcs, BROADCAST_ADDR);	// TO
	writeMsgByte(&msgPos, &fcs, BROADCAST_ADDR);	// FROM
	writeMsgByte(&msgPos, &fcs, 0);				// header ID
	writeMsgByte(&msgPos, &fcs, 0);				// header flags
#endif

	// actual "message payload"
	for (uint8_t i= 0; i < len; i++) {
		writeMsgByte(&msgPos, &fcs, data[i]);
	}

	// for compatibility with RadioHead  the same CRC approach is used here
	fcs= ~fcs;
	writeMsgByte(&msgPos, 0, fcs & 0xff);
	writeMsgByte(&msgPos, 0, fcs >> 8);

	_txBufferLen= (msgPos-msgStart) + (PREAMBLE_LEN<<1);

	// trigger interrupt based transmission
	_txIdx= _txBitIdx= 0;
	enableTx();
	setRunning();

	return true;
}

void ICACHE_RAM_ATTR ASK_Transmitter::doHandleInterrupt() {
	if (isIdle()) { return; }	// nothing to do

	// process next bit
	if (_txIdx < _txBufferLen) {
		write((_txBuffer[_txIdx] & (1 << _txBitIdx)) != 0);

		_txBitIdx+= 1;

		if (_txBitIdx == 6) {		// done with this symbol
			_txBitIdx= 0;
			_txIdx++;
		}
	} else {
		disableTx();	// done with this message
		setIdle();
	}
}

