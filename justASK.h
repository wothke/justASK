#ifndef JUST_ASK_h
#define JUST_ASK_h

#include <Arduino.h>

/*
 Poor man's amplitude-shift keying (ASK) driver.

 The supported ASK hardware modules provide very low-level functionality meaning that the actual
 timing and (un)marshalling logic is all implemented within this driver. Interrupt handlers are
 used for the uniform timing: Respective interrupts are triggered according to the used bit rate
 and 8x more often on the receiver side (see PLL ramp sampling logic).

 The implementation is based on https://wireless.murata.com/media/products/apnotes/tr_swg05.pdf?ref=rfm.com
 (the PDF can be found in the "docs" folder) - specifically see "DK200A.ASM" example page 31ff. This is the
 same base that seems to have been used for the creation of the Arduino VirtualWire/RadioHead library.
 The above document nicely explains the various aspects of the implementation, e.g.
 12-bit encoding VS Manchester encoding (etc). And since this implementation here merely
 tries to port that original ASM implementation to C++, respective explanations are NOT
 repeated here (where applicable, references to the names used in the ASM program were added
 in the comments).

 Note: This implementation is targeted at a very limited scope "use case" of mine where senders
 (sensors) broadcast their data and messages are received by "relay stations" that then forward the
 data using other transmission technologies (and since respective headers are provided in the
 application specific message content anyway, no addressing information whatsoever is actually
 required/implemented on the ASK-message level).


 The provided APIs are separated into a pure sender and a pure receiver part - since there are no transceivers
 among the cheap modules from AliExpress that I've been using, and I am using either one or the other. Known 
 limitation: A transmitter and a receiver cannot be used in the same sketch.)

 The implementation is NOT "optimized" for size but rather for speed, i.e. respective lookup table based
 logic could be replaced if size ever proved to be limited - see CRC calculation and 6-bit symbol
 encoding/decoding (no issue on ESP8266 or ATmega128). Also see #ifdef MEM_BEFORE_SPEED


 Note: This implementation has only been tested with ATmega128, ATmega328P and "Wemos D1 mini pro"
 (ESP8266 based). The Sloeber 3.0 IDE has been used. The code might not work with old Arduino IDEs and
 no effort whatsoever has been made in that regard.


 In addition to the implementation from the original ASM example code, some VirtualWire/RadioHead
 specific handling was added for compatibility (see RADIOHEAD_COMPAT_MODE below). The RADIOHEAD_COMPAT_MODE
 provided here only handles the RadioHead specific message format, but not the RadioHead specific features
 that may or may not be attached to it (e.g. messages sent by RH_ASK can be received but they will always
 be treated as if they were mere broadcasts - even when they were not).

 If you are looking for a more portable/comprehensive implementation you might want to use RH_ASK
 from the "RadioHead" library instead - which wasn't an option for me due to licensing concerns (GPL
 is just a NO-GO).


 Message transmission format:

 The below information refers to the raw message data - before 4-bit nibbles are encoded into 6-bit symbols that go
 on the air.
 Each ASK message starts with a 4-byte "preamble" which is followed by the actual "message content". The
 "message content" consists of the actual "payload data" prefixed by a 1-byte "length" field (The "length"
 field measures the complete "message content", i.e. everything expect the "preamble".) and optionally
 additional headers (in RADIOHEAD_COMPAT_MODE). The "payload data" is then followed by a 2-byte FCS (CRC).


 Copyright (C) 2018 Juergen Wothke

 License: CC BY-SA-NC (separate commercial license available on request)
 */

// maximum length of user defined message content (with the high ASK error rates messages should better be short)
#ifndef MAX_ASK_PAYLOAD_LEN
#define MAX_ASK_PAYLOAD_LEN 50
#endif

// use this if you want RadioHead compatibility (rather than use the settings from the original RFM example)
#define RADIOHEAD_COMPAT_MODE


// PLL stuff
#define RX_PLL_RAMP_TOP 159			// RMPT: PLL ramp top value (modulo 0 to 159)
#define RX_PLL_RAMP_SWITCH 80		// RMPS: PLL ramp switch value
#define RX_PLL_RAMP_INC 20			// RMPI: PLL ramp increment value
#define RX_PLL_RAMP_INC_ADVANCE 29	// RMPA: PLL 5.625% advance increment value (20 + 9)
#define RX_PLL_RAMP_INC_RETARD 11	// RMPR: PLL 5.625% retard increment value (20 - 9)
#define RX_PLL_SAMPLES 8			// number of samples per bit


// --------- message format --------

	// 1: message starts with "preamble"
#define PREAMBLE_LEN 4		// tstrt: 3 bytes "training preamble" (1-0-1-.. pattern) + 1 "start symbol"
	// 2: then follows a header
#ifndef RADIOHEAD_COMPAT_MODE
		// just a "content length" field (total length - PREAMBLE_LEN)
#define MSG_HEADER_LEN 1
#else
		// "content length" field and 4 additional FROM/TO/MsgID/Flags fields
#define MSG_HEADER_LEN 5	// RadioHead specific header
#endif
    // 3: after which comes the variable length "payload"
    // 4: finally comes a 16-bit FCS (CRC)

#define PROTOCOL_WRAPPER_LEN (MSG_HEADER_LEN + 2) // add trailing CRC



/**
 * Abstract base class for ASK divers.
 */
class ASK_DriverBase {
public:
	typedef enum {
		ASKIdle= 0,  // driver is NOT using interrupt handler
		ASKRunning   // driver is using interrupt handler
	} ASKMode;

	ASK_DriverBase(uint16_t bps);
	virtual ~ASK_DriverBase() {}

	/**
	 * Completes construction.
	 *
	 * Allow for error handling without having to rely on exceptions.
	 */
	virtual bool init() = 0;

protected:
	virtual void doHandleInterrupt();

    bool isIdle();
    void setIdle();
    void setRunning();

protected:
    uint16_t _bps;					// bit rate
    volatile ASKMode _mode;

    friend void handleInterrupt();
};


// disable use of separate transmitter enable pin (i.e. the transmitter does not use a respective "enable" pin)
#define TX_ALWAYS_ENABLED 0xff

/**
 * Transmitter used to send ASK messages.
 */
class ASK_Transmitter : public ASK_DriverBase {
public:
	 ASK_Transmitter(uint16_t bps, uint8_t txPin, uint8_t txEnablePin= TX_ALWAYS_ENABLED);

	virtual bool init();

	/*
	 * Sends the specified message.
	 *
	 * @return false if transmitter is not ready or input is invalid
	 */
    bool sendMsg(const uint8_t *data, uint8_t len);
protected:
	virtual void doHandleInterrupt();
private:
    void write(bool value);	// write transmitter's data pin
    void disableTx();
    void enableTx();
    bool waitMsgSent(uint16_t timeoutMs);


protected:
	uint8_t _txPin;			// transmitter data pin
	uint8_t _txEnablePin;	// transmitter power enable pin

    uint8_t _txBuffer[(PREAMBLE_LEN + MAX_ASK_PAYLOAD_LEN + PROTOCOL_WRAPPER_LEN )*2];	// contains 6-bit symbols
    uint8_t _txBufferLen;		// number of 6-bit symbols in _txBuffer

    // source of the bit that is to be sent next:
    uint8_t _txIdx;			// buffer index
    uint8_t _txBitIdx;		// bit index
};


/**
 * Receiver used to receive ASK messages.
 */
class ASK_Receiver : public ASK_DriverBase {
public:
	ASK_Receiver(uint16_t bps, uint8_t rxPin);

	virtual bool init();

	/*
	 * Checks if a message has been received.
	 */
    bool hasReceivedMsg();

    /*
     * Waits for reception of a message.
     *
     * @param timeoutMs in milliseconds; 0 means indefinitely
     */
    bool waitReceive(uint16_t timeoutMs);

    /*
     * Polls for reception of message.
     *
     * @return true If data was available/returned (completes reception cycle).
     */
    bool receiveMsg(uint8_t *buf, uint8_t *len);

protected:
    virtual void doHandleInterrupt();

    bool read();			// read receiver's data pin
    bool validateMsg();

    // 6-bit to 4-bit decoding
    static void assertReverseLookupTable();
    static uint8_t decode(uint8_t symbol6bit);

protected:
    uint8_t _rxPin;						// receiver data pin

    uint8_t _rxBuffer[MAX_ASK_PAYLOAD_LEN + PROTOCOL_WRAPPER_LEN];		// received data (plain 8-bit; without PREAMBLE)
    volatile uint8_t _rxBufferLen;										// length of received data

    volatile uint8_t _rxID;				// RXID: integrate-and-dump (I&D) bit estimation
    volatile uint8_t _rxRamp;			// R2: PLL ramp buffer
    volatile bool _rxPrevSample;		// LRXSM: previous RX input sample
    volatile bool _rxSymbolFlag;		// RXSFLG: RX symbol flag (signaling end of PREABLE)

    volatile uint16_t _rxBit;			// RXBIT: 12-bit shift register collects "unprocessed" received bits representing 1 byte
    volatile uint8_t _rxBitCount;		// RMSBC: number "unprocessed" received bits
    volatile uint8_t _rxExpectedLen;

    // 2-step processing
    volatile bool   _rxDataAvailable;		// non-validated message data is available
    volatile bool   _rxValidMsgAvailable;	// validated message data is available
};

#endif
