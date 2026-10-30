// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
// **********************************************************************************
// Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will
// be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public
// License for more details.
//
// Licence can be viewed at
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************

// **********************************************************************************
// Converted to AVR environment by Zulkar Nayem
// The AVR environment modified for STM32 compatibility by Daniel Mako
// **********************************************************************************

#include <stdint.h>
#include "RFM69registers.h"
#include "RFM69.h"

#define RF69_MODE_UNKNOWN     0xFF  // internal: forces the next setMode() to write the radio
#define RF69_SYNCVALUE1       0x2D  // written by rfm69_init(), used by rfm69_isConfigured()

// Global Variables
volatile uint8_t DATA[RF69_MAX_DATA_LEN + 1]; // RX/TX payload buffer, including end of string NULL char
volatile uint8_t DATALEN;
volatile uint8_t SENDERID;
volatile uint8_t TARGETID;                 // should match _address
volatile uint8_t PAYLOADLEN;
volatile uint8_t ACK_REQUESTED;
volatile uint8_t ACK_RECEIVED; // should be polled immediately after sending a packet with ACK request
volatile int16_t RSSI; // most accurate RSSI during reception (closest to the reception)
volatile uint8_t mode = RF69_MODE_STANDBY; // should be protected?
volatile uint8_t inISR = 0;
volatile uint8_t last_tx_RSSI = 0; // 0 means no valid value
volatile int16_t last_received_ack_RSSI = 0;
uint8_t isRFM69HW = 1;    // if RFM69HW model matches high power enable possible
uint8_t address;                           //nodeID
uint8_t powerLevel = 31;
uint8_t maxPowerLevel = 31;	// user limit of the power level, see setMaxPowerLevel()
uint8_t transmitLevelStep = 1;
uint8_t promiscuousMode = 0;
uint32_t millis_current;
uint8_t ACK_RSSI_enabled = 0;	//if enabled recipient RSSI is received via the ACK packet DATA[0]
static volatile uint8_t lockDepth = 0;	// nesting depth of rfm69_lock()

// External functions which shall be implemented outside of the library
extern void spi_transfer_sync(uint8_t *dataout, uint8_t *datain, uint8_t len);
extern void spi_transmit_sync(uint8_t *dataout, uint8_t len);
extern uint8_t spi_fast_shift(uint8_t data);
extern uint32_t millis();
extern void cli();
extern void sei();

// Keeps rfm69_ISR() away while the main context is in the middle of an SPI transaction or a
// read-modify-write sequence, otherwise the ISR's SPI bytes end up in the interrupted register access.
// Nestable. Inside rfm69_ISR() it does nothing, because the interrupt is already being serviced.
static void rfm69_lock(void) {
	if (inISR)
		return;
	if (lockDepth++ == 0)
		cli();
}

static void rfm69_unlock(void) {
	if (inISR)
		return;
	if (lockDepth > 0 && --lockDepth == 0)
		sei();
}

// Polls a register until (value & mask) == expected. Returns 1 on match, 0 on timeout.
// millis() has to advance, so it must not be used from rfm69_ISR().
static uint8_t waitForReg(uint8_t addr, uint8_t mask, uint8_t expected,
		uint32_t timeout_ms) {
	uint32_t start = millis();
	while ((readReg(addr) & mask) != expected) {
		if (millis() - start >= timeout_ms)
			return (readReg(addr) & mask) == expected; // last chance, e.g. if we were preempted for long
	}
	return 1;
}

// Writes a register until it reads back the same value, checks the SPI communication with the radio
static uint8_t checkRegWrite(uint8_t addr, uint8_t value) {
	uint32_t start = millis();
	do {
		writeReg(addr, value);
		if (readReg(addr) == value)
			return 1;
	} while (millis() - start < RF69_INIT_TIMEOUT_MS);
	return 0;
}

static void rxRestart(void) {
	rfm69_lock();
	writeReg(REG_PACKETCONFIG2,
			(readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
	rfm69_unlock();
}

static uint8_t maxAllowedPowerLevel(void) {
	uint8_t hwMax = (isRFM69HW == 1) ? 23 : 31;
	return (maxPowerLevel < hwMax) ? maxPowerLevel : hwMax;
}

// freqBand must be selected from 315, 433, 868, 915
// returns 1 on success, 0 if the radio doesn't respond (SPI or module failure)
uint8_t rfm69_init(uint16_t freqBand, uint8_t nodeID, uint8_t networkID) {
	const uint8_t CONFIG[][2] =
			{
			/* 0x01 */{ REG_OPMODE, RF_OPMODE_SEQUENCER_ON
					| RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
					/* 0x02 */{ REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET
							| RF_DATAMODUL_MODULATIONTYPE_FSK
							| RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
					/* 0x03 */{ REG_BITRATEMSB, RF_BITRATEMSB_55555 }, // default: 4.8 KBPS
					/* 0x04 */{ REG_BITRATELSB, RF_BITRATELSB_55555 },
					/* 0x05 */{ REG_FDEVMSB, RF_FDEVMSB_50000 }, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
					/* 0x06 */{ REG_FDEVLSB, RF_FDEVLSB_50000 },

					//* 0x07 */ { REG_FRFMSB, RF_FRFMSB_433},
					//* 0x08 */ { REG_FRFMID, RF_FRFMID_433},
					//* 0x09 */ { REG_FRFLSB, RF_FRFLSB_433},

					/* 0x07 */{ REG_FRFMSB, (uint8_t) (
							freqBand == RF_315MHZ ?
									RF_FRFMSB_315 :
									(freqBand == RF_433MHZ ?
											RF_FRFMSB_433 :
											(freqBand == RF_868MHZ ?
													RF_FRFMSB_868 :
													RF_FRFMSB_915))) },
					/* 0x08 */{ REG_FRFMID, (uint8_t) (
							freqBand == RF_315MHZ ?
									RF_FRFMID_315 :
									(freqBand == RF_433MHZ ?
											RF_FRFMID_433 :
											(freqBand == RF_868MHZ ?
													RF_FRFMID_868 :
													RF_FRFMID_915))) },
					/* 0x09 */{ REG_FRFLSB, (uint8_t) (
							freqBand == RF_315MHZ ?
									RF_FRFLSB_315 :
									(freqBand == RF_433MHZ ?
											RF_FRFLSB_433 :
											(freqBand == RF_868MHZ ?
													RF_FRFLSB_868 :
													RF_FRFLSB_915))) },

					// looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
					// +17dBm and +20dBm are possible on RFM69HW
					// +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
					// +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
					// +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
					///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
					///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

					// RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
					/* 0x19 */{ REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16
							| RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
					//for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
					/* 0x25 */{ REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
					/* 0x26 */{ REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
					/* 0x28 */{ REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
					/* 0x29 */{ REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
					///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
					/* 0x2E */{ REG_SYNCCONFIG, RF_SYNC_ON
							| RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2
							| RF_SYNC_TOL_0 },
					/* 0x2F */{ REG_SYNCVALUE1, RF69_SYNCVALUE1 }, // attempt to make this compatible with sync1 byte of RFM12B lib
					/* 0x30 */{ REG_SYNCVALUE2, networkID }, // NETWORK ID
					/* 0x37 */{ REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE
							| RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON
							| RF_PACKET1_CRCAUTOCLEAR_ON
							| RF_PACKET1_ADRSFILTERING_OFF },
					/* 0x38 */{ REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
					///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
					/* 0x3C */{ REG_FIFOTHRESH,
					RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
					/* 0x3D */{ REG_PACKETCONFIG2,
					RF_PACKET2_RXRESTARTDELAY_2BITS
							| RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
					//for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
					/* 0x6F */{ REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
					{ 255, 0 } };

	inISR = 0;
	lockDepth = 0;
	PAYLOADLEN = 0;

	if (!checkRegWrite(REG_SYNCVALUE1, 0xAA) || !checkRegWrite(REG_SYNCVALUE1, 0x55))
		return 0;

	for (uint8_t i = 0; CONFIG[i][0] != 255; i++)
		writeReg(CONFIG[i][0], CONFIG[i][1]);
	mode = RF69_MODE_STANDBY; // REG_OPMODE has just been written by the CONFIG table

	// Encryption is persistent between resets and can trip you up during debugging.
	// Disable it during initialization so we always start from a known state.
	encrypt(0);

	setHighPower(isRFM69HW);    // called regardless if it's a RFM69W or RFM69HW
	if (!waitForReg(REG_IRQFLAGS1, RF_IRQFLAGS1_MODEREADY,
			RF_IRQFLAGS1_MODEREADY, RF69_MODE_READY_TIMEOUT_MS))
		return 0;

	sei();

	address = nodeID;
	setAddress(address);            // setting this node id
	setNetwork(networkID);
	return 1;
}

// Returns 1 if the radio still holds the configuration of rfm69_init().
// A reset or brown-out of the radio restores the register defaults (datasheet Table 23), e.g. the
// carrier frequency goes back to 915MHz. In that case (0 is returned) the application should
// reset the radio and call rfm69_init() and its other settings again.
uint8_t rfm69_isConfigured(void) {
	return readReg(REG_SYNCVALUE1) == RF69_SYNCVALUE1;
}

// set this node's address
void setAddress(uint8_t addr) {
	writeReg(REG_NODEADRS, addr);
}

// set network address
void setNetwork(uint8_t networkID) {
	writeReg(REG_SYNCVALUE2, networkID);
}

uint8_t canSend() {
	uint8_t result = 0;
	rfm69_lock();
	// RssiValue is only meaningful once the receiver is ready (RxReady), before that it holds an old value
	if (mode == RF69_MODE_RX && PAYLOADLEN == 0
			&& (readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_RXREADY)
			&& readRSSI(0) < CSMA_LIMIT) // if signal stronger than -100dBm is detected assume channel activity
	{
		setMode(RF69_MODE_STANDBY);
		result = 1;
	}
	rfm69_unlock();
	return result;
}

// Transmit data
// returns 1 if the packet has been sent, 0 on timeout
uint8_t send(uint8_t toAddress, const void *buffer, uint8_t bufferSize,
		uint8_t requestACK) {
	rxRestart();
	millis_current = millis();
	while (!canSend() && millis() - millis_current < RF69_CSMA_LIMIT_MS)
		receiveDone();
	return sendFrame(toAddress, buffer, bufferSize, requestACK, 0);
}

// check whether an ACK was requested in the last received packet (non-broadcasted packet)
uint8_t ACKRequested() {
	return ACK_REQUESTED && (TARGETID != RF69_BROADCAST_ADDR);
}

// should be called immediately after reception in case sender wants ACK
// returns 1 if the ACK has been sent, 0 on timeout
uint8_t sendACK(const void *buffer, uint8_t bufferSize) {
	ACK_REQUESTED = 0; // TWS added to make sure we don't end up in a timing race and infinite loop sending Acks
	uint8_t sender = SENDERID;
	int16_t _RSSI = RSSI; // save payload received RSSI value
	uint8_t result;
	rxRestart();
	millis_current = millis();
	while (!canSend() && millis() - millis_current < RF69_CSMA_LIMIT_MS)
		receiveDone();
	SENDERID = sender; // TWS: Restore SenderID after it gets wiped out by receiveDone() n.b. actually now there is no receiveDone() :D
	result = sendFrame(sender, buffer, bufferSize, 0, 1);
	RSSI = _RSSI; // restore payload RSSI
	return result;
}


//setting the ACK_RSSI enabled(1) or disabled(1), if enabled recipient RSSI is received via the ACK packet DATA[0]
void set_ACK_RSSI_enabled(uint8_t enabled){

	ACK_RSSI_enabled = enabled;
	return;
}
//getting the ACK_RSSI enabled(1) or disabled(1) status
uint8_t get_ACK_RSSI_enabled(void){

	return ACK_RSSI_enabled;
}



// set *transmit/TX* output power: 0=min, 31=max
// this results in a "weaker" transmitted signal, and directly results in a lower RSSI at the receiver
// the power configurations are explained in the SX1231H datasheet (Table 10 on p21; RegPaLevel p66): http://www.semtech.com/images/datasheet/sx1231h.pdf
// valid powerLevel parameter values are 0-31 and result in a directly proportional effect on the output/transmission power
// this function implements 2 modes as follows:
//       - for RFM69W the range is from 0-31 [-18dBm to 13dBm] (PA0 only on RFIO pin)
//       - for RFM69HW the range is from 0-31 [5dBm to 20dBm]  (PA1 & PA2 on PA_BOOST pin & high Power PA settings - see section 3.3.7 in datasheet, p22)
/*void setPowerLevel(uint8_t powerLevel) {
 uint8_t _powerLevel = powerLevel;
 if (isRFM69HW == 1)
 _powerLevel /= 2;
 writeReg(REG_PALEVEL, (readReg(REG_PALEVEL) & 0xE0) | _powerLevel);
 }*/

//   - for RFM69 W/CW the range is from 0-31 [-18dBm to 13dBm] (PA0 only on RFIO pin)
//   - for RFM69 HW/HCW the range is from 0-22 [-2dBm to 20dBm]  (PA1 & PA2 on PA_BOOST pin & high Power PA settings - see section 3.3.7 in datasheet, p22)
//   - the HW/HCW 0-24 range is split into 3 REG_PALEVEL parts:
//     -  0-15 = REG_PALEVEL 16-31, ie [-2 to 13dBm] & PA1 only
//     - 16-19 = REG_PALEVEL 26-29, ie [12 to 15dBm] & PA1+PA2
//     - 20-23 = REG_PALEVEL 28-31, ie [17 to 20dBm] & PA1+PA2+HiPower (HiPower is only enabled before going in TX mode, ie by setMode(RF69_MODE_TX)
// The HW/HCW range overlaps are to smooth out transitions between the 3 PA domains, based on actual current/RSSI measurements
// Any changes to this function also demand changes in dependent function setPowerDBm()
// The level is limited to setMaxPowerLevel()
void setPowerLevel(uint8_t powerLevel_new) {
	uint8_t PA_SETTING;
	if (powerLevel_new > maxPowerLevel)
		powerLevel_new = maxPowerLevel;
	rfm69_lock();
	if (isRFM69HW == 1) {
		if (powerLevel_new > 23)
			powerLevel_new = 23;
		powerLevel = powerLevel_new;

		//now set Pout value & active PAs based on _powerLevel range as outlined in summary above
		if (powerLevel < 16) {
			powerLevel_new += 16;
			PA_SETTING = RF_PALEVEL_PA1_ON; // enable PA1 only
		} else {
			if (powerLevel < 20)
				powerLevel_new += 10;
			else
				powerLevel_new += 8;
			PA_SETTING = RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON; // enable PA1+PA2
		}
		// high power settings only while transmitting (datasheet RegTestPa1/2: revert when receiving)
		setHighPowerRegs(mode == RF69_MODE_TX);
	} else { //this is a W/CW, register value is the same as _powerLevel
		if (powerLevel_new > 31)
			powerLevel_new = 31;
		powerLevel = powerLevel_new;
		PA_SETTING = RF_PALEVEL_PA0_ON; // enable PA0 only
	}

	//write value to REG_PALEVEL
	writeReg(REG_PALEVEL, PA_SETTING | powerLevel_new);
	rfm69_unlock();
}

uint8_t getPowerLevel(void){
	return powerLevel;
}

// Upper limit of setPowerLevel() and of the automatic power increase of sendWithRetry_APC(),
// e.g. to keep the output within the legal limit. Same scale as setPowerLevel().
void setMaxPowerLevel(uint8_t level) {
	maxPowerLevel = level;
	if (powerLevel > maxPowerLevel)
		setPowerLevel(maxPowerLevel);
}

uint8_t getMaxPowerLevel(void) {
	return maxPowerLevel;
}

//put transceiver in sleep mode to save battery - to wake or resume receiving just call receiveDone()
void sleep() {
	setMode(RF69_MODE_SLEEP);
}

uint8_t readTemperature(uint8_t calFactor) // returns centigrade
{
	uint8_t temp;
	rfm69_lock();
	setMode(RF69_MODE_STANDBY); // measurement is possible only in standby or FS mode
	writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
	waitForReg(REG_TEMP1, RF_TEMP1_MEAS_RUNNING, 0, RF69_MEASURE_TIMEOUT_MS);
	temp = ~readReg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor; // 'complement' corrects the slope, rising temp = rising val
	rfm69_unlock();
	return temp;
} // COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction

// return the frequency (in Hz)
// Fstep = 32MHz / 2^19 = 15625 / 256 Hz, integer math to avoid pulling in the floating point library
uint32_t getFrequency() {
	uint32_t frf;
	rfm69_lock();
	frf = ((uint32_t) readReg(REG_FRFMSB) << 16)
			| ((uint32_t) readReg(REG_FRFMID) << 8) | readReg(REG_FRFLSB);
	rfm69_unlock();
	return (frf >> 8) * 15625 + (((frf & 0xFF) * 15625) >> 8);
}

// set the frequency (in Hz)
void setFrequency(uint32_t freqHz) {
	uint32_t frf = (freqHz / 15625) * 256 + ((freqHz % 15625) * 256) / 15625; // freqHz / Fstep
	rfm69_lock();
	uint8_t oldMode = mode;
	if (oldMode == RF69_MODE_TX) {
		setMode(RF69_MODE_RX);
	}
	writeReg(REG_FRFMSB, frf >> 16);
	writeReg(REG_FRFMID, frf >> 8);
	writeReg(REG_FRFLSB, frf);
	if (oldMode == RF69_MODE_RX) {
		setMode(RF69_MODE_SYNTH);
	}
	setMode(oldMode);
	rfm69_unlock();
}

// Read byte from register
uint8_t readReg(uint8_t addr) {
	rfm69_lock();
	select();
	spi_fast_shift(addr & 0x7F);
	uint8_t regval = spi_fast_shift(0);
	unselect();
	rfm69_unlock();
	return regval;
}

// Write byte to register
void writeReg(uint8_t addr, uint8_t value) {
	rfm69_lock();
	select();
	spi_fast_shift(addr | 0x80);
	spi_fast_shift(value);
	unselect();
	rfm69_unlock();
}

// To enable encryption: radio.encrypt("ABCDEFGHIJKLMNOP");
// To disable encryption: encrypt(null) or encrypt(0)
// KEY HAS TO BE 16 bytes !!!
void encrypt(const char *key) {
	rfm69_lock();
	setMode(RF69_MODE_STANDBY);
	if (key != 0) {
		select();
		spi_fast_shift(REG_AESKEY1 | 0x80);
		for (uint8_t i = 0; i < 16; i++)
			spi_fast_shift(key[i]);
		unselect();
	}
	writeReg(REG_PACKETCONFIG2,
			(readReg(REG_PACKETCONFIG2) & 0xFE) | (key ? 1 : 0));
	rfm69_unlock();
}

void setMode(uint8_t newMode) {
	if (newMode == mode)
		return;

	rfm69_lock();
	switch (newMode) {
	case RF69_MODE_TX:
		writeReg(REG_OPMODE,
				(readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
		if (isRFM69HW)
			setHighPowerRegs(1);
		break;
	case RF69_MODE_RX:
		writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
		if (isRFM69HW)
			setHighPowerRegs(0);
		break;
	case RF69_MODE_SYNTH:
		writeReg(REG_OPMODE,
				(readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
		break;
	case RF69_MODE_STANDBY:
		writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
		break;
	case RF69_MODE_SLEEP:
		writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
		break;
	default:
		rfm69_unlock();
		return;
	}
	// we are using packet mode, so this check is not really needed
	// but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
	if (mode == RF69_MODE_SLEEP)
		waitForReg(REG_IRQFLAGS1, RF_IRQFLAGS1_MODEREADY,
				RF_IRQFLAGS1_MODEREADY, RF69_MODE_READY_TIMEOUT_MS); // wait for ModeReady
	mode = newMode;
	rfm69_unlock();
}

//Set TX Output power in dBm:
// [-18..+13]dBm in RFM69 W/CW
// [ -2..+20]dBm in RFM69 HW/HCW
int8_t setPowerDBm(int8_t dBm) {
	if (isRFM69HW == 1) {
		//fix any out of bounds
		if (dBm < -2)
			dBm = -2;
		else if (dBm > 20)
			dBm = 20;

		//map dBm to _powerLevel according to implementation in setPowerLevel()
		if (dBm < 17)
			setPowerLevel(2 + dBm);
		//else if (dBm<16) setPowerLevel(4+dBm);
		else
			setPowerLevel(3 + dBm);
	} else { //W/CW
		if (dBm < -18)
			dBm = -18;
		else if (dBm > 13)
			dBm = 13;
		setPowerLevel(dBm + 18); // Pout = -18 + OutputPower (PA0)
		dBm = (int8_t) powerLevel - 18; // the level may have been limited by setMaxPowerLevel()
	}
	return dBm;
}

// internal function - for HW/HCW only:
// enables HiPower for 18-20dBm output
// should only be used with PA1+PA2
void setHighPowerRegs(uint8_t onOff) {

	if (isRFM69HW != 1 || powerLevel < 20) {
		onOff = 0;
	}

	rfm69_lock();
	if (onOff == 1) {
		writeReg(REG_TESTPA1, 0x5D);
		writeReg(REG_TESTPA2, 0x7C);
	} else {
		writeReg(REG_TESTPA1, 0x55);
		writeReg(REG_TESTPA2, 0x70);
	}
	rfm69_unlock();
}

// for RFM69HW only: you must call setHighPower(1) after rfm69_init() or else transmission won't work
void setHighPower(uint8_t onOff) {
	rfm69_lock();
	isRFM69HW = onOff;
	writeReg(REG_OCP, isRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
	// back to normal PA settings, the datasheet requires it with PA0 (RFM69W) and in RX,
	// on HW modules setMode(RF69_MODE_TX) enables the high power settings when needed
	setHighPowerRegs(0);
	setPowerLevel(powerLevel);
	rfm69_unlock();

	/*
	 if (isRFM69HW == 1) // turning ON
	 writeReg(REG_PALEVEL,
	 (readReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON
	 | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
	 else
	 writeReg(REG_PALEVEL,
	 RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF
	 | powerLevel); // enable P0 only
	 */
}

// get the received signal strength indicator (RSSI)
int16_t readRSSI(uint8_t forceTrigger) {
	int16_t rssi = 0;
	rfm69_lock();
	if (forceTrigger == 1) {
		// RSSI trigger not needed if DAGC is in continuous mode
		writeReg(REG_RSSICONFIG, RF_RSSI_START);
		waitForReg(REG_RSSICONFIG, RF_RSSI_DONE, RF_RSSI_DONE,
				RF69_MEASURE_TIMEOUT_MS); // wait for RSSI_Ready
	}
	rssi = -readReg(REG_RSSIVALUE);
	rfm69_unlock();
	rssi >>= 1;
	return rssi;
}

// internal function
// returns 1 if the packet has been sent, 0 on timeout (the radio is put back to standby)
uint8_t sendFrame(uint8_t toAddress, const void *buffer, uint8_t bufferSize,
		uint8_t requestACK, uint8_t sendACK) {
	uint8_t sent = 0;
	rfm69_lock();
	setMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
	if (waitForReg(REG_IRQFLAGS1, RF_IRQFLAGS1_MODEREADY,
			RF_IRQFLAGS1_MODEREADY, RF69_MODE_READY_TIMEOUT_MS)) { // wait for ModeReady
		if (bufferSize > RF69_MAX_DATA_LEN)
			bufferSize = RF69_MAX_DATA_LEN;

		// control byte
		uint8_t CTLbyte = 0x00;
		if (sendACK == 1)
			CTLbyte = RFM69_CTL_SENDACK;
		else if (requestACK == 1)
			CTLbyte = RFM69_CTL_REQACK;

		// write to FIFO
		select(); //enable data transfer
		spi_fast_shift(REG_FIFO | 0x80);
		spi_fast_shift(bufferSize + 3);
		spi_fast_shift(toAddress);
		spi_fast_shift(address);
		spi_fast_shift(CTLbyte);

		for (uint8_t i = 0; i < bufferSize; i++)
			spi_fast_shift(((uint8_t*) buffer)[i]);

		unselect();

		// no need to wait for transmit mode to be ready since its handled by the radio
		setMode(RF69_MODE_TX);
		sent = waitForReg(REG_IRQFLAGS2, RF_IRQFLAGS2_PACKETSENT,
				RF_IRQFLAGS2_PACKETSENT, RF69_TX_LIMIT_MS); // wait for PacketSent
	}
	if (!sent)
		mode = RF69_MODE_UNKNOWN; // the radio state is not what we think, force the standby write
	setMode(RF69_MODE_STANDBY);
	rfm69_unlock();
	return sent;
}

// Calibrate RC
void rcCalibration() {
	rfm69_lock();
	setMode(RF69_MODE_STANDBY); // calibration must be triggered in standby mode
	writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
	waitForReg(REG_OSC1, RF_OSC1_RCCAL_DONE, RF_OSC1_RCCAL_DONE,
			RF69_MEASURE_TIMEOUT_MS);
	rfm69_unlock();
}

uint8_t sendWithRetry(uint8_t toAddress, const void *buffer, uint8_t bufferSize,
		uint8_t retries, uint8_t retryWaitTime) {
	for (uint8_t i = 0; i <= retries; i++) {
		if (!send(toAddress, buffer, bufferSize, 1))
			continue; // TX timeout, no ACK can come
		millis_current = millis();
		while (millis() - millis_current < retryWaitTime) {
			if (ACKReceived(toAddress)) {
				return 1;
			}
		}
	}
	return 0;
}

// sendWithRetry_APC - this function increases the power level in each iteration, if the message is not received by the receiver
// the power level is not increased above setMaxPowerLevel()
uint8_t sendWithRetry_APC(uint8_t toAddress, const void *buffer,
		uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime) {
	uint8_t maxLevel = maxAllowedPowerLevel();
	for (uint8_t i = 0; i <= retries; i++) {
			if (send(toAddress, buffer, bufferSize, 1)) {
				millis_current = millis();
				while (millis() - millis_current < retryWaitTime) {
					if (ACKReceived(toAddress)) {
						if (ACK_RSSI_enabled == 1){
						last_tx_RSSI = DATA[0];
						}
						last_received_ack_RSSI = RSSI;
						return 1;
					}
				}
			}
			if (powerLevel < maxLevel) {
				setPowerLevel(powerLevel + transmitLevelStep);
			}
		}
	return 0;
}




// should be polled immediately after sending a packet with ACK request
uint8_t ACKReceived(uint8_t fromNodeID) {
	if (receiveDone())
		return (SENDERID == fromNodeID || fromNodeID == RF69_BROADCAST_ADDR)
				&& ACK_RECEIVED;
	return 0;
}

// checks if a packet was received and/or puts transceiver in receive (ie RX or listen) mode
uint8_t receiveDone() {
	uint8_t result = 0;
	rfm69_lock();

	if (mode == RF69_MODE_RX && PAYLOADLEN > 0) {
		setMode(RF69_MODE_STANDBY);
		result = 1;
	} else if (mode != RF69_MODE_RX) // not in RX yet, start receiving
	{
		receiveBegin();
	}
	rfm69_unlock();
	return result;
}

// internal function
void receiveBegin() {
	rfm69_lock();
	DATALEN = 0;
	SENDERID = 0;
	TARGETID = 0;
	PAYLOADLEN = 0;
	ACK_REQUESTED = 0;
	ACK_RECEIVED = 0;
	RSSI = 0;
	if (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
		rxRestart();
	writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
	setMode(RF69_MODE_RX);
	rfm69_unlock();
}

// 1  = disable filtering to capture all frames on network
// 0 = enable node/broadcast filtering to capture only frames sent to this/broadcast address
// The filtering is done in rfm69_ISR(). The hardware address filter stays off: the address byte is
// sent as part of the message, so with AES it is encrypted and the hardware filter would drop the
// packets (datasheet 5.5.5), besides it would limit the message to 48 bytes.
void promiscuous(uint8_t onOff) {
	promiscuousMode = onOff;
}

// Only reenable interrupts if we're not being called from the ISR
void maybeInterrupts() {
	if (!inISR)
		sei();
}

// Interrupt Service Routine
// cli()/sei() have to mask the interrupt this function is called from
void rfm69_ISR(void) {
	inISR = 1;
	if (mode == RF69_MODE_RX
			&& (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)) {
		int16_t rssi = readRSSI(0); // still in RX, holds the level of this packet
		setMode(RF69_MODE_STANDBY);
		select();
		spi_fast_shift(REG_FIFO & 0x7F);
		PAYLOADLEN = spi_fast_shift(0);
		if (PAYLOADLEN > 66)
			PAYLOADLEN = 66;
		TARGETID = spi_fast_shift(0);
		if (!(promiscuousMode || TARGETID == address
				|| TARGETID == RF69_BROADCAST_ADDR) // match this node's address, or broadcast address or anything in promiscuous mode
		|| PAYLOADLEN < 3) // address situation could receive packets that are malformed and don't fit this libraries extra fields
				{
			PAYLOADLEN = 0;
			unselect();
			receiveBegin();
			inISR = 0;
			return;
		}

		DATALEN = PAYLOADLEN - 3;
		if (DATALEN > RF69_MAX_DATA_LEN)
			DATALEN = RF69_MAX_DATA_LEN; // never overrun DATA[], the rest is dropped with the FIFO
		SENDERID = spi_fast_shift(0);
		uint8_t CTLbyte = spi_fast_shift(0);

		ACK_RECEIVED = CTLbyte & RFM69_CTL_SENDACK; // extract ACK-received flag
		ACK_REQUESTED = CTLbyte & RFM69_CTL_REQACK; // extract ACK-requested flag

		//interruptHook(CTLbyte);                   // TWS: hook to derived class interrupt function

		for (uint8_t i = 0; i < DATALEN; i++) {
			DATA[i] = spi_fast_shift(0);
		}
		if (DATALEN < RF69_MAX_DATA_LEN)
			DATA[DATALEN] = 0; // add null at end of string
		unselect();
		RSSI = rssi;
		setMode(RF69_MODE_RX);
	}
	inISR = 0;
}
