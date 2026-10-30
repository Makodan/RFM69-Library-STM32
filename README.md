# RFM69-Library-STM32 #
Original library is written for arduino by [LowPowerLab](https://github.com/LowPowerLab/RFM69).

The AVR based library can be found: [RFM69-Library-AVR](https://github.com/nayem-cosmic/RFM69-Library-AVR).

The AVR based library has been modified for STM32 compatibility.
Tested on STM32F103C8 and on STM32L031F6P6.

</br>

## I/O pin connections: ##

| RF Module | Microcontroller |
| --------- | --------------- |
|MOSI |	MOSI |
| MISO | MISO |
| SCK | SCK |
| SS | SS |
| DIO0 | Any interrupt enabled pin |

</br>

## Functions need to be implemented: ##

**uint8_t spi_fast_shift(uint8_t data)**:\
Clocks only one byte to target device and returns the received one.

**uint32_t millis()**:\
Return with the millisecs. It can be done by a timer or by the ticks. It has to advance while the library functions run, all blocking waits of the library are limited by it.

**void select()**:\
Enable SPI transfer. (Negated logic - Pulled LOW means selection)

**void unselect()**:\
Disable SPI transfer. (Negated logic - Pulled HIGH means unselection)

**void cli()**:\
Disable the interrupt of the RFM69 pin. It has to mask the interrupt from which `rfm69_ISR()` is called, but the pending edge must not be lost (e.g. `HAL_NVIC_DisableIRQ()`). The library uses it to keep the ISR away from its SPI transactions.

**void sei()**:\
Enable the interrupt of the RFM69 pin. 

**void spi_transfer_sync(...)**, **void spi_transmit_sync(...)**:\
Not used by the library, not needed.

**void reset_rfm69(void)**:\
OPTIONAL - Reset the RFM69 modul via its reset pin. (Datasheet: pull the reset pin high for 100us, then wait 5ms.)

These functions have to be externed, to make possible the function call by the library.

Call **rfm69_ISR()** from the rising edge interrupt of DIO0.

The library contains no hardware specific code (SPI, GPIO, watchdog), it can be used on any MCU with the functions above.

## Blocking times and error handling: ##

Every wait in the library has a timeout, so the library never hangs, e.g. if the radio is missing or it was reset by a brown-out. The timeouts can be overridden by defining them before including `RFM69.h` (e.g. `-DRF69_TX_LIMIT_MS=500`):

| Define | Default | Used for |
| ------ | ------- | -------- |
| RF69_CSMA_LIMIT_MS | 1000 | max wait for a free channel before sending |
| RF69_TX_LIMIT_MS | 1000 | max wait for PacketSent |
| RF69_INIT_TIMEOUT_MS | 50 | SPI communication check in `rfm69_init()` |
| RF69_MODE_READY_TIMEOUT_MS | 50 | ModeReady after a mode change |
| RF69_MEASURE_TIMEOUT_MS | 50 | RSSI, temperature and RC calibration |

The worst case blocking time of `send()` is ~`RF69_CSMA_LIMIT_MS + RF69_MODE_READY_TIMEOUT_MS + RF69_TX_LIMIT_MS`, for `sendWithRetry()` multiply it by `retries + 1` and add `retryWaitTime`. A watchdog of the application has to tolerate this.

`rfm69_init()`, `send()`, `sendACK()` return 0 on failure. Recovery is the job of the application, e.g.:

```
if (!sendWithRetry_APC(...) && !rfm69_isConfigured()) {
    reset_rfm69();
    rfm69_init(...);   // and the other settings: frequency, power, encryption key
}
```

## Function Description: ##

1.	**rfm69_init(*uint16_t freqBand, uint8_t nodeID, uint8_t networkID*):** Initializes rfm69 module. This function is called at the beginning of the program. Initializes IDs, modes etc. It takes three parameters. First one freqBand. You have to choose among 315, 433, 868 and 915. These specifies frequency in MHz. nodeID is analogues to device ID. Each RF module will have unique nodeID. Value must be within 0 to 255. Then comes notworkID. Say, a system has 5 rf modules to communicate with each other. All the modules must be in same networkID . networkID value range 0~255. Returns 1 on success, 0 if the radio doesn't respond.
1.	**rfm69_isConfigured():** Returns 0 if the radio lost its configuration (reset, brown-out), then it has to be initialized again.
1.	**setMaxPowerLevel(*uint8_t level*):** Upper limit of `setPowerLevel()` and of the automatic power increase of `sendWithRetry_APC()`, e.g. to keep the legal limit. Default: no limit.
2.	**setAddress(*uint8_t addr*):** Sets nodeID.
3.	**setNetwork(*uint8_t networkID*):** Sets networkID.
4.	**send(*uint8_t toAddress, const void\* buffer, uint8_t bufferSize, uint8_t requestACK*):** Transmits data to another node. First argument is toAddress that is address off receiver node/gateway. In buffer you can put any kind of buffer like string or array etc. In requestACK you can pass 0 or 1 whether you need acknowledgement of transmitted data.
5.	**sendWithRetry(*uint8_t toAddress, const void\* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime*):** This sends data with retry. Internally manages ACK. retryWaitTime – after transmitting data module waits for ack if doesn’t have ack then again transmits data. retryWaitTime is time interval between sending.
6.	**ACKRequested():** This function needed in listening process. Checks whether acknowledgement requested or not.
7.	**sendACK(*const void\* buffer , uint8_t bufferSize*):** If ACK requested, send ACK through this function.
8.	**receiveDone():**  Returns 1 if any data is present in receive buffer.
9.	**getFrequency():** Gets frequency Band.
10.	**setFrequency(*uint32_t freqHz*):** Sets frequency band. You can set frequency other than 315, 433, 868, 915 MHz through this function. Unit is Hz i.e 433000000. 
11.	**encrypt(*const char\* key*):** All device need same encryption key. And length must be 16. If you need no encryption just put 0 in argument. 
12.	**readRSSI(*uint8_t forceTrigger=0*):** You want to know received signal strength? :D
13.	**setHighPower(*uint8_t onOFF=1*):** RFM69 has different suffixes like, W, HW or HCW etcetra. Having ‘H’ word indicated high power enabled. If you use module having ‘H’ letter put 1 as argument. This function must be called after initialization.
14.	**setPowerLevel(*uint8_t level*):** Sets transmit power. Range 0~31.
15.	**readTemperature(*uint8_t calFactor=0*):** gets CMOS temperature (8bit)
16.	**rcCalibration():** Calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section 4.3.5. RC Timer Accuracy. Not tested yet.
17.	**promiscuous(*uint8_t onOff*):** 1 or 0. If on, module receives data indiscriminately. In another word, it receives all data in network when set to 1. The address filtering is done in software.

</br>

## Basic Operation: ##
#### Transmit data: ####

```
rfm69_init(freq, nodeID, networkID)
setHighPower(0 or 1)
setPowerLevel(0~31)
send(toNodeID, buffer, bufferLen, 0 or 1 )
```

#### Receive data: ####

```
rfm69_init(freq, nodeID, networkID)
setHighPower(0 or 1)
setPowerLevel(0~31)
mainloop
    if receiveDone() then
        if ACKRequested() then
            sendACK()
        extract received data from DATA buffer
```
