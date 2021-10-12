/*!
 *  @file Adafruit_PWMServoDriver.cpp
 *
 *  @mainpage Adafruit 16-channel PWM & Servo driver
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the 16-channel PWM & Servo driver.
 *
 *  Designed specifically to work with the Adafruit PWM & Servo driver.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/815
 *
 *  These displays use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  @section license License
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include "Adafruit_PWMServoDriver.h"
#include <pigpio/pigpio.h>


#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */
#define PCA9685_FREQUENCY_OSCILLATOR 25'000'000 /**< Int. osc. frequency in datasheet */


// REGISTER ADDRESSES
#define PCA9685_MODE1 0x00      /**< Mode Register 1 */
#define PCA9685_MODE2 0x01      /**< Mode Register 2 */
#define PCA9685_SUBADR1 0x02    /**< I2C-bus subaddress 1 */
#define PCA9685_SUBADR2 0x03    /**< I2C-bus subaddress 2 */
#define PCA9685_SUBADR3 0x04    /**< I2C-bus subaddress 3 */
#define PCA9685_ALLCALLADR 0x05 /**< LED All Call I2C-bus address */
#define PCA9685_LED0_ON_L 0x06  /**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H 0x07  /**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L 0xFA  /**< load all the LEDn_ON registers, low */
#define PCA9685_ALLLED_ON_H 0xFB  /**< load all the LEDn_ON registers, high */
#define PCA9685_ALLLED_OFF_L 0xFC /**< load all the LEDn_OFF registers, low */
#define PCA9685_ALLLED_OFF_H 0xFD /**< load all the LEDn_OFF registers,high */
#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE 0xFF     /**< defines the test mode to be entered */

// MODE1 bits
#define MODE1_ALLCAL 0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02    /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04    /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08    /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20      /**< Auto-Increment enabled */
#define MODE1_EXTCLK 0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART 0x80 /**< Restart enabled */
// MODE2 bits
#define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1                                                          \
  0x02 /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 0x04 /**< totem pole structure vs open-drain */
#define MODE2_OCH 0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 0x10  /**< Output logic state inverted */

// For putting thread to sleep
#include <thread>
// For picking how long to sleep
#include <chrono>
#include <cstdio>
#include <cassert>
#include <stdexcept>
#include <algorithm>

static void sleepMillis(int count) {
	std::this_thread::sleep_for(std::chrono::milliseconds(count));
}

static void printI2CError(int type) {
	switch(type) {
	case PI_BAD_I2C_BUS:
		printf("PI_BAD_I2C_BUS");
		break;
	case PI_BAD_I2C_ADDR:
		printf("PI_BAD_I2C_ADDR");
		break;
	case PI_BAD_FLAGS:
		printf("PI_BAD_FLAGS");
		break;
	case PI_NO_HANDLE:
		printf("PI_NO_HANDLE");
		break;
	case PI_I2C_OPEN_FAILED:
		printf("PI_I2C_OPEN_FAILED");
		break;
	case PI_BAD_PARAM:
		printf("PI_BAD_PARAM");
		break;
	case PI_I2C_READ_FAILED:
		printf("PI_I2C_READ_FAILED");
		break;
	case PI_BAD_HANDLE:
		printf("PI_BAD_HANDLE");
		break;
	case PI_BAD_POINTER:
		printf("PI_BAD_POINTER");
		break;
	case PI_BAD_I2C_CMD:
		printf("PI_BAD_I2C_CMD");
		break;
	case PI_BAD_I2C_RLEN:
		printf("PI_BAD_I2C_RLEN");
		break;
	case PI_BAD_I2C_WLEN:
		printf("PI_BAD_I2C_WLEN");
		break;
	default:
		printf("Unexpected i2c error code! 0x%X\n", type);
		break;
	}
}
static void throwI2CError(int type) {
	switch(type) {
	case PI_BAD_I2C_BUS:
		throw std::logic_error("PI_BAD_I2C_BUS");
	case PI_BAD_I2C_ADDR:
		throw std::logic_error("PI_BAD_I2C_ADDR");
	case PI_BAD_FLAGS:
		throw std::logic_error("PI_BAD_FLAGS");
	case PI_NO_HANDLE:
		throw std::logic_error("PI_NO_HANDLE");
	case PI_I2C_OPEN_FAILED:
		throw std::logic_error("PI_I2C_OPEN_FAILED");
	case PI_BAD_PARAM:
		throw std::logic_error("PI_BAD_PARAM");
	case PI_I2C_READ_FAILED:
		throw std::logic_error("PI_I2C_READ_FAILED");
	case PI_BAD_HANDLE:
		throw std::logic_error("PI_BAD_HANDLE");
	case PI_BAD_POINTER:
		throw std::logic_error("PI_BAD_POINTER");
	case PI_BAD_I2C_CMD:
		throw std::logic_error("PI_BAD_I2C_CMD");
	case PI_BAD_I2C_RLEN:
		throw std::logic_error("PI_BAD_I2C_RLEN");
	case PI_BAD_I2C_WLEN:
		throw std::logic_error("PI_BAD_I2C_WLEN");
	default:
		throw std::logic_error("Unexpected i2c error code! 0x%X\n", type);
	}
}
static void checkI2CError(int type) {
	if(type < 0) {
		throwError(type);
	}
}

/*!
 *  @brief  Instantiates a new PCA9685 PWM driver chip with the I2C address
 *  @param  addr The 7-bit I2C address to locate this chip, default is 0x40
 */
Adafruit_PWMServoDriver::Adafruit_PWMServoDriver(const uint8_t addr)
    : Adafruit_PWMServoDriver(addr, 0)
{}

/*!
 *  @brief  Instantiates a new PCA9685 PWM driver chip with the I2C address
 *  @param  addr The 7-bit I2C address to locate this chip, default is 0x40
 *  @param  bus The i2c bus to use on the pi, default is 0
 */
Adafruit_PWMServoDriver::Adafruit_PWMServoDriver(const uint8_t addr,
                                                 const uint8_t bus)
    : i2cAddr(addr)
	, i2cBus(bus)
	, handle(-1) 
	, oscillatorFreq(PCA9685_FREQUENCY_OSCILLATOR)
{}


Adafruit_PWMServoDriver::Adafruit_PWMServoDriver(const Adafruit_PWMServoDriver & other)
	: i2cAddr(other.i2cAddr)
	, i2cBus(other.i2cBus)
	, handle(-1)
	, oscillatorFreq(other.oscillatorFreq)
{
	/// Attempt to make a new connection
}
Adafruit_PWMServoDriver::Adafruit_PWMServoDriver(Adafruit_PWMServoDriver && other) noexcept 
	: i2cAddr(other.i2cAddr)
	, i2cBus(other.i2cBus)
	, handle(other.handle)
	, oscillatorFreq(other.oscillatorFreq)
{
	other.handle = -1;
}
Adafruit_PWMServoDriver & Adafruit_PWMServoDriver::operator=(const Adafruit_PWMServoDriver & other) {
	if(other.handle >= 0) {
		// Check to see if we can open another connection BEFORE we overwrite the data in this object.
		int result = i2cOpen(i2cBus, i2cAddr, 0);
		checkI2CError(result);
		if(handle >= 0) {
			i2cClose(handle);
		}
		handle = result;
	}
	else {
		if(handle >= 0) {
			i2cClose(handle);
		}
		handle = -1;
	}
	
	i2cAddr = other.i2cAddr;
	i2cBus = other.i2cBus;
	oscillatorFreq = other.oscillatorFreq;
	
	return *this;
}
Adafruit_PWMServoDriver & Adafruit_PWMServoDriver::operator=(Adafruit_PWMServoDriver && other) {
	if(handle >= 0) {
		i2cClose(handle);
	}
	i2cAddr = other.i2cAddr;
	i2cBus = other.i2cBus;
	handle = other.handle;
	oscillatorFreq = other.oscillatorFreq;
	
	other.handle = -1;
	return *this;
}
Adafruit_PWMServoDriver::~Adafruit_PWMServoDriver() {
	if(handle >= 0) {
		i2cClose(handle);
	}
}

bool Adafruit_PWMServoDriver::isValid() const noexcept {
	return handle >= 0;
}

/*!
 *  @brief  Setups the I2C interface and hardware
 *  @param  prescale
 *          Sets External Clock (Optional)
 */
void Adafruit_PWMServoDriver::begin() {
	assert(!isValid() && "Called begin twice!");
	
	int result = i2cOpen(i2cBus, i2cAddr, 0);
	checkI2CError(result);

	handle = result;

	// reset the state of the board
	reset();

	// set a default frequency
	setPWMFreq(50);
}

/*!
 *  @brief  Sends a reset command to the PCA9685 chip over I2C
 */
void Adafruit_PWMServoDriver::reset() {
	assert(isValid() && "Servo driver was not initialized!");
	
	int result = i2cWriteByteData(handle, PCA9685_MODE1, MODE1_RESTART);
	
	checkI2CError(result);

	sleepMillis(10);
}

/*!
 *  @brief  Puts board into sleep mode
 */
void Adafruit_PWMServoDriver::sleep() {
	assert(isValid() && "Servo driver was not initialized!");
	
	int awake = i2cReadByteData(handle, PCA9685_MODE1);
	checkI2CError(awake);

	uint8_t sleep = awake | MODE1_SLEEP; // set sleep bit high
	int result = i2cWriteByteData(handle, PCA9685_MODE1, sleep);
	checkI2CError(result);

	// wait until cycle ends for sleep to be active
	sleepMillis(5);
}

/*!
 *  @brief  Wakes board from sleep
 */
void Adafruit_PWMServoDriver::wakeup() {
	assert(isValid() && "Servo driver was not initialized!");
	
	int sleep = i2cReadByteData(handle, PCA9685_MODE1);
	checkI2CError(sleep);

	uint8_t wakeup = sleep & ~MODE1_SLEEP; // set sleep bit low
	int result = i2cWriteByteData(handle, PCA9685_MODE1, wakeup);
	checkI2CError(result);
}

/*!
 *  @brief  Sets EXTCLK pin to use the external clock
 *  @param  prescale
 *          Configures the prescale value to be used by the external clock
 */
void Adafruit_PWMServoDriver::setExtClk(uint8_t prescale) {
	assert(isValid() && "Servo driver was not initialized!");
	
	int oldmode = i2cReadByteData(handle, PCA9685_MODE1);
	checkI2CError(oldmode);

	// Remove the restart bit, add sleep bit
	uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
	
	int result = i2cWriteByteData(handle, PCA9685_MODE1, newmode);
	checkI2CError(result);
	
	// This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
	// use the external clock.
	result = i2cWriteByteData(handle, PCA9685_MODE1, (newmode |= MODE1_EXTCLK));
	checkI2CError(result);

	// set the prescaler
	result = i2cWriteByteData(handle, PCA9685_PRESCALE, prescale);
	checkI2CError(result);
	
	sleepMillis(5);
	
	// clear the SLEEP bit to start
	newmode = (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI;
	result = i2cWriteByteData(handle, PCA9685_MODE1, newmode);
	checkI2CError(result);

#ifdef ENABLE_DEBUG_OUTPUT
	printf("Mode now 0x%X\n", newmode);
#endif
}

/*!
 *  @brief  Sets the PWM frequency (in Hz) for the entire chip, up to ~1.6 KHz
 *  @param  freq Floating point frequency that we will attempt to match
 */
void Adafruit_PWMServoDriver::setPWMFreq(float freq) {
	assert(isValid() && "Servo driver was not initialized!");
	
#ifdef ENABLE_DEBUG_OUTPUT
	printf("Attempting to set PWM frequency to %f\n", freq);
#endif
	// Range output modulation frequency is dependant on oscillator
	
	// Clamp the frequency to a acceptable range.
	freq = std::max(1.f, freq);

	// 4096 ticks per cycle, multiply by the frequency to get the total ticks required
	// The prescale is for the tick rate
	// Add 0.5 to make sure it rounds up
	int iprescale = ((oscillatorFreq / (freq * 4096.0)) + 0.5) - 1;
	iprescale = std::max<int>(PCA9685_PRESCALE_MIN, std::min<int>(iprescale, PCA9685_PRESCALE_MAX));

	uint8_t prescale = (uint8_t)iprescale;
#ifdef ENABLE_DEBUG_OUTPUT
	printf("Final prescale: %d\n", prescale);
#endif

	// Docs say we must set sleep bit before setting the prescale.
	int oldmode = i2cReadByteData(handle, PCA9685_MODE1);
	checkI2CError(oldmode);

	uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
	
	int result = i2cWriteByteData(handle, PCA9685_MODE1, newmode);
	checkI2CError(result);

	result = i2cWriteByteData(handle, PCA9685_PRESCALE, prescale);
	checkI2CError(result);

	result = i2cWriteByteData(handle, PCA9685_MODE1, oldmode);
	checkI2CError(result);
	
	sleepMillis(5);
	
	// This sets the MODE1 register to turn on auto increment.
	result = i2cWriteByteData(handle, PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
	checkI2CError(result);

#ifdef ENABLE_DEBUG_OUTPUT
	printf("Mode now 0x%X\n", newmode);
#endif
}

/*!
 *  @brief  Sets the output mode of the PCA9685 to either
 *  open drain or push pull.
 *  Warning: LEDs with integrated zener diodes should
 *  only be driven in open drain mode.
 *  @param  enabled Push pull if true, open drain if false
 */
void Adafruit_PWMServoDriver::setPushPull(bool enabled) {
	//We just have to add or remove the MODE2_OUTDRV flag in this function.
	int oldmode = i2cReadByteData(handle, PCA9685_MODE2);
	checkI2CError(oldmode);

	uint8_t newmode;
	if (enabled) {
		newmode = oldmode | MODE2_OUTDRV;
	} else {
		newmode = oldmode & ~MODE2_OUTDRV;
	}
	int result = i2cWriteByteData(handle, PCA9685_MODE2, newmode);
	checkI2CError(result);
}

/*!
 *  @brief  Sets the output mode of the PCA9685 to either
 *  open drain or push pull / totempole.
 *  Warning: LEDs with integrated zener diodes should
 *  only be driven in open drain mode.
 *  @param  totempole Totempole if true, open drain if false.
 */
void Adafruit_PWMServoDriver::setOpenDrain(bool enabled) {
	setPushPull(!enabled);
}


/*!
 *  @brief  Reads set Prescale from PCA9685
 *  @return prescale value
 */
uint8_t Adafruit_PWMServoDriver::readPrescale(void) {
	int result = i2cReadByteData(handle, PCA9685_PRESCALE);
	checkI2CError(result);

	return (uint8_t)result;
}

/*!
 *  @brief  Gets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @return requested PWM output value
 */
uint8_t Adafruit_PWMServoDriver::getPWM(uint8_t num) {
	// Why request all 4 bytes?
	// Why use only the high byte?
	// This function need some work.
	char data[4];
	int result = i2cReadI2CBlockData(handle, PCA9685_LED0_ON_L + 4 * num, data, 4);
	checkI2CError(result);

	return data[0];
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  on At what point in the 4096-part cycle to turn the PWM output ON
 *  @param  off At what point in the 4096-part cycle to turn the PWM output OFF
 */
void Adafruit_PWMServoDriver::setPWM(uint8_t num, uint16_t on, uint16_t off) {
#ifdef ENABLE_DEBUG_OUTPUT
	printf("Setting PWM %d: %d -> %d\n", num, on, off);
#endif
	// on Low -> High, off Low -> High
	char data[4]{
		(char)(on & 0xFF),
		(char)(on >> 8),
		(char)(off & 0xFF),
		(char)(off >> 8),
	};
	
	int result = i2cWriteI2CBlockData(handle, PCA9685_LED0_ON_L + 4 * num, data, 4);
	checkI2CError(result);
}

/*!
 *   @brief  Helper to set pin PWM output. Sets pin without having to deal with
 * on/off tick placement and properly handles a zero value as completely off and
 * 4095 as completely on.  Optional invert parameter supports inverting the
 * pulse for sinking to ground.
 *   @param  num One of the PWM output pins, from 0 to 15
 *   @param  val The number of ticks out of 4096 to be active, should be a value
 * from 0 to 4095 inclusive.
 *   @param  invert If true, inverts the output, defaults to 'false'
 */
void Adafruit_PWMServoDriver::setPin(uint8_t num, uint16_t val, bool invert) {
	// Clamp value between 0 and 4095 inclusive.
	val = std::min<uint16_t>(val, 4095);
	if(invert) {
		val = 4095 - val;
	}
  
	if (val == 4095) {
		// Special value for signal fully on.
		setPWM(num, 4096, 0);
	} else if (val == 0) {
		// Special value for signal fully off.
		setPWM(num, 0, 4096);
	} else {
		setPWM(num, 0, val);
	}
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins based on the input
 * microseconds, output is not precise
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  Microseconds The number of Microseconds to turn the PWM output ON
 */
void Adafruit_PWMServoDriver::writeMicroseconds(uint8_t num,
                                                uint16_t Microseconds) {
#ifdef ENABLE_DEBUG_OUTPUT
	printf("Setting PWM via microseconds on output %d: %d ->\n", num, Microseconds);
#endif

	double pulse = Microseconds;
	double pulselength = 1'000'000; // 1,000,000 us per second

	// Read prescale
	uint16_t prescale = readPrescale();

#ifdef ENABLE_DEBUG_OUTPUT
	printf("%d PCA9685 chip prescale\n", prescale);
#endif

	// Calculate the pulse for PWM based on Equation 1 from the datasheet section
	// 7.3.5
	prescale += 1;
	pulselength = (pulselength * prescale) / oscillatorFreq;

#ifdef ENABLE_DEBUG_OUTPUT
	printf("%f us per bit\n", pulselength);
#endif

	pulse /= pulselength;

#ifdef ENABLE_DEBUG_OUTPUT
	printf("%f pulse for PWM\n", pulse);
#endif

	setPWM(num, 0, pulse);
}

/*!
 *  @brief  Getter for the internally tracked oscillator used for freq
 * calculations
 *  @returns The frequency the PCA9685 thinks it is running at (it cannot
 * introspect)
 */
uint32_t Adafruit_PWMServoDriver::getOscillatorFrequency(void) {
  return oscillatorFreq;
}

/*!
 *  @brief Setter for the internally tracked oscillator used for freq
 * calculations
 *  @param freq The frequency the PCA9685 should use for frequency calculations
 */
void Adafruit_PWMServoDriver::setOscillatorFrequency(uint32_t freq) {
	oscillatorFreq = freq;
}
