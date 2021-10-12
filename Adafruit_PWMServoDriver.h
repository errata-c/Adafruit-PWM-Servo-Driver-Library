/*!
 *  @file Adafruit_PWMServoDriver.h
 *
 *  This is a library for our Adafruit 16-channel PWM & Servo driver.
 *
 *  Designed specifically to work with the Adafruit 16-channel PWM & Servo
 * driver.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/815
 *
 *  These driver use I2C to communicate, 2 pins are required to interface.
 *  For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  BSD license, all text above must be included in any redistribution
 */
#ifndef _ADAFRUIT_PWMServoDriver_H
#define _ADAFRUIT_PWMServoDriver_H

#include <cinttypes>

/*!
 *  @brief  Class that stores state and functions for interacting with PCA9685
 * PWM chip
 */
class Adafruit_PWMServoDriver {
public:
	static constexpr uint32_t DefaultI2CAddress = 0x40;
	
	Adafruit_PWMServoDriver(const uint8_t addr = DefaultI2CAddress);
	Adafruit_PWMServoDriver(const uint8_t addr, const uint8_t _i2cBus);

	Adafruit_PWMServoDriver(const Adafruit_PWMServoDriver & other);
	Adafruit_PWMServoDriver(Adafruit_PWMServoDriver && other) noexcept;
	Adafruit_PWMServoDriver & operator=(const Adafruit_PWMServoDriver & other);
	Adafruit_PWMServoDriver & operator=(Adafruit_PWMServoDriver && other);
	~Adafruit_PWMServoDriver();
	
	bool isValid() const noexcept;

	void begin();
	void reset();
	void sleep();
	void wakeup();
	void setExtClk(uint8_t prescale);
	void setPWMFreq(float freq);

	void setPushPull(bool enabled);
	void setOpenDrain(bool enabled);

	uint8_t getPWM(uint8_t num);
	void setPWM(uint8_t num, uint16_t on, uint16_t off);
	void setPin(uint8_t num, uint16_t val, bool invert = false);
	uint8_t readPrescale(void);
	void writeMicroseconds(uint8_t num, uint16_t Microseconds);

	void setOscillatorFrequency(uint32_t freq);
	uint32_t getOscillatorFrequency(void);

private:
	uint8_t i2cAddr;
	uint8_t i2cBus;
	int handle;

	uint32_t oscillatorFreq;
};

#endif
