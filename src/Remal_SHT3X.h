/**
 * @file        Remal_SHT3X.h
 * @author      Mohammed Hani Ahmed (https://www.linkedin.com/in/mohammed-hani-ahmed/)
 * @date        August 2, 2023
 * @version     1.0
 * 
 * @brief       Header file for the Remal_SHT3X library, providing an interface to control
 *              the SHT3x-DIS temperature and humidity sensor using I2C communication.
 * 
 * @details     This library allows users to interact with the SHT3x-DIS temperature and humidity
 *              sensor over I2C. It provides functions to initialize the sensor, perform single-shot
 *              measurements, read temperature and humidity values, and more.
 * 
 * @note        This library is designed for use with the SHT3x-DIS sensor. Please ensure that
 *              you have connected the sensor correctly to your I2C bus before using this library.
 * 
 * @todo        - Implement additional error handling and reporting.
 *              - Add more error codes for specific errors.
 *              - Create a function for combining MSB and LSB, returns (msb << 8) | lsb.
 *              - Add examples for using the library.
 *              - Implement CRC verification for data integrity.
 *              - Implement ALERT functions for the sensor.
 *              - Periodic measurement functionality needs an overhaul (to go with the ALERT functions).
 *              - Create functions for getting periodic measurement data, calling Fetch internally.
 *              - Implement single-shot measurements without clock stretching??
 *              - Implement a changing repeatability in a different way (without switch case).
 *              - Add a parameter for heater status (on/off).
**/
#ifndef _REMAL_SHT3X_H_
#define _REMAL_SHT3X_H_

//<!-- Arduino includes -->
#include <Arduino.h>
#include <Wire.h>

#include <stdint.h>

/********************************
 * <!-- Defines -->
 ********************************/
// I2C address (0x44 => ADDR pin is LOW, 0x45 => ADDR pin is HIGH)
#define SHT3X_ADDRESS 0x44					// Default I2C address when the ADDR pin is set to LOW
#define SHT3X_ADDRESS_B 0x45				// I2C address when the ADDR pin is set to HIGH

// Measurement commands for single shot data acquisition mode
#define MEAS_HI_REP_CS_ENABLED 0x2C06		// Measure with high repeatability and clock stretching enabled
#define MEAS_MID_REP_CS_ENABLED 0x2C0D		// Measure with medium repeatability and clock stretching enabled
#define MEAS_LOW_REP_CS_ENABLED 0x2C10		// Measure with low repeatability and clock stretching enabled
#define MEAS_HI_REP_CS_DISABLED 0x2400		// Measure with high repeatability and clock stretching disabled
#define MEAS_MID_REP_CS_DISABLED 0x240B		// Measure with medium repeatability and clock stretching disabled
#define MEAS_LOW_REP_CS_DISABLED 0x2416		// Measure with low repeatability and clock stretching disabled

// Measurement commands for periodic data acquisition mode
#define MEAS_HI_REP_05_MPS 0x2032			// Measure with high repeatability at 0.5 measurements per second
#define MEAS_MID_REP_05_MPS 0x2024			// Measure with medium repeatability at 0.5 measurements per second
#define MEAS_LOW_REP_05_MPS 0x202F			// Measure with low repeatability at 0.5 measurements per second
#define MEAS_HI_REP_1_MPS 0x2130			// Measure with high repeatability at 1 measurements per second
#define MEAS_MID_REP_1_MPS 0x2126			// Measure with medium repeatability at 1 measurements per second
#define MEAS_LOW_REP_1_MPS 0x212D			// Measure with low repeatability at 1 measurements per second
#define MEAS_HI_REP_2_MPS 0x2236			// Measure with high repeatability at 2 measurements per second
#define MEAS_MID_REP_2_MPS 0x2220			// Measure with medium repeatability at 2 measurements per second
#define MEAS_LOW_REP_2_MPS 0x222B			// Measure with low repeatability at 2 measurements per second
#define MEAS_HI_REP_4_MPS 0x2334			// Measure with high repeatability at 4 measurements per second
#define MEAS_MID_REP_4_MPS 0x2322			// Measure with medium repeatability at 4 measurements per second
#define MEAS_LOW_REP_4_MPS 0x2329			// Measure with low repeatability at 4 measurements per second
#define MEAS_HI_REP_10_MPS 0x2737			// Measure with high repeatability at 10 measurements per second
#define MEAS_MID_REP_10_MPS 0x2721			// Measure with medium repeatability at 10 measurements per second
#define MEAS_LOW_REP_10_MPS 0x272A			// Measure with low repeatability at 10 measurements per second

#define FETCH_DATA_COMMAND 0xE000			// Fetch data command: readout of measurement results for periodic mode
#define ART_COMMAND 0x2B32					// ART command: periodic measurement with accelerated response time (ART)
#define BREAK_COMMAND 0x3093				// Break command: stop periodic data acquisition mode
#define SOFT_RESET_COMMAND 0x30A2			// Soft reset command: re-initialization
#define RESET_GENERAL_CALL_COMMAND 0x0006	// Reset through general call address command

// Heater commands
#define HEATER_ENABLE_COMMAND 0x306D		// Heater enable command
#define HEATER_DISABLE_COMMAND 0x3066		// Heater disable command


#define READ_STATUS_REGISTER_COMMAND 0xF32D	// Read out of status register command
#define CLEAR_STATUS_REGISTER_COMMAND 0x3041// Clear status register command

/**
 * @enum      Repeatability
 * @brief     Enumerates the repeatability settings for SHT3x sensor measurements.
 * @details   This enum represents the different repeatability settings that can be used 
 *            for single-shot measurements with the SHT3x sensor. Each setting
 *            corresponds to a different measurement accuracy, measurement duration, noise level,
 *            and energy consumption.
*/
enum Repeatability
{
  e_low,		// Low repeatability for lower accuracy but faster measurement and lower energy consumption.
  e_medium,		// Medium repeatability for moderate accuracy, measurement duration, and energy consumption.
  e_high		// High repeatability for higher accuracy but longer measurement duration and higher energy consumption.
};

/**
 * @class     SHT3x
 * @brief     Class to control the SHT3x-DIS temperature and humidity sensor using I2C communication.
 * @details   This class provides an interface to interact with the SHT3x temperature and humidity sensor
 *            using I2C communication. It allows users to initialize the sensor, perform single-shot
 *            measurements, read temperature and humidity values, and select the repeatability settings.
*/
class SHT3x
{
public:
  /**
   * @brief           Constructs a new SHT3x instance with the specified I2C address, uses default SDA, and SCL pins.
   * @param address   The I2C address of the SHT3x sensor. Can be SHT3X_ADDRESS (0x44) or SHT3X_ADDRESS_B (0x45).
  */
  SHT3x(uint8_t address);

#ifndef __AVR__        //AVR devices must use default SDA/SCL pins as their Wire library does not support anything else, below functions removed if AVR
  /**
   * @brief           Constructs a new SHT3x instance with the specified I2C address, SDA, and SCL pins.
   * @param address   The I2C address of the SHT3x sensor. Can be SHT3X_ADDRESS (0x44) or SHT3X_ADDRESS_B (0x45).
   * @param sda       The SDA pin for I2C communication.
   * @param scl       The SCL pin for I2C communication.
  */
  SHT3x(uint8_t address, uint8_t sda, uint8_t scl);

  /**
   * @brief             Constructs a new SHT3x instance with the specified I2C address, SDA, SCL pins, and clock frequency.
   * @param address     The I2C address of the SHT3x sensor. Can be SHT3X_ADDRESS (0x44) or SHT3X_ADDRESS_B (0x45).
   * @param sda         The SDA pin for I2C communication.
   * @param scl         The SCL pin for I2C communication.
   * @param frequency   The I2C communication frequency in Hz. Can be between 0 and 1000 kHz.
  */
  SHT3x(uint8_t address, uint8_t sda, uint8_t scl, uint32_t frequency);
#endif

  /**
   * @brief     Initalizes the SHT3x sensor for I2C communication.
   * @return    True if initialization is successful, false otherwise.
   * @note      This function should be called before performing any measurements or accessing the sensor.
  */
  bool Initialize();

  /**
   * @brief     Checks if the SHT3x sensor is connected and responsive.
   * @return    True if the sensor is connected and responsive, false otherwise.
  */
  bool IsConnected();

  /**
   * @brief   Performs a soft reset of the SHT3x sensor.
   * @return  True if the soft reset was successful, false otherwise.
  */
  bool SoftReset();

  /**
   * @brief   Reads the status register of the SHT3x sensor.
   * @return  The 16-bit value of the status register if the operation is successful, or 0xFFFF if there is an error.
  */
  uint16_t ReadStatusRegister();

  /**
   * @brief   Clears the flags in the status register of the SHT3x sensor.
   * @return  True if status register was cleared successfully, false otherwise.
  */
  bool ClearStatusRegister();

  /**
   * @brief   Gets the temperature value in Celsius from the SHT3x sensor.
   * @return  The temperature value in Celsius if the measurement is successful, or 
   *          +INFINITY if there is an error during the measurement or communication with the sensor.
  */
  float GetTemperatureCelsius();

  /**
   * @brief   Gets the temperature value in Fahrenheit from the SHT3x sensor.
   * @return  The temperature value in Fahrenheit if the measurement is successful, or 
   *          +INFINITY if there is an error during the measurement or communication with the sensor.
  */
  float GetTemperatureFahrenheit();

  /**
   * @brief   Gets the relative humidity value in %RH from the SHT3x sensor.
   * @return  The relative humidity value if the measurement is successful, or
   *          +INFINITY if there is an error during the measurement or communication with the sensor.
  */
  float GetHumidity();

  /**
   * @brief                 Sets the repeatability mode for temperature and humidity measurements.
   * @param repeatability   The desired repeatability mode to set. It should be one of the following: 
   *                        e_low (0), e_medium (1), or e_high (2).
   * @note                  The default repeatability mode is set to high when the SHT3x sensor is initialized.
  */
  void SetRepeatability(Repeatability repeatability);

private:
  uint8_t _address;					// 0x44 or 0x45
  uint8_t _sda;
  uint8_t _scl;
  uint32_t _frequency;				//Default: 100 kHz

  float _celsius;
  float _fahrenheit;
  float _humidity;

  Repeatability _repeatability;		// Default: e_high

  /**
   * @brief   Performs a temperature and humidity single-shot measurement with clock stretching.
   * @return  True if the measurement was successful, false otherwise.
  */
  bool Measure();

  // Periodic measurement functions are not ready in v1, will be overhauled in next version
  /**
   * @brief   Initiates a periodic measurement at 0.5 measurements per second (mps) with the current repeatability setting.
   * @return  True if the measurement initiation is successful, false otherwise.
  */
  bool MeasurePeriodic();

  // Periodic measurement functions are not ready in v1, will be overhauled in next version
  /**
   * @brief   Initiates a periodic measurement at 1 measurement per second (mps) with the current repeatability setting.
   * @return  True if the measurement initiation is successful, false otherwise.
  */
  bool MeasurePeriodic1MPS();

  // Periodic measurement functions are not ready in v1, will be overhauled in next version
  /**
   * @brief   Initiates a periodic measurement at 2 measurements per second (mps) with the current repeatability setting.
   * @return  True if the measurement initiation is successful, false otherwise.
  */
  bool MeasurePeriodic2MPS();

  // Periodic measurement functions are not ready in v1, will be overhauled in next version
  /**
   * @brief   Initiates a periodic measurement at 4 measurements per second (mps) with the current repeatability setting.
   * @return  True if the measurement initiation is successful, false otherwise.
  */
  bool MeasurePeriodic4MPS();

  // Periodic measurement functions are not ready in v1, will be overhauled in next version
  /**
   * @brief   Initiates a periodic measurement at 10 measurements per second (mps) with the current repeatability setting.
   * @return  True if the measurement initiation is successful, false otherwise.
  */
  bool MeasurePeriodic10MPS();

  // Periodic measurement functions are not ready in v1, will be overhauled in next version
  /**
   * @brief   Fetches the temperature and humidity measurement data after a periodic measurement.
   * @return  True if fetching data was successful, false otherwise.
  */
  bool FetchData();

  // Periodic measurement functions are not ready in v1, will be overhauled in next version
  /**
   * @brief   Initiates a periodic measurement with accelerated response time (ART).
   * @return  True if the measurement initiation is successful, false otherwise.
  */
  bool MeasurePeriodicART();

  // Periodic measurement functions are not ready in v1, will be overhauled in next version
  /**
   * @brief   Stops the periodic data acquisition mode using the break command.
   * @return  True if the break command was successful, false otherwise.
  */
  bool Break();

  /**
   * @brief   Enables the internal heater of the SHT3x sensor.
   * @return  True if the heater was enabled successfully, false otherwise.
  */
  bool EnableHeater();

  /**
   * @brief   Disables the internal heater of the SHT3x sensor.
   * @return  True if the heater was disabled successfully, false otherwise.
  */
  bool DisableHeater();

  /**
   * @brief           Writes a command to the SHT3x sensor through I2C communication.
   * @param command   The 16-bit command to be sent to the SHT3x sensor.
   * @return          True if the command was successfully sent and acknowledged by the sensor, false otherwise.
  */
  bool WriteCommand(uint16_t command);

  /**
   * @brief           Read three bytes from the SHT3x sensor via I2C communication.
   * @param msb       Pointer to a variable where the MSB of the data will be stored.
   * @param lsb       Pointer to a variable where the LSB of the data will be stored.
   * @param checksum  Pointer to a variable where the checksum byte will be stored.
   * @return          True if all three bytes are successfully read and available, false otherwise.
  */
  bool Read3Bytes(uint8_t* msb, uint8_t* lsb, uint8_t* checksum);

  /**
   * @brief             Read six bytes from the SHT3x sensor via I2C communication.
   * @param msb1        Pointer to a variable where the first MSB of the data will be stored.
   * @param lsb1        Pointer to a variable where the first LSB of the data will be stored.
   * @param checksum1   Pointer to a variable where the first checksum byte will be stored.
   * @param msb2        Pointer to a variable where the second MSB of the data will be stored.
   * @param lsb2        Pointer to a variable where the second LSB of the data will be stored.
   * @param checksum2   Pointer to a variable where the second checksum byte will be stored.
   * @return            True if all six bytes are successfully read and available, false otherwise.
  */
  bool Read6Bytes(uint8_t* msb1, uint8_t* lsb1, uint8_t* checksum1, uint8_t* msb2, uint8_t* lsb2, uint8_t* checksum2);

  /**
   * @brief           Extracts the MSB from a 16-bit command value.
   * @param command   The 16-bit command value from which to extract the MSB.
   * @return          The MSB of the given 16-bit command value.
  */
  uint8_t GetMSB(uint16_t command);

  /**
   * @brief           Extracts the LSB from a 16-bit command value.
   * @param command   The 16-bit command value from which to extract the LSB.
   * @return          The LSB of the given 16-bit command value.
  */
  uint8_t GetLSB(uint16_t command);

  /**
   * @brief           Converts the raw temperature value to Celsius.
   * @param rawValue  The 16-bit raw temperature value from the SHT3x sensor.
   * @return          The temperature value in degrees Celsius.
  */
  float RawValueToCelsius(uint16_t rawValue);

  /**
   * @brief           Converts the raw temperature value to Fahrenheit.
   * @param rawValue  The 16-bit raw temperature value from the SHT3x sensor.
   * @return          The temperature value in degrees Fahrenheit.
  */
  float RawValueToFahrenheit(uint16_t rawValue);

  /**
   * @brief           Converts the raw humidity value to relative humidity (%RH).
   * @param rawValue  The 16-bit raw humidity value from the SHT3x sensor.
   * @return          The relative humidity value as a percentage (%RH).
  */
  float RawValueToHumidity(uint16_t rawValue);

  /**
   * @brief       Calculates the CRC-8 checksum for a 16-bit data value.
   * @param data  The 16-bit data value for which the CRC-8 checksum needs to be calculated.
   * @return      The CRC-8 checksum value.
  */
  uint8_t CalculateCRC8(uint16_t data);

  /**
   * @brief           Verifies the CRC-8 checksum for two bytes of data.
   * @param msb       The MSB of the data.
   * @param lsb       The LSB of the data.
   * @param checksum  The received CRC-8 checksum to be verified.
   * @return          True if the calculated CRC-8 checksum matches the received checksum, false otherwise.
  */
  bool VerifyCRC8(uint8_t msb, uint8_t lsb, uint8_t checksum);

};

#endif /* _REMAL_SHT3X_H_ */