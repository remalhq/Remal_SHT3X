/*
 * Remal_SHT3X.cpp
 *
 *  # ALL INFO CAN BE FOUND IN THE HEADER FILE #
 */
#include "Remal_SHT3X.h"
#include <math.h>


// Public Functions

/**
 * @details The constructor initializes the SHT3x sensor with the given I2C address and default communication pins.
 *          It sets the default clock frequency to 100 kHz, the default repeatability mode to high, and the default periodic frequency to 10 mps.
*/
SHT3x::SHT3x(uint8_t address)
{
    _address = address;
    _sda = SDA;
    _scl = SCL;
    _frequency = 100000;// Default clock frequency set to 100 kHz
    _celsius = 0;
    _fahrenheit = 0;
    _humidity = 0;
    _repeatability = e_high;// Set default repeatability to high
    _periodicFrequency = e_10mps; // Set default periodic frequency to 10 mps
}

#ifndef __AVR__        //AVR devices must use default SDA/SCL pins as their Wire library does not support anything else, below functions removed if AVR
/**
 * @details The constructor initializes the SHT3x sensor with the given I2C address and communication pins.
 *          It sets the default clock frequency to 100 kHz, the default repeatability mode to high, and the default periodic frequency to 10 mps.
*/
SHT3x::SHT3x(uint8_t address, uint8_t sda, uint8_t scl)
{
    _address = address;
    _sda = sda;
    _scl = scl;
    _frequency = 100000;// Default clock frequency set to 100 kHz
    _celsius = 0;
    _fahrenheit = 0;
    _humidity = 0;
    _repeatability = e_high;// Set default repeatability to high
    _periodicFrequency = e_10mps; // Set default periodic frequency to 10 mps
}

/**
 * @details The constructor initializes the SHT3x sensor with the given I2C address, communication pins, and
 *          communication frequency. It also sets the default repeatability mode to high and the default periodic frequency to 10 mps.
*/
SHT3x::SHT3x(uint8_t address, uint8_t sda, uint8_t scl, uint32_t frequency)
{
    _address = address;
    _sda = sda;
    _scl = scl;
    _frequency = frequency;
    _celsius = 0;
    _fahrenheit = 0;
    _humidity = 0;
    _repeatability = e_high;// Set default repeatability to high
    _periodicFrequency = e_10mps; // Set default periodic frequency to 10 mps
}
#endif

/**
 * @details This function initializes the SHT3x sensor by starting the I2C communication with the specified
 *          SDA and SCL pins at the given frequency. It calls the break command, stopping the periodic mode if it was previously active.
 *          It also performs a soft reset on the sensor to ensure that it is in a known state before further operations. 
 *          The function returns true if the initialization is successful, and false otherwise.
*/
bool SHT3x::Initialize()
{  
#ifdef __AVR__      //AVR devices must use default SDA/SCL pins as their Wire library does not support anything else
    Wire.begin();
#else
    Wire.begin(_sda, _scl, _frequency);
#endif

    if(!StopPeriodic())// Break needed to end periodic mode in case it was running
    {
        return false;// Error
    }

    // SoftReset function also calls ClearStatusRegisterFunction
    if(!SoftReset())
    {
        return false;// Error
    }

    return true;
}

/**
 * @details This function checks if the SHT3x sensor is connected and responsive by reading its status register.
 *          If the status register value is 0xFFFF, it indicates that the sensor is not responding or connected.
 *          In such a case, the function returns false. Otherwise, it returns true, indicating that the sensor is
 *          connected and ready for communication.
*/
bool SHT3x::IsConnected()
{
    if(ReadStatusRegister() == 0xFFFF)
    {
        return false;// Error
    }
    return true;
}

/**
 * @details This function sends the soft reset command to the SHT3x sensor, which resets the sensor to its default
 *          configuration and puts it into a known state. After the soft reset, the sensor needs a short delay to
 *          complete the reset process before further operations can be performed. The function waits for a delay
 *          of 2 milliseconds, which is sufficient for the soft reset to complete. 
 *          The function also initializes the alert limits based on the data sheet standard values.
*/
bool SHT3x::SoftReset()
{
    if(!WriteCommand(SOFT_RESET_COMMAND))
    {
        return false;// Error
    }
    delay(2);// Soft reset time: typ = 0.5 ms, max = 1.5 ms
    
    // Called on reset and initialization, based on the data sheet standard values

    _humidityHighSetLimit = 80;
    _humidityHighClearLimit = 79;
    _humidityLowClearLimit = 22;
    _humidityLowSetLimit = 20;
  
    _temperatureHighSetLimit = 60;
    _temperatureHighClearLimit = 58;
    _temperatureLowClearLimit = -9;
    _temperatureLowSetLimit = -10;

    // Clear the status register to ensure proper functionality of ALERT pin
    if(!ClearStatusRegister())
    {
        return false;// Error
    }

    return true;
}

/**
 * @details This function sends the command to read the status register of the SHT3x sensor. If the command is
 *          successfully sent and the sensor responds with the status register data, the function returns the
 *          16-bit value of the status register. If there is an error during communication with the sensor, the
 *          function returns 0xFFFF to indicate the error.
*/
uint16_t SHT3x::ReadStatusRegister()
{
    if(!WriteCommand(READ_STATUS_REGISTER_COMMAND))
    {
        return 0xFFFF;// Error
    }
    
    uint8_t msb, lsb, checksum;

    if(!Read3Bytes(&msb, &lsb, &checksum))
    {
        return 0xFFFF;// Error
    }
    return (CombineBytes(msb, lsb));
}

/**
 * @details This function sends the command to clear the status register of the SHT3x sensor.
 *          The command clears (sets to zero) all of the flags (bits 15, 11, 10, 4) in the status register.
*/
bool SHT3x::ClearStatusRegister()
{
    if(!WriteCommand(CLEAR_STATUS_REGISTER_COMMAND))
    {
        return false;// Error
    }
    return true;
}

/**
 * @details This function reads the temperature value from the SHT3x sensor. It first calls the `Measure()` function
 *          to perform a measurement. If the measurement is successful, it returns the temperature value in Celsius.
 *          If there is an error during the measurement or communication with the sensor, the function returns
 *          +INFINITY to indicate the error.
*/
float SHT3x::GetTemperatureCelsius()
{
    if(!Measure())
    {
        return +INFINITY;// Error
    }
    return _celsius;
}

/**
 * @details This function reads the temperature value from the SHT3x sensor. It first calls the `Measure()` function
 *          to perform a measurement. If the measurement is successful, it returns the temperature value in Fahrenheit.
 *          If there is an error during the measurement or communication with the sensor, the function returns
 *          +INFINITY to indicate the error.
*/
float SHT3x::GetTemperatureFahrenheit()
{
    if(!Measure())
    {
        return +INFINITY;// Error
    }
    return _fahrenheit;
}

/**
 * @details This function reads the relative humidity value from the SHT3x sensor. It first calls the `Measure()` function
 *          to perform a measurement. If the measurement is successful, it returns the relative humidity value in percentage.
 *          If there is an error during the measurement or communication with the sensor, the function returns
 *          +INFINITY to indicate the error.
*/
float SHT3x::GetHumidity()
{
    if(!Measure())
    {
        return +INFINITY;// Error
    }
    return _humidity;
}

/**
 * @details This function allows you to set the repeatability mode for temperature and humidity measurements
 *          on the SHT3x sensor. The repeatability mode determines the measurement accuracy, duration,
 *          noise level, and energy consumption. You can choose between three repeatability modes: low, medium, and high,
 *          by passing the appropriate `Repeatability` enum value as the parameter.
*/
void SHT3x::SetRepeatability(Repeatability repeatability)
{
    _repeatability = repeatability;
}

/**
 * @details This function allows you to set the desired periodic measurement frequency for the SHT3x sensor.
 *          The provided `periodicFrequency` parameter should be a value from the `PeriodicFrequency` enum
 *          specifying the desired measurement frequency. The periodic measurement frequency can be
 *          0.5 mps, 1 mps, 2 mps, 4 mps, or 10 mps.
*/
void SHT3x::SetPeriodicFrequency(PeriodicFrequency periodicFrequency)
{
    _periodicFrequency = periodicFrequency;
}

/**
 * @details This function sets the high alert limits for both humidity and temperature on the SHT3x sensor.
 *          The provided `humidity` parameter should be a value within the range of 0 to 100, representing
 *          the desired high alert humidity limit in percentage. The `temperature` parameter should be a value
 *          within the range of -40 to 120, representing the desired high alert temperature limit in degrees Celsius.
 *          If the provided parameter values are outside these valid ranges, the function will return an error.
*/
bool SHT3x::SetHighAlertLimit(float humidity, float temperature)
{
    if(humidity < 0 || humidity > 100)// Invalid humidity range
    {
        return false;// Error
    }
    if(temperature < -40 || temperature > 120)// Invalid temperature range
    {
        return false;// Error
    }

    // Calculate limit
    uint16_t limit = CalculateLimit(humidity, temperature);
    
    // Write to registers
    if(!WriteAlertLimit(ALERT_HIGH_CLEAR_WRITE, limit))
    {
        return false;// Error
    }
    if(!WriteAlertLimit(ALERT_HIGH_SET_WRITE, limit))
    {
        return false;// Error
    }
    return true;
}

/**
 * @details This function sets the high alert limits for both humidity and temperature on the SHT3x sensor. Unlike the single
 *          `SetHighAlertLimit` function, this function allows separate set and clear limits for both humidity and temperature.
 *          The provided `humiditySet` parameter should be a value within the range of 0 to 100, representing the desired
 *          high alert humidity set limit in percentage. The `humidityClear` parameter should also be within the same range,
 *          representing the desired high alert humidity clear limit. Similarly, the `temperatureSet` parameter should be a
 *          value within the range of -40 to 120, representing the desired high alert temperature set limit in degrees Celsius,
 *          and the `temperatureClear` parameter should be within the same range, representing the desired high alert temperature
 *          clear limit. If any of the provided parameter values are outside these valid ranges, the function will return an error.
*/
bool SHT3x::SetHighAlertLimit(float humiditySet, float humidityClear, float temperatureSet, float temperatureClear)
{
    if(humiditySet < 0 || humiditySet > 100)// Invalid humidity set range
    {
        return false;// Error
    }
    if(humidityClear < 0 || humidityClear > 100)// Invalid humidity clear range
    {
        return false;// Error
    }
    if(temperatureSet < -40 || temperatureSet > 120)// Invalid temperature set range
    {
        return false;// Error
    }
    if(temperatureClear < -40 || temperatureClear > 120)// Invalid temperature clear range
    {
        return false;// Error
    }

    // Calculate limits
    uint16_t setLimit = CalculateLimit(humiditySet, temperatureSet);
    uint16_t clearLimit = CalculateLimit(humidityClear, temperatureClear);
    
    // Write to registers
    if(!WriteAlertLimit(ALERT_HIGH_SET_WRITE, setLimit))
    {
        return false;// Error
    }
    if(!WriteAlertLimit(ALERT_HIGH_CLEAR_WRITE, clearLimit))
    {
        return false;// Error
    }
    
    return true;
}

/**
 * @details This function sets the low alert limits for both humidity and temperature on the SHT3x sensor. The provided
 *          `humidity` parameter should be a value within the range of 0 to 100, representing the desired low alert humidity
 *          limit in percentage. The `temperature` parameter should be within the range of -40 to 120, representing the
 *          desired low alert temperature limit in degrees Celsius. If either of the provided parameter values are outside
 *          these valid ranges, the function will return an error.
*/
bool SHT3x::SetLowAlertLimit(float humidity, float temperature)
{
    if(humidity < 0 || humidity > 100)// Invalid humidity range
    {
        return false;// Error
    }
    if(temperature < -40 || temperature > 120)// Invalid temperature range
    {
        return false;// Error
    }

    // Calculate limit
    uint16_t limit = CalculateLimit(humidity, temperature);
    
    // Write to registers
    if(!WriteAlertLimit(ALERT_LOW_CLEAR_WRITE, limit))
    {
        return false;// Error
    }
    if(!WriteAlertLimit(ALERT_LOW_SET_WRITE, limit))
    {
        return false;// Error
    }
    return true;
}

/**
 * @details This function sets the low alert limits for both humidity and temperature on the SHT3x sensor. Two pairs of
 *          parameters are provided: `humidityClear` and `temperatureClear` represent the clear limits, while `humiditySet`
 *          and `temperatureSet` represent the set limits. The parameters `humidityClear` and `humiditySet` should be values
 *          within the range of 0 to 100, representing the desired low alert humidity clear and set limits in percentage.
 *          The `temperatureClear` and `temperatureSet` parameters should be within the range of -40 to 120, representing
 *          the desired low alert temperature clear and set limits in degrees Celsius. If any of the provided parameter
 *          values are outside these valid ranges, the function will return an error.
*/
bool SHT3x::SetLowAlertLimit(float humidityClear, float humiditySet, float temperatureClear, float temperatureSet)
{
    if(humidityClear < 0 || humidityClear > 100)// Invalid humidity clear range
    {
        return false;// Error
    }
    if(humiditySet < 0 || humiditySet > 100)// Invalid humidity set range
    {
        return false;// Error
    }
    if(temperatureClear < -40 || temperatureClear > 120)// Invalid temperature clear range
    {
        return false;// Error
    }
    if(temperatureSet < -40 || temperatureSet > 120)// Invalid temperature set range
    {
        return false;// Error
    }

    // Calculate limits
    uint16_t clearLimit = CalculateLimit(humidityClear, temperatureClear);
    uint16_t setLimit = CalculateLimit(humiditySet, temperatureSet);
    
    // Write to registers
    if(!WriteAlertLimit(ALERT_LOW_CLEAR_WRITE, clearLimit))
    {
        return false;// Error
    }
    if(!WriteAlertLimit(ALERT_LOW_SET_WRITE, setLimit))
    {
        return false;// Error
    }
    return true;
}

/**
 * @details This function reads and returns the high alert humidity set limit that is stored on the SHT3x sensor.
 *          If the sensor fails to respond or an error occurs during the read operation, the function returns
 *          positive infinity (+INFINITY).
*/
float SHT3x::GetHumidityHighSetLimit()
{
    if(!ReadHighSetAlertLimit())
    {
        return +INFINITY;// Error
    }
    return ceil(_humidityHighSetLimit);
}

/**
 * @details This function reads and returns the high alert humidity clear limit that is stored on the SHT3x sensor.
 *          If the sensor fails to respond or an error occurs during the read operation, the function returns
 *          positive infinity (+INFINITY).
*/
float SHT3x::GetHumidityHighClearLimit()
{
    if(!ReadHighClearAlertLimit())
    {
        return +INFINITY;// Error
    }
    return ceil(_humidityHighClearLimit);
}

/**
 * @details This function reads and returns the low alert humidity clear limit that is stored on the SHT3x sensor.
 *          If the sensor fails to respond or an error occurs during the read operation, the function returns
 *          positive infinity (+INFINITY).
*/
float SHT3x::GetHumidityLowClearLimit()
{
    if(!ReadLowClearAlertLimit())
    {
        return +INFINITY;// Error
    }
    return ceil(_humidityLowClearLimit);
}

/**
 * @details This function reads and returns the low alert humidity set limit that is stored on the SHT3x sensor.
 *          If the sensor fails to respond or an error occurs during the read operation, the function returns
 *          positive infinity (+INFINITY).
*/
float SHT3x::GetHumidityLowSetLimit()
{
    if(!ReadLowSetAlertLimit())
    {
        return +INFINITY;// Error
    }
    return ceil(_humidityLowSetLimit);
}

/**
 * @details This function reads and returns the high alert temperature set limit that is stored on the SHT3x sensor.
 *          If the sensor fails to respond or an error occurs during the read operation, the function returns
 *          positive infinity (+INFINITY).
*/
float SHT3x::GetTemperatureHighSetLimit()
{
    if(!ReadHighSetAlertLimit())
    {
        return +INFINITY;// Error
    }
    return ceil(_temperatureHighSetLimit);
}

/**
 * @details This function reads and returns the high alert temperature clear limit that is stored on the SHT3x sensor.
 *          If the sensor fails to respond or an error occurs during the read operation, the function returns
 *          positive infinity (+INFINITY).
*/
float SHT3x::GetTemperatureHighClearLimit()
{
    if(!ReadHighClearAlertLimit())
    {
        return +INFINITY;// Error
    }
    return ceil(_temperatureHighClearLimit);
}

/**
 * @details This function reads and returns the low alert temperature clear limit that is stored on the SHT3x sensor.
 *          If the sensor fails to respond or an error occurs during the read operation, the function returns
 *          positive infinity (+INFINITY).
*/
float SHT3x::GetTemperatureLowClearLimit()
{
    if(!ReadLowClearAlertLimit())
    {
        return +INFINITY;// Error
    }
    return ceil(_temperatureLowClearLimit);
}

/**
 * @details This function reads and returns the low alert temperature set limit that is stored on the SHT3x sensor.
 *          If the sensor fails to respond or an error occurs during the read operation, the function returns
 *          positive infinity (+INFINITY).
*/
float SHT3x::GetTemperatureLowSetLimit()
{
    if(!ReadLowSetAlertLimit())
    {
        return +INFINITY;// Error
    }
    return ceil(_temperatureLowSetLimit);
}

/**
 * @details This function initiates the periodic measurement mode on the SHT3x sensor using the previously set measurement
 *          frequency. The selected measurement frequency is determined by the value of `_periodicFrequency`,
 *          which can be changed using the 'SetPeriodicFrequency()' function before starting the periodic mode.
*/
bool SHT3x::StartPeriodic()
{
    switch(_periodicFrequency)
    {
        case e_halfmps:
            if(!MeasurePeriodicHalfMPS())
            {
                return false;// Error
            }
            break;
        case e_1mps:
            if(!MeasurePeriodic1MPS())
            {
                return false;// Error
            }
            break;
        case e_2mps:
            if(!MeasurePeriodic2MPS())
            {
                return false;// Error
            }
            break;
        case e_4mps:
            if(!MeasurePeriodic4MPS())
            {
                return false;// Error
            }
            break;
        case e_10mps:
            if(!MeasurePeriodic10MPS())
            {
                return false;// Error
            }
            break;
        default:
            return false;// Error
            break;
    }
    return true;
}

/**
 * @details This function initiates a periodic measurement with accelerated response time (ART).
 *          After calling this function, the sensor will start acquiring data with a frequency of 4 Hz.
*/
bool SHT3x::StartPeriodicART()
{
    if(!WriteCommand(ART_COMMAND))
    {
        return false;// Error
    }
    return true;
}

/**
 * @details This function sends the break command to the SHT3x sensor, which stops the periodic data acquisition mode.
 *          When the sensor is in periodic data acquisition mode, it continuously performs measurements at the specified
 *          measurement rate. Calling this function will halt the periodic measurements until another measurement command
 *          is issued. It is recommended to stop the periodic data acquisition prior to sending another command except fetch.
*/
bool SHT3x::StopPeriodic()
{
    if(!WriteCommand(BREAK_COMMAND))
    {
        return false;// Error
    }
    delay(1);// It takes 1 ms to abort periodic mode and enter single shot mode (data sheet section 4.8)
    return true;
}

/**
 * @details This function retrieves the temperature value from the most recent periodic measurement.
 *          It calls the `FetchData()` function to fetch the data from the sensor and then returns
 *          the temperature value in degrees Celsius.
*/
float SHT3x::GetTemperaturePeriodic()
{
    if(!FetchData())
    {
        return +INFINITY;// Error
    }
    return _celsius;
}

/**
 * @details This function retrieves the humidity value from the most recent periodic measurement.
 *          It calls the `FetchData()` function to fetch the data from the sensor and then returns
 *          the humidity value as a percentage.
*/
float SHT3x::GetHumidityPeriodic()
{
    if(!FetchData())
    {
        return +INFINITY;// Error
    }
    return _humidity;
}

// Private Functions

/**
 * @details This function performs a temperature and humidity measurement in single-shot mode on the SHT3x sensor
 *          based on the currently set repeatability mode. It selects the appropriate measurement command according
 *          to the chosen repeatability setting and reads the measurement results from the sensor.
*/
bool SHT3x::Measure()
{
    switch(_repeatability)
    {
        case e_low:
            if(!WriteCommand(MEAS_LOW_REP_CS_ENABLED))
            {
                return false;// Error
            }
            break;
        case e_medium:
            if(!WriteCommand(MEAS_MID_REP_CS_ENABLED))
            {
                return false;// Error
            }
            break;
        case e_high:
            if(!WriteCommand(MEAS_HI_REP_CS_ENABLED))
            {
                return false;// Error
            }
            break;

        default:
            return false;// Error
            break;
    }

    uint8_t tempmsb, templsb, tempchecksum, humiditymsb, humiditylsb, humiditychecksum;

    if(!Read6Bytes(&tempmsb, &templsb, &tempchecksum, &humiditymsb, &humiditylsb, &humiditychecksum))
    {
        return false;// Error
    }
    _celsius = RawValueToCelsius(CombineBytes(tempmsb, templsb));
    _fahrenheit = RawValueToFahrenheit(CombineBytes(tempmsb, templsb));
    _humidity = RawValueToHumidity(CombineBytes(humiditymsb, humiditylsb));

    return true;
}

/**
 * @details This function sends a command to the SHT3x sensor to enable its internal heater. The heater can be
 *          used for plausibility checking purposes only and should not be used to modify the measurement values
 *          of temperature and humidity. After calling this function, the heater will be activated, and the sensor
 *          will start generating heat.
*/
bool SHT3x::EnableHeater()
{
    if(!WriteCommand(HEATER_ENABLE_COMMAND))
    {
        return false;// Error
    }
    return true;
}

/**
 * @details This function sends a command to the SHT3x sensor to disable its internal heater. The heater can be
 *          used for plausibility checking purposes only and should not be used to modify the measurement values
 *          of temperature and humidity. After calling this function, the heater will be deactivated, and the sensor
 *          will stop generating heat.
*/
bool SHT3x::DisableHeater()
{
    if(!WriteCommand(HEATER_DISABLE_COMMAND))
    {
        return false;// Error
    }
    return true;
}

/**
 * @details This function sends a 16-bit command to the SHT3x sensor through the I2C bus. It first begins the I2C
 *          transmission, then writes the most significant byte (MSB) of the command followed by the least significant
 *          byte (LSB). After writing the command, it ends the I2C transmission and checks if the command was successfully
 *          acknowledged by the sensor.
*/
bool SHT3x::WriteCommand(uint16_t command)
{
    delay(1);// Needed to give time for the sensor to respond
    Wire.beginTransmission(_address);
    Wire.write(GetMSB(command));// Write the MSB
    Wire.write(GetLSB(command));// Write the LSB

    if(Wire.endTransmission() == 0)
    {
        return true;
    }
    else
    {
        return false;// Error
    }
}

/**
 * @details This function reads three bytes of data (MSB, LSB, and checksum) from the SHT3x sensor
 *          using the I2C communication protocol. The data is stored in the provided variables pointed
 *          to by `msb`, `lsb`, and `checksum`. The function returns true if the data is successfully
 *          read and available, and false otherwise.
*/
bool SHT3x::Read3Bytes(uint8_t* msb, uint8_t* lsb, uint8_t* checksum)
{
   Wire.requestFrom(_address, 3);// Request 3 bytes of data: MSB, LSB, checksum
   if(Wire.available() == 3)
   {
        *msb = Wire.read();// Read MSB
        *lsb = Wire.read();// Read LSB
        *checksum = Wire.read();// Read checksum

        return true;
   }
   else
   {
        return false;// Error
   }
}

/**
 * @details This function reads six bytes of data (MSB1, LSB1, checksum1, MSB2, LSB2, and checksum2)
 *          from the SHT3x sensor using the I2C communication protocol. The data is stored in the provided
 *          variables pointed to by `msb1`, `lsb1`, `checksum1`, `msb2`, `lsb2`, and `checksum2`. The function
 *          returns true if all six bytes are successfully read and available, and false otherwise.
*/
bool SHT3x::Read6Bytes(uint8_t* msb1, uint8_t* lsb1, uint8_t* checksum1, uint8_t* msb2, uint8_t* lsb2, uint8_t* checksum2)
{
   Wire.requestFrom(_address, 6);// Request 6 bytes of data: MSB1, LSB1, checksum1, MSB2, LSB2, checksum2
   if(Wire.available() == 6)
   {
        *msb1 = Wire.read();// Read MSB1
        *lsb1 = Wire.read();// Read LSB1
        *checksum1 = Wire.read();// Read checksum1

        *msb2 = Wire.read();// Read MSB2
        *lsb2 = Wire.read();// Read LSB2
        *checksum2 = Wire.read();// Read checksum2

        return true;
   }
   else
   {
        return false;// Error
   }
}

/**
 * @details This function takes a 16-bit command value as input and returns the most significant byte (MSB)
 *          of the command. The function is used to extract the MSB from the command value, 
 *          which is then used in I2C communication with the SHT3x sensor.
*/
uint8_t SHT3x::GetMSB(uint16_t command)
{
    // Shift the command value 8 bits to the right and use a bitwise AND operation to extract the MSB
    return ((command >> 8) & 0xFF);
}

/**
 * @details This function takes a 16-bit command value as input and returns the least significant byte (LSB)
 *          of the command. The function is used to extract the LSB from the command value, 
 *          which is then used in I2C communication with the SHT3x sensor.
*/
uint8_t SHT3x::GetLSB(uint16_t command)
{
    // Use a bitwise AND operation to extract the LSB from the command value
    return (command & 0xFF);
}

/**
 * @details This function takes a most significant byte (MSB) and a least significant byte (LSB) as input,
 *          and combines them to create a 16-bit unsigned integer value.
*/
uint16_t SHT3x::CombineBytes(uint8_t msb, uint8_t lsb)
{
    return ((msb << 8) | lsb);
}

/**
 * @details This function takes a 16-bit raw temperature value from the SHT3x sensor and converts it to Celsius.
 *          The raw value is first divided by the resolution (2^16 - 1), then multiplied by 175.0,
 *          and finally subtracted by 45 to obtain the temperature in degrees Celsius.
*/
float SHT3x::RawValueToCelsius(uint16_t rawValue)
{
    return (((rawValue / 65535.0) * 175.0) - 45);
}

/**
 * @details This function takes a 16-bit raw temperature value from the SHT3x sensor and converts it to Fahrenheit.
 *          The raw value is first divided by the resolution (2^16 - 1), then multiplied by 315.0,
 *          and finally subtracted by 49 to obtain the temperature in degrees Fahrenheit.
*/
float SHT3x::RawValueToFahrenheit(uint16_t rawValue)
{
    return (((rawValue / 65535.0) * 315.0) - 49);
}

/**
 * @details This function takes a 16-bit raw humidity value from the SHT3x sensor and converts it to relative humidity.
 *          The raw value is first divided by the resolution (2^16 - 1), then multiplied by 100.0 to
 *          obtain the relative humidity percentage.
*/
float SHT3x::RawValueToHumidity(uint16_t rawValue)
{
    return ((rawValue / 65535.0) * 100.0);
}

/**
 * @details This function calculates the CRC-8 checksum for a 16-bit data value using the CRC-8 algorithm.
 *          It takes the data value as input and returns the computed CRC-8 checksum.
*/
uint8_t SHT3x::CalculateCRC8(uint16_t data)
{
    uint8_t crc = 0xFF;

    // Calculate the CRC over the two bytes of data

    uint8_t currentByte = GetMSB(data);
    crc ^= currentByte;

    for (int j = 0; j < 8; j++)
    {
        if (crc & 0x80)
        {
            crc = (crc << 1) ^ 0x131;
        }
        else
        {
            crc = (crc << 1);
        }
    }

    currentByte = GetLSB(data);
    crc ^= currentByte;

    for (int j = 0; j < 8; j++)
    {
        if (crc & 0x80)
        {
            crc = (crc << 1) ^ 0x131;
        }
        else
        {
            crc = (crc << 1);
        }
    }

    return crc ^ 0x00;
}

/**
 * @details This function calculates the combined alert limit for both humidity and temperature
 *          based on the provided humidity and temperature values. It first converts the provided
 *          humidity and temperature values to their respective raw values using appropriate conversion
 *          functions. Then, it removes the least significant bits from both raw values to match
 *          the required bit positions for the alert limits. Finally, it combines the 7 most significant
 *          bits of the humidity raw value with the 9 least significant bits of the temperature raw value
 *          to create the 16-bit alert limit.
*/
uint16_t SHT3x::CalculateLimit(float humidity, float temperature)
{
    uint16_t rawHumidity = HumidityToRawValue(humidity);// Convert the relative humidity to the raw value
    uint16_t rawTemperature = CelsiusToRawValue(temperature);// Convert the temperature to the raw value

    // Remove 9 lsbs of humidity
    // Remove 7 lsbs of temp, shift to the right by 7 bits
    // Combine (OR) them to get the limit
    uint16_t limit = ((rawHumidity & 0b1111111000000000) | ((rawTemperature >> 7) & 0b0000000111111111));
    // Limit: 7 msbs are the humidity limit, 9 lsbs are the temperature limit
    return limit;
}

/**
 * @details This function writes the provided combined alert limit to the specified command register
 *          of the SHT3x sensor. It calculates the checksum based on the provided limit and includes
 *          it in the transmitted data. The function begins the I2C transmission, writing the command
 *          MSB, LSB, the limit MSB, LSB, and the checksum. It then checks the result of the transmission 
 *          and returns success or error accordingly.
*/
bool SHT3x::WriteAlertLimit(uint16_t command, uint16_t limit)
{
    // Calculate the checksum based on the limit
    uint8_t checksum = CalculateCRC8(limit);

    delay(1);// Needed to give time for the sensor to respond
    Wire.beginTransmission(_address);

    Wire.write(GetMSB(command));// Write the command MSB
    Wire.write(GetLSB(command));// Write the command LSB
    
    Wire.write(GetMSB(limit));// Write the 7 humidity bits + 1 temperature bit
    Wire.write(GetLSB(limit));// Write the remaining 8 temperature bits

    Wire.write(checksum);// Write the checksum

    if(Wire.endTransmission() == 0)
    {
        return true;
    }
    else
    {
        return false;// Error
    }
}

/**
 * @details This function reads the high alert set limit from the sensor. It sends the appropriate
 *          command to request the data, reads the received bytes, and then extracts and stores the humidity
 *          and temperature limits based on the received data. The humidity limit is extracted from the 7
 *          humidity bits (bits 15-9) of the combined limit, while the temperature limit is extracted from
 *          the 9 temperature bits (bits 8-0) of the combined limit.
*/
bool SHT3x::ReadHighSetAlertLimit()
{
    if(!WriteCommand(ALERT_HIGH_SET_READ))
    {
        return false;// Error
    }

    uint8_t msb, lsb, checksum;
    if(!Read3Bytes(&msb, &lsb, &checksum))
    {
        return false;// Error
    }

    uint16_t limit = CombineBytes(msb, lsb);

    _humidityHighSetLimit = RawValueToHumidity((limit & 0b1111111000000000));// 7 humidity bits (bits 15-9)
    _temperatureHighSetLimit = RawValueToCelsius((limit << 7) & 0b1111111110000000);// 9 temperature bits (bits 8-0)

    return true;
}

/**
 * @details This function reads the high alert clear limit from the sensor. It sends the appropriate
 *          command to request the data, reads the received bytes, and then extracts and stores the humidity
 *          and temperature limits based on the received data. The humidity limit is extracted from the 7
 *          humidity bits (bits 15-9) of the combined limit, while the temperature limit is extracted from
 *          the 9 temperature bits (bits 8-0) of the combined limit.
*/
bool SHT3x::ReadHighClearAlertLimit()
{
    if(!WriteCommand(ALERT_HIGH_CLEAR_READ))
    {
        return false;// Error
    }

    uint8_t msb, lsb, checksum;
    if(!Read3Bytes(&msb, &lsb, &checksum))
    {
        return false;// Error
    }

    uint16_t limit = CombineBytes(msb, lsb);

    _humidityHighClearLimit = RawValueToHumidity((limit & 0b1111111000000000));// 7 humidity bits (bits 15-9)
    _temperatureHighClearLimit = RawValueToCelsius((limit << 7) & 0b1111111110000000);// 9 temperature bits (bits 8-0)

    return true;
}

/**
 * @details This function reads the low alert clear limit from the sensor. It sends the appropriate
 *          command to request the data, reads the received bytes, and then extracts and stores the humidity
 *          and temperature limits based on the received data. The humidity limit is extracted from the 7
 *          humidity bits (bits 15-9) of the combined limit, while the temperature limit is extracted from
 *          the 9 temperature bits (bits 8-0) of the combined limit.
*/
bool SHT3x::ReadLowClearAlertLimit()
{
    if(!WriteCommand(ALERT_LOW_CLEAR_READ))
    {
        return false;// Error
    }

    uint8_t msb, lsb, checksum;
    if(!Read3Bytes(&msb, &lsb, &checksum))
    {
        return false;// Error
    }

    uint16_t limit = CombineBytes(msb, lsb);

    _humidityLowClearLimit = RawValueToHumidity(limit & 0b1111111000000000);// 7 humidity bits (bits 15-9)
    _temperatureLowClearLimit = RawValueToCelsius((limit << 7) & 0b1111111110000000);// 9 temperature bits (bits 8-0)

    return true;
}

/**
 * @details This function reads the low alert set limit from the sensor. It sends the appropriate
 *          command to request the data, reads the received bytes, and then extracts and stores the humidity
 *          and temperature limits based on the received data. The humidity limit is extracted from the 7
 *          humidity bits (bits 15-9) of the combined limit, while the temperature limit is extracted from
 *          the 9 temperature bits (bits 8-0) of the combined limit.
*/
bool SHT3x::ReadLowSetAlertLimit()
{
    if(!WriteCommand(ALERT_LOW_SET_READ))
    {
        return false;// Error
    }

    uint8_t msb, lsb, checksum;
    if(!Read3Bytes(&msb, &lsb, &checksum))
    {
        return false;// Error
    }

    uint16_t limit = CombineBytes(msb, lsb);

    _humidityLowSetLimit = RawValueToHumidity(limit & 0b1111111000000000);// 7 humidity bits (bits 15-9)
    _temperatureLowSetLimit = RawValueToCelsius((limit << 7) & 0b1111111110000000);// 9 temperature bits (bits 8-0)

    return true;
}

/**
 * @details This function converts a temperature value in Celsius to its raw value representation as required by
 *          the sensor. The temperature value is shifted by 45.0 and then scaled by 65535.0 divided by the range
 *          (175.0), which results in the corresponding raw value.
*/
uint16_t SHT3x::CelsiusToRawValue(float temperature)
{
    return (((temperature + 45.0) / 175.0) * 65535.0);
}

/**
 * @details This function converts a humidity value in percentage to its raw value representation as required by
 *          the sensor. The humidity value is scaled by 65535.0 divided by 100.0, resulting in the corresponding
 *          raw value.
*/
uint16_t SHT3x::HumidityToRawValue(float humidity)
{
    return ((humidity / 100.0) * 65535.0);
}

/**
 * @details This function initiates a periodic measurement based on the current repeatability setting of the sensor.
 *          After calling this function, the sensor will start the periodic measurement mode at 0.5 mps.
*/
bool SHT3x::MeasurePeriodicHalfMPS()
{
    switch(_repeatability)
    {
        case e_low:
            if(!WriteCommand(MEAS_LOW_REP_05_MPS))
            {
                return false;// Error
            }
            break;
        case e_medium:
            if(!WriteCommand(MEAS_MID_REP_05_MPS))
            {
                return false;// Error
            }
            break;
        case e_high:
            if(!WriteCommand(MEAS_HI_REP_05_MPS))
            {
                return false;// Error
            }
            break;

        default:
            return false;// Error
            break;
    }
    return true;
}

/**
 * @details This function initiates a periodic measurement based on the current repeatability setting of the sensor.
 *          After calling this function, the sensor will start the periodic measurement mode at 1 mps.
*/
bool SHT3x::MeasurePeriodic1MPS()
{
    switch(_repeatability)
    {
        case e_low:
            if(!WriteCommand(MEAS_LOW_REP_1_MPS))
            {
                return false;// Error
            }
            break;
        case e_medium:
            if(!WriteCommand(MEAS_MID_REP_1_MPS))
            {
                return false;// Error
            }
            break;
        case e_high:
            if(!WriteCommand(MEAS_HI_REP_1_MPS))
            {
                return false;// Error
            }
            break;

        default:
            return false;// Error
            break;
    }
    return true;
}

/**
 * @details This function initiates a periodic measurement based on the current repeatability setting of the sensor.
 *          After calling this function, the sensor will start the periodic measurement mode at 2 mps.
*/
bool SHT3x::MeasurePeriodic2MPS()
{
    switch(_repeatability)
    {
        case e_low:
            if(!WriteCommand(MEAS_LOW_REP_2_MPS))
            {
                return false;// Error
            }
            break;
        case e_medium:
            if(!WriteCommand(MEAS_MID_REP_2_MPS))
            {
                return false;// Error
            }
            break;
        case e_high:
            if(!WriteCommand(MEAS_HI_REP_2_MPS))
            {
                return false;// Error
            }
            break;

        default:
            return false;// Error
            break;
    }
    return true;
}

/**
 * @details This function initiates a periodic measurement based on the current repeatability setting of the sensor.
 *          After calling this function, the sensor will start the periodic measurement mode at 4 mps.
*/
bool SHT3x::MeasurePeriodic4MPS()
{
    switch(_repeatability)
    {
        case e_low:
            if(!WriteCommand(MEAS_LOW_REP_4_MPS))
            {
                return false;// Error
            }
            break;
        case e_medium:
            if(!WriteCommand(MEAS_MID_REP_4_MPS))
            {
                return false;// Error
            }
            break;
        case e_high:
            if(!WriteCommand(MEAS_HI_REP_4_MPS))
            {
                return false;// Error
            }
            break;

        default:
            return false;// Error
            break;
    }
    return true;
}

/**
 * @details This function initiates a periodic measurement based on the current repeatability setting of the sensor.
 *          After calling this function, the sensor will start the periodic measurement mode at 10 mps.
*/
bool SHT3x::MeasurePeriodic10MPS()
{
    switch(_repeatability)
    {
        case e_low:
            if(!WriteCommand(MEAS_LOW_REP_10_MPS))
            {
                return false;// Error
            }
            break;
        case e_medium:
            if(!WriteCommand(MEAS_MID_REP_10_MPS))
            {
                return false;// Error
            }
            break;
        case e_high:
            if(!WriteCommand(MEAS_HI_REP_10_MPS))
            {
                return false;// Error
            }
            break;

        default:
            return false;// Error
            break;
    }
    return true;
}

/**
 * @details This function sends a command to the SHT3x sensor to fetch the latest temperature and humidity
 *          measurement results in periodic measurement mode. The function reads the measurement data from
 *          the sensor and updates the internal temperature and humidity values of the SHT3x object.
*/
bool SHT3x::FetchData()
{
    if(!WriteCommand(FETCH_DATA_COMMAND))
    {
        return false;// Error
    }

    uint8_t tempmsb, templsb, tempchecksum, humiditymsb, humiditylsb, humiditychecksum;

    if(!Read6Bytes(&tempmsb, &templsb, &tempchecksum, &humiditymsb, &humiditylsb, &humiditychecksum))
    {
        return false;// Error
    }
    _celsius = RawValueToCelsius(CombineBytes(tempmsb, templsb));
    _fahrenheit = RawValueToFahrenheit(CombineBytes(tempmsb, templsb));
    _humidity = RawValueToHumidity(CombineBytes(humiditymsb, humiditylsb));

    return true;
}