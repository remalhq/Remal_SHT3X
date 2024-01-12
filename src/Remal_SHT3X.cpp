/*
 * Remal_SHT3X.cpp
 *
 *  # ALL INFO CAN BE FOUND IN THE HEADER FILE #
 */
#include "Remal_SHT3X.h"

/**
 * @details The constructor initializes the SHT3x sensor with the given I2C address and default communication pins.
 *          It sets the default clock frequency to 100 kHz and the default repeatability mode to high.
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
}

#ifndef __AVR__        //AVR devices must use default SDA/SCL pins as their Wire library does not support anything else, below functions removed if AVR
/**
 * @details The constructor initializes the SHT3x sensor with the given I2C address and communication pins.
 *          It sets the default clock frequency to 100 kHz and the default repeatability mode to high.
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
}

/**
 * @details The constructor initializes the SHT3x sensor with the given I2C address, communication pins, and
 *          communication frequency. It also sets the default repeatability mode to high.
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
}
#endif


/**
 * @details This function initializes the SHT3x sensor by starting the I2C communication with the specified
 *          SDA and SCL pins at the given frequency. It also performs a soft reset on the sensor to ensure
 *          that it is in a known state before further operations. The function returns true if the
 *          initialization is successful, and false otherwise.
*/
bool SHT3x::Initialize()
{  
#ifdef __AVR__      //AVR devices must use default SDA/SCL pins as their Wire library does not support anything else
    Wire.begin();
#else
    Wire.begin(_sda, _scl, _frequency);
#endif

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
 *          of approximately 2 milliseconds, which is sufficient for the soft reset to complete.
*/
bool SHT3x::SoftReset()
{
    if(!WriteCommand(SOFT_RESET_COMMAND))
    {
        return false;// Error
    }
    delay(2);// Soft reset time: typ = 0.5 ms, max = 1.5 ms
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
    return ((msb << 8) | lsb);
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
    _celsius = RawValueToCelsius((tempmsb << 8) | templsb);
    _fahrenheit = RawValueToFahrenheit((tempmsb << 8) | templsb);
    _humidity = RawValueToHumidity((humiditymsb << 8) | humiditylsb);

    return true;
}

// Periodic measurement functions are not ready in v1, will be overhauled in next version
/**
 * @details This function initiates a periodic measurement based on the current repeatability setting of the sensor.
 *          After calling this function, the sensor will start the periodic measurement mode at 0.5 mps.
*/
bool SHT3x::MeasurePeriodic()
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

// Periodic measurement functions are not ready in v1, will be overhauled in next version
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

// Periodic measurement functions are not ready in v1, will be overhauled in next version
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

// Periodic measurement functions are not ready in v1, will be overhauled in next version
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

// Periodic measurement functions are not ready in v1, will be overhauled in next version
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

// Periodic measurement functions are not ready in v1, will be overhauled in next version
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
    _celsius = RawValueToCelsius((tempmsb << 8) | templsb);
    _fahrenheit = RawValueToFahrenheit((tempmsb << 8) | templsb);
    _humidity = RawValueToHumidity((humiditymsb << 8) | humiditylsb);

    return true;
}

// Periodic measurement functions are not ready in v1, will be overhauled in next version
/**
 * @details This function initiates a periodic measurement with accelerated response time (ART).
 *          After calling this function, the sensor will start acquiring data with a frequency of 4 Hz.
*/
bool SHT3x::MeasurePeriodicART()
{
    if(!WriteCommand(ART_COMMAND))
    {
        return false;// Error
    }
    return true;
}

// Periodic measurement functions are not ready in v1, will be overhauled in next version
/**
 * @details This function sends the break command to the SHT3x sensor, which stops the periodic data acquisition mode.
 *          When the sensor is in periodic data acquisition mode, it continuously performs measurements at the specified
 *          measurement rate. Calling this function will halt the periodic measurements until another measurement command
 *          is issued. It is recommended to stop the periodic data acquisition prior to sending another command.
*/
bool SHT3x::Break()
{
    if(!WriteCommand(BREAK_COMMAND))
    {
        return false;// Error
    }
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
    for (int i = 0; i < 2; i++)
    {
        uint8_t currentByte = (data >> (8 * i)) & 0xFF;
        crc ^= currentByte;

        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x31;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc ^ 0x00;
}

/**
 * @details This function verifies the CRC-8 checksum for two bytes of data by calculating the CRC-8 checksum for
 *          the combined data and comparing it with the received checksum. It takes the most significant byte (MSB),
 *          least significant byte (LSB), and the received checksum as inputs. The function returns true if the
 *          calculated checksum matches the received checksum, indicating data integrity, or false otherwise.
*/
bool SHT3x::VerifyCRC8(uint8_t msb, uint8_t lsb, uint8_t checksum)
{
    // Combine the two bytes of data to form a 16-bit value
    uint16_t combinedData = (uint16_t(msb) << 8) | lsb;

    // Calculate the CRC-8 checksum for the combined data
    uint8_t calculatedChecksum = CalculateCRC8(combinedData);

    // Compare the calculated checksum with the received checksum
    return (calculatedChecksum == checksum);
}