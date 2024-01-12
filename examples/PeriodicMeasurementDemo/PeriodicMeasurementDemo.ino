/**
 * @file      PeriodicMeasurementDemo.ino
 * @author    Mohammed Hani Ahmed, Remal IoT
 * @date      August 25, 2023
 * @brief     Demonstrates the use of the Remal_SHT3X library for performing periodic temperature
 *            and humidity measurements using the SHT3x-DIS sensor.
 * @details   This sketch showcases how to use the Remal_SHT3X library to interact with the SHT3x-DIS
 *            temperature and humidity sensor. It initializes the sensor, sets the periodic measurement
 *            frequency, starts periodic data acquisition, and then periodically reads and prints the
 *            temperature and relative humidity values.
*/

#include "Remal_SHT3X.h"

/* If using AVR (Nabd or Atlas boards), you cannot change the default I2C pins as they are hardwired */
#ifdef __AVR__
SHT3x sensor(SHT3X_ADDRESS);
#else
#define SDA_PIN 2         // Define the SDA pin number for I2C communication
#define SCL_PIN 8         // Define the SCL pin number for I2C communication
// I2C address (0x44 => ADDR pin is LOW, 0x45 => ADDR pin is HIGH)
// You can use the following defines from the library for the address
// SHT3X_ADDRESS: 0x44, Default I2C address when the ADDR pin is set to LOW
// SHT3X_ADDRESS_B: 0x45, I2C address when the ADDR pin is set to HIGH
SHT3x sensor(SHT3X_ADDRESS, SDA_PIN, SCL_PIN);      // Create an instance of the SHT3x class with the specified I2C address and communication pins
#endif

void setup() 
{
  // Initialize the serial monitor at 9600 baud rate
  Serial.begin(9600);

  // Initialize the SHT3x sensor
  sensor.Initialize();

  // Set the periodic frequency to 1 measurement per second (e_1mps)
  //Avaialable options are e_halfmps, e_1mps, e_2mps, e_4mps, and e_10mps
  sensor.SetPeriodicFrequency(PeriodicFrequency::e_1mps);

  // Start the periodic data acquisition mode
  sensor.StartPeriodic();
}

void loop() 
{
  // Printing the temperature value acquired from periodic measurements
  Serial.print("Temperature: ");
  delay(1000);// A delay is needed depending on the selected periodic measurement frequency to ensure that the data is ready before fetching it from the sensor
  Serial.print(sensor.GetTemperaturePeriodic());// Fetch the periodic temperature data from the sensor
  Serial.println(" degrees Celsius");

  // Printing the relative humidity value acquired from periodic measurements
  Serial.print("Relative Humidity: ");
  delay(1000);// A delay is needed depending on the selected periodic measurement frequency to ensure that the data is ready before fetching it from the sensor
  Serial.print(sensor.GetHumidityPeriodic());// Fetch the periodic relative humidity data from the sensor
  Serial.println(" %");
}