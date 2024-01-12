/**
 * @file      SensorDemo.ino
 * @author    Mohammed Hani Ahmed, Remal IoT
 * @date      August 2, 2023
 * @brief     Example usage of the SHT3x temperature and humidity sensor library.
 * @details   This sketch demonstrates how to use the SHT3x library to interface with
 *            the SHT3x-DIS temperature and humidity sensor using I2C communication.
 *            The sketch initializes the sensor, reads its status, and periodically
 *            prints the temperature in Celsius and Fahrenheit, as well as the relative
 *            humidity, to the Serial Monitor.
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

  // Set the repeatability mode to high (e_high)
  // The repeatability mode determines the measurement accuracy and energy consumption.
  // Available options are e_low, e_medium, and e_high.
  sensor.SetRepeatability(e_high);
}

void loop() 
{
  // Check if the SHT3x sensor is connected and responsive
  if(sensor.IsConnected())
  {
    // Print a message indicating that the sensor is connected
    Serial.println("SHT3x Sensor is connnected.");

    // Read and print the value of the status register in hexadecimal format
    Serial.print("Status Register: ");
    Serial.println(sensor.ReadStatusRegister(), HEX);

    // Read and print the temperature value in Celsius
    Serial.print("Temperature: ");
    Serial.print(sensor.GetTemperatureCelsius());
    Serial.println(" degrees Celsius");

    // Read and print the temperature value in Fahrenheit
    Serial.print("Temperature: ");
    Serial.print(sensor.GetTemperatureFahrenheit());
    Serial.println(" degrees Fahrenheit");

    // Read and print the relative humidity value in percentage
    Serial.print("Relative Humidity: ");
    Serial.print(sensor.GetHumidity());
    Serial.println(" %");

    Serial.print("\n");
  }

  // Wait for one second before the next measurement
  delay(1000);
}