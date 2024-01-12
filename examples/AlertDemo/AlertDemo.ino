/**
 * @file      AlertDemo.ino
 * @author    Mohammed Hani Ahmed, Remal IoT
 * @date      August 25, 2023
 * @brief     Demonstrates the use of the Remal_SHT3X library for setting alert limits
 *            and handling alerts using the SHT3x-DIS sensor.
 * @details   This sketch demonstrates how to use the Remal_SHT3X library to interact with
 *            the SHT3x-DIS temperature and humidity sensor to set alert limits and handle
 *            alerts. It initializes the sensor, sets high and low alert limits for temperature
 *            and humidity, and monitors the alert pin. When an alert occurs, it reads and prints
 *            the temperature and humidity values, and enters an alert loop until the alert clears.
*/

#include "Remal_SHT3X.h"

// Connect the SHT3x sensor's alert pin to one of the board's GPIO pins (using pin 0 in this example)
#define DIGITAL_IO_PIN 0
// Alternatively, you can connect the alert pin to an LED to indicate when an alert happens

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

  // Set the high alert humidity and temperature limits
  // Humidity high set = 60 %
  // Humidity high clear = 60 %
  // Temperature high set = 30 degrees Celsius
  // Temperature high clear 30 degrees Celsius
  sensor.SetHighAlertLimit(60, 30);
  // This is equivalent to calling SetHighAlertLimit(60, 60, 30, 30)

  // Set the low alert humidity and temperature limits
  // Humidity low clear = 22 %
  // Humidity low set = 20 %
  // Temperature low clear = -10 degrees Celsius
  // Temperature  low set = -12 degrees Celsius
  sensor.SetLowAlertLimit(22, 20, -10, -12);

  // Set the periodic frequency to 2 measurements per second (e_2mps)
  //Avaialable options are e_halfmps, e_1mps, e_2mps, e_4mps, and e_10mps
  sensor.SetPeriodicFrequency(PeriodicFrequency::e_2mps);

  // Start the periodic data acquisition mode
  sensor.StartPeriodic();

  // Setting the digital IO pin as an input
  pinMode(DIGITAL_IO_PIN, INPUT);
}

void loop() 
{
  //Printing the temperature alert limits
  // Printing the temperature high set limit
  Serial.print("High set temperature limit: ");
  Serial.print(sensor.GetTemperatureHighSetLimit());
  Serial.println(" degrees Celsius");
  delay(1000);
  // Printing the temperature high clear limit
  Serial.print("High clear temperature limit: ");
  Serial.print(sensor.GetTemperatureHighClearLimit());
  Serial.println(" degrees Celsius");
  delay(1000);
  // Printing the temperature low clear limit
  Serial.print("Low clear temperature limit: ");
  Serial.print(sensor.GetTemperatureLowClearLimit());
  Serial.println(" degrees Celsius");
  delay(1000);
  // Printing the temperature low set limit
  Serial.print("Low set temperature limit: ");
  Serial.print(sensor.GetTemperatureLowSetLimit());
  Serial.println(" degrees Celsius");
  delay(1000);

  //Printing the humidity alert limits
  // Printing the humidity high set limit
  Serial.print("High set humidity limit: ");
  Serial.print(sensor.GetHumidityHighSetLimit());
  Serial.println(" %");
  delay(1000);
  // Printing the humidity high clear limit
  Serial.print("High clear humidity limit: ");
  Serial.print(sensor.GetHumidityHighClearLimit());
  Serial.println(" %");
  delay(1000);
  // Printing the humidity low clear limit
  Serial.print("Low clear humidity limit: ");
  Serial.print(sensor.GetHumidityLowClearLimit());
  Serial.println(" %");
  delay(1000);
  // Printing the humidity low set limit
  Serial.print("Low set humidity limit: ");
  Serial.print(sensor.GetHumidityLowSetLimit());
  Serial.println(" %");
  delay(1000);

  // When an alert occurs, the alert pin will become HIGH
  // If alert pin is connected to the digital IO pin, will enter this while loop until the alert ends
  // If alert pin is connected to an LED, the LED will turn on until the alert ends
  while(digitalRead(DIGITAL_IO_PIN))
  {
    Serial.println("ALERT!!!");

    // Printing the temperature value acquired from periodic measurements
    Serial.print("Temperature: ");
    delay(500);// A delay is needed depending on the selected periodic measurement frequency to ensure that the data is ready before fetching it from the sensor
    Serial.print(sensor.GetTemperaturePeriodic());// Fetch the periodic temperature data from the sensor
    Serial.println(" degrees Celsius");

    // Printing the relative humidity value acquired from periodic measurements
    Serial.print("Relative Humidity: ");
    delay(500);// A delay is needed depending on the selected periodic measurement frequency to ensure that the data is ready before fetching it from the sensor
    Serial.print(sensor.GetHumidityPeriodic());// Fetch the periodic relative humidity data from the sensor
    Serial.println(" %");
  }
}