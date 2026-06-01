/**
 *******************************************************************************
   @copyright   Copyright (C) by SenseAir AB. All rights reserved.
   @file        sunrise_i2c_change_address.ino
   @brief       Example showing how to change the sensor's I2C communication address.
   @details     See "I2C on Senseair Sunrise" documentation (available on www.senseair.com).
   @details     Tested on Arduino Mega 2560, Arduino Nano 33 BLE

   @author      Senseair FW team

 *******************************************************************************
*/
#include "senseair_sunrise.h"

/* Sensor addresses. Change if desired */
constexpr uint8_t CURRENT_SUNRISE_ADDR = SUNRISE_DEFAULT_COMM_ADDRESS;
constexpr uint8_t NEW_SUNRISE_ADDR     = 0x70u;

/* Sensor measurement configuration. Change if desired */
constexpr MeasurementMode MEASUREMENT_MODE = MeasurementMode::CONTINUOUS;

/* Pins for enable. Change if desired */
constexpr int SUNRISE_EN_PIN = 5;
/* Pin to select communication protocol (COMSEL): LOW = I2C, HIGH = Modbus */
constexpr int SUNRISE_COMSEL_PIN = 4;

/* Create Sunrise object */
SunriseI2C sunrise = SunriseI2C(Wire, CURRENT_SUNRISE_ADDR, SUNRISE_EN_PIN);

/* Global parameters */
int           ledValue             = HIGH;
unsigned long measurementTimeStamp = 0ul;
uint16_t      measPeriod           = 16u;

void setup()
{
    SunriseError error;
    String       errorMessage;

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, ledValue);

    /* Set COMSEL pin to select I2C */
    pinMode(SUNRISE_COMSEL_PIN, OUTPUT);
    digitalWrite(SUNRISE_COMSEL_PIN, LOW);
#if defined(PIN_SERIAL_TX)
    pinMode(PIN_SERIAL_TX, INPUT);
#endif

    /* Initialize I2C and use default pins defined for the board */
    Wire.begin();
    Wire.setClock(SUNRISE_I2C_CLOCK_HZ);
    Serial.begin(115200);

    sunrise.begin();
    sunrise.enable();

    SunriseConfiguration sunriseConfigurationTemp;
    error = sunrise.getSensorConfig(sunriseConfigurationTemp);
    if (error == SunriseError::NO_ERROR)
    {
        measPeriod = sunriseConfigurationTemp.measPeriod;

        if (MEASUREMENT_MODE != sunriseConfigurationTemp.measMode)
        {
            sunriseConfigurationTemp.measMode = MEASUREMENT_MODE;

            error = sunrise.setSensorConfig(sunriseConfigurationTemp);
            if (error == SunriseError::NO_ERROR)
            {
                Serial.println("New configuration set.");
            }
            else
            {
                errorMessage = SunriseBase::evaluateError(error, "set sensor configuration");
                Serial.println(errorMessage);
            }
        }
        else
        {
            Serial.println("Configuration already matched.");
        }
    }
    else
    {
        errorMessage = SunriseBase::evaluateError(error, "read sensor configuration");
        Serial.println(errorMessage);
    }

    delay(500);

    /* Change communication address on the sensor. New address valid after reset */
    error = sunrise.setSensorAddress(NEW_SUNRISE_ADDR);
    if (error == SunriseError::NO_ERROR)
    {
        Serial.println("New address set.");
    }
    else
    {
        errorMessage = SunriseBase::evaluateError(error, "Set new address");
        Serial.println(errorMessage);
    }
    /* SCR reset to read new values from EEPROM (resetBySCR waits for EEPROM settle internally) */
    error = sunrise.resetBySCR();
    if (error == SunriseError::NO_ERROR)
    {
        Serial.println("Restarting sensor to read new values from EEPROM.");
    }
    else
    {
        errorMessage = SunriseBase::evaluateError(error, "reset through SCR");
        Serial.println(errorMessage);
    }
    delay(2 * SUNRISE_STABILIZATION_MS);
    /* Change the address member in sunrise object to allow communication after reset */
    sunrise.setCommAddress(NEW_SUNRISE_ADDR);

    Serial.println("\nWaiting for next measurement...\n");
}

void loop()
{
    /* Perform measurement cycle every measurement period */
    if (millis() - measurementTimeStamp > measPeriod * 1000UL)
    {
        measurementTimeStamp = millis();

        SunriseError error;
        String       errorMessage;

        /* Performs a measurement and print results */
        SunriseMeasurement sunriseMeasurement = {0, 0, 0u};
        error                                 = sunrise.readMeasurement(sunriseMeasurement);
        if (error == SunriseError::NO_ERROR)
        {
            Serial.print("CO2: ");
            Serial.print(sunriseMeasurement.co2value);
            Serial.println(" ppm");
            Serial.print("Temp: ");
            Serial.print(sunriseMeasurement.temperature / (100.0f));
            Serial.println(" degree C");
            Serial.print("Error Status: 0x");
            Serial.println(sunriseMeasurement.errorStatus, HEX);
        }
        else
        {
            errorMessage = SunriseBase::evaluateError(error, "read sensor measurement");
            Serial.println(errorMessage);
        }

        /* Toggle working state */
        digitalWrite(LED_BUILTIN, ledValue);
        ledValue = ((ledValue == HIGH) ? LOW : HIGH);

        Serial.println("\nWaiting for next measurement...\n");
    }
}
