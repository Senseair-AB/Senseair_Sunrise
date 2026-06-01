/**
 *******************************************************************************
   @copyright   Copyright (C) by SenseAir AB. All rights reserved.
   @file        sunrise_scale_factor.ino
   @brief       Demonstrates reading/writing concentration scale factors
                and reading scaled measurements (firmware 4.10+).
   @details     Sunrise/Sunlight sensors only (S12 does not support scaling).
                Select protocol via SUNRISE_PROTOCOL define.
   @author      Senseair FW team
 *******************************************************************************
*/
#include "senseair_sunrise.h"

#if !defined(SUNRISE_PROTOCOL_I2C) && !defined(SUNRISE_PROTOCOL_MODBUS)
/* Protocol selection: Uncomment one of the following defines */
// #define SUNRISE_PROTOCOL_I2C
#define SUNRISE_PROTOCOL_MODBUS
#endif /*#if !defined(SUNRISE_PROTOCOL_I2C) && !defined(SUNRISE_PROTOCOL_MODBUS)*/

/* Define serial port based on protocol */
#ifdef SUNRISE_PROTOCOL_I2C
#define SUNRISE_SERIAL Wire
#else
#define SUNRISE_SERIAL Serial1
#endif

/* Set to true to write a new scale factor to the sensor.
 * Example values for Sunlight R32 %LFL conversion: numerator=4171, denominator=4096.
 * Set both to 0xFFFF to disable scaling.
 */
#ifndef CHANGE_SCALE_FACTOR
#define CHANGE_SCALE_FACTOR 0
#endif
constexpr uint16_t SCALE_FACTOR_NUM = 0xFFFFu;
constexpr uint16_t SCALE_FACTOR_DEN = 0xFFFFu;

/* Hardware configuration */
constexpr uint8_t SUNRISE_ADDR   = SUNRISE_DEFAULT_COMM_ADDRESS;
constexpr int     SUNRISE_EN_PIN = 5;
/* Pin to select communication protocol (COMSEL): LOW = I2C, HIGH = Modbus */
constexpr int SUNRISE_COMSEL_PIN = 4;
constexpr int SUNRISE_NRDY       = 7;

/* Measurement period for reading loop */
constexpr unsigned long MEASUREMENT_PERIOD_MS = 16000UL;

/* Create Sunrise object (NOT S12 — scaling is Sunrise/Sunlight only) */
#ifdef SUNRISE_PROTOCOL_I2C
SunriseI2C sunrise = SunriseI2C(SUNRISE_SERIAL, SUNRISE_ADDR, SUNRISE_EN_PIN, SUNRISE_NRDY);
#else
SunriseModbus sunrise = SunriseModbus(&SUNRISE_SERIAL, SUNRISE_ADDR, SUNRISE_EN_PIN, SUNRISE_NRDY);
#endif

static int           ledValue             = HIGH;
static unsigned long measurementTimeStamp = 0ul;

bool elapsed(uint32_t now, uint32_t since, uint32_t period)
{
    return (now - since) >= period;
}

void printErrorStatus(uint16_t errorStatus)
{
    if (errorStatus == 0)
        return;
    Serial.println("  Active error flags:");
    if (errorStatus & static_cast<uint16_t>(ErrorStatus::FATAL_ERROR))
        Serial.println("  - Fatal error");
    if (errorStatus & static_cast<uint16_t>(ErrorStatus::I2C_ERROR))
        Serial.println("  - I2C error");
    if (errorStatus & static_cast<uint16_t>(ErrorStatus::ALGORITHM_ERROR))
        Serial.println("  - Algorithm error");
    if (errorStatus & static_cast<uint16_t>(ErrorStatus::CALIBRATION_ERROR))
        Serial.println("  - Calibration error");
    if (errorStatus & static_cast<uint16_t>(ErrorStatus::SELF_DIAG_ERROR))
        Serial.println("  - Self-diagnostics error");
    if (errorStatus & static_cast<uint16_t>(ErrorStatus::OUT_OF_RANGE))
        Serial.println("  - Out of range");
    if (errorStatus & static_cast<uint16_t>(ErrorStatus::MEMORY_ERROR))
        Serial.println("  - Memory error");
    if (errorStatus & static_cast<uint16_t>(ErrorStatus::NO_MEASUREMENT))
        Serial.println("  - No measurement completed");
    if (errorStatus & static_cast<uint16_t>(ErrorStatus::LOW_VOLTAGE))
        Serial.println("  - Low internal voltage");
    if (errorStatus & static_cast<uint16_t>(ErrorStatus::MEAS_TIMEOUT))
        Serial.println("  - Measurement timeout");
    if (errorStatus & static_cast<uint16_t>(ErrorStatus::ABNORMAL_SIGNAL))
        Serial.println("  - Abnormal signal level");
}

void setup()
{
    SunriseError error;

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, ledValue);

    Serial.begin(115200);
    Serial.print("Waiting for serial connection...");
    while (!elapsed(millis(), 0, 5000))
    {
        Serial.print(".");
        delay(10);
    }
    Serial.println();

    /* Set COMSEL pin to select active communication protocol */
    pinMode(SUNRISE_COMSEL_PIN, OUTPUT);
#ifdef SUNRISE_PROTOCOL_I2C
    digitalWrite(SUNRISE_COMSEL_PIN, LOW);
#if defined(PIN_SERIAL_TX)
    pinMode(PIN_SERIAL_TX, INPUT);
#endif
    SUNRISE_SERIAL.begin();
    SUNRISE_SERIAL.setClock(SUNRISE_I2C_CLOCK_HZ);
    Serial.println("Interface: I2C");
#else
    digitalWrite(SUNRISE_COMSEL_PIN, HIGH);
    SUNRISE_SERIAL.begin(SUNRISE_MODBUS_BAUDRATE);
    Serial.println("Interface: Modbus");
#endif

    sunrise.begin();
    /* Reset sensor to apply communication interface settings */
    sunrise.disable();
    delay(SUNRISE_STABILIZATION_MS);
    sunrise.enable();

    {
        uint8_t fwType = 0;
        error          = sunrise.getFirmwareType(fwType);
        if (error == SunriseError::NO_ERROR)
        {
            Serial.print("Firmware type: 0x");
            Serial.println(fwType, HEX);
            if (fwType == S12_FW_TYPE)
            {
                Serial.println(
                    "Error: S12 sensor detected. This example is designed for Sunrise/Sunlight sensors only.");
                while (true)
                    ;
            }
        }
        else
        {
            Serial.println(SunriseBase::evaluateError(error, "read firmware type"));
            Serial.println("Error: Sunrise sensor not detected. Check connection!");
            while (true)
                ;
        }
    }

    /* Read and display current scale factor */
    SunriseScaleFactor currentFactor;
    error = sunrise.getScaleFactor(currentFactor);
    if (error == SunriseError::NO_ERROR)
    {
        Serial.println("-------------------");
        Serial.print("Scale factor numerator: ");
        Serial.println(currentFactor.numerator);
        Serial.print("Scale factor denominator: ");
        Serial.println(currentFactor.denominator);
        if (currentFactor.numerator == 0xFFFF && currentFactor.denominator == 0xFFFF)
        {
            Serial.println("Scaling is disabled.");
        }
    }
    else
    {
        Serial.println(SunriseBase::evaluateError(error, "read scale factor"));
        Serial.println("Note: Scale factors require firmware 4.10+");
    }

    /* Optionally write a new scale factor */
    if (CHANGE_SCALE_FACTOR)
    {
        SunriseScaleFactor newFactor = {SCALE_FACTOR_NUM, SCALE_FACTOR_DEN};
        error                        = sunrise.setScaleFactor(newFactor);
        if (error == SunriseError::NO_ERROR)
        {
            Serial.print("New scale factor set: ");
            Serial.print(newFactor.numerator);
            Serial.print("/");
            Serial.println(newFactor.denominator);

            /* Reset sensor to activate new scaling */
            error = sunrise.resetBySCR();
            if (error == SunriseError::NO_ERROR)
            {
                Serial.println("Sensor reset to activate new scale factor.");
                delay(2 * SUNRISE_STABILIZATION_MS);
            }
            else
            {
                Serial.println(SunriseBase::evaluateError(error, "reset through SCR"));
            }
        }
        else
        {
            Serial.println(SunriseBase::evaluateError(error, "set scale factor"));
        }
    }

    Serial.println("\nWaiting for next measurement...\n");
}

void loop()
{
    unsigned long currentTimeStamp = millis();

    if (elapsed(currentTimeStamp, measurementTimeStamp, MEASUREMENT_PERIOD_MS))
    {
        measurementTimeStamp = currentTimeStamp;

        /* Read standard measurement */
        SunriseMeasurement meas  = {0, 0, 0u};
        SunriseError       error = sunrise.readMeasurement(meas);
        if (error == SunriseError::NO_ERROR)
        {
            Serial.print("CO2: ");
            Serial.print(meas.co2value);
            Serial.println(" ppm");

            /* Read scaled measurement */
            int16_t scaledValue = 0;
            error               = sunrise.getScaledMeasurement(scaledValue);
            if (error == SunriseError::NO_ERROR)
            {
                Serial.print("Scaled: ");
                Serial.print(scaledValue);
                Serial.println(" ppm");
            }

            Serial.print("Temp: ");
            Serial.print(meas.temperature / 100.0f);
            Serial.println(" degree C");
            Serial.print("Error Status: 0x");
            Serial.println(meas.errorStatus, HEX);
            printErrorStatus(meas.errorStatus);
        }
        else
        {
            Serial.println(SunriseBase::evaluateError(error, "read measurement"));
        }

        digitalWrite(LED_BUILTIN, ledValue);
        ledValue = ((ledValue == HIGH) ? LOW : HIGH);

        Serial.println("\nWaiting for next measurement...\n");
    }
}
