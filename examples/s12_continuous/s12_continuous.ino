/**
 *******************************************************************************
   @copyright   Copyright (C) by SenseAir AB. All rights reserved.
   @file        s12_continuous.ino
   @brief       S12 CO2 sensor continuous mode example for both I2C and Modbus.
   @details     Tested on Arduino Mega 2560, Arduino Nano 33 BLE.
                Select protocol via SUNRISE_PROTOCOL define.
                Select output format via SERIAL_PLOTTER_MODE define.
   @author      Senseair FW team
 *******************************************************************************
*/

#include "senseair_sunrise.h"

#if !defined(SUNRISE_PROTOCOL_I2C) && !defined(SUNRISE_PROTOCOL_MODBUS)
/* Protocol selection: Uncomment one of the following defines */
// #define SUNRISE_PROTOCOL_I2C
#define SUNRISE_PROTOCOL_MODBUS
#endif /*#if !defined(SUNRISE_PROTOCOL_I2C) && !defined(SUNRISE_PROTOCOL_MODBUS)*/

/* Output mode: Uncomment to enable Serial Plotter optimized output */
// #define SERIAL_PLOTTER_MODE

/* Define serial port based on protocol */
#ifdef SUNRISE_PROTOCOL_I2C
#define SUNRISE_SERIAL Wire
#else
#define SUNRISE_SERIAL Serial1
#endif

/* Sensor measurement configuration. Change if desired.
 * S12 measurement period range: 1-2047 seconds.
 * S12 number of samples valid values: 0-20 (any), 29-79 (step 10), 99-999 (step 50).
 */
#ifndef CHANGE_MEASUREMENT_CONFIGURATION
#define CHANGE_MEASUREMENT_CONFIGURATION 0
#endif
constexpr MeasurementMode MEASUREMENT_MODE     = MeasurementMode::CONTINUOUS;
constexpr uint16_t        MEASUREMENT_PERIOD_S = 2u;
constexpr uint16_t        MEASUREMENT_SAMPLES  = 59u;
/* Sensor extended configuration settings. Change if desired */
#ifndef CHANGE_EXTENDED_CONFIGURATION
#define CHANGE_EXTENDED_CONFIGURATION 0
#endif
constexpr bool ABC_ENABLED           = true;
constexpr bool IIR_FILTER_ENABLED    = true;
constexpr bool PRESSURE_COMP_ENABLED = false;
constexpr bool NRDY_PIN_ENABLED      = true;
constexpr bool NRDY_PIN_INVERTED     = false;

/* Sensor general configuration. Change if desired */
constexpr int16_t  PRESSURE_VALUE   = 0;    /* Unit in 0.1 hPa, 0 = disabled */
constexpr uint16_t ABC_PERIOD_HOURS = 180u; /* Unit in hours */
constexpr uint8_t  SUNRISE_ADDR     = SUNRISE_DEFAULT_COMM_ADDRESS;

/* Hardware configuration */
constexpr int SUNRISE_EN_PIN = 5;
/* Pin to select communication protocol (COMSEL): LOW = I2C, HIGH = Modbus */
constexpr int      SUNRISE_COMSEL_PIN   = 4;
constexpr int      SUNRISE_NRDY         = 7;
constexpr int      CALIBRATE_BUTTON_PIN = 6;
constexpr uint32_t BUTTON_DEBOUNCE_MS   = 200;

/* How often to poll the measurement counter. Shorter = more responsive,
 * but generates more bus traffic. 100 ms is a good balance for a 2+ s period. */
constexpr uint32_t MEAS_COUNT_POLL_MS = 100;

#ifdef SUNRISE_PROTOCOL_I2C
S12I2C sensor = S12I2C(SUNRISE_SERIAL, SUNRISE_ADDR, SUNRISE_EN_PIN, SUNRISE_NRDY);
#else
S12Modbus sensor = S12Modbus(&SUNRISE_SERIAL, SUNRISE_ADDR, SUNRISE_EN_PIN, SUNRISE_NRDY);
#endif

static unsigned long sensorMeasurementPeriodMs = MEASUREMENT_PERIOD_S * 1000UL;
static int           ledValue                  = HIGH;

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
    bool         isResetNeeded = false;

    pinMode(CALIBRATE_BUTTON_PIN, INPUT_PULLUP);
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
    /* S12 supports I2C Fast Mode up to 400 kHz */
    SUNRISE_SERIAL.begin();
    SUNRISE_SERIAL.setClock(S12_I2C_CLOCK_HZ);
    Serial.println("Interface: I2C (S12, 400 kHz)");
#else
    digitalWrite(SUNRISE_COMSEL_PIN, HIGH);
    SUNRISE_SERIAL.begin(SUNRISE_MODBUS_BAUDRATE);
    Serial.println("Interface: Modbus (S12)");
#endif

    sensor.begin();
    /* Reset sensor to apply communication interface settings */
    sensor.disable();
    delay(S12_STABILIZATION_MS);
    sensor.enable();

    {
        uint8_t fwType = 0;
        error          = sensor.getFirmwareType(fwType);
        if (error == SunriseError::NO_ERROR)
        {
            Serial.print("Firmware type: 0x");
            Serial.println(fwType, HEX);
            if (fwType != S12_FW_TYPE)
            {
                Serial.println("Warning: unexpected firmware type, this example is designed for S12 sensors");
            }
        }
        else
        {
            Serial.println(SunriseBase::evaluateError(error, "read firmware type"));
        }
    }

#ifdef SUNRISE_PROTOCOL_MODBUS
    {
        SunriseIdentification sunriseIdentification;
        error = sensor.getSensorID(sunriseIdentification);
        if (error == SunriseError::NO_ERROR)
        {
            Serial.println("-------------------");
            Serial.println("Sensor's Device ID");
            Serial.print("Vendor Name: ");
            Serial.println(sunriseIdentification.vendorName);
            Serial.print("ProductCode: ");
            Serial.println(sunriseIdentification.productCode);
            Serial.print("MajorMinorRevision: ");
            Serial.println(sunriseIdentification.majMinRevision);
        }
        else
        {
            Serial.println(SunriseBase::evaluateError(error, "read sensor ID"));
        }
    }
#endif

    {
        SunriseConfiguration baseConfiguration;
        error = sensor.getSensorConfig(baseConfiguration);
        if (error == SunriseError::NO_ERROR)
        {
            Serial.println("-------------------");
            Serial.println("Sensor's base configuration");
            Serial.print("Measurement mode: ");
            Serial.println((baseConfiguration.measMode == MeasurementMode::CONTINUOUS) ? "CONTINUOUS" : "SINGLE");
            Serial.print("Measurement period: ");
            Serial.println(baseConfiguration.measPeriod);
            Serial.print("Number of samples per measurement: ");
            Serial.println(baseConfiguration.measSamples);

            if (CHANGE_MEASUREMENT_CONFIGURATION)
            {
                if ((baseConfiguration.measMode != MEASUREMENT_MODE) ||
                    (baseConfiguration.measPeriod != MEASUREMENT_PERIOD_S) ||
                    (baseConfiguration.measSamples != MEASUREMENT_SAMPLES))
                {
                    error = sensor.setSensorConfig({MEASUREMENT_MODE, MEASUREMENT_PERIOD_S, MEASUREMENT_SAMPLES});
                    Serial.println((error == SunriseError::NO_ERROR)
                                       ? "New configuration set."
                                       : SunriseBase::evaluateError(error, "set sensor configuration"));
                    isResetNeeded             = true;
                    sensorMeasurementPeriodMs = MEASUREMENT_PERIOD_S * 1000UL;
                }
                else
                {
                    Serial.println("Configuration already matched.");
                }
            }
            else
            {
                sensorMeasurementPeriodMs = baseConfiguration.measPeriod * 1000UL;
            }
        }
        else
        {
            Serial.println(SunriseBase::evaluateError(error, "read sensor configuration"));
        }
    }

    if (CHANGE_EXTENDED_CONFIGURATION)
    {
        SunriseExtendedConfiguration extendedConfiguration;
        error = sensor.getSensorExtendedConfig(extendedConfiguration);
        if (error == SunriseError::NO_ERROR)
        {
            Serial.println("-------------------");
            Serial.println("Sensor's extended configuration");
            Serial.print("Is ABC enabled: ");
            Serial.println(extendedConfiguration.isABCEnabled);
            Serial.print("Is pressure compensation enabled: ");
            Serial.println(extendedConfiguration.isPressureCompensationEnabled);
            Serial.print("Is IIR filter enabled: ");
            Serial.println(extendedConfiguration.isIIRFilterEnabled);
            Serial.print("Is nRDY pin enabled: ");
            Serial.println(extendedConfiguration.isNRDYPinEnabled);

            if (extendedConfiguration.isABCEnabled != ABC_ENABLED ||
                extendedConfiguration.isIIRFilterEnabled != IIR_FILTER_ENABLED ||
                extendedConfiguration.isPressureCompensationEnabled != PRESSURE_COMP_ENABLED ||
                extendedConfiguration.isNRDYPinEnabled != NRDY_PIN_ENABLED ||
                extendedConfiguration.isNRDYPinInverted != NRDY_PIN_INVERTED)
            {
                error = sensor.setSensorExtendedConfig(
                    {ABC_ENABLED, IIR_FILTER_ENABLED, PRESSURE_COMP_ENABLED, NRDY_PIN_ENABLED, NRDY_PIN_INVERTED});
                Serial.println((error == SunriseError::NO_ERROR)
                                   ? "Sensor extended configuration updated"
                                   : SunriseBase::evaluateError(error, "set sensor extended configuration"));
                isResetNeeded = true;
            }
        }
        else
        {
            Serial.println(SunriseBase::evaluateError(error, "read sensor extended configuration"));
        }
    }

    {
        uint16_t abcPeriodTemp = 0;
        error                  = sensor.getABCPeriod(abcPeriodTemp);
        if (error == SunriseError::NO_ERROR)
        {
            if (ABC_PERIOD_HOURS != abcPeriodTemp)
            {
                error = sensor.setABCPeriod(ABC_PERIOD_HOURS);
                Serial.println((error == SunriseError::NO_ERROR) ? "New ABC period set"
                                                                 : SunriseBase::evaluateError(error, "set ABC period"));
                isResetNeeded = true;
            }
            else
            {
                Serial.println("ABC period already matched");
            }
        }
        else
        {
            Serial.println(SunriseBase::evaluateError(error, "read ABC period"));
        }
    }

    if (isResetNeeded)
    {
        Serial.println("Restarting sensor to read new values from EEPROM.");
        sensor.disable();
        delay(S12_STABILIZATION_MS);
        sensor.enable();
        delay(2 * S12_STABILIZATION_MS);
    }

    if (PRESSURE_VALUE != 0)
    {
        error = sensor.setPressureValue(PRESSURE_VALUE);
        Serial.println((error == SunriseError::NO_ERROR) ? "Setting pressure."
                                                         : SunriseBase::evaluateError(error, "set pressure value"));
    }

    Serial.println("\nWaiting for next measurement...\n");
}

void loop()
{
    static enum { CALIB_STATE_IDLE = 0, CALIB_STATE_START, CALIB_STATE_WAIT } calibration_state = CALIB_STATE_IDLE;
    static uint8_t       lastMeasCount                                                          = 0;
    static bool          lastMeasCountValid                                                     = false;
    static unsigned long lastCountPollMs                                                        = 0ul;
    static unsigned long lastNewMeasMs                                                          = 0ul;
    static unsigned long calibrationTimeStamp                                                   = 0ul;
    static unsigned long lastButtonTime                                                         = 0ul;

    unsigned long currentTimeStamp = millis();

    /* Poll the measurement counter every MEAS_COUNT_POLL_MS.
     * A counter change means the sensor has completed a new measurement. */
    if (elapsed(currentTimeStamp, lastCountPollMs, MEAS_COUNT_POLL_MS))
    {
        lastCountPollMs = currentTimeStamp;

        uint8_t      currentMeasCount = 0;
        SunriseError error            = sensor.getMeasurementCount(currentMeasCount);
        if (error != SunriseError::NO_ERROR)
        {
            Serial.println(SunriseBase::evaluateError(error, "read measurement count"));
        }
        else if (!lastMeasCountValid)
        {
            /* First successful read: record the baseline, do not treat as new measurement. */
            lastMeasCount      = currentMeasCount;
            lastMeasCountValid = true;
            lastNewMeasMs      = currentTimeStamp;
        }
        else if (currentMeasCount != lastMeasCount)
        {
            /* Counter changed: a new measurement is ready. */
            lastMeasCount = currentMeasCount;
            lastNewMeasMs = currentTimeStamp;

            if (calibration_state == CALIB_STATE_START)
            {
                error = sensor.sendCalibrationCommand(CalCommand::BACKGROUND);
                if (error == SunriseError::NO_ERROR)
                {
                    Serial.println("Calibration command sent.");
                    calibrationTimeStamp = currentTimeStamp;
                    calibration_state    = CALIB_STATE_WAIT;
                }
                else
                {
                    calibration_state = CALIB_STATE_IDLE;
                    Serial.println(SunriseBase::evaluateError(error, "send calibration command"));
                }
            }

            SunriseMeasurement sunriseMeasurement = {0, 0, 0u};
            error                                 = sensor.readMeasurement(sunriseMeasurement);
            if (error == SunriseError::NO_ERROR)
            {
#ifdef SERIAL_PLOTTER_MODE
                Serial.print("Concentration:");
                Serial.print(sunriseMeasurement.co2value);
                Serial.print("\tTemperature:");
                Serial.print(sunriseMeasurement.temperature / (100.0f));
                Serial.print("\tError:0x");
                Serial.println(sunriseMeasurement.errorStatus, HEX);
#else
                Serial.print("CO2: ");
                Serial.print(sunriseMeasurement.co2value);
                Serial.println(" ppm");
                Serial.print("Temp: ");
                Serial.print(sunriseMeasurement.temperature / (100.0f));
                Serial.println(" degree C");
                Serial.print("Error Status: 0x");
                Serial.println(sunriseMeasurement.errorStatus, HEX);
                printErrorStatus(sunriseMeasurement.errorStatus);
#endif
            }
            else
            {
                Serial.println(SunriseBase::evaluateError(error, "read sensor measurement"));
            }

            if (calibration_state == CALIB_STATE_WAIT)
            {
                uint8_t calStatus = 0;
                error             = sensor.getCalibrationStatus(calStatus);
                if (error == SunriseError::NO_ERROR)
                {
                    if (calStatus == static_cast<uint8_t>(CalStatus::BACKGROUND))
                    {
                        Serial.println("Calibration: Success");
                        calibration_state = CALIB_STATE_IDLE;
                    }
                    else if (elapsed(currentTimeStamp, calibrationTimeStamp, sensorMeasurementPeriodMs * 3UL))
                    {
                        Serial.println("Calibration: NOT SUCCESSFUL, timed out");
                        calibration_state = CALIB_STATE_IDLE;
                    }
                }
                else
                {
                    Serial.println(SunriseBase::evaluateError(error, "read calibration status"));
                }
            }

            digitalWrite(LED_BUILTIN, ledValue);
            ledValue = ((ledValue == HIGH) ? LOW : HIGH);

            Serial.println("\nWaiting for next measurement...\n");
        }
        else if (lastMeasCountValid && elapsed(currentTimeStamp, lastNewMeasMs, sensorMeasurementPeriodMs * 3UL))
        {
            /* No new measurement in 3x the expected period — warn and reset the timer. */
            Serial.println("Warning: no new measurement detected, sensor may not be measuring.");
            lastNewMeasMs = currentTimeStamp;
        }
    }

    switch (calibration_state)
    {
    case CALIB_STATE_IDLE:
        if (digitalRead(CALIBRATE_BUTTON_PIN) == LOW)
        {
            if (elapsed(currentTimeStamp, lastButtonTime, BUTTON_DEBOUNCE_MS))
            {
                Serial.println("Calibration button pressed");
                calibration_state = CALIB_STATE_START;
            }
        }
        else
        {
            lastButtonTime = currentTimeStamp;
        }
        break;

    default:
        lastButtonTime = currentTimeStamp;
        break;
    }
}
