/**
 *******************************************************************************
   @copyright   Copyright (C) by SenseAir AB. All rights reserved.
   @file        sunrise_single.ino
   @brief       Unified example combining I2C and Modbus single mode operations
                for SenseAir Sunrise sensors. Based on sunrise_i2c_single.ino and
                sunrise_modbus_single.ino examples.
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
 * In single mode, measurement period will not have an effect on the sensor.
 * Instead the parameter is used by this host application to determine how often to send
 * measurement command to the sensor.
 */
#ifndef CHANGE_MEASUREMENT_CONFIGURATION
#define CHANGE_MEASUREMENT_CONFIGURATION 0 /**< Set to 1 to update base configuration */
#endif
constexpr MeasurementMode MEASUREMENT_MODE     = MeasurementMode::SINGLE;
constexpr uint16_t        MEASUREMENT_PERIOD_S = 16u;
constexpr uint16_t        MEASUREMENT_SAMPLES  = 8u;

/* Sensor extended configuration settings. Change if desired */
#ifndef CHANGE_EXTENDED_CONFIGURATION
#define CHANGE_EXTENDED_CONFIGURATION 0 /**< Set to 1 to update extended configuration */
#endif
constexpr bool ABC_ENABLED        = true;     /**< Enable/disable ABC */
constexpr bool IIR_FILTER_ENABLED = true;     /**< Enable/disable static and dynamic IIR.
                                                   It is recommended to disable filter for measurement period above 60s. */
constexpr bool PRESSURE_COMP_ENABLED = true;  /**< Enable/disable pressure compensation */
constexpr bool NRDY_PIN_ENABLED      = true;  /**< Enable/disable nRDY pin */
constexpr bool NRDY_PIN_INVERTED     = false; /**< Invert nRDY active level */

/* Sensor general configuration. Change if desired */
constexpr int16_t  PRESSURE_VALUE   = 9970; /* Unit in 0.1 hPa */
constexpr uint16_t ABC_PERIOD_HOURS = 180u;

/* Hardware configuration */
/* Sensor communication address, use 0xFE if address is unknown and only one sensor on the bus */
constexpr uint8_t SUNRISE_ADDR = SUNRISE_DEFAULT_COMM_ADDRESS;
/* Pin to enable/disable sensor */
constexpr int SUNRISE_EN_PIN = 5;
/* Pin to select communication protocol (COMSEL): LOW = I2C, HIGH = Modbus */
constexpr int SUNRISE_COMSEL_PIN = 4;
/* Pin to read nRDY signal state */
constexpr int SUNRISE_NRDY = 7;

/* Pin to be grounded for calibration. Change if desired. */
constexpr int CALIBRATE_BUTTON_PIN = 6;
/* Calibration button debounce time */
constexpr uint32_t BUTTON_DEBOUNCE_MS = 200;

/* Create Sunrise object */
#ifdef SUNRISE_PROTOCOL_I2C
SunriseI2C sunrise = SunriseI2C(SUNRISE_SERIAL, SUNRISE_ADDR, SUNRISE_EN_PIN, SUNRISE_NRDY);
#else
SunriseModbus sunrise = SunriseModbus(&SUNRISE_SERIAL, SUNRISE_ADDR, SUNRISE_EN_PIN, SUNRISE_NRDY);
#endif

/* Global constant used to keep track of hour interval */
constexpr unsigned long HOUR_INTERVAL_MS = 3600000ul;

/* Global parameters */
static int           ledValue             = HIGH;
static unsigned long measurementTimeStamp = 0ul;
/* Logging interval for sensor */
static unsigned long sunriseMeasurementPeriodMs = MEASUREMENT_PERIOD_S * 1000U;

static SunriseSingleModeState sunriseState;

/** Return true if time is passed */
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

    /* Initializes serial communication */
    Serial.begin(115200);
    /* Delay to wait until the Arduino reconnect to serial after upload */
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
#else
    digitalWrite(SUNRISE_COMSEL_PIN, HIGH);
    SUNRISE_SERIAL.begin(SUNRISE_MODBUS_BAUDRATE);
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
                    "Error: S12 sensor detected. Use the s12_continuous/s12_single example for S12 sensors.");
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

#ifdef SUNRISE_PROTOCOL_MODBUS
    /* Reads and prints sensor ID data */
    SunriseIdentification sunriseIdentification;
    error = sunrise.getSensorID(sunriseIdentification);
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
#endif

    {
        SunriseConfiguration baseConfiguration;
        error = sunrise.getSensorConfig(baseConfiguration);
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

            if (baseConfiguration.measMode == MeasurementMode::CONTINUOUS)
            {
                if (!(CHANGE_MEASUREMENT_CONFIGURATION && (MEASUREMENT_MODE == MeasurementMode::SINGLE)))
                {
                    Serial.println("Sensor is in continuous mode, not supported by this example!");
                    Serial.println("Set CHANGE_MEASUREMENT_CONFIGURATION=true and MEASUREMENT_MODE=SINGLE.");
                    while (true)
                        ;
                }
            }

            if (CHANGE_MEASUREMENT_CONFIGURATION)
            {
                if ((baseConfiguration.measMode != MEASUREMENT_MODE) ||
                    (baseConfiguration.measSamples != MEASUREMENT_SAMPLES))
                {
                    baseConfiguration.measMode    = MEASUREMENT_MODE;
                    baseConfiguration.measSamples = MEASUREMENT_SAMPLES;
                    error                         = sunrise.setSensorConfig(baseConfiguration);
                    Serial.println((error == SunriseError::NO_ERROR)
                                       ? "New configuration set."
                                       : SunriseBase::evaluateError(error, "set sensor configuration"));
                    isResetNeeded = true;
                }
                else
                {
                    Serial.println("Configuration already matched.");
                }
            }
        }
        else
        {
            Serial.println(SunriseBase::evaluateError(error, "read sensor configuration"));
        }
    }

    {
        SunriseExtendedConfiguration extendedConfiguration;
        error = sunrise.getSensorExtendedConfig(extendedConfiguration);
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

            if (CHANGE_EXTENDED_CONFIGURATION)
            {
                if (extendedConfiguration.isABCEnabled != ABC_ENABLED ||
                    extendedConfiguration.isIIRFilterEnabled != IIR_FILTER_ENABLED ||
                    extendedConfiguration.isPressureCompensationEnabled != PRESSURE_COMP_ENABLED ||
                    extendedConfiguration.isNRDYPinEnabled != NRDY_PIN_ENABLED ||
                    extendedConfiguration.isNRDYPinInverted != NRDY_PIN_INVERTED)
                {
                    error = sunrise.setSensorExtendedConfig(
                        {ABC_ENABLED, IIR_FILTER_ENABLED, PRESSURE_COMP_ENABLED, NRDY_PIN_ENABLED, NRDY_PIN_INVERTED});
                    Serial.println((error == SunriseError::NO_ERROR)
                                       ? "Sensor extended configuration updated"
                                       : SunriseBase::evaluateError(error, "set sensor extended configuration"));
                    isResetNeeded = true;
                }
            }
        }
        else
        {
            Serial.println(SunriseBase::evaluateError(error, "read sensor extended configuration"));
        }

        /* Sets ABC period if it differs from the current period on sensor */
        {
            uint16_t abcPeriodTemp = 0;
            error                  = sunrise.getABCPeriod(abcPeriodTemp);
            if (error == SunriseError::NO_ERROR)
            {
                if (ABC_PERIOD_HOURS != abcPeriodTemp)
                {
                    /* Set ABC period */
                    error = sunrise.setABCPeriod(ABC_PERIOD_HOURS);
                    Serial.println((error == SunriseError::NO_ERROR)
                                       ? "New ABC period set"
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
    }

    /* SCR reset to read new values from EEPROM */
    if (isResetNeeded)
    {
        error = sunrise.resetBySCR();
        if (error == SunriseError::NO_ERROR)
        {
            Serial.println("Restarting sensor to read new values from EEPROM.");
            delay(2 * SUNRISE_STABILIZATION_MS);
        }
        else
        {
            Serial.println(SunriseBase::evaluateError(error, "reset through SCR"));
        }
    }

    /* Sets pressure value */
    if (PRESSURE_VALUE != 0)
    {
        error = sunrise.setPressureValue(PRESSURE_VALUE);
        Serial.println((error == SunriseError::NO_ERROR) ? "Setting pressure."
                                                         : SunriseBase::evaluateError(error, "set pressure value"));
    }

    /* Performs an initial measurement to store Sensor State data */
    do
    {
        SunriseMeasurement sunriseMeasurement = {0, 0, 0u};
        error = sunrise.initialSingleMeasurement(sunriseMeasurement, sunriseState, MEASUREMENT_SAMPLES);
        if (error == SunriseError::NO_ERROR)
        {
            Serial.println("Performing initial measurement to store sensor data.");
            Serial.print("CO2: ");
            Serial.print(sunriseMeasurement.co2value);
            Serial.println(" ppm");
            Serial.print("Temp: ");
            Serial.print(sunriseMeasurement.temperature / (100.0f));
            Serial.println(" degree C");
            Serial.print("Error Status: 0x");
            Serial.println(sunriseMeasurement.errorStatus, HEX);
            printErrorStatus(sunriseMeasurement.errorStatus);
            Serial.print("Current ABC time: ");
            Serial.println(sunrise.getABCTimeFromState(sunriseState));
            Serial.println("\nWaiting for next measurement...\n");
            measurementTimeStamp = millis();
        }
        else
        {
            Serial.println(SunriseBase::evaluateError(error, "read initial sensor measurement"));
            Serial.println("Attempting again in 10 seconds");
            delay(10000);
        }
    } while (error != SunriseError::NO_ERROR);

    sunrise.disable();
}

void loop()
{
    static enum { CALIB_STATE_IDLE = 0, CALIB_STATE_START, CALIB_STATE_WAIT } calibration_state = CALIB_STATE_IDLE;
    static unsigned long lastButtonTime                                                         = 0ul;
    static unsigned long abcTimeStamp                                                           = 0ul;

    unsigned long currentTimeStamp = millis();

    /* Enables sensor and perform measurement cycle every measurement period */
    if (elapsed(currentTimeStamp, measurementTimeStamp, sunriseMeasurementPeriodMs))
    {
        SunriseError error;

        measurementTimeStamp = currentTimeStamp;
        sunrise.enable();

        /* Send calibration command after button press */
        if (calibration_state == CALIB_STATE_START)
        {
            error = sunrise.sendCalibrationCommand(CalCommand::BACKGROUND);
            if (error == SunriseError::NO_ERROR)
            {
                Serial.println("Calibration command sent.");
                calibration_state = CALIB_STATE_WAIT;
            }
            else
            {
                calibration_state = CALIB_STATE_IDLE;
                Serial.println(SunriseBase::evaluateError(error, "send calibration command"));
            }
        }

        /* Performs a measurement and print results */
        SunriseMeasurement sunriseMeasurement = {0, 0, 0u};
        error = sunrise.readSingleMeasurement(sunriseMeasurement, sunriseState, MEASUREMENT_SAMPLES);
        if (error == SunriseError::NO_ERROR)
        {
#ifdef SERIAL_PLOTTER_MODE
            /* Output optimized for Arduino "Serial Plotter" tool */
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
            Serial.print("Current ABC time: ");
            Serial.println(sunrise.getABCTimeFromState(sunriseState));
#endif
        }
        else
        {
            Serial.println(SunriseBase::evaluateError(error, "read sensor measurement"));
        }

        /* If a calibration command has previously been sent, checks calibration status. */
        if (calibration_state == CALIB_STATE_WAIT)
        {
            uint8_t calStatus = 0;
            error             = sunrise.getCalibrationStatus(calStatus);
            if (error == SunriseError::NO_ERROR)
            {
                if (calStatus == static_cast<uint8_t>(CalStatus::BACKGROUND))
                {
                    Serial.println("Calibration: Success");
                    calibration_state = CALIB_STATE_IDLE;
                }
                else
                {
                    Serial.println("Calibration: NOT SUCCESSFUL");
                    calibration_state = CALIB_STATE_IDLE;
                }
            }
            else
            {
                Serial.println(SunriseBase::evaluateError(error, "read calibration status"));
            }
        }

        /* Toggle working state */
        digitalWrite(LED_BUILTIN, ledValue);
        ledValue = ((ledValue == HIGH) ? LOW : HIGH);

        Serial.println("\nWaiting for next measurement...\n");
        sunrise.disable();
    }

    /* Set 'send calibration command' flag on button press */
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

    /* Increase ABC every hour */
    if (elapsed(currentTimeStamp, abcTimeStamp, HOUR_INTERVAL_MS))
    {
        if (ABC_ENABLED)
        {
            sunrise.incrementABCTimeInState(sunriseState);
            abcTimeStamp = currentTimeStamp;
            Serial.print("ABC Time incremented:");
            Serial.println(sunrise.getABCTimeFromState(sunriseState));
        }
    }
}