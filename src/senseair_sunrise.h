/**
 *******************************************************************************
 * @copyright   Copyright (C) by SenseAir AB. All rights reserved.
 * @file        senseair_sunrise.h
 * @brief       Public API for the Sunrise/Sunlight and S12 CO2 sensor library: class hierarchy,
 *              configuration/measurement structs, error codes, and register/bitmask definitions.
 *              Implements the "Modbus on Senseair Sunrise" and "I2C on Senseair Sunrise"
 *              documentation (available on the www.senseair.com website).
 * @details     Tested on Arduino Mega 2560, Arduino Nano 33 BLE
 *
 * @author      Senseair FW team
 * @version     1.0.0
 * @date        2026-06-01
 *
 *******************************************************************************
 */

#ifndef SENSEAIRSUNRISE_H
#define SENSEAIRSUNRISE_H

#include <Arduino.h>
#include <Stream.h>
#include <Wire.h>

/** @name Communication settings */
/** @{ */
constexpr int      SUNRISE_MODBUS_BAUDRATE       = 9600;    /**< Modbus baud rate (9600 only). */
constexpr long     SUNRISE_I2C_CLOCK_HZ          = 100000l; /**< I2C clock frequency (max 100 kHz). */
constexpr uint8_t  SUNRISE_DEFAULT_COMM_ADDRESS  = 0x68U;   /**< Default sensor address (104). */
constexpr uint8_t  SUNRISE_ANY_COMM_ADDRESS      = 0xFEU;   /**< Modbus "any sensor" address (254). */
constexpr uint16_t SUNRISE_STABILIZATION_MS      = 35u;     /**< Sunrise/Sunlight start-up time */
constexpr uint16_t SUNRISE_EEPROM_UPDATE_TIME_MS = 25u;     /**< Sunrise EEPROM update time */
/** @} */

/** @name S12-specific settings */
/** @{ */
constexpr long     S12_I2C_CLOCK_HZ          = 100000l; /**< I2C Fast Mode frequency (S12 supports up to 400 kHz). */
constexpr uint16_t S12_STABILIZATION_MS      = 30u;     /**< S12 start-up time */
constexpr uint16_t S12_EEPROM_UPDATE_TIME_MS = 180u;    /**< S12 EEPROM update time */
constexpr uint16_t S12_CALIBRATION_TIME_SINGLE_MODE_MS =
    275u; /**< S12 Calibration and EEPROM update time after measurement in single mode */
constexpr uint16_t S12_SINGLE_START_READY_MS =
    158u;                              /**< S12, start-up "Single Measurement Start" register write ignored time */
constexpr uint8_t S12_FW_TYPE = 0xC2u; /**< S12 firmware type identifier. */
/** @} */

/** @brief Error codes returned by library functions. */
enum class SunriseError
{
    NO_ERROR             = 0,
    ILLEGAL_FUNCTION     = -1,
    ILLEGAL_DATA_ADDRESS = -2,
    ILLEGAL_DATA_VALUE   = -3,
    DATA_TOO_LONG        = -123,
    NACK_ON_DATA         = -124,
    NACK_ON_ADDRESS      = -125,
    PACKET_TOO_SMALL     = -126,
    TIMEOUT_ERROR        = -127,
    COMMUNICATION_ERROR  = -128
};

/** @brief Sensor measurement modes. A system reset is required after changing. */
enum class MeasurementMode : uint8_t
{
    CONTINUOUS = 0, /**< Sensor measures continuously at configured period. */
    SINGLE     = 1  /**< Host triggers each measurement via start command. */
};

/** @brief Calibration commands written to the calibration command register. */
enum class CalCommand : uint16_t
{
    FACTORY_RESET = 0x7C02, /**< Restore factory calibration parameters. */
    FORCED_ABC    = 0x7C03, /**< Force ABC calibration if valid ABC data exists. */
    TARGET        = 0x7C05, /**< Calibrate using the calibration target value. */
    BACKGROUND    = 0x7C06, /**< Calibrate using ABC target as reference. */
    ZERO          = 0x7C07  /**< Calibrate using 0 ppm as reference. */
};

/** @brief Bitmask values read from the calibration status register after calibration. */
enum class CalStatus : uint16_t
{
    FACTORY_RESET = 0x0004, /**< Bit 2: Factory calibration restored. */
    ABC           = 0x0008, /**< Bit 3: ABC calibration completed. */
    TARGET        = 0x0010, /**< Bit 4: Target calibration completed. */
    BACKGROUND    = 0x0020, /**< Bit 5: Background calibration completed. */
    ZERO          = 0x0040  /**< Bit 6: Zero calibration completed. */
};

/** @brief Bitmask values for the sensor error status register. */
enum class ErrorStatus : uint16_t
{
    FATAL_ERROR       = 0x0001, /**< Bit 0: Fatal error, sensor is not working. */
    I2C_ERROR         = 0x0002, /**< Bit 1: I2C communication error. */
    ALGORITHM_ERROR   = 0x0004, /**< Bit 2: Algorithm error. */
    CALIBRATION_ERROR = 0x0008, /**< Bit 3: Calibration error. */
    SELF_DIAG_ERROR   = 0x0010, /**< Bit 4: Self-diagnostics error. */
    OUT_OF_RANGE      = 0x0020, /**< Bit 5: CO2 concentration out of range. */
    MEMORY_ERROR      = 0x0040, /**< Bit 6: Memory (EEPROM/flash) error. */
    NO_MEASUREMENT    = 0x0080, /**< Bit 7: No measurement completed yet. */
    LOW_VOLTAGE       = 0x0100, /**< Bit 8: Low internal regulator voltage. */
    MEAS_TIMEOUT      = 0x0200, /**< Bit 9: Measurement timeout. */
    ABNORMAL_SIGNAL   = 0x0400  /**< Bit 10: Abnormal signal level. */
};

/** @brief Basic sensor configuration (measurement mode, period, and samples). */
struct SunriseConfiguration
{
    MeasurementMode measMode;    /**< Measurement mode (continuous or single). */
    uint16_t        measPeriod;  /**< Measurement period in seconds (2–65534, continuous mode only). */
    uint16_t        measSamples; /**< Number of samples per measurement (1–1024). */
};

/** @brief Extended sensor configuration derived from the MeterControl register. */
struct SunriseExtendedConfiguration
{
    bool isABCEnabled;                  /**< Automatic Baseline Correction enabled. */
    bool isIIRFilterEnabled;            /**< Static and dynamic IIR filters enabled. */
    bool isPressureCompensationEnabled; /**< Pressure compensation enabled. */
    bool isNRDYPinEnabled;              /**< nRDY pin active (signals measurement ready). */
    bool isNRDYPinInverted;             /**< nRDY pin logic inverted (low during measurement). */
};

/** @brief Device identification data (Modbus Device ID objects 0x00–0x02). */
struct SunriseIdentification
{
    String vendorName;     /**< Object 0x00: Vendor name (e.g. "Senseair"). */
    String productCode;    /**< Object 0x01: Product code / article number. */
    String majMinRevision; /**< Object 0x02: Major.Minor firmware revision. */
};

/** @brief Measurement result from the sensor. */
struct SunriseMeasurement
{
    int16_t  co2value;    /**< Filtered, pressure-compensated CO2 concentration in ppm. */
    int16_t  temperature; /**< Chip temperature in degrees C x100 (e.g. 2223 = 22.23 C). */
    uint16_t errorStatus; /**< Sensor error status bitmask (0 = no errors). */
};

/**
 * @brief Opaque sensor state that must be saved between single-mode measurements.
 * @details In single measurement mode with the sensor powered down between cycles,
 *          the host must read this state after each measurement and write it back
 *          before the next one to preserve ABC and IIR filter continuity.
 */
struct SunriseSingleModeState
{
    static const uint8_t SENSOR_STATE_SZ = 15u;  /**< Number of 16-bit state registers. */
    uint16_t             state[SENSOR_STATE_SZ]; /**< ABC time, ABC params, filter params, pressure. */
};

/**
 * @brief Concentration scale factor (firmware 4.10+, Sunrise/Sunlight only).
 * @details scaled_concentration = co2_filtered * numerator / denominator.
 *          Both set to 0xFFFF disables scaling.
 */
struct SunriseScaleFactor
{
    uint16_t numerator;   /**< Scale factor numerator (0xFFFF = disabled). */
    uint16_t denominator; /**< Scale factor denominator (0xFFFF = disabled). */
};

/** @brief Firmware revision (main and sub version numbers). */
struct SunriseFirmwareRevision
{
    uint8_t revisionMain; /**< Main firmware version number. */
    uint8_t revisionSub;  /**< Sub firmware version number. */
};

/** @brief Register type: read-only (Input) or read/write (Holding). */
enum class RegType : uint8_t
{
    READ_ONLY, /**< Read-only register (Modbus Input Register, I2C read-only). */
    READ_WRITE /**< Read/write register (Modbus Holding Register, I2C read/write). */
};

/**
 * @brief Logical register definition with addresses for both protocols.
 * @details Maps one logical 16-bit sensor register to its Modbus and I2C addresses.
 *          i2cBytes specifies how many I2C bytes this register occupies (1 or 2).
 */
struct RegDef
{
    uint16_t modbusAddr; /**< Modbus register address. */
    uint8_t  i2cAddr;    /**< I2C register address. */
    uint8_t  i2cBytes;   /**< I2C byte width per logical register (1 or 2). */
    RegType  type;       /**< Register type (INPUT or HOLDING). */
    bool     eeprom;     /**< True if write requires EEPROM settle delay. */
};

/**
 * @brief Abstract transport interface for sensor communication.
 * @details Protocol-specific implementations (Modbus, I2C) translate logical
 *          register operations into protocol-specific I/O.
 */
class SunriseTransport
{
public:
    virtual ~SunriseTransport() = default;

    /** @brief Read consecutive 16-bit registers. */
    virtual SunriseError readRegs(const RegDef& reg, uint16_t count, uint16_t* values) = 0;
    /** @brief Write consecutive 16-bit registers. */
    virtual SunriseError writeRegs(const RegDef& reg, uint16_t count, const uint16_t* values) = 0;

    /** @brief Read a single 16-bit register. */
    SunriseError readU16(const RegDef& reg, uint16_t& value);
    /** @brief Write a single 16-bit register. */
    SunriseError writeU16(const RegDef& reg, uint16_t value);
    /** @brief Read a single 8-bit register (low byte of 16-bit value). */
    SunriseError readU8(const RegDef& reg, uint8_t& value);
    /** @brief Write a single 8-bit register (as 16-bit with high byte zero). */
    SunriseError writeU8(const RegDef& reg, uint8_t value);

    /** @brief Read sequential registers with mixed byte widths in one transaction. */
    virtual SunriseError readMulti(const RegDef regs[], uint16_t count, uint16_t* values) = 0;
    /** @brief Write sequential registers with mixed byte widths in one transaction. */
    virtual SunriseError writeMulti(const RegDef regs[], uint16_t count, const uint16_t* values) = 0;

    /** @brief Records the timestamp of the last EEPROM write. */
    virtual void eepromDelay() = 0;

    /** @brief Update the stored communication address. */
    virtual void setAddress(uint8_t address) = 0;

    /** @brief Returns the millis() value recorded at the last EEPROM write, or 0 if none has occurred. */
    unsigned long lastEepromWriteTime() const
    {
        return m_lastEepromWriteTime;
    }

protected:
    unsigned long m_lastEepromWriteTime = 0;
};

/**
 * @brief Modbus RTU transport implementation.
 * @details Handles serial framing, CRC generation, and Modbus function codes.
 */
class SunriseModbusTransport : public SunriseTransport
{
public:
    /** @brief Construct a Modbus transport bound to a serial stream and sensor address. */
    SunriseModbusTransport(Stream* serial, uint8_t commAddress);

    SunriseError readRegs(const RegDef& reg, uint16_t count, uint16_t* values) override;
    SunriseError writeRegs(const RegDef& reg, uint16_t count, const uint16_t* values) override;
    SunriseError readMulti(const RegDef regs[], uint16_t count, uint16_t* values) override;
    SunriseError writeMulti(const RegDef regs[], uint16_t count, const uint16_t* values) override;
    /** @brief No-op for Modbus (EEPROM settle handled by sensor between commands). */
    void eepromDelay() override;
    void setAddress(uint8_t address) override;

    /** @brief Read a Modbus Device Identification object (function code 0x2B). */
    SunriseError readIdentObject(uint8_t objId, String& objValue);

    /** @brief Read all Device Identification objects (vendor, product code, revision) into @p ident. */
    SunriseError readIdentification(SunriseIdentification& ident);

protected:
    Stream* m_serial = nullptr;
    uint8_t m_commAddress;
    uint8_t m_buffer[256]; /**< Working buffer for one full Modbus request/response PDU. */

    /** @brief Compute Modbus RTU CRC16 over a PDU. */
    static uint16_t generateCRC(uint8_t pdu[], int len);
    /** @brief Extract a 16-bit register value from the response buffer. */
    uint16_t getRegisterFromResponse(uint8_t registerNumber);
    /** @brief Validate response CRC and decode any exception codes. */
    SunriseError handler(uint8_t pdu[], uint8_t funCode, int len);
    /** @brief Wait for and read a Modbus response into the working buffer. */
    SunriseError modbusReadResponse(uint8_t funCode, uint8_t& availBytes);
    /** @brief Send a read request (function 0x03 or 0x04) and receive the response. */
    SunriseError readRegisters(uint8_t funCode, uint16_t regAddrStart, uint16_t howMany);
    /** @brief Send a "write multiple holding registers" request (function 0x10). */
    SunriseError writeMultipleRegisters(uint16_t regAddrStart, uint16_t howMany, const uint16_t writeData[]);
};

/**
 * @brief I2C transport implementation.
 * @details Handles Wire I/O, sensor wake-up, and byte packing.
 */
class SunriseI2CTransport : public SunriseTransport
{
public:
    /**
     * @brief Construct an I2C transport bound to a Wire bus and sensor address.
     * @param twoWire I2C bus instance to use for communication.
     * @param commAddress 7-bit sensor I2C address.
     */
    SunriseI2CTransport(TwoWire& twoWire, uint8_t commAddress);

    SunriseError readRegs(const RegDef& reg, uint16_t count, uint16_t* values) override;
    SunriseError writeRegs(const RegDef& reg, uint16_t count, const uint16_t* values) override;
    SunriseError readMulti(const RegDef regs[], uint16_t count, uint16_t* values) override;
    SunriseError writeMulti(const RegDef regs[], uint16_t count, const uint16_t* values) override;
    void         eepromDelay() override;
    void         setAddress(uint8_t address) override;

protected:
    /** @brief Wake the sensor from sleep before a transaction. Returns true on success. */
    virtual bool wakeUp();
    /** @brief Wake, send register address, and request the given number of bytes. */
    SunriseError requestFromRegisters(uint8_t regAddrStart, uint8_t howMany);
    /** @brief Wake, beginTransmission, and send the register address. */
    SunriseError beginWrite(uint8_t regAddrStart);
    /** @brief endTransmission and map the Wire result code to SunriseError. */
    SunriseError endWrite();

    TwoWire& m_twoWire;
    uint8_t  m_commAddress;
};

/**
 * @brief I2C transport for S12 sensors (no wake-up sequence required).
 */
class S12I2CTransport : public SunriseI2CTransport
{
public:
    /** @brief Construct an S12 I2C transport with the S12 EEPROM settle delay. */
    S12I2CTransport(TwoWire& twoWire, uint8_t commAddress);

protected:
    /** @brief S12 does not require a wake-up sequence; always succeeds. */
    bool wakeUp() override;
};

/**
 * @brief Base class for Senseair Sunrise/Sunlight/S12 sensor communication.
 * @details Sensor logic is written once here; protocol details are delegated
 *          to a SunriseTransport implementation (Modbus or I2C).
 */
class SunriseBase
{
public:
    /** @brief Construct with enable and nRDY pins. */
    SunriseBase(SunriseTransport& transport, int pinEnable, int pinNRDY);
    /** @brief Construct with enable pin only (no nRDY). */
    SunriseBase(SunriseTransport& transport, int pinEnable);
    /** @brief Construct without enable or nRDY pins. */
    SunriseBase(SunriseTransport& transport);

    /** @brief Update the stored communication address (call after setSensorAddress + reset). */
    void setCommAddress(uint8_t address);
    /** @brief Configure pin modes for enable and nRDY pins. Call once in setup(). */
    void begin();
    /** @brief Power on the sensor (drive EN high) and wait for stabilization. */
    virtual void enable();
    /** @brief Power off the sensor (drive EN low) and wait for any pending EEPROM write to complete. */
    void disable();

    /** @brief Convert a SunriseError to a human-readable String for diagnostics. */
    static String evaluateError(SunriseError error, const char* currentAction);

    /** @brief Extract ABC time (hours) from a saved single-mode state. */
    static uint16_t getABCTimeFromState(const SunriseSingleModeState& sunriseState);
    /** @brief Increment ABC time by one hour in a saved single-mode state. */
    static void incrementABCTimeInState(SunriseSingleModeState& sunriseState);

    /** @brief Write extended configuration (MeterControl register) to EEPROM. */
    SunriseError setSensorExtendedConfig(const SunriseExtendedConfiguration config);
    /** @brief Read extended configuration (MeterControl register) from EEPROM. */
    SunriseError getSensorExtendedConfig(SunriseExtendedConfiguration& config);

    /** @brief Enable Automatic Baseline Correction (clear bit 1 in MeterControl). */
    SunriseError enableABC();
    /** @brief Disable Automatic Baseline Correction (set bit 1 in MeterControl). */
    SunriseError disableABC();
    /** @brief Enable static and dynamic IIR filters (clear bits 2–3 in MeterControl). */
    SunriseError enableIIRFilters();
    /** @brief Disable static and dynamic IIR filters (set bits 2–3 in MeterControl). */
    SunriseError disableIIRFilters();
    /** @brief Enable nRDY pin output (clear bit 0 in MeterControl). */
    SunriseError enableNRDY();
    /** @brief Disable nRDY pin output (set bit 0 in MeterControl). */
    SunriseError disableNRDY();
    /** @brief Invert nRDY pin logic: low during measurement (clear bit 5 in MeterControl). */
    SunriseError enableNRDYInverted();
    /** @brief Normal nRDY pin logic: high during measurement (set bit 5 in MeterControl). */
    SunriseError disableNRDYInverted();
    /** @brief Enable pressure compensation (clear bit 4 in MeterControl). */
    SunriseError enablePressureComp();
    /** @brief Disable pressure compensation (set bit 4 in MeterControl). */
    SunriseError disablePressureComp();

    /** @brief Wait for nRDY pin toggle or measurement timeout. @return true if ready, false on timeout. */
    bool waitForReady(uint16_t measurementSamples);

    /** @brief Check if period and samples are within valid ranges for this sensor variant. */
    virtual bool isValidConfig(const SunriseConfiguration& config) = 0;

    /** @brief Write measurement mode, period, and number of samples. Requires reset. */
    SunriseError setSensorConfig(const SunriseConfiguration config);
    /** @brief Read current measurement mode, period, and number of samples. */
    SunriseError getSensorConfig(SunriseConfiguration& config);
    /** @brief Set measurement mode (continuous/single). Requires reset to activate. */
    SunriseError setMeasurementMode(MeasurementMode mode);

    /** @brief Read ABC period in hours. */
    SunriseError getABCPeriod(uint16_t& period);
    /** @brief Write ABC period in hours. */
    SunriseError setABCPeriod(uint16_t period);

    /** @brief Send a calibration command. Clears the status register first. */
    SunriseError sendCalibrationCommand(CalCommand cmd);
    /** @brief Read calibration status bitmask. Compare with CalStatus values. */
    SunriseError getCalibrationStatus(uint8_t& calStatus);
    /** @brief Set calibration target value in ppm (used by CalCommand::TARGET). */
    SunriseError setCalibrationTarget(int16_t target);

    /** @brief Set barometric pressure for compensation, in 0.1 hPa (e.g. 9970 = 997 hPa). */
    SunriseError setPressureValue(int16_t press);
    /** @brief Set new sensor communication address. Requires reset to activate. */
    SunriseError setSensorAddress(uint8_t address);
    /** @brief Read measurement counter (0–255, incremented after each measurement). */
    SunriseError getMeasurementCount(uint8_t& measCount);
    /** @brief Read firmware type identifier. */
    SunriseError getFirmwareType(uint8_t& type);
    /** @brief Read firmware revision (main and sub version numbers). */
    SunriseError getFirmwareRevision(SunriseFirmwareRevision& revision);
    /** @brief Read sensor serial number (32-bit unique identifier). */
    SunriseError getSensorSerialNumber(uint32_t& serialNumber);
    /** @brief Reset the sensor by writing 0xFF to the SCR register. */
    SunriseError resetBySCR();

    /** @brief Returns true if no EEPROM write has occurred, or if the sensor-specific EEPROM settle time has elapsed
     * since the last write. */
    bool isEepromSettled() const;

    /** @brief Write raw MeterControl register value to EEPROM. */
    SunriseError setMeterControl(uint8_t value);
    /** @brief Read raw MeterControl register value from EEPROM. */
    SunriseError getMeterControl(uint8_t& value);

    /**
     * @brief Perform the first single-mode measurement to collect initial state.
     * @note A NO_ERROR return only means the transaction succeeded. If the measurement-ready
     *       wait times out, the result may be stale; always check @p meas.errorStatus against
     *       ErrorStatus::NO_MEASUREMENT before trusting the values.
     */
    virtual SunriseError initialSingleMeasurement(SunriseMeasurement& meas, SunriseSingleModeState& sunriseState,
                                                  uint16_t measurementSamples);
    /**
     * @brief Perform a subsequent single-mode measurement using saved state.
     * @note See initialSingleMeasurement: check @p meas.errorStatus for ErrorStatus::NO_MEASUREMENT
     *       to confirm a fresh measurement completed.
     */
    virtual SunriseError readSingleMeasurement(SunriseMeasurement& meas, SunriseSingleModeState& sunriseState,
                                               uint16_t measurementSamples);
    /**
     * @brief Read a measurement in continuous mode.
     * @note A NO_ERROR return reflects only communication success. Check @p meas.errorStatus
     *       against ErrorStatus::NO_MEASUREMENT (and other ErrorStatus flags) to confirm the
     *       reading is valid.
     */
    SunriseError readMeasurement(SunriseMeasurement& meas);

protected:
    /** @brief EEPROM settle duration used by isEepromSettled(). Overridden by S12. */
    virtual uint16_t eepromUpdateTimeMs() const;

    /** @brief Set specified bits in the MeterControl register (read-modify-write). */
    SunriseError setMeterControlBits(uint8_t mask);
    /** @brief Clear specified bits in the MeterControl register (read-modify-write). */
    SunriseError clearMeterControlBits(uint8_t mask);
    /** @brief Read the full sensor state block into the given struct. */
    SunriseError readSensorState(SunriseSingleModeState& sunriseState);

    SunriseTransport* m_transport;
    bool              m_useNRDY;
    bool              m_useEnable;
    int               m_pinEnable = -1;
    int               m_pinNRDY   = -1;
};

/**
 * @brief Sunrise/Sunlight sensor with scale factor support (firmware 4.10+).
 */
class Sunrise : public SunriseBase
{
public:
    using SunriseBase::SunriseBase;

    bool isValidConfig(const SunriseConfiguration& config) override;

    /** @brief Read concentration scale factor (numerator/denominator). Firmware 4.10+. */
    SunriseError getScaleFactor(SunriseScaleFactor& factor);
    /** @brief Write concentration scale factor to EEPROM. Requires reset. Firmware 4.10+. */
    SunriseError setScaleFactor(const SunriseScaleFactor factor);
    /** @brief Read scaled measured concentration. Firmware 4.10+. */
    SunriseError getScaledMeasurement(int16_t& scaledConcentration);
};

/**
 * @brief S12 CO2 sensor with RDYN control and S12-specific timing.
 */
class S12 : public SunriseBase
{
public:
    using SunriseBase::SunriseBase;

    bool isValidConfig(const SunriseConfiguration& config) override;

    /** @brief Power on the S12 and record the timestamp for the start-measurement guard. */
    void enable() override;

    SunriseError initialSingleMeasurement(SunriseMeasurement& meas, SunriseSingleModeState& sunriseState,
                                          uint16_t measurementSamples) override;
    SunriseError readSingleMeasurement(SunriseMeasurement& meas, SunriseSingleModeState& sunriseState,
                                       uint16_t measurementSamples) override;

    /** @brief Set RDYN pin to open-drain output (set bit 6 in MeterControl). S12 only. */
    SunriseError enableRDYNOpenDrain();
    /** @brief Set RDYN pin to push-pull output (clear bit 6 in MeterControl). S12 only. */
    SunriseError disableRDYNOpenDrain();

protected:
    uint16_t eepromUpdateTimeMs() const override;

private:
    unsigned long m_enableTimestamp = 0;
};

/**
 * @brief Modbus RTU Sunrise/Sunlight sensor.
 */
class SunriseModbus : public Sunrise
{
public:
    /** @brief Construct with enable and nRDY pins. */
    SunriseModbus(Stream* serial, uint8_t commAddress, int pinEnable, int pinNRDY);
    /** @brief Construct with enable pin only (no nRDY). */
    SunriseModbus(Stream* serial, uint8_t commAddress, int pinEnable);
    /** @brief Construct without enable or nRDY pins. */
    SunriseModbus(Stream* serial, uint8_t commAddress);

    /** @brief Read device identification (vendor, product code, revision). Modbus only. */
    SunriseError getSensorID(SunriseIdentification& ident);

private:
    SunriseModbusTransport m_modbusTransport;
};

/**
 * @brief I2C Sunrise/Sunlight sensor.
 */
class SunriseI2C : public Sunrise
{
public:
    /** @brief Construct with enable and nRDY pins. */
    SunriseI2C(TwoWire& twoWire, uint8_t commAddress, int pinEnable, int pinNRDY);
    /** @brief Construct with enable pin only (no nRDY). */
    SunriseI2C(TwoWire& twoWire, uint8_t commAddress, int pinEnable);
    /** @brief Construct without enable or nRDY pins. */
    SunriseI2C(TwoWire& twoWire, uint8_t commAddress);

private:
    SunriseI2CTransport m_i2cTransport;
};

/**
 * @brief Modbus RTU S12 sensor.
 */
class S12Modbus : public S12
{
public:
    /** @brief Construct with enable and nRDY pins. */
    S12Modbus(Stream* serial, uint8_t commAddress, int pinEnable, int pinNRDY);
    /** @brief Construct with enable pin only (no nRDY). */
    S12Modbus(Stream* serial, uint8_t commAddress, int pinEnable);
    /** @brief Construct without enable or nRDY pins. */
    S12Modbus(Stream* serial, uint8_t commAddress);

    /** @brief Read device identification (vendor, product code, revision). Modbus only. */
    SunriseError getSensorID(SunriseIdentification& ident);

private:
    SunriseModbusTransport m_modbusTransport;
};

/**
 * @brief I2C S12 sensor.
 */
class S12I2C : public S12
{
public:
    /** @brief Construct with enable and nRDY pins. */
    S12I2C(TwoWire& twoWire, uint8_t commAddress, int pinEnable, int pinNRDY);
    /** @brief Construct with enable pin only (no nRDY). */
    S12I2C(TwoWire& twoWire, uint8_t commAddress, int pinEnable);
    /** @brief Construct without enable or nRDY pins. */
    S12I2C(TwoWire& twoWire, uint8_t commAddress);

private:
    S12I2CTransport m_i2cTransport;
};

#endif /*#ifndef SENSEAIRSUNRISE_H*/
