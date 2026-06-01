/**
 *******************************************************************************
 * @copyright   Copyright (C) by SenseAir AB. All rights reserved.
 * @file        senseair_sunrise.cpp
 * @brief       Implementation of the Sunrise/Sunlight and S12 sensor library: Modbus RTU and I2C
 *              transports (framing, CRC, wake-up, byte packing) and the shared sensor logic built
 *              on top of them.
 * @details     Tested on Arduino Mega 2560, Arduino Nano 33 BLE
 *
 * @author      Senseair FW team
 * @version     1.0.0
 * @date        2026-06-01
 *
 *******************************************************************************
 */

#include "senseair_sunrise.h"

/* ============================================================================
 * File-scope constants
 * ============================================================================ */

/* Modbus framing */
/* For baudrate equal 9600 the Modbus 3.5T interval is close to 3.5 ms, we round it to 6 ms as some have not accurate
 * 1ms system timer */
static constexpr int     INTER_PACKET_INTERVAL_MS = 6;
static constexpr int     MODBUS_TIMEOUT           = 180;
static constexpr uint8_t MODBUS_FUN_READ_HOLDING  = 0x03u;
static constexpr uint8_t MODBUS_FUN_READ_INPUT    = 0x04u;
static constexpr uint8_t MODBUS_FUN_WRITE_MULTI   = 0x10u;
static constexpr uint8_t MODBUS_FUN_READ_IDENT    = 0x2Bu;

/* Modbus exception codes */
static constexpr int MODBUS_ILLEGAL_FUNCTION     = 1;
static constexpr int MODBUS_ILLEGAL_DATA_ADDRESS = 2;
static constexpr int MODBUS_ILLEGAL_DATA_VALUE   = 3;

/* Modbus Device Identification object IDs */
static constexpr uint8_t IDENT_VENDOR_NAME  = 0u;
static constexpr uint8_t IDENT_PRODUCT_CODE = 1u;
static constexpr uint8_t IDENT_REVISION     = 2u;

/* Timing */
static constexpr uint8_t  POLL_NRDY_DELAY_MS            = 100u;
static constexpr uint16_t SINGLE_MEASUREMENT_TIMEOUT_MS = 300u;

/* Meter control bitmasks */
static constexpr uint8_t METER_CONTROL_NRDY            = 0x01u;
static constexpr uint8_t METER_CONTROL_ABC             = 0x02u;
static constexpr uint8_t METER_CONTROL_FILTERS         = 0x0Cu;
static constexpr uint8_t METER_CONTROL_PRESSURE        = 0x10u;
static constexpr uint8_t METER_CONTROL_NRDY_INVERTED   = 0x20u;
static constexpr uint8_t METER_CONTROL_RDYN_OPEN_DRAIN = 0x40u;

/* Register map: each register has Modbus and I2C addresses + I2C byte width */
static constexpr RegDef REG_ERROR_STATUS  = {0x0000, 0x00, 2, RegType::READ_ONLY, false};
static constexpr RegDef REG_MEAS_COUNT    = {0x0006, 0x0D, 1, RegType::READ_ONLY, false};
static constexpr RegDef REG_SCALED_CONC   = {0x000D, 0x1A, 2, RegType::READ_ONLY, false};
static constexpr RegDef REG_FW_TYPE       = {0x0017, 0x2F, 1, RegType::READ_ONLY, false};
static constexpr RegDef REG_FW_REV        = {0x001C, 0x38, 2, RegType::READ_ONLY, false};
static constexpr RegDef REG_SENSOR_ID     = {0x001D, 0x3A, 2, RegType::READ_ONLY, false};
static constexpr RegDef REG_CAL_STATUS    = {0x0000, 0x81, 1, RegType::READ_WRITE, false};
static constexpr RegDef REG_CAL_COMMAND   = {0x0001, 0x82, 2, RegType::READ_WRITE, false};
static constexpr RegDef REG_CAL_TARGET    = {0x0002, 0x84, 2, RegType::READ_WRITE, false};
static constexpr RegDef REG_MEAS_MODE     = {0x000A, 0x95, 1, RegType::READ_WRITE, true};
static constexpr RegDef REG_MEAS_PERIOD   = {0x000B, 0x96, 2, RegType::READ_WRITE, true};
static constexpr RegDef REG_MEAS_SAMPLES  = {0x000C, 0x98, 2, RegType::READ_WRITE, true};
static constexpr RegDef REG_ABC_PERIOD    = {0x000D, 0x9A, 2, RegType::READ_WRITE, true};
static constexpr RegDef REG_SCR           = {0x0011, 0xA3, 1, RegType::READ_WRITE, false};
static constexpr RegDef REG_METER_CONTROL = {0x0012, 0xA5, 1, RegType::READ_WRITE, true};
static constexpr RegDef REG_SENSOR_ADDR   = {0x0013, 0xA7, 1, RegType::READ_WRITE, true};
static constexpr RegDef REG_SCALE_NUM     = {0x0014, 0xA8, 2, RegType::READ_WRITE, true};
static constexpr RegDef REG_SCALE_DEN     = {0x0015, 0xAA, 2, RegType::READ_WRITE, true};
static constexpr RegDef REG_START_MEAS    = {0x0021, 0xC3, 1, RegType::READ_WRITE, false};
static constexpr RegDef REG_SENSOR_STATE  = {0x0021, 0xC2, 2, RegType::READ_WRITE, false};
static constexpr RegDef REG_BARO_PRESS    = {0x002E, 0xDC, 2, RegType::READ_WRITE, false};

/* ============================================================================
 * File-scope helpers
 * ============================================================================ */

/* Big-endian byte order helpers (sensor registers are big-endian) */
static inline uint16_t beToU16(uint8_t hi, uint8_t lo)
{
    return (static_cast<uint16_t>(hi) << 8u) | static_cast<uint16_t>(lo);
}

static inline uint8_t u16Hi(uint16_t val)
{
    return static_cast<uint8_t>(val >> 8u);
}

static inline uint8_t u16Lo(uint16_t val)
{
    return static_cast<uint8_t>(val);
}

/* Map a Wire endTransmission() status code to a SunriseError. */
static SunriseError mapEndTransmission(uint8_t code)
{
    switch (code)
    {
    case 0:
        return SunriseError::NO_ERROR;
    case 1:
        return SunriseError::DATA_TOO_LONG;
    case 2:
        return SunriseError::NACK_ON_ADDRESS;
    case 3:
        return SunriseError::NACK_ON_DATA;
    default:
        return SunriseError::COMMUNICATION_ERROR;
    }
}

/* True if `next` immediately follows `prev` in I2C address space. */
static bool i2cIsContiguous(const RegDef& prev, const RegDef& next)
{
    return next.i2cAddr == prev.i2cAddr + prev.i2cBytes;
}

/* Sunrise/Sunlight config validation. */
static bool isValidSunriseConfig(const SunriseConfiguration& config)
{
    if (config.measPeriod < 2 || config.measPeriod > 65534)
    {
        return false;
    }
    if (config.measSamples < 1 || config.measSamples > 1024)
    {
        return false;
    }
    return true;
}

/* S12 config validation.
 * Per the S12 datasheet, the valid "number of samples" values are:
 *   0-20  : any value (0 selects the sensor default),
 *   29-79 : in steps of 10,
 *   99-999: in steps of 50.
 * Measurement period is 1-2047 s. */
static bool isValidS12Config(const SunriseConfiguration& config)
{
    uint16_t s = config.measSamples;
    if (config.measPeriod < 1 || config.measPeriod > 2047)
    {
        return false;
    }
    if (s <= 20)
    {
        return true;
    }
    if (s >= 29 && s <= 79 && ((s - 29) % 10 == 0))
    {
        return true;
    }
    if (s >= 99 && s <= 999 && ((s - 99) % 50 == 0))
    {
        return true;
    }
    return false;
}

/* ============================================================================
 * SunriseTransport - non-virtual convenience methods
 * ============================================================================ */

SunriseError SunriseTransport::readU16(const RegDef& reg, uint16_t& value)
{
    return readRegs(reg, 1u, &value);
}

SunriseError SunriseTransport::writeU16(const RegDef& reg, uint16_t value)
{
    return writeRegs(reg, 1u, &value);
}

SunriseError SunriseTransport::readU8(const RegDef& reg, uint8_t& value)
{
    uint16_t     val16;
    SunriseError error = readRegs(reg, 1u, &val16);
    if (error == SunriseError::NO_ERROR)
    {
        value = u16Lo(val16);
    }
    return error;
}

SunriseError SunriseTransport::writeU8(const RegDef& reg, uint8_t value)
{
    uint16_t val16 = value;
    return writeRegs(reg, 1u, &val16);
}

/* ============================================================================
 * SunriseModbusTransport
 * ============================================================================ */

SunriseModbusTransport::SunriseModbusTransport(Stream* serial, uint8_t commAddress)
    : m_serial(serial), m_commAddress(commAddress)
{
}

void SunriseModbusTransport::setAddress(uint8_t address)
{
    m_commAddress = address;
}

void SunriseModbusTransport::eepromDelay()
{
    m_lastEepromWriteTime = millis();
}

SunriseError SunriseModbusTransport::readRegs(const RegDef& reg, uint16_t count, uint16_t* values)
{
    uint8_t      funCode = (reg.type == RegType::READ_ONLY) ? MODBUS_FUN_READ_INPUT : MODBUS_FUN_READ_HOLDING;
    SunriseError error   = readRegisters(funCode, reg.modbusAddr, count);
    if (error == SunriseError::NO_ERROR)
    {
        for (uint16_t i = 0; i < count; i++)
        {
            values[i] = getRegisterFromResponse(i);
        }
    }
    return error;
}

SunriseError SunriseModbusTransport::writeRegs(const RegDef& reg, uint16_t count, const uint16_t* values)
{
    return writeMultipleRegisters(reg.modbusAddr, count, values);
}

SunriseError SunriseModbusTransport::readMulti(const RegDef regs[], uint16_t count, uint16_t* values)
{
    uint16_t     runStart = 0;
    SunriseError error    = SunriseError::NO_ERROR;
    for (uint16_t i = 1; i <= count; i++)
    {
        if (i == count || regs[i].modbusAddr != regs[i - 1].modbusAddr + 1)
        {
            uint16_t runLen = i - runStart;
            error           = readRegs(regs[runStart], runLen, &values[runStart]);
            if (error != SunriseError::NO_ERROR)
            {
                return error;
            }
            runStart = i;
        }
    }
    return error;
}

SunriseError SunriseModbusTransport::writeMulti(const RegDef regs[], uint16_t count, const uint16_t* values)
{
    uint16_t     runStart = 0;
    SunriseError error    = SunriseError::NO_ERROR;
    for (uint16_t i = 1; i <= count; i++)
    {
        if (i == count || regs[i].modbusAddr != regs[i - 1].modbusAddr + 1)
        {
            uint16_t runLen = i - runStart;
            error           = writeRegs(regs[runStart], runLen, &values[runStart]);
            if (error != SunriseError::NO_ERROR)
            {
                return error;
            }
            runStart = i;
        }
    }
    return error;
}

SunriseError SunriseModbusTransport::readIdentObject(uint8_t objId, String& objValue)
{
    uint8_t availBytes;

    m_buffer[0] = m_commAddress;
    m_buffer[1] = MODBUS_FUN_READ_IDENT;
    m_buffer[2] = 0x0E;
    m_buffer[3] = 0x04;
    m_buffer[4] = objId;

    uint16_t crc = generateCRC(m_buffer, 5);
    m_buffer[5]  = crc & 0xFF;
    m_buffer[6]  = (crc >> 8);

    m_serial->write(m_buffer, 7);

    SunriseError error = modbusReadResponse(MODBUS_FUN_READ_IDENT, availBytes);
    objValue           = "";
    if ((availBytes >= 11u) && (error == SunriseError::NO_ERROR))
    {
        uint8_t objLength = m_buffer[9u];
        for (uint8_t slot = 10u; slot < (10u + objLength); ++slot)
        {
            objValue += (char)m_buffer[slot];
        }
    }
    else if (error == SunriseError::NO_ERROR)
    {
        error = SunriseError::PACKET_TOO_SMALL;
    }
    return error;
}

SunriseError SunriseModbusTransport::readIdentification(SunriseIdentification& ident)
{
    SunriseError error = readIdentObject(IDENT_VENDOR_NAME, ident.vendorName);
    error = (error == SunriseError::NO_ERROR) ? readIdentObject(IDENT_PRODUCT_CODE, ident.productCode) : error;
    error = (error == SunriseError::NO_ERROR) ? readIdentObject(IDENT_REVISION, ident.majMinRevision) : error;
    return error;
}

uint16_t SunriseModbusTransport::generateCRC(uint8_t pdu[], int len)
{
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)pdu[pos];
        for (int n = 8; n != 0; n--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

uint16_t SunriseModbusTransport::getRegisterFromResponse(uint8_t registerNumber)
{
    uint16_t offset = (uint16_t)registerNumber * 2 + 3;
    return beToU16(m_buffer[offset], m_buffer[offset + 1]);
}

SunriseError SunriseModbusTransport::handler(uint8_t pdu[], uint8_t funCode, int len)
{
    SunriseError error            = SunriseError::NO_ERROR;
    uint8_t      exceptionFunCode = funCode + 0x80;
    if (len >= 4)
    {
        uint16_t crc   = generateCRC(pdu, (len - 2));
        uint8_t  crcHi = (crc >> 8);
        uint8_t  crcLo = crc & 0xFF;

        if (crcLo != pdu[len - 2] || crcHi != pdu[len - 1])
        {
            error = SunriseError::COMMUNICATION_ERROR;
        }
        else if (pdu[1] == exceptionFunCode)
        {
            switch (pdu[2])
            {
            case MODBUS_ILLEGAL_FUNCTION:
                error = SunriseError::ILLEGAL_FUNCTION;
                break;
            case MODBUS_ILLEGAL_DATA_ADDRESS:
                error = SunriseError::ILLEGAL_DATA_ADDRESS;
                break;
            case MODBUS_ILLEGAL_DATA_VALUE:
                error = SunriseError::ILLEGAL_DATA_VALUE;
                break;
            default:
                error = SunriseError::COMMUNICATION_ERROR;
                break;
            }
        }
    }
    else
    {
        error = SunriseError::PACKET_TOO_SMALL;
    }
    return error;
}

SunriseError SunriseModbusTransport::modbusReadResponse(uint8_t funCode, uint8_t& availBytes)
{
    unsigned long byteTime = millis();
    availBytes             = 0;

    while ((availBytes = m_serial->available()) == 0)
    {
        if (millis() - byteTime > (unsigned long)MODBUS_TIMEOUT)
        {
            return SunriseError::TIMEOUT_ERROR;
        }
    }

    byteTime = millis();
    unsigned long timestamp;
    do
    {
        int newAvailableBytes = m_serial->available();
        timestamp             = millis();
        if (availBytes != newAvailableBytes)
        {
            byteTime   = timestamp;
            availBytes = newAvailableBytes;
        }
    } while (timestamp - byteTime < (unsigned long)INTER_PACKET_INTERVAL_MS);

    for (int n = 0; n < availBytes; n++)
    {
        m_buffer[n] = m_serial->read();
    }

    return handler(m_buffer, funCode, availBytes);
}

SunriseError SunriseModbusTransport::readRegisters(uint8_t funCode, uint16_t regAddrStart, uint16_t howMany)
{
    m_buffer[0] = m_commAddress;
    m_buffer[1] = funCode;
    m_buffer[2] = (regAddrStart >> 8);
    m_buffer[3] = regAddrStart & 0xFF;
    m_buffer[4] = (howMany >> 8);
    m_buffer[5] = howMany & 0xFF;

    uint16_t crc = generateCRC(m_buffer, 6);
    m_buffer[6]  = crc & 0xFF;
    m_buffer[7]  = (crc >> 8);

    m_serial->write(m_buffer, 8);

    uint8_t      availBytes;
    SunriseError error = modbusReadResponse(funCode, availBytes);
    if ((error == SunriseError::NO_ERROR) && (availBytes < 5 + howMany * 2))
    {
        error = SunriseError::PACKET_TOO_SMALL;
    }
    return error;
}

SunriseError SunriseModbusTransport::writeMultipleRegisters(uint16_t regAddrStart, uint16_t howMany,
                                                            const uint16_t writeData[])
{
    uint8_t numBytes = howMany * 2;
    if (numBytes > 246)
    {
        return SunriseError::DATA_TOO_LONG;
    }

    int requestSize = 7 + numBytes;
    m_buffer[0]     = m_commAddress;
    m_buffer[1]     = MODBUS_FUN_WRITE_MULTI;
    m_buffer[2]     = (regAddrStart >> 8);
    m_buffer[3]     = regAddrStart & 0xFFu;
    m_buffer[4]     = (howMany >> 8);
    m_buffer[5]     = howMany & 0xFFu;
    m_buffer[6]     = numBytes;

    uint8_t counter = 7;
    for (uint16_t n = 0; n < howMany; n++)
    {
        m_buffer[counter]     = u16Hi(writeData[n]);
        m_buffer[counter + 1] = u16Lo(writeData[n]);
        counter += 2;
    }

    uint16_t crc              = generateCRC(m_buffer, requestSize);
    m_buffer[requestSize]     = crc & 0xFF;
    m_buffer[requestSize + 1] = (crc >> 8);
    requestSize += 2;

    m_serial->write(m_buffer, requestSize);
    uint8_t availBytes;
    return modbusReadResponse(MODBUS_FUN_WRITE_MULTI, availBytes);
}

/* ============================================================================
 * SunriseI2CTransport
 * ============================================================================ */

SunriseI2CTransport::SunriseI2CTransport(TwoWire& twoWire, uint8_t commAddress)
    : m_twoWire(twoWire), m_commAddress(commAddress)
{
}

void SunriseI2CTransport::setAddress(uint8_t address)
{
    m_commAddress = address;
}

void SunriseI2CTransport::eepromDelay()
{
    m_lastEepromWriteTime = millis();
}

SunriseError SunriseI2CTransport::readRegs(const RegDef& reg, uint16_t count, uint16_t* values)
{
    uint8_t      totalBytes = reg.i2cBytes * count;
    SunriseError error      = requestFromRegisters(reg.i2cAddr, totalBytes);
    if (error == SunriseError::NO_ERROR)
    {
        for (uint16_t i = 0; i < count; i++)
        {
            if (reg.i2cBytes == 1)
            {
                values[i] = m_twoWire.read();
            }
            else
            {
                uint8_t hi = m_twoWire.read();
                uint8_t lo = m_twoWire.read();
                values[i]  = beToU16(hi, lo);
            }
        }
    }
    return error;
}

SunriseError SunriseI2CTransport::writeRegs(const RegDef& reg, uint16_t count, const uint16_t* values)
{
    SunriseError error = beginWrite(reg.i2cAddr);
    if (error == SunriseError::NO_ERROR)
    {
        for (uint16_t i = 0; i < count; i++)
        {
            if (reg.i2cBytes == 2)
            {
                m_twoWire.write(u16Hi(values[i]));
            }
            m_twoWire.write(u16Lo(values[i]));
        }
        error = endWrite();
        if (reg.eeprom)
        {
            eepromDelay();
        }
    }
    return error;
}

SunriseError SunriseI2CTransport::readMulti(const RegDef regs[], uint16_t count, uint16_t* values)
{
    uint16_t     runStart = 0;
    SunriseError error    = SunriseError::NO_ERROR;
    for (uint16_t i = 1; i <= count; i++)
    {
        if (i == count || !i2cIsContiguous(regs[i - 1], regs[i]))
        {
            uint8_t totalBytes = 0;
            for (uint16_t j = runStart; j < i; j++)
            {
                totalBytes += regs[j].i2cBytes;
            }

            error = requestFromRegisters(regs[runStart].i2cAddr, totalBytes);
            if (error != SunriseError::NO_ERROR)
            {
                return error;
            }
            for (uint16_t j = runStart; j < i; j++)
            {
                if (regs[j].i2cBytes == 1)
                {
                    values[j] = m_twoWire.read();
                }
                else
                {
                    uint8_t hi = m_twoWire.read();
                    uint8_t lo = m_twoWire.read();
                    values[j]  = beToU16(hi, lo);
                }
            }
            runStart = i;
        }
    }
    return error;
}

SunriseError SunriseI2CTransport::writeMulti(const RegDef regs[], uint16_t count, const uint16_t* values)
{
    uint16_t     runStart  = 0;
    SunriseError error     = SunriseError::NO_ERROR;
    bool         hasEeprom = false;
    for (uint16_t i = 1; (i <= count) && (error == SunriseError::NO_ERROR); i++)
    {
        if (i == count || !i2cIsContiguous(regs[i - 1], regs[i]))
        {
            error = beginWrite(regs[runStart].i2cAddr);
            if (error == SunriseError::NO_ERROR)
            {
                for (uint16_t j = runStart; j < i; j++)
                {
                    hasEeprom |= regs[j].eeprom;
                    if (regs[j].i2cBytes == 2)
                    {
                        m_twoWire.write(u16Hi(values[j]));
                    }
                    m_twoWire.write(u16Lo(values[j]));
                }
                error = endWrite();
            }
            runStart = i;
        }
    }

    if (hasEeprom)
    {
        eepromDelay();
    }
    return error;
}

bool SunriseI2CTransport::wakeUp()
{
    int error;
    m_twoWire.beginTransmission(m_commAddress);
    error = m_twoWire.endTransmission(true);
    return ((error == 0) || (error == 2));
}

SunriseError SunriseI2CTransport::requestFromRegisters(uint8_t regAddrStart, uint8_t howMany)
{
    SunriseError error;
    if (wakeUp())
    {
        m_twoWire.beginTransmission(m_commAddress);
        m_twoWire.write(regAddrStart);
        error = mapEndTransmission(m_twoWire.endTransmission(false));
        if (error == SunriseError::NO_ERROR)
        {
            if (howMany != m_twoWire.requestFrom(m_commAddress, howMany))
            {
                error = SunriseError::COMMUNICATION_ERROR;
            }
        }
    }
    else
    {
        error = SunriseError::COMMUNICATION_ERROR;
    }

    return error;
}

SunriseError SunriseI2CTransport::beginWrite(uint8_t regAddrStart)
{
    if (!wakeUp())
    {
        return SunriseError::COMMUNICATION_ERROR;
    }
    m_twoWire.beginTransmission(m_commAddress);
    m_twoWire.write(regAddrStart);
    return SunriseError::NO_ERROR;
}

SunriseError SunriseI2CTransport::endWrite()
{
    return mapEndTransmission(m_twoWire.endTransmission());
}

/* ============================================================================
 * S12I2CTransport
 * ============================================================================ */

S12I2CTransport::S12I2CTransport(TwoWire& twoWire, uint8_t commAddress) : SunriseI2CTransport(twoWire, commAddress)
{
}

bool S12I2CTransport::wakeUp()
{
    return true;
}

/* ============================================================================
 * SunriseBase
 * ============================================================================ */

/* Invariant: this constructor only stores the address of `transport`; it must not dereference it.
 * The concrete sensor classes (e.g. SunriseModbus) pass their own transport member to this base
 * constructor, which — per C++ member-init order — runs before that member is constructed. Storing
 * the address is well-defined; using the transport here would be undefined behaviour. */
SunriseBase::SunriseBase(SunriseTransport& transport, int pinEnable, int pinNRDY)
    : m_transport(&transport), m_useNRDY(true), m_useEnable(true), m_pinEnable(pinEnable), m_pinNRDY(pinNRDY)
{
}

SunriseBase::SunriseBase(SunriseTransport& transport, int pinEnable)
    : m_transport(&transport), m_useNRDY(false), m_useEnable(true), m_pinEnable(pinEnable)
{
}

SunriseBase::SunriseBase(SunriseTransport& transport) : m_transport(&transport), m_useNRDY(false), m_useEnable(false)
{
}

void SunriseBase::setCommAddress(uint8_t address)
{
    m_transport->setAddress(address);
}

void SunriseBase::begin()
{
    if (m_useEnable)
    {
        pinMode(m_pinEnable, OUTPUT);
    }
    if (m_useNRDY)
    {
        pinMode(m_pinNRDY, INPUT_PULLUP);
    }
}

void SunriseBase::enable()
{
    if (m_useEnable)
    {
        digitalWrite(m_pinEnable, HIGH);
    }
    delay(SUNRISE_STABILIZATION_MS);
}

void SunriseBase::disable()
{
    if (m_useEnable)
    {
        while (!isEepromSettled())
        {
        }
        digitalWrite(m_pinEnable, LOW);
    }
}

bool SunriseBase::isEepromSettled() const
{
    unsigned long lastWrite = m_transport->lastEepromWriteTime();
    if (lastWrite == 0)
        return true;
    return (millis() - lastWrite) >= eepromUpdateTimeMs();
}

uint16_t SunriseBase::eepromUpdateTimeMs() const
{
    return SUNRISE_EEPROM_UPDATE_TIME_MS;
}

String SunriseBase::evaluateError(SunriseError error, const char* currentAction)
{
    String errorMessage;
    switch (error)
    {
    case SunriseError::ILLEGAL_FUNCTION:
        errorMessage = "Illegal function code";
        break;
    case SunriseError::ILLEGAL_DATA_ADDRESS:
        errorMessage = "Illegal data address";
        break;
    case SunriseError::ILLEGAL_DATA_VALUE:
        errorMessage = "Illegal data value";
        break;
    case SunriseError::DATA_TOO_LONG:
        errorMessage = "Too much data";
        break;
    case SunriseError::NACK_ON_DATA:
        errorMessage = "NACK on data";
        break;
    case SunriseError::NACK_ON_ADDRESS:
        errorMessage = "NACK on address";
        break;
    case SunriseError::PACKET_TOO_SMALL:
        errorMessage = "Packet too small";
        break;
    case SunriseError::TIMEOUT_ERROR:
        errorMessage = "Response timed out";
        break;
    case SunriseError::COMMUNICATION_ERROR:
        errorMessage = "Communication error";
        break;
    default:
        errorMessage = "Error code is: " + String(static_cast<int>(error));
        break;
    }
    return String("Failed to ") + currentAction + ": " + errorMessage;
}

uint16_t SunriseBase::getABCTimeFromState(const SunriseSingleModeState& sunriseState)
{
    return sunriseState.state[1u];
}

void SunriseBase::incrementABCTimeInState(SunriseSingleModeState& sunriseState)
{
    sunriseState.state[1u] += 1u;
}

SunriseError SunriseBase::setSensorExtendedConfig(const SunriseExtendedConfiguration config)
{
    uint8_t      oldMeterControl;
    SunriseError error = getMeterControl(oldMeterControl);
    if (error == SunriseError::NO_ERROR)
    {
        uint8_t newMeterControl =
            oldMeterControl & ((uint8_t)~(METER_CONTROL_ABC | METER_CONTROL_PRESSURE | METER_CONTROL_FILTERS |
                                          METER_CONTROL_NRDY | METER_CONTROL_NRDY_INVERTED));

        if (!config.isABCEnabled)
        {
            newMeterControl |= METER_CONTROL_ABC;
        }
        if (!config.isPressureCompensationEnabled)
        {
            newMeterControl |= METER_CONTROL_PRESSURE;
        }
        if (!config.isIIRFilterEnabled)
        {
            newMeterControl |= METER_CONTROL_FILTERS;
        }
        if (!config.isNRDYPinEnabled)
        {
            newMeterControl |= METER_CONTROL_NRDY;
        }
        if (!config.isNRDYPinInverted)
        {
            newMeterControl |= METER_CONTROL_NRDY_INVERTED;
        }

        if (oldMeterControl != newMeterControl)
        {
            error = setMeterControl(newMeterControl);
        }
    }
    return error;
}

SunriseError SunriseBase::getSensorExtendedConfig(SunriseExtendedConfiguration& config)
{
    uint8_t      meterControl;
    SunriseError error = getMeterControl(meterControl);
    if (error == SunriseError::NO_ERROR)
    {
        config.isABCEnabled                  = ((meterControl & METER_CONTROL_ABC) == 0u);
        config.isPressureCompensationEnabled = ((meterControl & METER_CONTROL_PRESSURE) == 0u);
        config.isIIRFilterEnabled            = ((meterControl & METER_CONTROL_FILTERS) == 0u);
        config.isNRDYPinEnabled              = ((meterControl & METER_CONTROL_NRDY) == 0u);
        config.isNRDYPinInverted             = ((meterControl & METER_CONTROL_NRDY_INVERTED) == 0u);
    }
    return error;
}

SunriseError SunriseBase::enableABC()
{
    return clearMeterControlBits(METER_CONTROL_ABC);
}
SunriseError SunriseBase::disableABC()
{
    return setMeterControlBits(METER_CONTROL_ABC);
}
SunriseError SunriseBase::enableIIRFilters()
{
    return clearMeterControlBits(METER_CONTROL_FILTERS);
}
SunriseError SunriseBase::disableIIRFilters()
{
    return setMeterControlBits(METER_CONTROL_FILTERS);
}
SunriseError SunriseBase::enableNRDY()
{
    return clearMeterControlBits(METER_CONTROL_NRDY);
}
SunriseError SunriseBase::disableNRDY()
{
    return setMeterControlBits(METER_CONTROL_NRDY);
}
SunriseError SunriseBase::enableNRDYInverted()
{
    return clearMeterControlBits(METER_CONTROL_NRDY_INVERTED);
}
SunriseError SunriseBase::disableNRDYInverted()
{
    return setMeterControlBits(METER_CONTROL_NRDY_INVERTED);
}
SunriseError SunriseBase::enablePressureComp()
{
    return clearMeterControlBits(METER_CONTROL_PRESSURE);
}
SunriseError SunriseBase::disablePressureComp()
{
    return setMeterControlBits(METER_CONTROL_PRESSURE);
}

bool SunriseBase::waitForReady(uint16_t measurementSamples)
{
    bool          res         = true;
    unsigned long measTimeout = static_cast<unsigned long>(measurementSamples) * SINGLE_MEASUREMENT_TIMEOUT_MS;
    if (m_useNRDY)
    {
        unsigned long nrdyTimeStamp = millis();
        delay(POLL_NRDY_DELAY_MS);
        bool nrdyInitialState = digitalRead(m_pinNRDY);
        do
        {
            res = (digitalRead(m_pinNRDY) != nrdyInitialState);
        } while ((res == false) && ((millis() - nrdyTimeStamp) < measTimeout));
    }
    else
    {
        delay(measTimeout);
    }
    return res;
}

SunriseError SunriseBase::setSensorConfig(const SunriseConfiguration config)
{
    const RegDef regs[]   = {REG_MEAS_MODE, REG_MEAS_PERIOD, REG_MEAS_SAMPLES};
    uint16_t     values[] = {static_cast<uint16_t>(config.measMode), config.measPeriod, config.measSamples};
    if (!isValidConfig(config))
    {
        return SunriseError::ILLEGAL_DATA_VALUE;
    }
    return m_transport->writeMulti(regs, 3u, values);
}

SunriseError SunriseBase::getSensorConfig(SunriseConfiguration& config)
{
    const RegDef regs[] = {REG_MEAS_MODE, REG_MEAS_PERIOD, REG_MEAS_SAMPLES};
    uint16_t     values[3];
    SunriseError error = m_transport->readMulti(regs, 3u, values);
    if (error == SunriseError::NO_ERROR)
    {
        config.measMode    = (values[0] == 0) ? MeasurementMode::CONTINUOUS : MeasurementMode::SINGLE;
        config.measPeriod  = values[1];
        config.measSamples = values[2];
    }
    return error;
}

SunriseError SunriseBase::setMeasurementMode(MeasurementMode mode)
{
    return m_transport->writeU8(REG_MEAS_MODE, static_cast<uint8_t>(mode));
}

SunriseError SunriseBase::getABCPeriod(uint16_t& period)
{
    return m_transport->readU16(REG_ABC_PERIOD, period);
}

SunriseError SunriseBase::setABCPeriod(uint16_t period)
{
    return m_transport->writeU16(REG_ABC_PERIOD, period);
}

SunriseError SunriseBase::sendCalibrationCommand(CalCommand cmd)
{
    const RegDef regs[]   = {REG_CAL_STATUS, REG_CAL_COMMAND};
    uint16_t     values[] = {0u, static_cast<uint16_t>(cmd)};
    return m_transport->writeMulti(regs, 2u, values);
}

SunriseError SunriseBase::getCalibrationStatus(uint8_t& calStatus)
{
    return m_transport->readU8(REG_CAL_STATUS, calStatus);
}

SunriseError SunriseBase::setCalibrationTarget(int16_t target)
{
    return m_transport->writeU16(REG_CAL_TARGET, static_cast<uint16_t>(target));
}

SunriseError SunriseBase::setPressureValue(int16_t press)
{
    return m_transport->writeU16(REG_BARO_PRESS, static_cast<uint16_t>(press));
}

SunriseError SunriseBase::setSensorAddress(uint8_t address)
{
    return m_transport->writeU8(REG_SENSOR_ADDR, address);
}

SunriseError SunriseBase::getMeasurementCount(uint8_t& measCount)
{
    return m_transport->readU8(REG_MEAS_COUNT, measCount);
}

SunriseError SunriseBase::getFirmwareType(uint8_t& type)
{
    return m_transport->readU8(REG_FW_TYPE, type);
}

SunriseError SunriseBase::getFirmwareRevision(SunriseFirmwareRevision& revision)
{
    uint16_t     val;
    SunriseError error = m_transport->readU16(REG_FW_REV, val);
    if (error == SunriseError::NO_ERROR)
    {
        revision.revisionMain = u16Hi(val);
        revision.revisionSub  = u16Lo(val);
    }
    return error;
}

SunriseError SunriseBase::getSensorSerialNumber(uint32_t& serialNumber)
{
    uint16_t     values[2];
    SunriseError error = m_transport->readRegs(REG_SENSOR_ID, 2u, values);
    if (error == SunriseError::NO_ERROR)
    {
        serialNumber = (static_cast<uint32_t>(values[0]) << 16u) | values[1];
    }
    return error;
}

SunriseError SunriseBase::resetBySCR()
{
    while (!isEepromSettled())
    {
    }
    return m_transport->writeU8(REG_SCR, 0xFFu);
}

SunriseError SunriseBase::setMeterControl(uint8_t value)
{
    return m_transport->writeU8(REG_METER_CONTROL, value);
}

SunriseError SunriseBase::getMeterControl(uint8_t& value)
{
    return m_transport->readU8(REG_METER_CONTROL, value);
}

SunriseError SunriseBase::initialSingleMeasurement(SunriseMeasurement& meas, SunriseSingleModeState& sunriseState,
                                                   uint16_t measurementSamples)
{
    SunriseError error = m_transport->writeU16(REG_START_MEAS, 1u);
    if (error == SunriseError::NO_ERROR)
    {
        waitForReady(measurementSamples);
        error = readMeasurement(meas);
        if (error == SunriseError::NO_ERROR)
        {
            error = readSensorState(sunriseState);
        }
    }
    return error;
}

SunriseError SunriseBase::readSingleMeasurement(SunriseMeasurement& meas, SunriseSingleModeState& sunriseState,
                                                uint16_t measurementSamples)
{
    sunriseState.state[0] = 1u;
    SunriseError error    = m_transport->writeRegs(REG_SENSOR_STATE, sunriseState.SENSOR_STATE_SZ, sunriseState.state);
    if (error == SunriseError::NO_ERROR)
    {
        waitForReady(measurementSamples);
        error = readMeasurement(meas);
        if (error == SunriseError::NO_ERROR)
        {
            error = readSensorState(sunriseState);
        }
    }
    return error;
}

SunriseError SunriseBase::readMeasurement(SunriseMeasurement& meas)
{
    uint16_t     values[5];
    SunriseError error = m_transport->readRegs(REG_ERROR_STATUS, 5u, values);
    if (error == SunriseError::NO_ERROR)
    {
        meas.errorStatus = values[0];
        meas.co2value    = static_cast<int16_t>(values[3]);
        meas.temperature = static_cast<int16_t>(values[4]);
    }
    return error;
}

SunriseError SunriseBase::setMeterControlBits(uint8_t mask)
{
    uint8_t      meterControl;
    SunriseError error = getMeterControl(meterControl);
    if ((error == SunriseError::NO_ERROR) && ((meterControl & mask) != mask))
    {
        error = setMeterControl(meterControl | mask);
    }
    return error;
}

SunriseError SunriseBase::clearMeterControlBits(uint8_t mask)
{
    uint8_t      meterControl;
    SunriseError error = getMeterControl(meterControl);
    if ((error == SunriseError::NO_ERROR) && ((meterControl & mask) != 0u))
    {
        error = setMeterControl(meterControl & (uint8_t)~mask);
    }
    return error;
}

SunriseError SunriseBase::readSensorState(SunriseSingleModeState& sunriseState)
{
    return m_transport->readRegs(REG_SENSOR_STATE, sunriseState.SENSOR_STATE_SZ, sunriseState.state);
}

/* ============================================================================
 * Sunrise (Sunrise/Sunlight specifics: scale factors)
 * ============================================================================ */

bool Sunrise::isValidConfig(const SunriseConfiguration& config)
{
    return isValidSunriseConfig(config);
}

SunriseError Sunrise::getScaleFactor(SunriseScaleFactor& factor)
{
    uint16_t     values[2];
    SunriseError error = m_transport->readRegs(REG_SCALE_NUM, 2u, values);
    if (error == SunriseError::NO_ERROR)
    {
        factor.numerator   = values[0];
        factor.denominator = values[1];
    }
    return error;
}

SunriseError Sunrise::setScaleFactor(const SunriseScaleFactor factor)
{
    uint16_t values[] = {factor.numerator, factor.denominator};
    return m_transport->writeRegs(REG_SCALE_NUM, 2u, values);
}

SunriseError Sunrise::getScaledMeasurement(int16_t& scaledConcentration)
{
    uint16_t     val;
    SunriseError error = m_transport->readU16(REG_SCALED_CONC, val);
    if (error == SunriseError::NO_ERROR)
    {
        scaledConcentration = static_cast<int16_t>(val);
    }
    return error;
}

/* ============================================================================
 * S12 (S12 specifics: timing, RDYN)
 * ============================================================================ */

bool S12::isValidConfig(const SunriseConfiguration& config)
{
    return isValidS12Config(config);
}

void S12::enable()
{
    if (m_useEnable)
    {
        m_enableTimestamp = millis();
    }
    SunriseBase::enable();
}

SunriseError S12::initialSingleMeasurement(SunriseMeasurement& meas, SunriseSingleModeState& sunriseState,
                                           uint16_t measurementSamples)
{
    if (m_useEnable)
    {
        unsigned long elapsed = millis() - m_enableTimestamp;
        if (elapsed < S12_SINGLE_START_READY_MS)
        {
            delay(S12_SINGLE_START_READY_MS - elapsed);
        }
    }
    return SunriseBase::initialSingleMeasurement(meas, sunriseState, measurementSamples);
}

SunriseError S12::readSingleMeasurement(SunriseMeasurement& meas, SunriseSingleModeState& sunriseState,
                                        uint16_t measurementSamples)
{
    if (m_useEnable)
    {
        unsigned long elapsed = millis() - m_enableTimestamp;
        if (elapsed < S12_SINGLE_START_READY_MS)
        {
            delay(S12_SINGLE_START_READY_MS - elapsed);
        }
    }
    return SunriseBase::readSingleMeasurement(meas, sunriseState, measurementSamples);
}

SunriseError S12::enableRDYNOpenDrain()
{
    return setMeterControlBits(METER_CONTROL_RDYN_OPEN_DRAIN);
}

SunriseError S12::disableRDYNOpenDrain()
{
    return clearMeterControlBits(METER_CONTROL_RDYN_OPEN_DRAIN);
}

uint16_t S12::eepromUpdateTimeMs() const
{
    return S12_EEPROM_UPDATE_TIME_MS;
}

/* ============================================================================
 * SunriseModbus
 * ============================================================================ */

SunriseModbus::SunriseModbus(Stream* serial, uint8_t commAddress, int pinEnable, int pinNRDY)
    : Sunrise(m_modbusTransport, pinEnable, pinNRDY), m_modbusTransport(serial, commAddress)
{
}

SunriseModbus::SunriseModbus(Stream* serial, uint8_t commAddress, int pinEnable)
    : Sunrise(m_modbusTransport, pinEnable), m_modbusTransport(serial, commAddress)
{
}

SunriseModbus::SunriseModbus(Stream* serial, uint8_t commAddress)
    : Sunrise(m_modbusTransport), m_modbusTransport(serial, commAddress)
{
}

SunriseError SunriseModbus::getSensorID(SunriseIdentification& ident)
{
    return m_modbusTransport.readIdentification(ident);
}

/* ============================================================================
 * SunriseI2C
 * ============================================================================ */

SunriseI2C::SunriseI2C(TwoWire& twoWire, uint8_t commAddress, int pinEnable, int pinNRDY)
    : Sunrise(m_i2cTransport, pinEnable, pinNRDY), m_i2cTransport(twoWire, commAddress)
{
}

SunriseI2C::SunriseI2C(TwoWire& twoWire, uint8_t commAddress, int pinEnable)
    : Sunrise(m_i2cTransport, pinEnable), m_i2cTransport(twoWire, commAddress)
{
}

SunriseI2C::SunriseI2C(TwoWire& twoWire, uint8_t commAddress)
    : Sunrise(m_i2cTransport), m_i2cTransport(twoWire, commAddress)
{
}

/* ============================================================================
 * S12Modbus
 * ============================================================================ */

S12Modbus::S12Modbus(Stream* serial, uint8_t commAddress, int pinEnable, int pinNRDY)
    : S12(m_modbusTransport, pinEnable, pinNRDY), m_modbusTransport(serial, commAddress)
{
}

S12Modbus::S12Modbus(Stream* serial, uint8_t commAddress, int pinEnable)
    : S12(m_modbusTransport, pinEnable), m_modbusTransport(serial, commAddress)
{
}

S12Modbus::S12Modbus(Stream* serial, uint8_t commAddress)
    : S12(m_modbusTransport), m_modbusTransport(serial, commAddress)
{
}

SunriseError S12Modbus::getSensorID(SunriseIdentification& ident)
{
    return m_modbusTransport.readIdentification(ident);
}

/* ============================================================================
 * S12I2C
 * ============================================================================ */

S12I2C::S12I2C(TwoWire& twoWire, uint8_t commAddress, int pinEnable, int pinNRDY)
    : S12(m_i2cTransport, pinEnable, pinNRDY), m_i2cTransport(twoWire, commAddress)
{
}

S12I2C::S12I2C(TwoWire& twoWire, uint8_t commAddress, int pinEnable)
    : S12(m_i2cTransport, pinEnable), m_i2cTransport(twoWire, commAddress)
{
}

S12I2C::S12I2C(TwoWire& twoWire, uint8_t commAddress) : S12(m_i2cTransport), m_i2cTransport(twoWire, commAddress)
{
}
