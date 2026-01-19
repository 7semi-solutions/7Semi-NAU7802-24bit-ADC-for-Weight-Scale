/**
 * 7Semi NAU7802 Load Cell ADC Library
 *
 * - Driver for the NAU7802 24-bit ADC (commonly used with load cells)
 * - I2C interface using Arduino Wire (TwoWire)
 * - Supports: power-up/init, gain/LDO config, sample rate, raw reads, averaging, tare, and scale calibration
 *
 * Notes for users:
 * - Call begin() once in setup() to initialize the IC and start conversions.
 * - For stable weight readings:
 *   - Call tare() with the platform unloaded (no weight).
 *   - Call calibrateScale() with a known weight to compute scale_factor.
 * - getWeight() returns false if not calibrated (scale_factor == 0) or if read timeout occurs.
 */

#include "7Semi_NAU7802.h"

/**
 * Constructor
 *
 * - Stores the I2C bus reference
 * - Sets default device address and sane defaults for sampling/calibration
 */
NAU7802_7Semi::NAU7802_7Semi(TwoWire &wire)
{
    i2c = &wire;
    address = 0x2A;   // NAU7802 default I2C address
    samples = 10;     // default averaging samples (used if user chooses to use it)
    timeout = 500;    // default read timeout in ms
    tare_offset = 0;  // tare (zero) offset in raw ADC counts
    scale_factor = 1.0f; // scale factor (counts per unit weight). Calibrate to set correctly.
}

/**
 * Initialize sensor using I2C interface
 *
 * Inputs:
 * - sdaPin         : optional SDA pin for ESP platforms (0xFF = default pins)
 * - sclPin         : optional SCL pin for ESP platforms (0xFF = default pins)
 * - i2cClockSpeed  : I2C clock speed in Hz (typical 100000 or 400000)
 *
 * Return:
 * - true  : I2C started and NAU7802 configured successfully
 * - false : sensor not responding or configuration step failed
 *
 * User note:
 * - If this returns false, check wiring, address (0x2A), power, and pullups.
 */
bool NAU7802_7Semi::begin(uint8_t sdaPin, uint8_t sclPin, uint32_t i2cClockSpeed)
{
#if defined(ESP32) || defined(ESP8266)
    /** - ESP platforms may allow custom SDA/SCL pins */
    if (sdaPin != 0xFF && sclPin != 0xFF)
        i2c->begin(sdaPin, sclPin);
    else
        i2c->begin();
#else
    /** - Other platforms ignore custom pins and use default Wire pins */
    (void)sdaPin;
    (void)sclPin;
    i2c->begin();
#endif

    /** - Configure I2C clock speed */
    i2c->setClock(i2cClockSpeed);

    /** - Power up analog + digital blocks and wait for ready */
    if(!power(true)) return false;

    /** - Set LDO and gain (common setup: 3.3V internal LDO, max gain for load cell) */
    if(!setConfig(LDO_3_3V, PGA_128)) return false;

    /** - Set conversion rate */
    if(!setSampleRate(CRS_80)) return false;

    /**
     * Disable clock chopper
     *
     * - Some applications prefer disabling chopper for stability/noise behavior.
     * - We set bits 4 and 5 (0x30) in ADC_CTRL1.
     */
    uint8_t adcVal;
    if(!readReg(REG_ADC_CTRL1, adcVal)) return false;
    adcVal |= 0x30;
    if(!writeReg(REG_ADC_CTRL1, adcVal)) return false;

    /**
     * Enable PGA cap
     *
     * - Helps stability with PGA operation as per typical NAU7802 usage.
     */
    if(!writeReg(REG_PWR_CTRL, 0x80)) return false;

    /**
     * Clear LDO mode bit for low noise
     *
     * - Sets ADC_CTRL3 to 0x40 as per existing library behavior.
     */
    if(!writeReg(REG_ADC_CTRL3, 0x40)) return false;

    /** - Allow time for analog to settle */
    delay(250);

    /**
     * Start continuous conversions
     *
     * - After startReading(), data-ready will toggle and readRaw() can be used.
     */
    if(!startReading()) return false;

    /** - Dummy read to flush first conversion and ensure pipeline is running */
    int32_t tmp;
    (void)readRaw(tmp, 100);

    return true;
}

/**
 * Configure LDO output voltage and PGA gain
 *
 * Inputs:
 * - ldo  : LDO output selection enum (placed in CTRL1 bits 3..5)
 * - gain : PGA gain selection enum (placed in CTRL1 bits 0..2)
 *
 * Return:
 * - true  : register updated successfully
 * - false : I2C read/write failed
 */
bool NAU7802_7Semi::setConfig(LDO_Output_Voltage ldo, PGA_gain_select gain)
{
    uint8_t v;
    if(!readReg(REG_CTRL1, v)) return false;

    /** - Clear LDO bits (3..5) and gain bits (0..2) */
    v &= 0xC7;

    /** - Apply new LDO and gain */
    v |= ((ldo & 0x07) << 3) | (gain & 0x07);

    return writeReg(REG_CTRL1, v);
}

/**
 * Power up/down the NAU7802
 *
 * Inputs:
 * - powerUP : true = power up analog + digital, false = power down
 *
 * Return:
 * - true  : command succeeded and (if power up) ready bit asserted
 * - false : I2C failed or ready timeout
 */
bool NAU7802_7Semi::power(bool powerUP)
{
    if (powerUP)
    {
        /** - 0x06 powers up digital + analog */
        if(!writeReg(REG_PU_CTRL, 0x06)) return false;

        /** - Wait for power ready (bit 3) */
        unsigned long start = millis();
        uint8_t val;
        while (true)
        {
            if (!readReg(REG_PU_CTRL, val))
                return false;

            if (val & (1 << 3))
                break;

            if (millis() - start > 100)
                return false;

            delay(1);
        }
    }
    else
    {
        /** - Power down everything */
        if(!writeReg(REG_PU_CTRL, 0x00)) return false;
    }
    return true;
}

/**
 * Start conversions
 *
 * Return:
 * - true  : CS bit set successfully
 * - false : I2C read/write failed
 *
 * User note:
 * - After this, use available() to poll data-ready.
 */
bool NAU7802_7Semi::startReading()
{
    uint8_t val;
    if(!readReg(REG_PU_CTRL, val)) return false;

    /** - Set CS bit (bit 4) to start conversions */
    val |= (1 << 4);

    if(!writeReg(REG_PU_CTRL, val)) return false;
    return true;
}

/**
 * Check if new data is ready
 *
 * Return:
 * - true  : data ready bit is set
 * - false : not ready OR I2C read failed
 *
 * User note:
 * - If this keeps returning false, check that begin() succeeded and conversions started.
 */
bool NAU7802_7Semi::available()
{
    uint8_t val;
    if(!readReg(REG_PU_CTRL, val)) return false;

    /** - DRDY bit (bit 5) indicates a new conversion is ready */
    return (val & (1 << 5)) != 0;
}

/**
 * Read raw 24-bit ADC value with timeout
 *
 * Inputs:
 * - rawValue    : output raw signed ADC count
 * - maxTimeout  : maximum wait time in ms for data ready
 *
 * Return:
 * - true  : data read successfully
 * - false : timeout occurred (rawValue set to 0)
 */
bool NAU7802_7Semi::readRaw(int32_t &rawValue, unsigned long maxTimeout)
{
    unsigned long start = millis();

    while (true)
    {
        if (available())
        {
            rawValue = readADC24();
            return true;
        }

        if (millis() - start > maxTimeout)
        {
            rawValue = 0;
            return false;
        }

        /** - Small delay prevents hammering the I2C bus */
        delay(1);
    }
}

/**
 * Read average of multiple samples
 *
 * Inputs:
 * - avgValue    : output averaged raw ADC count
 * - numSamples  : number of samples to average
 * - maxTimeout  : total timeout budget in ms for all samples
 *
 * Return:
 * - true  : average computed successfully
 * - false : timeout / read failure
 *
 * User note:
 * - For best stability, choose 5..20 samples depending on speed needs.
 */
bool NAU7802_7Semi::readAverage(int32_t &avgValue, uint8_t numSamples, unsigned long maxTimeout)
{
    if(numSamples == 0) return false;

    int64_t sum = 0;
    uint8_t count = 0;
    unsigned long start = millis();

    while(count < numSamples)
    {
        int32_t raw;

        /** - Split timeout across samples so a single sample doesn't consume the entire budget */
        if(readRaw(raw, maxTimeout / numSamples))
        {
            sum += raw;
            count++;
        }
        else
        {
            avgValue = 0;
            return false;
        }

        if(millis() - start > maxTimeout)
        {
            avgValue = 0;
            return false;
        }
    }

    avgValue = (int32_t)(sum / count);
    return true;
}

/**
 * Tare (zero) the scale
 *
 * Inputs:
 * - tareValue : output tare offset value stored (raw counts)
 * - samples   : number of samples used for averaging tare
 * - timeout   : timeout budget in ms for tare operation
 *
 * Return:
 * - true  : tare stored successfully
 * - false : read failure/timeout
 *
 * User note:
 * - Make sure platform is unloaded when calling tare().
 */
bool NAU7802_7Semi::tare(int32_t &tareValue, uint8_t samples, unsigned long timeout)
{
    int32_t avg;
    if (!readAverage(avg, samples, timeout))
        return false;

    tare_offset = avg;
    tareValue = tare_offset;

    /** - Debug print to confirm tare lock value */
    Serial.print("Tare locked: ");
    Serial.println((long)tare_offset);

    return true;
}

/**
 * Manually set tare offset
 *
 * - Useful if you want to store tare in EEPROM/flash and restore on boot
 */
void NAU7802_7Semi::setTareOffset(int32_t offset)
{
    tare_offset = offset;
}

/**
 * Get current tare offset (raw counts)
 */
int32_t NAU7802_7Semi::getTareOffset()
{
    return tare_offset;
}

/**
 * Calibrate scale factor using a known weight
 *
 * Inputs:
 * - knownWeight : known calibration weight in your chosen units (grams, kg, etc.)
 * - numSamples  : averaging samples for calibration reading
 * - maxTimeout  : timeout budget in ms for calibration reading
 *
 * Return:
 * - true  : scale_factor updated successfully
 * - false : invalid knownWeight or read failure
 *
 * User note:
 * - Call tare() first, then place knownWeight, then call calibrateScale().
 */
bool NAU7802_7Semi::calibrateScale(float knownWeight, uint8_t numSamples, unsigned long maxTimeout)
{
    if(knownWeight == 0) return false;

    int32_t avg;
    if(!readAverage(avg, numSamples, maxTimeout)) return false;

    /** - counts per unit weight */
    scale_factor = (float)(avg - tare_offset) / knownWeight;
    return true;
}

/**
 * Manually set scale factor
 *
 * - Useful if you store calibration factor and restore at boot
 */
void NAU7802_7Semi::setScaleFactor(float factor)
{
    scale_factor = factor;
}

/**
 * Get current scale factor (counts per unit weight)
 */
float NAU7802_7Semi::getScaleFactor()
{
    return scale_factor;
}

/**
 * Set default sampling parameters (optional helper)
 *
 * Inputs:
 * - average_samples : preferred averaging samples (minimum 1)
 * - readTimeout     : preferred timeout in ms (minimum 100)
 *
 * User note:
 * - This does not automatically change getWeight() behaviour unless you pass these values when calling.
 * - It is kept as a convenient place to store defaults for your application.
 */
void NAU7802_7Semi::setSampling(uint8_t average_samples, unsigned long readTimeout)
{
    if (average_samples < 1)
        samples = 1;
    else if (average_samples > 200)
        samples = 200;
    else
        samples = average_samples;

    if (readTimeout < 100)
        timeout = 100;
    else
        timeout = readTimeout;
}


/**
 * Get weight in units
 *
 * Inputs:
 * - weight        : output weight in the same units used during calibrateScale()
 *
 * Return:
 * - true  : weight computed successfully
 * - false : not calibrated / read failure
 *
 * User note:
 * - If you see always 0, make sure:
 *   - tare() completed
 *   - calibrateScale() completed
 *   - scale_factor is not 0
 */
bool NAU7802_7Semi::getWeight(float &weight)
{
    if(scale_factor == 0.0f)
    {
        weight = 0;
        return false;
    }

    int32_t avg;
    if(!readAverage(avg, samples, timeout))
    {
        weight = 0;
        return false;
    }

    /** - Remove tare offset */
    int32_t net = avg - tare_offset;

    /** - Optional clipping to avoid negative values when allowNegative is false */
    // if(!allowNegative && net < 0) net = 0;

    /** - Convert raw counts to weight units */
    weight = (float)net / scale_factor;
    return true;
}

/**
 * Set PGA gain (raw gain code 0..7)
 *
 * Inputs:
 * - gain : gain code (matches PGA_gain_select enum values)
 *
 * Return:
 * - true  : gain updated
 * - false : I2C read/write failed
 *
 * User note:
 * - Prefer using setConfig() if you want to set LDO + gain together.
 */
bool NAU7802_7Semi::setGain(uint8_t gain)
{
    uint8_t v;
    if(!readReg(REG_CTRL1, v)) return false;

    /** - Keep upper bits, replace gain bits 0..2 */
    v &= 0xF8;
    v |= (gain & 0x07);

    return writeReg(REG_CTRL1, v);
}

/**
 * Read current PGA gain setting (enum)
 *
 * Return:
 * - PGA_gain_select enum value, or PGA_1 as a safe default if read fails
 */
PGA_gain_select NAU7802_7Semi::getPGA()
{
    uint8_t v;
    if(!readReg(REG_CTRL1, v)) return PGA_1;

    return (PGA_gain_select)(v & 0x07);
}

/**
 * Get numeric gain value (1/2/4/8/16/32/64/128)
 *
 * Return:
 * - numeric gain corresponding to getPGA()
 */
uint16_t NAU7802_7Semi::getGain()
{
    switch(getPGA())
    {
        case PGA_1:   return 1;
        case PGA_2:   return 2;
        case PGA_4:   return 4;
        case PGA_8:   return 8;
        case PGA_16:  return 16;
        case PGA_32:  return 32;
        case PGA_64:  return 64;
        case PGA_128: return 128;
        default:      return 1;
    }
}

/**
 * Set conversion rate
 *
 * Inputs:
 * - rate : Conversion_rate_select enum (written into CTRL2 bits 4..6)
 *
 * Return:
 * - true  : rate updated successfully
 * - false : I2C read/write failed
 */
bool NAU7802_7Semi::setSampleRate(Conversion_rate_select rate)
{
    uint8_t v;
    if(!readReg(REG_CTRL2, v)) return false;

    /** - Keep top bit and low bits, clear rate field (bits 4..6) */
    v &= 0x8F;
    v |= (rate & 0x07) << 4;

    return writeReg(REG_CTRL2, v);
}

/**
 * Read a single register
 *
 * Inputs:
 * - reg   : register address
 * - value : output register value
 *
 * Return:
 * - true  : read ok
 * - false : I2C error
 */
bool NAU7802_7Semi::readReg(uint8_t reg, uint8_t &value)
{
    i2c->beginTransmission(address);
    i2c->write(reg);

    /** - Repeated start (endTransmission(false)) keeps bus active for read */
    if(i2c->endTransmission(false) != 0) return false;

    if(i2c->requestFrom(address, (uint8_t)1) != 1) return false;

    value = i2c->read();
    return true;
}

/**
 * Write a single register
 *
 * Inputs:
 * - reg : register address
 * - val : value to write
 *
 * Return:
 * - true  : write ok
 * - false : I2C error
 */
bool NAU7802_7Semi::writeReg(uint8_t reg, uint8_t val)
{
    i2c->beginTransmission(address);
    i2c->write(reg);
    i2c->write(val);
    return i2c->endTransmission() == 0;
}

/**
 * Set a single bit in a register
 *
 * Inputs:
 * - reg : register address
 * - bit : bit index (0..7)
 *
 * Return:
 * - true  : bit set successfully
 * - false : I2C read/write failed
 */
bool NAU7802_7Semi::setBit(uint8_t reg, uint8_t bit)
{
    uint8_t v;
    if(!readReg(reg, v)) return false;

    v |= (1 << bit);
    return writeReg(reg, v);
}

/**
 * Read 24-bit ADC conversion value (signed)
 *
 * Return:
 * - signed 32-bit value with sign-extended 24-bit data
 *
 * User note:
 * - This is a low-level read. Prefer readRaw() or readAverage() in normal usage.
 */
int32_t NAU7802_7Semi::readADC24()
{
    /** - Point to ADC output MSB register and read 3 bytes */
    i2c->beginTransmission(address);
    i2c->write(REG_ADCO_B2);
    i2c->endTransmission(false);

    if(i2c->requestFrom(address, (uint8_t)3) != 3) return 0;

    uint32_t value =
        ((uint32_t)i2c->read() << 16) |
        ((uint32_t)i2c->read() << 8)  |
        (uint32_t)i2c->read();

    /** - Sign extend 24-bit to 32-bit */
    if(value & 0x00800000) value |= 0xFF000000;

    return (int32_t)value;
}
