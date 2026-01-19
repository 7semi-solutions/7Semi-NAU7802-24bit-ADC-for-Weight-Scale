/**
 * 7Semi NAU7802 Load Cell ADC Library (Header)
 *
 * - This header defines NAU7802 register map, configuration enums, and the NAU7802_7Semi class API.
 * - The library is intended for Arduino-compatible platforms using the Wire (I2C) interface.
 *
 * User notes (to avoid confusion):
 * - begin() must be called before any reads.
 * - tare() stores the current no-load offset into tare_offset.
 * - calibrateScale() computes scale_factor using a known weight (same unit you will later read).
 * - getWeight() returns false if calibration is not done (scale_factor == 0) or if read timeout occurs.
 * - For ESP32/ESP8266 you may pass custom SDA/SCL pins. For other MCUs, pins are ignored.
 */

#ifndef _7SEMI_NAU7802_H_
#define _7SEMI_NAU7802_H_

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

/**
 * NAU7802 I2C Register Map
 *
 * - These are the core registers used by the library.
 * - Register meanings come from the NAU7802 datasheet.
 *
 * Note:
 * - REG_ADC_CTRL1 and REG_OPT_DATA1 share address 0x15 in the datasheet naming.
 *   This library keeps both defines for readability depending on the use-case.
 */
#define REG_PU_CTRL   0x00
#define REG_CTRL1     0x01
#define REG_CTRL2     0x02

#define REG_OCAL1_B2  0x03
#define REG_OCAL1_B1  0x04
#define REG_OCAL1_B0  0x05
#define REG_GCAL1_B3  0x06
#define REG_GCAL1_B2  0x07
#define REG_GCAL1_B1  0x08
#define REG_GCAL1_B0  0x09

#define REG_OCAL2_B2  0x0A
#define REG_OCAL2_B1  0x0B
#define REG_OCAL2_B0  0x0C
#define REG_GCAL2_B3  0x0D
#define REG_GCAL2_B2  0x0E
#define REG_GCAL2_B1  0x0F
#define REG_GCAL2_B0  0x10

#define REG_I2C_CTRL  0x11

/** - ADC output registers (24-bit value: B2 = MSB, B0 = LSB) */
#define REG_ADCO_B2   0x12
#define REG_ADCO_B1   0x13
#define REG_ADCO_B0   0x14

#define REG_ADC_CTRL1 0x15

/** - Optional naming aliases (same addresses as per datasheet) */
#define REG_OPT_DATA1 0x15
#define REG_OPT_DATA2 0x16
#define REG_OPT_DATA3 0x17

#define REG_ADC_CTRL3 0x1B
#define REG_PWR_CTRL  0x1C

/** - Chip revision ID */
#define REG_REV_ID    0x1F

/**
 * LDO Output Voltage Options
 *
 * - These values map to CTRL1 bits 3..5 (LDO selection).
 * - Use setConfig() to apply LDO and PGA together safely.
 */
enum LDO_Output_Voltage
{
    LDO_2_4V = 0b111,
    LDO_2_7V = 0b110,
    LDO_3_0V = 0b101,
    LDO_3_3V = 0b100,
    LDO_3_6V = 0b011,
    LDO_3_9V = 0b010,
    LDO_4_2V = 0b001,
    LDO_4_5V = 0b000
};

/**
 * PGA Gain Options
 *
 * - These values map to CTRL1 bits 0..2 (gain selection).
 * - For load cells, PGA_128 is commonly used (depends on bridge output and wiring).
 */
enum PGA_gain_select
{
    PGA_1   = 0b000,
    PGA_2   = 0b001,
    PGA_4   = 0b010,
    PGA_8   = 0b011,
    PGA_16  = 0b100,
    PGA_32  = 0b101,
    PGA_64  = 0b110,
    PGA_128 = 0b111
};

/**
 * Conversion Rate Options (samples per second)
 *
 * - These values map to CTRL2 bits 4..6.
 * - Higher SPS gives faster updates but may increase noise.
 */
enum Conversion_rate_select
{
    CRS_10  = 0b000,
    CRS_20  = 0b001,
    CRS_40  = 0b010,
    CRS_80  = 0b011,
    CRS_320 = 0b111
};

/**
 * NAU7802_7Semi Class
 *
 * - High level API for NAU7802 initialization, configuration, and reading data.
 * - Designed for simple load cell workflows: begin -> tare -> calibrate -> getWeight.
 */
class NAU7802_7Semi
{
public:
    /**
     * Constructor
     *
     * - wirePort defaults to Wire
     * - address defaults to 0x2A inside implementation
     */
    NAU7802_7Semi(TwoWire &wirePort = Wire);

    /**
     * Initialize sensor
     *
     * Inputs:
     * - sdaPin        : ESP only (0xFF uses default pins)
     * - sclPin        : ESP only (0xFF uses default pins)
     * - i2cClockSpeed : I2C speed in Hz (default 400k)
     *
     * Return:
     * - true  : init OK
     * - false : NAU7802 not detected or init/config failed
     */
    bool begin(uint8_t sdaPin = 0xFF, uint8_t sclPin = 0xFF, uint32_t i2cClockSpeed = 400000);

    /**
     * Power control
     *
     * - power(true) powers up analog+digital and waits for ready
     * - power(false) powers down
     */
    bool power(bool powerUP);

    /**
     * Start ADC conversions
     *
     * - Must be called after power-up and configuration
     * - After this, available() should begin toggling
     */
    bool startReading();

    /**
     * Tare (Zero)
     *
     * Inputs:
     * - tareValue : output tare offset (raw ADC counts)
     * - samples   : number of readings averaged
     * - timeout   : max time budget in ms
     *
     * Return:
     * - true  : tare stored successfully
     * - false : read failed/timeout
     */
    bool tare(int32_t &tareValue, uint8_t samples = 8, unsigned long timeout = 1000);

    /**
     * Set/Get tare offset manually
     *
     * - Useful for saving/restoring tare from EEPROM/flash
     */
    void setTareOffset(int32_t offset);
    int32_t getTareOffset();

    /**
     * Calibrate scale factor with a known weight
     *
     * Inputs:
     * - knownWeight : known reference weight (your chosen unit)
     * - samples     : number of readings averaged
     * - timeout     : max time budget in ms
     *
     * Return:
     * - true  : scale_factor computed
     * - false : invalid knownWeight or read failed
     */
    bool calibrateScale(float knownWeight, uint8_t samples = 8, unsigned long timeout = 1000);

    /**
     * Set/Get scale factor manually
     *
     * - Useful for saving/restoring calibration from EEPROM/flash
     */
    void setScaleFactor(float factor);
    float getScaleFactor();

    /**
     * Get weight in units
     *
     * Inputs:
     * - weight        : output weight in same unit as used during calibration
     * - allowNegative : true allows negative readings (useful for direction tests)
     * - samples       : number of samples averaged
     * - timeout       : max time budget in ms
     *
     * Return:
     * - true  : weight updated
     * - false : not calibrated or read failed/timeout
     */
    bool getWeight(float &weight);

    /**
     * Configure LDO output voltage and PGA gain in a single call
     *
     * - Updates CTRL1 register:
     *   - CTRL1[5:3] : LDO voltage select
     *   - CTRL1[2:0] : PGA gain select
     *
     * LDO options:
     * - LDO_2_4V : 2.4V
     * - LDO_2_7V : 2.7V
     * - LDO_3_0V : 3.0V
     * - LDO_3_3V : 3.3V (recommended for most modules)
     * - LDO_3_6V : 3.6V
     * - LDO_3_9V : 3.9V
     * - LDO_4_2V : 4.2V
     * - LDO_4_5V : 4.5V
     *
     * Gain options:
     * - PGA_1   : x1
     * - PGA_2   : x2
     * - PGA_4   : x4
     * - PGA_8   : x8
     * - PGA_16  : x16
     * - PGA_32  : x32
     * - PGA_64  : x64
     * - PGA_128 : x128 (most common for load cells)
     *
     * Common use:
     * - Most NAU7802 modules + load cell:
     *   - setConfig(LDO_3_3V, PGA_128)
     *
     * - Return:
     *   - true  : register write success
     *   - false : I2C error / sensor not responding
     */
    bool setConfig(LDO_Output_Voltage ldo, PGA_gain_select gain);

    /**
     * Change only PGA gain 
     *
     * - Updates CTRL1 register:
     *   - CTRL1[2:0] : PGA gain select
     *
     * When to use:
     * - You already set LDO using setConfig()
     * - You want to change only gain without touching the LDO value
     *
     * Gain options:
     * - PGA_1   : x1
     * - PGA_2   : x2
     * - PGA_4   : x4
     * - PGA_8   : x8
     * - PGA_16  : x16
     * - PGA_32  : x32
     * - PGA_64  : x64
     * - PGA_128 : x128
     *
     * Notes:
     * - If readings clip/saturate (stuck near max/min), reduce gain:
     *   - try PGA_64 or PGA_32
     *
     * - Return:
     *   - true  : register write success
     *   - false : I2C error / sensor not responding
     */
    bool setGain(uint8_t gain);

    /**
     * Set NAU7802 conversion rate (samples per second)
     *
     * - Updates CTRL2 register:
     *   - CTRL2[6:4] : conversion rate select
     *
     * Rate options:
     * - CRS_10  : 10 SPS  (best stability / least noise)
     * - CRS_20  : 20 SPS
     * - CRS_40  : 40 SPS
     * - CRS_80  : 80 SPS  (good balance for many scales)
     * - CRS_320 : 320 SPS (fastest, more noise)
     *
     * Notes:
     * - Lower rate gives smoother readings for weighing scales
     * - Higher rate gives faster updates but increases noise
     *
     * - Return:
     *   - true  : register write success
     *   - false : I2C error / sensor not responding
     */
    bool setSampleRate(Conversion_rate_select rate);



    /**
     * Data reading helpers
     *
     * - available() indicates when a fresh conversion is ready
     * - readRaw() returns a single signed 24-bit sample (sign-extended)
     * - readAverage() returns an averaged raw value over N samples
     */
    bool available();
    bool readRaw(int32_t &rawValue, unsigned long timeout = 100);
    bool readAverage(int32_t &avgValue, uint8_t samples = 8, unsigned long timeout = 1000);

    /**
     * Store default sampling parameters
     *
     * - These are convenience defaults for your application (optional)
     * - You can still pass explicit samples/timeout to getWeight/readAverage
     */
    void setSampling(uint8_t averageSamples, unsigned long readTimeout);

    /**
     * Gain readback helpers
     *
     * - getPGA() returns enum value (0..7)
     * - getGain() returns numeric gain (1..128)
     */
    PGA_gain_select getPGA();
    uint16_t getGain();

private:
    /** - I2C handle */
    TwoWire *i2c;

    /** - NAU7802 I2C address (default 0x2A) */
    uint8_t address;

    /**
     * Calibration storage
     *
     * - tare_offset  : raw ADC counts measured at zero load
     * - scale_factor : counts per unit weight
     */
    int32_t tare_offset;
    float scale_factor;

    /**
     * Optional user defaults
     *
     * - samples : default averaging samples (min 1)
     * - timeout : default read timeout in ms (min 1)
     */
    uint8_t samples;
    unsigned long timeout;

    /**
     * Low-level I2C helpers
     *
     * - readReg()/writeReg() access NAU7802 registers
     * - setBit()/clearBit() are convenience bit operations (if used elsewhere)
     * - readADC24() reads 24-bit ADC output and sign-extends it
     */
    bool readReg(uint8_t reg, uint8_t &value);
    bool writeReg(uint8_t reg, uint8_t val);
    bool setBit(uint8_t reg, uint8_t bit);
    bool clearBit(uint8_t reg, uint8_t bit);

    int32_t readADC24();
};

#endif
