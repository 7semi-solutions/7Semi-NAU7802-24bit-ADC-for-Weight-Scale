/**
 * 7Semi NAU7802 24-bit Weigh Scale
 *
 * Connections (NAU7802 module -> MCU):
 * - VCC  -> 3.3V or 5V (depends on your module)
 * - GND  -> GND
 * - SDA  -> SDA pin of MCU
 * - SCL  -> SCL pin of MCU
 *
 * Load Cell connections (to NAU7802 load-cell connector, typical 4-wire):
 * - E+ (AVDD)      -> Red
 * - E- (GND)       -> Black
 * - A+ (Signal +)  -> Green
 * - A- (Signal -)  -> White
 *
 * Flow:
 * - Init NAU7802
 * - Configure Sensor
 * - Tare with empty platform
 * - Calibrate with known weight
 * - Print weight continuously
 *
 * Notes:
 * - Keep platform EMPTY when doing TARE.
 * - Place the exact known weight during CALIBRATION.
 * - CAL_WEIGHT value must match the weight you place (in grams or your chosen unit).
  * - Wire colors can vary by load cell vendor. Always confirm with your load cell datasheet.
 * - Use a stable power supply and good wiring. Noise and loose connections will affect readings.
 * - Keep module and load cell wires away from motors/relays/high-current wiring.
 */


#include <7Semi_NAU7802.h>

NAU7802_7Semi scale(Wire);

/** - Put your known calibration weight here (grams) */
static const float CAL_WEIGHT = 100.0f;

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("7Semi NAU7802 24bit Weigh Scale");
  Serial.println("--------------------------------");

  /** - Initialize the sensor */
  Serial.println("Initializing NAU7802...");
  if (!scale.begin()) {
    Serial.println("ERROR: NAU7802 not detected!");
    Serial.println("Check wiring, power, and I2C connections.");
    while (1) delay(1000);
  }

  /** - Configure LDO + Gain */
  if (!scale.setConfig(LDO_3_3V, PGA_128)) {
    Serial.println("WARNING: Failed to configure sensor (setConfig).");
  }

  /** - Optional sampling settings */
  scale.setSampling(20, 2000);

  /** - Tare */
  Serial.println("STEP 1/2: TARE (ZERO)\n"
                 "- Remove all weight from the scale.\n"
                 "- When platform is EMPTY, press Enter.");
  Serial.println("--------------------------------------------------");

  while (!Serial.available())
    ;
  Serial.readString();
  int32_t tareVal = 0;
  Serial.println("Taring... please keep it empty.");
  if (scale.tare(tareVal, 100, 10000)) {
    // Serial.print("Tare complete, offset: ");
    // Serial.println((long)tareVal);
  } else {
    Serial.println("ERROR: Tare failed!");
    Serial.println("- Check load cell wiring/noise");
    Serial.println("- Try increasing timeout or reducing vibration");
    Serial.println("- Reset to retry");
    while (1) delay(100);
  }

  /** - Calibration */
  Serial.println();
  Serial.println("STEP 2/2: CALIBRATION");
  Serial.print("- Place known weight on the scale: ");
  Serial.print(CAL_WEIGHT, 2);
  Serial.println(" g");
  Serial.println();
  Serial.println("When the weight is placed and stable, press Enter to calibrate.");
  Serial.println("--------------------------------------------------");

  while (!Serial.available())
    ;

  Serial.println("Calibrating... please do not touch the scale.");
  if (scale.calibrateScale(CAL_WEIGHT, 100, 10000)) {
    Serial.print("Calibration complete. Scale factor: ");
    Serial.println(scale.getScaleFactor(), 6);
  } else {
    Serial.println("ERROR: Calibration failed!");
    Serial.println("- Make sure the weight value in CAL_WEIGHT is correct");
    Serial.println("- Ensure the weight is stable and not moving");
    Serial.println("- Increase timeout if needed");
    while (1) delay(100);
  }

  Serial.println();
  Serial.println("Setup done! Reading weight every 1 second...");
}

void loop() {
  float weight = 0;

  /** - Read weight (uses library defaults: samples/timeout previously set by setSampling) */
  if (scale.getWeight(weight)) {
    Serial.print("Weight: ");
    Serial.print(weight, 2);
    Serial.println(" g");
  } else {
    Serial.println("WARN: Failed to read weight (timeout/not calibrated).");
  }

  delay(1000);
}
