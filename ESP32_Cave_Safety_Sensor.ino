#include <Arduino.h>
#include <Wire.h>
#include <esp_task_wdt.h>

#include <SparkFun_SCD4x_Arduino_Library.h>
#include "DFRobot_OxygenSensor.h"
#include <Adafruit_BME680.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================
// CONFIGURATION - MODIFY FOR YOUR DEPLOYMENT
// ============================================

// Define which sensors are REQUIRED for operation (safety-critical)
// Set to 'true' ONLY if you have that sensor installed AND it's critical
#define REQUIRE_BME_SENSOR true
#define REQUIRE_SCD_SENSOR true
#define REQUIRE_O2_SENSOR false
#define REQUIRE_MQ5_SENSOR true
#define REQUIRE_H2S_SENSOR false
#define REQUIRE_CO_SENSOR false
#define REQUIRE_CH4_SENSOR false
#define REQUIRE_NH3_SENSOR false
#define REQUIRE_H2_SENSOR false

// Watchdog timeout (seconds) - system will reset if loop() hangs
#define WDT_TIMEOUT 10

// Enable detailed debug output
#define DEBUG_VERBOSE false

// ============================================
// OLED DISPLAY CONFIGURATION
// ============================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Yellow/Blue OLED zones
// IMPORTANT: Adjust these for your specific display!
// Common configurations:
//   - Top 16 pixels yellow, bottom 48 blue (older displays)
//   - Top 20 pixels yellow, bottom 44 blue (newer displays) ← YOUR DISPLAY
//   - Some displays have yellow at BOTTOM instead of top!
#define YELLOW_ZONE_HEIGHT 20
#define BLUE_ZONE_START 20

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Display pages
enum DisplayPage {
  PAGE_SUMMARY,
  PAGE_TOXIC_GASES,
  PAGE_EXPLOSIVE_GASES,
  PAGE_ENVIRONMENT,
  PAGE_STATUS,
  PAGE_COUNT
};

DisplayPage currentPage = PAGE_SUMMARY;
unsigned long lastPageChange = 0;
const unsigned long PAGE_DURATION_MS = 3000;  // 3 seconds per page

// ============================================
// PIN DEFINITIONS
// ============================================
const uint8_t PIN_LED_GREEN = 2;
const uint8_t PIN_LED_RED = 4;
const uint8_t PIN_BUZZER = 5;

// Analog sensor pins - Avoiding strapping pins and input-only pins
// Good ADC pins: 32, 33, 27, 14, 12, 13, 15
const uint8_t PIN_MQ5_AO = 32;   // LPG/Propane
const uint8_t PIN_H2S_AO = 33;   // Hydrogen Sulfide
const uint8_t PIN_CO_AO = 27;    // Carbon Monoxide
const uint8_t PIN_CH4_AO = 14;   // Methane
const uint8_t PIN_NH3_AO = 12;   // Ammonia
const uint8_t PIN_H2_AO = 13;    // Hydrogen

// I2C Pins (default for ESP32)
const uint8_t PIN_SDA = 21;
const uint8_t PIN_SCL = 22;

// ============================================
// ADC CONFIGURATION
// ============================================
const int ADC_RESOLUTION = 12;           // 12-bit ADC (0-4095)
const int ADC_MAX_VALUE = 4095;
const float ADC_VOLTAGE = 3.3;           // Reference voltage

// Floating pin detection thresholds
const int ADC_FLOAT_MIN = 100;           // Below this = likely shorted/error
const int ADC_FLOAT_MAX = 3900;          // Above this = likely floating
const int ADC_FLOAT_VARIANCE = 300;      // Max variance for stable reading

// Valid sensor reading range (real sensors typically in middle range)
// Most MEMS/electrochemical sensors in clean air: 1500-3500 range
const int ADC_SENSOR_MIN = 800;          // Minimum plausible sensor reading
const int ADC_SENSOR_MAX = 3800;         // Maximum plausible sensor reading

// ============================================
// SAFETY THRESHOLDS
// ============================================

// Oxygen (% by volume)
const float O2_WARN = 19.5f;      // OSHA warning level
const float O2_ALARM = 19.0f;     // OSHA alarm level (oxygen deficient)
const float O2_HIGH_WARN = 23.5f; // Oxygen enriched warning

// Carbon Dioxide (ppm)
const uint16_t CO2_WARN = 1500;   // ACGIH TLV-TWA: 5000, we use 1500 for early warning
const uint16_t CO2_ALARM = 5000;  // OSHA PEL: 5000 ppm

// MQ5 - LPG/Propane (inverse logic: voltage drops with gas)
// Will be calibrated at runtime
const int MQ5_WARN_DROP = 200;    // Raw ADC units below baseline
const int MQ5_ALARM_DROP = 400;   // Raw ADC units below baseline

// Hydrogen Sulfide (ppm)
const float H2S_WARN = 10.0f;     // OSHA ceiling: 20 ppm, we warn at 10
const float H2S_ALARM = 20.0f;    // OSHA ceiling limit

// Carbon Monoxide (ppm)
const float CO_WARN = 35.0f;      // OSHA TWA: 50 ppm, we warn at 35
const float CO_ALARM = 200.0f;    // OSHA ceiling: 200 ppm

// Methane (ppm)
const uint16_t CH4_WARN = 1000;   // 0.1% (LEL is 5%)
const uint16_t CH4_ALARM = 5000;  // 0.5% (10% of LEL)

// Ammonia (ppm)
const float NH3_WARN = 25.0f;     // OSHA TWA: 50 ppm, we warn at 25
const float NH3_ALARM = 50.0f;    // OSHA TWA limit

// Hydrogen (ppm)
const uint16_t H2_WARN = 1000;    // 0.1% (LEL is 4%)
const uint16_t H2_ALARM = 10000;  // 1.0% (25% of LEL)

// ============================================
// CALIBRATION STORAGE
// ============================================
// These will be populated during calibration
int MQ5_R0_RAW = 0;     // Baseline in clean air
int H2S_R0_RAW = 0;     // Baseline in clean air
int CO_R0_RAW = 0;      // Baseline in clean air
int CH4_R0_RAW = 0;     // Baseline in clean air
int NH3_R0_RAW = 0;     // Baseline in clean air
int H2_R0_RAW = 0;      // Baseline in clean air

// Scaling factors (ppm per ADC unit drop from baseline)
// Conservative estimates - will be refined during calibration
float H2S_SCALE = 0.020f;   // ~50 ppm over 2500 ADC units
float CO_SCALE = 2.0f;      // ~5000 ppm over 2500 ADC units
float CH4_SCALE = 4.0f;     // ~10000 ppm over 2500 ADC units
float NH3_SCALE = 0.12f;    // ~300 ppm over 2500 ADC units
float H2_SCALE = 0.4f;      // ~1000 ppm over 2500 ADC units

// ============================================
// SENSOR WARMUP PERIODS (milliseconds)
// ============================================
const unsigned long MQ5_WARMUP_MS = 60000;    // 60 seconds (MQ series)
const unsigned long O2_WARMUP_MS = 300000;    // 5 minutes (electrochemical)
const unsigned long SCD41_WARMUP_MS = 15000;  // 15 seconds (NDIR)
const unsigned long MEMS_WARMUP_MS = 180000;  // 3 minutes (MEMS gas sensors)
const unsigned long BME_WARMUP_MS = 60000;    // 1 minute (environmental)

// Grace period before any alarms trigger
const unsigned long STARTUP_GRACE_MS = max(max(O2_WARMUP_MS, MEMS_WARMUP_MS), MQ5_WARMUP_MS) + 10000;

// ============================================
// TIMING CONSTANTS
// ============================================
const unsigned long POLL_MS = 1000;           // Main sensor polling interval

const unsigned long GREEN_TOGGLE_MS = 250;    // Green LED blink rate (fault)
const unsigned long RED_ALARM_TOGGLE_MS = 125; // Red LED blink rate (alarm)

const unsigned long BUZZ_WARN_ON_MS = 200;    // Warning beep duration
const unsigned long BUZZ_WARN_OFF_MS = 800;   // Warning silence duration
const unsigned long BUZZ_ALARM_ON_MS = 150;   // Alarm beep duration
const unsigned long BUZZ_ALARM_OFF_MS = 150;  // Alarm silence duration

const unsigned long FAULT_CHIRP_EVERY_MS = 12000;  // Fault chirp interval
const unsigned long FAULT_CHIRP_LEN_MS = 60;       // Fault chirp duration

// Buzzer frequencies (Hz)
const uint16_t BUZ_STARTUP_FREQ = 2000;
const uint16_t BUZ_WARN_FREQ = 1800;
const uint16_t BUZ_FAULT_FREQ = 1100;
const uint16_t BUZ_ALARM_FREQ_LOW = 1600;
const uint16_t BUZ_ALARM_FREQ_HIGH = 2200;

// Alarm uses two-tone pattern instead of sweep
const unsigned long ALARM_TONE_DURATION_MS = 250;

// ============================================
// SCD41 CONFIGURATION
// ============================================
const unsigned long CO2_STALE_MS = 30000;     // Data older than this is stale
const uint8_t CO2_FAIL_MAX = 3;               // Consecutive failed reads before fault
const uint16_t CO2_MIN_VALID = 100;           // Minimum plausible CO2 (sensor can read low during warmup)
const uint16_t CO2_MAX_VALID = 40000;         // Maximum plausible CO2 reading

// ============================================
// BME680 IAQ CONFIGURATION
// ============================================
const int BME_WARMUP_READINGS = 10;           // Initial warmup samples
const int BME_CALIBRATION_SAMPLES = 20;       // Baseline calibration samples
const unsigned long BME_SAMPLE_INTERVAL_MS = 3000; // Time between calibration samples

// ============================================
// ENUMERATIONS
// ============================================
enum AirState : uint8_t {
  SAFE,
  WARN,
  ALARM
};

enum SensorStatus : uint8_t {
  SS_MISSING,   // Sensor not detected at boot
  SS_WARMUP,    // Sensor warming up
  SS_OK,        // Sensor operational
  SS_ERROR      // Sensor detected but not responding
};

// ============================================
// I2C ADDRESSES
// ============================================
#define Oxygen_IICAddress ADDRESS_3
#define OXY_COLLECT_NUMBER 10

const uint8_t BME680_ADDR_1 = 0x77;
const uint8_t BME680_ADDR_2 = 0x76;

// ============================================
// SENSOR OBJECTS
// ============================================
SCD4x scd41;
DFRobot_OxygenSensor oxygen;
Adafruit_BME680 bme;

// ============================================
// RUNTIME STATE VARIABLES
// ============================================
unsigned long bootMs = 0;
unsigned long lastPollMs = 0;

// LED state
bool greenPhase = false;
unsigned long lastGreenToggleMs = 0;
bool redPhase = false;
unsigned long lastRedToggleMs = 0;

// Buzzer state
bool buzzerOn = false;
bool buzzerToneHigh = false;  // For two-tone alarm
unsigned long lastBuzzToggleMs = 0;

// Fault chirp state
bool faultChirpActive = false;
unsigned long faultChirpEndMs = 0;
unsigned long lastFaultChirpMs = 0;

// ============================================
// SENSOR INITIALIZATION FLAGS
// ============================================
bool bme_init_ok = false;
bool scd_init_ok = false;
bool o2_init_ok = false;
bool mq5_init_ok = false;
bool h2s_init_ok = false;
bool co_init_ok = false;
bool ch4_init_ok = false;
bool nh3_init_ok = false;
bool h2_init_ok = false;
bool oled_init_ok = false;

// ============================================
// EXPECTED SENSOR FLAGS (latched at boot)
// ============================================
bool expected_bme = false;
bool expected_scd = false;
bool expected_o2 = false;
bool expected_mq5 = false;
bool expected_h2s = false;
bool expected_co = false;
bool expected_ch4 = false;
bool expected_nh3 = false;
bool expected_h2 = false;

// ============================================
// SENSOR DATA VALIDITY FLAGS
// ============================================
bool ok_bme = false;
bool ok_co2 = false;
bool ok_o2 = false;
bool ok_mq5 = false;
bool ok_h2s = false;
bool ok_co = false;
bool ok_ch4 = false;
bool ok_nh3 = false;
bool ok_h2 = false;

// ============================================
// SENSOR READINGS
// ============================================

// Environmental (BME680)
float bme_tempC = 0.0f;
float bme_rh = 0.0f;
float bme_hPa = 0.0f;
uint32_t bme_gas_ohms = 0;
uint32_t bme_gas_reference = 0;
bool bme_calibrated = false;
uint16_t bme_iaq_score = 0;  // 0-500, higher = worse
uint8_t bme_voc_pct = 0;     // 0-100%, higher = worse

// Gas readings
float o2_pct = 0.0f;
int mq5_raw = 0;
float h2s_ppm = 0.0f;
float co_ppm = 0.0f;
uint16_t ch4_ppm = 0;
float nh3_ppm = 0.0f;
uint16_t h2_ppm = 0;

// SCD41 cache
uint16_t co2_ppm_last = 0;
bool co2_has_value = false;
bool co2_fault = false;
bool co2_soft_fault = false;
uint8_t co2_fail_count = 0;
unsigned long co2_last_update_ms = 0;

// ============================================
// ESP32 PWM CONFIGURATION
// ============================================
const uint8_t BUZZER_RESOLUTION = 8;

// ============================================
// I2C SCANNER AND DIAGNOSTICS
// ============================================

void scanI2C() {
  Serial.println(F("\n===================================="));
  Serial.println(F("   I2C BUS SCAN"));
  Serial.println(F("===================================="));
  
  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("Device found at 0x"));
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.print(F(" - "));
      
      // Identify common devices
      switch (address) {
        case 0x3C:
        case 0x3D:
          Serial.println(F("SSD1306 OLED Display"));
          break;
        case 0x62:
          Serial.println(F("SCD41 CO2 Sensor"));
          break;
        case 0x73:
          Serial.println(F("DFRobot O2 Sensor (ADDRESS_3)"));
          break;
        case 0x74:
          Serial.println(F("DFRobot O2 Sensor (ADDRESS_4)"));
          break;
        case 0x75:
          Serial.println(F("DFRobot O2 Sensor (ADDRESS_5)"));
          break;
        case 0x76:
        case 0x77:
          Serial.println(F("BME680 Environmental Sensor"));
          break;
        default:
          Serial.println(F("Unknown device"));
          break;
      }
      nDevices++;
    }
  }

  if (nDevices == 0) {
    Serial.println(F("No I2C devices found!"));
    Serial.println(F("Check wiring and pull-up resistors."));
  } else {
    Serial.print(F("\nTotal devices found: "));
    Serial.println(nDevices);
  }
  
  Serial.println(F("====================================\n"));
}

// ============================================
// UTILITY FUNCTIONS
// ============================================

static inline unsigned long upTimeMs() {
  return millis() - bootMs;
}

static inline bool inStartupGrace() {
  return upTimeMs() < STARTUP_GRACE_MS;
}

static inline void setGreenLed(bool on) {
  digitalWrite(PIN_LED_GREEN, on ? LOW : HIGH);
}

static inline void setRedLed(bool on) {
  digitalWrite(PIN_LED_RED, on ? LOW : HIGH);
}

// ============================================
// STARTUP AUDIO FEEDBACK
// ============================================

void startupTripleBeep() {
  for (int i = 0; i < 3; i++) {
    ledcWriteTone(PIN_BUZZER, BUZ_STARTUP_FREQ);
    delay(80);
    ledcWriteTone(PIN_BUZZER, 0);
    delay(80);
  }
  delay(40);
}

void criticalFailureAlarm() {
  // Continuous alternating alarm for missing critical sensors
  while (true) {
    setRedLed(true);
    setGreenLed(true);
    ledcWriteTone(PIN_BUZZER, 2000);
    delay(250);
    
    setRedLed(false);
    setGreenLed(false);
    ledcWriteTone(PIN_BUZZER, 2500);
    delay(250);
    
    esp_task_wdt_reset();  // Keep watchdog happy
  }
}

// ============================================
// SENSOR DETECTION (IMPROVED)
// ============================================

// Sensor detection removed - using configuration flags only

// ============================================
// CALIBRATION ROUTINES
// ============================================

void calibrateAnalogSensor(uint8_t pin, int* r0_raw, float* scale, 
                          const char* name, float maxPpm, 
                          unsigned long warmupMs) {
  Serial.println(F("===================================="));
  Serial.print(F("   "));
  Serial.print(name);
  Serial.println(F(" CALIBRATION"));
  Serial.println(F("===================================="));
  
  // Check if sensor is present
  if (*r0_raw == 0) {
    Serial.print(name);
    Serial.println(F(" not installed, skipping."));
    return;
  }
  
  if (oled_init_ok) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    // Yellow zone - Sensor name
    display.setCursor(0, 6);
    display.print(F("CAL: "));
    display.println(name);
    display.drawLine(0, 17, 127, 17, SSD1306_WHITE);
    
    // Blue zone - Status
    display.setCursor(0, 24);
    display.println(F("Place in CLEAN AIR"));
    display.setCursor(0, 36);
    display.println(F("Warmup in progress..."));
    display.display();
  }
  
  Serial.print(F("Warming up ("));
  Serial.print(warmupMs / 1000);
  Serial.println(F(" sec)..."));
  
  unsigned long warmupEnd = millis() + warmupMs;
  int dotCount = 0;
  
  while (millis() < warmupEnd) {
    delay(1000);
    Serial.print(F("."));
    dotCount++;
    if (dotCount >= 60) {
      Serial.println();
      dotCount = 0;
    }
    esp_task_wdt_reset();
  }
  Serial.println();
  
  Serial.println(F("Collecting baseline samples..."));
  
  const int numSamples = 20;
  long total = 0;
  int minVal = ADC_MAX_VALUE;
  int maxVal = 0;
  
  for (int i = 0; i < numSamples; i++) {
    int reading = analogRead(pin);
    total += reading;
    if (reading < minVal) minVal = reading;
    if (reading > maxVal) maxVal = reading;
    
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.println(reading);
    
    delay(500);
    esp_task_wdt_reset();
  }
  
  int baseline = total / numSamples;
  int range = maxVal - minVal;
  
  *r0_raw = baseline;
  
  // Calculate scaling factor
  // Assume sensor can drop to ~500 ADC units at max concentration
  int usableRange = baseline - 500;
  if (usableRange > 0) {
    *scale = maxPpm / (float)usableRange;
  }
  
  Serial.println(F("===================================="));
  Serial.print(F("Baseline: "));
  Serial.println(baseline);
  Serial.print(F("Range: "));
  Serial.print(range);
  Serial.print(F(" ("));
  Serial.print((range * 100.0f) / baseline, 1);
  Serial.println(F("%)"));
  Serial.print(F("Scale: "));
  Serial.print(*scale, 4);
  Serial.println(F(" ppm/unit"));
  Serial.println(F("====================================\n"));
  
  if (oled_init_ok) {
    display.clearDisplay();
    display.setTextSize(1);
    
    // Yellow zone - Result
    display.setCursor(0, 6);
    display.print(name);
    display.print(F(" OK"));
    display.drawLine(0, 17, 127, 17, SSD1306_WHITE);
    
    // Blue zone - Values
    display.setCursor(0, 24);
    display.print(F("Base: "));
    display.println(baseline);
    display.setCursor(0, 36);
    display.print(F("Var: "));
    display.print((range * 100.0f) / baseline, 1);
    display.println(F("%"));
    display.display();
    delay(1500);
  }
}

void calibrateBME680() {
  Serial.println(F("===================================="));
  Serial.println(F("   BME680 IAQ CALIBRATION"));
  Serial.println(F("===================================="));

  if (!bme_init_ok) {
    Serial.println(F("BME680 missing, skipping calibration."));
    bme_calibrated = false;
    return;
  }

  if (oled_init_ok) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    // Yellow zone - Title
    display.setCursor(0, 6);
    display.println(F("BME680 CAL"));
    display.drawLine(0, 17, 127, 17, SSD1306_WHITE);
    
    // Blue zone - Status
    display.setCursor(0, 24);
    display.println(F("Place in CLEAN AIR"));
    display.setCursor(0, 36);
    display.println(F("Warmup + baseline"));
    display.setCursor(0, 52);
    display.println(F("Please wait..."));
    display.display();
  }

  Serial.print(F("Warmup ("));
  Serial.print(BME_WARMUP_READINGS);
  Serial.println(F(" readings)..."));

  for (int i = 0; i < BME_WARMUP_READINGS; i++) {
    if (bme.performReading()) {
      Serial.print(i + 1);
      Serial.print(F(": "));
      Serial.print(bme.gas_resistance);
      Serial.println(F(" Ohms"));
    }
    delay(BME_SAMPLE_INTERVAL_MS);
    esp_task_wdt_reset();
  }

  Serial.println(F("\nCollecting baseline...\n"));

  uint32_t total = 0;
  uint8_t count = 0;
  uint32_t min_gas = 0xFFFFFFFFu;
  uint32_t max_gas = 0;

  for (int i = 0; i < BME_CALIBRATION_SAMPLES; i++) {
    if (bme.performReading()) {
      uint32_t g = bme.gas_resistance;

      if (g > 0) {
        total += g;
        count++;
        if (g < min_gas) min_gas = g;
        if (g > max_gas) max_gas = g;

        Serial.print(i + 1);
        Serial.print(F(": "));
        Serial.print(g);
        Serial.println(F(" Ohms"));
      }
    }
    delay(BME_SAMPLE_INTERVAL_MS);
    esp_task_wdt_reset();
  }

  if (count < 10) {
    Serial.println(F("\nERROR: Too few gas samples; baseline not trusted."));
    bme_gas_reference = 100000;  // fallback
    bme_calibrated = false;
  } else {
    bme_gas_reference = total / count;
    bme_calibrated = true;
  }

  uint32_t range = (max_gas >= min_gas) ? (max_gas - min_gas) : 0;

  Serial.println(F("\n===================================="));
  Serial.print(F("Baseline: "));
  Serial.print(bme_gas_reference);
  Serial.println(F(" Ohms"));
  Serial.print(F("Range: "));
  Serial.print(range);
  Serial.println(F(" Ohms"));

  float var_pct = 0.0f;
  if (bme_gas_reference > 0) {
    var_pct = ((float)range / (float)bme_gas_reference) * 100.0f;
  }

  Serial.print(F("Variance: "));
  Serial.print(var_pct, 1);
  Serial.println(F("%"));

  if (var_pct > 20.0f) {
    Serial.println(F("WARNING: High variance (air not stable)."));
  } else {
    Serial.println(F("Calibration OK"));
  }

  Serial.println(F("====================================\n"));

  if (oled_init_ok) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    // Yellow zone - Title + status
    display.setCursor(0, 6);
    display.print(F("BME680 CAL "));
    display.println(bme_calibrated ? F("OK") : F("FAIL"));
    display.drawLine(0, 17, 127, 17, SSD1306_WHITE);
    
    // Blue zone - Details
    display.setCursor(0, 24);
    display.print(F("Ref: "));
    display.print(bme_gas_reference);
    display.println(F(" ohm"));
    display.setCursor(0, 36);
    display.print(F("Var: "));
    display.print(var_pct, 1);
    display.println(F("%"));
    display.display();
    delay(1500);
  }
}

// ============================================
// SENSOR POLLING FUNCTIONS
// ============================================

void pollBME680() {
  ok_bme = false;
  
  if (!bme.performReading()) {
    return;
  }

  bme_tempC = bme.temperature;
  bme_rh = bme.humidity;
  bme_hPa = bme.pressure / 100.0f;
  bme_gas_ohms = bme.gas_resistance;

  ok_bme = true;

  // Calculate IAQ if calibrated
  if (bme_calibrated && bme_gas_reference > 0) {
    calculateBME680_AQ();
  }
}

void pollSCD41_cached() {
  co2_soft_fault = false;

  uint16_t data_ready = 0;
  if (!scd41.getDataReadyStatus(data_ready)) {
    co2_soft_fault = true;
    co2_fail_count++;
    co2_fault = (co2_fail_count >= CO2_FAIL_MAX);
    Serial.println(F("SCD41: Readiness check failed"));
    return;
  }

  if ((data_ready & 0x07FF) == 0) {
    return;
  }

  if (!scd41.readMeasurement()) {
    co2_fail_count++;
    co2_fault = (co2_fail_count >= CO2_FAIL_MAX);
    Serial.println(F("SCD41: Read measurement failed"));
    return;
  }

  uint16_t c = scd41.getCO2();

  // Sanity check - allow low values during warmup but warn
  if (c < CO2_MIN_VALID || c > CO2_MAX_VALID) {
    co2_fail_count++;
    co2_fault = (co2_fail_count >= CO2_FAIL_MAX);
    #if DEBUG_VERBOSE
    Serial.print(F("SCD41: Invalid CO2 = "));
    Serial.println(c);
    #endif
    return;
  }
  
  // Warn about suspiciously low values (likely sensor still stabilizing)
  if (c < 350 && upTimeMs() > SCD41_WARMUP_MS) {
    #if DEBUG_VERBOSE
    Serial.print(F("SCD41: Low CO2 = "));
    Serial.print(c);
    Serial.println(F(" (sensor may still be stabilizing)"));
    #endif
  }

  co2_fail_count = 0;
  co2_fault = false;
  co2_ppm_last = c;
  co2_has_value = true;
  co2_last_update_ms = millis();
}

void deriveCO2_ok() {
  ok_co2 = false;
  
  if (!co2_has_value) return;
  if (upTimeMs() < SCD41_WARMUP_MS) return;
  if ((millis() - co2_last_update_ms) > CO2_STALE_MS) return;
  
  ok_co2 = true;
}

void pollOxygen() {
  ok_o2 = false;
  
  float v = oxygen.getOxygenData(OXY_COLLECT_NUMBER);
  
  // Sanity check (normal atmospheric O2 is 20.9%)
  if (v < 10.0f || v > 25.0f) {
    return;
  }
  
  o2_pct = v;
  ok_o2 = true;
}

void pollMQ5() {
  ok_mq5 = false;
  
  int raw = analogRead(PIN_MQ5_AO);
  
  mq5_raw = raw;
  ok_mq5 = true;
  
  #if DEBUG_VERBOSE
  Serial.print(F("[MQ5 poll: raw="));
  Serial.print(raw);
  Serial.print(F(" ok="));
  Serial.print(ok_mq5);
  Serial.println(F("]"));
  #endif
}

void pollH2S() {
  ok_h2s = false;
  
  int raw = analogRead(PIN_H2S_AO);
  

  // Calculate drop from baseline
  int delta = H2S_R0_RAW - raw;
  if (delta < 0) delta = 0;

  h2s_ppm = delta * H2S_SCALE;
  h2s_ppm = constrain(h2s_ppm, 0, 100);

  ok_h2s = true;
}

void pollCO() {
  ok_co = false;
  
  int raw = analogRead(PIN_CO_AO);
  

  int delta = CO_R0_RAW - raw;
  if (delta < 0) delta = 0;

  co_ppm = delta * CO_SCALE;
  co_ppm = constrain(co_ppm, 0, 10000);

  ok_co = true;
}

void pollCH4() {
  ok_ch4 = false;
  
  int raw = analogRead(PIN_CH4_AO);
  

  int delta = CH4_R0_RAW - raw;
  if (delta < 0) delta = 0;

  float ppm = delta * CH4_SCALE;
  ch4_ppm = constrain((int)ppm, 0, 50000);

  ok_ch4 = true;
}

void pollNH3() {
  ok_nh3 = false;
  
  int raw = analogRead(PIN_NH3_AO);
  

  int delta = NH3_R0_RAW - raw;
  if (delta < 0) delta = 0;

  nh3_ppm = delta * NH3_SCALE;
  nh3_ppm = constrain(nh3_ppm, 0, 500);

  ok_nh3 = true;
}

void pollH2() {
  ok_h2 = false;
  
  int raw = analogRead(PIN_H2_AO);
  

  int delta = H2_R0_RAW - raw;
  if (delta < 0) delta = 0;

  float ppm = delta * H2_SCALE;
  h2_ppm = constrain((int)ppm, 0, 40000);

  ok_h2 = true;
}

// ============================================
// SENSOR STATUS FUNCTIONS
// ============================================

SensorStatus statusBME() {
  if (!bme_init_ok) return SS_MISSING;
  if (upTimeMs() < BME_WARMUP_MS) return SS_WARMUP;
  return ok_bme ? SS_OK : SS_ERROR;
}

SensorStatus statusSCD() {
  if (!scd_init_ok) return SS_MISSING;
  if (upTimeMs() < SCD41_WARMUP_MS) return SS_WARMUP;
  return ok_co2 ? SS_OK : SS_ERROR;
}

SensorStatus statusO2() {
  if (!o2_init_ok) return SS_MISSING;
  if (upTimeMs() < O2_WARMUP_MS) return SS_WARMUP;
  return ok_o2 ? SS_OK : SS_ERROR;
}

SensorStatus statusMQ5() {
  if (!mq5_init_ok) return SS_MISSING;
  if (upTimeMs() < MQ5_WARMUP_MS) return SS_WARMUP;
  return ok_mq5 ? SS_OK : SS_ERROR;
}

SensorStatus statusH2S() {
  if (!h2s_init_ok) return SS_MISSING;
  if (upTimeMs() < MEMS_WARMUP_MS) return SS_WARMUP;
  return ok_h2s ? SS_OK : SS_ERROR;
}

SensorStatus statusCO() {
  if (!co_init_ok) return SS_MISSING;
  if (upTimeMs() < MEMS_WARMUP_MS) return SS_WARMUP;
  return ok_co ? SS_OK : SS_ERROR;
}

SensorStatus statusCH4() {
  if (!ch4_init_ok) return SS_MISSING;
  if (upTimeMs() < MEMS_WARMUP_MS) return SS_WARMUP;
  return ok_ch4 ? SS_OK : SS_ERROR;
}

SensorStatus statusNH3() {
  if (!nh3_init_ok) return SS_MISSING;
  if (upTimeMs() < MEMS_WARMUP_MS) return SS_WARMUP;
  return ok_nh3 ? SS_OK : SS_ERROR;
}

SensorStatus statusH2() {
  if (!h2_init_ok) return SS_MISSING;
  if (upTimeMs() < MEMS_WARMUP_MS) return SS_WARMUP;
  return ok_h2 ? SS_OK : SS_ERROR;
}

const char* ssText(SensorStatus s) {
  switch (s) {
    case SS_MISSING: return "MISS";
    case SS_WARMUP: return "WARM";
    case SS_OK: return "OK";
    default: return "ERR";
  }
}

// ============================================
// SYSTEM STATUS EVALUATION
// ============================================

bool sensorsError() {
  if (inStartupGrace()) return false;

  // Only check sensors that were present at boot
  if (expected_bme && statusBME() == SS_ERROR) return true;
  if (expected_scd && statusSCD() == SS_ERROR) return true;
  if (expected_o2 && statusO2() == SS_ERROR) return true;
  if (expected_mq5 && statusMQ5() == SS_ERROR) return true;
  if (expected_h2s && statusH2S() == SS_ERROR) return true;
  if (expected_co && statusCO() == SS_ERROR) return true;
  if (expected_ch4 && statusCH4() == SS_ERROR) return true;
  if (expected_nh3 && statusNH3() == SS_ERROR) return true;
  if (expected_h2 && statusH2() == SS_ERROR) return true;

  return false;
}

AirState evaluateAir() {
  if (inStartupGrace()) return SAFE;

  bool warn = false;
  bool alarm = false;

  // Oxygen levels
  if (statusO2() == SS_OK) {
    if (o2_pct < O2_ALARM) {
      alarm = true;
    } else if (o2_pct < O2_WARN || o2_pct > O2_HIGH_WARN) {
      warn = true;
    }
  }

  // Carbon Dioxide
  if (statusSCD() == SS_OK) {
    if (co2_ppm_last >= CO2_ALARM) {
      alarm = true;
    } else if (co2_ppm_last >= CO2_WARN) {
      warn = true;
    }
  }

  // MQ5 - LPG/Propane (inverse logic)
  if (statusMQ5() == SS_OK) {
    int drop = MQ5_R0_RAW - mq5_raw;
    if (drop >= MQ5_ALARM_DROP) {
      alarm = true;
    } else if (drop >= MQ5_WARN_DROP) {
      warn = true;
    }
  }

  // Hydrogen Sulfide
  if (statusH2S() == SS_OK) {
    if (h2s_ppm >= H2S_ALARM) {
      alarm = true;
    } else if (h2s_ppm >= H2S_WARN) {
      warn = true;
    }
  }

  // Carbon Monoxide
  if (statusCO() == SS_OK) {
    if (co_ppm >= CO_ALARM) {
      alarm = true;
    } else if (co_ppm >= CO_WARN) {
      warn = true;
    }
  }

  // Methane
  if (statusCH4() == SS_OK) {
    if (ch4_ppm >= CH4_ALARM) {
      alarm = true;
    } else if (ch4_ppm >= CH4_WARN) {
      warn = true;
    }
  }

  // Ammonia
  if (statusNH3() == SS_OK) {
    if (nh3_ppm >= NH3_ALARM) {
      alarm = true;
    } else if (nh3_ppm >= NH3_WARN) {
      warn = true;
    }
  }

  // Hydrogen
  if (statusH2() == SS_OK) {
    if (h2_ppm >= H2_ALARM) {
      alarm = true;
    } else if (h2_ppm >= H2_WARN) {
      warn = true;
    }
  }

  if (alarm) return ALARM;
  if (warn) return WARN;
  return SAFE;
}

// ============================================
// LED AND BUZZER CONTROL
// ============================================

void driveGreenLED(bool err) {
  if (inStartupGrace()) {
    setGreenLed(true);
    return;
  }
  
  if (!err) {
    setGreenLed(true);
    return;
  }

  // Blink on error
  unsigned long now = millis();
  if (now - lastGreenToggleMs >= GREEN_TOGGLE_MS) {
    lastGreenToggleMs = now;
    greenPhase = !greenPhase;
  }
  setGreenLed(greenPhase);
}

void driveRedLED(AirState st) {
  if (inStartupGrace()) {
    setRedLed(true);
    return;
  }

  if (st == SAFE) {
    setRedLed(false);
    return;
  }
  
  if (st == WARN) {
    setRedLed(true);
    return;
  }

  // Rapid blink on alarm
  unsigned long now = millis();
  if (now - lastRedToggleMs >= RED_ALARM_TOGGLE_MS) {
    lastRedToggleMs = now;
    redPhase = !redPhase;
  }
  setRedLed(redPhase);
}

void updateFaultChirp(bool err, AirState st) {
  unsigned long now = millis();

  // If chirp is active, wait for it to finish
  if (faultChirpActive) {
    if (now >= faultChirpEndMs) {
      faultChirpActive = false;
      ledcWriteTone(PIN_BUZZER, 0);
    }
    return;
  }

  // Only chirp during SAFE state when there's an error
  if (inStartupGrace()) return;
  if (!err) return;
  if (st != SAFE) return;

  // Time for next chirp?
  if (now - lastFaultChirpMs >= FAULT_CHIRP_EVERY_MS) {
    lastFaultChirpMs = now;
    faultChirpActive = true;
    faultChirpEndMs = now + FAULT_CHIRP_LEN_MS;
    ledcWriteTone(PIN_BUZZER, BUZ_FAULT_FREQ);
  }
}

void driveBuzzer(AirState st, bool err) {
  // Fault chirp takes priority
  updateFaultChirp(err, st);
  if (faultChirpActive) return;

  // No sound during startup or if air is safe
  if (inStartupGrace() || st == SAFE) {
    buzzerOn = false;
    ledcWriteTone(PIN_BUZZER, 0);
    return;
  }

  unsigned long now = millis();
  unsigned long onMs = (st == WARN) ? BUZZ_WARN_ON_MS : BUZZ_ALARM_ON_MS;
  unsigned long offMs = (st == WARN) ? BUZZ_WARN_OFF_MS : BUZZ_ALARM_OFF_MS;

  unsigned long interval = buzzerOn ? onMs : offMs;

  if (now - lastBuzzToggleMs >= interval) {
    lastBuzzToggleMs = now;
    buzzerOn = !buzzerOn;

    if (!buzzerOn) {
      ledcWriteTone(PIN_BUZZER, 0);
      return;
    }

    // Warning: single tone
    if (st == WARN) {
      ledcWriteTone(PIN_BUZZER, BUZ_WARN_FREQ);
    } 
    // Alarm: alternating two-tone
    else {
      buzzerToneHigh = !buzzerToneHigh;
      uint16_t freq = buzzerToneHigh ? BUZ_ALARM_FREQ_HIGH : BUZ_ALARM_FREQ_LOW;
      ledcWriteTone(PIN_BUZZER, freq);
    }
  }
}

// ============================================
// BME680 IAQ CALCULATION
// ============================================

void calculateBME680_AQ() {
  float humidity = bme_rh;
  uint32_t gas_resistance = bme_gas_ohms;

  // Humidity score (0-25): optimal around 40% RH
  float hum_score;
  if (humidity >= 38.0f && humidity <= 42.0f) {
    hum_score = 25.0f;
  } else if (humidity < 38.0f) {
    hum_score = 25.0f * (humidity / 38.0f);
  } else {
    hum_score = 25.0f * ((100.0f - humidity) / 58.0f);
  }
  hum_score = constrain(hum_score, 0.0f, 25.0f);

  // Gas score (0-75): higher resistance = cleaner air
  float gas_clean = (float)bme_gas_reference;
  float gas_poor = gas_clean * 0.5f;  // 50% drop = poor air

  float gas_score;
  if ((float)gas_resistance >= gas_clean) {
    gas_score = 75.0f;
  } else if ((float)gas_resistance <= gas_poor) {
    gas_score = 0.0f;
  } else {
    float ratio = ((float)gas_resistance - gas_poor) / (gas_clean - gas_poor);
    gas_score = 75.0f * ratio;
  }
  gas_score = constrain(gas_score, 0.0f, 75.0f);

  // Combine scores
  float quality = hum_score + gas_score;  // 0-100 (higher=better)
  float iaq = (100.0f - quality) * 5.0f;  // 0-500 (higher=worse)
  iaq = constrain(iaq, 0.0f, 500.0f);
  bme_iaq_score = (uint16_t)iaq;

  // VOC percentage
  if (gas_resistance >= bme_gas_reference) {
    bme_voc_pct = 0;
  } else {
    float gas_drop = (float)(bme_gas_reference - gas_resistance);
    float max_drop = (float)bme_gas_reference * 0.5f;
    float pct = (max_drop > 0.0f) ? (gas_drop / max_drop) * 100.0f : 0.0f;
    pct = constrain(pct, 0.0f, 100.0f);
    bme_voc_pct = (uint8_t)pct;
  }
}

const char* getBME680_AQText(uint16_t iaq) {
  if (iaq <= 50) return "Excellent";
  if (iaq <= 100) return "Good";
  if (iaq <= 150) return "Light Poll";
  if (iaq <= 200) return "Mod Poll";
  if (iaq <= 250) return "Heavy Poll";
  return "Severe Poll";
}

// ============================================
// OLED DISPLAY FUNCTIONS (Optimized for Yellow/Blue OLED)
// Yellow Zone: 0-19, Blue Zone: 20-63
// ============================================

void drawHeader(AirState st) {
  // ===== YELLOW ZONE (rows 0-19) - Critical status only =====
  // This section is most visible - use for immediate status recognition
  
  display.setTextSize(2);  // Larger text for yellow zone visibility
  display.setTextColor(SSD1306_WHITE);
  
  // Status on left (large and obvious)
  display.setCursor(0, 2);  // Centered in 20px zone
  if (st == SAFE) {
    display.print(F("SAFE"));
  } else if (st == WARN) {
    display.print(F("WARN"));
  } else {
    display.print(F("ALARM"));  // Maximum visibility for danger
  }
  
  // Icon/indicator on right
  display.setTextSize(2);
  display.setCursor(100, 2);
  if (st == SAFE) {
    display.print(F("OK"));
  } else if (st == WARN) {
    display.print(F("!!"));
  } else {
    display.print(F("**"));  // Attention grabbing
  }
  
  // Divider line at bottom of yellow zone (row 19)
  display.drawLine(0, 17, 127, 17, SSD1306_WHITE);
  
  // ===== BLUE ZONE starts at row 20 =====
}

void drawSummaryPage() {
  display.clearDisplay();
  
  // ===== YELLOW ZONE (0-19) - Most critical reading =====
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  
  AirState st = evaluateAir();
  
  // Show most critical gas in yellow zone for maximum visibility
  bool criticalShown = false;
  
  // Priority 1: O2 (life critical)
  if (ok_o2 && (o2_pct < O2_WARN || o2_pct > O2_HIGH_WARN)) {
    display.setCursor(0, 2);
    display.print(F("O2:"));
    display.print(o2_pct, 1);
    display.print(F("%"));
    if (o2_pct < O2_ALARM) {
      display.setCursor(100, 2);
      display.print(F("**"));
    }
    criticalShown = true;
  }
  // Priority 2: CO (deadly, odorless)
  else if (ok_co && co_ppm >= CO_WARN) {
    display.setCursor(0, 2);
    display.print(F("CO:"));
    display.print((int)co_ppm);
    if (co_ppm >= CO_ALARM) {
      display.setCursor(100, 2);
      display.print(F("**"));
    }
    criticalShown = true;
  }
  // Priority 3: H2S (highly toxic)
  else if (ok_h2s && h2s_ppm >= H2S_WARN) {
    display.setCursor(0, 2);
    display.print(F("H2S:"));
    display.print(h2s_ppm, 1);
    if (h2s_ppm >= H2S_ALARM) {
      display.setCursor(100, 2);
      display.print(F("**"));
    }
    criticalShown = true;
  }
  // Priority 4: CO2 (asphyxiant)
  else if (ok_co2 && co2_ppm_last >= CO2_WARN) {
    display.setCursor(0, 2);
    display.print(F("CO2:"));
    display.print(co2_ppm_last);
    if (co2_ppm_last >= CO2_ALARM) {
      display.setCursor(100, 2);
      display.print(F("**"));
    }
    criticalShown = true;
  }
  // No issues: Show status
  else {
    display.setCursor(0, 2);
    display.print(F("SAFE"));
    display.setCursor(100, 2);
    display.print(F("OK"));
  }
  
  // Divider at row 19
  display.drawLine(0, 17, 127, 17, SSD1306_WHITE);
  
  // ===== BLUE ZONE (20-63) - Detailed readings =====
  display.setTextSize(1);
  
  int y = 22;  // Start in blue zone (20px + 2px margin)
  
  // O2 (always show if available)
  if (ok_o2) {
    display.setCursor(0, y);
    display.print(F("O2:"));
    display.setCursor(30, y);
    display.print(o2_pct, 1);
    display.print(F("%"));
    y += 10;
  }
  
  // CO2 (always show if available)
  if (ok_co2) {
    display.setCursor(0, y);
    display.print(F("CO2:"));
    display.setCursor(30, y);
    display.print(co2_ppm_last);
    display.print(F("ppm"));
    y += 10;
  }
  
  // Toxic gases (if installed)
  if (ok_h2s) {
    display.setCursor(0, y);
    display.print(F("H2S:"));
    display.setCursor(30, y);
    display.print(h2s_ppm, 1);
    y += 10;
  }
  
  if (ok_co) {
    display.setCursor(0, y);
    display.print(F("CO:"));
    display.setCursor(30, y);
    display.print((int)co_ppm);
    y += 10;
  }
  
  // Explosive gases (brief if present)
  if (ok_mq5) {
    display.setCursor(0, y);
    display.print(F("LPG:"));
    display.setCursor(30, y);
    int drop = MQ5_R0_RAW - mq5_raw;
    if (drop >= MQ5_WARN_DROP) {
      display.print(F("DETECT"));
    } else {
      display.print(F("OK"));
    }
    y += 10;
  }
  
  display.display();
}

void drawToxicGasesPage() {
  display.clearDisplay();
  
  // ===== YELLOW ZONE (0-19) - Page title =====
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 6);
  display.print(F("TOXIC GASES"));
  
  // Show worst reading in yellow if any alarm
  if (ok_h2s && h2s_ppm >= H2S_ALARM) {
    display.setCursor(85, 6);
    display.print(F("H2S ALARM"));
  } else if (ok_co && co_ppm >= CO_ALARM) {
    display.setCursor(85, 6);
    display.print(F("CO ALARM"));
  } else if (ok_nh3 && nh3_ppm >= NH3_ALARM) {
    display.setCursor(85, 6);
    display.print(F("NH3 ALARM"));
  }
  
  display.drawLine(0, 17, 127, 17, SSD1306_WHITE);

  // ===== BLUE ZONE (16-63) - Details =====
  display.setTextSize(1);

  // H2S
  display.setCursor(0, 22);
  display.print(F("H2S:"));
  display.setCursor(40, 22);
  if (ok_h2s) {
    display.print(h2s_ppm, 1);
    display.print(F("ppm"));
  } else {
    display.print(F("--"));
  }
  display.setCursor(100, 22);
  display.print(ssText(statusH2S()));

  // CO
  display.setCursor(0, 32);
  display.print(F("CO:"));
  display.setCursor(40, 32);
  if (ok_co) {
    display.print(co_ppm, 0);
    display.print(F("ppm"));
  } else {
    display.print(F("--"));
  }
  display.setCursor(100, 32);
  display.print(ssText(statusCO()));

  // NH3
  display.setCursor(0, 42);
  display.print(F("NH3:"));
  display.setCursor(40, 42);
  if (ok_nh3) {
    display.print(nh3_ppm, 1);
    display.print(F("ppm"));
  } else {
    display.print(F("--"));
  }
  display.setCursor(100, 42);
  display.print(ssText(statusNH3()));

  display.print((int)CO_WARN);

  display.display();
}

void drawExplosiveGasesPage() {
  display.clearDisplay();
  
  // ===== YELLOW ZONE (0-19) - Page title =====
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 6);
  display.print(F("EXPLOSIVE GASES"));
  
  // Alert if any explosive gas detected
  bool explosive_detected = false;
  if (ok_ch4 && ch4_ppm >= CH4_WARN) explosive_detected = true;
  if (ok_h2 && h2_ppm >= H2_WARN) explosive_detected = true;
  if (ok_mq5 && (MQ5_R0_RAW - mq5_raw) >= MQ5_WARN_DROP) explosive_detected = true;
  
  if (explosive_detected) {
    display.setCursor(100, 6);
    display.print(F("RISK!"));
  }
  
  display.drawLine(0, 17, 127, 17, SSD1306_WHITE);

  // ===== BLUE ZONE (16-63) - Details =====
  display.setTextSize(1);

  // CH4
  display.setCursor(0, 22);
  display.print(F("CH4:"));
  display.setCursor(40, 22);
  if (ok_ch4) {
    display.print(ch4_ppm);
    display.print(F("ppm"));
  } else {
    display.print(F("--"));
  }
  display.setCursor(100, 22);
  display.print(ssText(statusCH4()));

  // H2
  display.setCursor(0, 32);
  display.print(F("H2:"));
  display.setCursor(40, 32);
  if (ok_h2) {
    display.print(h2_ppm);
    display.print(F("ppm"));
  } else {
    display.print(F("--"));
  }
  display.setCursor(100, 32);
  display.print(ssText(statusH2()));

  // MQ5 (LPG/Propane)
  display.setCursor(0, 42);
  display.print(F("LPG:"));
  display.setCursor(40, 42);
  if (ok_mq5) {
    display.print(mq5_raw);
  } else {
    display.print(F("--"));
  }
  display.setCursor(100, 42);
  display.print(ssText(statusMQ5()));

  display.display();
}

void drawEnvironmentPage() {
  display.clearDisplay();
  
  // ===== YELLOW ZONE (0-17) - Page title =====
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 6);
  display.print(F("ENVIRONMENT"));
  
  display.drawLine(0, 17, 127, 17, SSD1306_WHITE);  // 2px higher

  // ===== BLUE ZONE (18-63) - All data same size =====
  display.setTextSize(1);

  // Temperature
  display.setCursor(0, 20);
  display.print(F("Temp:"));
  display.setCursor(50, 20);
  if (ok_bme) {
    display.print(bme_tempC, 1);
    display.print(F("C"));
  } else {
    display.print(F("--"));
  }

  // Humidity
  display.setCursor(0, 30);
  display.print(F("RH:"));
  display.setCursor(50, 30);
  if (ok_bme) {
    display.print(bme_rh, 1);
    display.print(F("%"));
  } else {
    display.print(F("--"));
  }

  // Pressure
  display.setCursor(0, 40);
  display.print(F("Press:"));
  display.setCursor(50, 40);
  if (ok_bme) {
    display.print(bme_hPa, 0);
    display.print(F("hPa"));
  } else {
    display.print(F("--"));
  }

  // IAQ score
  display.setCursor(0, 50);
  display.print(F("IAQ:"));
  display.setCursor(50, 50);
  if (ok_bme && bme_calibrated && bme_gas_reference > 0) {
    display.print(bme_iaq_score);
    display.print(F(" "));
    display.print(getBME680_AQText(bme_iaq_score));
  } else {
    display.print(F("--"));
  }

  display.display();
}

void drawStatusPage() {
  display.clearDisplay();
  
  // ===== YELLOW ZONE (0-19) - System status =====
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 2);
  display.print(F("SYSTEM STATUS"));
  
  display.setTextSize(1);
  display.setCursor(0, 10);
  
  if (sensorsError()) {
    display.print(F("FAULT"));
  } else {
    display.print(F("OK"));
  }
  
  // Uptime indicator
  display.setTextSize(1);
  unsigned long uptimeSec = upTimeMs() / 1000;
  unsigned long hours = uptimeSec / 3600;
  display.setCursor(85, 10);
  display.print(hours);
  display.print(F("h"));
  
  display.drawLine(0, 17, 127, 17, SSD1306_WHITE);

  // ===== BLUE ZONE (16-63) - Sensor details =====
  display.setTextSize(1);

  int y = 22;

  // BME680
  display.setCursor(0, y);
  display.print(F("BME:"));
  display.setCursor(50, y);
  display.print(ssText(statusBME()));
  y += 10;

  // SCD41
  display.setCursor(0, y);
  display.print(F("SCD:"));
  display.setCursor(50, y);
  display.print(ssText(statusSCD()));
  y += 10;

  // O2
  display.setCursor(0, y);
  display.print(F("O2:"));
  display.setCursor(50, y);
  display.print(ssText(statusO2()));
  y += 10;

  // Gas sensors summary
  display.setCursor(0, y);
  display.print(F("GAS:"));
  display.setCursor(50, y);
  int gasOK = 0;
  int gasTotal = 0;
  
  if (expected_h2s) {
    gasTotal++;
    if (statusH2S() == SS_OK) gasOK++;
  }
  if (expected_co) {
    gasTotal++;
    if (statusCO() == SS_OK) gasOK++;
  }
  if (expected_ch4) {
    gasTotal++;
    if (statusCH4() == SS_OK) gasOK++;
  }
  if (expected_nh3) {
    gasTotal++;
    if (statusNH3() == SS_OK) gasOK++;
  }
  if (expected_h2) {
    gasTotal++;
    if (statusH2() == SS_OK) gasOK++;
  }
  if (expected_mq5) {
    gasTotal++;
    if (statusMQ5() == SS_OK) gasOK++;
  }

  display.print(gasOK);
  display.print(F("/"));
  display.print(gasTotal);

  display.display();
}



void updateDisplay() {
  if (!oled_init_ok) return;

  unsigned long now = millis();
  AirState st = evaluateAir();

  // Determine which pages have issues
  bool toxicIssue = false;
  bool explosiveIssue = false;
  bool environmentIssue = false;

  if (statusH2S() == SS_OK && (h2s_ppm >= H2S_WARN)) toxicIssue = true;
  if (statusCO() == SS_OK && (co_ppm >= CO_WARN)) toxicIssue = true;
  if (statusNH3() == SS_OK && (nh3_ppm >= NH3_WARN)) toxicIssue = true;

  if (statusCH4() == SS_OK && (ch4_ppm >= CH4_WARN)) explosiveIssue = true;
  if (statusH2() == SS_OK && (h2_ppm >= H2_WARN)) explosiveIssue = true;
  if (statusMQ5() == SS_OK && ((MQ5_R0_RAW - mq5_raw) >= MQ5_WARN_DROP)) explosiveIssue = true;

  if (statusO2() == SS_OK && (o2_pct < O2_WARN || o2_pct > O2_HIGH_WARN)) environmentIssue = true;
  if (statusSCD() == SS_OK && (co2_ppm_last >= CO2_WARN)) environmentIssue = true;

  // If alarm or warning, show only relevant pages
  if (st == ALARM || st == WARN) {
    DisplayPage issuePages[4];
    int issuePageCount = 0;

    issuePages[issuePageCount++] = PAGE_SUMMARY;

    if (toxicIssue) issuePages[issuePageCount++] = PAGE_TOXIC_GASES;
    if (explosiveIssue) issuePages[issuePageCount++] = PAGE_EXPLOSIVE_GASES;
    if (environmentIssue) issuePages[issuePageCount++] = PAGE_ENVIRONMENT;

    if (now - lastPageChange >= PAGE_DURATION_MS) {
      lastPageChange = now;

      // Find current page in issue list
      int currentIndex = -1;
      for (int i = 0; i < issuePageCount; i++) {
        if (issuePages[i] == currentPage) {
          currentIndex = i;
          break;
        }
      }

      // Advance to next page
      if (currentIndex == -1 || currentIndex >= issuePageCount - 1) {
        currentPage = issuePages[0];
      } else {
        currentPage = issuePages[currentIndex + 1];
      }
    }

    // Verify current page is valid
    bool validPage = false;
    for (int i = 0; i < issuePageCount; i++) {
      if (issuePages[i] == currentPage) {
        validPage = true;
        break;
      }
    }
    if (!validPage) {
      currentPage = issuePages[0];
    }

  } else {
    // Normal operation - rotate through all pages
    if (now - lastPageChange >= PAGE_DURATION_MS) {
      lastPageChange = now;
      currentPage = (DisplayPage)((currentPage + 1) % PAGE_COUNT);
    }
  }

  // Draw current page
  switch (currentPage) {
    case PAGE_SUMMARY:
      drawSummaryPage();
      break;
    case PAGE_TOXIC_GASES:
      drawToxicGasesPage();
      break;
    case PAGE_EXPLOSIVE_GASES:
      drawExplosiveGasesPage();
      break;
    case PAGE_ENVIRONMENT:
      drawEnvironmentPage();
      break;
    case PAGE_STATUS:
      drawStatusPage();
      break;
  }
}

// ============================================
// CRITICAL SENSOR VALIDATION
// ============================================

void validateCriticalSensors() {
  bool critical_failure = false;

  Serial.println(F("\n===================================="));
  Serial.println(F("   CRITICAL SENSOR VALIDATION"));
  Serial.println(F("===================================="));

  #if REQUIRE_O2_SENSOR
  if (!o2_init_ok) {
    Serial.println(F("CRITICAL: O2 sensor REQUIRED but MISSING!"));
    Serial.println(F("  Set REQUIRE_O2_SENSOR to false"));
    Serial.println(F("  if sensor is not installed."));
    critical_failure = true;
  } else {
    Serial.println(F("O2 sensor: OK (required)"));
  }
  #else
  if (o2_init_ok) {
    Serial.println(F("O2 sensor: OK (optional)"));
  } else {
    Serial.println(F("O2 sensor: Not installed (optional)"));
  }
  #endif

  #if REQUIRE_CO_SENSOR
  if (!co_init_ok) {
    Serial.println(F("CRITICAL: CO sensor REQUIRED but MISSING!"));
    Serial.println(F("  Set REQUIRE_CO_SENSOR to false"));
    Serial.println(F("  if sensor is not installed."));
    critical_failure = true;
  } else {
    Serial.println(F("CO sensor: OK (required)"));
  }
  #else
  if (co_init_ok) {
    Serial.println(F("CO sensor: OK (optional)"));
  } else {
    Serial.println(F("CO sensor: Not installed (optional)"));
  }
  #endif

  #if REQUIRE_SCD_SENSOR
  if (!scd_init_ok) {
    Serial.println(F("CRITICAL: SCD41 sensor REQUIRED but MISSING!"));
    Serial.println(F("  Set REQUIRE_SCD_SENSOR to false"));
    Serial.println(F("  if sensor is not installed."));
    critical_failure = true;
  } else {
    Serial.println(F("SCD41 sensor: OK (required)"));
  }
  #else
  if (scd_init_ok) {
    Serial.println(F("SCD41 sensor: OK (optional)"));
  } else {
    Serial.println(F("SCD41 sensor: Not installed (optional)"));
  }
  #endif

  Serial.println(F("===================================="));

  if (critical_failure) {
    Serial.println(F("\n** SYSTEM HALTED **"));
    Serial.println(F("Required sensors are missing."));
    Serial.println(F("Either install the sensors OR"));
    Serial.println(F("set REQUIRE_X_SENSOR to false"));
    Serial.println(F("at the top of the code.\n"));
    
    if (oled_init_ok) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println(F("** CONFIG ERROR **"));
      display.setCursor(0, 16);
      display.println(F("Required sensors"));
      display.println(F("not installed!"));
      display.setCursor(0, 40);
      display.println(F("Update config or"));
      display.println(F("install sensors."));
      display.display();
    }

    criticalFailureAlarm();  // Never returns
  }

  // Count installed sensors
  int installed = 0;
  if (bme_init_ok) installed++;
  if (scd_init_ok) installed++;
  if (o2_init_ok) installed++;
  if (mq5_init_ok) installed++;
  if (h2s_init_ok) installed++;
  if (co_init_ok) installed++;
  if (ch4_init_ok) installed++;
  if (nh3_init_ok) installed++;
  if (h2_init_ok) installed++;

  Serial.println(F("\n** VALIDATION PASSED **"));
  Serial.print(F("Sensors detected: "));
  Serial.println(installed);
  Serial.println(F("System will adapt to available sensors.\n"));
}

// ============================================
// SETUP
// ============================================

void setup() {
  Serial.begin(115200);
  delay(500);

  // Configure ADC
  analogReadResolution(ADC_RESOLUTION);
  analogSetAttenuation(ADC_11db);  // 0-3.3V range

  // Enable internal pulldowns on all analog pins
  // Now using pins that SUPPORT pulldowns (32, 33, 25, 26, 27, 14)
  pinMode(PIN_MQ5_AO, INPUT_PULLDOWN);
  pinMode(PIN_H2S_AO, INPUT_PULLDOWN);
  pinMode(PIN_CO_AO, INPUT_PULLDOWN);
  pinMode(PIN_CH4_AO, INPUT_PULLDOWN);
  pinMode(PIN_NH3_AO, INPUT_PULLDOWN);
  pinMode(PIN_H2_AO, INPUT_PULLDOWN);

  // Initialize GPIO
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);

  setGreenLed(false);
  setRedLed(false);

  // Setup PWM for buzzer
  ledcAttach(PIN_BUZZER, 2000, BUZZER_RESOLUTION);
  ledcWriteTone(PIN_BUZZER, 0);

  // Initialize watchdog (ESP32 Arduino Core 3.x API)
  // Check if watchdog is already initialized by Arduino core
  esp_err_t wdt_status = esp_task_wdt_status(NULL);
  if (wdt_status == ESP_ERR_NOT_FOUND) {
    // Watchdog not initialized, set it up
    esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WDT_TIMEOUT * 1000,
      .idle_core_mask = 0,
      .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
  }
  // Subscribe this task to the watchdog
  esp_task_wdt_add(NULL);

  bootMs = millis();

  Serial.println();
  Serial.println(F("============================================"));
  Serial.println(F("   CAVE AIR SAFETY MONITOR - ESP32"));
  Serial.println(F("   Version 2.0 - Production Release"));
  Serial.println(F("============================================\n"));

  // Power-up delay for sensors to stabilize
  Serial.println(F("Waiting for sensors to power up..."));
  delay(2000);
  esp_task_wdt_reset();

  // Visual/audio startup indication
  setGreenLed(true);
  setRedLed(true);
  startupTripleBeep();

  // Initialize I2C
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);  // 100kHz for reliability
  Wire.setTimeout(3000);

  // I2C Bus Diagnostics
  Serial.println(F("\n--- I2C Bus Scan ---"));
  int deviceCount = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print(F("I2C device found at 0x"));
      if (addr < 16) Serial.print(F("0"));
      Serial.print(addr, HEX);
      
      // Identify common devices
      if (addr == 0x3C) Serial.print(F(" (likely OLED)"));
      else if (addr == 0x62) Serial.print(F(" (likely SCD41)"));
      else if (addr == 0x76 || addr == 0x77) Serial.print(F(" (likely BME680)"));
      else if (addr == 0x73) Serial.print(F(" (likely O2 sensor)"));
      
      Serial.println();
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println(F("No I2C devices found!"));
    Serial.println(F("Check: SDA/SCL wiring, pullup resistors"));
  } else {
    Serial.print(F("Found "));
    Serial.print(deviceCount);
    Serial.println(F(" I2C device(s)"));
  }
  Serial.println();

  // ========================================
  // OLED Display
  // ========================================
  Serial.print(F("Init OLED SSD1306... "));
  oled_init_ok = display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  Serial.println(oled_init_ok ? F("OK") : F("NOT FOUND"));
  
  if (oled_init_ok) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 10);
    display.println(F("CAVE AIR"));
    display.setCursor(10, 30);
    display.println(F("MONITOR"));
    display.setTextSize(1);
    display.setCursor(20, 50);
    display.println(F("Booting..."));
    display.display();
  }

  // ========================================
  // BME680 Environmental Sensor
  // ========================================
  Serial.print(F("Init BME680... "));
  bme_init_ok = (bme.begin(BME680_ADDR_1) || bme.begin(BME680_ADDR_2));
  Serial.println(bme_init_ok ? F("OK") : F("NOT FOUND"));
  
  if (bme_init_ok) {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);  // 320°C for 150ms
    expected_bme = true;
  }

  // ========================================
  // SCD41 CO2 Sensor
  // ========================================
  Serial.print(F("Init SCD41... "));
  
  // Try to initialize SCD41 with retries
  scd_init_ok = false;
  for (int attempt = 1; attempt <= 3; attempt++) {
    if (scd41.begin()) {
      // Stop any running measurements first
      scd41.stopPeriodicMeasurement();
      delay(500);
      
      // Try to read sensor info to verify communication
      char serialNumber[13];
      if (scd41.getSerialNumber(serialNumber)) {
        Serial.print(F("Found (SN: "));
        Serial.print(serialNumber);
        Serial.print(F(") "));
      }
      
      // Try to start periodic measurement
      if (scd41.startPeriodicMeasurement()) {
        scd_init_ok = true;
        Serial.println(F("OK"));
        expected_scd = true;
        break;
      } else {
        Serial.print(F("start failed (attempt "));
        Serial.print(attempt);
        Serial.println(F(")"));
      }
    } else {
      Serial.print(F("begin failed (attempt "));
      Serial.print(attempt);
      Serial.println(F(")"));
    }
    
    if (attempt < 3) {
      delay(1000);
      esp_task_wdt_reset();
    }
  }
  
  if (!scd_init_ok) {
    Serial.println(F("NOT FOUND (after 3 attempts)"));
    Serial.println(F("  Possible causes:"));
    Serial.println(F("  - Wrong I2C address (should be 0x62)"));
    Serial.println(F("  - Insufficient power (needs 3.3V, 75mA)"));
    Serial.println(F("  - Library version mismatch"));
    Serial.println(F("  - Sensor defective or fake"));
  }

  // ========================================
  // Oxygen Sensor
  // ========================================
  Serial.print(F("Init Oxygen sensor... "));
  o2_init_ok = oxygen.begin(Oxygen_IICAddress);
  Serial.println(o2_init_ok ? F("OK") : F("NOT FOUND"));
  
  if (o2_init_ok) {
    expected_o2 = true;
  }

  // ========================================
  // Analog Gas Sensors - Configuration Based
  // ========================================
  Serial.println(F("\n--- Analog Gas Sensor Configuration ---"));

  // MQ5
  mq5_init_ok = REQUIRE_MQ5_SENSOR;
  if (mq5_init_ok) {
    expected_mq5 = true;
    Serial.println(F("MQ5 (LPG): ENABLED"));
    // Set baseline - will be calibrated later
    MQ5_R0_RAW = 2000;  // Default, will be calibrated
  } else {
    Serial.println(F("MQ5 (LPG): DISABLED"));
  }

  // H2S
  h2s_init_ok = REQUIRE_H2S_SENSOR;
  if (h2s_init_ok) {
    expected_h2s = true;
    Serial.println(F("H2S: ENABLED"));
    H2S_R0_RAW = 2000;
  } else {
    Serial.println(F("H2S: DISABLED"));
  }

  // CO
  co_init_ok = REQUIRE_CO_SENSOR;
  if (co_init_ok) {
    expected_co = true;
    Serial.println(F("CO: ENABLED"));
    CO_R0_RAW = 2000;
  } else {
    Serial.println(F("CO: DISABLED"));
  }

  // CH4
  ch4_init_ok = REQUIRE_CH4_SENSOR;
  if (ch4_init_ok) {
    expected_ch4 = true;
    Serial.println(F("CH4: ENABLED"));
    CH4_R0_RAW = 2000;
  } else {
    Serial.println(F("CH4: DISABLED"));
  }

  // NH3
  nh3_init_ok = REQUIRE_NH3_SENSOR;
  if (nh3_init_ok) {
    expected_nh3 = true;
    Serial.println(F("NH3: ENABLED"));
    NH3_R0_RAW = 2000;
  } else {
    Serial.println(F("NH3: DISABLED"));
  }

  // H2
  h2_init_ok = REQUIRE_H2_SENSOR;
  if (h2_init_ok) {
    expected_h2 = true;
    Serial.println(F("H2: ENABLED"));
    H2_R0_RAW = 2000;
  } else {
    Serial.println(F("H2: DISABLED"));
  }

  // Set expected flags for I2C sensors based on config
  expected_bme = REQUIRE_BME_SENSOR;
  expected_scd = REQUIRE_SCD_SENSOR;
  expected_o2 = REQUIRE_O2_SENSOR;

  // ========================================
  // Critical Sensor Validation
  // ========================================
  esp_task_wdt_reset();
  validateCriticalSensors();

  // ========================================
  // Sensor Calibration
  // ========================================
  Serial.println(F("\n===================================="));
  Serial.println(F("   STARTING CALIBRATION SEQUENCE"));
  Serial.println(F("===================================="));
  Serial.println(F("Calibrating detected sensors only."));
  Serial.println(F("Missing sensors will be skipped."));
  Serial.println(F(""));
  Serial.println(F("IMPORTANT: Ensure device is in"));
  Serial.println(F("clean, fresh air for accurate"));
  Serial.println(F("baseline measurements."));
  Serial.println(F("====================================\n"));
  
  delay(2000);
  esp_task_wdt_reset();

  // BME680 calibration
  if (bme_init_ok) {
    calibrateBME680();
    esp_task_wdt_reset();
  }

  // Analog sensor calibration
  if (mq5_init_ok) {
    Serial.println(F("\n===================================="));
    Serial.println(F("   MQ5 CALIBRATION"));
    Serial.println(F("===================================="));
    Serial.println(F("Reading baseline..."));
    
    int sum = 0;
    for (int i = 0; i < 20; i++) {
      sum += analogRead(PIN_MQ5_AO);
      delay(100);
    }
    MQ5_R0_RAW = sum / 20;
    
    Serial.print(F("Baseline: "));
    Serial.println(MQ5_R0_RAW);
    Serial.println(F("Calibration OK"));
    Serial.println(F("====================================\n"));
    esp_task_wdt_reset();
  }

  if (h2s_init_ok) {
    calibrateAnalogSensor(PIN_H2S_AO, &H2S_R0_RAW, &H2S_SCALE, 
                         "H2S", 100.0f, MEMS_WARMUP_MS);
    esp_task_wdt_reset();
  }

  if (co_init_ok) {
    calibrateAnalogSensor(PIN_CO_AO, &CO_R0_RAW, &CO_SCALE, 
                         "CO", 10000.0f, MEMS_WARMUP_MS);
    esp_task_wdt_reset();
  }

  if (ch4_init_ok) {
    calibrateAnalogSensor(PIN_CH4_AO, &CH4_R0_RAW, &CH4_SCALE, 
                         "CH4", 50000.0f, MEMS_WARMUP_MS);
    esp_task_wdt_reset();
  }

  if (nh3_init_ok) {
    calibrateAnalogSensor(PIN_NH3_AO, &NH3_R0_RAW, &NH3_SCALE, 
                         "NH3", 500.0f, MEMS_WARMUP_MS);
    esp_task_wdt_reset();
  }

  if (h2_init_ok) {
    calibrateAnalogSensor(PIN_H2_AO, &H2_R0_RAW, &H2_SCALE, 
                         "H2", 40000.0f, MEMS_WARMUP_MS);
    esp_task_wdt_reset();
  }

  // Note: MQ5 calibration is passive (already collected baseline)
  
  Serial.println(F("\n===================================="));
  Serial.println(F("   CALIBRATION COMPLETE"));
  Serial.println(F("====================================\n"));

  Serial.println(F("System ready. Entering monitoring mode."));
  Serial.print(F("Startup grace period: "));
  Serial.print(STARTUP_GRACE_MS / 1000);
  Serial.println(F(" seconds\n"));

  if (oled_init_ok) {
    display.clearDisplay();
    display.setTextSize(1);
    
    // Yellow zone - Status
    display.setCursor(0, 6);
    display.println(F("CALIBRATION"));
    display.drawLine(0, 17, 127, 17, SSD1306_WHITE);
    
    // Blue zone - Details
    display.setCursor(0, 24);
    display.println(F("COMPLETE"));
    display.setCursor(0, 36);
    display.println(F("System Ready"));
    display.setCursor(0, 52);
    display.print(F("Warmup: "));
    display.print(STARTUP_GRACE_MS / 1000);
    display.println(F("s"));
    display.display();
    delay(2000);
  }

  esp_task_wdt_reset();
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {
  static AirState st = SAFE;
  static bool err = false;

  unsigned long now = millis();

  // Reset watchdog
  esp_task_wdt_reset();

  // Main sensor polling
  if (now - lastPollMs >= POLL_MS) {
    lastPollMs = now;

    // Poll all available sensors
    if (bme_init_ok) pollBME680();
    
    if (scd_init_ok) {
      pollSCD41_cached();
      deriveCO2_ok();
    } else {
      ok_co2 = false;
    }
    
    if (o2_init_ok) pollOxygen();
    else ok_o2 = false;
    
    if (mq5_init_ok) pollMQ5();
    else ok_mq5 = false;

    if (h2s_init_ok) pollH2S();
    else ok_h2s = false;
    
    if (co_init_ok) pollCO();
    else ok_co = false;
    
    if (ch4_init_ok) pollCH4();
    else ok_ch4 = false;
    
    if (nh3_init_ok) pollNH3();
    else ok_nh3 = false;
    
    if (h2_init_ok) pollH2();
    else ok_h2 = false;

    // Evaluate system state
    st = evaluateAir();
    err = sensorsError();

    // Serial output
    Serial.print(F("["));
    Serial.print(upTimeMs() / 1000);
    Serial.print(F("s] "));

    // Environmental data
    if (ok_bme) {
      Serial.print(F("T="));
      Serial.print(bme_tempC, 1);
      Serial.print(F("C RH="));
      Serial.print(bme_rh, 1);
      Serial.print(F("% "));
      
      if (bme_calibrated && bme_gas_reference > 0) {
        Serial.print(F("IAQ="));
        Serial.print(bme_iaq_score);
        Serial.print(F(" "));
      }
    }

    // Critical gases
    if (ok_o2) {
      Serial.print(F("O2="));
      Serial.print(o2_pct, 1);
      Serial.print(F("% "));
    }

    if (ok_co2) {
      Serial.print(F("CO2="));
      Serial.print(co2_ppm_last);
      Serial.print(F("ppm "));
    }

    if (ok_co) {
      Serial.print(F("CO="));
      Serial.print(co_ppm, 0);
      Serial.print(F("ppm "));
    }

    if (ok_h2s) {
      Serial.print(F("H2S="));
      Serial.print(h2s_ppm, 1);
      Serial.print(F("ppm "));
    }

    if (mq5_init_ok) {  // Changed from ok_mq5 to mq5_init_ok
      Serial.print(F("LPG="));
      Serial.print(mq5_raw);
      Serial.print(F(" "));
    }

    // Status
    Serial.print(F("| "));
    if (inStartupGrace()) {
      Serial.print(F("WARMUP("));
      Serial.print((STARTUP_GRACE_MS - upTimeMs()) / 1000);
      Serial.print(F("s) "));
    } else {
      Serial.print(st == SAFE ? F("SAFE") : (st == WARN ? F("WARN") : F("ALARM")));
      Serial.print(F(" "));
    }
    Serial.println(err ? F("FAULT") : F("OK"));

    // Update display
    updateDisplay();

    // Update indicators
    driveGreenLED(err);
    driveRedLED(st);
  }

  // Update buzzer (needs frequent updates for tones)
  driveBuzzer(st, err);
}
