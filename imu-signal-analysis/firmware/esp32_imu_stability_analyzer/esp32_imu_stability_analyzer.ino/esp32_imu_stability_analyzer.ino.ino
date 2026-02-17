#include <MPU6050_tockn.h>
#include <Wire.h>
#include <math.h>

// === YOUR WORKING I2C PINS ===
#define SDA 13
#define SCL 14

MPU6050 mpu6050(Wire);

int16_t ax, ay, az;
int16_t gx, gy, gz;

// ===== Real-Time Sampling Setup =====
const uint32_t SAMPLE_HZ = 100;
const uint32_t PERIOD_US = 1000000UL / SAMPLE_HZ; // 10 ms = 100Hz

uint32_t lastSampleUs = 0;
uint32_t dtUs = 0;
uint32_t sampleCount = 0;

// ===== Moving Average Filter (on accel magnitude) =====
#define FILTER_SIZE 10           // 10 samples @100Hz = 100ms smoothing
float magBuf[FILTER_SIZE] = {0};
uint8_t magIdx = 0;
float magSum = 0.0f;
bool magPrimed = false;

// ===== Timing Stability Metrics (Step 5) =====
uint32_t minDtUs = 1000000;
uint32_t maxDtUs = 0;
uint64_t sumDtUs = 0;
uint32_t dtCount = 0;
uint32_t missed = 0;

uint32_t lastReportMs = 0;

void setup() {
  Serial.begin(115200);

  // Attach I2C
  Wire.begin(SDA, SCL);
  Wire.setClock(100000);

  // Initialize MPU6050
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  lastSampleUs = micros();
  lastReportMs = millis();

  // CSV header (optional)
  Serial.println("t_us,dt_us,ax_g,ay_g,az_g,a_mag,a_mag_filt");
}

void loop() {
  uint32_t nowUs = micros();

  // === Fixed-rate loop (100Hz) ===
  if ((uint32_t)(nowUs - lastSampleUs) < PERIOD_US) return;

  dtUs = nowUs - lastSampleUs;
  lastSampleUs = nowUs;

  // ----- Step 5: timing stats -----
  if (dtUs < minDtUs) minDtUs = dtUs;
  if (dtUs > maxDtUs) maxDtUs = dtUs;
  sumDtUs += dtUs;
  dtCount++;
  if (dtUs > (PERIOD_US * 2)) missed++;

  // Update sensor
  mpu6050.update();
  getMotion6();

  // Convert raw accel to g
  float ax_g = (float)ax / 16384.0f;
  float ay_g = (float)ay / 16384.0f;
  float az_g = (float)az / 16384.0f;

  // Accel magnitude (g)
  float a_mag = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

  // ---- Moving average filter on a_mag ----
  if (!magPrimed) {
    // Prime buffer on first run so filtered output isn't weird at start
    for (int i = 0; i < FILTER_SIZE; i++) magBuf[i] = a_mag;
    magSum = a_mag * FILTER_SIZE;
    magPrimed = true;
  } else {
    magSum -= magBuf[magIdx];
    magBuf[magIdx] = a_mag;
    magSum += magBuf[magIdx];

    magIdx++;
    if (magIdx >= FILTER_SIZE) magIdx = 0;
  }

  float a_mag_filt = magSum / FILTER_SIZE;

  // Print CSV at 10Hz so serial doesn't distort timing
  sampleCount++;
  if (sampleCount % 10 == 0) {
    Serial.print(nowUs); Serial.print(",");
    Serial.print(dtUs); Serial.print(",");
    Serial.print(ax_g, 3); Serial.print(",");
    Serial.print(ay_g, 3); Serial.print(",");
    Serial.print(az_g, 3); Serial.print(",");
    Serial.print(a_mag, 3); Serial.print(",");
    Serial.println(a_mag_filt, 3);
  }

  // Print jitter summary every 5 seconds (separate from CSV)
  uint32_t nowMs = millis();
  if (nowMs - lastReportMs >= 5000) {
    lastReportMs = nowMs;

    uint32_t avgDtUs = (dtCount > 0) ? (uint32_t)(sumDtUs / dtCount) : 0;

    Serial.print("JITTER(us) min=");
    Serial.print(minDtUs);
    Serial.print(" max=");
    Serial.print(maxDtUs);
    Serial.print(" avg=");
    Serial.print(avgDtUs);
    Serial.print(" missed=");
    Serial.println(missed);

    // reset stats window
    minDtUs = 1000000;
    maxDtUs = 0;
    sumDtUs = 0;
    dtCount = 0;
    missed = 0;
  }
}

void getMotion6(void) {
  ax = mpu6050.getRawAccX();
  ay = mpu6050.getRawAccY();
  az = mpu6050.getRawAccZ();

  gx = mpu6050.getRawGyroX();
  gy = mpu6050.getRawGyroY();
  gz = mpu6050.getRawGyroZ();
}
