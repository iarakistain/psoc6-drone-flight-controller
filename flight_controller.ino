#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <BMM350.h>
#include <Dps3xx.h>

// ---------- Configuration ----------
static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint32_t LOOP_PERIOD_US = 10000; // 100 Hz
static constexpr float SEA_LEVEL_PRESSURE_PA = 101325.0f;

// Board sensors
BMI270 imu;
BMM350 mag(0x14);
Dps3xx baro;

struct Vec3 {
  float x;
  float y;
  float z;
};

struct Quaternion {
  float w;
  float x;
  float y;
  float z;
};

struct SensorSample {
  Vec3 accelG;
  Vec3 gyroDps;
  Vec3 magUT;
  float pressurePa;
  float temperatureC;
  float altitudeM;
};

class MadgwickAHRS {
 public:
  explicit MadgwickAHRS(float beta = 0.08f) : beta_(beta) {}

  void update(float gx, float gy, float gz,
              float ax, float ay, float az,
              float mx, float my, float mz,
              float dt) {
    float q1 = q_.w, q2 = q_.x, q3 = q_.y, q4 = q_.z;

    const float normA = sqrtf(ax * ax + ay * ay + az * az);
    if (normA <= 1e-6f) {
      integrateGyro(gx, gy, gz, dt);
      return;
    }

    ax /= normA;
    ay /= normA;
    az /= normA;

    const float normM = sqrtf(mx * mx + my * my + mz * mz);
    if (normM <= 1e-6f) {
      integrateGyro(gx, gy, gz, dt);
      return;
    }

    mx /= normM;
    my /= normM;
    mz /= normM;

    const float _2q1mx = 2.0f * q1 * mx;
    const float _2q1my = 2.0f * q1 * my;
    const float _2q1mz = 2.0f * q1 * mz;
    const float _2q2mx = 2.0f * q2 * mx;
    const float _2q1 = 2.0f * q1;
    const float _2q2 = 2.0f * q2;
    const float _2q3 = 2.0f * q3;
    const float _2q4 = 2.0f * q4;
    const float _2q1q3 = 2.0f * q1 * q3;
    const float _2q3q4 = 2.0f * q3 * q4;
    const float q1q1 = q1 * q1;
    const float q1q2 = q1 * q2;
    const float q1q3 = q1 * q3;
    const float q1q4 = q1 * q4;
    const float q2q2 = q2 * q2;
    const float q2q3 = q2 * q3;
    const float q2q4 = q2 * q4;
    const float q3q3 = q3 * q3;
    const float q3q4 = q3 * q4;
    const float q4q4 = q4 * q4;

    const float hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    const float hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    const float _2bx = sqrtf(hx * hx + hy * hy);
    const float _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    const float _4bx = 2.0f * _2bx;
    const float _4bz = 2.0f * _2bz;

    const float s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    const float s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    const float s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    const float s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);

    float normS = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
    if (normS > 1e-6f) {
      normS = 1.0f / normS;
    }

    const float qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta_ * s1 * normS;
    const float qDot2 = 0.5f * ( q1 * gx + q3 * gz - q4 * gy) - beta_ * s2 * normS;
    const float qDot3 = 0.5f * ( q1 * gy - q2 * gz + q4 * gx) - beta_ * s3 * normS;
    const float qDot4 = 0.5f * ( q1 * gz + q2 * gy - q3 * gx) - beta_ * s4 * normS;

    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;
    q4 += qDot4 * dt;

    const float normQ = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    if (normQ > 1e-6f) {
      const float invNorm = 1.0f / normQ;
      q_.w = q1 * invNorm;
      q_.x = q2 * invNorm;
      q_.y = q3 * invNorm;
      q_.z = q4 * invNorm;
    }
  }

  Quaternion getQuaternion() const { return q_; }

 private:
  void integrateGyro(float gx, float gy, float gz, float dt) {
    float q1 = q_.w, q2 = q_.x, q3 = q_.y, q4 = q_.z;
    const float qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz);
    const float qDot2 = 0.5f * ( q1 * gx + q3 * gz - q4 * gy);
    const float qDot3 = 0.5f * ( q1 * gy - q2 * gz + q4 * gx);
    const float qDot4 = 0.5f * ( q1 * gz + q2 * gy - q3 * gx);

    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;
    q4 += qDot4 * dt;

    const float norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    if (norm > 1e-6f) {
      q_.w = q1 / norm;
      q_.x = q2 / norm;
      q_.y = q3 / norm;
      q_.z = q4 / norm;
    }
  }

  float beta_;
  Quaternion q_ {1.0f, 0.0f, 0.0f, 0.0f};
};

MadgwickAHRS ahrs;

// Hard-iron and soft-iron calibration (replace with your calibration values)
const float MAG_HARD_IRON[3] = {1.29f, 0.07f, -6.49f};
const float MAG_SOFT_IRON[3][3] = {
    {0.974f, -0.009f, -0.005f},
    {-0.009f, 0.973f, 0.009f},
    {-0.005f, 0.009f, 1.056f}
};

SensorSample sample {};
Vec3 eulerDeg {};
Quaternion attitude {};

bool debugEnabled = true;
bool lowPowerIdle = false;
bool sensorHealthy = true;
uint32_t lastLoopUs = 0;
uint32_t lastDriftReportMs = 0;
Vec3 gyroBiasDps {0.0f, 0.0f, 0.0f};

String serialLine;

static Vec3 applyMagCalibration(const Vec3& raw) {
  Vec3 centered {
      raw.x - MAG_HARD_IRON[0],
      raw.y - MAG_HARD_IRON[1],
      raw.z - MAG_HARD_IRON[2]};

  Vec3 calibrated;
  calibrated.x = MAG_SOFT_IRON[0][0] * centered.x + MAG_SOFT_IRON[0][1] * centered.y + MAG_SOFT_IRON[0][2] * centered.z;
  calibrated.y = MAG_SOFT_IRON[1][0] * centered.x + MAG_SOFT_IRON[1][1] * centered.y + MAG_SOFT_IRON[1][2] * centered.z;
  calibrated.z = MAG_SOFT_IRON[2][0] * centered.x + MAG_SOFT_IRON[2][1] * centered.y + MAG_SOFT_IRON[2][2] * centered.z;
  return calibrated;
}

static float computeAltitudeMeters(float pressurePa, float temperatureC) {
  if (pressurePa <= 0.0f) {
    return NAN;
  }
  const float tempK = temperatureC + 273.15f;
  return ((powf(SEA_LEVEL_PRESSURE_PA / pressurePa, 1.0f / 5.257f) - 1.0f) * tempK) / 0.0065f;
}

static void quaternionToEuler(const Quaternion& q, Vec3& euler) {
  const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
  const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
  euler.x = atan2f(sinr_cosp, cosr_cosp) * 180.0f / PI;  // roll

  const float sinp = 2.0f * (q.w * q.y - q.z * q.x);
  if (fabsf(sinp) >= 1.0f) {
    euler.y = copysignf(90.0f, sinp);  // pitch
  } else {
    euler.y = asinf(sinp) * 180.0f / PI;
  }

  const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
  const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
  euler.z = atan2f(siny_cosp, cosy_cosp) * 180.0f / PI;  // yaw
}

static float headingFromYaw(float yawDeg) {
  float heading = fmodf(yawDeg + 360.0f, 360.0f);
  if (heading < 0.0f) {
    heading += 360.0f;
  }
  return heading;
}

static void printStatus(const char* level, const char* message) {
  Serial.print("{\"type\":\"status\",\"data\":{\"level\":\"");
  Serial.print(level);
  Serial.print("\",\"message\":\"");
  Serial.print(message);
  Serial.print("\",\"lowPowerIdle\":");
  Serial.print(lowPowerIdle ? "true" : "false");
  Serial.print("},\"timestamp\":");
  Serial.print(millis());
  Serial.println("}");
}

static bool initializeSensors() {
  Wire.begin();

  if (imu.beginI2C(BMI2_I2C_PRIM_ADDR) != BMI2_OK) {
    printStatus("error", "BMI270 init failed");
    return false;
  }

  const int8_t imuSelfTest = imu.selfTest();
  if (imuSelfTest != BMI2_OK) {
    printStatus("warning", "BMI270 self-test reported non-zero status");
  }

  if (!mag.begin(&Wire)) {
    printStatus("error", "BMM350 init failed");
    return false;
  }
  mag.setCalibration(MAG_HARD_IRON, MAG_SOFT_IRON);

  baro.begin(Wire);
  int16_t baroInit = baro.startMeasureBothCont(5, 2, 5, 2);
  if (baroInit != 0) {
    printStatus("error", "DPS368 continuous measurement init failed");
    return false;
  }

  char sensorInfo[64];
  snprintf(sensorInfo, sizeof(sensorInfo), "Sensor check BMI addr=0x%02X BMM chip=0x%02X", BMI2_I2C_PRIM_ADDR, mag.getChipID());
  printStatus("info", sensorInfo);
  printStatus("info", "All sensors initialized");
  return true;
}

static bool readSensors(SensorSample& out) {
  const int8_t imuStatus = imu.getSensorData();
  if (imuStatus != BMI2_OK) {
    if (debugEnabled) {
      printStatus("warning", "BMI270 read failed");
    }
    return false;
  }

  out.accelG = {imu.data.accelX, imu.data.accelY, imu.data.accelZ};
  out.gyroDps = {imu.data.gyroX, imu.data.gyroY, imu.data.gyroZ};

  Vec3 magRaw {};
  if (!mag.readMagnetometerData(magRaw.x, magRaw.y, magRaw.z)) {
    if (debugEnabled) {
      printStatus("warning", "BMM350 read failed");
    }
    return false;
  }
  out.magUT = applyMagCalibration(magRaw);

  float temperatureBuf[4] = {};
  float pressureBuf[4] = {};
  uint8_t temperatureCount = 4;
  uint8_t pressureCount = 4;
  const int16_t dpsStatus = baro.getContResults(temperatureBuf, temperatureCount, pressureBuf, pressureCount);
  if (dpsStatus == 0 && temperatureCount > 0 && pressureCount > 0) {
    out.temperatureC = temperatureBuf[temperatureCount - 1];
    out.pressurePa = pressureBuf[pressureCount - 1];
  } else if (baro.measureTempOnce(out.temperatureC) != 0 || baro.measurePressureOnce(out.pressurePa) != 0) {
    if (debugEnabled) {
      printStatus("warning", "DPS368 read failed");
    }
    return false;
  }

  out.altitudeM = computeAltitudeMeters(out.pressurePa, out.temperatureC);
  return true;
}

static void detectDrift(const SensorSample& s) {
  const float accelNorm = sqrtf(s.accelG.x * s.accelG.x + s.accelG.y * s.accelG.y + s.accelG.z * s.accelG.z);
  if (fabsf(accelNorm - 1.0f) < 0.08f) {
    gyroBiasDps.x = 0.995f * gyroBiasDps.x + 0.005f * s.gyroDps.x;
    gyroBiasDps.y = 0.995f * gyroBiasDps.y + 0.005f * s.gyroDps.y;
    gyroBiasDps.z = 0.995f * gyroBiasDps.z + 0.005f * s.gyroDps.z;
  }

  const float driftMag = sqrtf(gyroBiasDps.x * gyroBiasDps.x + gyroBiasDps.y * gyroBiasDps.y + gyroBiasDps.z * gyroBiasDps.z);
  if (driftMag > 1.5f && millis() - lastDriftReportMs > 1000) {
    lastDriftReportMs = millis();
    printStatus("warning", "Gyro drift detected");
  }
}

static void processSerialCommands() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\n' || ch == '\r') {
      if (serialLine == "IDLE:1") {
        lowPowerIdle = true;
        imu.enableAdvancedPowerSave(true);
        mag.setPowerMode(BMM350_SUSPEND_MODE);
        printStatus("info", "Entered low power idle mode");
      } else if (serialLine == "IDLE:0") {
        lowPowerIdle = false;
        imu.enableAdvancedPowerSave(false);
        mag.setPowerMode(BMM350_NORMAL_MODE);
        printStatus("info", "Exited low power idle mode");
      } else if (serialLine == "DEBUG:1") {
        debugEnabled = true;
        printStatus("info", "Debug enabled");
      } else if (serialLine == "DEBUG:0") {
        debugEnabled = false;
        printStatus("info", "Debug disabled");
      }
      serialLine = "";
    } else if (serialLine.length() < 48) {
      serialLine += ch;
    }
  }
}

static void streamSensorJson(const SensorSample& s) {
  Serial.print("{\"type\":\"sensor\",\"data\":{");
  Serial.print("\"accel\":{\"x\":"); Serial.print(s.accelG.x, 5);
  Serial.print(",\"y\":"); Serial.print(s.accelG.y, 5);
  Serial.print(",\"z\":"); Serial.print(s.accelG.z, 5);
  Serial.print("},\"gyro\":{\"x\":"); Serial.print(s.gyroDps.x, 5);
  Serial.print(",\"y\":"); Serial.print(s.gyroDps.y, 5);
  Serial.print(",\"z\":"); Serial.print(s.gyroDps.z, 5);
  Serial.print("},\"mag\":{\"x\":"); Serial.print(s.magUT.x, 5);
  Serial.print(",\"y\":"); Serial.print(s.magUT.y, 5);
  Serial.print(",\"z\":"); Serial.print(s.magUT.z, 5);
  Serial.print("},\"baro\":{\"pressurePa\":"); Serial.print(s.pressurePa, 2);
  Serial.print(",\"temperatureC\":"); Serial.print(s.temperatureC, 2);
  Serial.print(",\"altitudeM\":"); Serial.print(s.altitudeM, 2);
  Serial.print("}},\"timestamp\":");
  Serial.print(millis());
  Serial.println("}");
}

static void streamAhrsJson(const Quaternion& q, const Vec3& euler) {
  Serial.print("{\"type\":\"ahrs\",\"data\":{");
  Serial.print("\"roll\":"); Serial.print(euler.x, 3);
  Serial.print(",\"pitch\":"); Serial.print(euler.y, 3);
  Serial.print(",\"yaw\":"); Serial.print(euler.z, 3);
  Serial.print(",\"heading\":"); Serial.print(headingFromYaw(euler.z), 3);
  Serial.print(",\"quaternion\":{\"w\":"); Serial.print(q.w, 6);
  Serial.print(",\"x\":"); Serial.print(q.x, 6);
  Serial.print(",\"y\":"); Serial.print(q.y, 6);
  Serial.print(",\"z\":"); Serial.print(q.z, 6);
  Serial.print("}},\"timestamp\":");
  Serial.print(millis());
  Serial.println("}");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    delay(10);
  }

  printStatus("info", "Booting flight controller");
  sensorHealthy = initializeSensors();
  lastLoopUs = micros();
}

void loop() {
  processSerialCommands();

  if (lowPowerIdle) {
    delay(50);
    return;
  }

  const uint32_t nowUs = micros();
  if (nowUs - lastLoopUs < LOOP_PERIOD_US) {
    return;
  }
  const float dt = (nowUs - lastLoopUs) * 1e-6f;
  lastLoopUs = nowUs;

  if (!sensorHealthy) {
    static uint32_t lastRetryMs = 0;
    if (millis() - lastRetryMs > 1000) {
      lastRetryMs = millis();
      sensorHealthy = initializeSensors();
    }
    return;
  }

  if (!readSensors(sample)) {
    sensorHealthy = false;
    printStatus("error", "Sensor read failed, entering recovery mode");
    return;
  }

  detectDrift(sample);

  ahrs.update(
      radians(sample.gyroDps.x), radians(sample.gyroDps.y), radians(sample.gyroDps.z),
      sample.accelG.x, sample.accelG.y, sample.accelG.z,
      sample.magUT.x, sample.magUT.y, sample.magUT.z,
      dt);

  attitude = ahrs.getQuaternion();
  quaternionToEuler(attitude, eulerDeg);

  streamSensorJson(sample);
  streamAhrsJson(attitude, eulerDeg);
}
