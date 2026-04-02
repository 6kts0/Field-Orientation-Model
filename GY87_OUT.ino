/*
 *  Reads the GY-87 IMU module and outputs roll, pitch, yaw as CSV over
 *  serial for a browser-based 3D aircraft visualizer.
 * ============================================================================
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <QMC5883LCompass.h>
#include <math.h>

// ---- Objects ----
Adafruit_MPU6050 mpu;
QMC5883LCompass compass;

// ---- Attitude State ----
float roll  = 0.0f;
float pitch = 0.0f;
float yaw   = 0.0f;

// ---- Gyro Calibration ----
float gyro_offset_x = 0.0f;
float gyro_offset_y = 0.0f;
float gyro_offset_z = 0.0f;

// ---- Timing ----
unsigned long last_micros = 0;
float dt = 0.001f;

// ---- Filter Tuning ----
#define COMP_ALPHA 0.98f         // Complementary filter: gyro trust
#define GYRO_CAL_SAMPLES 1500
#define LOOP_RATE_US 2000        // 500 Hz target loop

// ==== SETUP  ====

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Wire.begin();
    Wire.setClock(400000);  // 400 kHz fast I2C

    // ---- Initialize MPU6050 ----
    Serial.println("# Initializing MPU6050...");
    if (!mpu.begin()) {
        Serial.println("# ERROR: MPU6050 not found! Check wiring.");
        while (1) { delay(500); }
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    Serial.println("# MPU6050 OK");

    // ---- Enable I2C Bypass for QMC5883L ----
    // The QMC5883L sits behind the MPU6050's auxiliary I2C bus.
    // We must enable bypass mode so the Arduino can talk to it directly.
    Wire.beginTransmission(0x68);
    Wire.write(0x6A);  // USER_CTRL register
    Wire.write(0x00);  // Disable I2C master mode
    Wire.endTransmission(true);

    Wire.beginTransmission(0x68);
    Wire.write(0x37);  // INT_PIN_CFG register
    Wire.write(0x02);  // Enable I2C bypass
    Wire.endTransmission(true);
    delay(10);

    // ---- Initialize QMC5883L Magnetometer ----
    Serial.println("# Initializing QMC5883L magnetometer...");
    compass.init();
    // NOTE: For best results, run the calibration sketch first!
    // Then paste your calibration offsets here:
    // compass.setCalibrationOffsets(...);
    // compass.setCalibrationScales(...);
    Serial.println("# QMC5883L OK (calibrate for best yaw accuracy)");

    // ---- Calibrate Gyroscope ----
    Serial.println("# Calibrating gyro - keep board STILL...");
    calibrateGyro();
    Serial.println("# Gyro calibration done");

    // ---- Ready ----
    last_micros = micros();
    Serial.println("# READY - streaming attitude data");
    Serial.println("# Format: R:roll,P:pitch,Y:yaw,AX:ax,AY:ay,AZ:az");
}


// ==== MAIN LOOP ====

void loop() {
    // ---- Timing ----
    unsigned long now = micros();
    dt = (float)(now - last_micros) / 1000000.0f;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.002f;
    last_micros = now;

    // ---- Read MPU6050 ----
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Gyro in deg/s (Adafruit library returns rad/s, convert)
    float gx = (gyro.gyro.x * 57.2958f) - gyro_offset_x;
    float gy = (gyro.gyro.y * 57.2958f) - gyro_offset_y;
    float gz = (gyro.gyro.z * 57.2958f) - gyro_offset_z;

    // Accel in m/s^2
    float ax = accel.acceleration.x;
    float ay = accel.acceleration.y;
    float az = accel.acceleration.z;

    // ---- Accelerometer Angles ----
    float accel_roll  = atan2f(ay, sqrtf(ax * ax + az * az)) * 57.2958f;
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2958f;

    // ---- Complementary Filter (Roll & Pitch) ----
    roll  = COMP_ALPHA * (roll  + gx * dt) + (1.0f - COMP_ALPHA) * accel_roll;
    pitch = COMP_ALPHA * (pitch + gy * dt) + (1.0f - COMP_ALPHA) * accel_pitch;

    // ---- Magnetometer Yaw (drift-free heading) ----
    compass.read();
    float mx = (float)compass.getX();
    float my = (float)compass.getY();
    float mz = (float)compass.getZ();

    // Tilt-compensated heading
    float roll_rad  = roll  * 0.0174533f;
    float pitch_rad = pitch * 0.0174533f;

    float mx_comp = mx * cosf(pitch_rad) + mz * sinf(pitch_rad);
    float my_comp = mx * sinf(roll_rad) * sinf(pitch_rad)
                  + my * cosf(roll_rad)
                  - mz * sinf(roll_rad) * cosf(pitch_rad);

    float mag_yaw = atan2f(-my_comp, mx_comp) * 57.2958f;

    // Blend magnetometer yaw with gyro yaw for smoothness
    // Use a lighter alpha for yaw since mag provides absolute reference
    float gyro_yaw = yaw + gz * dt;
    yaw = 0.90f * gyro_yaw + 0.10f * mag_yaw;

    // Normalize yaw to -180..+180
    if (yaw > 180.0f)  yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;

    // ---- Output CSV ----
    Serial.print("R:");  Serial.print(roll, 2);
    Serial.print(",P:"); Serial.print(pitch, 2);
    Serial.print(",Y:"); Serial.print(yaw, 2);
    Serial.print(",AX:"); Serial.print(ax, 2);
    Serial.print(",AY:"); Serial.print(ay, 2);
    Serial.print(",AZ:"); Serial.print(az, 2);
    Serial.println();

    // ---- Loop rate control ----
    unsigned long elapsed = micros() - now;
    if (elapsed < LOOP_RATE_US) {
        delayMicroseconds(LOOP_RATE_US - elapsed);
    }
}


//  ==== GYRO CALIBRATION ====
void calibrateGyro() {
    float sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        sensors_event_t a, g, t;
        mpu.getEvent(&a, &g, &t);
        sx += g.gyro.x * 57.2958f;
        sy += g.gyro.y * 57.2958f;
        sz += g.gyro.z * 57.2958f;
        delayMicroseconds(600);
    }
    gyro_offset_x = sx / GYRO_CAL_SAMPLES;
    gyro_offset_y = sy / GYRO_CAL_SAMPLES;
    gyro_offset_z = sz / GYRO_CAL_SAMPLES;
    Serial.print("# Offsets: ");
    Serial.print(gyro_offset_x, 3); Serial.print(", ");
    Serial.print(gyro_offset_y, 3); Serial.print(", ");
    Serial.println(gyro_offset_z, 3);
}