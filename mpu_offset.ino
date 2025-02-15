#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
int16_t gx, gy, gz;
float gyroZ_offset = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }

    Serial.println("Calibrating Gyro... Keep the MPU6050 steady.");
    
    long sum = 0;
    int samples = 1000;  // Take 1000 readings for calibration

    for (int i = 0; i < samples; i++) {
        mpu.getRotation(&gx, &gy, &gz);
        sum += gz;  // Only using Z-axis for rotation
        delay(3);
    }

    gyroZ_offset = sum / samples;  // Compute the average offset
    Serial.print("Gyro Z-axis Offset: ");
    Serial.println(gyroZ_offset);
}

void loop() {
    mpu.getRotation(&gx, &gy, &gz);
    float gyroZ_corrected = (gz - gyroZ_offset) / 131.0;  // Apply offset correction

    Serial.print("Gyro Z (Corrected): ");
    Serial.println(gyroZ_corrected);
    delay(100);
}
