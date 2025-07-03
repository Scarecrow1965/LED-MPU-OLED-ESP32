// +-------------------------------------------------------------
//
// Equipment:
// DOIT ESP32 Dev Kit V1, MPU-6050, OLED SSD1306, WS2812B Addressable LEDS x3,
// 4-relay module, 2 linear actuators, RT-11(JCHT35K9) remote/handset,
// and 650W 12V Power supply
//
// File: IMU_Error.h
//
// Description:
//
// Provides the inital zero function for the
// MPU-6050, and the linear actuators to control the movement of the standup
// portion of the workstation.
//
// History:     2-Mar-2024     Scarecrow1965   Created
//
// +-------------------------------------------------------------

#ifndef IMU_ERROR_H
#define IMU_ERROR_H

#include <Arduino.h>
#include <Wire.h>

// #include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <I2Cdev.h>
#include <MPU6050.h>

// const int MPU = 0x68; // MPU6050 I2C address
const int MPU6050_addr = 0x68; // to initialize the address of the MPU-6050
extern float accX, accY, accZ;
extern float gyroX, gyroY, gyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

void calculate_IMU_error(void)
{
    // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
    // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
    // Read accelerometer values 200 times
    while (c < 200)
    {
        Wire.beginTransmission(MPU6050_addr);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        // Wire.requestFrom(MPU6050_addr, 6, true);
        // was getting a C++ warning: ISO C++ says that these are ambiguous, even though the worst conversion for the first is better than the worst conversion for the second
        Wire.requestFrom(MPU6050_addr, 6);
        accX = (Wire.read() << 8 | Wire.read()) / 16384.0;
        accY = (Wire.read() << 8 | Wire.read()) / 16384.0;
        accZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
        // Sum all readings
        AccErrorX = AccErrorX + ((atan((accY) / sqrt(pow((accX), 2) + pow((accZ), 2))) * 180 / PI));
        AccErrorY = AccErrorY + ((atan(-1 * (accX) / sqrt(pow((accY), 2) + pow((accZ), 2))) * 180 / PI));
        c++;
    }
    // Divide the sum by 200 to get the error value
    AccErrorX = AccErrorX / 200;
    AccErrorY = AccErrorY / 200;
    c = 0;
    // Read gyro values 200 times
    while (c < 200)
    {
        Wire.beginTransmission(MPU6050_addr);
        Wire.write(0x43);
        Wire.endTransmission(false);
        // Wire.requestFrom(MPU6050_addr, 6, true);
        Wire.requestFrom(MPU6050_addr, 6);
        gyroX = Wire.read() << 8 | Wire.read();
        gyroY = Wire.read() << 8 | Wire.read();
        gyroZ = Wire.read() << 8 | Wire.read();
        // Sum all readings
        GyroErrorX = GyroErrorX + (gyroX / 131.0);
        GyroErrorY = GyroErrorY + (gyroY / 131.0);
        GyroErrorZ = GyroErrorZ + (gyroZ / 131.0);
        c++;
    }
    // Divide the sum by 200 to get the error value
    GyroErrorX = GyroErrorX / 200;
    GyroErrorY = GyroErrorY / 200;
    GyroErrorZ = GyroErrorZ / 200;
    // Print the error values on the Serial Monitor
    Serial.print("AccErrorX: ");
    Serial.println(AccErrorX);
    Serial.print("AccErrorY: ");
    Serial.println(AccErrorY);
    Serial.print("GyroErrorX: ");
    Serial.println(GyroErrorX);
    Serial.print("GyroErrorY: ");
    Serial.println(GyroErrorY);
    Serial.print("GyroErrorZ: ");
    Serial.println(GyroErrorZ);
    // response was:
    // AccErrorX: 0.04 AccErrorY : -0.24 GyroErrorX : 2.87 GyroErrorY : 0.62 GyroErrorZ : 0.19
    // when table is down to floor:
    // AccErrorX: 9.19 AccErrorY: -0.24 GyroErrorX : 2.88 GyroErrorY : 0.62 GyroErrorZ : 0.20
    // AccErrorX: 13.32 AccErrorY : -0.23 GyroErrorX : 2.88 GyroErrorY : 0.63 GyroErrorZ : 0.20

}; // END OF calculate_IMU_error()

// void setup()
// {
//     Serial.begin(19200);
//     Wire.begin();                // Initialize comunication
//     Wire.beginTransmission(MPU6050_addr); // Start communication with MPU6050 // MPU=0x68
//     Wire.write(0x6B);            // Talk to the register 6B
//     Wire.write(0x00);            // Make reset - place a 0 into the 6B register
//     Wire.endTransmission(true);  // end the transmission
//     /*
//     // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
//     Wire.beginTransmission(MPU);
//     Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
//     Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
//     Wire.endTransmission(true);
//     // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
//     Wire.beginTransmission(MPU);
//     Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
//     Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
//     Wire.endTransmission(true);
//     delay(20);
//     */
//     // Call this function if you need to get the IMU error values for your module
//     calculate_IMU_error();
//     delay(20);
// }

void IMUCommunications(void)
{
    // === Read acceleromter data === //
    Wire.beginTransmission(MPU6050_addr);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    // Wire.requestFrom(MPU6050_addr, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    Wire.requestFrom(MPU6050_addr, 6);
    // For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    accX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
    accY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
    accZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
    // Calculating Roll and Pitch from the accelerometer data
    accAngleX = (atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI) - 0.58;      // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
    accAngleY = (atan(-1 * accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)

    // === Read gyroscope data === //
    previousTime = currentTime;                        // Previous time is stored before the actual time read
    currentTime = millis();                            // Current time actual time read
    elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
    Wire.beginTransmission(MPU6050_addr);
    Wire.write(0x43); // Gyro data first register address 0x43
    Wire.endTransmission(false);
    // Wire.requestFrom(MPU6050_addr, 6, true);          // Read 4 registers total, each axis value is stored in 2 registers
    Wire.requestFrom(MPU6050_addr, 6);
    gyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    gyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    gyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
    // Correct the outputs with the calculated error values
    gyroX = gyroX + 0.56; // GyroErrorX ~(-0.56)
    gyroY = gyroY - 2;    // GyroErrorY ~(2)
    gyroZ = gyroZ + 0.79; // GyroErrorZ ~ (-0.8)

    // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
    gyroAngleX = gyroAngleX + gyroX * elapsedTime; // deg/s * s = deg
    gyroAngleY = gyroAngleY + gyroY * elapsedTime;
    yaw = yaw + gyroZ * elapsedTime;

    // Complementary filter - combine acceleromter and gyro angle values
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

    // Print the values on the serial monitor
    Serial.println("from IMUCommunications()");
    Serial.println("roll / pitch / yaw");
    Serial.print(roll);
    Serial.print("/");
    Serial.print(pitch);
    Serial.print("/");
    Serial.println(yaw);
    // from stationary position: 253.31/-101.09/74.51
    // from moving position: 480.66/-194.66/146.39S
    // 645.62/-262.29/197.40
    // 831.06/-334.85/251.96
    // 1180.92/-471.94/357.08
    
}; // END OF IMUCommunications()

#endif // 

// =========================================================
// END OF PROGRAM
// =========================================================