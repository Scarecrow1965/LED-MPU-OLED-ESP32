// +-------------------------------------------------------------
//
// Equipment:
// DOIT ESP32 Dev Kit V1, MPU-6050, OLED SSD1306, WS2812B Addressable LEDS x3,
// 4-relay module, 2 linear actuators, RT-11(JCHT35K9) remote/handset,
// and 650W 12V Power supply
//
// File: standupTableMovement.h
//
// Description:
//
// Provides the computer workstation the ability to use the remote/handset,
// MPU-6050, and the linear actuators to control the movement of the standup
// portion of the workstation.
//
// History:     8-Dec-2023     Scarecrow1965   Created
//
// +-------------------------------------------------------------

#ifndef STANDUPTABLEMOVEMENT_H
#define STANDUPTABLEMOVEMENT_H

// ================
// install ibraries
// ================
#include <Arduino.h>
#include <Wire.h>
// #include <SPI.h>
// for MPU-6050
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "IMU_Zero.h"
#include "IMU_Error.h"
// for OLED screen
// #include <Adafruit_SSD1306.h>
// #include <Adafruit_GFX.h>
// #include <U8g2lib.h>

// for addressable LED strips
#include "LEDeffects.h"

static char lastCommand = 0;

// legend: ..Point1 = left, ..Point2 = right
const int middlePoint1 = 143;
const int orangePoint1 = 149;
const int redPoint1 = 154;
const int tiltFwdRedPoint1 = 165;
const int tiltFwdOrangePoint1 = 170;
const int endPoint1 = 175;
const int tiltBckOrangePoint1 = 180;
const int tiltBckRedPoint1 = 185;
const int middlePoint2 = 142;
const int orangePoint2 = 137;
const int redPoint2 = 132;
const int tiltFwdRedPoint2 = 120;
const int tiltFwdOrangePoint2 = 115;
const int endPoint2 = 110;
const int tiltBckOrangePoint2 = 105;
const int tiltBckRedPoint2 = 100;

// for the OLED Screen
#include "seeOLED.h"
void displayAllGyro2(); // Declare the missing function

// extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
// extern Adafruit_SSD1306 display;
extern bool dataChanged; // Variable to store data change status
// extern float fps;        // Variable to store the frame rate

// ======================
// setup for the MPU-6050
// ======================
#define INTERRUPT_PIN 4        // interrupt connected to MPU-6050 // GPIO 4 on the ESP32
// const int MPU6050_addr = 0x68; // to initialize the address of the MPU-6050
// const int MPU6050_addr = 0x3c;// #define SCREEN_I2C_ADDR 0x3C = normally would be the OLED screen address
JSONVar readings; // Json Variable to Hold Sensor Readings
Adafruit_MPU6050 mpu;       // to create an object of the MPU-6050 using the Adafruit_MPU6050 library
MPU6050 mpu1;               // to create an object of the MPU-6050 using the I2Cdev library
sensors_event_t a, g, temp; // to create objects for storing the sensor readings

float gyroX, gyroY, gyroZ; // gyro sensor readings
float accX, accY, accZ;    // accelerometer sensor readings
float temperature;         // temperature sensor readings

// float accelBias[3] = {0, 0, 0}; // to store the accelerometer bias // used for calibration and website
// Define thresholds for change detection
const float gyroThreshold = 2.3;  // Original: 10.0 -> 7.3 // Adjust the threshold as needed for a change in the MPU movement
const float accelThreshold = 1.3; // Original: 10.0 -> 7.3 // Adjust the threshold as needed for a change in the MPU movement
// Gyroscope sensor deviation
float gyroXerror = 0.07; // original float gyroXerror = 0.07; // modified, was 0.35 but recently changed to .045
float gyroYerror = 0.03; // original float gyroYerror = 0.03; // modified to 0.15
float gyroZerror = 0.01; // original float gyroZerror = 0.01; // modified to 0.05

int16_t previousGyroX = 0, previousGyroY = 0, previousAccX = 0, previousAccY = 0; // to help with providing an interrupt
long acc_total_vector; // to store the total accelerometer vector
boolean set_gyro_angles = false;
float angle_roll_acc, angle_pitch_acc;       // to store the accelerometer angles
float angle_pitch, angle_roll;               // to store the gyro angles
int angle_pitch_buffer, angle_roll_buffer;   // to store the gyro buffer angles
float angle_pitch_output, angle_roll_output; // to store the gyro output angles

// SETUP TIMERS AND TEMP VARIABLES
long loop_timer;
unsigned long start = 0;

// function to set up the MPU-6050 for
// void IRAM_ATTR isr()
// {
//     // ISR code here
// }
// ====================

// ===================================================
// Setup for relay Module and linear actuator commands
// ===================================================
// Pin assignments for relays controlling linear actuators
// Legend: LA = Linear Actuator, LHP = Left Actuator Positive, LHM = Left Actuator Negative, RHP = Right Actuator Positive, RHM = Right Actuator Negative
const int relay1_LA1LHP_Pin = 27; // Relay controlling actuator #1 positive // it is actually pin 6 on the DOIT ESP32 Dev Kit V1 or GPIO27
const int relay2_LA1LHM_Pin = 26; // Relay controlling actuator #1 negative // it is actually pin 7 on the DOIT ESP32 Dev Kit V1 or GPIO26
const int relay3_LA2RHP_Pin = 25; // Relay controlling actuator #2 positive // it is actually pin 8 on the DOIT ESP32 Dev Kit V1 or GPIO25
const int relay4_LA2RHM_Pin = 33; // Relay controlling actuator #2 negative // it is actually pin 9 on the DOIT ESP32 Dev Kit V1 or GPIO33

// Threshold values for detecting motion
const int threshold = 5; // Adjust this value according to your needs
// const int threshold = 3; // Adjust this value according to your needs

// Variables to store current and previous position
int currentPosition = 0;
int previousPosition = 0;
int initialPosition = 0;
// Determine the position based on accelerometer data
int newPosition = currentPosition;

// Variables to control linear actuator movement
// Define flags to track the state of each actuator
bool isMovingUpLeft = false;
bool isMovingDownLeft = false;
bool isMovingUpRight = false;
bool isMovingDownRight = false;
bool isStopMovementLeft = false;
bool isStopMovementRight = false;
bool isStopMovement = false;
bool isMovingUp = false;
bool isMovingDown = false;
bool isCommandBeingProcessed;
// Variable to indicate if the table is level
bool isTableLevel = false;

// to ensure the linear actuators are moving
const int BASE_SPEED = 100;          // Base speed for linear actuator movement = 100%
const int MAX_SPEED_ADJUSTMENT = 15; // Maximum speed adjustment for linear actuator movement = 15%

// ===================================================
// Setup for RT-11 Remote
// ===================================================
// Define the pins for RT-11 remote
// const int memoryButtonPin = 1; // Pin for "Memory" button
// const int dataDisplayPin = 2;  // Pin for data display
// const int groundPin = 3;       // Ground pin
// const int txPin = 4;           // Pin for TX
// const int vccPin = 5;          // Pin for VCC
// const int memoryRecallPin = 6; // Pin for "Memory Recall" button
// const int upPin = 7;           // Pin for "Up" button
// const int downPin = 8;         // Pin for "Down" button

// ===================================================
// FUNCTIONS FOR THE MPU-6050
// ===================================================
// Initialize MPU6050
void initMPU(void)
{
    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip using Adafruit_MPU6050 library!");
        while (1)
        {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found with Adafruit_MPU6050 Library!");

    // default settings:
    // MPU6050_RANGE_2_G = 0b00, ///< +/- 2g (default value)
    // MPU6050_RANGE_250_DEG, ///< +/- 250 deg/s (default value)
    mpu.setCycleRate(MPU6050_CYCLE_40_HZ);
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    // mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    // mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    // IMUsetup(); // this function is to calculate what it would take ot zero-ize the IMU
    // got the information on 3-Mar-2024 and 4-Mar-2024
    // first pass was done when table was up:
    //      XAccel	                YAccel	                    ZAccel                      	XGyro               	YGyro           	ZGyro
    // [-511,-510] --> [-8,7]	[-4229,-4228] --> [-3,17]	[5460,5460] --> [16375,16387]	[-189,-188] --> [0,3]	[-40,-40] --> [0,1]	[-13,-12] --> [0,3]
    // second pass was done when table was down:
    //      XAccel	                YAccel	                    ZAccel                      	XGyro               	YGyro           	ZGyro
    // [-527,-526] --> [-9,9]	[-4183,-4182] --> [-4,14]	[5473,5474] --> [16384,16401]	[-190,-189] --> [-2,2]	[-41,-40] --> [0,2]	[-14,-13] --> [0,3]
    // this function comes from the IMU_Zero.h file

    Serial.println("MPU6050 Initialized!");
};

// Calibrate MPU6050
void imuOffset(void)
{
    // Call this function if you need to get the IMU error values for your module
    calculate_IMU_error();

    // this is using the MPU6050.h library
    // setting up gyro offset at the startup from IMU_Zero and MPU-6050 Calibration check, variables to be entered as default
    // IMU ZERO reports:            calibration checks reports:
    // int16_t ax_offset = -406;    int16_t ax_offset = -405;
    // int16_t ay_offset = -4151;   int16_t ay_offset = -4151;
    // int16_t az_offset = 5491;    int16_t az_offset = 5495;
    // int16_t gx_offset = -146;    int16_t gx_offset = -147;
    // int16_t gy_offset = -14;     int16_t gy_offset = -13;
    // int16_t gz_offset = 73;      int16_t gz_offset = 73;
    // I use:
    int16_t ax_offset = -527;
    int16_t ay_offset = -4183;
    int16_t az_offset = 5473; // adjusted manually to reflect closer to zero reading
    int16_t gx_offset = -189; // adjusted manually to reflect closer to zero reading
    int16_t gy_offset = -41;
    int16_t gz_offset = -14;
    mpu1.setXAccelOffset(ax_offset);
    mpu1.setYAccelOffset(ay_offset);
    mpu1.setZAccelOffset(az_offset);
    mpu1.setXGyroOffset(gx_offset);
    mpu1.setYGyroOffset(gy_offset);
    mpu1.setZGyroOffset(gz_offset);

    // Calibrate accelerometer
    // for (int i = 0; i < 1000; i++)
    // {
    //     sensors_event_t a, g, temp;
    //     mpu.getEvent(&a, &g, &temp);
    // 
    //     // Serial.print("Temperature: "); // used for testing purposes only
    //     // Serial.print(temp.temperature); // used for testing purposes only
    //     // Serial.println(" degC"); // used for testing purposes only
    // 
    //     // data being written into an array
    //     accelBias[0] += a.acceleration.x;
    //     accelBias[1] += a.acceleration.y;
    //     accelBias[2] += a.acceleration.z - (float)9.81; // Subtract gravity
    // 
    //     delay(20);
    // }
    // accelBias[0] /= 1000;
    // accelBias[1] /= 1000;
    // accelBias[2] /= 1000;
    Serial.println("MPU6050 Calibrated!");
};

// getting Gyro information
String getGyroReadings(void)
{
    mpu.getEvent(&a, &g, &temp);

    float gyroX_temp = g.gyro.x;
    if (abs(gyroX_temp) > gyroXerror)
    {
        gyroX += gyroX_temp / 50.00;
    }

    float gyroY_temp = g.gyro.y;
    if (abs(gyroY_temp) > gyroYerror)
    {
        gyroY += gyroY_temp / 70.00;
    }

    float gyroZ_temp = g.gyro.z;
    if (abs(gyroZ_temp) > gyroZerror)
    {
        gyroZ += gyroZ_temp / 90.00;
    }

    // the following code lines are used to enter those readings into the JSON object
    readings["gyroX"] = String(gyroX);
    readings["gyroY"] = String(gyroY);
    readings["gyroZ"] = String(gyroZ);
    String jsonString = JSON.stringify(readings);
    return jsonString;
    // ===============
};

// getting Accelerometer information
String getAccReadings(void)
{
    mpu.getEvent(&a, &g, &temp);
    // Get current acceleration values
    accX = a.acceleration.x;
    accY = a.acceleration.y;
    accZ = a.acceleration.z;

    // the following code lines are used to enter those readings into the JSON object
    readings["accX"] = String(accX);
    readings["accY"] = String(accY);
    readings["accZ"] = String(accZ);
    String accString = JSON.stringify(readings);
    return accString;
    // ===============
};

// getting Temperature information
String getTemperature(void)
{
    mpu.getEvent(&a, &g, &temp);
    temperature = temp.temperature;
    return String(temperature);
};

// function to read data from the MPU-6050
void readMPUdata(void)
{
    // get all the various information from the MPU-6050 from the other functions
    getAccReadings();
    getGyroReadings();
    getTemperature();
}; // end reading MPU data

//  to check to see if there is a change in the table position
void checkMPUData(bool &dataChanged)
{
    readMPUdata(); // reads the MPU-6050 // Read gyro and accelerometer data

    // Check if gyro or accelerometer data has changed beyond the thresholds
    if (abs(gyroX - previousGyroX) > gyroThreshold ||
        abs(gyroY - previousGyroY) > gyroThreshold ||
        abs(accX - previousAccX) > accelThreshold ||
        abs(accY - previousAccY) > accelThreshold)
    {
        // Data has changed
        previousGyroX = gyroX;
        previousGyroY = gyroY;
        previousAccX = accX;
        previousAccY = accY;
        dataChanged = true;
        Serial.println("Data has changed"); // used for testing purposes only
    }
    else
    {
        // Data has not changed
        dataChanged = false;
        // Serial.println("Data has NOT Changed"); // used for testing purposes only
    }
}; // end boolean to check if gyro or accel from mpu has changed position

// Interrupt Service Routine (ISR)
// void handleInterrupt()
// {
//     checkMPUData(dataChanged);
// };

// ===================================================
// FUNCTIONS FOR MOVEMENT OF THE TABLE DISPLAYING LEDS
// ===================================================
// visually see the table is level through stand up LEDs
void testLEDsLevel(void)
{
    // visual total length of the leveling LEDs is from LED 110 -> 175 then my middle points are LEDs 142-143
    FastLED.clear();
    // version 1.0.0
    // for (int i = 0; i < NUM_LED_PIN1 + 1; i++)
    // {
    //     // NUM_LED_PIN1 = 222; // 220 LEDs
    //
    //     if (i < 110 || i > 175)
    //     {
    //         led_Strip1[i] = CRGB::Black;
    //         // Serial.print(i); // used for testing purposes only
    //         FastLED.show();
    //     }
    //     else
    //     {
    //         led_Strip1[i] = CRGB::White;
    //         // Serial.print(i); // used for testing purposes only
    //         FastLED.show();
    //     }
    // }

    // version 2.0.0
    // const int middlePoint1 = 143;
    // const int orangePoint1 = 149;
    // const int redPoint1 = 154;
    // const int tiltFwdRedPoint1 = 165;
    // const int tiltFwdOrangePoint1 = 170;
    // const int endPoint1 = 175;
    // const int tiltBckOrangePoint1 = 180;
    // const int tiltBckRedPoint1 = 185;
    // const int middlePoint2 = 142;
    // const int orangePoint2 = 137;
    // const int redPoint2 = 132;
    // const int tiltFwdRedPoint2 = 120;
    // const int tiltFwdOrangePoint2 = 115;
    // const int endPoint2 = 110;
    // const int tiltBckOrangePoint2 = 105;
    // const int tiltBckRedPoint2 = 100;
    // Start from the middle and expand outwards
    for (int i = 0; i <= middlePoint1; i++)
    {
        // Check if indices are within bounds before writing to them
        if (middlePoint2 - i >= 0 && middlePoint1 - i < NUM_LED_PIN1)
        {
            if (middlePoint2 - i < endPoint2)
            {
                led_Strip1[middlePoint2 - i] = CRGB::Black;
            }
            else
            {
                led_Strip1[middlePoint2 - i] = CRGB::White;
            }
        }

        if (middlePoint1 + i >= 0 && middlePoint1 + i < NUM_LED_PIN1)
        {
            if (middlePoint1 + i > endPoint1)
            {
                led_Strip1[middlePoint1 + i] = CRGB::Black;
            }
            else
            {
                led_Strip1[middlePoint1 + i] = CRGB::White;
            }
        }
        FastLED.show();
    }
}; // end test LEDs length level

void setLEDsLevel(void)
{
    FastLED.clear();
    // version 1.0.0
    // for (int i = 0; i < NUM_LED_PIN1 + 1; i++)
    // {
    //     // if visual total length of the leveling LEDs is from LED 110 -> 175 then my middle points are LEDs 142-143
    //     // for the level LEDs to be 10 LEDs, I would need to add 4 LEDs to the left and 4 LEDs to the right = 143+4 and 142-4 = 146-139
    //     if (i < 139 || i > 146)
    //     {
    //         led_Strip1[i] = CRGB::Black;
    //         // Serial.print(i); // used for testing purposes only
    //         FastLED.show();
    //     }
    //     else
    //     {
    //         led_Strip1[i] = CRGB::Green;
    //         // Serial.print(i); // used for testing purposes only
    //         FastLED.show();
    //     }
    // }

    // version 2.0.0
    // const int middlePoint1 = 143;
    // const int orangePoint1 = 149;
    // const int redPoint1 = 154;
    // const int tiltFwdRedPoint1 = 165;
    // const int tiltFwdOrangePoint1 = 170;
    // const int endPoint1 = 175;
    // const int tiltBckOrangePoint1 = 180;
    // const int tiltBckRedPoint1 = 185;
    // const int middlePoint2 = 142;
    // const int orangePoint2 = 137;
    // const int redPoint2 = 132;
    // const int tiltFwdRedPoint2 = 120;
    // const int tiltFwdOrangePoint2 = 115;
    // const int endPoint2 = 110;
    // const int tiltBckOrangePoint2 = 105;
    // const int tiltBckRedPoint2 = 100;
    // Start from the middle and expand outwards
    for (int i = 0; i <= middlePoint1; i++)
    {
        // Check if indices are within bounds before writing to them
        if (middlePoint2 - i >= 0 && middlePoint2 - i < NUM_LED_PIN1)
        {
            if (middlePoint2 - i < orangePoint2)
            {
                led_Strip1[middlePoint2 - i] = CRGB::Black;
            }
            else
            {
                led_Strip1[middlePoint2 - i] = CRGB::Green;
            }
        }
        if (middlePoint1 + i >= 0 && middlePoint1 + i < NUM_LED_PIN1)
        {
            if (middlePoint1 + i > orangePoint1)
            {
                led_Strip1[middlePoint1 + i] = CRGB::Black;
            }
            else
            {
                led_Strip1[middlePoint1 + i] = CRGB::Green;
            }
        }
        FastLED.show();
    }
}; // end set LEDs level

// visually see the tilted left level through stand up LEDs
void setLEDsTiltedLeft(int left)
{
    FastLED.clear();
    // version 1.0.0
    // then I would have to blank out 147-222 and 138-0
    // then I would add 2 LEDs to the left and 2 LEDs to the right for blanking spots from the level LEDs = 138-1 and 147+1 = 147-148 and 138-137
    // for the small tilt left to be 5 LEDs, 149-153 (left == 1).
    // if (left == 1)
    // {
    //     for (int i = 0; i < NUM_LED_PIN1 + 1; i++)
    //     {
    //         if (i < 149 || i > 153)
    //         {
    //             led_Strip1[i] = CRGB::Black;
    //             // Serial.print(i); // used for testing purposes only
    //             FastLED.show();
    //         }
    //         else
    //         {
    //             led_Strip1[i] = CRGB::Orange;
    //             // Serial.print(i); // used for testing purposes only
    //             FastLED.show();
    //         }
    //     }
    // }
    // // for the large tilt left to be 5 LEDs, 154-158 (left == 2)
    // else if (left == 2)
    // {
    //     for (int i = 0; i < NUM_LED_PIN1 + 1; i++)
    //     {
    //         if (i < 154 || i > 158)
    //         {
    //             led_Strip1[i] = CRGB::Black;
    //             // Serial.print(i); // used for testing purposes only
    //             FastLED.show();
    //         }
    //         else
    //         {
    //             led_Strip1[i] = CRGB::Red;
    //             // Serial.print(i); // used for testing purposes only
    //             FastLED.show();
    //         }
    //     }
    // }

    // version 2.0.0
    // const int middlePoint1 = 143;
    // const int orangePoint1 = 149;
    // const int redPoint1 = 154;
    // const int tiltFwdRedPoint1 = 165;
    // const int tiltFwdOrangePoint1 = 170;
    // const int endPoint1 = 175;
    // const int tiltBckOrangePoint1 = 180;
    // const int tiltBckRedPoint1 = 185;
    // const int middlePoint2 = 142;
    // const int orangePoint2 = 137;
    // const int redPoint2 = 132;
    // const int tiltFwdRedPoint2 = 120;
    // const int tiltFwdOrangePoint2 = 115;
    // const int endPoint2 = 110;
    // const int tiltBckOrangePoint2 = 105;
    // const int tiltBckRedPoint2 = 100;
    CRGB colour1;
    CRGB colour2;
    CRGB colour3;

    if (left == 1)
    {
        colour1 = CRGB::Green;
        colour2 = CRGB::Orange;
        // Turn on the LEDs within the range
        for (int i = 0; i < NUM_LED_PIN1; i++)
        {
            if (i >= middlePoint1 && i <= orangePoint1)
            {
                led_Strip1[i] = colour1;
                FastLED.show();
            }
            else if (i >= (orangePoint1 - 1) && i <= redPoint1)
            {
                led_Strip1[i] = colour2;
                FastLED.show();
            }
        }
    }
    else if (left == 2)
    {
        colour1 = CRGB::Green;
        colour2 = CRGB::Orange;
        colour3 = CRGB::Red;
        // Turn on the LEDs within the range
        for (int i = 0; i < NUM_LED_PIN1; i++)
        {
            if (i >= middlePoint1 && i <= orangePoint1)
            {
                led_Strip1[i] = colour1;
                FastLED.show();
            }
            else if (i >= (orangePoint1 - 1) && i <= redPoint1)
            {
                led_Strip1[i] = colour2;
                FastLED.show();
            }
            else if (i >= (redPoint1 - 1) && i <= redPoint1 + 5)
            {
                led_Strip1[i] = colour3;
                FastLED.show();
            }
        }
    }
}; // end set LEDs titled left

// visually see the tilted right level through stand up LEDs
void setLEDsTiltedRight(int right)
{
    FastLED.clear();
    // version 1.0.0
    // for the small tilt right to be 5 LEDs, 136-132.
    // if (right == 1)
    // {
    //     for (int i = 0; i < NUM_LED_PIN1 + 1; i++)
    //     {
    //         if (i < 132 || i > 136)
    //         {
    //             led_Strip1[i] = CRGB::Black;
    //             // Serial.print(i); // used for testing purposes only
    //             FastLED.show();
    //         }
    //         else
    //         {
    //             led_Strip1[i] = CRGB::Orange;
    //             // Serial.print(i); // used for testing purposes only
    //             FastLED.show();
    //         }
    //     }
    // }
    // // for the large tilt right to be 5 LEDs, 131-127.
    // else if (right == 2)
    // {
    //     for (int i = 0; i < NUM_LED_PIN1 + 1; i++)
    //     {
    //         if (i < 127 || i > 131)
    //         {
    //             led_Strip1[i] = CRGB::Black;
    //             // Serial.print(i); // used for testing purposes only
    //             FastLED.show();
    //         }
    //         else
    //         {
    //             led_Strip1[i] = CRGB::Red;
    //             // Serial.print(i); // used for testing purposes only
    //             FastLED.show();
    //         }
    //     }
    // }

    // version 2.0.0
    // const int middlePoint1 = 143;
    // const int orangePoint1 = 149;
    // const int redPoint1 = 154;
    // const int tiltFwdRedPoint1 = 165;
    // const int tiltFwdOrangePoint1 = 170;
    // const int endPoint1 = 175;
    // const int tiltBckOrangePoint1 = 180;
    // const int tiltBckRedPoint1 = 185;
    // const int middlePoint2 = 142;
    // const int orangePoint2 = 137;
    // const int redPoint2 = 132;
    // const int tiltFwdRedPoint2 = 120;
    // const int tiltFwdOrangePoint2 = 115;
    // const int endPoint2 = 110;
    // const int tiltBckOrangePoint2 = 105;
    // const int tiltBckRedPoint2 = 100;
    CRGB colour1;
    CRGB colour2;
    CRGB colour3;

    if (right == 1)
    {
        colour1 = CRGB::Green;
        colour2 = CRGB::Orange;
        // Turn on the LEDs within the range
        for (int i = 0; i < NUM_LED_PIN1; i++)
        {
            if (i <= middlePoint2 && i >= orangePoint2)
            {
                led_Strip1[i] = colour1;
                FastLED.show();
            }
            else if (i <= (orangePoint2 + 1) && i >= redPoint2)
            {
                led_Strip1[i] = colour2;
                FastLED.show();
            }
        }
    }
    else if (right == 2)
    {
        colour1 = CRGB::Green;
        colour2 = CRGB::Orange;
        colour3 = CRGB::Red;
        // Turn on the LEDs within the range
        for (int i = 0; i < NUM_LED_PIN1; i++)
        {
            if (i <= middlePoint2 && i >= orangePoint2)
            {
                led_Strip1[i] = colour1;
                FastLED.show();
            }
            else if (i <= (orangePoint2 + 1) && i >= redPoint2)
            {
                led_Strip1[i] = colour2;
                FastLED.show();
            }
            else if (i <= (redPoint2 + 1) && i >= redPoint2 - 5)
            {
                led_Strip1[i] = colour3;
                FastLED.show();
            }
        }
    }
}; // end set LEDs titled right

// visually see the tilted formward level through stand up LEDs
void setLEDsTiltedForward(int fwd)
{
    FastLED.clear();
    // version 1.0.0
    // if visual total length of the leveling LEDs is from LED 110->175
    // for the tilt forward to be 5 LEDs, I can use the 175-171 and 110-114.
    // if (fwd == 1)
    // {
    //     for (int i = 0; i < NUM_LED_PIN1 + 1; i++)
    //     {
    //         if ((i < 171 || i > 175) && (i < 110 || i > 114))
    //         {
    //             led_Strip1[i] = CRGB::Black;
    //             // Serial.print(i); // used for testing purposes only
    //             FastLED.show();
    //         }
    //         else
    //         {
    //             led_Strip1[i] = CRGB::Orange;
    //             // Serial.print(i); // used for testing purposes only
    //             FastLED.show();
    //         }
    //     }
    // }
    // // for the tilt forward more to be 5 LEDs, I can use the 170-165 and 115-119.
    // else if (fwd == 2)
    // {
    //     for (int i = 0; i < NUM_LED_PIN1 + 1; i++)
    //     {
    //         if ((i < 165 || i > 170) && (i < 115 || i > 119))
    //         {
    //             led_Strip1[i] = CRGB::Black;
    //             // Serial.print(i); // used for testing purposes only
    //             FastLED.show();
    //         }
    //         else
    //         {
    //             led_Strip1[i] = CRGB::Red;
    //             // Serial.print(i); // used for testing purposes only
    //             FastLED.show();
    //         }
    //     }
    // }

    // version 2.0.0
    // const int middlePoint1 = 143;
    // const int orangePoint1 = 149;
    // const int redPoint1 = 154;
    // const int tiltFwdRedPoint1 = 165;
    // const int tiltFwdOrangePoint1 = 170;
    // const int endPoint1 = 175;
    // const int tiltBckOrangePoint1 = 180;
    // const int tiltBckRedPoint1 = 185;
    // const int middlePoint2 = 142;
    // const int orangePoint2 = 137;
    // const int redPoint2 = 132;
    // const int tiltFwdRedPoint2 = 120;
    // const int tiltFwdOrangePoint2 = 115;
    // const int endPoint2 = 110;
    // const int tiltBckOrangePoint2 = 105;
    // const int tiltBckRedPoint2 = 100;
    CRGB colour1;
    CRGB colour2;

    if (fwd == 1)
    {
        colour1 = CRGB::Orange;
        // Turn on the LEDs within the range
        for (int i = 0; i < NUM_LED_PIN1; i++)
        {
            if ((i >= tiltFwdOrangePoint1 && i <= endPoint1) || (i >= endPoint2 && i <= tiltFwdOrangePoint2))
            {
                led_Strip1[i] = colour1;
                FastLED.show();
            }
        }
    }
    else if (fwd == 2)
    {
        colour1 = CRGB::Orange;
        colour2 = CRGB::Red;
        // Turn on the LEDs within the range
        for (int i = 0; i < NUM_LED_PIN1; i++)
        {
            if ((i >= tiltFwdOrangePoint1 && i <= endPoint1) || (i >= endPoint2 && i <= tiltFwdOrangePoint2))
            {
                led_Strip1[i] = colour1;
                FastLED.show();
            }
            else if ((i <= (tiltFwdOrangePoint1 - 1) && i >= tiltFwdRedPoint1) || (i >= (tiltFwdOrangePoint2 + 1) && i <= tiltFwdRedPoint2))
            {
                led_Strip1[i] = colour2;
                // FastLED.show();
            }
            FastLED.show();
        }
    }
}; // end set LEDs titled forward

// visually see the tilted backward level through stand up LEDs
void setLEDsTiltedBackward(int back)
{
    FastLED.clear();
    // version 1.0.0
    // if visual total length of the leveling LEDs is from LED 110->175
    // for the tilt backward to be 5 LEDs, I can use the 176-180 and 109-105.
    // if (back == 1)
    // {
    //     for (int i = 0; i < NUM_LED_PIN1 + 1; i++)
    //     {
    //         if ((i < 176 || i > 180) && (i < 105 || i > 109))
    //         {
    //             led_Strip1[i] = CRGB::Black;
    //             // Serial.print(i); // used for testing purposes only
    //             FastLED.show();
    //         }
    //         else
    //         {
    //             led_Strip1[i] = CRGB::Orange;
    //             // Serial.print(i); // used for testing purposes only
    //             FastLED.show();
    //         }
    //     }
    // }
    // // for the tilt backward more to be 5 LEDs, I can use the 181-185 and 104-100.
    // else if (back == 2)
    // {
    //     for (int i = 0; i < NUM_LED_PIN1 + 1; i++)
    //     {
    //         if ((i < 181 || i > 185) && (i < 100 || i > 104))
    //         {
    //             led_Strip1[i] = CRGB::Black;
    //             // Serial.print(i); // used for testing purposes only
    //             FastLED.show();
    //         }
    //         else
    //         {
    //             led_Strip1[i] = CRGB::Red;
    //             // Serial.print(i); // used for testing purposes only
    //             FastLED.show();
    //         }
    //     }
    // }

    // version 2.0.0
    // const int middlePoint1 = 143;
    // const int orangePoint1 = 149;
    // const int redPoint1 = 154;
    // const int tiltFwdRedPoint1 = 165;
    // const int tiltFwdOrangePoint1 = 170;
    // const int endPoint1 = 175;
    // const int tiltBckOrangePoint1 = 180;
    // const int tiltBckRedPoint1 = 185;
    // const int middlePoint2 = 142;
    // const int orangePoint2 = 137;
    // const int redPoint2 = 132;
    // const int tiltFwdRedPoint2 = 120;
    // const int tiltFwdOrangePoint2 = 115;
    // const int endPoint2 = 110;
    // const int tiltBckOrangePoint2 = 105;
    // const int tiltBckRedPoint2 = 100;
    CRGB colour1;
    CRGB colour2;

    if (back == 1)
    {
        colour1 = CRGB::Orange;
        // Turn on the LEDs within the range
        for (int i = 0; i < NUM_LED_PIN1; i++)
        {
            if ((i >= (endPoint1 + 1) && i <= tiltBckOrangePoint1) || (i <= (endPoint2 - 1) && i >= tiltBckOrangePoint2))
            {
                led_Strip1[i] = colour1;
                FastLED.show();
            }
        }
    }
    else if (back == 2)
    {
        colour1 = CRGB::Orange;
        colour2 = CRGB::Red;
        // Turn on the LEDs within the range
        for (int i = 0; i < NUM_LED_PIN1; i++)
        {
            if ((i >= (endPoint1 + 1) && i <= tiltBckOrangePoint1) || (i <= (endPoint2 - 1) && i >= tiltBckOrangePoint2))
            {
                led_Strip1[i] = colour1;
                FastLED.show();
            }
            else if ((i >= (tiltBckOrangePoint1 + 1) && i <= tiltBckRedPoint1) || (i <= (tiltBckOrangePoint2 - 1) && i >= tiltBckRedPoint2))
            {
                led_Strip1[i] = colour2;
                FastLED.show();
            }
        }
    }
}; // end set LEDs titled backward

// to display position of the table using the LEDs when moving up or down
void displayPosition(void)
{
    // if the table is level, display the level
    if ((angle_roll_output < 1.00) && (angle_roll_output > -1.00))
    {
        // Set LEDs to indicate the table is level
        setLEDsLevel();
        return;
    }
    // if the table is tilted left, display the tilt left
    else if ((angle_roll_output > -2.00) && (angle_roll_output < -1.01))
    {
        // Set LEDs to indicate the table is tilted left
        setLEDsTiltedLeft(1);
        return;
    }
    else if (angle_roll_output < -2.01)
    {
        // Set LEDs to indicate the table is tilted left more
        setLEDsTiltedLeft(2);
        return;
    }
    // if the table is tilted right, display the tilt right
    else if ((angle_roll_output > 1.01) && (angle_roll_output < 2.00))
    {
        // Set LEDs to indicate the table is tilted right
        setLEDsTiltedRight(1);
        return;
    }
    else if (angle_roll_output > 2.01)
    {
        // Set LEDs to indicate the table is tilted right more
        setLEDsTiltedRight(2);
        return;
    }
    // if the table is tilted forward or tilted backward, display the tilt
    else if ((angle_pitch_output > 1.01) && (angle_pitch_output < 2.00))
    {
        // Set LEDs to indicate the table is tilted forwards
        setLEDsTiltedForward(1);
        return;
    }
    else if (angle_pitch_output < -2.01)
    {
        // Set LEDs to indicate the table is tilted forwards more
        setLEDsTiltedForward(2);
        return;
    }
    // if the table is , display the backward tilt, display the tilt
    else if (angle_pitch_output < -2.01)
    {
        // Set LEDs to indicate the table is tilted backwards more
        setLEDsTiltedBackward(1);
        return;
    }
    else if ((angle_pitch_output > -2.00) && (angle_pitch_output < -1.01))
    {
        // Set LEDs to indicate the table is tilted backwards
        setLEDsTiltedBackward(2);
        return;
    }
}; // end display position function



// ===================================================
// FUNCTIONS FOR MOVEMENT OF THE TABLE
// ===================================================
// void resetFlags(void)
// {
//     if (!isMovingUpLeft && !isMovingDownLeft && isStopMovementLeft)
//     {
//         isStopMovementLeft = false;
//     }
//     if (!isMovingUpRight && !isMovingDownRight && isStopMovementRight)
//     {
//         isStopMovementRight = false;
//     }
//     if (isStopMovement)
//     {
//         isStopMovement = false;
//     }
//     if (isMovingUp && isMovingDown)
//     {
//         isMovingUpLeft = false;
//         isMovingDownLeft = false;
//         isMovingUpRight = false;
//         isMovingDownRight = false;
//     }
// }; // resetting the flags for movement

void moveActuators(bool leftUp, bool rightUp)
{
    if (leftUp)
    {
        digitalWrite(relay1_LA1LHP_Pin, LOW);
        digitalWrite(relay2_LA1LHM_Pin, HIGH);
    }
    else
    {
        digitalWrite(relay1_LA1LHP_Pin, HIGH);
        digitalWrite(relay2_LA1LHM_Pin, LOW);
    }

    if (rightUp)
    {
        digitalWrite(relay3_LA2RHP_Pin, LOW);
        digitalWrite(relay4_LA2RHM_Pin, HIGH);
    }
    else
    {
        digitalWrite(relay3_LA2RHP_Pin, HIGH);
        digitalWrite(relay4_LA2RHM_Pin, LOW);
    }
}

void moveUp()
{
    moveActuators(true, true);
    displayPosition();
}

void moveDown()
{
    moveActuators(false, false);
    displayPosition();
}

void stopMovement()
{
    moveActuators(false, false);
    displayPosition();
}

void tableLevel()
{
    if (isTableLevel)
    {
        stopMovement();
    }
}

// function to move the linear actuators up
// void moveUp(void)
// {
//     // Check if the left actuator is not already moving up
//     if (!isMovingUpLeft)
//     {
//         // Move left actuator up
//         digitalWrite(relay1_LA1LHP_Pin, LOW);
//         digitalWrite(relay2_LA1LHM_Pin, HIGH); // Redundant protection
//         // Set flag to indicate left actuator is moving up
//         isMovingUpLeft = true;
//     }

//     // Check if the right actuator is not already moving up
//     if (!isMovingUpRight)
//     {
//         // Move right actuator up
//         digitalWrite(relay3_LA2RHP_Pin, LOW);
//         digitalWrite(relay4_LA2RHM_Pin, HIGH); // Redundant protection
//         // Set flag to indicate right actuator is moving up
//         isMovingUpRight = true;
//     }
//     // Activate the relay controlling actuator #1 positive
//     // digitalWrite(relay1_LA1LHP_Pin, LOW);
//     // digitalWrite(relay2_LA1LHM_Pin, HIGH); // redundant protection
//     // Activate the relay controlling actuator #1 negative
//     // digitalWrite(relay3_LA2RHP_Pin, LOW);
//     // digitalWrite(relay4_LA2RHM_Pin, HIGH); // redundant protection

//     // Set the movement flags
//     isMovingUp = true;
//     isMovingDown = false;
//     resetFlags(); // Reset flags after movement
// 
//     // display the LEDs to show the table movement
//     Serial.println("Moving Up");
//     displayPosition();
// }; // end move up linear actuators function

//  function to move the linear actuators down
// void moveDown(void)
// {
//     // Check if the left actuator is not already moving down
//     if (!isMovingDownLeft)
//     {
//         // Move left actuator down
//         digitalWrite(relay1_LA1LHP_Pin, HIGH); // Redundant protection
//         digitalWrite(relay2_LA1LHM_Pin, LOW);
//         // Set flag to indicate left actuator is moving down
//         isMovingDownLeft = true;
//     }
// 
//     // Check if the right actuator is not already moving down
//     if (!isMovingDownRight)
//     {
//         // Move right actuator down
//         digitalWrite(relay3_LA2RHP_Pin, HIGH); // Redundant protection
//         digitalWrite(relay4_LA2RHM_Pin, LOW);
//         // Set flag to indicate right actuator is moving down
//         isMovingDownRight = true;
//     }
//     // Activate the relay controlling actuator #2 positive
//     // digitalWrite(relay1_LA1LHP_Pin, HIGH); // redundant protection
//     // digitalWrite(relay2_LA1LHM_Pin, LOW);
//     // Activate the relay controlling actuator #2 negative
//     // digitalWrite(relay3_LA2RHP_Pin, HIGH); // redundant protection
//     // digitalWrite(relay4_LA2RHM_Pin, LOW);
// 
//     // Set the movement flags
//     isMovingUp = false;
//     isMovingDown = true;
//     resetFlags(); // Reset flags after movement
// 
//     // display the LEDs to show the table movement
//     Serial.println("Moving Down");
//     displayPosition();
// }; // end move down linear actuator function

// function to stop mevement of the linear actuators
// void stopMovement(void)
// {
//     // Check if the left actuator is not already moving down
//     if (!isStopMovementLeft)
//     {
//         // Move left actuator down
//         digitalWrite(relay1_LA1LHP_Pin, HIGH); // Redundant protection
//         digitalWrite(relay2_LA1LHM_Pin, HIGH);
//         // Set flag to indicate left actuator is stopping
//         isStopMovementLeft = true;
//         displayPosition();
//     }
//     // Check if the right actuator is not already moving down
//     if (!isStopMovementRight)
//     {
//         // Move right actuator down
//         digitalWrite(relay3_LA2RHP_Pin, HIGH); // Redundant protection
//         digitalWrite(relay4_LA2RHM_Pin, HIGH);
//         // Set flag to indicate right actuator is stopping
//         isStopMovementRight = true;
//         displayPosition();
//     }
//     // Deactivate all relays // FULL STOP
//     if (isStopMovement)
//     {
//         digitalWrite(relay1_LA1LHP_Pin, HIGH);
//         digitalWrite(relay2_LA1LHM_Pin, HIGH);
//         digitalWrite(relay3_LA2RHP_Pin, HIGH);
//         digitalWrite(relay4_LA2RHM_Pin, HIGH);
//         // display the LEDs to show the table movement
//         for (int i = 0; i < 10; i++)
//         {
//             Serial.println("Stopping Movement");
//             displayPosition();
//         }
//     }
// 
//     // Clear the movement flags
//     isMovingUp = false;
//     isMovingDown = false;
//     resetFlags(); // Reset flags after movement
// };                // end stopping movment function

// the MAIN table movement function
void tableMovement(void)
{
    // read what is the gyro and accel position
    // readMPUdata();
    checkMPUData(dataChanged);
    // if (dataChanged)
    // {
    //     // Reset dataChanged to false when you've handled the change
    //     dataChanged = false;
    //     // Data has changed, take appropriate action
    //     // For example, call another function or update your LED effects
    //     return;
    // }
    Serial.println("from getReadings");
    Serial.print("gyroX = ");
    Serial.print(gyroX);
    Serial.print(", \tgyroY = ");
    Serial.print(gyroY);
    Serial.print(", \tgyroZ = ");
    Serial.println(gyroZ);
    Serial.print("accX = ");
    Serial.print(accX);
    Serial.print(", \taccY = ");
    Serial.print(accY);
    Serial.print(", \taccZ = ");
    Serial.println(accZ);
    // stationary
    // gyroX = 3.41, gyroY = -1.40, gyroZ = 1.01 accX = 1.34, accY = -0.03, accZ = -10.16
    // gyroX = 3.44, gyroY = -1.36, gyroZ = 0.98 accX = 1.36, accY = -0.01, accZ = -10.04
    // when changing offsets to:


    // to display the gyro and accelerometer data
    displayAllGyro2(); // Call the function to display the gyro data without fps
    // float tiltAngle = atan2(accelData.x, sqrt(pow(accelData.y, 2) + pow(accelData.z, 2))) * (180.0 / PI);
    // IMUCommunications();
    // Use gyro data to adjust linear actuator speed
    float speedAdjustment = map(gyroX, -90, 90, -MAX_SPEED_ADJUSTMENT, MAX_SPEED_ADJUSTMENT);

    // if (!isCommandBeingProcessed)
    // {
    //     int absoluteX = abs(angle_roll_output);
    //     int absoluteY = abs(angle_pitch_output);
    //     if (absoluteX <= threshold && absoluteY <= threshold)
    //     {
    //         isTableLevel = true;
    //         if (!isStopMovement)
    //         {
    //             stopMovement();
    //         }
    //     }
    //     else
    //     {
    //         isTableLevel = false;
    //     }
    // }

    // version 1
    // float rightActuatorSpeed = BASE_SPEED - speedAdjustment;
    // float leftActuatorSpeed = BASE_SPEED + speedAdjustment;
    // 
    // // Synchronize movements of left and right actuators
    // if (rightActuatorSpeed > 0)
    // {
    //     moveDown();
    // }
    // else if (rightActuatorSpeed < 0)
    // {
    //     moveUp();
    // }
    // else
    // {
    //     stopMovement();
    // }
    // 
    // if (leftActuatorSpeed > 0)
    // {
    //     moveDown();
    // }
    // else if (leftActuatorSpeed < 0)
    // {
    //     moveUp();
    // }
    // else
    // {
    //     stopMovement();
    // }
    // =======================

    // version 2
    if (!isCommandBeingProcessed)
    {
        float leftActuatorSpeed = BASE_SPEED + speedAdjustment;
        float rightActuatorSpeed = BASE_SPEED - speedAdjustment;

        // Synchronize movements of left and right actuators
        if (rightActuatorSpeed > 0)
        {
            moveDown(); // Changed from moveUp() to moveDown()
        }
        else if (rightActuatorSpeed < 0)
        {
            moveUp(); // Changed from moveDown() to moveUp()
        }
        else
        {
            stopMovement();
        }

        if (leftActuatorSpeed > 0)
        {
            moveDown(); // Changed from moveUp() to moveDown()
        }
        else if (leftActuatorSpeed < 0)
        {
            moveUp(); // Changed from moveDown() to moveUp()
        }
        else
        {
            stopMovement();
        }
    }

    // Gyro angle calculations. Note: 0.0000611 = 1 / (250Hz x 65.5)
    // Calculate the traveled pitch angle and add this to the angle_pitch variable
    angle_pitch += gyroX * 0.0000611;
    // Calculate the traveled roll angle and add this to the angle_roll variable
    // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
    angle_roll += gyroY * 0.0000611;

    // If the IMU has yawed transfer the roll angle to the pitch angle
    angle_pitch += angle_roll * sin(gyroZ * 0.000001066);
    // If the IMU has yawed transfer the pitch angle to the roll angle
    angle_roll -= angle_pitch * sin(gyroZ * 0.000001066);

    // ACCELEROMETER ANGLE CALCULATIONS
    // Calculate the total accelerometer vector
    acc_total_vector = sqrt((accX * accX) + (accY * accY) + (accZ * accZ));
    // 57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
    // Calculate the pitch angle
    angle_pitch_acc = asin((float)accY / acc_total_vector) * 57.296;
    // Calculate the roll angle
    angle_roll_acc = asin((float)accX / acc_total_vector) * -57.296;

    // Accelerometer calibration value for pitch
    angle_pitch_acc -= 0.0;
    // Accelerometer calibration value for roll
    angle_roll_acc -= 0.0;

    if (set_gyro_angles)
    {
        // If the IMU has been running
        // Correct the drift of the gyro pitch angle with the accelerometer pitch angle
        angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
        // Correct the drift of the gyro roll angle with the accelerometer roll angle
        angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
    }
    else
    {
        // IMU has just started
        // Set the gyro pitch angle equal to the accelerometer pitch angle
        angle_pitch = angle_pitch_acc;
        // Set the gyro roll angle equal to the accelerometer roll angle
        angle_roll = angle_roll_acc;
        // Set the IMU started flag
        set_gyro_angles = true;
    }

    // TO DAMPEN THE PITCH AND ROLL ANGLES A COMPLEMENTARY FILTER IS USED
    // Take 90% of the output pitch value and add 10% of the raw pitch value
    angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
    Serial.print("angle_pitch_output = ");
    Serial.println(angle_pitch_output);
    // Take 90% of the output roll value and add 10% of the raw roll value
    angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;
    Serial.print("angle_roll_output = ");
    Serial.println(angle_roll_output);
    // Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop

    // Update position based on gyro data
    if (abs(angle_roll_output) > threshold || abs(angle_pitch_output) > threshold)
    {
        if (angle_roll_output < 0 || angle_pitch_output < 0)
        {
            newPosition--;
            Serial.print("Position =");
            Serial.println(newPosition);
        }
        else
        {
            newPosition++;
            Serial.print("Position =");
            Serial.println(newPosition);
        }
    }

    // // Calculate the absolute values of accelerometer data
    // int absoluteX = abs(angle_roll_output);
    // // Serial.print("absoluteX ="); // used for testing purposes only
    // // Serial.println(absoluteX);   // used for testing purposes only
    // // Serial.print("ANGLE_ROLL_OUTPUT ="); // used for testing purposes only
    // // Serial.println(angle_roll_output); // used for testing purposes only
    // int absoluteY = abs(angle_pitch_output);
    // // Serial.print("absoluteY ="); // used for testing purposes only
    // // Serial.println(absoluteY);   // used for testing purposes only
    // // Serial.print("ANGLE_PITCH_OUTPUT ="); // used for testing purposes only
    // // Serial.println(angle_pitch_output); // used for testing purposes only
    // // int absoluteZ = abs(accelerometerZ); // not used

    // if (absoluteX > threshold)
    // {
    //     // Tilt detected along the X-axis (front or back)
    //     if (accX < 0)
    //     {
    //         // Tilted forward, move the table down
    //         newPosition--;
    //     }
    //     else
    //     {
    //         // Tilted backward, move the table up
    //         newPosition++;
    //     }
    // }

    // if (absoluteY > threshold)
    // {
    //     // Tilt detected along the Y-axis (left or right)
    //     if (accY < 0)
    //     {
    //         // Tilted to the left, move the table down
    //         newPosition--;
    //     }
    //     else
    //     {
    //         // Tilted to the right, move the table up
    //         newPosition++;
    //     }
    // }

    // Update the position only if it has changed
    if (newPosition != currentPosition)
    {
        // Stop the previous movement
        stopMovement();
        // Update the current position
        currentPosition = newPosition;
        // Move the table to the new position
        if (currentPosition > previousPosition)
        {
            // Move the table up
            moveUp();
        }
        else if (currentPosition < previousPosition)
        {
            // Move the table down
            moveDown();
        }
        // Update the previous position
        previousPosition = currentPosition;
        Serial.print("Curent Position = ");
        Serial.println("currentPosition");
    }

    // Check if the table is level
    // if (absoluteX <= threshold && absoluteY <= threshold)
    // {
    //     isTableLevel = true;
    // }
    // else
    // {
    //     isTableLevel = false;
    // }
}; // end table movement function

void processCommand(char command)
{
    // Process the command
    switch (command)
    {
    case 'Q':
        Serial.println(F("Quitting Program"));
        exit(0);
        break;
    case 'q':
        Serial.println(F("Quitting Program"));
        exit(0);
        break;
    case 'M':
        Serial.println(F("Constant Gyro Monitoring"));
        while (true)
        {
            readMPUdata();
            displayAllGyro2();
            delay(100);
        }
        break;
    case 'm':
        Serial.println(F("Constant Gyro Monitoring"));
        while (true)
        {
            readMPUdata();
            displayAllGyro2();
            delay(100);
        }
        break;
    case 'D':
        Serial.println(F("Moving Down"));
        // movingDown(); // display on the OLED Screen arrow down
        moveDown();
        break;
    case 'd':
        Serial.println(F("Moving Down"));
        // movingDown(); // display on the OLED Screen arrow down
        moveDown();
        break;
    case 'U':
        Serial.println(F("Moving Up"));
        // movingUp(); // display on the OLED Screen arrow down
        moveUp();
        break;
    case 'u':
        Serial.println(F("Moving Up"));
        // movingUp(); // display on the OLED Screen arrow down
        moveUp();
        break;
    case 'S':
        Serial.println(F("Stopping Movement"));
        // heartBeat(); // display on the OLED Screen the hearbeat
        isStopMovement = true;
        stopMovement();
        break;
    case 's':
        Serial.println(F("Stopping Movement"));
        // heartBeat(); // display on the OLED Screen the hearbeat
        isStopMovement = true;
        stopMovement();
        break;
    default:
        Serial.println(F("Invalid Command"));
        break;
    }
}; // end process command function

void tableCommands(void)
{
    // RT-11 commands
    // Handle remote signals
    // if (digitalRead(upPin) == LOW)
    // {
    //     // Activate linear actuators to raise the desk
    //     digitalWrite(relayPin1, HIGH);
    //     digitalWrite(relayPin2, HIGH);
    //     digitalWrite(relayPin3, HIGH);
    //     digitalWrite(relayPin4, HIGH);
    // }
    //
    // if (digitalRead(downPin) == LOW)
    // {
    //     // Activate linear actuators to lower the desk
    //     digitalWrite(relayPin1, LOW);
    //     digitalWrite(relayPin2, LOW);
    //     digitalWrite(relayPin3, LOW);
    //     digitalWrite(relayPin4, LOW);
    // }

    Serial.println(F("\nSend 'U'/'u' for Up, 'D'/'d' for Down, 'S'/'s' for STOP \n'Q'/'q' to Quit Program, 'M'/'m' for Constant Gyro Monitoring.\n"));

    while (!Serial.available())
    {
        delay(100);
    }

    // Check if a command is available from the Serial Monitor
    if (Serial.available())
    {
        // bool isCommandBeingProcessed = true;
        char command = Serial.read();

        // Consume any additional characters from the Serial buffer
        while (Serial.available())
        {
            Serial.read();
        }

        // Debug output to display the received command
        Serial.print(F("Received Command: "));
        Serial.println(command);

        // Save the command for later use
        lastCommand = command;

        // Process the command
        isCommandBeingProcessed = true; // Set the flag to true before processing the command
        processCommand(command);
        isCommandBeingProcessed = false; // Set the flag back to false after the command is processed

        // After processing user commands, call tableMovement to adjust the table position
        tableMovement();
    }
    else
    {
        // If no new command is received, use the last command
        if (lastCommand != 0)
        {
            Serial.print(F("Repeating last command: "));
            Serial.println(lastCommand);

            // Process the last command
            isCommandBeingProcessed = true; // Set the flag to true before processing the command
            processCommand(lastCommand);
            isCommandBeingProcessed = false; // Set the flag back to false after the command is processed
        }
        else
        {
            Serial.println(F("No command received"));
            isStopMovement = true;
            stopMovement();
        }
    }
}; // end table commands function

#endif // STANDUPTABLEMOVEMENT_H

// =========================================================
// END OF PROGRAM
// =========================================================
