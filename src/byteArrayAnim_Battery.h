// +-------------------------------------------------------------
//
// Equipment:
// ESP32, OLED SSD1306
//
// File: byteArrayAnim_Battery.h
//
// Description:
//
// main file to engage all byte array graphics, whether they be
// non-animated or animated on the ESP 32 platform
//
// History:     1-Dec-2023     Scarecrow1965   Created
//
// +-------------------------------------------------------------

// install ibraries
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SD.h>

#include "seeOLED.h"

extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
extern Adafruit_SSD1306 display;

static const uint8_t totalarrays_Battery PROGMEM = 4; // ensure this is the same as the number of arrays in the BatteryArray below

Frame BatteryArray[4] {
    {"/by_batLv.bin", "Battery Level"}, // BatteryaArray[0] = {"/by_batLv.bin", "Battery Level"},
    {"/by_chBat.bin", "Charged Battery"}, // BatteryaArray[1] = {"/by_chBat.bin", "Charged Battery"},
    {"/by_cgBat.bin", "Charging Battery"}, // BatteryaArray[2] = {"/by_cgBat.bin", "Charging Battery"},
    {"/by_lwBat.bin", "Low Battery"}, // BatteryaArray[3] = {"/by_lwBat.bin", "Low Battery"},
};

void byteArrayBattery_Anim(void)
{
    // Serial.println("Starting Battery byte Array loop");

    for (uint8_t i = 0; i < totalarrays_Battery; i++)
    {
        uint8_t *data;
        uint8_t dataSize;

        loadAnimation(BatteryArray[i].fileName, &data, &dataSize);

        uint8_t frame = 0;
        uint8_t effecttime = 30;

        u8g2.clearBuffer();

        while (effecttime > 0)
        {
            u8g2.home();
            u8g2.setCursor(3, oled_LineH * 1 + 2);
            u8g2.print(BatteryArray[i].name);
            u8g2.sendBuffer();

            // Serial.print("Displaying frame= "); // used for testing purposes only
            // Serial.println(frame); // used for testing purposes only

            display.drawBitmap(0, 15, &data[frame * 288], height_width1, height_width1, 1);
            display.display();
            frame = (frame + 1) % frameCount;
            effecttime--;
            // delay(1000); // used for testing purposes only
            display.clearDisplay();
        }
        // Reset frame to 0 for the next animation
        frame = 0;
        // Don't forget to delete the data array to free up memory
        delete[] data;
    }
    // Serial.println("ending loop");
}; // end byte Array Animation Loop function

void byteArrayBattery_Display(uint8_t i)
{
    uint8_t *data;
    uint8_t dataSize;

    loadAnimation(BatteryArray[i].fileName, &data, &dataSize);

    uint8_t frame = 0;
    for (uint8_t j = 0; j < frameCount; j++)
    {
        display.drawBitmap(0, 15, &data[frame * 288], height_width1, height_width1, 1);
        display.display();
        frame = (frame + 1) % frameCount;
        display.clearDisplay();
    }
    // Don't forget to free the allocated memory when you're done with it
    delete[] data;
}; // end byte Array Animation Display function

void Battery_isLevel(void)
{
    // BatteryaArray[0] = {"/by_batLv.bin", "Battery Level"},
    byteArrayBattery_Display(0);
};

void Battery_isCharged(void)
{
    // BatteryaArray[1] = {"/by_chBat.bin", "Charged Battery"},
    byteArrayBattery_Display(1);
};

void Battery_isCharging(void)
{
    // BatteryaArray[2] = {"/by_cgBat.bin", "Charging Battery"},
    byteArrayBattery_Display(2);
};

void Battery_isLow(void)
{
    // BatteryaArray[3] = {"/by_lwBat.bin", "Low Battery"},
    byteArrayBattery_Display(3);
};

// =========================================================
// END OF PROGRAM
// =========================================================