// +-------------------------------------------------------------
//
// Equipment:
// ESP32, OLED SSD1306
//
// File: byteArrayAnim_System.h
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

const uint8_t totalarrays_System PROGMEM = 12; // ensure this is the same as the number of arrays in the SyustemArray array

Frame SystemArray[12] {
    {"/by_bell.bin", "Bell"},            // SystemArray[0] = {"/by_bell.bin", "Bell"},
    {"/by_chkOK.bin", "Checkmark OK"},   // SystemArray[1] = {"/by_chkOK.bin", "Checkmark OK"},
    {"/by_clksp.bin", "Spinning Clock"}, // SystemArray[2] = {"/by_clksp.bin", "Spinning Clock"},
    {"/by_globe.bin", "Globe"},          // SystemArray[3] = {"/by_globe.bin", "Globe"},
    {"/by_home.bin", "Home"},            // SystemArray[4] = {"/by_home.bin", "Home"},
    {"/by_hrgl.bin", "Hourglass"},       // SystemArray[5] = {"/by_hrgl.bin", "Hourglass"},
    {"/by_noCon.bin", "No Connection"},  // SystemArray[6] = {"/by_noCon.bin", "No Connection"},
    {"/by_snd.bin", "Sound"},            // SystemArray[7] = {"/by_snd.bin", "Sound"},
    {"/by_wifish.bin", "WIFI Search"},   // SystemArray[8] = {"/by_wifish.bin", "WIFI Search"},
    {"/by_gear.bin", "Gear"},            // SystemArray[9] = {"/by_gear.bin", "Gear"},
    {"/by_gears.bin", "Gears"},          // SystemArray[10] = {"/by_gears.bin", "Gears"},
    {"/by_setng.bin", "Settings"},       // SystemArray[11] = {"/by_setng.bin", "Settings"},
};

void byteArraySystem_Anim(void)
{
    // Serial.println("Starting System byte Array loop");

    for (uint8_t i = 0; i < totalarrays_System; i++)
    {
        uint8_t *data;
        uint8_t dataSize;

        loadAnimation(SystemArray[i].fileName, &data, &dataSize);

        uint8_t frame = 0;
        uint8_t effecttime = 30;

        u8g2.clearBuffer();

        while (effecttime > 0)
        {
            u8g2.home();
            u8g2.setCursor(3, oled_LineH * 1 + 2);
            u8g2.print(SystemArray[i].name);
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

void byteArraySystem_Display(uint8_t i)
{
    uint8_t *data;
    uint8_t dataSize;

    loadAnimation(SystemArray[i].fileName, &data, &dataSize);

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

void System_bell(void)
{
    // SystemArray[0] = {"/by_bell.bin", "Bell"},
    byteArraySystem_Display(0);
}; // end bell function

void System_checkmarkOK(void)
{
    // SystemArray[1] = {"/by_chkOK.bin", "Checkmark OK"},
    byteArraySystem_Display(1);
}; // end checkmarkOK function

void System_clockspin(void)
{
    // SystemArray[2] = {"/by_clksp.bin", "Spinning Clock"},
    byteArraySystem_Display(2);
}; // end clockspin function

void System_globe(void)
{
    // SystemArray[3] = {"/by_globe.bin", "Globe"},
    byteArraySystem_Display(3);
}; // end globe function

void System_home(void)
{
    // SystemArray[4] = {"/by_home.bin", "Home"},
    byteArraySystem_Display(4);
}; // end home function

void System_hourglass(void)
{
    // SystemArray[5] = {"/by_hrgl.bin", "Hourglass"},
    byteArraySystem_Display(5);
}; // end hourglass function

void System_noConnection(void)
{
    // SystemArray[6] = {"/by_noCon.bin", "No Connection"},
    byteArraySystem_Display(6);
}; // end noConnection function

void System_sound(void)
{
    // SystemArray[7] = {"/by_snd.bin", "Sound"},
    byteArraySystem_Display(7);
}; // end sound function

void System_wifisearch(void)
{
    // SystemArray[8] = {"/by_wifish.bin", "WIFI Search"},
    byteArraySystem_Display(8);
}; // end wifisearch function

void System_gear(void)
{
    // SystemArray[9] = {"/by_gear.bin", "Gear"},
    byteArraySystem_Display(9);
}; // end gear function

void System_gears(void)
{
    // SystemArray[10] = {"/by_gears.bin", "Gears"},
    byteArraySystem_Display(10);
}; // end gears function

void System_settings(void)
{
    // SystemArray[11] = {"/by_setng.bin", "Settings"},
    byteArraySystem_Display(11);
}; // end settings function

// =========================================================
// END OF PROGRAM
// =========================================================
