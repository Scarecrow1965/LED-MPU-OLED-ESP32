// +-------------------------------------------------------------
//
// Equipment:
// ESP32, OLED SSD1306
//
// File: byteArrayAnim_Position.h
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

const uint8_t totalarrays_Position PROGMEM = 5; // ensure this amount is ther same as the number of arrays in the PositionArray below

Frame PositionArray[5] {
    {"/by_unUpd.bin", "Uninstalling Updates"}, // PositionArray[0] = {"/by_unUpd.bin", "Uninstalling Updates"},
    {"/by_insUpd.bin", "Installing Updates"},  // PositionArray[1] = {"/by_insUpd.bin", "Installing Updates"},
    {"/by_upld.bin", "Upload"},                // PositionArray[2] = {"/by_upld.bin", "Upload"},
    {"/by_dwnld.bin", "Download"},             // PositionArray[3] = {"/by_dwnld.bin", "Download"},
    {"/by_dwnAr.bin", "Down Arrow"},           // PositionArray[4] = {"/by_dwnAr.bin", "Down Arrow"},
};

void byteArrayPosition_Anim(void)
{
    // Serial.println("Starting Position byte Array loop");

    for (uint8_t i = 0; i < totalarrays_Position; i++)
    {
        uint8_t *data;
        uint8_t dataSize;

        loadAnimation(PositionArray[i].fileName, &data, &dataSize);

        uint8_t frame = 0;
        uint8_t effecttime = 30;

        u8g2.clearBuffer();

        while (effecttime > 0)
        {
            u8g2.home();
            u8g2.setCursor(3, oled_LineH * 1 + 2);
            u8g2.print(PositionArray[i].name);
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

void byteArrayPosition_Display(uint8_t i)
{
    uint8_t *data;
    uint8_t dataSize;

    loadAnimation(PositionArray[i].fileName, &data, &dataSize);

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

void Position_UninstallingUpdates(void)
{
    // PositionArray[0] = {"/by_unUpd.bin", "Uninstalling Updates"},
    byteArrayPosition_Display(0);
}; // end Uninstalling updates arrow Loop function

void Position_InstallingUpdates(void)
{
    // PositionArray[1] = {"/by_insUpd.bin", "Installing Updates"},
    byteArrayPosition_Display(1);
}; // end Installing updates arrow Loop function

void Position_Upload(void)
{
    // PositionArray[2] = {"/by_upld.bin", "Upload"},
    byteArrayPosition_Display(2);
}; // end Upload arrow Loop function

void Position_Download(void)
{
    // PositionArray[3] = {"/by_dwnld.bin", "Download"},
    byteArrayPosition_Display(3);
}; // end Download arrow Loop function

void Position_Down(void)
{
    // PositionArray[4] = {"/by_dwnAr.bin", "Down Arrow"},
    byteArrayPosition_Display(4);
}; // end Down arrow Loop function

// =========================================================
// END OF PROGRAM
// =========================================================
