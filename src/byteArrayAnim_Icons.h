// +-------------------------------------------------------------
//
// Equipment:
// ESP32, OLED SSD1306
//
// File: byteArrayAnim_Icons.h
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

extern const uint8_t height_width1;
extern const uint8_t frameCount;

const uint8_t totalarrays_Icons PROGMEM = 7; // ensure this is the same as the number of arrays in the IconsArray array

Frame IconsArray[7] {
    {"/by_hrtbt.bin", "Heartbeat"},     // IconsArray[0] = {"/by_hrtbt.bin", "Heartbeat"},
    {"/by_acft.bin", "Aircraft"},       // IconsArray[1] = {"/by_acft.bin", "Aircraft"},
    {"/by_event.bin", "Event"},         // IconsArray[2] = {"/by_event.bin", "Event"},
    {"/by_plot.bin", "Plot"},           // IconsArray[3] = {"/by_plot.bin", "Plot"},
    {"/by_toggl.bin", "Toggle"},        // IconsArray[4] = {"/by_toggl.bin", "Toggle"},
    {"/by_opLet.bin", "Open Letter"},   // IconsArray[5] = {"/by_opLet.bin", "Open Letter"},
    {"/by_phrng.bin", "Phone Ringing"}, // IconsArray[6] = {"/by_phrng.bin", "Phone Ringing"},
};

void byteArrayIcons_Anim(void)
{
    // Serial.println("Starting Icons byte Array loop");

    for (uint8_t i = 0; i < totalarrays_Icons; i++)
    {
        uint8_t *data;
        uint8_t dataSize;

        loadAnimation(IconsArray[i].fileName, &data, &dataSize);

        uint8_t frame = 0;
        uint8_t effecttime = 30;

        u8g2.clearBuffer();

        while (effecttime > 0)
        {
            u8g2.home();
            u8g2.setCursor(3, oled_LineH * 1 + 2);
            u8g2.print(IconsArray[i].name);
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

void byteArrayIcons_Display(uint8_t i)
{
    uint8_t *data;
    uint8_t dataSize;

    loadAnimation(IconsArray[i].fileName, &data, &dataSize);

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

void Icons_heartbeat(void)
{
    // IconsArray[0] = {"/by_hrtbt.bin", "Heartbeat"},
    byteArrayIcons_Display(0);
}; // end heartbeatDisplay function

void Icons_aircraft(void)
{
    // IconsArray[1] = {"/by_acft.bin", "Aircraft"},
    byteArrayIcons_Display(1);
}; // end aircraftDisplay function

void Icons_event(void)
{
    // IconsArray[2] = {"/by_event.bin", "Event"},
    byteArrayIcons_Display(2);
}; // end eventDisplay function

void Icons_plot(void)
{
    // IconsArray[3] = {"/by_plot.bin", "Plot"},
    byteArrayIcons_Display(3);
}; // end plotDisplay function

void Icons_toggle(void)
{
    // IconsArray[4] = {"/by_toggl.bin", "Toggle"},
    byteArrayIcons_Display(4);
}; // end toggleDisplay function

void Icons_openLetter(void)
{
    // IconsArray[5] = {"/by_opLet.bin", "Open Letter"},
    byteArrayIcons_Display(5);
}; // end openLetterDisplay function

void Icons_phoneRinging(void)
{
    // IconsArray[6] = {"/by_phrng.bin", "Phone Ringing"},
    byteArrayIcons_Display(6);
}; // end phoneringingDisplay function

// =========================================================
// END OF PROGRAM
// =========================================================
