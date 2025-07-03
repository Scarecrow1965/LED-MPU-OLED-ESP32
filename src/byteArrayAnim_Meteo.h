// +-------------------------------------------------------------
//
// Equipment:
// ESP32, OLED SSD1306
//
// File: byteArrayAnim_Meteo.h
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

const uint8_t totalarrays_Meteo PROGMEM = 11; // ensure this is the same as the number of arrays in the WeatherArray below

Frame MeteoArray[11] {
    {"/by_cldWx.bin", "Cloudy Weather"},          // MeteoArray[0] = {"/by_cldWx.bin", "Cloudy Weather"},
    {"/by_lSnWx.bin", "Light Snow Weather"},      // MeteoArray[1] = {"/by_lSnWx.bin", "Light Snow Weather"},
    {"/by_lngWx.bin", "Lightning Weather"},       // MeteoArray[2] = {"/by_lngWx.bin", "Lightning Weather"},
    {"/by_bltWx.bin", "Lightning Bolt Weather"},  // MeteoArray[3] = {"/by_bltWx.bin", "Lightning Bolt Weather"},
    {"/by_rngWx.bin", "Rainy Weather"},           // MeteoArray[4] = {"/by_rngWx.bin", "Rainy Weather"},
    {"/by_snoWx.bin", "Snowstorm Weather"},       // MeteoArray[5] = {"/by_snoWx.bin", "Snowstorm Weather"},
    {"/by_stoWx.bin", "Stormy Weather"},          // MeteoArray[6] = {"/by_stoWx.bin", "Stormy Weather"},
    {"/by_sunWx.bin", "Sun Weather"},             // MeteoArray[7] = {"/by_sunWx.bin", "Sun Weather"},
    {"/by_tmpWx.bin", "Temperature Weather"},     // MeteoArray[8] = {"/by_tmpWx.bin", "Temperature Weather"},
    {"/by_tRnWx.bin", "Torrential Rain Weather"}, // MeteoArray[9] = {"/by_tRnWx.bin", "Torrential Rain Weather"},
    {"/by_wndWx.bin", "Windy Weather"},           // MeteoArray[10] = {"/by_wndWx.bin", "Windy Weather"},
};

void byteArrayMeteo_Anim(void)
{
    // Serial.println("Starting Meteo byte Array loop");

    for (uint8_t i = 0; i < totalarrays_Meteo; i++)
    {
        uint8_t *data;
        uint8_t dataSize;

        loadAnimation(MeteoArray[i].fileName, &data, &dataSize);

        uint8_t frame = 0;
        uint8_t effecttime = 30;

        u8g2.clearBuffer();

        while (effecttime > 0)
        {
            u8g2.home();
            u8g2.setCursor(3, oled_LineH * 1 + 2);
            u8g2.print(MeteoArray[i].name);
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

void byteArrayMeteo_Display(uint8_t i)
{
    uint8_t *data;
    uint8_t dataSize;

    loadAnimation(MeteoArray[i].fileName, &data, &dataSize);

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

void Meteo_cloudyWeather(void)
{
    // MeteoArray[0] = {"/by_cldWx.bin", "Cloudy Weather"},
    byteArrayMeteo_Display(0);
}; // end cloudyWeather Display function

void Meteo_lightSnowWeather(void)
{
    // MeteoArray[1] = {"/by_lSnWx.bin", "Light Snow Weather"},
    byteArrayMeteo_Display(1);
}; // end lightSnowWeather Display function

void Meteo_lightningWeather(void)
{
    // MeteoArray[2] = {"/by_lngWx.bin", "Lightning Weather"},
    byteArrayMeteo_Display(2);
}; // end lightningWeather Display function

void Meteo_lightningboltWeather(void)
{
    // MeteoArray[3] = {"/by_bltWx.bin", "Lightning Bolt Weather"},
    byteArrayMeteo_Display(3);
}; // end lightningboltWeather Display function

void Meteo_rainyWeather(void)
{
    // MeteoArray[4] = {"/by_rngWx.bin", "Rainy Weather"},
    byteArrayMeteo_Display(4);
}; // end rainyWeather Display function

void Meteo_snowStormWeather(void)
{
    // MeteoArray[5] = {"/by_snoWx.bin", "Snowstorm Weather"},
    byteArrayMeteo_Display(5);
}; // end snowStormWeather Display function

void Meteo_stormyWeather(void)
{
    // MeteoArray[6] = {"/by_stoWx.bin", "Stormy Weather"},
    byteArrayMeteo_Display(6);
}; // end stormyWeather Display function

void Meteo_sunWeather(void)
{
    // MeteoArray[7] = {"/by_sunWx.bin", "Sun Weather"},
    byteArrayMeteo_Display(7);
}; // end sunWeather Display function

void Meteo_temperatureWeather(void)
{
    // MeteoArray[8] = {"/by_tmpWx.bin", "Temperature Weather"},
    byteArrayMeteo_Display(8);
}; // end temperatureWeather Display function

void Meteo_torrentialRainWeather(void)
{
    // MeteoArray[9] = {"/by_tRnWx.bin", "Torrential Rain Weather"},
    byteArrayMeteo_Display(9);
}; // end torrentialRainWeather Display function

void Meteo_windyWeather(void)
{
    // MeteoArray[10] = {"/by_wndWx.bin", "Windy Weather"},
    byteArrayMeteo_Display(10);
}; // end windyWeather Display function

// =========================================================
// END OF PROGRAM
// =========================================================
