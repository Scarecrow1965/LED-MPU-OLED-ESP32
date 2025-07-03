// +-------------------------------------------------------------
//
// Equipment:
// Arduino Mega, MPU-6050, OLED SSD1306, WS2812B Addressable LEDS x3,
// 4-relay module, 2 linear actuators, RT-11(JCHT35K9) remote/handset,
// and 650W 12V Power supply
//
// File: seeOLED.h
//
// Description:
//
// Creates a library to enable all information that is seen on the
// OLED display to go through here
//
// History:     25-Jan-2024     Scarecrow1965   Created
//
// +-------------------------------------------------------------

#ifndef SEEOLED_H
#define SEEOLED_H

// ================
// install ibraries
// ================
#include <Arduino.h>
#include <Wire.h>

// this set of libraries is for File manipulation
#include <SD.h>
// #include <SPI.h>

// for OLED screen
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <U8g2lib.h>

// this library is for the MPU-6050
#include "standupTableMovement.h"

extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
extern Adafruit_SSD1306 display;
extern float fps; // to start recording the fps of the display
extern bool bLED;

static uint8_t oled_LineH = 0;
const uint8_t height_width1 = 48;
const uint8_t height_width2 = 50;
const uint8_t frameCount = 28;

struct Frame
{
    const char *fileName;
    const String name;
};

// void loadAnimation(const char *fileName, uint8_t **data, uint8_t *dataSize);
void loadAnimation(const char *fileName, uint8_t **data, uint8_t *dataSize)
{
    if (!SD.begin(5))
    {
        Serial.println("SD card initialization failed!");
        return;
    }

    // Open the file
    File file = SD.open(fileName);
    if (!file)
    {
        Serial.println("Failed to open file");
        return;
    }

    // Get the size of the file
    *dataSize = file.size() / 288;

    // Allocate memory for the data
    *data = new uint8_t[*dataSize * 288];

    // Read the data from the file
    for (int i = 0; i < *dataSize; i++)
    {
        file.read(&(*data)[i * 288], 288);
    }

    // Close the file
    file.close();
}; // end of loading animations from file.

void loadBinFile(const char *filename, uint8_t **buffer, size_t *size)
{
    if (!SD.begin())
    {
        Serial.println("Failed to initialize SD card");
        return;
    }

    File file = SD.open(filename, FILE_READ);
    if (!file)
    {
        Serial.println("Failed to open file");
        return;
    }

    // Calculate the total size of the animation
    *size = file.size();

    // Allocate memory for the animation buffer
    *buffer = new uint8_t[*size];
    if (*buffer == nullptr)
    {
        Serial.println("Failed to allocate memory");
        return;
    }
    // Read animation data into the buffer
    for (size_t i = 0; i < *size; i++)
    {
        (*buffer)[i] = file.read();
    }

    file.close();
}; // end loadBinFile function

#include "byteArrayAnim_Battery.h"
#include "byteArrayAnim_Icons.h"
#include "byteArrayAnim_Meteo.h"
#include "byteArrayAnim_Position.h"
#include "byteArrayAnim_System.h"

#include "charArrayNonAnim.h"
#include "charArrayAnim.h"
charArrayNonAnim charNoAnimation;
charArrayAnim charAnimation;

// =============================
// displaying the arduino logo when staring up the OLED display
void startingOLED(void)
{
    // Serial.println("Starting OLED Display called"); // used for testing purposes only
    charNoAnimation.ARDUINOLogo();
};
// =============================
// to display all the Battery animations
void displayBattery_Anim(void)
{
    // Serial.println("Starting Battery Animation called"); // used for testing purposes only
    byteArrayBattery_Anim();
};
// to display all the Icons animations
void displayIcons_Anim(void)
{
    // Serial.println("Starting Icons Animation called"); // used for testing purposes only
    byteArrayIcons_Anim();
};
// to display all the Meteo animations
void displayMeteo_Anim(void)
{
    // Serial.println("Starting Meteo Animation called"); // used for testing purposes only
    byteArrayMeteo_Anim();
};
// to display all the Position animations
void displayPosition_Anim(void)
{
    // Serial.println("Starting Position Animation called"); // used for testing purposes only
    byteArrayPosition_Anim();
};
// to display all the System animations
void displaySystem_Anim(void)
{
    // Serial.println("Starting System Animation called"); // used for testing purposes only
    byteArraySystem_Anim();
};
// to display all the charArrayNonAnim
void displayCharArrayNonAnim(void)
{
    // Serial.println("Starting charArrayNonAnim called"); // used for testing purposes only
    charArrayNonAnim CharArray_NonAnim();
};
// to display all the charArrayAnim
void displayCharArrayAnim(void)
{
    // Serial.println("Starting charArrayAnim called"); // used for testing purposes only
    charArrayAnim verifyAllAnimations();
};
// =============================
// =============================
// Individual animations


// =============================
// USED AS A TEST FUNCTION
// =============================
// FramesPerSecond function
// Tracks a weighted average in order to smooth out the values that is given. Computes the FPS as the simple reciprocal
// of the amount of time taken specified by the caller. so 1/3 of a second is 3 fps. It takes about 10 frames to stabilize.
double FramesPerSecond(double seconds)
{
    static double framesPerSeconds;
    framesPerSeconds = (framesPerSeconds * .9) + (1.0 / seconds * .1);
    return framesPerSeconds;
}; // end Frames per second function

void DrawLinesAndGraphicsFrame(int framePerSecond)
{
    u8g2.clearBuffer();
    u8g2.home();

    // Draw some text on the left hand side
    u8g2.setCursor(3, oled_LineH * 2 + 2);
    // u8g2.setCursor(5, 32);
    u8g2.print("Hello");
    u8g2.setCursor(3, oled_LineH * 3 + 2);
    // u8g2.setCursor(5, 42);
    u8g2.print("World");
    u8g2.setCursor(3, oled_LineH * 4 + 2);
    u8g2.printf("%03d \n", framePerSecond); // Placeholder for framerate

    u8g2.drawFrame(0, 0, u8g2.getWidth(), u8g2.getHeight()); // Draw a border around the display

    // draw a moire pattern like it's 1984
    // for (int x = 0; x < u8g2.getWidth(); x++) // if I use this then all I see is two triangle atop one another
    for (int x = 0; x < u8g2.getWidth(); x += 4) // this will give me some separation of lines within the two triangles
    {
        u8g2.drawLine(x, 0, u8g2.getWidth() - x, u8g2.getHeight());
    }

    // Draw a reticle on the right hand side
    const int reticleY = u8g2.getHeight() / 2;           // Vertical center of display
    const int reticleR = u8g2.getHeight() / 4 - 2;       // Slightly less than 1/4 screen height
    const int reticleX = u8g2.getWidth() - reticleR - 8; // Right-justified with a small margin

    for (int r = reticleR; r > 0; r -= 3)
    { // draw a series of nested circles
        u8g2.drawCircle(reticleX, reticleY, r);
        u8g2.drawHLine(reticleX - reticleR - 5, reticleY, 2 * reticleR + 10); // H line through reticle center
        u8g2.drawVLine(reticleX, reticleY - reticleR - 5, 2 * reticleR + 10); // V line through reticle center
    }

    u8g2.sendBuffer(); // Send it out
}; // end draw lines, graphics function
// =============================

// function to display information on the OLED screen
void displayAllGyro(double framepersecond)
{
    bLED = !bLED; // toggle LED State
    digitalWrite(LED_BUILTIN, bLED);

    u8g2.clearBuffer();
    u8g2.home();
    u8g2.drawFrame(0, 0, u8g2.getWidth(), u8g2.getHeight()); // Draw a border around the display

    // readMPUdata(); // reads the MPU-6050

    u8g2.setFont(u8g2_font_profont10_tf);
    u8g2.setCursor(3, oled_LineH * 1 + 2);
    u8g2.print("FPS: ");
    u8g2.print(framepersecond); // Placeholder for framerate
    u8g2.setCursor(3, oled_LineH * 3 + 2);
    u8g2.println("Accel(m/s^2)");
    u8g2.setCursor(3, oled_LineH * 4 + 2);
    u8g2.print("X: ");
    u8g2.print(accX);
    u8g2.print(", Y: ");
    u8g2.print(accY);
    u8g2.print(", Z: ");
    u8g2.print(accZ);
    u8g2.setCursor(3, oled_LineH * 5 + 2);
    u8g2.println("Gyro(rps)");
    u8g2.setCursor(3, oled_LineH * 6 + 2);
    u8g2.print("X: ");
    u8g2.print(gyroX, 1);
    u8g2.print(", Y: ");
    u8g2.print(gyroY);
    u8g2.print(", Z: ");
    u8g2.print(gyroZ);
    u8g2.setCursor(3, oled_LineH * 7 + 2);
    u8g2.print("Temp= ");
    u8g2.print(temperature);
    // u8g2.print(temp.temperature);
    u8g2.print(" ");
    u8g2.print(char(176));
    u8g2.setFont(u8g2_font_profont10_tf);
    u8g2.print("C");
    u8g2.sendBuffer(); // Send it out
}; // end displaying MPU-6050 information with fps

// function to display information on the OLED screen without FPS
void displayAllGyro2(void)
{
    bLED = !bLED; // toggle LED State
    digitalWrite(LED_BUILTIN, bLED);
    
    display.clearDisplay();
    u8g2.clearBuffer();
    u8g2.home();
    u8g2.drawFrame(0, 0, u8g2.getWidth(), u8g2.getHeight()); // Draw a border around the display
    u8g2.setFont(u8g2_font_profont10_tf);
    u8g2.setCursor(3, oled_LineH * 3 + 2);
    u8g2.println("Accel(m/s^2)");
    u8g2.setCursor(3, oled_LineH * 4 + 2);
    u8g2.print("X:");
    u8g2.print(accX);
    u8g2.print(", Y:");
    u8g2.print(accY);
    u8g2.print(", Z:");
    u8g2.print(accZ);
    u8g2.setCursor(3, oled_LineH * 5 + 2);
    u8g2.println("Gyro(rps)");
    u8g2.setCursor(3, oled_LineH * 6 + 2);
    u8g2.print("X:");
    u8g2.print(gyroX, 1);
    u8g2.print(", Y:");
    u8g2.print(gyroY);
    u8g2.print(", Z:");
    u8g2.print(gyroZ);
    u8g2.setCursor(3, oled_LineH * 7 + 2);
    u8g2.print("Temp= ");
    u8g2.print(temp.temperature);
    u8g2.print(" ");
    u8g2.print(char(176));
    u8g2.setFont(u8g2_font_profont10_tf);
    u8g2.print("C");
    u8g2.sendBuffer(); // Send it out
}; // end displaying the MPU-6050 info without the fps

// displays information on the SSD1306 OLED screen
void displayOLED(double fps)
{
    bLED = !bLED; // toggle LED State
    digitalWrite(LED_BUILTIN, bLED);

    displayAllGyro(fps); // displays all Gyro information and FPS
    // displayFPS(fps); // displays only FPS and used with animation
}; // end of displaying information on OLED Screen

// USING ADAFRUIT AND U8G2
// display FPS, added to animation
void displayFPS(double framepersecond)
{
    bLED = !bLED; // toggle LED State
    digitalWrite(LED_BUILTIN, bLED);

    u8g2.home();
    u8g2.setFont(u8g2_font_profont10_tf);
    u8g2.setCursor(3, oled_LineH * 1 + 2);
    u8g2.print("FPS: ");
    u8g2.print(framepersecond); // Placeholder for framerate
    u8g2.sendBuffer();
    // Serial.print("FPS = "); // used for testing purposes only
    // Serial.print(framespersecond); // used for testing purposes only
    // Serial.print("\n");            // used for testing purposes only
}; // end of displaying FPS function


// heatbeat signal animation with FPS Counter
// void heartBeat(double framespersecond)
// {
//     if (currentMillis - oledanimPreviousMillis >= timePeriod010)
//     {
//         oledanimPreviousMillis = currentMillis;

//         bLED = !bLED; // toggle LED State
//         digitalWrite(LED_BUILTIN, bLED);

//         // Adafruit SSD1306 codes
//         display.clearDisplay();
//         displayFPS(framespersecond);
//         // void Adafruit_GFX::drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h, uint16_t color, uint16_t bg)
//         /*******************************************************/
//         /*
//         @param    x   Top left corner x coordinate
//         @param    y   Top left corner y coordinate
//         @param    bitmap  byte array with monochrome bitmap
//         @param    w   Width of bitmap in pixels
//         @param    h   Height of bitmap in pixels
//         @param    color 16-bit 5-6-5 Color to draw pixels with
//         @param    bg 16-bit 5-6-5 Color to draw background with
//         */
//         /*******************************************************/
//         // changed code from display.drawBitmap(32, 0, heartbeatframe[frame], FRAME_WIDTH, FRAME_HEIGHT, 1); // used for animation without FPS
//         display.drawBitmap(0, 10, heartbeatframe[frame], FRAME_WIDTH, FRAME_HEIGHT, 1); // used for animation with FPS
//         display.display();
//         frame = (frame + 1) % FRAME_COUNT3;
//         // =======================================
//         // U8G2 codes: DOES NOT WORK
//         // u8g2.clearBuffer();
//         // u8g2.setCursor(3, oled_LineH * 3 + 2);
//         // void drawBitmap(u8g2_uint_t x, u8g2_uint_t y, u8g2_uint_t cnt, u8g2_uint_t h, const uint8_t *bitmap)
//         // where position x = x , position y = y, cnt, height = h, *bitmap = bitmap array
//         // u8g2.drawBitmap(32, 0, 512, FRAME_HEIGHT, heartbeatframe[frame]);
//         // u8g2.sendBuffer();
//         // frame = (frame + 1) % FRAME_COUNT3;
//         // delay(FRAME_DELAY);
//         // ---------------------------------------
//     }
// } // end heartbeat function with FPS Counter on OLED Screen

// // heart beat animation
// void heartBeat()
// {
//     if (currentMillis - oledanimPreviousMillis >= timePeriod010)
//     {
//         oledanimPreviousMillis = currentMillis;

//         bLED = !bLED; // toggle LED State
//         digitalWrite(LED_BUILTIN, bLED);

//         // Adafruit SSD1306 codes
//         display.clearDisplay();
//         display.drawBitmap(32, 0, heartbeatframe[frame], FRAME_WIDTH, FRAME_HEIGHT, 1); // used for animation without FPS
//         // display.drawBitmap(0, 10, heartbeatframe[frame], FRAME_WIDTH, FRAME_HEIGHT, 1); // used for animation with FPS
//         display.display();
//         frame = (frame + 1) % FRAME_COUNT3;
//     }
// } // end heartbeat function on OLED Screen

// // moving down arrow with FPS animation
// void movingdownFPS(double framespersecond)
// {
//     if (currentMillis - oledanimPreviousMillis >= timePeriod010)
//     {
//         oledanimPreviousMillis = currentMillis;
// 
//         bLED = !bLED; // toggle LED State
//         digitalWrite(LED_BUILTIN, bLED);
// 
//         display.clearDisplay();
//         displayFPS(framespersecond);
//         //   display.drawBitmap(32, 0, framedown[frame], FRAME_WIDTH, FRAME_HEIGHT, 1); // used for animation without FPS
//         display.drawBitmap(0, 10, framedown[frame], FRAME_WIDTH, FRAME_HEIGHT, 1); // used for animation with FPS
//         display.display();
//         frame = (frame + 1) % FRAME_COUNT1;
//     }
// } // end moving down arrow with FPS counter on OLED Screen function

// // moving down without FPS counter animation
// void movingdown(void)
// {
//     if (currentMillis - oledanimPreviousMillis >= timePeriod010)
//     {
//         oledanimPreviousMillis = currentMillis;
// 
//         bLED = !bLED; // toggle LED State
//         digitalWrite(LED_BUILTIN, bLED);
// 
//         display.clearDisplay();
//         display.drawBitmap(32, 0, framedown[frame], FRAME_WIDTH, FRAME_HEIGHT, 1); // used for animation without FPS
//         // display.drawBitmap(0, 10, framedown[frame], FRAME_WIDTH, FRAME_HEIGHT, 1);
//         display.display();
//         frame = (frame + 1) % FRAME_COUNT1;
//     }
// } // end moving down arrow on OLED Screen function

// setup for Adafruit SSD1306
// does not work
// moving up animation
// void movingup(void)
// {
//     bLED = !bLED; // toggle LED State
//     digitalWrite(LED_BUILTIN, bLED);
//     display.clearDisplay();
//     display.drawBitmap(0, 10, frameup[frame], FRAME_WIDTH, FRAME_HEIGHT, 1);
//     display.display();
//     frame = (frame + 1) % FRAME_COUNT2;
//     // delay(FRAME_DELAY);
// }

#endif // SEEOLED_H

// =========================================================
// END OF PROGRAM
// =========================================================
