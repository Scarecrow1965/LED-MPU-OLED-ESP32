// +-------------------------------------------------------------
//
// Equipment:
// ESP32, OLED SSD1306
//
// File: main.cpp
//
// Description:
//
// main file to engage all char array graphics, whether they be
// non-animated or animated on the ESP 32 platform
//
// History:     1-Nov-2023     Scarecrow1965   Created
//
// +-------------------------------------------------------------

// CONFIRMED: WORKS ON ESP32 AND OLED SCREEN

// install ibraries
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// this set of libraries is for File manipulation
#include <SPI.h>
#include <SD.h>

// library used to sort files when listing directory
#include <vector>
#include <algorithm>

// This will enable for the OLED screen to display information
// definition of OLED display SSD1306 for Arduino Mega SCA & SDL
#define OLED_CLOCK 17 // SCA pin on Display = SCL on ESP32
#define OLED_DATA 20  // SDL pin on display = SDA on ESP32

// U8G2 SSD1306 Driver here to run OLED Screen
// built constructor for the OLED function
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, OLED_CLOCK, OLED_DATA, U8X8_PIN_NONE); // This works but according to the function, it shouldn't
static uint8_t oled_LineH = 0;

// #define LED_BUILTIN 2 // pin for onboard LED or use LED_BUILTIN as the default location
static bool bLED = LOW;

// ADAFRUIT SSD1306 Driver here to run animation
// NOTE: confirmed by I2C Scanner address for SSD1306 is 0x3c
// NOTE: const int SSD1306_addr = 0x3c;
// to initialilze the address of the OLED display
#define SCREEN_I2C_ADDR 0x3C // or 0x3C
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RST_PIN -1      // Reset pin (-1 if not available)
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RST_PIN);

// adding the SD card reader
#define SCK 17
#define MISO 19
#define MOSI 23
#define CS 5

const char *file0 = "/";
File file;
SPIClass spi = SPIClass(VSPI);

const uint8_t height_width = 50;

// =====================================
// LIST DIRECTORY function
// =====================================
// including functions for SD card reader
// ORIGINAL FUNCTION for ARDUINO ONLY (aka no sorting)
// void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
// {
//     Serial.printf("Listing directory: %s\n", dirname);
//
//     File root = fs.open(dirname);
//     if (!root)
//     {
//         Serial.println("Failed to open directory");
//         return;
//     }
//     if (!root.isDirectory())
//     {
//         Serial.println("Not a directory");
//         return;
//     }
//
//     File file = root.openNextFile();
//     while (file)
//     {
//         if (file.isDirectory())
//         {
//             Serial.print("  DIR : ");
//             Serial.println(file.name());
//             if (levels)
//             {
//                 listDir(fs, file.name(), levels - 1);
//             }
//         }
//         else
//         {
//             Serial.print("  FILE: ");
//             Serial.print(file.name());
//             Serial.print("  SIZE: ");
//             Serial.println(file.size());
//         }
//         file = root.openNextFile();
//     }
// }; // end listDir function

// NEW FUNCTION for ESP32 ONLY
void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if (!root)
    {
        Serial.println("Failed to open directory");
        return;
    }
    if (!root.isDirectory())
    {
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    std::vector<String> files;
    while (file)
    {
        if (file.isDirectory())
        {
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if (levels)
            {
                listDir(fs, file.name(), levels - 1);
            }
        }
        else
        {
            files.push_back(String(file.name()) + " - " + String(file.size()) + " bytes");
        }
        file = root.openNextFile();
    }
    // Sort the files
    std::sort(files.begin(), files.end());

    // Print the sorted files
    for (const String &file : files)
    {
        Serial.println(file);
    }
}; // end listDir function

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

// must have this so that LoadBinFile can be called by other functions
#include "charArrayNonAnim.h"
#include "charArrayAnim.h"

charArrayNonAnim charNoAnimation;
charArrayAnim charAnimation;

// ==================================
// ONE TIME MANDATORY FUNCTION - DO NOT REMOVE
// ==================================
void setup(void)
{
    pinMode(LED_BUILTIN, OUTPUT); // relying on GPIO2 LED to light up on MB

    Serial.begin(115200);
    Serial.println("Starting setup");

    // for U8G2 library setup
    u8g2.begin();
    u8g2.clear();
    u8g2.setFont(u8g2_font_profont10_tf);
    oled_LineH = u8g2.getFontAscent() + u8g2.getFontAscent();

    // for Adafruit library setup
    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_I2C_ADDR);
    display.clearDisplay();
    
    spi.begin(SCK, MISO, MOSI, CS);

    // for SD card setup
    if (!SD.begin(5))
    {
        Serial.println("Card Mount Failed");
        return;
    }
    
    uint8_t cardType = SD.cardType();

    if (cardType == CARD_NONE)
    {
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC)
    {
        Serial.println("MMC");
    }
    else if (cardType == CARD_SD)
    {
        Serial.println("SDSC");
    }
    else if (cardType == CARD_SDHC)
    {
        Serial.println("SDHC");
    }
    else
    {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
    Serial.println("Setup complete");
} // end setup function

// ==================================
// REPETITIVE MANDATORY FUNCTION - DO NOT REMOVE
// ==================================
void loop(void)
{
    Serial.println("looking for animation files");
    // Commands for SD card reader
    // listDir(SD, "/", 0);
    // createDir(SD, "/mydir");
    // removeDir(SD, "/mydir");
    // writeFile(SD, "/hello.txt", "Hello ");
    // appendFile(SD, "/hello.txt", "World!\n");
    // readFile(SD, "/hello.txt");
    // deleteFile(SD, "/foo.txt");
    // renameFile(SD, "/hello.txt", "/foo.txt");
    // readFile(SD, "/foo.txt");
    // testFileIO(SD, "/test.txt");
    listDir(SD, file0, 0);

    bLED = !bLED; // toggle LED State
    digitalWrite(LED_BUILTIN, bLED);
    Serial.println("Starting non-Animations");
    // calling the function from charArrayNonAnim.h
    // this one cycles through all the char array non-animations
    charNoAnimation.charArray_NonAnim(); // this works when in a class

    bLED = !bLED; // toggle LED State
    digitalWrite(LED_BUILTIN, bLED);
    Serial.println("Starting Animations");
    // calling the function from charArrayAnim.h
    // this one cycles through all the char array animations
    charAnimation.verifyAllAnimations();

    bLED = !bLED; // toggle LED State
    digitalWrite(LED_BUILTIN, bLED);
    Serial.println("Starting an individual Non-Animation");
    // CPLUSPLUSLogo(); // this works not in a class
    charNoAnimation.CPLUSPLUSLogo(); // this works when in a class
    delay(1000);                     // used for single graphics display

    bLED = !bLED; // toggle LED State
    digitalWrite(LED_BUILTIN, bLED);
    Serial.println("Starting an individual Animation");
    charAnimation.download_ANIMATION();
} // end loop function

// =========================================================
// END OF PROGRAM
// =========================================================
