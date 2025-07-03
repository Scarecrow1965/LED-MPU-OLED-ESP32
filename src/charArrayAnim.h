// +-------------------------------------------------------------
//
// Equipment:
// ESP32, OLED SSD1306
//
// File: charArrayAnim.h
//
// Description:
//
// Creates graphical displays on the SSD1306 OLED Display of char
// arrays which holds animated graphics on the ESP 32 platform
//
// History:     16-Nov-2023     Scarecrow1965   Created
//
// +-------------------------------------------------------------

// install ibraries
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "seeOLED.h"

extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
extern uint8_t oled_LineH;

extern bool bLED;

extern Adafruit_SSD1306 display;

class charArrayAnim
{

private:
    /* data */
    
    const int totalcharAnimArray = 5;
    uint8_t *charAnimationsBuffer;
    size_t charAnimationsSize;
    const int arraySize1 = 288;
    const int arraySize2 = 350;

    struct Animation
    {
        const String name;
        const uint8_t width;
        const uint8_t height;
        const char *fileName;
        const int dataLength;
    };

    // char array animations short filename array for Arduino
    Animation charArrayAnimation[5] {
        {"Ambulance", height_width2, height_width2, "/ch_ambAn.bin", arraySize2},       // charArrayAnimation[0]: {"Ambulance", 50, 50, "/ch_ambAn.bin", 350},
        {"Download", height_width1, height_width1, "/ch_dwnAn.bin", arraySize1},      // charArrayAnimation[1]: {"Download", 48, 48, "/ch_dwnAn.bin", 288},
        {"Upload", height_width1, height_width1, "/ch_uplAn.bin", arraySize1},        // charArrayAnimation[2]: {"Upload", 48, 48, "/ch_uplAn.bin", 288},
        {"Black Heartbeat", height_width2, height_width2, "/ch_hbkAn.bin", arraySize2}, // charArrayAnimation[3]: {"Black Heartbeat", 50, 50, "/ch_hbkAn.bin", 350},
        {"White Heartbeat", height_width2, height_width2, "/ch_hbwAn.bin", arraySize2}, // charArrayAnimation[4]: {"White Heartbeat", 50, 50, "/ch_hbwAn.bin", 350},
    };

    // char array animations long filename array for ESP 32
    // Animation charArrayAnimation[5]{
    //     {ambulance_Anim[0], height_width2, height_width2, "Ambulance", sizeof(ambulance_Anim[0]), "/ambulance_Anim.bin", 350},
    //     {download_Anim[0], height_width1, height_width1, "Download", sizeof(download_Anim[0]), "/download_Anim.bin", 288},
    //     {upload_Anim[0], height_width1, height_width1, "Upload", sizeof(upload_Anim[0]), "/upload_Anim.bin", 288},
    //     {heartbeat1_Anim[0], height_width2, height_width2, "Black Heartbeat", sizeof(heartbeat1_Anim[0]), "/heartbeat_black_kAnim.bin", 350},
    //     {heartbeat2_Anim[0], height_width2, height_width2, "White Heartbeat", sizeof(heartbeat2_Anim[0]), "/heartbeat_white_Anim.bin", 350},
    // };

    // end private data

public:
    // charArrayAnim(/* args */);
    charArrayAnim()
    {
        Serial.println("Char Array Animation Constructor called");
    }; // end to calling an individual's animation function

    // Destructor
    ~charArrayAnim()
    {
        // Serial.println("Char Array Animation Destructor completed.");
        display.clearDisplay();
        u8g2.clearBuffer();
    }; // end to clearing memory of the class charArrayAnim function

    unsigned long getFileSize(const char *fileName)
    {
        File file = SD.open(fileName);
        if (!file)
        {
            // Serial.println("Failed to open file");
            return 0;
        }
        unsigned long size = file.size();
        file.close();
        // Serial.print("File size: ");
        return size;
    };

    // function to provide individual animation
    void charArray_Animate2(int i)
    {
        unsigned long size = getFileSize(charArrayAnimation[i].fileName);
        if (size == 0)
        {
            Serial.println("Could not open file");
            return;
        }
        // Serial.print("Size of ");
        // Serial.print(charArrayAnimation[i].fileName);
        // Serial.print(": ");
        // Serial.println(size);

        File file = SD.open(charArrayAnimation[i].fileName);
        if (!file)
        {
            Serial.println("Failed to open file");
            return;
        }

        Serial.println("Opening file: " + String(charArrayAnimation[i].fileName));

        // creating a buffer memory
        uint8_t *bitmap = new uint8_t[charArrayAnimation[i].width * charArrayAnimation[i].height];

        // Serial.println("Char Array Animation Constructor called"); // used for testing purposes only
        for (int count = 0; count < frameCount; count++)
        {
            // clear display
            display.clearDisplay();

            // Seek to the correct position in the file
            file.seek(count * charArrayAnimation[i].dataLength);

            // // Read one frame from the file into the bitmap
            file.read(bitmap, charArrayAnimation[i].dataLength);

            // draw the frame
            display.drawBitmap(0, 17, bitmap, charArrayAnimation[i].width, charArrayAnimation[i].height, 1);

            // display the frame
            display.display();

            // Serial.print("Frame count: "); // used for testing purposes only
            // Serial.println(count); // used for testing purposes only
        }
        delete[] bitmap;
        file.close();
    }; // end to calling an individual's animation function
    void ambulance_ANIMATION()
    {
        // charArrayAnimation[0]: {"Ambulance", 50, 50, "/ch_ambAn.bin", 350},
        charArray_Animate2(0);
    }; // end to display the ambulance animation function ONLY

    void upload_ANIMATION()
    {
        // charArrayAnimation[2]: {"Upload", 48, 48, "/ch_uplAn.bin", 288},
        charArray_Animate2(2);
    }; // end to display the upload animation function ONLY

    void download_ANIMATION()
    {
        // charArrayAnimation[1]: {"Download", 48, 48, "/ch_dwnAn.bin", 288},
        charArray_Animate2(1);
    }; // end to display the download animation function ONLY

    void heartbeat1_ANIMATION()
    {
        // charArrayAnimation[3]: {"Black Heartbeat", 50, 50, "/ch_hbkAn.bin", 350},
        charArray_Animate2(3);
    }; // end to display black heartbeat animation function ONLY

    void heartbeat2_ANIMATION()
    {
        // charArrayAnimation[4]: {"White Heartbeat", 50, 50, "/ch_hbwAn.bin", 350},
        charArray_Animate2(4);
    }; // end to display white heartbeat animation function ONLY

    // ============================
    // reading and displaying char array animations from file
    // ============================
    void charArrayAnimate(const Animation &animation)
    {
        File file = SD.open(animation.fileName);
        if (!file)
        {
            Serial.println("Failed to open file");
            return;
        }
        // Serial.println("Opening file: " + String(animation.fileName));

        // creating a buffer memory
        uint8_t *bitmap = new uint8_t[animation.width * animation.height];

        // Serial.println("Char Array Animation Constructor called"); // used for testing purposes only
        for (int count = 0; count < frameCount; count++)
        {
            // clear display
            display.clearDisplay();
            u8g2.clearBuffer();
            // start the display set up
            u8g2.home();
            u8g2.setCursor(3, oled_LineH * 1 + 2);
            u8g2.print(animation.name);
            u8g2.sendBuffer();

            // Seek to the correct position in the file
            file.seek(count * animation.dataLength);

            // // Read one frame from the file into the bitmap
            file.read(bitmap, animation.dataLength);

            // draw the frame
            display.drawBitmap(0, 17, bitmap, animation.width, animation.height, 1);

            // display the frame
            display.display();

            // Serial.print("Frame count: "); // used for testing purposes only
            // Serial.println(count); // used for testing purposes only
        }
        delete[] bitmap;
        file.close();
    }; // end to calling an individual's animation function

    // ============================
    // Verifying (reading and displaying) char array animations from file
    // ============================
    void verifyAllAnimations()
    {
        for (uint8_t i = 0; i < totalcharAnimArray; i++)
        {
            // reading and Displaying the animation
            charArrayAnimate(charArrayAnimation[i]);
        }
    }; // end of verifyAllAnimations function

}; // end class charArrayAnim

// charArrayAnim::charArrayAnim(/* args */)
// {
// }

// charArrayAnim::~charArrayAnim()
// {
// }

// charArrayAnim charAnimation; // need an object to call the function from charArrayAnim.h

// =========================================================
// END OF PROGRAM
// =========================================================