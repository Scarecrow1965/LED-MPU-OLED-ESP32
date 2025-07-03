// +-------------------------------------------------------------
//
// Equipment:
// Arduino Mega, MPU-6050, OLED SSD1306, WS2812B Addressable LEDS x3,
// 4-relay module, 2 linear actuators, RT-11(JCHT35K9) remote/handset,
// and 650W 12V Power supply
//
// File: LEDeffects.h
//
// Description:
//
// Creates effects on addressable LEDs unto my workstation desk
//
// History:     7-Dec-2023     Scarecrow1965   Created
//
// +-------------------------------------------------------------

#ifndef LEDEFFECTS_H
#define LEDEFFECTS_H

// ================
// install ibraries
// ================
#include <Arduino.h>
#include <Wire.h>

// for addressable LED strips
#define FASTLED_INTERNAL // Suppress build banner
#include <FastLED.h>

#include "standupTableMovement.h"
// since the below function is defined in the standupTableMovement.h file
void checkMPUData(bool &);

// ====================================
// Setup for the addressable LED Strips
// ====================================
// definitions of assets and parameters for ESP32
// #define LED_PIN0 13 // Test Bench = option #1: GPIO25(pin 8), option #2:GPIO13(pin 3) for the ESP32
// #define LED_PIN1 12 // Standup Desk = option #1: GPIO26(pin 7), option #2:GPIO12(pin 4) for the ESP32
// #define LED_PIN2 14 // Storage Bench = option #1: GPIO27(pin 6), option #2:GPIO14(pin 5) for the ESP32

// Define the number of LEDs in each strip
#define NUM_LED_PIN0 278 // Test Bench
#define NUM_LED_PIN1 222 // Standup Desk
#define NUM_LED_PIN2 242 // Storage Bench
// extern int NUM_LED_PIN0; // Test Bench
// extern int NUM_LED_PIN1; // Standup Desk
// extern int NUM_LED_PIN2; // Storage Bench

// #define BRIGHTNESS 32    // 0 to 255 of the LEDs brightness
extern int BRIGHTNESS;
// #define LED_TYPE WS2812B // type works for the WS2815 which I am using
// #define COLOR_ORDER RGB

// // Define the LED arrays for each strip
// CRGB led_Strip0[NUM_LED_PIN0]; // Test Bench
// CRGB led_Strip1[NUM_LED_PIN1]; // Standup Desk
// CRGB led_Strip2[NUM_LED_PIN2]; // Storage Bench
extern CRGB led_Strip0[];
extern CRGB led_Strip1[];
extern CRGB led_Strip2[];

// CRGBPalette16 means we are using 16 colour variables
// Part of the FastLED libraries
CRGBPalette16 currentPalette;
TBlendType currentBlending;

// to randomize the LED light show
uint8_t startIndex = 0; // setting things up for the various Palette functions
int effectIndices[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
typedef void (*EffectFunction)(void);
typedef void (*EffectFunctionWithArgs)(uint8_t, uint8_t, uint8_t);
extern bool ledEffectActive;
extern bool gyroOrAccelChanged;
extern bool dataChanged;

// ========================================
// Random Lighting Effects
// ========================================
// to test out the LEDs when starting
void ledStripTest(CRGB *leds, int numLEDs)
{
    FastLED.clear();

    FastLED.show();
    for (int i = 0; i < numLEDs; i++)
    {
        leds[i] = CRGB::White; // set our current dot to red
        FastLED.show();
        leds[i] = CRGB::Black; // set our current dot to black before we continue
    }
};

// UnicornPuke (RBGers know this) LED Lighting Effect
void unicornPuke2()
{
    FastLED.clear();
    int effects = 30; // 30 seconds of display
    while (effects > 0)
    {
        checkMPUData(dataChanged);
        if (dataChanged)
        {
            // Reset dataChanged to false when you've handled the change
            dataChanged = false;
            // Data has changed, take appropriate action
            // For example, call another function or update your LED effects
            return;
        }

        // Update each LED with a random color
        for (int i = 0; i < NUM_LED_PIN0; i++)
        {
            led_Strip0[i] = CRGB(random(256), random(256), random(256));
        }
        for (int j = 0; j < NUM_LED_PIN1; j++)
        {
            led_Strip1[j] = CRGB(random(256), random(256), random(256));
        }
        for (int k = 0; k < NUM_LED_PIN2; k++)
        {
            led_Strip2[k] = CRGB(random(256), random(256), random(256));
        }
        FastLED.show();
        effects--;
    }
}; // end unicorn puke ver 2 function

// Cylon Demo function
void fadeall()
{
    for (int i = 0; i < NUM_LED_PIN1; i++)
    {
        led_Strip0[i].nscale8(200);
        led_Strip1[i].nscale8(200);
        led_Strip2[i].nscale8(200);
    }
}; // end fade all function -> tied to cylon demo function

void cylondemo()
{
    FastLED.clear();
    // First slide the led in one direction
    for (int i = 0; i < NUM_LED_PIN1; i++)
    {
        checkMPUData(dataChanged);
        if (dataChanged)
        {
            // Reset dataChanged to false when you've handled the change
            dataChanged = false;
            // Data has changed, take appropriate action
            // For example, call another function or update your LED effects
            return;
        }

        // Set the i'th led to red
        led_Strip0[i] = CRGB::Red;
        led_Strip1[i] = CRGB::Red;
        led_Strip2[i] = CRGB::Red;
        // Show the leds
        FastLED.show();
        // now that we've shown the leds, reset the i'th led to black
        fadeall();
        // Wait a little bit before we loop around and do it again
        delay(5);
    }

    // Now go in the other direction.
    for (int i = NUM_LED_PIN1 - 1; i >= 0; i--)
    {
        checkMPUData(dataChanged);
        if (dataChanged)
        {
            // Reset dataChanged to false when you've handled the change
            dataChanged = false;
            // Data has changed, take appropriate action
            // For example, call another function or update your LED effects
            return;
        }

        // Set the i'th led to red
        led_Strip0[i] = CRGB::Red;
        led_Strip1[i] = CRGB::Red;
        led_Strip2[i] = CRGB::Red;
        // Show the leds
        FastLED.show();
        // now that we've shown the leds, reset the i'th led to black
        fadeall();
        // Wait a little bit before we loop around and do it again
        delay(5);
    }
}; // end cylon demo function

// blinking Primary Colors ver 2
void blinkingPrimaryColors2()
{
    const int blinkInterval = 250; // Blink interval in milliseconds
    // Turn off all LEDs
    FastLED.clear();
    int effects = 15; // 15 seconds of display
    while (effects > 0)
    {
        checkMPUData(dataChanged);
        if (dataChanged)
        {
            // Reset dataChanged to false when you've handled the change
            dataChanged = false;
            // Data has changed, take appropriate action
            // For example, call another function or update your LED effects
            return;
        }

        // For example, alternating red, green, and blue lights blinking
        static int colorIndex = 0;
        // Set the current color
        CRGB currentColor;
        // Alternating primary colors (Red, Green, Blue)
        if (colorIndex == 0)
        {
            currentColor = CRGB::Red;
            colorIndex = 1;
        }
        else if (colorIndex == 1)
        {
            currentColor = CRGB::Green;
            colorIndex = 2;
        }
        else if (colorIndex == 2)
        {
            currentColor = CRGB::Blue;
            colorIndex = 0;
        }
        // Blink all LEDs with the current color
        fill_solid(led_Strip0, NUM_LED_PIN0, currentColor);
        fill_solid(led_Strip1, NUM_LED_PIN1, currentColor);
        fill_solid(led_Strip2, NUM_LED_PIN2, currentColor);
        FastLED.show();
        delay(blinkInterval);
        // Turn off all LEDs
        FastLED.clear();
        FastLED.show();
        delay(blinkInterval);
        effects--;
    }
}; // end blinkingPrimaryColors ver 2 function

// Christmas LED Lighting effect ver 2
void christmasTheme2()
{
    const int twinkleProbability = 50; // Probability of twinkle effect (out of 100)
    const int twinkleDelay = 200;      // Delay between twinkle effect in milliseconds
    int effects = 40;                  // delay for about 30ish seconds
    // Turn off all LEDs
    FastLED.clear();
    // Set colors for Christmas theme
    CRGB redColor = CRGB::Red;
    CRGB greenColor = CRGB::Green;
    CRGB whiteColor = CRGB::White;
    while (effects >= 0)
    {
        checkMPUData(dataChanged);
        if (dataChanged)
        {
            // Reset dataChanged to false when you've handled the change
            dataChanged = false;
            // Data has changed, take appropriate action
            // For example, call another function or update your LED effects
            return;
        }

        // Light up LEDs with alternating red and green colors
        for (int i = 0; i < NUM_LED_PIN0; i += 2)
        {
            led_Strip0[i] = redColor;
            led_Strip0[i + 1] = greenColor;
        }
        for (int j = 0; j < NUM_LED_PIN1; j += 2)
        {
            led_Strip1[j] = redColor;
            led_Strip1[j + 1] = greenColor;
        }
        for (int k = 0; k < NUM_LED_PIN2; k += 2)
        {
            led_Strip2[k] = redColor;
            led_Strip2[k + 1] = greenColor;
        }

        // Add twinkle effect
        if (random(100) < twinkleProbability)
        {
            int randomLED0 = random(NUM_LED_PIN0);
            int randomLED1 = random(NUM_LED_PIN1);
            int randomLED2 = random(NUM_LED_PIN2);
            led_Strip0[randomLED0] = whiteColor;
            led_Strip1[randomLED1] = whiteColor;
            led_Strip2[randomLED2] = whiteColor;
            delay(twinkleDelay);
        }
        FastLED.show();
        delay(1);
        effects--;
    }
}; // end Christmmas theme ver 2 function

//  displays a fading blue light
void fadingBlue()
{
    FastLED.clear();
    static boolean pulse = 0;
    int effects = 120; // delay for about 10 seconds
    while (effects >= 0)
    {
        checkMPUData(dataChanged);
        if (dataChanged)
        {
            // Reset dataChanged to false when you've handled the change
            dataChanged = false;
            // Data has changed, take appropriate action
            // For example, call another function or update your LED effects
            return;
        }

        EVERY_N_MILLISECONDS_I(timingFade, 1)
        {
            pulse = !pulse;
            if (pulse)
            {
                timingFade.setPeriod(700); // time to hold before fading
                for (int i = 0; i < NUM_LED_PIN0; i++)
                {
                    led_Strip0[i] = CRGB::DarkBlue;
                }
                for (int j = 0; j < NUM_LED_PIN1; j++)
                {
                    led_Strip1[j] = CRGB::DarkBlue;
                }
                for (int k = 0; k < NUM_LED_PIN2; k++)
                {
                    led_Strip2[k] = CRGB::DarkBlue;
                }
            }
            else
            {
                timingFade.setPeriod(1000);
            }
        }
        if (pulse == 0)
        {
            EVERY_N_MILLISECONDS(10)
            {
                for (int i = 0; i < NUM_LED_PIN0; i++)
                {
                    led_Strip0[i].fadeToBlackBy(12); // fade out
                }
                for (int j = 0; j < NUM_LED_PIN1; j++)
                {
                    led_Strip1[j].fadeToBlackBy(12); // fade out
                }
                for (int k = 0; k < NUM_LED_PIN2; k++)
                {
                    led_Strip2[k].fadeToBlackBy(12); // fade out
                }
            }
        }
        FastLED.show();
        delay(1);
        effects--;
    }
}; // end fading blue light function

// Red Alert version 2 with fading out red light
void redAlert2()
{
    static boolean pulse = 0;
    int effects = 120; // delay for about 10 seconds
    while (effects >= 0)
    {
        checkMPUData(dataChanged);
        if (dataChanged)
        {
            // Reset dataChanged to false when you've handled the change
            dataChanged = false;
            // Data has changed, take appropriate action
            // For example, call another function or update your LED effects
            return;
        }

        EVERY_N_MILLISECONDS_I(timingFade, 1)
        {
            pulse = !pulse;
            if (pulse)
            {
                timingFade.setPeriod(750); // time to hold before fading
                for (int i = 0; i < NUM_LED_PIN0; i++)
                {
                    led_Strip0[i] = CRGB::Red;
                }
                for (int j = 0; j < NUM_LED_PIN1; j++)
                {
                    led_Strip1[j] = CRGB::Red;
                }
                for (int k = 0; k < NUM_LED_PIN2; k++)
                {
                    led_Strip2[k] = CRGB::Red;
                }
            }
            else
            {
                timingFade.setPeriod(1000);
            }
        }
        if (pulse == 0)
        {
            EVERY_N_MILLISECONDS(10)
            {
                for (int i = 0; i < NUM_LED_PIN0; i++)
                {
                    led_Strip0[i].fadeToBlackBy(12); // fade out
                }
                for (int j = 0; j < NUM_LED_PIN1; j++)
                {
                    led_Strip1[j].fadeToBlackBy(12); // fade out
                }
                for (int k = 0; k < NUM_LED_PIN2; k++)
                {
                    led_Strip2[k].fadeToBlackBy(12); // fade out
                }
            }
        }
        FastLED.show();
        delay(1);
        effects--;
    }
}; // end of red alert ver 2 fading red light

// Random sparkles in LED Strips
void sparkles(uint8_t sparkel_duration, uint8_t sparkel_amount, uint8_t sparkel_spread)
{
    int effects = 30; // delay for about 30 seconds
    FastLED.clear();
    static uint8_t sparkle_pixel0;
    static uint8_t sparkle_pixel1;
    static uint8_t sparkle_pixel2;

    while (effects > 0)
    {
        checkMPUData(dataChanged);
        if (dataChanged)
        {
            // Reset dataChanged to false when you've handled the change
            dataChanged = false;
            // Data has changed, take appropriate action
            // For example, call another function or update your LED effects
            return;
        }

        EVERY_N_MILLISECONDS_I(timingObj, 1)
        {
            timingObj.setPeriod(sparkel_duration);
            led_Strip0[sparkle_pixel0] = CRGB::Black;
            led_Strip1[sparkle_pixel1] = CRGB::Black;
            led_Strip2[sparkle_pixel2] = CRGB::Black;
            uint8_t previous_pixel0 = sparkle_pixel0;
            uint8_t previous_pixel1 = sparkle_pixel1;
            uint8_t previous_pixel2 = sparkle_pixel2;

            while (previous_pixel0 == sparkle_pixel0)
            { // pixel can't repeat
                sparkle_pixel0 = random8(NUM_LED_PIN0 - 23);
                // random8() can go only up to 255 and LED_Strip0 has 278 LEDs
            }
            while (previous_pixel1 == sparkle_pixel1)
            { // pixel can't repeat
                sparkle_pixel1 = random8(NUM_LED_PIN1);
            }
            while (previous_pixel2 == sparkle_pixel2)
            { // pixel can't repeat
                sparkle_pixel2 = random8(NUM_LED_PIN2);
            }
            if (random8(100) < sparkel_amount)
            {
                // leds[sparkle_pixel] = CRGB(random8(), random8(), random8());
                led_Strip0[sparkle_pixel0] = CHSV(random8(), random8(20, 200), random8(50, 255));
                led_Strip1[sparkle_pixel1] = CHSV(random8(), random8(20, 200), random8(50, 255));
                led_Strip2[sparkle_pixel2] = CHSV(random8(), random8(20, 200), random8(50, 255));
                FastLED.show();
            }
            FastLED.show();
        }
        EVERY_N_MILLISECONDS(10)
        {
            // fadeToBlackBy(leds, NUM_LEDS, 1);  // fade out a bit over time
            blur1d(led_Strip0, NUM_LED_PIN0, sparkel_spread); // spreads and fades out color over time
            blur1d(led_Strip1, NUM_LED_PIN1, sparkel_spread); // spreads and fades out color over time
            blur1d(led_Strip2, NUM_LED_PIN2, sparkel_spread); // spreads and fades out color over time
            FastLED.show();
        }
        effects--;
    }
}; // end sparkles function

// ==============================================================
// Fill LEDs from a Palette.
// This function will load several times with differernt palettes
// ==============================================================
void fillLEDsFromPaletteColours(uint8_t colorIndex)
{
    int effects = 30; // delay for about 30 seconds
    while (effects > 0)
    {
        checkMPUData(dataChanged);
        if (dataChanged)
        {
            // Reset dataChanged to false when you've handled the change
            dataChanged = false;
            // Data has changed, take appropriate action
            // For example, call another function or update your LED effects
            return;
        }

        for (int i = 0; i < NUM_LED_PIN0; i++)
        {
            led_Strip0[i] = ColorFromPalette(currentPalette, colorIndex, BRIGHTNESS, currentBlending);
            colorIndex += 3;
        }
        for (int j = 0; j < NUM_LED_PIN1; j++)
        {
            led_Strip1[j] = ColorFromPalette(currentPalette, colorIndex, BRIGHTNESS, currentBlending);
            colorIndex += 3;
        }
        for (int k = 0; k < NUM_LED_PIN2; k++)
        {
            led_Strip2[k] = ColorFromPalette(currentPalette, colorIndex, BRIGHTNESS, currentBlending);
            colorIndex += 3;
        }
        FastLED.show();
        delay(500);
        effects--;
    }
}; // end fill LED from palette colours function

// random colours palette generator
void randomPalette(uint8_t startIndex)
{
    for (int i = 0; i < 16; i++)
    {
        currentPalette[i] = CHSV(random8(), 255, random8());
    }
    fillLEDsFromPaletteColours(startIndex);
}; // end random colour palette function

// black and white striped colours palette
void blackWhiteStripePalette(uint8_t startIndex)
{
    // 'black out' all 16 palette entries...
    fill_solid(currentPalette, 16, CRGB::Black);
    // and set every fourth one to white.
    currentPalette[0] = CRGB::White;
    currentPalette[4] = CRGB::White;
    currentPalette[8] = CRGB::White;
    currentPalette[12] = CRGB::White;
    fillLEDsFromPaletteColours(startIndex);
}; // end black white stripe palette function

// purple and green colours palette
void purpleGreenPalette(uint8_t startIndex)
{
    CRGB purple = CHSV(HUE_PURPLE, 255, 255);
    CRGB green = CHSV(HUE_GREEN, 255, 255);
    CRGB black = CRGB::Black;
    currentPalette = CRGBPalette16(
        green, green, black, black,
        purple, purple, black, black,
        green, green, black, black,
        purple, purple, black, black);
    fillLEDsFromPaletteColours(startIndex);
}; // end purple and green palette function

// red, white, and blue colours palette
void redWhiteBluePalette(uint8_t startIndex)
{
    // cannot be used within a function but can be housed before the setup function
    // extern CRGBPalette16 myRedWhiteBluePalette;
    // extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

    // This example shows how to set up a static color palette
    // which is stored in PROGMEM (flash), which is almost always more
    // plentiful than RAM.  A static PROGMEM palette like this
    // takes up 64 bytes of flash.
    const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM = {
        CRGB::Red,
        CRGB::Gray, // 'white' is too bright compared to red and blue
        CRGB::Blue,
        CRGB::Black,

        CRGB::Red,
        CRGB::Gray,
        CRGB::Blue,
        CRGB::Black,

        CRGB::Red,
        CRGB::Red,
        CRGB::Gray,
        CRGB::Gray,
        CRGB::Blue,
        CRGB::Blue,
        CRGB::Black,
        CRGB::Black};

    currentPalette = myRedWhiteBluePalette_p;
    currentBlending = NOBLEND;
    fillLEDsFromPaletteColours(startIndex);
}; // end red white and blue colour palette function

// raindow colurs palette
void rainbowPalette(uint8_t startIndex)
{
    // currentPalette is a Template housed in FastLED libraries: colorpalettes.cpp
    currentPalette = RainbowColors_p;
    currentBlending = NOBLEND;
    fillLEDsFromPaletteColours(startIndex);
}; // end rainbow colour palette function (kinda like rainbow puke)

// rainbow striped colours palette
void rainbowStripePalette(uint8_t startIndex)
{
    // currentPalette is a Template housed in FastLED libraries: colorpalettes.cpp
    currentPalette = RainbowStripeColors_p;
    currentBlending = NOBLEND;
    fillLEDsFromPaletteColours(startIndex);
}; // end rainbow stripe palette function

// clouds colours palette
void cloudColoursPalette(uint8_t startIndex)
{
    // currentPalette is a Template housed in FastLED libraries: colorpalettes.cpp
    currentPalette = CloudColors_p;
    currentBlending = NOBLEND;
    fillLEDsFromPaletteColours(startIndex);
}; // end clouds colour palette function

// party colours palette
void partyColoursPalette(uint8_t startIndex)
{
    // currentPalette is a Template housed in FastLED libraries: colorpalettes.cpp
    currentPalette = PartyColors_p;
    currentBlending = NOBLEND;
    fillLEDsFromPaletteColours(startIndex);
}; // end party colour palette function

// lava colours palette
void lavaColoursPalette(uint8_t startIndex)
{
    // currentPalette is a Template housed in FastLED libraries: colorpalettes.cpp
    currentPalette = LavaColors_p;
    currentBlending = NOBLEND;
    fillLEDsFromPaletteColours(startIndex);
}; // end laval colours palette function

// forest colours palette
void forestColoursPalette(uint8_t startIndex)
{
    // currentPalette is a Template housed in FastLED libraries: colorpalettes.cpp
    currentPalette = ForestColors_p;
    currentBlending = NOBLEND;
    fillLEDsFromPaletteColours(startIndex);
}; // end forest colour palette function

// ocean colours palette
void oceanColoursPalette(uint8_t startIndex)
{
    // currentPalette is a Template housed in FastLED libraries: colorpalettes.cpp
    currentPalette = OceanColors_p;
    currentBlending = NOBLEND;
    fillLEDsFromPaletteColours(startIndex);
}; // end ocean colour palette function

// heat colours palette
void heatColoursPalette(uint8_t startIndex)
{
    // currentPalette is a Template housed in FastLED libraries: colorpalettes.cpp
    currentPalette = HeatColors_p;
    currentBlending = NOBLEND;
    fillLEDsFromPaletteColours(startIndex);
}; // end heat colour palette function

// wrapper functions to be used for LED function randomizer
void unicornPuke2Wrapper()
{
    // Serial.println("Starting Unicorn Puke ver 2"); // used for testing purposes only
    unicornPuke2();
}; // end wrapper functions for unicorn puke LED effectsq
void cylondemoWrapper()
{
    // Serial.println("Starting Cylon Demo"); // used for testing purposes only
    cylondemo();
}; // end wrapper functions for cylon LED effects
void blinkingPrimaryColors2Wrapper()
{
    // Serial.println("Starting Blinking Colours ver 2"); // used for testing purposes only
    blinkingPrimaryColors2();
}; // end wrapper functions for primary colour blinking LED effects
void christmasTheme2Wrapper()
{
    // Serial.println("Starting Christmas Theme ver 2"); // used for testing purposes only
    christmasTheme2();
}; // end wrapper functions for christmas theme LED effects
void fadingBlueWrapper()
{
    // Serial.println("Starting Fading Blue Light"); // used for testing purposes only
    fadingBlue();
}; // end wrapper functions for fading blue LED effects
void redAlert2Wrapper()
{
    // Serial.println("RED ALERT VERSION 2!!!"); // used for testing purposes only
    redAlert2();
}; // end wrapper functions for red alert LED effects
void sparklesWrapper()
{
    // Serial.print("Starting sparkles"); // used for testing purposes only
    sparkles(random8(80, 200), 100, 30);
}; // end wrapper functions for sparkles

void rainbowPaletteWrapper()
{
    // Serial.println("Starting Rainbow Palette"); // used for testing purposes only
    rainbowPalette(startIndex);
}; // end wrapper functions for rainbow palette
void rainbowStripePaletteWrapper()
{
    // Serial.println("Starting Rainbow Stripe Palette"); // used for testing purposes only
    rainbowStripePalette(startIndex);
}; // end wrapper functions for rainbow stripe palette
void randomPaletteWrapper()
{
    // Serial.println("Starting Random Palette"); // used for testing purposes only
    randomPalette(startIndex);
}; // end wrapper functions for random palette
void blackWhiteStripePaletteWrapper()
{
    // Serial.println("Starting Black and White Stripe Palette"); // used for testing purposes only
    blackWhiteStripePalette(startIndex);
}; // end wrapper functions for black and white stripe palette
void purpleGreenPaletteWrapper()
{
    // Serial.println("Starting Purple and Green Palette"); // used for testing purposes only
    purpleGreenPalette(startIndex);
}; // end wrapper functions for purple and green palette
void cloudColoursPaletteWrapper()
{
    // Serial.println("Starting Cloud Colours"); // used for testing purposes only
    cloudColoursPalette(startIndex);
}; // end wrapper functions for cloud colours palette
void partyColoursPaletteWrapper()
{
    // Serial.println("Starting Party Colours"); // used for testing purposes only
    partyColoursPalette(startIndex);
}; // end wrapper functions for party colours palette
void redWhiteBluePaletteWrapper()
{
    // Serial.println("Starting Red White and Blue Colours"); // used for testing purposes only
    redWhiteBluePalette(startIndex);
}; // end wrapper functions for red white and blue colours palette
void heatColoursPaletteWrapper()
{
    // Serial.println("Starting Heat Colours"); // used for testing purposes only
    heatColoursPalette(startIndex);
}; // end wrapper functions for heat colours palette
void oceanColoursPaletteWrapper()
{
    // Serial.println("Starting Ocean Colours"); // used for testing purposes only
    oceanColoursPalette(startIndex);
}; // end wrapper functions for ocean colours palette
void forestColoursPaletteWrapper()
{
    // Serial.println("Starting Forest Colours"); // used for testing purposes only
    forestColoursPalette(startIndex);
}; // end wrapper functions for forest colours palette
void lavaColoursPaletteWrapper()
{
    // Serial.println("Starting Lava Colours"); // used for testing purposes only
    lavaColoursPalette(startIndex);
}; // end wrapper functions for lava colours palette

// struct created to evaluate each function's variable requirement
struct Effect
{
    EffectFunction func;
    EffectFunctionWithArgs funcWithArgs;
    uint8_t arg1;
    uint8_t arg2;
    uint8_t arg3;
};

// Create an array of effects
Effect effects[] = {
    {unicornPuke2Wrapper, nullptr, 0, 0, 0},
    {cylondemoWrapper, nullptr, 0, 0, 0},
    {blinkingPrimaryColors2Wrapper, nullptr, 0, 0, 0},
    {christmasTheme2Wrapper, nullptr, 0, 0, 0},
    {fadingBlueWrapper, nullptr, 0, 0, 0},
    {redAlert2Wrapper, nullptr, 0, 0, 0},
    {sparklesWrapper, 0, 0, 0},
    {rainbowPaletteWrapper, 0, 0, 0},
    {rainbowStripePaletteWrapper, 0, 0, 0},
    {randomPaletteWrapper, 0, 0, 0},
    {blackWhiteStripePaletteWrapper, 0, 0, 0},
    {purpleGreenPaletteWrapper, 0, 0, 0},
    {cloudColoursPaletteWrapper, 0, 0, 0},
    {partyColoursPaletteWrapper, 0, 0, 0},
    {redWhiteBluePaletteWrapper, 0, 0, 0},
    {heatColoursPaletteWrapper, 0, 0, 0},
    {oceanColoursPaletteWrapper, 0, 0, 0},
    {forestColoursPaletteWrapper, 0, 0, 0},
    {lavaColoursPaletteWrapper, 0, 0, 0},
    {nullptr, nullptr, 0, 0, 0} // Placeholder for ledStripTest
    // Add other effects here
}; // 19 effects so far

// shuffle array function to shuffle the effects list of LED Lights functions
void shuffleArray(int arr[], int size)
{
    for (int s = size - 1; s > 0; s--)
    {
        int t = random(0, s + 1);
        if (s != t)
        {
            int temp = arr[s];
            arr[s] = arr[t];
            arr[t] = temp;
        }
    }
}; // end shffle array function

void randomizedLEDEffect()
{
    int numEffects = sizeof(effectIndices) / sizeof(effectIndices[0]);
    // Serial.println("Shuffling LED array"); // used for testing purposes only
    // Serial.println("This is the randomized list"); // used for testing purposes only
    shuffleArray(effectIndices, numEffects); // Shuffle the effect order to create a randomized sequence

    // used for testing purposes only
    // for (int i = 0; i < numEffects; i++)
    // {
    //     Serial.print(effectIndices[i]);
    //     Serial.print(",");
    // }

    for (int i = 0; i < numEffects; i++)
    {
        checkMPUData(dataChanged);
        if (dataChanged)
        {
            // Reset dataChanged to false when you've handled the change
            dataChanged = false;
            // Data has changed, take appropriate action
            // For example, call another function or update your LED effects
            return;
        }
        int effectIndex = effectIndices[i];
        // Serial.print("Executing effect = "); // used for testing purposes only
        // Serial.println(effectIndex);         // used for testing purposes only

        if (effects[effectIndex].func)
        {
            effects[effectIndex].func();
        }
        else if (effects[effectIndex].funcWithArgs)
        {
            effects[effectIndex].funcWithArgs(effects[effectIndex].arg1, effects[effectIndex].arg2, effects[effectIndex].arg3);
        }
    }
}; // end Random LED Display function


// Predefined RGB colors from FastLED's libraries
// Colour Name          | HTML Code | HexaDecimal | Decimal         | Basic
//                      |           | Code        | Code            | Colour?
//                      |           | #RRGGBB     | (R, G, B)       |
// Black                | 000000    | 0x000000    | (0, 0, 0)       | Yes
// Navy                 | 000080    | 0x000080    | (0, 0, 128)     | Yes
// DarkBlue             | 00008B    | 0x00008B    |                 |
// MediumBlue           | 0000CD    | 0x0000CD    |                 |
// Blue                 | 0000FF    | 0x0000FF    | (0, 0, 255)     | Yes
// DarkGreen            | 006400    | 0x006400    |                 |
// Green                | 008000    | 0x008000    | (0, 128, 0)     | Yes
// Teal                 | 008080    | 0x008080    | (0, 128, 128)   | Yes
// DarkCyan             | 008B8B    | 0x008B8B    |                 |
// DeepSkyBlue          | 00BFFF    | 0x00BFFF    |                 |
// DarkTurquoise        | 00CED1    | 0x00CED1    |                 |
// MediumSpringGreen    | 00FA9A    | 0x00FA9A    |                 |
// Lime                 | 00FF00    | 0x00FF00    | (0, 255, 0)     | Yes
// SpringGreen          | 00FF7F    | 0x00FF7F    |                 |
// Aqua                 | 00FFFF    | 0x00FFFF    |                 |
// Cyan / Aqua          | 00FFFF    | 0x00FFFF    | (0, 255, 255)   | Yes
// MidnightBlue         | 191970    | 0x191970    |                 |
// DodgerBlue           | 1E90FF    | 0x1E90FF    |                 |
// LightSeaGreen        | 20B2AA    | 0x20B2AA    |                 |
// ForestGreen          | 228B22    | 0x228B22    |                 |
// SeaGreen             | 2E8B57    | 0x2E8B57    |                 |
// DarkSlateGray        | 2F4F4F    | 0x2F4F4F    |                 |
// DarkSlateGrey        | 2F4F4F    | 0x2F4F4F    |                 |
// LimeGreen            | 32CD32    | 0x32CD32    |                 |
// MediumSeaGreen       | 3CB371    | 0x3CB371    |                 |
// Turquoise            | 40E0D0    | 0x40E0D0    |                 |
// RoyalBlue            | 4169E1    | 0x4169E1    |                 |
// SteelBlue            | 4682B4    | 0x4682B4    |                 |
// DarkSlateBlue        | 483D8B    | 0x483D8B    |                 |
// MediumTurquoise      | 48D1CC    | 0x48D1CC    |                 |
// Indigo               | 4B0082    | 0x4B0082    |                 |
// DarkOliveGreen       | 556B2F    | 0x556B2F    |                 |
// CadetBlue            | 5F9EA0    | 0x5F9EA0    |                 |
// CornflowerBlue       | 6495ED    | 0x6495ED    |                 |
// MediumAquamarine     | 66CDAA    | 0x66CDAA    |                 |
// DimGray              | 696969    | 0x696969    |                 |
// DimGrey              | 696969    | 0x696969    |                 |
// SlateBlue            | 6A5ACD    | 0x6A5ACD    |                 |
// OliveDrab            | 6B8E23    | 0x6B8E23    |                 |
// SlateGray            | 708090    | 0x708090    |                 |
// SlateGrey            | 708090    | 0x708090    |                 |
// LightSlateGray       | 778899    | 0x778899    |                 |
// LightSlateGrey       | 778899    | 0x778899    |                 |
// MediumSlateBlue      | 7B68EE    | 0x7B68EE    |                 |
// LawnGreen            | 7CFC00    | 0x7CFC00    |                 |
// Chartreuse           | 7FFF00    | 0x7FFF00    |                 |
// Aquamarine           | 7FFFD4    | 0x7FFFD4    |                 |
// Maroon               | 800000    | 0x800000    | (128, 0, 0)     | Yes
// Purple               | 800080    | 0x800080    | (128, 0, 128)   | Yes
// Olive                | 808000    | 0x808000    | (128, 128, 0)   | Yes
// Gray                 | 808080    | 0x808080    | (128, 128, 128) | Yes
// Grey                 | 808080    | 0x808080    | (128, 128, 128) | Yes
// SkyBlue              | 87CEEB    | 0x87CEEB    |                 |
// LightSkyBlue         | 87CEFA    | 0x87CEFA    |                 |
// BlueViolet           | 8A2BE2    | 0x8A2BE2    |                 |
// DarkRed              | 8B0000    | 0x8B0000    |                 |
// DarkMagenta          | 8B008B    | 0x8B008B    |                 |
// SaddleBrown          | 8B4513    | 0x8B4513    |                 |
// DarkSeaGreen         | 8FBC8F    | 0x8FBC8F    |                 |
// LightGreen           | 90EE90    | 0x90EE90    |                 |
// MediumPurple         | 9370DB    | 0x9370DB    |                 |
// DarkViolet           | 9400D3    | 0x9400D3    |                 |
// PaleGreen            | 98FB98    | 0x98FB98    |                 |
// DarkOrchid           | 9932CC    | 0x9932CC    |                 |
// Amethyst             | 9966CC    | 0x9966CC    |                 |
// YellowGreen          | 9ACD32    | 0x9ACD32    |                 |
// Sienna               | A0522D    | 0xA0522D    |                 |
// Brown                | A52A2A    | 0xA52A2A    |                 |
// DarkGray             | A9A9A9    | 0xA9A9A9    |                 |
// DarkGrey             | A9A9A9    | 0xA9A9A9    |                 |
// LightBlue            | ADD8E6    | 0xADD8E6    |                 |
// GreenYellow          | ADFF2F    | 0xADFF2F    |                 |
// PaleTurquoise        | AFEEEE    | 0xAFEEEE    |                 |
// LightSteelBlue       | B0C4DE    | 0xB0C4DE    |                 |
// PowderBlue           | B0E0E6    | 0xB0E0E6    |                 |
// FireBrick            | B22222    | 0xB22222    |                 |
// DarkGoldenrod        | B8860B    | 0xB8860B    |                 |
// MediumOrchid         | BA55D3    | 0xBA55D3    |                 |
// RosyBrown            | BC8F8F    | 0xBC8F8F    |                 |
// DarkKhaki            | BDB76B    | 0xBDB76B    |                 |
// Silver               | C0C0C0    | 0xC0C0C0    | (192, 192, 192) | Yes
// MediumVioletRed      | C71585    | 0xC71585    |                 |
// Plaid                | CC5533    | 0xCC5533    |                 |
// IndianRed            | CD5C5C    | 0xCD5C5C    |                 |
// Peru                 | CD853F    | 0xCD853F    |                 |
// Chocolate            | D2691E    | 0xD2691E    |                 |
// Tan                  | D2B48C    | 0xD2B48C    |                 |
// LightGrey            | D3D3D3    | 0xD3D3D3    |                 |
// Thistle              | D8BFD8    | 0xD8BFD8    |                 |
// Orchid               | DA70D6    | 0xDA70D6    |                 |
// Goldenrod            | DAA520    | 0xDAA520    |                 |
// PaleVioletRed        | DB7093    | 0xDB7093    |                 |
// Crimson              | DC143C    | 0xDC143C    |                 |
// Gainsboro            | DCDCDC    | 0xDCDCDC    |                 |
// Plum                 | DDA0DD    | 0xDDA0DD    |                 |
// BurlyWood            | DEB887    | 0xDEB887    |                 |
// LightCyan            | E0FFFF    | 0xE0FFFF    |                 |
// Lavender             | E6E6FA    | 0xE6E6FA    |                 |
// DarkSalmon           | E9967A    | 0xE9967A    |                 |
// Violet               | EE82EE    | 0xEE82EE    |                 |
// PaleGoldenrod        | EEE8AA    | 0xEEE8AA    |                 |
// LightCoral           | F08080    | 0xF08080    |                 |
// Khaki                | F0E68C    | 0xF0E68C    |                 |
// AliceBlue            | F0F8FF    | 0xF0F8FF    |                 |
// Honeydew             | F0FFF0    | 0xF0FFF0    |                 |
// Azure                | F0FFFF    | 0xF0FFFF    |                 |
// SandyBrown           | F4A460    | 0xF4A460    |                 |
// Wheat                | F5DEB3    | 0xF5DEB3    |                 |
// Beige                | F5F5DC    | 0xF5F5DC    |                 |
// MintCream            | F5FFFA    | 0xF5FFFA    |                 |
// WhiteSmoke           | F5F5F5    | 0xF5F5F5    |                 |
// Salmon               | FA8072    | 0xFA8072    |                 |
// GhostWhite           | F8F8FF    | 0xF8F8FF    |                 |
// AntiqueWhite         | FAEBD7    | 0xFAEBD7    |                 |
// Linen                | FAF0E6    | 0xFAF0E6    |                 |
// LightGoldenrodYellow | FAFAD2    | 0xFAFAD2    |                 |
// OldLace              | FDF5E6    | 0xFDF5E6    |                 |
// Red                  | FF0000    | 0xFF0000    | (255, 0, 0)     | Yes
// Fuchsia              | FF00FF    | 0xFF00FF    |                 |
// Magenta / Fuschia    | FF00FF    | 0xFF00FF    | (255, 0, 255)   | Yes
// DeepPink             | FF1493    | 0xFF1493    |                 |
// Tomato               | FF6347    | 0xFF6347    |                 |
// OrangeRed            | FF4500    | 0xFF4500    |                 |
// HotPink              | FF69B4    | 0xFF69B4    |                 |
// Coral                | FF7F50    | 0xFF7F50    |                 |
// DarkOrange           | FF8C00    | 0xFF8C00    |                 |
// LightSalmon          | FFA07A    | 0xFFA07A    |                 |
// LightPink            | FFB6C1    | 0xFFB6C1    |                 |
// Orange               | FFA500    | 0xFFA500    |                 |
// Pink                 | FFC0CB    | 0xFFC0CB    |                 |
// Gold                 | FFD700    | 0xFFD700    |                 |
// PeachPuff            | FFDAB9    | 0xFFDAB9    |                 |
// NavajoWhite          | FFDEAD    | 0xFFDEAD    |                 |
// Moccasin             | FFE4B5    | 0xFFE4B5    |                 |
// Bisque               | FFE4C4    | 0xFFE4C4    |                 |
// MistyRose            | FFE4E1    | 0xFFE4E1    |                 |
// BlanchedAlmond       | FFEBCD    | 0xFFEBCD    |                 |
// PapayaWhip           | FFEFD5    | 0xFFEFD5    |                 |
// LavenderBlush        | FFF0F5    | 0xFFF0F5    |                 |
// Seashell             | FFF5EE    | 0xFFF5EE    |                 |
// Cornsilk             | FFF8DC    | 0xFFF8DC    |                 |
// LemonChiffon         | FFFACD    | 0xFFFACD    |                 |
// FloralWhite          | FFFAF0    | 0xFFFAF0    |                 |
// Snow                 | FFFAFA    | 0xFFFAFA    |                 |
// Yellow               | FFFF00    | 0xFFFF00    | (255, 255, 0)   | Yes
// Ivory                | FFFFF0    | 0xFFFFF0    |                 |
// LightYellow          | FFFFE0    | 0xFFFFE0    |                 |
// White                | FFFFFF    | 0xFFFFFF    | (255, 255, 255) | Yes

#endif // LEDEFFECTS_H

// =========================================================
// END OF PROGRAM
// =========================================================
