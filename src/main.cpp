// +-------------------------------------------------------------
//
// Equipment:
// Arduino Mega, MPU-6050, OLED SSD1306, WS2812B Addressable LEDS x3,
// 4-relay module, 2 linear actuators, RT-11(JCHT35K9) remote/handset,
// and 650W 12V Power supply
//
// File: main.cpp
//
// Description:
//
// Creates effects on addressable LEDs, displays info on OLED Screen
// and provides commands to standup desk
//
// History:     13-Oct-2023     Scarecrow1965   Created
//
// +-------------------------------------------------------------

// ================
// installed ibraries
// ================
#include <Arduino.h>
#include <Wire.h>

// for OLED screen
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <U8g2lib.h>

// for MPU-6050
#include <Adafruit_MPU6050.h>

// for file management
#include <SD.h>
// #include <SPI.h>
// library used to sort files when listing directory
#include <vector>
#include <algorithm>

// for WiFi and WebServer
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>

// additional libraries required
#include "seeOLED.h"
#include "standupTableMovement.h"
#include "LEDeffects.h"

// ================
// global variables
// ================

// #define LED_BUILTIN 2  // pin for onboard LED or use LED_BUILTIN as the default location
bool bLED = LOW;

// definitions of assets and parameters for ESP32
#define LED_PIN0 13 // Test Bench = option #1: GPIO25(pin 8), option #2:GPIO13(pin 3) for the ESP32
#define LED_PIN1 12 // Standup Desk = option #1: GPIO26(pin 7), option #2:GPIO12(pin 4) for the ESP32
#define LED_PIN2 14 // Storage Bench = option #1: GPIO27(pin 6), option #2:GPIO14(pin 5) for the ESP32

// Define the number of LEDs in each strip
#define NUM_LED_PIN0 278 // Test Bench
#define NUM_LED_PIN1 222 // Standup Desk
#define NUM_LED_PIN2 242 // Storage Bench

// #define BRIGHTNESS 32    // 0 to 255 of the LEDs brightness
int BRIGHTNESS = 32;     // 0 to 255 of the LEDs brightness
#define LED_TYPE WS2812B // type works for the WS2815 which I am using
#define COLOR_ORDER RGB

// Define the LED arrays for each strip
CRGB led_Strip0[NUM_LED_PIN0]; // Test Bench
CRGB led_Strip1[NUM_LED_PIN1]; // Standup Desk
CRGB led_Strip2[NUM_LED_PIN2]; // Storage Bench

// ================
// adding the SD card reader for the ESP32
#define SCK 18  // GPIO 18 = VSPI_CLK = Pin 22 on ESP32 DEVKIT V1
#define MISO 19 // GPIO 19 = VSPI_MISO = pin 21 on ESP32 DEVKIT V1
#define MOSI 23 // GPIO 23 = VSPI_MOSI = pin 16 on ESP32 DEVKIT V1
#define CS 5    // GPIO 5 = VSPI_CS = pin 23 on ESP32 DEVKIT V1

File file;
const char *file0 = "/";
// SPIClass spi = SPIClass(VSPI);

// ================
// Webserver information
const char *ssid = "Sc4r3Cr0w245";
const char *password = "R0b0tech_1";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
// JSONVar readings;

// Timer variables
unsigned long lastTime = 0;
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc = 0;
unsigned long gyroDelay = 10;
unsigned long temperatureDelay = 1000;
unsigned long accelerometerDelay = 200;

// ================
// This will enable for the OLED screen to display information
// definition of OLED display SSD1306 for ESP32
#define OLED_CLOCK 22 // SCA pin on Display = pin 17 (I2C_SCL) on ESP32 DEVKIT V1 = GPIO 22
#define OLED_DATA 21  // SDL pin on display = pin 20 (I2C_SDA) on ESP32 DEVKIT V1 = GPIO 21
// U8G2 SSD1306 Driver here to run OLED Screen
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, OLED_CLOCK, OLED_DATA, U8X8_PIN_NONE); // This works but according to the function, it shouldn't
float fps = 0;                                                                           // to start recording the fps of the display

// ADAFRUIT SSD1306 Driver here to run animation
#define SCREEN_I2C_ADDR 0x3C // or 0x3C // also should be the same address as the MPU6050 which causes problems
// #define SCREEN_I2C_ADDR 0x68 // alternate addr for the OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RST_PIN -1  // Reset pin (-1 if not available)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST_PIN);

// ================
// MPU-6050 variables
bool dataChanged = false;
bool ledEffectActive = false;
bool gyroOrAccelChanged = false;

// =============================
// Setup for the millis function
// =============================
// Removing delay command and using millis variables for the various functions
// timing variables
unsigned long oledPreviousMillis = 0;
unsigned long ledPreviousMillis = 0;
unsigned long tablePreviousMillis = 0;
unsigned long currentMillis = 0;
// delay equivalence
const unsigned int timePeriod005 = 5;    // delay equal to 5 ms
const unsigned int timePeriod010 = 10;   // Delay equal to 10 ms
const unsigned int TimePeriod050 = 500;   // Delay equal to 1/2 second
const unsigned int timePeriod1 = 1000;   // Delay equal to 1 sec pause
const unsigned int timePeriod2 = 2000;   // Delay equal to 2 second pause
const unsigned int timePeriod10 = 10000; // Delay equal to 10 seconds pause
const unsigned int timePeriod15 = 15000; // Delay equal to 15 seconds duration
const unsigned int timePeriod30 = 30000; // Delay equal to 30 second duration
// =============================

// ================
// WiFi and SD functions
// ================

// Initialize WiFi
void initWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println("");
    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("");
    Serial.println(WiFi.localIP());
    // go to https://192.168.50.143/ to see the web page
    // current ESP32 MAC Address:  B0:A7:32:29:53:0C
}; // initializing the WiFi

void checkWiFi()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi is not connected. Attempting to reconnect...");
        WiFi.begin(ssid, password);

        unsigned long startTime = millis();
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(500);
            Serial.print(".");
            if (millis() - startTime > 5000)
            {
                Serial.println("Failed to reconnect to WiFi.");
                return;
            }
        }

        Serial.println("Reconnected to WiFi.");
    }

    IPAddress ip = WiFi.localIP();
    if (ip.toString() == "0.0.0.0")
    {
        Serial.println("ESP32 doesn't have an IP address.");
    }
    else
    {
        Serial.println("ESP32 IP address: " + ip.toString()); // used for testing purposes only
    }

    // Print WiFi signal strength
    long rssi = WiFi.RSSI();
    Serial.println("WiFi signal strength: " + String(rssi) + " dBm");
}; // checking the WiFi connection

// including functions for SD card reader
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
// ================

// ================
// I2C Scanner - used for testing purposes only
void I2C_Scanner(void)
{
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }

    if (nDevices == 0)
    {
        Serial.println("No I2C devices found\n");
    }
    else
    {
        Serial.println("done\n");
    }
}
// ================

// ===================================================
// ONE TIME (SETUP) MANDATORY FUNCTION - DO NOT REMOVE
// ===================================================
void setup()
{
    delay(1000);  // power-up safety delay

    esp_reset_reason_t reason = esp_reset_reason(); // used for testing purposes only and to debug
    Serial.printf("Reset Reason: %d\n", reason);    // used for testing purposes only and to debug

    Wire.begin(); // start the wire library communications
    Wire.setClock(400000); // Set I2C clock frequency to 400kHz

    pinMode(LED_BUILTIN, OUTPUT); // this is the LED on the ESP32 board as output
    pinMode(LED_PIN0, OUTPUT);    // This is the Test Bench LEDs as output
    pinMode(LED_PIN1, OUTPUT);    // This is the Stand Up Desk LEDs as output
    pinMode(LED_PIN2, OUTPUT);    // This is the Storage bench LEDS as output

    // Set up the interrupt pin, and configure the interrupt to be triggered on a rising edge
    pinMode(INTERRUPT_PIN, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), isr, RISING);

    Serial.begin(115200);
    // Serial.println("Starting setup"); // used for testing purposes only

    // ================
    // Set up the data pin and LED arrays
    FastLED.addLeds<LED_TYPE, LED_PIN0, COLOR_ORDER>(led_Strip0, NUM_LED_PIN0).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<LED_TYPE, LED_PIN1, COLOR_ORDER>(led_Strip1, NUM_LED_PIN1).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<LED_TYPE, LED_PIN2, COLOR_ORDER>(led_Strip2, NUM_LED_PIN2).setCorrection(TypicalLEDStrip);
    // Setup LEDs brightness and ensure that they start not lit (off)
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.clear();
    // to test the LEDs
    Serial.println("Starting LED Strip test");
    ledStripTest(led_Strip0, NUM_LED_PIN0); // this works
    ledStripTest(led_Strip1, NUM_LED_PIN1); // this works
    ledStripTest(led_Strip2, NUM_LED_PIN2); // this works

    FastLED.clear();
    // ================

    // ================
    // used for testing purposes only
    // testing the balance of the table with LED display
    // Serial.println("Testing the level LED Strip");
    // for (int i = 0; i < 2; i++)
    // {
    //     testLEDsLevel(); // used for testing purposes only
    //     delay(500);        // used for testing purposes only
    // }
    // Serial.println("Testing Level LEDs"); // used for testing purposes only
    // for (int i = 0; i < 3; i++)
    // {
    //     setLEDsLevel(); // used for testing purposes only
    //     delay(500);        // used for testing purposes only
    // }
    // Serial.println("Testing LEDs Tilted Left"); // used for testing purposes only
    // for (int i = 0; i < 3; i++)
    // {
    //     setLEDsTiltedLeft(1); // used for testing purposes only
    //     delay(500);           // used for testing purposes only
    // }
    // Serial.println("Testing LEDs More Tilted Left"); // used for testing purposes only
    // for (int i = 0; i < 3; i++)
    // {
    //     setLEDsTiltedLeft(2); // used for testing purposes only
    //     delay(500);           // used for testing purposes only
    // }
    // Serial.println("Testing LEDs Tilted Right"); // used for testing purposes only
    // for (int i = 0; i < 3; i++)
    // {
    //     setLEDsTiltedRight(1); // used for testing purposes only
    //     delay(500);    // used for testing purposes only
    // }
    // Serial.println("Testing LEDs More Tilted Right"); // used for testing purposes only
    // for (int i = 0; i < 3; i++)
    // {
    //     setLEDsTiltedRight(2); // used for testing purposes only
    //     delay(500);           // used for testing purposes only
    // }
    // Serial.println("Testing LEDs Tilted Forwards"); // used for testing purposes only
    // for (int i = 0; i < 3; i++)
    // {
    //     setLEDsTiltedForward(1); // used for testing purposes only
    //     delay(500);           // used for testing purposes only
    // }
    // Serial.println("Testing LEDs More Tilted Forwards"); // used for testing purposes only
    // for (int i = 0; i < 3; i++)
    // {
    //     setLEDsTiltedForward(2); // used for testing purposes only
    //     delay(500);           // used for testing purposes only
    // }
    // Serial.println("Testing LEDs Tilted Backwards"); // used for testing purposes only
    // for (int i = 0; i < 3; i++)
    // {
    //     setLEDsTiltedBackward(1); // used for testing purposes only
    //     delay(500);           // used for testing purposes only
    // }
    // Serial.println("Testing LEDs More Tilted Backwards"); // used for testing purposes only
    // for (int i = 0; i < 3; i++)
    // {
    //     setLEDsTiltedBackward(2); // used for testing purposes only
    //     delay(500);           // used for testing purposes only
    // }
    // FastLED.clear();
    // ================

    // ================
    // I2C Scanner - used for testing purposes only
    // I2C_Scanner(); // used for testing purposes only
    // I2C Scanner reveals the following addresses:
    // I2C device found at address 0x3C !
    // I2C device found at address 0x68 !
    // ================

    // ================
    // to enable pull-up resistors on the SDA and SCL pins within the ESP32
    // pinMode(OLED_DATA, INPUT_PULLUP); // Enable pull-up resistor on SDA pin
    // pinMode(OLED_CLOCK, INPUT_PULLUP); // Enable pull-up resistor on SCL pin
    // NOTE: the ESP32 will automatically enable the internal pull-up resistors for the SDA and SCL pins
    // to disable pull-up resistors on the SDA and SCL pins of the ESP32
    // pinMode(OLED_DATA, INPUT); // Disable pull-up resistor on SDA pin
    // pinMode(OLED_CLOCK, INPUT); // Disable pull-up resistor on SCL pin
    // NOTE: Not recommended to disable the pull-up resistors on the SDA and SCL pins of the ESP32
    // ================

    // ================
    // for SD card setup, used for testing purposes
    // if (!SD.begin(CS))
    // {
    //     Serial.println("Card Mount Failed, Card failed, or not present");
    //     return;
    // }
    //
    // // spi.begin(SCK, MISO, MOSI, CS);
    // uint8_t cardType = SD.cardType();
    //
    // if (cardType == CARD_NONE)
    // {
    //     Serial.println("No SD card attached");
    //     return;
    // }
    //
    // Serial.print("SD Card Type: ");
    // if (cardType == CARD_MMC)
    // {
    //     Serial.println("MMC");
    // }
    // else if (cardType == CARD_SD)
    // {
    //     Serial.println("SDSC");
    // }
    // else if (cardType == CARD_SDHC)
    // {
    //     Serial.println("SDHC");
    // }
    // else
    // {
    //     Serial.println("UNKNOWN");
    // }
    //
    // uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    // Serial.printf("SD Card Size: %lluMB\n", cardSize);
    // ================

    // ================
    // Commands for SD card reader
    // listDir(SD, file0, 0); // used for testing purposes only
    // ================

    // ================
    // setup for the OLED display
    u8g2.begin();
    u8g2.clear();
    u8g2.setFont(u8g2_font_profont10_tf);
    oled_LineH = u8g2.getFontAscent() - u8g2.getFontDescent();
    if (!u8g2.begin())
    {
        Serial.println(F("SSD1306 allocation failed! using the U8G2 library"));
        for (;;)
        {
            // don't proceed, loop forever
        }
    };

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_I2C_ADDR))
    {
        Serial.println(F("SSD1306 allocation failed! using the Adafruit library"));
        for (;;)
        {
            // don't proceed, loop forever
        }
    };
    display.clearDisplay();
    Serial.println("OLED Display Setup Complete");
    // ================

    // ================
    // testing OLED Graphics
    Serial.println("Starting OLED test"); // used for testing purposes only
    for (int i = 0; i < 30; i++)
    {
        // function located in seeOLED.h
        startingOLED(); // displays the arduino logo when starting up the OLED display
    }
    u8g2.clearBuffer();     // clears the screen buffer
    display.clearDisplay(); // clears the diplay buffer
    u8g2.sendBuffer();      // blanks out the screen
    display.display();      // blanks out the screen
    // ================

    // ================
    // starting the MPU-6050
    initMPU(); // function located in standupTableMovement.h
    Wire.beginTransmission(MPU6050_addr); // Start communication with MPU6050 // MPU=0x68
    Wire.write(0x6B);            // Talk to the register 6B
    Wire.write(0x00);            // Make reset - place a 0 into the 6B register
    Wire.endTransmission(true);  // end the transmission
    /*
    // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
    Wire.beginTransmission(MPU);
    Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
    Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission(true);
    // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
    Wire.beginTransmission(MPU);
    Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
    Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
    Wire.endTransmission(true);
    delay(20);
    */
    //    
    // imuOffset(); // to ensure the MPU-6050 is properly calibrated
    // this one takes time to start up
    // ================

    // ================
    // Wifi and File system setup
    // to find out what the MAC ID of the ESP32 is
    // Serial.print("ESP Board MAC Address:  ");
    // Serial.println(WiFi.macAddress());
    //
    // to start the wifi
    // initWiFi();
    // ================

    // ================
    // MAY HAVE TO RELOOK THIS ONE OVER AS I AM EXPERIENCING CONNECTION ISSUES
    // SINCE THE ESP32 IS UNDER A METAL DESK
    // Handle Web Server
    // server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
    //           { request->send(SPIFFS, "/index.html", "text/html"); });
    // server.on("/data/index.html", HTTP_GET, [](AsyncWebServerRequest *request)
    //           { request->send(SD, "/data/index.html", "text/html"); });
    //
    // server.on("/data/script.js", HTTP_GET, [](AsyncWebServerRequest *request)
    //           { request->send(SD, "/data/script.js", "application/javascript"); });
    //
    // server.on("/data/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
    //           { request->send(SD, "/data/style.css", "text/css"); });
    //
    // // server.serveStatic("/", SPIFFS, "/");
    // server.serveStatic("/", SD, "/");
    //
    // server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request)
    //           {
    // gyroX=0;
    // gyroY=0;
    // gyroZ=0;
    // request->send(200, "text/plain", "OK"); });
    //
    // server.on("/resetX", HTTP_GET, [](AsyncWebServerRequest *request)
    //           {
    // gyroX=0;
    // request->send(200, "text/plain", "OK"); });
    //
    // server.on("/resetY", HTTP_GET, [](AsyncWebServerRequest *request)
    //           {
    // gyroY=0;
    // request->send(200, "text/plain", "OK"); });
    //
    // server.on("/resetZ", HTTP_GET, [](AsyncWebServerRequest *request)
    //           {
    // gyroZ=0;
    // request->send(200, "text/plain", "OK"); });
    //
    // // Handle Web Server Events
    // events.onConnect([](AsyncEventSourceClient *client)
    //                  {
    // if(client->lastId()){
    //   Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    // }
    // // send event with message "hello!", id current millis
    // // and set reconnect delay to 1 second
    // client->send("hello!", NULL, millis(), 10000); });
    // server.addHandler(&events);
    //
    // // this part is given by GitHub Co-pilot
    // server.on("/post", HTTP_POST, [](AsyncWebServerRequest *request)
    //           {
    // int params = request->params();
    // for(int i = 0; i < params; i++)
    // {
    //   AsyncWebParameter* p = request->getParam(i);
    //   if(p->isFile())
    //   {
    //     Serial.printf("POST[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
    //   } else if(p->isPost())
    //   {
    //     Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
    //   } else
    //   {
    //     Serial.printf("GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
    //   }
    // }
    // request->send(200, "text/plain", "Data received"); });
    // // end of part given by GitHub Co-pilot
    //
    // server.begin();
    // ================

    // // Read the raw acc and gyro data from the MPU-6050 1000 times, if using MPU-6050.h library
    // for (int cal_int = 0; cal_int < 1000; cal_int++)
    // {
    //     readMPUdata();
    //     // Add the gyro x offset to the gyro_x_cal variable
    //     gyro_x_cal += gyro_x;
    //     // Add the gyro y offset to the gyro_y_cal variable
    //     gyro_y_cal += gyro_y;
    //     // Add the gyro z offset to the gyro_z_cal variable
    //     gyro_z_cal += gyro_z;
    //     // Delay 3us to have 250Hz for-loop
    //     delay(3);
    // }

    // // Divide all results by 1000 to get average offset, if using the MPU-6050.h library
    // gyro_x_cal /= 1000;
    // gyro_y_cal /= 1000;
    // gyro_z_cal /= 1000;

    // can only be used on an Arduino
    // randomSeed(analogRead(A0)); // Seed the random number generator with analog input

    // ================
    // Time for the relay activation
    Serial.println("Starting relay activation"); // used for testing purposes only
    // Set relay pins as outputs
    // const int relay1_LA1LHP_Pin PROGMEM = 27;
    // const int relay2_LA1LHM_Pin PROGMEM = 26;
    // const int relay3_LA2RHP_Pin PROGMEM = 25;
    // const int relay4_LA2RHM_Pin PROGMEM = 33;
    pinMode(relay1_LA1LHP_Pin, OUTPUT);
    pinMode(relay2_LA1LHM_Pin, OUTPUT);
    pinMode(relay3_LA2RHP_Pin, OUTPUT);
    pinMode(relay4_LA2RHM_Pin, OUTPUT);

    // Set initial relay states (HIGH to deactivate all relays)
    digitalWrite(relay1_LA1LHP_Pin, HIGH);
    digitalWrite(relay2_LA1LHM_Pin, HIGH);
    digitalWrite(relay3_LA2RHP_Pin, HIGH);
    digitalWrite(relay4_LA2RHM_Pin, HIGH);

    // printout the commands list once
    // Serial.println(F("\nSend 'U' for Up, 'D' for Down, 'S' for STOP"));
    // this command is included in the tableMovement function in standupTableMovement.h

    // Initialize pins for RT-11 remote wires
    // pinMode(memoryButtonPin, INPUT_PULLUP);
    // pinMode(dataDisplayPin, INPUT_PULLUP);
    // pinMode(groundPin, INPUT_PULLUP);
    // pinMode(txPin, INPUT_PULLUP);
    // pinMode(vccPin, INPUT_PULLUP);
    // pinMode(memoryRecallPin, INPUT_PULLUP);
    // pinMode(upPin, INPUT_PULLUP);
    // pinMode(downPin, INPUT_PULLUP);

    Serial.println("Setup complete"); // used for testing purposes only
};

// ==================================
// LOOP (REPETITIVE) MANDATORY FUNCTION - DO NOT REMOVE
// ==================================
void loop()
{
    Serial.println("Starting loop"); // used for testing purposes only
    FastLED.clear(); // clear whatever is on the LED strips whenever the loop starts and/or is restarted
    
    // ================
    // this is to start keeping data for the MPU-6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    // used for calculating the gyro readings and for website display
    // Subtract bias
    // a.acceleration.x -= accelBias[0];
    // a.acceleration.y -= accelBias[1];
    // a.acceleration.z -= accelBias[2];
    // ================

    // ================
    // checkWiFi(); // check the WiFi connection and reconnect if necessary
    // ================
    //
    // ================
    // to send the information to the website
    // this is to display the data from the MPU-6050 on the website
    // if ((millis() - lastTime) > gyroDelay)
    // {
    //     // Send Events to the Web Server with the Sensor Readings
    //     events.send(getGyroReadings().c_str(), "gyro_readings", millis());
    //     // String gyroData = getGyroReadings(); // used for testing purposes only
    //     // Serial.println("Gyro data sent to " + WiFi.localIP().toString() + ": " + gyroData); // used for testing purposes only
    //     lastTime = millis();
    // }
    // if ((millis() - lastTimeAcc) > accelerometerDelay)
    // {
    //     // Send Events to the Web Server with the Sensor Readings
    //     events.send(getAccReadings().c_str(), "accelerometer_readings", millis());
    //     // String accData = getAccReadings(); // used for testing purposes only
    //     // Serial.println("Accelerometer data sent to " + WiFi.localIP().toString() + ": " + accData); // used for testing purposes only
    //     lastTimeAcc = millis();
    // }
    // if ((millis() - lastTimeTemperature) > temperatureDelay)
    // {
    //     // Send Events to the Web Server with the Sensor Readings
    //     events.send(getTemperature().c_str(), "temperature_reading", millis());
    //     // String tempData = getTemperature(); // used for testing purposes only
    //     // Serial.println("Temperature data sent to " + WiFi.localIP().toString() + ": " + tempData); // used for testing purposes only
    //     lastTimeTemperature = millis();
    // }
    // ================

    // =============================
    // Setup for the millis function
    // =============================
    // Removing delay command and using millis variables for the various functions
    // timing variables
    // unsigned long oledPreviousMillis = 0;
    // unsigned long ledPreviousMillis = 0;
    // unsigned long tablePreviousMillis = 0;
    // unsigned long currentMillis = 0;
    // delay equivalence
    // const unsigned int timePeriod005 = 5;    // delay equal to 5 ms
    // const unsigned int timePeriod010 = 10;   // Delay equal to 10 ms
    // const unsigned int TimePeriod050 = 500;  // Delay equal to 1/2 second
    // const unsigned int timePeriod1 = 1000;   // Delay equal to 1 sec pause
    // const unsigned int timePeriod2 = 2000;   // Delay equal to 2 second pause
    // const unsigned int timePeriod10 = 10000; // Delay equal to 10 seconds pause
    // const unsigned int timePeriod15 = 15000; // Delay equal to 15 seconds duration
    // const unsigned int timePeriod30 = 30000; // Delay equal to 30 second duration
    // =============================

    // variable for the LEDs animation
    startIndex = startIndex + 1; /* motion speed */

    // setting up for timings
    currentMillis = millis();
    // Serial.print("Starting millis= ");
    // Serial.println(currentMillis); // used for testing purposes only : print curent time
    // Serial.print("\n");

    // start timings for FPS
    double dStart = millis() / 1000.0; // record the start time
    // NOTE: there must be an action between variables dStart and dEnd to report an actual fps

    // ================
    // // version 1
    // // Execute table movement function
    // if (currentMillis - tablePreviousMillis >= timePeriod005)
    // {
    //     Serial.println("Starting table movement"); // used for testing purposes only
    //     tableMovement();
    //     tablePreviousMillis = currentMillis;
    // }
    // 
    // // Check if table movement is in progress, if not, execute LED effect
    // if (!dataChanged && currentMillis - ledPreviousMillis >= TimePeriod050)
    // {
    //     Serial.println("Starting random LED Effects"); // used for testing purposes only
    //     randomizedLEDEffect();
    //     ledPreviousMillis = currentMillis;
    // }
    // 
    // // Execute OLED update function at fixed interval
    // if (currentMillis - oledPreviousMillis >= timePeriod010)
    // {
    //     Serial.println("display gyro information on OLED"); // used for tssting purposes only
    //     displayOLED(fps);
    //     oledPreviousMillis = currentMillis;
    // }
    // // version 2
    // if (!dataChanged)
    // {
    //     // If table is not moving, execute LED effect and update OLED
    //     randomizedLEDEffect();
    //     // displayOLED(fps);
    // }
    // else
    // {
    //     // If table is moving, execute table movement and update LEDs accordingly
    //     tableMovement();
    //     // You can add code here to update LEDs based on table position
    // }
    // ================
    // end of version 1 and 2
        // TEST TO SEE IF OLED WILL DISPLAY GYRO INFORMATION
    // for (int t = 0; t > 100; t++)
    // {
    //     displayAllGyro2(); // displays Gyro and FPS information on the OLED without fps
    // }
    // for (int t = 0; t > 100; t++)
    // {
    //     displayAllGyro(fps); // displays Gyro and FPS information on the OLED
    // }
    // displayFPS(fps); // displays only FPS before other animation starts
    // ================
    // THIS WORKS
    checkMPUData(dataChanged);
    displayAllGyro(fps);

    // randomizedLEDEffect();

    tableCommands();
    // ================

    double dEnd = millis() / 1000.0; // record the completion time
    fps = FramesPerSecond(dEnd - dStart);
    // ================

    // INITIAL OLED and LED ON ESP32 TEST
    // this tests the display of the OLED screen in case of any issues, to ensure that it is working, and will work if the SD card does not
    // used for testing purposes only
    // for (;;)
    // {
    //     bLED = !bLED; // toggle LED State
    //     digitalWrite(LED_BUILTIN, bLED);
    // 
    //     double dStart = millis() / 1000.0; // record the start time
    //     DrawLinesAndGraphicsFrame(fps);
    //     double dEnd = millis() / 1000.0; // record the completion time
    //     fps = FramesPerSecond(dEnd - dStart);
    //     // DrawLinesAndGraphicsFrame(0);
    // }


    // Serial.println("Finished displaying gyro information on OLED"); // used for testing purposes only
    // Serial.println("Displaying another OLED screen");
    // DrawLinesAndGraphicsFrame(fps); // displays the lines and graphics on the OLED // used for testing purposes only

    Serial.println("Restarting Loop");
};

// =========================================================
// END OF PROGRAM
// =========================================================
