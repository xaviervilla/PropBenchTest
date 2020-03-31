#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "HX711.h"
#include <Servo.h>

#define ledPin 13

// HX711 circuit wiring for the scale
#define LOADCELL_DOUT_PIN 4
#define LOADCELL_SCK_PIN 5
#define CALCULATED_CALIBRATION 2098
HX711 scale;
float thrustReadings[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float currentReading = 0;

// Servo information
Servo esc;
#define ESC_SIGNAL 9  // pin D8
float throttle = 0;     // initialize throttle to zero
uint8_t incriments = 20; // number of incriments. Ex: 20 means 5% incriments
int throttleCount = 0;
int warningThrottle = 255;     // this is a warning burst strength (0-255)->(0%-100%)
int warningBurstTime = 750;     // duration of warning burst in ms

// Buttons
#define LEFT_PRESS 11
#define RIGHT_PRESS 12

// Screen
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define SPLASH_HEIGHT   64
#define SPLASH_WIDTH    128
static const unsigned char PROGMEM splash_bmp[] =
{
  B00000000, B00000000, B00000000, B11000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000001, B11100000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000011, B11110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000111, B11111000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000011, B11111000, B00000001, B11111111, B11100000, B00111111, B11111100, B00001111, B11111111, B00000000, B00000011, B11111111, B11000000, B00000000, B00000000, B00000000, B00000011, B11100000, B00000000, B11111111, B11100000, B00111111, B11111000, B00000111, B11111111, B00000000, B00000011, B11111111, B11000000, B00000000, B00000000, B00000000, B00000011, B11100000, B00000000, B11111111, B11100000, B00111111, B11110000, B00000111, B11111111, B00000000, B00000001, B11111111, B11000000, B00000000, B00000000, B00000000, B00000011, B11100000, B00000001, B11111111, B11100000, B00111111, B11110000, B00001111, B11111111, B00000000, B00000011, B11111111, B11000000, B00000000, B00000000, B00110001, B10000011, B11100000, B00000001, B11111111, B11100000, B00111111, B11110000, B00001111, B11111111, B00000000, B00000011, B11111111, B11000000, B00000000, B00000000, B11110011, B11100011, B11100000, B00000011, B11111111, B11100000, B00111111, B11110000, B00011111, B11111111, B10000000, B00000111, B11111111, B11000000, B00000000, B00000011, B11110111, B11111011, B11100000, B00000011, B11111111, B11110000, B01111111, B11100000, B00011111, B11111111, B10000000, B00000111, B11111111, B11000000, B00000000, B00000111, B11110111, B11111111, B11100000, B00000111, B11111111, B11110000, B01111111, B11100000, B00111111, B11111111, B10000000, B00001111, B11111111, B11100000, B00000000, B00001111, B11110011, B11111111, B11100000, B00000111, B11111111, B11110000, B01111111, B11100000, B00111111, B11111111, B10000000, B00001111, B11111111, B11100000, B00000000, B00001111, B11100001, B11111111, B11100000, B00001111, B11111111, B11110000, B01111111, B11100000, B01111111, B11111111, B10000000, B00011111, B11111111, B11100000, B00000000, B00011111, B11000000, B01111111, B11100000, B00001111, B11111111, B11110000, B01111111, B11100000, B01111111, B11111111, B10000000, B00011111, B11111111, B11100000, B00000000, B00111111, B00000000, B00011111, B11100000, B00011111, B11111111, B11110000, B11111111, B11100000, B11111111, B11111111, B10000000, B00111111, B11111111, B11100000, B00000000, B00111110, B00011111, B00001111, B11100000, B00011111, B11111111, B11110000, B11111111, B11000000, B11111111, B11111111, B11000000, B00111111, B11111111, B11100000, B00000000, B00111110, B00111111, B10001111, B11100000, B00111111, B11011111, B11110000, B11111111, B11000001, B11111111, B11111111, B11000000, B01111111, B10111111, B11100000, B00000000, B01111100, B01111111, B11000111, B11100000, B00111111, B11011111, B11111000, B11111111, B11000001, B11111110, B11111111, B11000000, B01111111, B10111111, B11100000, B00000000, B01111100, B11111111, B11100111, B11100000, B01111111, B10011111, B11111000, B11111111, B11000011, B11111110, B11111111, B11000000, B11111111, B00111111, B11110000, B00000000, B01111100, B11111111, B11100111, B11100000, B01111111, B10011111, B11111000, B11111111, B11000011, B11111100, B01111111, B11000000, B11111111, B00111111, B11110000, B00000000, B01111000, B11111111, B11100011, B11100000, B11111111, B10011111, B11111001, B11111111, B11000111, B11111100, B01111111, B11000001, B11111110, B00111111, B11110000, B00000000, B01111000, B11111111, B11100011, B11100000, B11111111, B11111111, B11111001, B11111111, B10000111, B11111111, B11111111, B11000001, B11111111, B11111111, B11110111, B11100000, B11111000, B11111111, B11110011, B11100001, B11111111, B11111111, B11111001, B11111111, B10001111, B11111111, B11111111, B11000011, B11111111, B11111111, B11110000, B00000000, B11111000, B11111111, B11100011, B11100001, B11111111, B11111111, B11111001, B11111111, B10001111, B11111111, B11111111, B11100011, B11111111, B11111111, B11110011, B11111000, B11111100, B11111111, B11100111, B11100011, B11111111, B11111111, B11111101, B11111111, B10011111, B11111111, B11111111, B11100111, B11111111, B11111111, B11110000, B00000000, B11111100, B11111111, B11100111, B11100011, B11111111, B11111111, B11111101, B11111111, B10011111, B11111111, B11111111, B11100111, B11111111, B11111111, B11110011, B11111100, B11111100, B01111111, B11000111, B11100111, B11111111, B11111111, B11111111, B11111111, B10111111, B11111111, B11111111, B11101111, B11111111, B11111111, B11111000, B00000000, B11111110, B00111111, B10001111, B11100111, B11111111, B11111111, B11111111, B11111111, B00111111, B11111111, B11111111, B11101111, B11111111, B11111111, B11111011, B11111110, B01111110, B00011111, B00001111, B11001111, B11111111, B11111111, B11111111, B11111111, B01111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111000, B00000000, B01111111, B00000000, B00011111, B11001111, B11111111, B11111111, B11111111, B11111111, B01111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111011, B11111111, B01111111, B11000000, B01111111, B11011111, B11110000, B00000111, B11111111, B11111111, B11111111, B10000000, B00111111, B11111111, B11100000, B00011111, B11111000, B00000000, B00111111, B11111111, B11111111, B10011111, B11110000, B00000111, B11111111, B11111111, B11111111, B10000000, B00111111, B11111111, B11000000, B00001111, B11111000, B00000000, B00011111, B11111111, B11111111, B00111111, B11100000, B00000111, B11111111, B11111111, B11111111, B00000000, B00111111, B11111111, B11000000, B00001111, B11111100, B00000000, B00011111, B11111111, B11111111, B00111111, B11100000, B00000111, B11111111, B11111111, B11111111, B00000000, B00111111, B11111111, B10000000, B00001111, B11111100, B00000000, B00001111, B11111111, B11111110, B01111111, B11100000, B00000111, B11111111, B11111111, B11111111, B00000000, B00111111, B11111111, B10000000, B00001111, B11111100, B00000000, B00000111, B11111111, B11111100, B11111111, B11100000, B00001111, B11111111, B11111111, B11111111, B00000000, B01111111, B11111111, B11000000, B00011111, B11111110, B00000000, B00000001, B11111111, B11110000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B01111111, B11000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00001100, B00011000, B00111110, B00000000, B01110000, B01100000, B00000000, B00000000, B00000000, B00000000, B00000000, B00011000, B00000000, B00000000, B00000000, B00000000, B00001100, B00011000, B01100111, B00000000, B01110000, B01100000, B00000000, B00000000, B00000000, B00000000, B00000000, B00011000, B00000000, B00000000, B00000000, B00000000, B00001100, B00011000, B11000011, B00000000, B01110000, B11100000, B00000000, B00000000, B00000000, B00000000, B00000000, B00011000, B00000000, B00000000, B00000000, B00000000, B00001100, B00011000, B11000000, B00000000, B01111000, B11100000, B11111000, B00111111, B00011111, B00000111, B11000001, B11111000, B00000000, B00000000, B00000000, B00000000, B00001100, B00011000, B11000000, B00000000, B01111001, B11100001, B11001100, B00111000, B00111001, B10001110, B01100011, B10111000, B00000000, B00000000, B00000000, B00000000, B00001100, B00011000, B11000000, B00000000, B01101101, B11100001, B10001100, B00110000, B00110000, B00001100, B01100011, B00011000, B00000000, B00000000, B00000000, B00000000, B00001100, B00011000, B11000000, B00000000, B01101111, B01100001, B11111100, B00110000, B00110000, B00001111, B11100011, B00011000, B00000000, B00000000, B00000000, B00111111, B00001100, B00011000, B11000000, B00000000, B01100111, B01100001, B10000000, B00110000, B00110000, B00001100, B00000011, B00011000, B01111110, B00000000, B00000000, B00000000, B00001100, B00111000, B11000011, B00000000, B01100111, B01100001, B10000000, B00110000, B00110000, B00001100, B00000011, B00011000, B00000000, B00000000, B00000000, B00000000, B00000110, B00110000, B01100111, B00000000, B01100000, B01100001, B11001100, B00110000, B00111001, B10001110, B01100011, B10111000, B00000000, B00000000, B00000000, B00000000, B00000011, B11100000, B00111110, B00000000, B01100000, B01100000, B11111000, B00110000, B00011111, B00000111, B11000001, B11111000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00001111, B10000000, B00010000, B00000000, B00000000, B00000001, B00000000, B00000010, B00001111, B00010000, B00000011, B00110000, B00000000, B00000000, B00000000, B00000000, B00000010, B00001000, B00000000, B00000000, B00000000, B00000011, B00000000, B00000010, B00001000, B00010000, B00000010, B00100000, B00000000, B00000000, B00000000, B00000000, B00000010, B00001110, B00010000, B11000011, B11001110, B00000010, B10000110, B00011110, B00001000, B00111000, B00100111, B01110000, B00000000, B00000000, B00000000, B00000000, B00000010, B00001001, B00010001, B00100010, B01010000, B00000100, B10001001, B00010010, B00000111, B00010001, B00100010, B00100000, B00000000, B00000000, B00000000, B00000000, B00000010, B00001001, B00010001, B00100010, B01001110, B00000111, B10001001, B00010010, B00000000, B10010001, B00100010, B00100000, B00000000, B00000000, B00000000, B00000000, B00000010, B00001001, B00010001, B00100010, B01000001, B00000100, B01001001, B00010010, B00000000, B10010001, B00100010, B00100000, B00000000, B00000000, B00000000, B00000000, B00000010, B00000001, B00010000, B00100011, B11001110, B00001000, B01000001, B00011110, B00001111, B00011001, B11100010, B00100000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B01000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000011, B10000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000
};

static const unsigned char PROGMEM prop1[] ={
B00000000, B00001100, B00000000, B00000000, B00001101, B11000000, B00000000, B00001101, B11100000, B00000010, B00001100, B00010000, B00000110, B00001100, B00011000, B00001100, B00001100, B00001100, B00001100, B00001100, B00000100, B00001000, B00001100, B00000100, B00011000, B00001100, B00000000, B00011000, B00011110, B00000000, B00011000, B00111111, B00000000, B00011000, B00111111, B00000000, B00011000, B00111111, B10000000, B00011001, B11111111, B11000000, B00000011, B11000100, B11110000, B00001111, B00000000, B00111100, B00001100, B00000000, B00001110, B00001000, B00000000, B00000110, B00000000, B00000000, B00011000, B00000000, B00000000, B00110000, B00000000, B00000001, B11100000, B00000000, B00000011, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000, B00000000
};
static const unsigned char PROGMEM prop2[] ={
B00000000, B00000000, B00000000, B00000000, B01111100, B00000000, B00000001, B11000000, B00000000, B00000111, B00000000, B00000000, B00001100, B00000000, B00000100, B00000000, B00000000, B00001100, B00111000, B00000000, B00111000, B00011110, B00000000, B00100010, B00000111, B10000101, B11100110, B00000001, B11111111, B11000010, B00000000, B01111111, B00000011, B00000000, B00111111, B00000010, B00000000, B00111111, B00000010, B00000000, B00111110, B00000010, B00010000, B00001100, B00000100, B00001000, B00000100, B00000100, B00001100, B00001110, B00000000, B00000110, B00001100, B00000000, B00000011, B10001110, B00000000, B00000001, B11100100, B00000000, B00000000, B01101110, B00000000, B00000000, B00000110, B00000000, B00000000, B00000000, B00000000
};

// These are the states that the arduino will be in at any given time
#define INITIAL_STATE 0
#define WARNING_STATE 1
#define TESTING_STATE 2
#define RESULTS_STATE 3
#define CALIBRATION_STATE 4
int currentState = 0;
  
void drawButtons(String leftText, String middleText, String rightText){
    if(middleText.length()==0){
      display.clearDisplay();
      display.drawFastHLine(0, 55, 128, 1);// drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
      display.drawFastVLine(64, 55, 9, 1);// drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
      // you need 6 pixels wide per character;
      display.setTextSize(1);
      display.setTextColor(1);
      int lSize = 6*leftText.length();
      int rSize = 6*rightText.length();
      display.setCursor(128/4-lSize/2, 57);
      display.print(leftText);
      display.setCursor((128/4*3-rSize/2), 57);
      display.print(rightText);
      display.display();
      delay(1);
    }
    else{
      display.clearDisplay();
      display.drawFastHLine(0, 55, 128, 1);// drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
      display.drawFastVLine(37, 55, 9, 1);// drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
      display.drawFastVLine(91, 55, 9, 1);// drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
      // you need 6 pixels wide per character;
      display.setTextSize(1);
      display.setTextColor(1);
      int lSize = 6*leftText.length()+6;
      int rSize = 6*rightText.length()-6;
      int mSize = 6*middleText.length();
      display.setCursor(128/6-lSize/2, 57);
      display.print(leftText);
      display.setCursor((128/6*5-rSize/2), 57);
      display.print(rightText);
      display.setCursor(128/6*3-mSize/2, 57);
      display.print(middleText);
      display.display();
      delay(1);
    }
}


void setup() {
  Serial.begin(9600);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c)) { // Address 0x3D for 128x64
    //Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // initialize scale
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(CALCULATED_CALIBRATION); // this number was obtained by calibrating the scale
  scale.tare(); // this zeros the scale

  // set up the ESC throttle control
  esc.attach(ESC_SIGNAL);
  esc.write(throttle);// this sets throttle to zero
  
  // initialize buttons
  pinMode(LEFT_PRESS, INPUT_PULLUP);
  pinMode(RIGHT_PRESS, INPUT_PULLUP);

  // Clear the buffer in case it contains data
  display.clearDisplay();

  // draw the splash screen
  display.drawBitmap(
    (display.width()  - SPLASH_WIDTH ) / 2,
    (display.height() - SPLASH_HEIGHT) / 2,
    splash_bmp, SPLASH_WIDTH, SPLASH_HEIGHT, 1);
  display.display();
  delay(2000);
  display.clearDisplay();
  display.display();
  delay(500);

  while(true)
  if (scale.is_ready()) {
    //Serial.println("Scale detected!");
    break;
  } else {
    //Serial.println("Scale not detected, HX711 chip needs troubleshooting.");
    delay(1000);
  }
 
}

void loop() {

  long timeStamp = millis();
  long wait = 250;
  bool myBool = true;
  
  switch(currentState)
  {
    case INITIAL_STATE: 
     //Serial.println(F("INITIAL_STATE"));
      delay(1000);
      drawButtons("Tare","","Start");
      timeStamp = millis();
      wait = 250;
      myBool = true;
      while (myBool)
      {
        if(!digitalRead(LEFT_PRESS)){
          scale.tare();
          delay(250);
        }
        else if(!digitalRead(RIGHT_PRESS)){
          // Change state to Warning State
          //Serial.println(currentState);
          currentState = 1;
          //Serial.println(currentState);
          myBool = false;
          delay(250);
        }
        if (millis()-timeStamp > wait){
          display.writeFillRect(0, 0, 128, 55, BLACK); //writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
          currentReading = scale.get_units(1);
//          Serial.println(currentReading);
          display.setTextSize(3);
          display.setTextColor(WHITE);
          display.setCursor(0, 0);
          display.print(currentReading);
          display.print(F("g"));
          display.display();
          timeStamp = millis();
        }
      }
      break;  
    case WARNING_STATE: 
     //Serial.println(F("WARNING_STATE"));
      drawButtons("Cancel","","Cancel");
      esc.write(warningThrottle);
      timeStamp = millis();
      myBool = true;
      wait = 5000;
      display.setTextSize(3);
      display.setTextColor(BLACK, WHITE);
      display.setCursor(0, 0);
      display.print(F("WARNING"));
      display.setTextSize(1);
      display.setCursor(0, 26);
      display.setTextColor(WHITE, BLACK);
      display.print(F("Begining Test, Foo!"));
      display.display();
      while (myBool){
        if(!digitalRead(LEFT_PRESS)){
          currentState = 0;
          myBool = false;
          esc.write(0);
          delay(250);
        }
        if(!digitalRead(RIGHT_PRESS)){
          currentState = 0;
          myBool = false;
          esc.write(0);
          delay(250);
        }
        if (millis()-timeStamp > warningBurstTime){
          esc.write(0);
        }
        if(millis()-timeStamp > wait){
          currentState = 2;
          myBool = false;
          esc.write(0);
        }
      }
      break;
    case TESTING_STATE:
     //Serial.println(F("TESTING_STATE"));
      drawButtons("Stop Test","","Stop Test");
      timeStamp = millis();
      wait = 500;
      myBool = true;
      throttleCount = 0;
      bool propBool = true;
      while (myBool){
        display.writeFillRect(102, 0, 24, 24, BLACK);
        if (propBool){ display.drawBitmap( 102, 0, prop1, 24, 24, 2); propBool = !propBool; }
        else{ display.drawBitmap( 102, 0, prop2, 24, 24, 2); propBool = !propBool; }
        display.display();
        if(!digitalRead(LEFT_PRESS)){
          throttle = 0;
          esc.write(throttle);
          currentState = 0;
          myBool = false;
          delay(250);
        }
        if(!digitalRead(RIGHT_PRESS)){
          throttle = 0;
          esc.write(throttle);
          currentState = 0;
          myBool = false;
          delay(250);
        }
        if (millis()-timeStamp > wait){ // if the time limit has elapsed
          currentReading = scale.get_units(5);// measure force
          thrustReadings[throttleCount] = currentReading;
          display.writeFillRect(0, 0, 128, 55, BLACK);
          display.setTextSize(2);
          display.setTextColor(WHITE);
          display.setCursor(0, 0);
          display.print(throttleCount*100/incriments);
          display.print(F("% (g)"));
          display.setTextSize(3);
          display.setTextColor(WHITE);
          display.setCursor(0, 24);
          display.print(currentReading);
          if (propBool){ display.drawBitmap( 102, 0, prop1, 24, 24, 1); propBool = !propBool; }
          else{ display.drawBitmap( 102, 0, prop2, 24, 24, 1); propBool = !propBool; }
          display.display();
          throttleCount++;
//          Serial.print(throttle);
//          Serial.print(F(" ~"));
//          Serial.println(currentReading);
          throttle += 255.0/incriments; // increase force
          // if force is > 100, set bool to false, set state to results
          // else set new timestamp
          if (throttle >= 257){
            throttle = 0;
            esc.write(throttle);
            myBool = false;
            currentState = 3;
            delay(250);
          }
          else{
            esc.write(throttle);
            timeStamp = millis();
          }
        }
      }
    //break;
    case RESULTS_STATE:
      if (currentState!= RESULTS_STATE){ break; }
      display.clearDisplay();
      //Serial.println(F("RESULTS_STATE"));
      drawButtons("<", "reset", ">");
      timeStamp = millis();
      wait = 10000;
      myBool = true;
      currentReading = thrustReadings[20];
      int count = 20;
      for (int i = 0; i < incriments+1; i++){
        Serial.print((i)*(100/incriments)); Serial.print(F("%\t")); Serial.println(thrustReadings[i]);
      }
          display.writeFillRect(0, 0, 128, 55, BLACK);
          display.setTextSize(2);
          display.setTextColor(WHITE);
          display.setCursor(0, 0);
          display.print(count*100/incriments);
          display.print(F("% (g)"));
          display.setTextSize(3);
          display.setTextColor(WHITE);
          display.setCursor(0, 24);
          display.print(currentReading);
          display.display();
      while (myBool){
        if(!digitalRead(LEFT_PRESS) && !digitalRead(RIGHT_PRESS)){ // press both buttons to start over
          for (int i = 0; i < incriments+1; i++){
            thrustReadings[i] = 0;  // reset all the readings in case the next test is cancelled or it will provide false data.
          }
          currentState = 0;
          myBool = false;
          delay(250);
        }
        else if(!digitalRead(LEFT_PRESS)){
          if (count == 0){
            count = 20;
          }
          else{
            count--;
          }
          currentReading = thrustReadings[count];
          display.writeFillRect(0, 0, 128, 55, BLACK);
          display.setTextSize(2);
          display.setTextColor(WHITE);
          display.setCursor(0, 0);
          display.print(count*100/incriments);
          display.print(F("% (g)"));
          display.setTextSize(3);
          display.setTextColor(WHITE);
          display.setCursor(0, 24);
          display.print(currentReading);
          display.display();
          delay(250);
        }
        else if(!digitalRead(RIGHT_PRESS)){
          if (count == 20){
            count = 0;
          }
          else{
            count++;
          }
          currentReading = thrustReadings[count];
          display.writeFillRect(0, 0, 128, 55, BLACK);
          display.setTextSize(2);
          display.setTextColor(WHITE);
          display.setCursor(0, 0);
          display.print(count*100/incriments);
          display.print(F("% (g)"));
          display.setTextSize(3);
          display.setTextColor(WHITE);
          display.setCursor(0, 24);
          display.print(currentReading);
          display.display();
          delay(250);
        }
      }
  }
}
