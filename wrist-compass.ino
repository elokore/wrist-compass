#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "compass.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define COMPASS_RADIUS 31
#define COMPASS_CHAR_PADDING 4
#define CHAR_WIDTH 5
#define CHAR_HEIGHT 7
#define RAD_TO_DEG 180 / M_PI

#define CIRCLE_CENTER_X SCREEN_WIDTH / 2
#define CIRCLE_CENTER_Y SCREEN_HEIGHT / 2

#define CIRCLE_TOP_Y CIRCLE_CENTER_Y - COMPASS_RADIUS
#define CIRCLE_BOTTOM_Y CIRCLE_CENTER_Y + COMPASS_RADIUS

#define CIRCLE_LEFT_X CIRCLE_CENTER_X - COMPASS_RADIUS
#define CIRCLE_RIGHT_X CIRCLE_CENTER_X + COMPASS_RADIUS
#define CALIBRATE_BUTTON 6

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool isCalibrating = false;
bool debounce = false;

void setup() {
  //delay(1000);
  //Serial.println("Starting setup");

  if (!init_compass()) {
    while(1);
  }

  pinMode(CALIBRATE_BUTTON, INPUT_PULLUP);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    //Serial.println(F("SSD1306 allocation failed"));
    while(1); // Don't proceed, loop forever
  }

  display.clearDisplay();
}

/*
  Draws the compass visual, a circle with each of the cardinal directions
*/
void drawCompass(double angleRad) {
  display.drawCircle(CIRCLE_CENTER_X, CIRCLE_CENTER_Y, COMPASS_RADIUS, 1);
  display.drawChar(CIRCLE_CENTER_X - (CHAR_WIDTH / 2), CIRCLE_TOP_Y + COMPASS_CHAR_PADDING, 'N', 1, 0, 1);
  display.drawChar(CIRCLE_RIGHT_X - CHAR_WIDTH - COMPASS_CHAR_PADDING, CIRCLE_CENTER_Y - (CHAR_HEIGHT / 2), 'E', 1, 0, 1);
  display.drawChar(CIRCLE_CENTER_X - (CHAR_WIDTH / 2), CIRCLE_BOTTOM_Y - CHAR_HEIGHT - COMPASS_CHAR_PADDING, 'S', 1, 0, 1);
  display.drawChar(CIRCLE_LEFT_X + COMPASS_CHAR_PADDING, CIRCLE_CENTER_Y - (CHAR_HEIGHT / 2), 'W', 1, 0, 1);

  uint8_t lineStartX = CIRCLE_CENTER_X;
  uint8_t lineStartY = CIRCLE_CENTER_Y;

  int lineX = (int)(lineStartX + cos(angleRad) * COMPASS_RADIUS * 0.75);
  int lineY = (int)(lineStartY + sin(angleRad) * COMPASS_RADIUS * 0.75);
  display.drawLine(lineStartX, lineStartY, lineX, lineY, 1);
}

/*
  Draws a small circle on the screen. When the device is level the circle will be positioned at
  the center of the compass. When the compass is tilted the circle will move in the direction of tilt.
  This is used to show the user how level the device is, the more level it is the more accurate the reading will be
*/
void drawLevelIndicator(float pitch, float roll) {
  int16_t rollSign = roll / abs(roll);
  int16_t offsetY = pitch * RAD_TO_DEG * 0.5;
  int16_t offsetX = (abs(roll) - M_PI) * RAD_TO_DEG * rollSign * 0.5;
  display.drawCircle(CIRCLE_CENTER_X + offsetX, CIRCLE_CENTER_Y + offsetY, 6, 1);
}

void loop() {
  float pitch = 0.0;
  float roll = 0.0;
  float heading = 0.0;

  if (digitalRead(CALIBRATE_BUTTON) == 0) {
    if (!debounce) {
      debounce = true;
      isCalibrating = !isCalibrating;

      if (isCalibrating) {
        resetCalibration();
      }
    }
  } else {
    debounce = false;
  }

  // put your main code here, to run repeatedly:
  display.clearDisplay();
  processCompassData(isCalibrating);

  if (isCalibrating) {
    display.setCursor(0, 0);
    display.setTextColor(1);
    display.setTextSize(1);
    display.println("Rotate device slowly in all directions to calibrate");
  } else {
    getPitchAndRoll(&pitch, &roll);

    // Hide display when the user has dropped their arm
    bool shouldHideDisplay = ((roll * RAD_TO_DEG) < 110 && (roll * RAD_TO_DEG) > 0) && (pitch * RAD_TO_DEG) > -30 && (pitch * RAD_TO_DEG) < 10;

    if (!shouldHideDisplay) {
      heading = getCompassHeading() - M_PI_2;
      drawLevelIndicator(pitch, roll);
      drawCompass(heading);
    }
  }

  display.display();
}
