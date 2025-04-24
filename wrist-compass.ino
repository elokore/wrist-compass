#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define COMPASS_RADIUS 31
#define COMPASS_CHAR_PADDING 4
#define CHAR_WIDTH 5
#define CHAR_HEIGHT 7

#define CIRCLE_CENTER_X SCREEN_WIDTH / 2
#define CIRCLE_CENTER_Y SCREEN_HEIGHT / 2

#define CIRCLE_TOP_Y CIRCLE_CENTER_Y - COMPASS_RADIUS
#define CIRCLE_BOTTOM_Y CIRCLE_CENTER_Y + COMPASS_RADIUS

#define CIRCLE_LEFT_X CIRCLE_CENTER_X - COMPASS_RADIUS
#define CIRCLE_RIGHT_X CIRCLE_CENTER_X + COMPASS_RADIUS

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
}

void drawCompass() {
  display.drawCircle(CIRCLE_CENTER_X, CIRCLE_CENTER_Y, COMPASS_RADIUS, 1);
  display.drawChar(CIRCLE_CENTER_X - (CHAR_WIDTH / 2), CIRCLE_TOP_Y + COMPASS_CHAR_PADDING, 'N', 1, 0, 1);
  display.drawChar(CIRCLE_RIGHT_X - CHAR_WIDTH - COMPASS_CHAR_PADDING, CIRCLE_CENTER_Y - (CHAR_HEIGHT / 2), 'E', 1, 0, 1);
  display.drawChar(CIRCLE_CENTER_X - (CHAR_WIDTH / 2), CIRCLE_BOTTOM_Y - CHAR_HEIGHT - COMPASS_CHAR_PADDING, 'S', 1, 0, 1);
  display.drawChar(CIRCLE_LEFT_X + COMPASS_CHAR_PADDING, CIRCLE_CENTER_Y - (CHAR_HEIGHT / 2), 'W', 1, 0, 1);
}

void drawNeedleAtAngle(double angleRad) {
  uint8_t lineStartX = CIRCLE_CENTER_X;
  uint8_t lineStartY = CIRCLE_CENTER_Y;

  int lineX = (int)(lineStartX + cos(angleRad) * COMPASS_RADIUS * 0.75);
  int lineY = (int)(lineStartY + sin(angleRad) * COMPASS_RADIUS * 0.75);
  display.drawLine(lineStartX, lineStartY, lineX, lineY, 1);
}

double angle = 0.0;

void loop() {
  // put your main code here, to run repeatedly:
  display.clearDisplay();

  drawCompass();
  drawNeedleAtAngle(angle);
  angle += 0.2;

  display.display();
}
