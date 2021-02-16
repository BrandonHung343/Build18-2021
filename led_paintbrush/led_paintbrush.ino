#include <Adafruit_DotStar.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>
#define NUMPX 144
#define MAXBRIGHT 255
#define CLK 27
#define DAT 26

Adafruit_DotStar strip(NUMPX, CLK, DAT, DOTSTAR_BRG);
int set_brightness_percentage(float percent){
  return int(percent * MAXBRIGHT);
}

void setup() {
  // put your setup code here, to run once:
  strip.begin();
  strip.show();
  strip.setBrightness(set_brightness_percentage(0.1));
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 0; i < NUMPX; i++)
  {
    strip.setPixelColor(i, 255, 255, 255);
    delay(10);
    strip.clear();
  }
}
