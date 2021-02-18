#include <SD.h>
#include <JPEGDecoder.h>

#define SDMOSI 11
#define SDCS 10
#define SDCLK 13
#define SDMISO 12
#define FNAME "test.jpg"

File myFile;

void setup() {
   if (!SD.begin(SDCS)) {
    Serial.println("SD Initializing... failed!");
    while (1) delay(0);
  }
  Serial.println("SD Initialization done");

  myFile = SD.open(FNAME, FILE_READ);
  JpegDec.decodeSdFile(myFile);

}

void loop() {
  // put your main code here, to run repeatedly:

}
