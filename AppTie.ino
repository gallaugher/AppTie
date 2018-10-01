/*
Not the prettiest code, but allowed me to power a Flora + Bluetooth LE
module to color pick & send animation signals to animate a necktie.
I was trying to do something fun for Parents Day at Boston College.
For video showing the product in action, see:
https://twitter.com/gallaugher/status/1045750298495594497

I used the same wiring scheme for the sewable neopixels that you'll
find in Adafruit Sparkle Skirt:
https://learn.adafruit.com/sparkle-skirt-playground?view=all

And I wired the Flora Bluetooth LE module to the Flora with this
wiring scheme:
https://learn.adafruit.com/adafruit-flora-bluefruit-le/overview
*/
/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"
#include <Adafruit_NeoPixel.h>
#define NEOPIXEL_PIN 8
#define NEOPIXEL_TIE_PIN 6
#define NUMBER_OF_PIXELS 6 // I have six pixels on my tie
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMBER_OF_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel tieStrip = Adafruit_NeoPixel(NUMBER_OF_PIXELS, NEOPIXEL_TIE_PIN, NEO_GRB + NEO_KHZ800);

uint8_t RED_COLOR = 0;
uint8_t GREEN_COLOR = 255;
uint8_t BLUE_COLOR = 0;

// These are for Larson scanner
int pos = 0, dir = 1; // position, direction for 'eye' of scanner

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0 // 1 when testing
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
#define BLUEFRUIT_HWSERIAL_NAME           Serial1
Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/

int led = 7; // I added this

void setup(void)
{
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);  
  // and initialize strip from working Neopixel sketch
  strip.begin();
  strip.setBrightness(50);
  strip.show(); // Initialize all pixels to 'off'
  
  tieStrip.begin();
  tieStrip.setBrightness(50);
  tieStrip.setPixelColor(0, tieStrip.Color(0, 0, 0));
  tieStrip.show(); // Initialize all pixels to 'off'

  // I commented this out so it works without requiring serial monitor
//  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  /* Got a packet! */
  // printHex(packetbuffer, len);

  // Color
  if (packetbuffer[1] == 'C') {
    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];

    RED_COLOR = red;
    GREEN_COLOR = green;
    BLUE_COLOR = blue;

    // my code from blinky
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(300);               // wait for a second
      digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
      delay(300);               // wait for a second
    
    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);

        // added from adafruit forums
    uint16_t pixelNumber = 0;
    Serial.print ("RGB: ");
    Serial.println(red);
    Serial.println(green);
    Serial.println(blue);

    strip.setPixelColor(pixelNumber, strip.Color(red, green, blue));
    strip.show();
    tieStrip.setPixelColor(pixelNumber, tieStrip.Color(red, green, blue));
    tieStrip.show();

      for(int i=0;i<NUMBER_OF_PIXELS;i++){
          // tieStrip.Color takes RGB values, from 0,0,0 up to 255,255,255
          tieStrip.setPixelColor(i, tieStrip.Color(red, green, blue)); // Moderately bright green color.
          tieStrip.show(); // This sends the updated pixel color to the hardware.
          delay(300); // Delay for a period of time (in milliseconds).
      }
  }

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);

    // do colorwipe
    if (buttnum == 1) {
      Serial.print("Button is 1");
      theaterChase(strip.Color(RED_COLOR, GREEN_COLOR, BLUE_COLOR), 50);
    }
    if (buttnum == 2) {
      rainbow(10);
    }

    if (buttnum == 3) {
      rainbowCycle(10);
    }

    if (buttnum == 4) {
      larsenScanner(20);
    }
    
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<tieStrip.numPixels(); i++) {
    tieStrip.setPixelColor(i, c);
    tieStrip.show();
    delay(wait);
  }
  for(int i=0;i<NUMBER_OF_PIXELS;i++){
      tieStrip.setPixelColor(i, tieStrip.Color(0, 0, 0)); // Moderately bright green color.
      tieStrip.show(); // This sends the updated pixel color to the hardware.
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<tieStrip.numPixels(); i++) {
      tieStrip.setPixelColor(i, Wheel((i+j) & 255));
    }
    tieStrip.show();
    delay(wait);
  }
  for(int i=0;i<NUMBER_OF_PIXELS;i++){
      tieStrip.setPixelColor(i, tieStrip.Color(0, 0, 0)); // Moderately bright green color.
      tieStrip.show(); // This sends the updated pixel color to the hardware.
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< tieStrip.numPixels(); i++) {
      tieStrip.setPixelColor(i, Wheel(((i * 256 / tieStrip.numPixels()) + j) & 255));
    }
    tieStrip.show();
    delay(wait);
  }
  for(int i=0;i<NUMBER_OF_PIXELS;i++){
      tieStrip.setPixelColor(i, tieStrip.Color(0, 0, 0)); // Moderately bright green color.
      tieStrip.show(); // This sends the updated pixel color to the hardware.
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < tieStrip.numPixels(); i=i+3) {
        tieStrip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      tieStrip.show();

      delay(wait);

      for (uint16_t i=0; i < tieStrip.numPixels(); i=i+3) {
        tieStrip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
  for(int i=0;i<NUMBER_OF_PIXELS;i++){
      tieStrip.setPixelColor(i, tieStrip.Color(0, 0, 0)); // Moderately bright green color.
      tieStrip.show(); // This sends the updated pixel color to the hardware.
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < tieStrip.numPixels(); i=i+3) {
        tieStrip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      tieStrip.show();

      delay(wait);

      for (uint16_t i=0; i < tieStrip.numPixels(); i=i+3) {
        tieStrip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
  for(int i=0;i<NUMBER_OF_PIXELS;i++){
      tieStrip.setPixelColor(i, tieStrip.Color(0, 0, 0)); // Moderately bright green color.
      tieStrip.show(); // This sends the updated pixel color to the hardware.
  }
}

void larsenScanner(int iterations) {
  for (int i = 0; i < iterations; i++) {
    int j;
      // Draw 5 pixels centered on pos.  setPixelColor() will clip any
      // pixels off the ends of the strip, we don't need to watch for that.
      tieStrip.setPixelColor(pos - 2, 0x100000); // Dark red
      tieStrip.setPixelColor(pos - 1, 0x800000); // Medium red
      tieStrip.setPixelColor(pos    , 0xFF3000); // Center pixel is brightest
      tieStrip.setPixelColor(pos + 1, 0x800000); // Medium red
      tieStrip.setPixelColor(pos + 2, 0x100000); // Dark red
     
      tieStrip.show();
      delay(100);
     
      // Rather than being sneaky and erasing just the tail pixel,
      // it's easier to erase it all and draw a new one next time.
      for(j=-2; j<= 2; j++) tieStrip.setPixelColor(pos+j, 0);
     
      // Bounce off ends of strip
      pos += dir;
      if(pos < 0) {
        pos = 1;
        dir = -dir;
      } else if(pos >= tieStrip.numPixels()) {
        pos = tieStrip.numPixels() - 2;
        dir = -dir;
      }
    }
   
     for(int i=0;i<NUMBER_OF_PIXELS;i++){
      tieStrip.setPixelColor(i, tieStrip.Color(0, 0, 0)); // Moderately bright green color.
      tieStrip.show(); // This sends the updated pixel color to the hardware.
    }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return tieStrip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return tieStrip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return tieStrip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
