#include <string.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include "MSHUtils.h"

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"


struct colour {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};
typedef struct colour Colour;

struct colourPoint {
  uint8_t i;
  Colour colour;
};
typedef struct colourPoint ColourPoint;

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

#define LED 13
#define PIN 11
Adafruit_NeoPixel strip = Adafruit_NeoPixel(92, PIN, NEO_GRB + NEO_KHZ800);

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void setup() {
  delay(3000);
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  // End of trinket special code

  Serial.begin(115200);
  
  Serial.println(F("MSH Neopixels Yay"));
  Serial.println(F("-----------------------------------------"));
  
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  
  ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("Bluefruit setup"));
  Serial.println(F("-----------------------------------------"));

  strip.setBrightness(50);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  randomFairyInit();  

  Serial.print("Free Ram: " );
  Serial.println(freeRam());
  Serial.println(F("-----------------------------------------"));
}


Colour colour = {0, 0, 255 };
// the loop function runs over and over again forever
void loop() {  
  
  static uint8_t index = 0;
  static int8_t inc = 1;
  //Serial.print("Loop: ");
  //Serial.println(micros());

  colour = optionallyReadColour(colour);  
  strip.show();

  consideredFairy(colour);
  //chase(colour);  
  delay(10);  
  
}



#define POINTS 32
#define SPEED 16
static uint16_t points[POINTS];

static void randomFairyInit() {
  for(uint8_t i = 0; i < POINTS; i++ )
    points[i] = random(0, strip.numPixels());
}

#define FRAMES_TO_MAX 16
#define FRAMES_TO_NEXT 4

static void consideredFairy(Colour c) {
  static uint16_t sequence = 0;

  wash(c);

  if( FRAMES_TO_MAX > FRAMES_TO_NEXT*POINTS/2 )
    return;
  
  Colour dim;
  for(uint8_t i=0;i<POINTS;i++) {
    uint16_t seqi = sequence; //how long i has been alive.
    seqi = (sequence + i*FRAMES_TO_NEXT) % (FRAMES_TO_NEXT*POINTS);
    if( seqi == 0 )
      points[i] = random(0, strip.numPixels());        
    if( seqi < FRAMES_TO_MAX ) {
      dim = fade(c, map(seqi, 0, FRAMES_TO_MAX, 16, 255 ) ); 
      //strip.setPixelColor( points[i], dim.red, dim.green, dim.blue );
      mergePixelColor( points[i], dim.red, dim.green, dim.blue );
    } else if( seqi > FRAMES_TO_NEXT * POINTS - FRAMES_TO_MAX ) {
      dim = fade(c, map(seqi, FRAMES_TO_NEXT * POINTS - FRAMES_TO_MAX, FRAMES_TO_NEXT * POINTS, 255, 16 ) ); 
      //strip.setPixelColor( points[i], dim.red, dim.green, dim.blue ); 
      mergePixelColor( points[i], dim.red, dim.green, dim.blue );
    } else {
      //strip.setPixelColor( points[i], c.red, c.green, c.blue );
      mergePixelColor( points[i], c.red, c.green, c.blue );
    }    
  }

  sequence++;
  if( sequence == FRAMES_TO_NEXT*POINTS )
    sequence = 0;
}

static void mergePixelColor( uint16_t n, uint8_t r, uint8_t g, uint8_t b ) {
  uint32_t e = strip.getPixelColor(n);
  uint8_t 
      re = (uint8_t)(e >> 16),
      ge = (uint8_t)(e >>  8),
      be = (uint8_t)e;  

  r = max(r, re);
  g = max(g, ge);
  b = max(b, be);
  strip.setPixelColor(n, r, g, b );  
}

static void wash(Colour c) {
  Colour dim = fade(c, 16 );
  for(uint8_t i=0; i<strip.numPixels(); i++ )
    strip.setPixelColor( i, dim.red, dim.green, dim.blue );
}



//Note that the strip is 2xWIDTH wide, half faded in, half faded out.
#define WIDTH 16
static void chase(Colour c) {  

  static uint8_t i;
  static int8_t inc = 1;
  
  for(int j = 1; j<WIDTH; j++ ) {    
    Colour dim = fade(c, map(j, 0, WIDTH, 0, 255));
    strip.setPixelColor(Pos(i-j), dim.red, dim.green, dim.blue); // Draw new pixel    
  }
  for(int j = 0; j<WIDTH; j++ ) {
    Colour dim = fade(c, map(j, 0, WIDTH, 255, 0));
    strip.setPixelColor(Pos(i-j-WIDTH), dim.red, dim.green, dim.blue); // Draw new pixel    
  }  
  //strip.setPixelColor(Pos(i-2*WIDTH), 0); // Erase pixel a few steps back   
  
  switch( random(0, strip.numPixels() * 3) )
  {
    case 0:
      inc = 1;
      break;
    case 1:
      inc = -1;
      break;
    case 2:
      inc = 2;
      break;
    case 3:
      inc = -2;
      break;
  }
  i=Pos(i+inc);    
}

static Colour fade(Colour in, uint8_t brightness) {  
  in.red = (in.red * brightness) >> 8;
  in.green = (in.green * brightness) >> 8;
  in.blue = (in.blue * brightness) >> 8;
  return in;
}

static uint16_t Pos(uint16_t raw) {
  return (strip.numPixels() + raw) % strip.numPixels();
}

static void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    Serial.print("iter: ");
    Serial.print(j);
    Serial.println();
    for(i=0; i<strip.numPixels(); i++) {      
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
static uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

static Colour optionallyReadColour(Colour c) {
  if(ble.available()) {
  //if(true) {
    uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
    if (len == 0) return c;
    printHex(packetbuffer, len);
    // Color
    if (packetbuffer[1] == 'C') {
      uint8_t red = packetbuffer[2];
      uint8_t green = packetbuffer[3];
      uint8_t blue = packetbuffer[4];
      Colour out = {red,green,blue};
      return(out);
    } else {
      return c;     
    }
  }
  return c;  
}
