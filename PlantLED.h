/** 
 *  @file    PlantLED.h
 *  @author  peter c
 *  @date    4/14/2017  
 *  @version 0.1
 *  
 *
 *  @section DESCRIPTION
 *  Lighting control for LED strips
 *  
 *
 */
#ifndef PLANTLED_H
#define PLANTLED_H
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <pins_arduino.h>
#endif

#include <Adafruit_GFX.h>
#include <Adafruit_NeoPixel.h>


#define MIN_DUTY_CYCLE 70
#define MAX_DUTY_CYCLE 95
#define NUM_OF_SEGMENTS 10



const uint32_t RED =   0xFF0000; ;// (uint32_t) 0xFF << 16
const uint32_t GREEN = 0x00FF00;
const uint32_t BLUE = 0xFF  ;
const uint32_t WHITE = RED + GREEN + BLUE;
const uint32_t BLACK = 0;





enum  stripPattern {  MOSTLY_BLUE, MOSTLY_RED, BLEND_BLUE_RED };

class PlantLEDStrip : public Adafruit_NeoPixel
{
public:


  PlantLEDStrip(int lenngth = 60, uint8_t pin = 6, uint8_t pixel_type = NEO_GRB + NEO_KHZ800 );
  /**
   * [refresh update the LED segment]
   */
  void refresh();
  void initialize();  
  void setPattern( stripPattern aPattern );
  void fillSegment( uint32_t aColor );
  void setDutyCycle( unsigned short aDutyCycle );
  const uint32_t blue = Color(0, 0, 255);
  const uint32_t red = Color(255, 0, 0);
  void randomizeDutyCycle();
  bool isLightsOn();
/**
 * [turnOff turnoff the LED strip and don't refesh]
 */
  void turnOff();
/**
 * [turnOn enale refreshing of LED to take palce]
 */
  void turnOn();
  /**
   * [flipColors flip the red and blue colors]
   */
  void flipColors();
  unsigned short getDutyCycle();

private:
  stripPattern currentPattern;
  uint16_t stripLength = 0;
  unsigned long interval;    // milliseconds between updates
  unsigned long lastUpdate;  // last update of position
  uint32_t majorityColor, minorityColor;    // What colors are in use
  unsigned short currentDutyCycle;
  unsigned int currentPixel = 0;
  bool _isLightsOn = true;
  void doRegularPlantLighting();

};

#endif
