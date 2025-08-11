#ifndef RADARSENSOR_H
#define RADARSENSOR_H

#include <Arduino.h>
#include <SoftwareSerial.h>

typedef struct RadarTarget {
  float distance;  // mm
  float angle;     // radians
  float speed;     // cm/s
  int16_t x;       // mm
  int16_t y;       // mm
  bool detected;
} RadarTarget;

class RadarSensor {
  public:
    RadarSensor(uint8_t rxPin, uint8_t txPin);         // Original SoftwareSerial constructor
    RadarSensor(HardwareSerial& serial);               // New HardwareSerial constructor

    void begin(unsigned long baud = 256000);
    bool update();
    RadarTarget getTarget();

  private:
    Stream* radarStream;               // Generic interface to both hardware and software serial
    SoftwareSerial* softSerial = nullptr; // Only if SoftwareSerial is used
    RadarTarget target;
    bool parseData(const uint8_t *buffer, size_t len);
};

#endif
