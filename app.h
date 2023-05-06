#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
// #include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// ============================================================================
//  DEBUG PRINT MACROS
//
#define DEBUG 1

#ifdef DEBUG
  #define Dprintln(message) {Serial.println(message);}
  #define Dprint(message) {Serial.print(message);}
#else
  #define Dprintln {}
  #define Dprint {}
#endif

#define ON HIGH
#define OFF LOW

// ============================================================================
// TIMERs
//
#define T1SEC 1000
uint32_t timer_1sec = 0; // 1 sec periodic task timer
uint32_t upTime = 0;

#define T1MIN 60000
uint32_t timer_1min = 0; // 1 min periodic task timer

#define OTATIME 20
uint32_t timer_ota = 0;

// LED ONE-SHOT TIMER
#define BLINK 10
uint32_t ledTimer = 0;  // oneshot timer for blinking LED

// ============================================================================
//  Function Prototype
//
void mqttCallback(char* topic, byte* payload, unsigned int length);
// void PublishMessage(String topic, String message, boolean retained = false);
void PublishMessage(const char *topic, char *message, boolean retained = false);
