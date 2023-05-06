#include "app.h"
#include "env.h"
#include "json_support.h"
#include "mqtt_support.h"

// ######################################
// #####     GLOBAL VARIABLES       #####
// ######################################

// See env.h for Ethernet and MQTT connection parameters

// ######################################
// #####           SETUP            #####
// ######################################
void setup()
{
    #ifdef DEBUG
    Serial.begin(9600);
    while(!Serial) {}
    #endif

    Dprintln("Starting...");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, OFF); // Turn off LED

    Ethernet.begin(mac, ip);
    delay(1500);
    mqttClient.setServer(MQTT_SERVER, 1883);
    mqttClient.setCallback(mqttCallback);
    delay(100);
}

// ######################################
// #####            LOOP            #####
// ######################################
void loop()
{
    if(!mqttClient.connected()) mqttConnect();
    mqttClient.loop(); // allow MQTT Client to run

    

    

    


    // ----- END OF LOOP -----
    delayMicroseconds(1000);
}
