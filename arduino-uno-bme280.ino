#include "app.h"
#include "env.h"
// #include "json_support.h"
#include "mqtt_support.h"

// ######################################
// #####     GLOBAL VARIABLES       #####
// ######################################

// See env.h for Ethernet and MQTT connection parameters

Adafruit_BME280 bme;
#define SEALEVELPRESSURE_HPA (1013.25)
#define CALIBRATE (4.59)

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

  if (!bme.begin(0x76)) {
    Dprintln("BME280 not found");
    while (1);
	} 
  else {
    Dprintln("BME280 found!");
  }

    // ----- Start Periodic Timer ------
  timer_1sec = millis();
  timer_1min = millis();
}

// ######################################
// #####            LOOP            #####
// ######################################
void loop() {
  // --------------------------------------------------------------------------
  //  House keeping section
  // --------------------------------------------------------------------------
  // LED control with one-shot timer
  if(((ledTimer !=0) && (millis() - ledTimer) > BLINK)) {
    ledTimer = 0; // disarm one-shot LED timer
    digitalWrite(LED_BUILTIN, HIGH); // turn off LED
  }

  // Check for MQTT connection
  if(!mqttClient.connected()) {
    delay(1000);
    mqttConnect();
  }
  else {
    mqttClient.loop(); // allow MQTT Client to run
    loopCont(); // run functional code of the loop()
  }
} // end of loop()

// --------------------------------------------------------------------------
//  Functional code section
// --------------------------------------------------------------------------
void loopCont() {
  // ----------------------------------
  // 1 sec timer task
  if( (millis() - timer_1sec) > T1SEC) {
    timer_1sec = millis();
    // -----  Turn on LED -----
    digitalWrite(LED_BUILTIN, LOW); // turn on LED
    ledTimer = millis();  // start one-shot LED timer

    // ----- Increment uptime counter -----
    upTime += 1;
    /*
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println("*C");

    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println("hPa");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println("%");

    Serial.println();
    */
  }

  // ----------------------------------
  // 1 min timer task
  if( (millis() - timer_1min) > T1MIN) {
      timer_1min = millis();
      byte szF[12];
      // ----- publish uptime -----
      char valueCSTR[20];
      sprintf(valueCSTR, "%d", upTime);
      PublishMessage(OFFICEUPTIME, valueCSTR);
      // ----- publish temperature -----
      // float t = (bme.readTemperature() * 1.8) +32;
      float t = bme.readTemperature() - CALIBRATE;
      dtostrf( t, 5, 2, szF );
      // sprintf(valueCSTR, "%5.2f", t);
      sprintf( valueCSTR, "%s", szF );
      PublishMessage(OFFICETEMP, valueCSTR);
      // ----- publish relative humidity -----
      float h = bme.readHumidity();
      dtostrf( h, 5, 2, szF );
      // sprintf(valueCSTR, "%5.2f", h);
      sprintf( valueCSTR, "%s", szF );
      PublishMessage(OFFICERH, valueCSTR);
      // ----- publish atomouspheric pressure -----
      float p = bme.readPressure() / 100.0F; // hPa
      dtostrf( p, 6, 2, szF );
      // sprintf(valueCSTR, "%5.2f", p);
      sprintf( valueCSTR, "%s", szF );
      PublishMessage(OFFICEHG, valueCSTR);
  }

    
}
