

// ===== Object Instances =====
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);
DynamicJsonDocument jsonDoc(1024);

// ===== MQTT VARIABLES =====
const char TOPIC[ ] = "home/office";
const char SUBTOPIC[ ] = "home/office/#";          // MQTT device subscripton Topic
const char DEVSTATUS[ ] = "home/office/status";    // MQTT device status toic
const char DEVCMD[ ] = "home/office/command";      // MQTT device command topic
const char WILLMSG[ ] = "{\"office_monitor\": \"offline\"}";
const char ONLINEMSG[ ] = "{\"office_monitor\": \"online\"}";

//------------------------------------------
void mqttConnect() {
    while (!mqttClient.connected()) {
        Dprintln("Attempting MQTT connection...");
        
        if (mqttClient.connect(
                MQTT_CLIENT, // client id
                MQTT_USERNAME, // username
                MQTT_PASSWORD, // password
                DEVSTATUS, // will topic
                1,  // will qos
                true, // will retain
                WILLMSG // will message
            )) {
            Dprintln("Connected!");
            mqttClient.publish(DEVSTATUS,"{\"office_monitor\": \"online\"}", true);
            mqttClient.subscribe(DEVCMD);
        }
        else {
            Dprint("Failed to connect, rc=");
            Dprintln(mqttClient.state());
            delay(5000);  // wait 5 sec before retry
        }
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++    Publish Data Over MQTT     ++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++
void PublishMessage(String topic, String message, boolean retained)
{
    char pub_topic[80] = {0};
    char pub_msg[80] = {0};

    topic.toCharArray(pub_topic, topic.length() + 1);
    message.toCharArray(pub_msg, message.length() + 1);

    if(!mqttClient.publish(pub_topic, pub_msg, retained))
    {
      Dprintln("Failed to publish online message");
  }
}

// ######################################
// #####        MQTT HANDLERS       #####
// ######################################
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    // handle message arrived
    payload[length] = '\0';
    String s = String((char*)payload);
    Dprint(topic);
    Dprint(": ");
    Dprintln(s);

    auto error = deserializeJson(jsonDoc, payload);
    if (error) {
        Serial.print(F("deserializeJson() failed with code "));
        Serial.println(error.c_str());
        PublishMessage(TOPIC, jsonKeyValue("json_parse", "failed"));
    }
    else  {   // Parse incoming command
        JsonObject root = jsonDoc.as<JsonObject>();
        String command = root["command"];
        if(command == "status") {
            Dprintln("status command");
            // PublishMessage(TOPIC, jsonKeyValue("motion", motionState));
        }
        else {
            PublishMessage(TOPIC, jsonKeyValue("command", "invalid"));
        }
        // End of JSON command parsing
    }
    // End of JSON parsing
}
