#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "pb_common.h"
#include  "pb.h"
#include "pb_decode.h"

#include "src/sensor.pb.h"

/**********************************************
 * Macros definitions
 *********************************************/
#define UART_DEBUG          1 // Set to 0 to desactivate debug

#if UART_DEBUG
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINT(x)    Serial.print(x)
#else
  #define DEBUG_PRINTLN(x)  0
  #define DEBUG_PRINT(x)    0
#endif

// Wi-Fi credentials
const char* ssid = "";
const char* password = "";

// MQTT Server address
const char* mqtt_server = "";

byte sensorUID; 
float temperature;
float humidity;
float batteryVoltage;




// Initializes the espClient. You should change the espClient name if you have multiple ESPs running in your home automation system
WiFiClient espServer;
PubSubClient client(espServer);

void setup() {
    /* Initialize UART communication at 115200 bauds */
    Serial.begin(115200);
    /* Connect to Wi-Fi */
    setup_wifi();
    /* Connect to the MQTT server */
    client.setServer(mqtt_server, 1883);
    /* Set the callback function */
    client.setCallback(callback);

    client.subscribe("room/sensor");
}

void loop() {

  /* Check MQTT connection */
    if (!client.connected()) {
    /* Reconnect if connection lost */
    reconnect();
    }
    if(!client.loop()) { 
      client.connect("SERVER", "your_username", "your_password");
    }

}


/*-----------------------------------------------------------------------------------*/

void setup_wifi() {
  
  WiFi.begin(ssid, password);
  DEBUG_PRINT("Connecting to ");
  DEBUG_PRINT(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    DEBUG_PRINT(".");
 }
    DEBUG_PRINTLN("");
    DEBUG_PRINT("WiFi connected - ESP IP address: ");
    DEBUG_PRINTLN(WiFi.localIP()); 
}

/*-----------------------------------------------------------------------------------*/

void callback(String topic, byte* message, unsigned int length) {
  
  uint8_t buffer[128];

  DEBUG_PRINT("Message arrived on topic: ");
  DEBUG_PRINT(topic);
  DEBUG_PRINT(". Message: ");

  for (int i = 0; i < length; i++) {
    Serial.printf("%02X", message[i]);
    buffer[i] = message[i];
  }
  DEBUG_PRINTLN();


  if(topic=="room/sensor"){

     SensorMessage inMessage = SensorMessage_init_zero;

     pb_istream_t stream = pb_istream_from_buffer(buffer, length);

     /* dncode the mesage */
    bool status = pb_decode(&stream, SensorMessage_fields, &inMessage);

    if (!status) {
      Serial.println("Fail to decode");
    return;
    } else {
      
    /* set sensor UID */
    sensorUID = inMessage.sensor_uid;

    /* set temperatre payload */
    temperature = inMessage.sensor_temperature;

    /* set humidity payload */
     humidity = inMessage.sensor_humidity;

    /* set battery voltage */
    batteryVoltage = inMessage.battery_voltage;
    }

    Serial.print("UID\t"), Serial.print(sensorUID);
    Serial.print("\t|\t");
    Serial.print("T\t"), Serial.print(temperature);
    Serial.print("\t|\t");
    Serial.print("H\t"), Serial.print(humidity);
    Serial.print("\t|\t");
    Serial.print("B\t"), Serial.println(batteryVoltage);
    
  }
  DEBUG_PRINTLN();
}

/*-----------------------------------------------------------------------------------*/

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    DEBUG_PRINT("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("SERVER", "your_username", "your_password")) {
      DEBUG_PRINTLN("connected");  
      client.subscribe("room/sensor");
    } else {
      DEBUG_PRINT("failed, rc=");
      DEBUG_PRINT(client.state());
      DEBUG_PRINTLN(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
