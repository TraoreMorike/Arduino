#include <SHT3x.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "pb_common.h"
#include  "pb.h"
#include "pb_encode.h"

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

 
#define R2              100000.0
#define R1              350000.0
#define VREF            1.0
#define ADC_RES         1024.0


// Wi-Fi credentials
const char* ssid = "";
const char* password = "";

// MQTT Server address
const char* mqtt_server = "";

float temperature;
float humidity;
float batteryVoltage;

uint8_t buffer[128];

// Initializes the espClient. You should change the espClient name if you have multiple ESPs running in your home automation system
WiFiClient espClient;
PubSubClient client(espClient);


SHT3x Sensor( 0x45,         //Set the address
                SHT3x::Zero,  //Functions will return zeros in case of error
                255,          //If you DID NOT connected RESET pin
                SHT3x::SHT30, //Sensor type
                SHT3x::Single_LowRep_ClockStretch //Low repetability mode
                );
              

void setup() {

    /* Initialize UART communication at 115200 bauds */
    Serial.begin(115200);
    /* Initialize SHT30 Sensor */
    Sensor.Begin();
    /* Connect to Wi-Fi */
    setup_wifi();
    /* Connect to the MQTT server */
    client.setServer(mqtt_server, 1883);
    /* Set the callback function */
    client.setCallback(callback);


}

void loop() {

    /* Check MQTT connection */
    if (!client.connected()) {
    /* Connect to Wi-Fi */
    //setup_wifi();
    /* Reconnect if connection lost */
    mqttReconnect();
    }
    if(!client.loop()) { 
      client.connect("SENSOR", "your_username", "your_password");
    }

    /* Read temperature & humidity */
    Sensor.UpdateData();

    temperature = Sensor.GetTemperature();

    humidity = Sensor.GetRelHumidity();

    /* Read battery voltage */
    getBatteryVoltage(analogRead(0), &batteryVoltage);

    /* Debug output */
    //Format : 
    // T  21  |   H 48  | B 4.19
    
    Serial.print("T\t"), Serial.print(Sensor.GetTemperature(),2);
    Serial.print("\t|\t");
    Serial.print("H\t"), Serial.print(Sensor.GetRelHumidity(),2);
    Serial.print("\t|\t");
    Serial.print("B\t"), Serial.println(batteryVoltage);

    SensorMessage message = SensorMessage_init_zero; 

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    /* set sensor UID */
    message.sensor_uid = 0xFF;

    /* set temperature payload */
    message.sensor_temperature = temperature;

    /* set humidity payload */
    message.sensor_humidity = humidity;

    /* set battery voltage */
    message.battery_voltage = batteryVoltage;

    /* encode the mesage */
    bool status = pb_encode(&stream, SensorMessage_fields, &message);

    if (!status) {
      Serial.println("Fail to encode");
    return;
    }

    Serial.print("Message Length: ");
    Serial.println(stream.bytes_written);

    Serial.print("Message: ");

    for (int i=0; i<stream.bytes_written; i++) {
      Serial.printf("%02X", buffer[i]);
    }
   
   Serial.println();

   if (!client.publish("room/sensor", buffer, stream.bytes_written)) {
     Serial.println("Fail to publish");

   }

  ESP.deepSleep(10e6); 
  
  
 }




/*********************************************************************************************/

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

/******************************************************************/

void callback(String topic, byte* message, unsigned int length) {
  
  DEBUG_PRINT("Message arrived on topic: ");
  DEBUG_PRINT(topic);
  DEBUG_PRINT(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    DEBUG_PRINT((char)message[i]);
    messageTemp += (char)message[i];
  }
  DEBUG_PRINTLN();

}

/******************************************************************/

void mqttReconnect() {
  
  /* Loop until we're reconnected */
  while (!client.connected()) {
    DEBUG_PRINT("Attempting MQTT connection...");
    /* Attempt to connect */
    if (client.connect("SENSOR", "your_username", "your_password")) {
      DEBUG_PRINTLN("connected");  
    } else {
      DEBUG_PRINT("failed, rc=");
      DEBUG_PRINT(client.state());
      DEBUG_PRINTLN(" try again in 5 seconds");
      /* Wait 5 seconds before retrying */
      delay(5000);
    }
  }
}

/******************************************************************/

/** Read the battery voltage.
 * This function read the battery voltage after performing and ADC read
 * @param rawAdc : Raw adc measurement
 * @param voltage : output pointer, convert the raw adc measurement into volts
 * @return none
 */
void getBatteryVoltage(uint16_t rawAdc, float* voltage) {
  
  float adcVolts; 

  adcVolts = (uint16_t)rawAdc *(VREF/ADC_RES);
  
  *voltage = adcVolts / (R2/(R1+R2));
  
}
