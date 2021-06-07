/*
 * FYP/Academic Project WIA3002/WIA3003
 * Title: Solar Powered LED Street Light System with Battery Storage (and Sensor Array)
 * 
 * Wireless Sensor Monitor and Controller
 * Device used: Arduino NANO 33 IoT
 */
#include <SPI.h>
#include <WiFiNINA.h>
#include <dhtnew.h>
#include <PubSubClient.h>

// Credentials
#define WIFISSID ""
#define WIFIPASS ""
#define MqttClient_Id "Node2"
#define MqttClient_user ""
#define MqttClient_password ""
#define SERVER ""

// Pins
int humidData = 21;
const int sharpLEDPin = 19;   
const int sharpVoPin = A6;  

// Dust sensor setup
#define USE_AVG
#ifdef USE_AVG
#define N 50
static unsigned long VoRawTotal = 0;
static int VoRawCount = 0;
#endif // USE_AVG
static float K = 0.5;
static float Voc = 0.6;

// Function prototypes 
float getDust();
void mqttReconnect();
void mqttPublish(char* topic, float payload);

// MQTT Topics
#define MqttTopic_Humidity "environment/Node2/humidity"
#define MqttTopic_Temperature "environment/Node2/temperature"
#define MqttTopic_Dust "environment/Node2/dust"

// MQTT Initialisation
WiFiClient wifi;
PubSubClient client(wifi);

void setup() {
  pinMode(sharpLEDPin,OUTPUT);
  pinMode(sharpVoPin,INPUT);
  pinMode(humidData,INPUT);
  
  // Wifi connection for uploading sensor data to server

  Serial.begin(9600);
  while (!Serial);

  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.println(WIFISSID);
    status = WiFi.begin(WIFISSID, WIFIPASS);
    delay(5000);
  }

  Serial.println("Wifi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(SERVER,1883);
  client.connect(MqttClient_Id, MqttClient_user, MqttClient_password);
}

void loop() {
  if (!client.connected()) {
    mqttReconnect();
  }
  DHTNEW dht(humidData);
  dht.read();
  float temp = dht.getTemperature();
  float humid = dht.getHumidity();
  float dust = getDust();
  client.loop();
  mqttPublish(MqttTopic_Temperature, temp);
  mqttPublish(MqttTopic_Humidity, humid);
  mqttPublish(MqttTopic_Dust, dust);
  delay(1000);
}

float getDust() {
  digitalWrite(sharpLEDPin, LOW);
  delayMicroseconds(280);
  int VoRaw = analogRead(sharpVoPin);
  digitalWrite(sharpLEDPin, HIGH);
  delayMicroseconds(9620);
  float Vo = VoRaw;
 
  Vo = Vo / 1024.0 * 3.3;
  // Convert to Dust Density in units of ug/m3.
  float dV = Vo - Voc;
  if ( dV < 0 ) {
    dV = 0;
    Voc = Vo;
  }
  float dustDensity = dV / K * 100.0;
  return dustDensity;
}

void mqttReconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MqttClient_Id, MqttClient_user, MqttClient_password)) {
      Serial.println("connected");
      //client.subscribe("esp32/output");
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void mqttPublish(char* topic, float payload) {
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(payload,3);
  String sstring = String(payload, 3);
  client.publish(topic, sstring.c_str(), true);
}
