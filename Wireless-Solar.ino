/*
 * FYP/Academic Project WIA3002/WIA3003
 * Title: Solar Powered LED Street Light System with Battery Storage (and Sensor Array)
 * 
 * Wireless Sensor Monitor and Controller
 * Device used: Arduino NANO 33 IoT
 */
#include <SPI.h>
#include <WiFiNINA.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <SAMDTimerInterrupt.h>
#include <SAMD_ISR_Timer.h>

// Credentials
#define WIFISSID ""
#define WIFIPASS ""
#define MqttClient_Id "Node1"
#define MqttClient_user ""
#define MqttClient_password ""
#define SERVER ""

// Pins
#define DHTPIN 14
#define DHTTYPE DHT22
const int sharpLEDPin = 16;   
const int sharpVoPin = A1;  

// Timer setup
#define HW_TIMER_INTERVAL_MS 10
#define samplerate 50L
SAMDTimer ITimer(TIMER_TC3);
SAMD_ISR_Timer ISR_Timer;
void TimerHandler(void){
  ISR_Timer.run();
}

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
void getTemp();
void getHumid();
void getDust();
void mqttReconnect();
void mqttPublish(char* topic, float payload);

// MQTT Topics
#define MqttTopic_Humidity "environment/Node1/humidity"
#define MqttTopic_Temperature "environment/Node1/temperature"
#define MqttTopic_Dust "environment/Node1/dust"

// MQTT Initialisation
WiFiClient wifi;
PubSubClient client(wifi);

DHT dht(DHTPIN, DHTTYPE);

// Sensor Data Variables
float temp, humid, dust;

void setup() {
  pinMode(sharpLEDPin,OUTPUT);
  pinMode(sharpVoPin,INPUT);
  
  dht.begin();
  
  Serial.begin(9600);
  while (!Serial);

  // Wifi connection for uploading sensor data to server
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

  // Establish MQTT connection to server
  client.setServer(SERVER,1883);
  while (!client.connected()) {
    Serial.print("Connecting to MQTT server...");
    client.connect(MqttClient_Id, MqttClient_user, MqttClient_password);
  }
  Serial.println("successful.");

  if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, TimerHandler))
  {
    Serial.print(F("Starting ITimer OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer. Select another freq. or timer"));

  // Timer interrupts
  ISR_Timer.setInterval(samplerate*60, getTemp);
  ISR_Timer.setInterval(samplerate*60, getHumid);
  ISR_Timer.setInterval(samplerate*60, getDust);
  ISR_Timer.setInterval(samplerate*80, wifiReconnect);
  ISR_Timer.setInterval(samplerate*80, mqttReconnect);
}

void loop() {}

void wifiReconnect() {
  while (status != WL_CONNECTED) {
    Serial.print("WiFi disconnected. Reconnecting to ");
    Serial.println(WIFISSID);
    status = WiFi.begin(WIFISSID, WIFIPASS);
    delay(5000);
  }
}

void getTemp() {
  temp = dht.readTemperature();
  mqttPublish(MqttTopic_Temperature, temp);
}

void getHumid() {
  humid = dht.readHumidity();
  mqttPublish(MqttTopic_Humidity, humid);
}

void getDust() {
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
  dust = dV / K * 100.0;
  mqttPublish(MqttTopic_Dust, dust);
}

void mqttReconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MqttClient_Id, MqttClient_user, MqttClient_password)) {
      Serial.println("connected");
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
