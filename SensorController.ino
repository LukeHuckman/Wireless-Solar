/*
 * FYP/Academic Project WIA3002/WIA3003
 * Title: Solar Powered LED Street Light System with Battery Storage (and Sensor Array)
 * 
 * Wireless Sensor Monitor and Controller
 * Device used: Arduino NANO 33 IoT
 */
#include <SPI.h>
#include <WiFiNINA.h>

int dustLed = 19;
int dustData = 20;
int humidData = 21;

void setup() {
  pinMode(dustLed,OUTPUT);
  pinMode(dustData,INPUT);
  pinMode(humidData,INPUT);
  // TODO solar power and battery system input

  digitalWrite(dustLed, HIGH);
  
  // Wifi connection for uploading sensor data to server
  char ssid[] = "";
  char pw[] = "";

  Serial.begin(9600);
  while (!Serial);

  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pw);
    delay(5000);
  }

  Serial.println("Wifi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  // TODO establish connection to DB server
}

void loop() {
  float dust = analogRead(dustData);
  float humidity = analogRead(humidData);
  Serial.print("Air quality: ");
  Serial.println(dustData);
  Serial.print("Humidity: ");
  Serial.println(dustData);
  delay(3000);
}
