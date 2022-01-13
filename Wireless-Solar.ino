/*
 * FYP/Academic Project WIA3002/WIA3003
 * Title: Solar Powered LED Street Light System with Battery Storage (and Sensor Array)
 * 
 * Wireless Sensor Monitor and Controller
 * Device used: Arduino NANO 33 IoT
 * 
 * Pins configuration:-
 * 1) D2 and D6 for complementary switching for cell balancer (Using TCCO)
 * 2) D7 for MPPT (Using TCC1)
 * 3) D4 for battery charging with 50% duty cycle (using TCC1)
 * 4) D3 for relay
 * 5) D14(A0) for temperature sensor
 * 6) D15(A1) for dust sensor
 * 7) D16(A2) for dust sensor
 * 8) D17(A3) for VPV
 * 9) D18(A4) for Vbattery
 * 10) D19 (A5) for current sensor
 * 11) D20 (A6) for LDR
 */

// Library Includes
#include <SPI.h> // [internal library]
#include <WiFiNINA.h> // WiFiNINA 1.8.13 (Arduino)
#include <DHTesp.h> // DHT sensor library for ESPx 1.18 (beegee_tokyo)
#include <PubSubClient.h> // PubSubClient 2.8 (Nick O'Leary)
#include <SAMDTimerInterrupt.h> // SAMD_TimerInterrupt 1.5.0 (Khoi Hoang)
#include <SAMD_ISR_Timer.h> // SAMD_TimerInterrupt 1.4.0 (Khoi Hoang)
#include <WiFiUdp.h>  // [internal library]
#include <NTPClient.h> // NTPClient 3.2.0 (Arduino)

// Credentials
#define WIFISSID ""
#define WIFIPASS ""
#define MqttClient_Id ""
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
void getPowerStat(void);
void getLDR(void);
void getVbat(void);
void powerSystem(void);
void getDHT(void);
//void getTemp();
//void getHumid();
void getDust(void);
void reconnect(void);
void wifiReconnect(void);
void mqttReconnect(void);
void mqttPublish(char* topic, float payload);
void updateTime(void);

// WiFi Status
int status = WL_IDLE_STATUS;
int test;
int test_count;

// NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
int hourOfDay;

// MQTT Topics
#define MqttTopic_Voltage "power/Node1/voltage"
#define MqttTopic_Current "power/Node1/current"
#define MqttTopic_Ppv "power/Node1/ppv"
#define MqttTopic_Vbat "power/Node1/vbat"
#define MqttTopic_LDR "power/Node1/ldr"
#define MqttTopic_Relay "power/Node1/relay"
#define MqttTopic_Humidity "environment/Node1/humidity"
#define MqttTopic_Temperature "environment/Node1/temperature"
#define MqttTopic_Dust "environment/Node1/dust"
#define MqttTopic_Debug "management/Node1/debug"

// MQTT Initialisation
WiFiClient wifi;
PubSubClient client(wifi);

DHTesp dht;

// Global Variables
TempAndHumidity dhtdata;
float
  voltage,
  current,
  Ppv,
  Pprev = 0,
  Vprev = 0,
  Vbat,
  LDR,
  Dutycycle = 150,
  Dutycycle2 = 150,
  dust;
bool charging = false, batlow;
int Dutycycle3;
String temp, humid, netTime;

void setup() {
  pinMode(3,OUTPUT);

  // Feed GCLK0 at 48MHz to TCC0 and TCC1
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Enable GCLK0 as a clock source (1 << 14)
                      GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK0 at 48MHz (0 << 8)
                      GCLK_CLKCTRL_ID_TCC0_TCC1;   // Route GCLK0 to TCC0 and TCC1 (0x1A << 0)
  while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

  // Enable the port multiplexer for pins D6
  PORT->Group[g_APinDescription[6].ulPort].PINCFG[g_APinDescription[6].ulPin].bit.PMUXEN = 1;

  // D6 is on EVEN port pin PA04 and TCC0/WO[0] channel 0 is on peripheral E
  PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg |= PORT_PMUX_PMUXE_E;

  // Enable the port multiplexer for pins D2
  PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1;

  // D2 is on EVEN port pin PB10 and TCC0/WO[4] channel 0 is on peripheral F
  PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;

  // Normal (single slope) PWM operation: timer countinuously counts up to PER register value and then is reset to 0
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;          // Setup single slope PWM on TCC0  (2 << 0)
  while (TCC1->SYNCBUSY.bit.WAVE);                 // Wait for synchronization

  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |      // Set prescaler to 1, 48MHz/1 (0 << 8)
                    TCC_CTRLA_PRESCSYNC_PRESC;      // Set the reset/reload to trigger on prescaler clock (1 << 12)

  REG_TCC0_PER = 1453;                            // Set the frequency of the PWM on TCC0 to 33kHz: 48MHz / (1 * 1453 + 1) = 33kHz
  while (TCC0->SYNCBUSY.bit.PER);                  // Wait for synchronization

  REG_TCC0_CC0 = 726;                           // TCC1 CC0 - 50% duty cycle
  while (TCC0->SYNCBUSY.bit.CC0);                 // Wait for synchronization

  // dead time generation (edit the value to adjust the dead time on Low side (bit: 16) and High side (bit:24))
  REG_TCC0_WEXCTRL |= (50 << 16) | (50 << 24) | (1 << 8) | (2 << 0) ;  // (1 << 8): implement dead time at output WO[0] and WO[4]
  // (2 << 0): output matrix (all output use CC0

  TCC0->CTRLA.bit.ENABLE = 1;                     // Enable the TCC0 counter
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization



  // Feed GCLK0 at 48MHz to TCC0 and TCC1
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Enable GCLK0 as a clock source
                      GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK0 at 48MHz
                      GCLK_CLKCTRL_ID_TCC0_TCC1;   // Route GCLK0 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

  // Enable the port multiplexer for pins D7
  PORT->Group[g_APinDescription[7].ulPort].PINCFG[g_APinDescription[7].ulPin].bit.PMUXEN = 1;

  // D7 is on EVEN port pin PA06 and TCC1/WO[0] channel 0 is on peripheral E
  PORT->Group[g_APinDescription[7].ulPort].PMUX[g_APinDescription[7].ulPin >> 1].reg |= /*PORT_PMUX_PMUXO_E |*/ PORT_PMUX_PMUXE_E;

   // Enable the port multiplexer for pins D4
  PORT->Group[g_APinDescription[4].ulPort].PINCFG[g_APinDescription[4].ulPin].bit.PMUXEN = 1;

  // D7 is on EVEN port pin PA06 and TCC1/WO[0] channel 0 is on peripheral E
  PORT->Group[g_APinDescription[4].ulPort].PMUX[g_APinDescription[4].ulPin >> 1].reg |= /*PORT_PMUX_PMUXO_E |*/ PORT_PMUX_PMUXO_E;
  
  // Normal (single slope) PWM operation: timer countinuously counts up to PER register value and then is reset to 0
  TCC1->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;          // Setup single slope PWM on TCC1
  while (TCC1->SYNCBUSY.bit.WAVE);                 // Wait for synchronization

  TCC1->CTRLA.reg = TC_CTRLA_PRESCALER_DIV8 |      // Set prescaler to 8, 48MHz/8 = 6MHz
                    TC_CTRLA_PRESCSYNC_PRESC;      // Set the reset/reload to trigger on prescaler clock
  
  TCC1->PER.reg = 300;                            // Set the frequency of the PWM on TCC1 to 1kHz: 48MHz / (8 * 5999 + 1) = 1kHz
  while (TCC1->SYNCBUSY.bit.PER);                  // Wait for synchronization
  
  TCC1->CC[1].reg = 150;                        // TCC1 CC0 - 50% duty cycle on D4
  while (TCC1->SYNCBUSY.bit.CC1);                  // Wait for synchronization
  
  TCC1->CC[0].reg = 150;                          // TCC1 CC0 - 50% duty cycle on D7
  while (TCC1->SYNCBUSY.bit.CC0);                  // Wait for synchronization
  
  TCC1->CTRLA.bit.ENABLE = 1;                     // Enable the TCC1 counter
  while (TCC1->SYNCBUSY.bit.ENABLE);

  pinMode(sharpLEDPin,OUTPUT);
  pinMode(sharpVoPin,INPUT);
  
  dht.setup(DHTPIN, DHTesp::DHTTYPE);
  
  Serial.begin(9600);

  // Wifi connection for uploading sensor data to server
  while (status != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.println(WIFISSID);
    status = WiFi.begin(WIFISSID, WIFIPASS);
    delay(5000);
  }

  Serial.println("Wifi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Start NTP client
  timeClient.begin();
  timeClient.update();
  hourOfDay = timeClient.getFormattedTime().substring(0,2).toInt();
  
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
  ISR_Timer.setInterval(samplerate*2, powerSystem);
  ISR_Timer.setInterval(samplerate*100, getDHT);
  ISR_Timer.setInterval(samplerate*100, getDust);
  ISR_Timer.setInterval(samplerate*600, reconnect);
  //ISR_Timer.setInterval(samplerate*80, wifiReconnect);
  //ISR_Timer.setInterval(samplerate*80, mqttReconnect);
  ISR_Timer.setInterval(samplerate*20, updateTime);
}

//int counter = 1;

void loop() {}

void reconnect() {
  wifiReconnect();
  //updateTime();
  mqttReconnect();
}

void wifiReconnect() {
  mqttPublish(MqttTopic_Debug, "wifiReconnect");
  int retries = 1;
  while (status != WL_CONNECTED) {
    Serial.print("WiFi disconnected. Reconnecting to ");
    Serial.println(WIFISSID);
    status = WiFi.begin(WIFISSID, WIFIPASS);
    if(retries <= 10){
      delay(5000);
      retries++;
    }
    else {
      break;
    }
  }
  retries = 1;
}

void updateTime() {
  mqttPublish(MqttTopic_Debug, "updateTime");
  timeClient.update();
  hourOfDay = timeClient.getFormattedTime().substring(0,2).toInt();
}

void getPowerStat() {
  mqttPublish(MqttTopic_Debug, "getPowerStat");
  current = 0;
  mqttPublish(MqttTopic_Debug, "start_getPowerStat");
  for (int i = 0; i <= 20; i++) {
    float vol = analogRead(A5) * (3.3 / 1024.0);
    current = current + (15 * (vol - 2.5))/0.625;
  }
  
  current = current/20;
  
  if (current < 0)
    current = 0;
    voltage = 0;
  for (int i = 0; i <= 20; i++) {
    voltage += analogRead(A3) * (3.3 / 1024.0);
  }
  voltage /= 20;
  voltage = (voltage * 103.3/3.3) ;
  if (voltage <0)
    voltage=0;

  Ppv = voltage * current;

  mqttPublish(MqttTopic_Voltage, String(voltage, 3));
  mqttPublish(MqttTopic_Current, String(current, 3));
  mqttPublish(MqttTopic_Ppv, String(Ppv, 3));
}

void getLDR() {
  mqttPublish(MqttTopic_Debug, "getLDR");
  LDR = analogRead(A6) * (1000/1024.0);
  mqttPublish(MqttTopic_LDR, String(LDR, 3));
}

void getVbat() {
  mqttPublish(MqttTopic_Debug, "getVbat");
  Vbat = 0;
  for(int i = 0; i < 20; i++) {
    Vbat += analogRead(A4) * (3.3 / 1024.0)+0.06;
  }
  Vbat /= 20;
  Vbat = (Vbat * 103.3/3.3);
  mqttPublish(MqttTopic_Vbat, String(Vbat, 3));
}

void powerSystem() {
  //Serial.println(timeClient.getFormattedTime() + "\n" + hourOfDay);
  /*
  counter++;
  switch(counter){
    case 50:
      getDHT();
      getDust();
      test_count=1;
      break;
    case 100:
      reconnect();
      test_count=1;
      break;
  }
  if(counter == 100)
    counter = 1;
  */
  
  mqttPublish(MqttTopic_Debug, "powerSystem");
  getPowerStat();
  getLDR();
  getVbat();
  if (LDR > 550 && hourOfDay < 11 || hourOfDay >= 23){
    if(!charging && Vbat < 15){
      charging = true;
    }
    else if(charging && Vbat >= 13) {
      batlow = false;
    }
    else if(charging && Vbat >= 16){
      charging = false;
    }
    
    if(charging){
      if (test==0){
        test=1;
        Dutycycle = 150;
      }
      if ((Ppv > Pprev) && (voltage > Vprev)){
        if (Dutycycle > 20)
           Dutycycle -= 5;
      }
      else if ((Ppv > Pprev) && (voltage < Vprev)){
        if (Dutycycle < 280)
          Dutycycle += 5;
      }
      else if ((Ppv < Pprev) && (voltage > Vprev)){
        if (Dutycycle > 20) 
          Dutycycle -=  5;
      }
      // if ((Ppv < Pprev) && (voltage < Vprev)){
      else {
        if (Dutycycle < 280)
          Dutycycle +=  5;
      }
      TCC1->CCB[0].reg = Dutycycle;     // TCC1 CCB1 - 25% duty cycle on D7
      while (TCC1->SYNCBUSY.bit.CCB0);  
      // analogWrite(9,500);
      Dutycycle2 = 150;
      TCC1->CCB[1].reg = Dutycycle2;    // TCC1 CCB1 - 25% duty cycle on D4
      while (TCC1->SYNCBUSY.bit.CCB1);
      // analogWrite(6, Dutycycle2);
    }
    else {
      TCC1->CCB[1].reg = 0;    // TCC1 CCB1 - 25% duty cycle on D4
      while (TCC1->SYNCBUSY.bit.CCB1);
    }
    Dutycycle3 = LOW;
    digitalWrite (3,Dutycycle3);  // Turn off relay
  }
  else { 
    test=0;
    Dutycycle = 0;
    TCC1->CCB[0].reg = Dutycycle;     // TCC1 CCB1 - 25% duty cycle on D7
    while (TCC1->SYNCBUSY.bit.CCB0);
    Dutycycle2 = 0;
    TCC1->CCB[1].reg = Dutycycle2;    // TCC1 CCB1 - 25% duty cycle on D4
    while (TCC1->SYNCBUSY.bit.CCB1);
    if(Vbat >= 12 && !batlow) {
      Dutycycle3 = HIGH;
      digitalWrite (3,Dutycycle3);
    }
    else {
      batlow = true;
      Dutycycle3 = LOW;               // Battery voltage too low, turn off relay 
      digitalWrite (3,Dutycycle3);
    }
  }
  Pprev = Ppv;
  Vprev = voltage;
  mqttPublish(MqttTopic_Relay, String(Dutycycle3));
}

void getDHT() {
  mqttPublish(MqttTopic_Debug, "getDHT");
  dhtdata = dht.getTempAndHumidity();
  temp = dhtdata.temperature;
  humid = dhtdata.humidity;
  mqttPublish(MqttTopic_Temperature, temp);
  mqttPublish(MqttTopic_Humidity, humid);
}

void getDust() {
  mqttPublish(MqttTopic_Debug, "getDust");
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
  mqttPublish(MqttTopic_Dust, String(dust, 5));
}

void mqttReconnect() {
  mqttPublish(MqttTopic_Debug, "mqttReconnect");
  int retries = 1;
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MqttClient_Id, MqttClient_user, MqttClient_password)) {
      Serial.println("connected");
      retries = 1;
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      if(retries <= 10){
        delay(5000);
        retries++;
      }
      else {
        break;
      }
    }
  }
}

void mqttPublish(char* topic, String payload) {
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(payload);
  client.publish(topic, payload.c_str(), true);
}
