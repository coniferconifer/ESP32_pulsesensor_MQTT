
/*  Pulse Sensor Amped 1.5    by Joel Murphy and Yury Gitman   http://www.pulsesensor.com

  ----------------------  Notes ----------------------  ----------------------
  This code:
  1) Blinks an LED to User's Live Heartbeat   PIN 13
  2) Fades an LED to User's Live HeartBeat    PIN 5
  3) Determines BPM
  4) Prints All of the Above to Serial

  Read Me:
  https://github.com/WorldFamousElectronics/PulseSensor_Amped_Arduino/blob/master/README.md
  ----------------------       ----------------------  ----------------------
*/
// June 5,2018
// ESP32 for MQTTversion by coniferconifer
// https://github.com/WorldFamousElectronics/PulseSensor_Amped_Arduino/
// bpm and ibi are transmitted to MQTT server (Thingsboard)
// see #ifdef ESP32 part

#define PROCESSING_VISUALIZER 1
#define SERIAL_PLOTTER  2

//  Variables
#define ESP32

#ifdef ESP32
#include "time.h"
#define TIMEZONE 9 //in Japan
#define NTP1 "time.google.com"
#define NTP2 "ntp.nict.jp"
#define NTP3 "ntp.jst.mfeed.ad.jp"

#define MQTT 3
#define LEDC_CHANNEL_0     0
#define LEDC_CHANNEL_1     1
#define LEDC_TIMER_8_BIT  8
#define LEDC_BASE_FREQ     5000
int pulsePin = 34;                 // Pulse Sensor purple wire connected to analog pin 34 , ADC6
int blinkPin = 13;                 // pin to blink led at each beat
int fadePin = 5;                  // pin to do fancy classy fading blink at each beat
#else
int pulsePin = 0;                 // Pulse Sensor purple wire connected to analog pin 0
int blinkPin = 13;                // pin to blink led at each beat
int fadePin = 5;                  // pin to do fancy classy fading blink at each beat
#endif


int fadeRate = 0;                 // used to fade LED on with PWM on fadePin

// Volatile Variables, used in the interrupt service routine!
volatile int BPM;                   // int that holds raw Analog in 0. updated every 2mS
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded!
volatile boolean Pulse = false;     // "True" when User's live heartbeat is detected. "False" when not a "live beat".
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.

// SET THE SERIAL OUTPUT TYPE TO YOUR NEEDS
// PROCESSING_VISUALIZER works with Pulse Sensor Processing Visualizer
//      https://github.com/WorldFamousElectronics/PulseSensor_Amped_Processing_Visualizer
// SERIAL_PLOTTER outputs sensor data for viewing with the Arduino Serial Plotter
//      run the Serial Plotter at 115200 baud: Tools/Serial Plotter or Command+L
//static int outputType = SERIAL_PLOTTER;
//static int outputType = PROCESSING_VISUALIZER;

#ifdef ESP32
static int outputType = MQTT;
#define VERSION "201800605"
#include <WiFi.h>
#include <PubSubClient.h>
#define VERBOSE
// index of WiFi access point
int AP = -1; // access point is not yet found , -1 means not WiFi , but Simple BLE mode
// do not use ADC2* pins with WiFi ON
// referer to "ADC2 Channel cannot be used when WiFi is in use #440"
// https://github.com/espressif/arduino-esp32/issues/440
//
// #define WIFI_POWERSAVE // intermittent WiFi connection reduces power consumpution from 0.18A to 0.08 A on average
//-------- Customise these values -----------
#include "credentials.h"
// credentials.h should include #define WIFI_SSID XXXXXX
//                        #define WIFI_PASS YYYYYY
//                       home , office
char* ssidArray[] = { WIFI_SSID , WIFI_SSID1, WIFI_SSID2};
char* passwordArray[] = {WIFI_PASS, WIFI_PASS1, WIFI_PASS2};
char* tokenArray[] = { TOKEN , TOKEN1, TOKEN2};
char* serverArray[] = {SERVER, SERVER1, SERVER2};
#define MQTTRETRY 1
#define DEVICE_TYPE "ESP32" // 
String clientId = DEVICE_TYPE ; //uniq clientID will be generated from MAC
char topic[] = "v1/devices/me/telemetry"; //for Thingsboard
#define MQTTPORT 1883 //for Thingsboard or MQTT server
#define WARMUPTIME 10000 // 10sec
#define INTERVAL 30000 // msec to send data to MQTT server
WiFiClient wifiClient;
PubSubClient client(serverArray[0], MQTTPORT, wifiClient);

void displayTime() {
  struct tm timeInfo;
  if ( AP == -1) return; // BLE mode , do nothing
  getLocalTime(&timeInfo);
  Serial.printf("Date: %04d/%02d/%02d %02d:%02d:%02d , ",
                timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday,
                timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);

}
#define MAX_TRY 15
int initWiFi() {
  int i ;
  int numaccesspt = (sizeof(ssidArray) / sizeof((ssidArray)[0]));

#ifdef VERBOSE
  Serial.print("Number of Access Point = "); Serial.println(numaccesspt);
#endif
  for (i = 0;  i < numaccesspt; i++) {
#ifdef VERBOSE
    Serial.print("WiFi connecting to "); Serial.println(ssidArray[i]);
#endif
    WiFi.mode(WIFI_OFF);
    WiFi.begin(ssidArray[i], passwordArray[i]);

    int j;
    for (j = 0; j < MAX_TRY; j++) {
      if (WiFi.status() == WL_CONNECTED) {

        int rssi = WiFi.RSSI();
        Serial.printf("RSSI= %d\n", rssi);

        configTime(TIMEZONE * 3600L, 0,  NTP1, NTP2, NTP3);
#ifdef VERBOSE
        Serial.print("WiFi connected, IP address: "); Serial.println(WiFi.localIP());
#endif
        return (i);
      }

      delay(500);
#ifdef VERBOSE
      Serial.print(".");
#endif

    }
#ifdef VERBOSE
    Serial.println(" can not connect to WiFi AP");
#endif

  }
  return (-1);
}



int initWiFi_retry() {
#ifdef VERBOSE
  Serial.print("initWiFi_retry() WiFi connecting to "); Serial.println(ssidArray[AP]);
#endif
  //  Serial.print(" "); Serial.print(passwordArray[AP]);
  WiFi.mode(WIFI_OFF);
  WiFi.begin(ssidArray[AP], passwordArray[AP]);

  int j;
  for (j = 0; j < MAX_TRY; j++) {
    if (WiFi.status() == WL_CONNECTED) {
#ifdef VERBOSE
      Serial.print("WiFi connected, IP address: "); Serial.println(WiFi.localIP());
#endif
      return (AP);
    }
    delay(500);
#ifdef VERBOSE
    Serial.print(".");
#endif
  }
#ifdef VERBOSE
  Serial.println(" can not connect to WiFi AP");
#endif
  return (-1);
}

boolean mqttflag = false;
void publishToMQTT() {
  // WiFi mode
  String payload = "{";
  payload += "\"bpm\":"; payload +=  BPM ; payload += ",";
  payload += "\"ibi\":"; payload += IBI ;
  payload += "}";

  // dont send too long string to MQTT server
  // max 128byte
  displayTime();
#ifdef VERBOSE
  Serial.println(payload);
  Serial.print("AP = "); Serial.print(AP);
#endif

#ifdef VERBOSE
  Serial.print(" Reconnecting client to "); Serial.println(serverArray[AP]);
#endif
  int mqttloop = 0;
  while (1) { // for thingsboard MQTT server
    mqttflag = client.connect(clientId.c_str(),  tokenArray[AP], NULL);
    if (mqttflag == true) break;
    Serial.print("-"); delay(500);
    mqttloop++;
    if (mqttloop > MQTTRETRY) { //there may be something wrong
      mqttflag = false;
      initWiFi_retry();
      // ESP.restart();
      break;
    }
  }

  if (mqttflag == true) {
#ifndef DEEPSLEEP
    if (millis() > WARMUPTIME) { // check if pulse sensor get enough warm up time
#else
    if (true) { //in case of DEEP SLEEP , MQTT publish any time
#endif
      if (client.publish(topic, (char*) payload.c_str())) {
#ifdef VERBOSE
        Serial.println("Publish ok");
#endif
      } else {
#ifdef VERBOSE
        Serial.println("Publish failed");
#endif
      }
    } // WARMUP decision end
  } else {
#ifdef VERBOSE
    Serial.println("unable to connect to MQTT server");
#endif
  }

}
#endif  //ESP32


void setup() {
  pinMode(blinkPin, OUTPUT);        // pin that will blink to your heartbeat!
  pinMode(fadePin, OUTPUT);         // pin that will fade to your heartbeat!
  Serial.begin(115200);             // we agree to talk fast!
#ifdef ESP32
  // generate uniq clientId
  uint64_t chipid;
  chipid = ESP.getEfuseMac();
  clientId += "-";
  //  clientId += DEVICE_ID;
  clientId += String((uint32_t)chipid, HEX);
  Serial.println("clientId :" + clientId);

  AP = initWiFi();
  if ( AP != -1) {  // found  WiFi AP
    client.setClient(wifiClient);
    client.setServer(serverArray[AP], MQTTPORT); // MQTT server for NodeRED or MQTT by Thingsboarxd
  }
#endif

  interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS
  // IF YOU ARE POWERING The Pulse Sensor AT VOLTAGE LESS THAN THE BOARD VOLTAGE,
  // UN-COMMENT THE NEXT LINE AND APPLY THAT VOLTAGE TO THE A-REF PIN
  //   analogReference(EXTERNAL);
#ifdef ESP32
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT) ; // 8bit precision
  ledcAttachPin(fadePin, LEDC_CHANNEL_0) ; // assaign LED1 to CH0
#endif
}

long counter; long precounter;
boolean loopflag = false;
//  Where the Magic Happens
void loop() {
  long counter;
  serialOutput() ;

  if (QS == true) {    // A Heartbeat Was Found
    // BPM and IBI have been Determined
    // Quantified Self "QS" true when arduino finds a heartbeat
    fadeRate = 255;         // Makes the LED Fade Effect Happen
    // Set 'fadeRate' Variable to 255 to fade LED with pulse
    serialOutputWhenBeatHappens();   // A Beat Happened, Output that to serial.
    QS = false;                      // reset the Quantified Self flag for next time
  }

  ledFadeToBeat();                      // Makes the LED Fade Effect Happen
  delay(20);                             //  take a break

#ifdef ESP32
  counter = millis();
  counter = counter / INTERVAL; // send data to MQTT server every INTERVAL msec
  if ( counter == precounter) {
    if (AP != -1) {
      publishToMQTT();
      precounter = counter + 1;
    }
  }
#endif

}





void ledFadeToBeat() {
  fadeRate -= 15;                         //  set LED fade value
  fadeRate = constrain(fadeRate, 0, 255); //  keep LED fade value from going into negative numbers!
#ifdef ESP32
  ledcWrite(LEDC_CHANNEL_0, fadeRate) ;
#else
  analogWrite(fadePin, fadeRate);         //  fade LED
#endif

}
