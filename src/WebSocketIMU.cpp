/*
   WebSocketClient.ino

    Created on: 24.05.2015
    Aurther Ben Conway
    Version v1
 */
extern "C" {
  #include "osapi.h"
  #include "user_interface.h"
}

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <Hash.h>
#include <MPU9250.h>
#include <WebSocketsClient.h>
//#include <String.h>

#define LED = 2;
#define ABS_IMU_SAMPLERATE_DELAY_MS (10)
#define IMU_SAMPLERATE_DELAY_MS (1) // millis between samples 1/freq * 1000
#define ABS_IMU_OUT true
#define IMU_OUT false

// Update these with values suitable for your network.
const char *_ssid = "CPT Sensors";
const char *_password = "crossword";
const char *_server = "192.168.0.102";
/*const char *_ssid = "W1F1";
const char *_password = "B3NR1CHJ0RD4N14N";
const char *_server = "192.168.0.21";
*/
uint16_t _port = 81;
unsigned  long lastMsg = 0, lastacc = 0, start_loop = 0, end_loop = 0, lastmsg = 0, now = 0;
bool ACC_data = false, Q_data = false;
int samples = 0, IMU_sample = 0;
char buf[10][4][10];
//char IMU_buf[10][3][10];
long Stime[10], IMU_time[10];
char Quaternion[50], MPU_ACC[50];

WiFiClient espClient;
WebSocketsClient webSocket;
Adafruit_BNO055 bno = Adafruit_BNO055(); // Init Sensor
MPU9250 myIMU;

os_timer_t ABS_IMU_timer;
//os_timer_t IMU_timer;
volatile bool socket_connected = false;
void ABS_IMU();
//void IMU();

typedef enum {
        wAxis = 0,
        xAxis = 1,
        yAxis = 2,
        zAxis = 3,
} Axis;
/*
██ ███    ██ ████████ ███████ ██████  ██████  ██    ██ ██████  ████████
██ ████   ██    ██    ██      ██   ██ ██   ██ ██    ██ ██   ██    ██
██ ██ ██  ██    ██    █████   ██████  ██████  ██    ██ ██████     ██
██ ██  ██ ██    ██    ██      ██   ██ ██   ██ ██    ██ ██         ██
██ ██   ████    ██    ███████ ██   ██ ██   ██  ██████  ██         ██
*/
/**
 * [timerCallback description]
 * @param pArg [description]
 */
void ABS_IMU_timerCB(void *pArg) {
    ABS_IMU();
}
/*void IMU_timerCB(void *pArg) {
    IMU();
}*/
/*
 █████  ██████  ███████     ██ ███    ███ ██    ██
██   ██ ██   ██ ██          ██ ████  ████ ██    ██
███████ ██████  ███████     ██ ██ ████ ██ ██    ██
██   ██ ██   ██      ██     ██ ██  ██  ██ ██    ██
██   ██ ██████  ███████     ██ ██      ██  ██████
*/
/**
 * [ABS_IMU description]
 */
void ABS_IMU(void){
  Q_data = true;
  // get Quaternion
  imu::Quaternion quat = bno.getQuat();
  Stime[samples] = millis();
  float qW = quat.w();
  float qX = quat.x();
  float qY = quat.y();
  float qZ = quat.z();

  dtostrf(qW, 5, 2, buf[samples][wAxis]);
  dtostrf(qX, 5, 2, buf[samples][xAxis]);
  dtostrf(qY, 5, 2, buf[samples][yAxis]);
  dtostrf(qZ, 5, 2, buf[samples][zAxis]);
  samples++;
}
/*
██ ███    ███ ██    ██
██ ████  ████ ██    ██
██ ██ ████ ██ ██    ██
██ ██  ██  ██ ██    ██
██ ██      ██  ██████
*/
/**
 * [IMU description]
 */
/*void IMU(){
  ACC_data = true;
  IMU_time[IMU_sample] = millis();
  myIMU.readAccelData(myIMU.accelCount); // Read the x/y/z adc values
  myIMU.getAres();
  // Now we'll calculate the accleration value into actual g's
  // This depends on scale being set
  myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - accelBias[0];
  myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - accelBias[1];
  myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - accelBias[2];

  dtostrf(myIMU.ax, 5, 2, IMU_buf[IMU_sample][xAxis]);
  dtostrf(myIMU.ay, 5, 2, IMU_buf[IMU_sample][yAxis]);
  dtostrf(myIMU.az, 5, 2, IMU_buf[IMU_sample][zAxis]);
  //webSocket.sendTXT(MPU_ACC);
  //Serial.println(MPU_ACC);
}*/
/*
██     ██ ███████ ██████  ███████  ██████   ██████ ██   ██ ███████ ████████
██     ██ ██      ██   ██ ██      ██    ██ ██      ██  ██  ██         ██
██  █  ██ █████   ██████  ███████ ██    ██ ██      █████   █████      ██
██ ███ ██ ██      ██   ██      ██ ██    ██ ██      ██  ██  ██         ██
 ███ ███  ███████ ██████  ███████  ██████   ██████ ██   ██ ███████    ██
*/
/**
 * [webSocketEvent description]
 * @param type    [description]
 * @param payload [description]
 * @param length  [description]
 */
void webSocketEvent( WStype_t type, uint8_t *payload, size_t length) {

  switch (type) {
  case WStype_DISCONNECTED:
    Serial.printf("[WSc] Disconnected!\n");
    socket_connected = false;
    break;
  case WStype_CONNECTED:
    Serial.printf("[WSc] Connected to url: %s\n", payload);
    //Serial.printf("WS:   client [%i] connected from %d.%d.%d.%d url: %s\n",
    //num, ip[0], ip[1], ip[2], ip[3], payload
    //);
    // send message to server when Connected
    socket_connected = true;
    break;
  case WStype_TEXT:
    Serial.printf("[WSc] get text: %s\n", payload);
    // send message to server
    // webSocket.sendTXT("message here");
    break;
  case WStype_BIN:
    Serial.printf("[WSc] get binary length: %u\n", length);
    hexdump(payload, length);
    // send data to server
    // webSocket.sendBIN(payload, length);
    break;
  default:
    Serial.printf("WS:   unhandled event type: %i\n", type);
    break;
  }
}
/*
   ██     ██ ██ ███████ ██
   ██     ██ ██ ██      ██
   ██  █  ██ ██ █████   ██
   ██ ███ ██ ██ ██      ██
   ███   ███ ██ ██      ██
 */
/**
 * [setup_wifi description]
 */
/*void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(_ssid);
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.begin(_ssid, _password);
  // wifi_set_sleep_type(NONE_SLEEP_T);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    // ESP.restart();
    // WiFi.begin(_ssid, _password);
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  delay(100);
}*/
/*
   ██████  ███    ██  ██████   ██████  ███████ ███████
   ██   ██ ████   ██ ██    ██ ██  ████ ██      ██
   ██████  ██ ██  ██ ██    ██ ██ ██ ██ ███████ ███████
   ██   ██ ██  ██ ██ ██    ██ ████  ██      ██      ██
   ██████  ██   ████  ██████   ██████  ███████ ███████
 */
/**
 * [setup_BNO055 description]
 */
void setup_BNO055() {
  // Check if sensor started
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print(
        "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    // while (1);
  }
  delay(100);
  bno.setExtCrystalUse(true);
}
/*
   ███    ███ ██████  ██    ██  █████   ██ ███████  ██████
   ████  ████ ██   ██ ██    ██ ██   ██ ███ ██      ██  ████
   ██ ████ ██ ██████  ██    ██  ██████  ██ ███████ ██ ██ ██
   ██  ██  ██ ██      ██    ██      ██  ██      ██ ████  ██
   ██      ██ ██       ██████   █████   ██ ███████  ██████
 */
/**
 * [setup_MPU_9150 description]
 */
/*void setup_MPU_9150() {
  myIMU.initMPU9250();
  myIMU.MPU9250SelfTest(myIMU.SelfTest);
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  myIMU.initAK8963(myIMU.magCalibration);
}*/
/*
███████ ███████ ████████ ██    ██ ██████
██      ██         ██    ██    ██ ██   ██
███████ █████      ██    ██    ██ ██████
     ██ ██         ██    ██    ██ ██
███████ ███████    ██     ██████  ██
*/
/**
 * [setup description]
 */
void setup() {
  Serial.begin(460800);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println();
  Serial.println();
  system_update_cpu_freq(80);
  //os_update_cpu_frequency(80);


  /*for(uint8_t t = 4; t > 0; t--) {
      Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
      Serial.flush();
      delay(1000);
     }*/
  if (ABS_IMU_OUT){
    setup_BNO055();
    os_timer_setfn(&ABS_IMU_timer, ABS_IMU_timerCB, NULL);
    os_timer_arm(&ABS_IMU_timer, ABS_IMU_SAMPLERATE_DELAY_MS, true);
  }
  /*if (IMU_OUT){
    setup_MPU_9150();
    os_timer_setfn(&IMU_timer, IMU_timerCB, NULL);
    os_timer_arm(&IMU_timer, IMU_SAMPLERATE_DELAY_MS, true);
  }*/
  //setup_wifi();
  webSocket.begin(_server, _port);
  // webSocket.setAuthorization("user", "Password"); // HTTP Basic Authorization
  webSocket.onEvent(webSocketEvent);
  delay(500);
  Serial.println("Boot Done...");
}
/*
   ██       ██████   ██████  ██████
   ██      ██    ██ ██    ██ ██   ██
   ██      ██    ██ ██    ██ ██████
   ██      ██    ██ ██    ██ ██
   ███████  ██████   ██████  ██
 */
/**
 * [loop description]
 */
void loop() {
  //if (socket_connected){
  if (samples>=10){
      String data;
      for(int i=0;i<samples;i++) {
        data = "{\"Quaternion\":{";
        data += "\"w\":";
        data += buf[i][wAxis];
        data += ",\"x\":";
        data += buf[i][xAxis];
        //  data += xstr[i];
        data += ",\"y\":";
        data += buf[i][yAxis];
        //data += ystr[i];
        data += ",\"z\":";
        data += buf[i][zAxis];
        //data += zstr[i];
        data += "},\"Time\":";
        data += Stime[i];
        data += "}";
        webSocket.sendTXT(data);
      }
      Serial.println(data);
      samples = 0;
    //}
  }
  /*if (IMU_sample>=100){
      String data;
      for(int i=0;i<IMU_sample;i++) {
        data = "{\"IMU\":{";
        data += "\"x\":[";
        data += buf[i][xAxis];
        data += buf[i][xAxis];
        data += "],\"y\":";
        data += buf[i][yAxis];
        data += ",\"z\":";
        data += buf[i][zAxis];
        data += "},\"Time\":";
        data += Stime[i];
        data += "}";
        webSocket.sendTXT(data);
      }
      IMU_sample = 0;
    //}
  }*/
  yield();
  //delay(500);
  //Serial.print('.');
}
