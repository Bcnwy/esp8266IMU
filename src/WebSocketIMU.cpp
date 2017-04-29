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
//#include <stdint.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <Hash.h>
#include <MPU9250.h>
#include <WebSocketsClient.h>
#include <string.h>


#define LED = 2;
#define ABS_IMU_SAMPLERATE_DELAY_MS (100)
#define MPU_9150_delay (10) // millis between samples 1/freq * 1000
#define ABS_IMU_OUT true
#define IMU_OUT false


// Update these with values suitable for your network.
/*const char *_ssid = "CPT Sensors";
const char *_password = "crossword";
const char *_server = "192.168.0.102";
*/
const char *_ssid = "W1F1";
const char *_password = "B3NR1CHJ0RD4N14N";
const char *_server = "192.168.0.21";

uint16_t _port = 81;
unsigned  long lastMsg = 0, lastacc = 0, start_loop = 0, end_loop = 0, lastmsg = 0, now = 0;
bool ACC_data = false, Q_data = false;
int samples = 0;
String data, wstr[10], xstr, ystr, zstr;
char buf[10][4][10];
long Stime[10];

char Quaternion[50], MPU_ACC[50];

WiFiClient espClient;
WebSocketsClient webSocket;
Adafruit_BNO055 bno = Adafruit_BNO055(); // Init Sensor
//MPU9250 myIMU;

os_timer_t ABS_IMU_timer;
volatile bool socket_connected = false;
void ABS_IMU();

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
  //char wbuf[10], xbuf[10], ybuf[10], zbuf[10];

  dtostrf(qW, 5, 2, buf[samples][wAxis]);
  dtostrf(qX, 5, 2, buf[samples][xAxis]);
  dtostrf(qY, 6, 2, buf[samples][yAxis]);
  dtostrf(qZ, 6, 2, buf[samples][zAxis]);
  samples++;
  //xstr +=  (String)xbuf + ',';
  //ystr +=  (String)ybuf + ',';
  //zstr +=  (String)zbuf + ',';
  /*sprintf(Quaternion, "{\"Quaternion\":{\"w\":\"%s\",\"x\":\"%s\",\"y\":\"%"
                      "s\",\"z\":\"%s\"},\"Time\":\"%i\"}",
                      wbuf, xbuf, ybuf, zbuf, sTime);
                      */
  //Serial.println(wstr[0]);
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
/*
void IMU(){}
  ACC_data = true;
  myIMU.readAccelData(myIMU.accelCount); // Read the x/y/z adc values
  myIMU.getAres();
  // Now we'll calculate the accleration value into actual g's
  // This depends on scale being set
  myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - accelBias[0];
  myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - accelBias[1];
  myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - accelBias[2];
  char axbuf[10];
  char aybuf[10];
  char azbuf[10];
  dtostrf(myIMU.ax, 3, 4, axbuf);
  dtostrf(myIMU.ay, 3, 4, aybuf);
  dtostrf(myIMU.az, 3, 4, azbuf);
  sprintf(MPU_ACC, "{\"Accelerometer\":{\"x\":\"%s\",\"y\":\"%s\",\"z\":\"%"
                   "s\"},\"Time\":\"%i\"}",
          axbuf, aybuf, azbuf, millis());
  //data = MPU_ACC;
  //webSocket.sendTXT(MPU_ACC);
  //Serial.println(MPU_ACC);
}
*/
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
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(_ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(_ssid, _password);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
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
}
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
/*
void setup_MPU_9150() {
  myIMU.initMPU9250();
  // Initialize device for active mode read of acclerometer, gyroscope, and
  // temperature
  myIMU.MPU9250SelfTest(myIMU.SelfTest);
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  myIMU.initAK8963(myIMU.magCalibration);
}
*/
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
  //Serial.setDebugOutput(true);
  Serial.println();
  Serial.println();
  Serial.println();
  system_update_cpu_freq(80);
  //os_update_cpu_frequency(160);
  os_timer_setfn(&ABS_IMU_timer, ABS_IMU_timerCB, NULL);
  /*for(uint8_t t = 4; t > 0; t--) {
      Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
      Serial.flush();
      delay(1000);
     }*/
  if (ABS_IMU_OUT) setup_BNO055();
  //if (IMU_OUT) setup_MPU_9150();
  setup_wifi();
  webSocket.begin(_server, _port);
  // webSocket.setAuthorization("user", "Password"); // HTTP Basic Authorization
  webSocket.onEvent(webSocketEvent);
  //lastMsg = 0;
  delay(500);
  os_timer_arm(&ABS_IMU_timer, ABS_IMU_SAMPLERATE_DELAY_MS, true);
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
      for(int i=0;i<samples;i++)
      {
        data = "{\"Quaternion\":{";
        data += "\"w\":[";
        data += buf[i][wAxis];
        data += "],\"x\":[";
        data += buf[i][xAxis];
      //  data += xstr[i];
        data += "],\"y\":[";
        data += buf[i][yAxis];
        //data += ystr[i];
        data += "],\"z\":[";
        data += buf[i][zAxis];
        //data += zstr[i];
        data += "]},\"Time\":[";
        data += Stime[i];
        data += "]}";
        webSocket.sendTXT(data);
      Serial.println("sent..");
      }
      samples = 0;
    //}
  }
  //yield();
//if (!socket_connected)webSocket.begin(_server, _port);
  /*  if ((now - lastmsg) >= refresh_delay) {
      lastmsg = now;
*/
//while (1);
/*  if ((((now = millis()) - lastMsg) >= BNO055_SAMPLERATE_DELAY_MS )&& socket_connected) {
    lastMsg = now;
    ABS_IMU();
  }*/
    /*  if(((now - lastacc) >= MPU_9150_delay)) {
        lastacc = now;
        IMU();
      }*/
  //delay(500);
  //Serial.print('.');

}
