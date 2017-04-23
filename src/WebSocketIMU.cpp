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
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <Hash.h>
#include <MPU9250.h>
#include <WebSocketsClient.h>

#define LED = 2;
#define BNO055_SAMPLERATE_DELAY_MS (10)
#define MPU_9150_delay (1) // millis between samples 1/freq * 1000
#define ABS_IMU_OUT false
#define IMU_OUT true
#define refresh_delay (10)

// Update these with values suitable for your network.

const char *_ssid = "W1F1";                 //"CPT Sensors";
const char *_password = "B3NR1CHJ0RD4N14N"; //"crossword";
const char *_server = "192.168.0.21";
uint16_t _port = 81;
long lastMsg = 0, lastacc = 0, start_loop = 0, end_loop = 0, lastmsg = 0;
bool ACC_data = false, Q_data = false;
int sample = 0;
String data;
char Quaternion[50], MPU_ACC[50];

WiFiClient espClient;
WebSocketsClient webSocket;
Adafruit_BNO055 bno = Adafruit_BNO055(); // Init Sensor
MPU9250 myIMU;

// static os_timer_t myTimer;

bool socket_connected = false;
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
void timerCallback(void *pArg) {}

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
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {

  switch (type) {
  case WStype_DISCONNECTED:
    Serial.printf("[WSc] Disconnected!\n");
    // socket_connected = false;
    break;
  case WStype_CONNECTED:
    Serial.printf("[WSc] Connected to url: %s\n", payload);
    // send message to server when Connected
    // webSocket.sendTXT("Connected");
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
  }
}
/*
   ██     ██ ██ ███████ ██
   ██     ██ ██ ██      ██
   ██  █  ██ ██ █████   ██
   ██ ███ ██ ██ ██      ██
   ███ ███  ██ ██      ██
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
    delay(1000);
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
void setup_MPU_9150() {
  myIMU.initMPU9250();
  // Initialize device for active mode read of acclerometer, gyroscope, and
  // temperature
  //  myIMU.MPU9250SelfTest(myIMU.SelfTest);
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  // myIMU.initAK8963(myIMU.magCalibration);
}
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
  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  Serial.println();
  Serial.println();
  Serial.println();
  system_update_cpu_freq(160);
  // os_timer_setfn(&myTimer, timerCallback, NULL);
  /*for(uint8_t t = 4; t > 0; t--) {
      Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
      Serial.flush();
      delay(1000);
     }*/
  setup_BNO055();
  setup_MPU_9150();
  setup_wifi();
  webSocket.begin(_server, _port);
  // webSocket.setAuthorization("user", "Password"); // HTTP Basic Authorization
  webSocket.onEvent(webSocketEvent);
  // os_timer_arm(&myTimer, 10, true);
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
  //  webSocket.loop();
  // create varibles to strore sensor data
  if (socket_connected) {
    long now = millis();
    if ((now - lastmsg) >= refresh_delay && sample >= 10) {
      Serial.println(now - lastmsg);
      lastmsg = now;
      if (ACC_data) {
        for (int i = 0; i < 10; i++) {
          webSocket.sendTXT(data);
          sample = 0;
        }
      }
      if (Q_data)
        webSocket.sendTXT(Quaternion);
      Q_data = false;
      ACC_data = false;
    }
    if ((now - lastacc) >= MPU_9150_delay && IMU_OUT) {
      lastacc = now;
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
              axbuf, aybuf, azbuf, now);
      data = MPU_ACC;
      // Serial.println(MPU_ACC);
    }
    if ((now - lastMsg) >= BNO055_SAMPLERATE_DELAY_MS && ABS_IMU_OUT) {
      lastMsg = now;
      Q_data = true;
      sensors_event_t event;
      bno.getEvent(&event);
      // imu::Vector<3> lineacc =
      // bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

      /* //get euler data
         float X = event.orientation.x;
         float Y = event.orientation.y;
         float Z = event.orientation.z;

         char xbuf[10];
         char ybuf[10];
         char zbuf[10];
         dtostrf( X,3,4,xbuf);
         dtostrf( Y,3,4,ybuf);
         dtostrf( Z,3,4,zbuf);
         sprintf(data, "{\"orientation\":{\"euler\":[%s,%s,%s]}}", xbuf, ybuf,
         zbuf );
       */

      // get Quaternion
      imu::Quaternion quat = bno.getQuat();
      float qW = quat.w();
      float qX = quat.x();
      float qY = quat.y();
      float qZ = quat.z();

      char wbuf[10];
      char xbuf[10];
      char ybuf[10];
      char zbuf[10];
      dtostrf(qW, 3, 4, wbuf);
      dtostrf(qX, 3, 4, xbuf);
      dtostrf(qY, 3, 4, ybuf);
      dtostrf(qZ, 3, 4, zbuf);
      sprintf(Quaternion, "{\"Quaternion\",{\"w\":\"%s\",\"x\":\"%s\",\"y\":\"%"
                          "s\",\"z\":\"%s\"},\"Time\":\"%i\"}",
              wbuf, xbuf, ybuf, zbuf, now);
      // Serial.println(Quaternion);
    }
  }
}
