///////////////////////////////
//DEFINE
///////////////////////////////
#define VERSION "1.0"
#define BUTTON_PIN     21
#define LED_BUILTIN 2

#define DOUT_pin  23
#define CLK_pin  22
#define DEC_POINT  2
#define DEBOUNCE_TIME 50
tm timeinfo;
time_t now;

long unsigned lastNTPtime;
unsigned long lastEntryTime;
char time_output[30];







#define zero_factor 8389642
float offset = 0;
float calibration_factor = 206140;


String data;

//////////////////////////////
// GLOBAL VARIABLES
/////////////////////////////


// Config Debounce sw//
typedef struct {
  bool lastSteadyState = 0;      // the previous steady state from the input pin
  bool lastFlickerableState = 0;  // the previous flickerable state from the input pin
  bool currentState;                // the current reading from the input pin
  bool check;
  uint32_t lastDebounceTime = 0;  // the last time the output pin was toggled
} config_d;
config_d cfg_sw;



// Config Wifi//
typedef struct {
  char ssid[64] = "EmOne_2.4G";
  char pass[64] = "nice2meetu";
} config_wf;
config_wf cfg_wifi;


// Config Timezone///
typedef struct {
  char NTP_SERVER[64] = "ch.pool.ntp.org";
  char TZ_INFO[64] = "<+07>-7";
  
} config_t;
config_t cfg_TZ;



///////////////////////
//INCLUDE
//////////////////////

/////Servo/////
#include <ESP32Servo.h>
Servo myservo;

/////Loadcell//////
#include "src/HX711.h"
HX711 scale(DOUT_pin, CLK_pin);

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

// WIFI
#ifdef ESP32
#include <WiFi.h>
#include <WiFiMulti.h>
#endif

/////////////////////////////////////
// Setup
////////////////////////////////////

// WIFI
TimerHandle_t wifiReconnectTimer;

void wifi_setup() {
  Serial.println("WIFI: SETUP");
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(wifi_connect));
  WiFi.onEvent(wifi_event);
  wifi_connect();
}

void wifi_connect() {
  Serial.println("WIFI: Connecting...");
  WiFi.begin(cfg_wifi.ssid, cfg_wifi.pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  digitalWrite(LED_BUILTIN, 1);
}


void wifi_event(WiFiEvent_t event) {
  Serial.print("WIFI: event=");
  Serial.println(event);

  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WIFI: Connected");
      Serial.print("WIFI: IP address=");
      Serial.println(WiFi.localIP());
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WIFI: Lost connection");
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }

}
///////////////////////
///Timezone
///////////////////
TimerHandle_t TimezoneTimer;
void Timezone_setup() {
//  Serial.println("Timezone: SETUP");
  configTime(0, 0, cfg_TZ.NTP_SERVER);
  setenv("TZ", cfg_TZ.TZ_INFO, 1);
  TimezoneTimer = xTimerCreate("TimezoneTimer", pdMS_TO_TICKS(1000), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(getNTPtime));
  xTimerStart(TimezoneTimer, 0);
}

void getNTPtime() {

  uint32_t start = millis();
  do {
    time(&now);
    localtime_r(&now, &timeinfo);
    //    Serial.print(".");
    delay(10);
    //sec=10
  } while (((millis() - start) <= (1000 * 10)) && (timeinfo.tm_year < (2016 - 1900)));
  //  Serial.print("now ");  Serial.println(now);

  strftime(time_output, 30, "%a  %d-%m-%y %T", localtime(&now));
  //  Serial.println(time_output);
  //  Serial.println();
}



//////////////////////////////
//Switch
/////////////////////////////////
void Switch(bool currentState)
{
  // If the switch/button changed, due to noise or pressing:
  if (cfg_sw.currentState != cfg_sw.lastFlickerableState) {
    // reset the debouncing timer
    cfg_sw.lastDebounceTime = millis();
    // save the the last flickerable state
    cfg_sw.lastFlickerableState = cfg_sw.currentState;
  }

  if ((millis() - cfg_sw.lastDebounceTime) > DEBOUNCE_TIME) {

    // if the button state has changed:
    if (cfg_sw.lastSteadyState == HIGH && cfg_sw.currentState == LOW)
    {
      //      Serial.println("The button is pressed");

    }

    else if (cfg_sw.lastSteadyState == LOW && cfg_sw.currentState == HIGH)
    {
      //      Serial.println("The button is released");
      cfg_sw.check = 1;
      Serial.println(time_output);
      
    }

    // save the the last steady state
    cfg_sw.lastSteadyState = cfg_sw.currentState;

  }
}


//////////////////////////////
//Gripper
/////////////////////////////
void Gripper()
{
  for (int pos = 90; pos >= 0; pos -= 1) {
    myservo.write(pos);

    delay(15); // waits 15ms to reach the position
  }

  for (int pos = 0; pos <= 90; pos += 1) {
    // in steps of 1 degree
    myservo.write(pos);

    delay(15); // waits 15ms to reach the position
  }

  // rotates from 180 degrees to 0 degrees



}


////////Loadcell//
TimerHandle_t sensorTimer;
void sensor_setup() {
  sensorTimer = xTimerCreate("sensorTimer", pdMS_TO_TICKS(1000), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(sensor_update));
  xTimerStart(sensorTimer, 0);
}


void sensor_update() {
  //  Serial.print("Reading: ");
  data = String(scale.get_units() + offset, DEC_POINT);
}


void setup() {

  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  myservo.attach(15);
  myservo.write(90);
  wifi_setup();
  Timezone_setup();
  sensor_setup();
  scale.set_scale(calibration_factor);
  scale.set_offset(zero_factor);
}

void loop() {

  cfg_sw.currentState = digitalRead(BUTTON_PIN);
  Switch(cfg_sw.currentState);
  if (cfg_sw.check == 1) {
    Gripper();
    cfg_sw.check = 0;
    Serial.print(data);
    Serial.println(" kg");
    Serial.println();
  }


}
