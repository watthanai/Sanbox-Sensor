
//////////////////////////
//DEFINE
////////////////////////

const char * root_ca = \
                       "-----BEGIN CERTIFICATE-----\n" \
                       "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n"\
                       "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\"\
                       "-----END CERTIFICATE-----\n";



#define VERSION "1.1"
#define BUTTON_PIN  21
#define LED_IN 2
#define LED_SW 19
#define DOUT_pin 23
#define CLK_pin 22
#define DEC_POINT  2
//////////////////////////////
// GLOBAL VARIABLES
/////////////////////////////
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}


#ifdef ESP32
#include <WiFi.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <HTTPClient.h>
#include "src/HX711.h"
Servo myservo;
HX711 scale(DOUT_pin, CLK_pin);
#endif

//////////////////////////////
//Google Sheet//
////////////////////////////
typedef struct {
  String host = "https://script.google.com/macros/s/";
  String GAS_ID = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"; //--> spreadsheet script ID
} config_google;
config_google cfg_gs;


////////////////////////
// Config Wifi//
////////////////////////
typedef struct {
  char ssid[64] = "xxxxxxxxxxxx";
  char pass[64] = "xxxxxxxxxxxxxx";
} config_wf;
config_wf cfg_wifi;
TimerHandle_t wifiReconnectTimer;

////////////////////////////
// Config Debounce sw//
///////////////////////////
typedef struct {
  bool lastSteadyState = 0;      // the previous steady state from the input pin
  bool lastFlickerableState = 0;  // the previous flickerable state from the input pin
  bool currentState = 0;              // the current reading from the input pin
  uint16_t check = 0;
  uint32_t lastDebounceTime = 0;  // the last time the output pin was toggled
  uint16_t Debouncetime = 50;  //#define DEBOUNCE_TIME 50

} config_sw;
config_sw cfg_sw;


//////////////////////
// SENSOR VALUE //
//////////////////////
typedef struct {
  float offset =0 ;
  float calibration = 206140;
  uint32_t zero_factor = 8389642;
  uint32_t WeightScale = 0;
  String DataScale;
} sensor_t;
sensor_t sensor;
TimerHandle_t sensorTimer;

////////////////////////
// Config Timezone///
///////////////////////
typedef struct {
  char NTP_SERVER[64] = "ch.pool.ntp.org";
  char TZ_INFO[64] = "<+07>-7";
  char time_output[30];
  char starttime[30];
  char endtime[30]; 
} config_t;
config_t cfg_TZ;
tm timeinfo;
time_t now;


/////////**************///////////////
// Setup
/////////*************//////////////


//******WIFI***********//
void wifi_setup() {
  Serial.println("WIFI: SETUP");
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(wifi_connect));
  wifi_connect();
}

void wifi_connect() {
  Serial.println("WIFI: Connecting...");
  WiFi.begin(cfg_wifi.ssid, cfg_wifi.pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);

    digitalWrite(LED_IN, !digitalRead(LED_IN));
  }
  digitalWrite(LED_IN, 1);
  Serial.println("WIFI: Connected");
  Serial.print("WIFI: IP address=");
  Serial.println(WiFi.localIP());
}
//*******************//



//******Deboucing Switch***********//

void Switch(bool currentState)
{
  // If the switch/button changed, due to noise or pressing:
  if (cfg_sw.currentState != cfg_sw.lastFlickerableState) {
    cfg_sw.lastDebounceTime = millis();
    cfg_sw.lastFlickerableState = cfg_sw.currentState;
  }

  if ((millis() - cfg_sw.lastDebounceTime) > cfg_sw.Debouncetime) {

    if (cfg_sw.lastSteadyState == HIGH && cfg_sw.currentState == LOW)
    {

      digitalWrite(LED_SW, 1);
      strncpy(cfg_TZ.starttime, cfg_TZ.time_output, sizeof(cfg_TZ.starttime));
      Gripper();
      strncpy(cfg_TZ.endtime, cfg_TZ.time_output, sizeof(cfg_TZ.endtime));
      sendData();

    }

    else if (cfg_sw.lastSteadyState == LOW && cfg_sw.currentState == HIGH)
    {

      digitalWrite(LED_SW, 0);

    }
    cfg_sw.lastSteadyState = cfg_sw.currentState;
  }
}

//*******************//

//****Gripper*****////
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
}


//*******************//
///*****SENSOR*********//

void sensor_setup() {

  sensorTimer = xTimerCreate("sensorTimer", pdMS_TO_TICKS(1000), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(sensor_update));
  xTimerStart(sensorTimer, 0);
}
void sensor_update() {
  sensor.WeightScale = random(0, 100);
//  sensor.DataScale = String(scale.get_units() + sensor.offset, DEC_POINT);
  
}


///*****Timezone*******//
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
    delay(10);
    //sec=10
  } while (((millis() - start) <= (1000 * 1)) && (timeinfo.tm_year < (2016 - 1900)));
  // Serial.print("now ");  Serial.println(now);
  strftime(cfg_TZ.time_output, 30, "%d-%m-%y+%H:%M:%S3", localtime(&now));

}
//*******************//

///****Google Sheet****///
void sendData() {
  String url = cfg_gs.host+ cfg_gs.GAS_ID + "/exec?starttime=" + cfg_TZ.starttime + "&endtime=" + cfg_TZ.endtime + "&weightscale=" + sensor.WeightScale;
  //  "https://script.google.com/macros/s/xxxxxxxxxxxxxxxx/exec?starttime=06-07-22+21:13:583&endtime=06-07-22+21:14:013&weightscale=11";
  
  Serial.print("requesting URL: ");
  Serial.print("POST data to spreadsheet:");
  Serial.println(url);
  HTTPClient http;
  http.begin(url, root_ca);
  int httpResponseCode = http.GET();
  if (httpResponseCode > 0) {
    Serial.print("HTTP ");
    Serial.println(httpResponseCode);
    String payload = http.getString();
    Serial.println();
    Serial.println(payload);
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
    Serial.println(":-(");
  }
}

//*******************//


void setup() {
  Serial.begin(115200);
  pinMode(LED_IN, OUTPUT);
  pinMode(LED_SW, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  myservo.attach(15);
  myservo.write(0);
  sensor_setup();
  wifi_setup();
  Timezone_setup();
  scale.set_scale(sensor.calibration);
  scale.set_offset(sensor.zero_factor);

}

void loop() {
  cfg_sw.currentState = digitalRead(BUTTON_PIN);
  Switch(cfg_sw.currentState);
}