const int DOUT = 23;
const int PD_SCK = 22;
long dataArray ;
double output;
TimerHandle_t sensorTimer;



void sensor_setup() {

  sensorTimer = xTimerCreate("sensorTimer", pdMS_TO_TICKS(400), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(sensor_update));
  xTimerStart(sensorTimer, 0);
}


//  switch (gain) {
//    case 128:   // channel A, gain factor 128
//      GAIN = 1;
//      break;
//    case 64:    // channel A, gain factor 64
//      GAIN = 3;
//      break;
//    case 32:    // channel B, gain factor 32
//      GAIN = 2;
//      break;
//  }


bool is_ready() {
  return digitalRead(DOUT) == LOW;
}


void sensor_update() {

  while (!is_ready());
  byte data[3];

  // pulse the clock pin 24 times to read the data
  for (byte j = 3; j--;) {
    for (char i = 8; i--;) {
      digitalWrite(PD_SCK, HIGH);
      bitWrite(data[j], i, digitalRead(DOUT));
      digitalWrite(PD_SCK, LOW);
    }

  }

  // set the channel and the gain factor for the next reading using the clock pin
  for (int i = 0; i < 1; i++) {
    digitalWrite(PD_SCK, HIGH);
    digitalWrite(PD_SCK, LOW);
  }

  data[2] ^= 0x80;
 
//  Serial.println(((uint32_t) data[2] << 16) | ((uint32_t) data[1] << 8) | (uint32_t) data[0]);
  dataArray = ((uint32_t) data[2] << 16) | ((uint32_t) data[1] << 8) | (uint32_t) data[0];
//  Serial.println(dataArray);
  output=(dataArray-8445000)*5/(9500000-8445000);
  Serial.println(output);
         
}





void setup() {
  Serial.begin(115200);
  pinMode(PD_SCK, OUTPUT);
  pinMode(DOUT, INPUT);
  sensor_setup();
}

void loop() {

}
