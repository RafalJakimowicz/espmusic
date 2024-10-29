#include <Arduino.h>
#include "BluetoothA2DPSink.h"
#define PWM_PIN 2


BluetoothA2DPSink a2dp_sink;
SemaphoreHandle_t mutex;
int16_t *values;
uint32_t len;
int absmax = 1024;
void read_data_stream(const uint8_t *data, uint32_t length) {
  if(xSemaphoreTake(mutex,portMAX_DELAY)){
    len = length/2;
    values = (int16_t*) data;
    xSemaphoreGive(mutex);
  }
}

void startBTRecieve(void * tParameters){
  Serial.print("Start recieving on core: ");
  Serial.println(xPortGetCoreID());
  // output to callback and no I2S 
  a2dp_sink.set_stream_reader(read_data_stream, false);
  // connect to MyMusic with no automatic reconnect
  a2dp_sink.start("ESP_32", false);  
  for (;;){
    delay(1000);
  }
}

void CreatePWMFromData(void * tParameters){
  int16_t *valBuff;
  while(true){
    if(xSemaphoreTake(mutex, portMAX_DELAY)){
      valBuff = values;
      xSemaphoreGive(mutex);
    }
  }
  for(int i = 0; i < len; i++){
    int data = valBuff[i] + absmax;
    if(data != absmax){
      float percent = (float)data / (float)(2*absmax);
      int dutyCycle = (int)255.00*percent;
      Serial.print(valBuff[i]);
      Serial.print(",");
      Serial.println(dutyCycle);
      analogWrite(PWM_PIN, dutyCycle);
    }
    else{
      analogWrite(PWM_PIN, 0);
    }
  }
}

void setup() {
  Serial.begin(115200);
  //Create recieving task on core 0
  mutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(startBTRecieve, "recieveData", 64000, NULL, 0, NULL, 0);
  xTaskCreatePinnedToCore(CreatePWMFromData, "PWMData", 64000, NULL, 0, NULL, 1);
}


void loop() {
 //-----------
}
