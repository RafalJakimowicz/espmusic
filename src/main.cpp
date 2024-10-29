#include <Arduino.h>
#include "BluetoothA2DPSink.h"
#define PWM_PIN 2


BluetoothA2DPSink a2dp_sink;
SemaphoreHandle_t mutex;
TaskHandle_t task0;
//TaskHandle_t task1;
int16_t *values;
uint32_t len;
int absmax = 32768;
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

void CreatePWMFromData(){
  int16_t *valBuff = values;
  if(xSemaphoreTake(mutex, portMAX_DELAY)){
    valBuff = values;
    xSemaphoreGive(mutex);
  }
  for(int i = 0; i < len; i++){
    int data = valBuff[i] + absmax;
    if(data != absmax){
      double percent = (double)data / (double)(2*absmax);
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
  xTaskCreatePinnedToCore(startBTRecieve, "recieveData", 24000, NULL, 0, &task0, 0);
  //xTaskCreatePinnedToCore(CreatePWMFromData, "PWMData", 64000, NULL, 1, &task1, 1);
}


void loop() {
 CreatePWMFromData();
}
