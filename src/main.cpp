#include <Arduino.h>
#include "BluetoothA2DPSink.h"
#define PWM_PIN 2


BluetoothA2DPSink a2dp_sink;
TaskHandle_t taskOnCore0;
int16_t *values;
uint32_t len;
int absmax = 1024;
void read_data_stream(const uint8_t *data, uint32_t length) {
    len = length/2;
    values = (int16_t*) data;
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

void setup() {
  Serial.begin(115200);
  //Create recieving task on core 0
  xTaskCreatePinnedToCore(startBTRecieve, "recieveData", 20000, NULL, 0, &taskOnCore0, 0);
}


void loop() {
  int16_t *valBuff = values;
  for(int i = 0; i < len; i++){
    int data = valBuff[i] + 1024;
    if(data != 1024){
      float percent = (float)data / 2048.0000;
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
