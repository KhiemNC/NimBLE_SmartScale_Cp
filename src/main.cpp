#include <Arduino.h>
#include "scale.h"
#include "config.h"

void TaskLEDControl(void *pvParameters) {
  pinMode(PIN_LED, OUTPUT); // Initialize LED pin
  int ledState = 0;
  while(1) {
    
    if (ledState == 0) {
      digitalWrite(PIN_LED, HIGH); // Turn ON LED
    } else {
      digitalWrite(PIN_LED, LOW); // Turn OFF LED
    }
    ledState = 1 - ledState;
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin (115200);
  
  xTaskCreate(runScaleService, "ScaleBLEService", 10240, NULL, 1, NULL);
  xTaskCreate(TaskLEDControl, "LEDControl", 1024, NULL, 2, NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
}

