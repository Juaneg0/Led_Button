#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define LED_PIN 13
#define SWITCH_PIN 15

QueueHandle_t cola_sensor;  // Envía lecturas del giroscopio

void leer_boton(void *p){
  int estado_boton = 0;
  while (1) {
    int estado_boton = digitalRead(SWITCH_PIN);
    xQueueSend(cola_sensor, &estado_boton, portMAX_DELAY);
    vTaskDelay(200 / portTICK_PERIOD_MS); // Leer cada 200 ms
  }
}

void controlar_led(void *p){
  int estado_recibido = 0;
  while (1) {
    if (xQueueReceive(cola_sensor, &estado_recibido, portMAX_DELAY) == pdTRUE) {
      if (estado_recibido == HIGH) {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("LED ON");
      } else {
        digitalWrite(LED_PIN, LOW);
        Serial.println("LED OFF");
      }
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);

  cola_sensor = xQueueCreate(10, sizeof(int));
  
  xTaskCreate(leer_boton, "Lectura_Boton", configMINIMAL_STACK_SIZE*10, NULL, 1, NULL);
  xTaskCreate(controlar_led, "Control_LED", configMINIMAL_STACK_SIZE*10, NULL, 1, NULL);
}

void loop() {
  vTaskDelete(NULL);
}

