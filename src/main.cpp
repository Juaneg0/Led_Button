#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define LED_PIN 13
#define SWITCH_PIN 15

#define TIEMPO_CICLO 5000 // Tiempo del ciclo esperado para el molde en ms
#define CICLOS_FALLIDOS_PERMITIDOS 3

QueueHandle_t cola_sensor;   // Envía lecturas del sensor de la maquina
QueueHandle_t cola_flg_paro; // Envía banderas con detección de paro

void leer_sensor(void *p)
{
  char estado_sensor = 0;
  while (1)
  {
    estado_sensor = digitalRead(SWITCH_PIN);
    xQueueSend(cola_sensor, &estado_sensor, portMAX_DELAY);
    vTaskDelay(100 / portTICK_PERIOD_MS); // Leer cada 100 ms
  }
}

void deteccion_paro(void *p)
{
  unsigned long tiempo_actual = xTaskGetTickCount() * portTICK_PERIOD_MS;
  unsigned long tiempo_ultimo_cambio = xTaskGetTickCount() * portTICK_PERIOD_MS;
  unsigned long tiempo_transcurrido = 0;
  unsigned long tiempo_ciclo = 0;
  unsigned long tiempo_ciclo_anterior = xTaskGetTickCount() * portTICK_PERIOD_MS;
  char estado_ciclo = 0;
  char estado_anterior = 0;
  char estado_actual = 0;
  char flg_estado_paro = 0;

  while (1)
  {
    // Se recive el estado del sensor y se toma el tiempo actual
    xQueueReceive(cola_sensor, &estado_actual, portMAX_DELAY);
    tiempo_actual = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (estado_actual != estado_anterior) // Se detecta un cambio en el estado del sensor
    {
      if (estado_actual == 1)
      {
        tiempo_ciclo = tiempo_actual - tiempo_ciclo_anterior;
        tiempo_ciclo_anterior = tiempo_actual;
        Serial.print("Tiempo de ciclo: ");
        Serial.print(tiempo_ciclo / 1000);
        Serial.println(" segundos.");
      }

      estado_anterior = estado_actual;
      tiempo_ultimo_cambio = xTaskGetTickCount() * portTICK_PERIOD_MS;
      flg_estado_paro = 0;
      Serial.println("Cambio detectado en el sensor.");
      Serial.print("Estado Sensor: \t");
      Serial.println(estado_actual, DEC);
    }

    // Se verifica si ha pasado el tiempo permitido sin cambios en el sensor
    tiempo_transcurrido = tiempo_actual - tiempo_ultimo_cambio;
    if (tiempo_transcurrido >= (TIEMPO_CICLO * CICLOS_FALLIDOS_PERMITIDOS))
    {
      flg_estado_paro = 1;
      Serial.println("Se dectecto un paro de la máquina!");
      Serial.print("Tiempo sin cambio: ");
      Serial.print(tiempo_transcurrido / 1000);
      Serial.println(" segundos.");
    }

    xQueueSend(cola_flg_paro, &flg_estado_paro, portMAX_DELAY);
    vTaskDelay(100 / portTICK_PERIOD_MS); // Comprobar cada 100 ms
  }
}

void controlar_led(void *p)
{
  char flg_recibida = 0;
  digitalWrite(LED_PIN, LOW); // Iniciar con LED apagado
  while (1)
  {
    xQueueReceive(cola_flg_paro, &flg_recibida, portMAX_DELAY);
    if (flg_recibida == 1)
    {
      digitalWrite(LED_PIN, HIGH); // Encender LED si hay paro
    }
    else
    {
      digitalWrite(LED_PIN, LOW); // Apagar LED si no hay paro
    }
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);

  cola_sensor = xQueueCreate(10, sizeof(char));
  cola_flg_paro = xQueueCreate(10, sizeof(char));

  xTaskCreate(leer_sensor, "Lectura_Boton", configMINIMAL_STACK_SIZE * 10, NULL, 1, NULL);
  xTaskCreate(deteccion_paro, "Deteccion_Paro", configMINIMAL_STACK_SIZE * 10, NULL, 1, NULL);
  xTaskCreate(controlar_led, "Control_LED", configMINIMAL_STACK_SIZE * 10, NULL, 1, NULL);
}

void loop()
{
  vTaskDelete(NULL);
}
