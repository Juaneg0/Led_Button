#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <string.h>

#define LED_PIN 13
#define SWITCH_PIN 15

#define TIEMPO_CICLO 5000 // Tiempo del ciclo esperado para el molde en ms
#define CICLOS_FALLIDOS_PERMITIDOS 2
#define NUM_MAQUINA "Inyectora_1"

#define MSG_BUFFER_SIZE (40)

// --- Configuración WiFi  ---
const char *ssid = "Farmaplast";
const char *password = "FPfpnetwork10";
// --- Configuración MQTT ---
const char *mqtt_device = "TTGO_Juan";    // Nombre del dispositivo MQTT
const char *mqtt_server = "10.0.201.128"; // IP del broker MQTT
// --- Configuración NTP ---
const char *ntpServer1 = "time.google.com";
const char *ntpServer2 = "co.pool.ntp.org";
const char *ntpServer3 = "time.cloudflare.com";
const long gmtOffset_sec = -5 * 3600; // Colombia
const int daylightOffset_sec = 0;
// --- Topics MQTT ---
char Topic_Tiempo_Ciclo[MSG_BUFFER_SIZE];
char Topic_Tiempo_Paro[MSG_BUFFER_SIZE];
// --- Colas FreeRTOS ---
QueueHandle_t cola_sensor;       // Envía lecturas del sensor de la maquina
QueueHandle_t cola_tiempo_ciclo; // Envía tiempo de ciclo medido
QueueHandle_t cola_tiempo_paro;  // Envía tiempo de paro detectado
QueueHandle_t cola_flg_paro;     // Envía banderas con detección de paro

WiFiClient espClient;
PubSubClient client(espClient);

// --------------------- Funciones varias ---------------------
void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int ii = 0; ii < length; ii++)
  {
    Serial.print((char)payload[ii]);
  }
  Serial.println();
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqtt_device))
    {
      // Once connected, publish an announcement...
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait 1 second(s) before retrying
    }
  }
}

// --------------------- Tareas ---------------------
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
  char estado_anterior = 0;
  char estado_actual = 0;
  char flg_estado_paro = 0;
  char flg_estado_paro_anterior = 0;

  struct tm tiempo_paro_maquina;

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
        xQueueSend(cola_tiempo_ciclo, &tiempo_ciclo, portMAX_DELAY);
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
    }
    else
    {
      flg_estado_paro = 0;
    }

    if (flg_estado_paro == 1)
    {
      if (flg_estado_paro_anterior == 0)
      {
        Serial.println("Se dectecto un paro de la máquina!");
        if (!getLocalTime(&tiempo_paro_maquina))
        {
          Serial.println("Error al obtener hora para tiempo de paro.");
        }
        xQueueSend(cola_tiempo_paro, &tiempo_paro_maquina, portMAX_DELAY);
      }
      Serial.print("Tiempo sin cambio: ");
      Serial.print(tiempo_transcurrido / 1000);
      Serial.println(" segundos.");
    }

    // Actualiza el estado de paro
    flg_estado_paro_anterior = flg_estado_paro;

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
    digitalWrite(LED_PIN, flg_recibida ? HIGH : LOW);
  }
}

void conexion_mqtt(void *p)
{
  while (1)
  {
    if (!client.connected())
      reconnect();
    client.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void publish_mqtt(void *p)
{
  char msg_Tiempo_Ciclo[MSG_BUFFER_SIZE];
  char msg_Tiempo_Paro[MSG_BUFFER_SIZE];
  unsigned long tiempo_ciclo_recibido = 0;
  struct tm tiempo_paro_maquina;

  // Se crean los topics MQTT
  snprintf(Topic_Tiempo_Ciclo, MSG_BUFFER_SIZE, "%s/tiempo_ciclo", NUM_MAQUINA);
  snprintf(Topic_Tiempo_Paro, MSG_BUFFER_SIZE, "%s/tiempo_paro", NUM_MAQUINA);

  while (1)
  {
    // Si se completo un ciclo y se recibe el dato del tiempo de ciclo
    if (xQueueReceive(cola_tiempo_ciclo, &tiempo_ciclo_recibido, 0) == pdTRUE)
    {
      if (client.connected())
      {
        snprintf(msg_Tiempo_Ciclo, MSG_BUFFER_SIZE, "%d", tiempo_ciclo_recibido / 1000);
        client.publish(Topic_Tiempo_Ciclo, msg_Tiempo_Ciclo);
        Serial.print("Tiempo de ciclo Publicado: ");
        Serial.print(tiempo_ciclo_recibido / 1000);
        Serial.println(" segundos.");
      }
      else
      {
        Serial.println("No conectado a MQTT, no se pudo publicar el mensaje.");
      }
    }

    // Si se paro la maquina
    if (xQueueReceive(cola_tiempo_paro, &tiempo_paro_maquina, 0) == pdTRUE)
    {
      if (client.connected())
      {
        snprintf(msg_Tiempo_Paro, MSG_BUFFER_SIZE, "%04d-%02d-%02d %02d:%02d:%02d",
                 tiempo_paro_maquina.tm_year + 1900,
                 tiempo_paro_maquina.tm_mon + 1,
                 tiempo_paro_maquina.tm_mday,
                 tiempo_paro_maquina.tm_hour,
                 tiempo_paro_maquina.tm_min,
                 tiempo_paro_maquina.tm_sec);
        client.publish(Topic_Tiempo_Paro, msg_Tiempo_Paro);
        Serial.print("Tiempo de paro Publicado: ");
        Serial.println(msg_Tiempo_Paro);
      }
      else
      {
        Serial.println("No conectado a MQTT, no se pudo publicar el mensaje.");
      }
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);

  // --- Conexión WiFi ---
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // --- Configurar NTP ---
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2, ntpServer3);
  Serial.println("Sincronizando hora NTP...");
  struct tm timeinfo;
  if (getLocalTime(&timeinfo))
  {
    Serial.println(&timeinfo, "Hora sincronizada: %Y-%m-%d %H:%M:%S");
  }
  else
  {
    Serial.println("Error al obtener hora inicial.");
  }

  Serial.println("Inicializacion terminada.");

  // --- Crear colas ---
  cola_sensor = xQueueCreate(10, sizeof(char));
  cola_tiempo_ciclo = xQueueCreate(10, sizeof(unsigned long));
  cola_tiempo_paro = xQueueCreate(10, sizeof(struct tm));
  cola_flg_paro = xQueueCreate(10, sizeof(char));

  xTaskCreate(leer_sensor, "Lectura_Boton", configMINIMAL_STACK_SIZE * 10, NULL, 1, NULL);
  xTaskCreate(deteccion_paro, "Deteccion_Paro", configMINIMAL_STACK_SIZE * 10, NULL, 1, NULL);
  xTaskCreate(controlar_led, "Control_LED", configMINIMAL_STACK_SIZE * 10, NULL, 1, NULL);
  xTaskCreate(conexion_mqtt, "Conexion_MQTT", configMINIMAL_STACK_SIZE * 10, NULL, 3, NULL);
  xTaskCreate(publish_mqtt, "Publicar_MQTT", configMINIMAL_STACK_SIZE * 50, NULL, 2, NULL);
}

void loop()
{
  vTaskDelete(NULL);
}
