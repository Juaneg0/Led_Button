#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <WiFi.h>
#include <PubSubClient.h>

#define LED_PIN 13
#define SWITCH_PIN 15

#define TIEMPO_CICLO 5000 // Tiempo del ciclo esperado para el molde en ms
#define CICLOS_FALLIDOS_PERMITIDOS 3

// --- Configuración WiFi  ---
const char *ssid = "Farmaplast";
const char *password = "FPfpnetwork10";
// --- Configuración MQTT ---
const char *mqtt_device = "TTGO_Juanes";
const char *mqtt_server = "10.0.201.128";
// --- Configuración NTP ---
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -5 * 3600; // Colombia
const int daylightOffset_sec = 0;

QueueHandle_t cola_sensor;   // Envía lecturas del sensor de la maquina
QueueHandle_t cola_flg_paro; // Envía banderas con detección de paro

WiFiClient espClient;
PubSubClient client(espClient);

#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

float ficticious_Value = 20.0;

// --------------------- Funciones varias ---------------------
void imprimirHoraActual(const char *mensaje)
{
  struct tm timeinfo;
  if (getLocalTime(&timeinfo))
  {
    Serial.printf("[%04d-%02d-%02d %02d:%02d:%02d] %s\n",
                  timeinfo.tm_year + 1900,
                  timeinfo.tm_mon + 1,
                  timeinfo.tm_mday,
                  timeinfo.tm_hour,
                  timeinfo.tm_min,
                  timeinfo.tm_sec,
                  mensaje);
  }
  else
  {
    Serial.println("Error obteniendo hora NTP");
  }
}

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
  Serial.println("\nWiFi conectado ✅");
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
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
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait 5 seconds before retrying
    }
  }
}

// --------------------- Tareas ---------------------
void conexion_mqtt(void *p)
{
  while (1)
  {
    if (!client.connected())
      reconnect();
    client.publish("outTopic", "hello world");
    client.loop();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

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
      imprimirHoraActual("Hora de detección de paro:");
    }

    if (flg_estado_paro == 1)
    {
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
    digitalWrite(LED_PIN, flg_recibida ? HIGH : LOW);
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
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
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

  cola_sensor = xQueueCreate(10, sizeof(char));
  cola_flg_paro = xQueueCreate(10, sizeof(char));

  xTaskCreate(conexion_mqtt, "Conexion_MQTT", configMINIMAL_STACK_SIZE * 10, NULL, 1, NULL);
  xTaskCreate(leer_sensor, "Lectura_Boton", configMINIMAL_STACK_SIZE * 10, NULL, 1, NULL);
  xTaskCreate(deteccion_paro, "Deteccion_Paro", configMINIMAL_STACK_SIZE * 10, NULL, 1, NULL);
  xTaskCreate(controlar_led, "Control_LED", configMINIMAL_STACK_SIZE * 10, NULL, 1, NULL);
}

void loop()
{
  vTaskDelete(NULL);
}
