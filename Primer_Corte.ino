/**
 * @file greenhouse_monitor_rtos_debug.ino
 * @brief Monitoreo de invernadero usando FreeRTOS (ESP32) con mensajes de depuración.
 *
 * Requerimientos:
 * - Uso de tareas con RTOS (FreeRTOS integrado en ESP32).
 * - Manejo de interrupciones (botón e infrarrojo) mediante semáforos.
 * - Formateo de datos con timestamp para envío por Serial.
 * - Manejo de errores en la lectura de sensores.
 * - Inclusión de reloj de tiempo real DS3231.
 * - Gestión de potencia (placeholder para low power).
 * - Generación de alarmas visuales y sonoras.
 * - Documentación con Doxygen.
 * - (Opcional) Despliegue en LCD/OLED.
 */

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <Wire.h>
#include <RTClib.h>
#include <DHT.h>

// Definiciones de pines y parámetros
#define DHTPIN         4       ///< Pin del sensor DHT
#define LDRPIN         34      ///< Pin del sensor LDR
#define DHTTYPE        DHT11   ///< Tipo de sensor DHT
#define LED_B          5       ///< Pin del LED (alarma visual)
#define BUZZER_PIN     14       ///< Pin del buzzer (alarma sonora)
#define BUTTON_PIN     15      ///< Pin del botón (para interrupción)
#define IR_PIN         18       ///< Pin del sensor IR (para interrupción)

// Objetos y variables globales
DHT dht(DHTPIN, DHTTYPE);   ///< Objeto para el sensor DHT
RTC_DS3231 rtc;            ///< Objeto para el reloj DS3231

volatile float temperature = 0;  ///< Temperatura actual leída
volatile float humidity    = 0;  ///< Humedad actual leída
volatile int   lightValue  = 0;  ///< Valor del LDR

volatile bool alarmTriggered = false; ///< Bandera de activación de alarma

// Semáforos para interrupciones
SemaphoreHandle_t buttonSemaphore;
SemaphoreHandle_t irSemaphore;

/**
 * @brief Rutina de servicio de interrupción para el botón.
 */
void IRAM_ATTR handleButtonInterrupt() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(buttonSemaphore, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

/**
 * @brief Rutina de servicio de interrupción para el sensor IR.
 */
void IRAM_ATTR handleIRInterrupt() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(irSemaphore, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

/**
 * @brief Tarea para leer la temperatura y la humedad del sensor DHT.
 */
void TaskReadTempHum(void *pvParameters) {
  Serial.println("[TaskReadTempHum] Iniciada la tarea de lectura DHT.");
  for (;;) {
    Serial.println("[TaskReadTempHum] Leyendo DHT...");
    float temp = dht.readTemperature();
    float hum  = dht.readHumidity();

    if (isnan(temp) || isnan(hum)) {
      Serial.println("[TaskReadTempHum] Error leyendo DHT (NaN).");
    } else {
      temperature = temp;
      humidity    = hum;
      Serial.print("[TaskReadTempHum] Temp: ");
      Serial.print(temperature);
      Serial.print("°C, Hum: ");
      Serial.print(humidity);
      Serial.println("%");

      // Condición de alarma: temperatura > 24°C y humedad > 70%
      if (temperature > 24 && humidity > 70) {
        alarmTriggered = true;
        Serial.println("[TaskReadTempHum] Alarma activada (Temp/Hum).");
      } else {
        alarmTriggered = false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2500));
  }
}

/**
 * @brief Tarea para leer el sensor de luz (LDR).
 */
void TaskReadLight(void *pvParameters) {
  Serial.println("[TaskReadLight] Iniciada la tarea de lectura LDR.");
  for (;;) {
    int ldr = analogRead(LDRPIN);
    lightValue = ldr;
    Serial.print("[TaskReadLight] LDR value: ");
    Serial.println(lightValue);

    // Si el valor supera el umbral, activar la alarma
    if (ldr > 500) {
      alarmTriggered = true;
      Serial.println("[TaskReadLight] Alarma activada (Luz).");
    }
    vTaskDelay(pdMS_TO_TICKS(1600));
  }
}

/**
 * @brief Tarea para mostrar datos con timestamp por Serial.
 */
void TaskDisplay(void *pvParameters) {
  Serial.println("[TaskDisplay] Iniciada la tarea de despliegue de datos.");
  for (;;) {
    DateTime now = rtc.now();
    char buffer[80];
    snprintf(buffer, sizeof(buffer),
             "[%02d:%02d:%02d] Temp: %.2f°C, Hum: %.2f%%, Luz: %d",
             now.hour(), now.minute(), now.second(),
             temperature, humidity, lightValue);

    Serial.println(buffer);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

/**
 * @brief Tarea para alarmas (LED y buzzer).
 */
void TaskAlarm(void *pvParameters) {
  Serial.println("[TaskAlarm] Iniciada la tarea de alarma.");
  for (;;) {
    if (alarmTriggered) {
      // Alarma activa: LED y buzzer intermitentes
      Serial.println("[TaskAlarm] Alarma activa: encendiendo LED y buzzer.");
      digitalWrite(LED_B, HIGH);
      digitalWrite(BUZZER_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(700));
      digitalWrite(LED_B, LOW);
      digitalWrite(BUZZER_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(500));
    } else {
      // Sin alarma
      digitalWrite(LED_B, LOW);
      digitalWrite(BUZZER_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
}

/**
 * @brief Tarea para la gestión de potencia (placeholder).
 */
void TaskPowerManagement(void *pvParameters) {
  Serial.println("[TaskPowerManagement] Iniciada la tarea de gestion de potencia (placeholder).");
  for (;;) {
    // Aquí podrías poner el ESP32 en modo ligero o profundo de suspensión
    // según las condiciones de tu proyecto.
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

/**
 * @brief Configuración inicial del sistema.
 */
void setup() {
  Serial.begin(115200);
  Serial.println("[setup] Iniciando setup...");

  // Inicializar DHT
  dht.begin();
  Serial.println("[setup] DHT inicializado.");

  // Inicializar RTC
  Serial.println("[setup] Inicializando RTC...");
  if (!rtc.begin()) {
    Serial.println("[setup] No se pudo encontrar el RTC. Se queda en bucle infinito.");
    while (1);
  }
  Serial.println("[setup] RTC inicializado correctamente.");

  // Ajustar fecha/hora si es necesario:
  // if (!rtc.isrunning()) {
  //   rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  //   Serial.println("[setup] Ajustando fecha/hora del RTC a la compilacion actual.");
  // }

  // Configuración de pines
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_B, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(IR_PIN, INPUT_PULLUP);

  Serial.println("[setup] Pines configurados. Creando semaforos...");

  // Crear semáforos
  buttonSemaphore = xSemaphoreCreateBinary();
  irSemaphore     = xSemaphoreCreateBinary();
  Serial.println("[setup] Semaforos creados. Configurando interrupciones...");

  // Configurar interrupciones
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), handleIRInterrupt, FALLING);
  Serial.println("[setup] Interrupciones configuradas. Creando tareas...");

  // Crear tareas (ajusta tamaños de pila y prioridades según requieras)
  xTaskCreate(TaskReadTempHum, "ReadTempHum", 2048, NULL, 1, NULL);
  xTaskCreate(TaskReadLight,   "ReadLight",   2048, NULL, 1, NULL);
  xTaskCreate(TaskDisplay,     "Display",     4096, NULL, 1, NULL);
  xTaskCreate(TaskAlarm,       "Alarm",       2048, NULL, 2, NULL);
  xTaskCreate(TaskPowerManagement, "PowerMng", 2048, NULL, 1, NULL);

  Serial.println("[setup] Tareas creadas. Finalizando setup.");
}

/**
 * @brief loop() vacío en ESP32, pues todo corre en tareas FreeRTOS.
 */
void loop() {
  // Vacío: las tareas se ejecutan bajo FreeRTOS
}
