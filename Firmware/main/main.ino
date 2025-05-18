#include <SPI.h>
#include <math.h>
#include <WiFi.h>

#include <WebSocketsClient.h>
#include <MQTTPubSubClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

const int addr_energy = 0;  // Dirección en EEPROM donde guardar el valor

#define LED_PIN 17

#define SELPIN 5     // Pin CS del MCP3208
#define MOSI_PIN 23  // MOSI
#define MISO_PIN 19  // MISO
#define SCK_PIN 18   // Reloj SPI

SPIClass spi(VSPI);

const int SAMPLE_RATE = 0.2;           // Tiempo entre muestras (ms)
const int BUFFER_SIZE = 2000;          // Número de muestras
const double VREF = 5.0;               // Voltaje de referencia
const double ADC_RESOLUTION = 4095.0;  // 12 bits

// Ganancias aplicadas a los valores RMS
const double GAIN_VOLTAGE = 137.0323;
double GAIN_CURRENT = 1;
// Ganancias de corriente
const double GAIN_CURRENT_LOW = 1;   // Ganancia cuando los pines están en HIGH
const double GAIN_CURRENT_HIGH = 1;  // Ganancia cuando los pines están en LOW

// Pines de control de ganancia
const int GAIN_PIN_1 = 2;
const int GAIN_PIN_2 = 15;

double buffer_ch7[BUFFER_SIZE];  // Voltaje
double buffer_ch4[BUFFER_SIZE];  // Corriente

unsigned long lastPrintTime = 0;
unsigned long previousMillis = 0;
float energy_active = 0;  // Energía activa acumulada en Wh
float energy_stored = 0;

double rms_ch7 = 0;
double rms_ch4 = 0;


// Configuración WiFi
const char* ssid = "LUIS QUICENO";
const char* pass = "10067940814";

// Configuración EMQX
const char* mqtt_server = "52.67.106.144";
const int mqtt_port = 8083;
const char* mqtt_path = "/mqtt";

WebSocketsClient client;
MQTTPubSubClient mqtt;


void connect() {
connect_to_wifi:
  WiFi.disconnect();
  WiFi.begin(ssid, pass);
  Serial.print("Conectando a WiFi...");
  if (WiFi.status() != WL_CONNECTED) {

    Serial.print(".");
    delay(2000);
  }
  Serial.println(" Conectado!");

connect_to_host:
  Serial.println("Conectando al broker EMQX...");
  client.disconnect();
  client.begin(mqtt_server, mqtt_port, mqtt_path, "mqtt");
  client.setReconnectInterval(2000);

  Serial.print("Estableciendo conexión MQTT...");
  while (!mqtt.connect("PS0001")) {  // ID único para tu dispositivo
    Serial.print(".");
    delay(1000);
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi desconectado");
      goto connect_to_wifi;
    }
    if (!client.isConnected()) {
      Serial.println("WebSocket desconectado");
      goto connect_to_host;
    }
  }
  Serial.println(" Conectado a MQTT!");
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
}

String crearJSON(float Vrms, float Irms, float P, float fp, float E, float Q) {
  // Crear objeto JSON con los datos de medición
  StaticJsonDocument<256> doc;

  doc["dispositivo_id"] = 6;      // ID fijo o configurable
  doc["voltaje"] = Vrms;          // Valor simulado
  doc["corriente"] = Irms;        // Valor simulado
  doc["potencia_activa"] = P;     // Valor simulado
  doc["potencia_reactiva"] = Q;   // Valor simulado
  doc["factor_potencia"] = fp;    // Valor simulado
  doc["consumo_energetico"] = E;  // Valor simulado

  String output;
  serializeJson(doc, output);
  return output;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  WiFi.begin(ssid, pass);
  Serial.println("Conectando a Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando...");
  }
  Serial.println("Conexión Wi-Fi establecida");
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);

  // Inicializar cliente MQTT
  mqtt.begin(client);

  // Establecer conexión
  connect();

  // Suscribirse al topic "variables" para recibir posibles comandos
  mqtt.subscribe("variables", [](const String& payload, const size_t size) {
    Serial.print("Mensaje recibido en variables: ");
    Serial.println(payload);

    // Aquí puedes añadir lógica para procesar comandos recibidos
  });

  spi.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SELPIN);

  pinMode(SELPIN, OUTPUT);
  digitalWrite(SELPIN, HIGH);

  // Iniciar pines de control de ganancia en LOW
  pinMode(GAIN_PIN_1, OUTPUT);
  pinMode(GAIN_PIN_2, OUTPUT);
  digitalWrite(GAIN_PIN_1, LOW);
  digitalWrite(GAIN_PIN_2, LOW);

  EEPROM.begin(512);
}

void loop() {
  // Mantener la conexión activa

  if (!mqtt.isConnected()) {
    connect();
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Conexión Wi-Fi perdida. Reconectando...");
    WiFi.disconnect();
    WiFi.begin(ssid, pass);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
      delay(1000);
      Serial.println("Intentando reconectar...");
      attempts++;
    }
  }

  unsigned long currentMillis = millis();
  // Solo ejecutar la medición y la impresión si ha pasado el intervalo definido
  if (currentMillis - lastPrintTime >= 60000) {  // 1 minuto
    lastPrintTime = currentMillis;
    mqtt.update();

    // Calcular el tiempo transcurrido en horas
    double elapsedTime = (currentMillis - previousMillis) / 3600000.0;  // Convierte ms a horas
    previousMillis = currentMillis;

    // Adquirir datos
    for (int i = 0; i < BUFFER_SIZE; i++) {
      buffer_ch7[i] = adc_to_voltage(read_adc(7));  // Voltaje
      buffer_ch4[i] = adc_to_voltage(read_adc(4));  // Corriente
    }

    // Calcular offset DC (promedio)
    double offset_ch7 = calculate_dc_offset(buffer_ch7, BUFFER_SIZE);
    double offset_ch4 = calculate_dc_offset(buffer_ch4, BUFFER_SIZE);


    // Calcular RMS eliminando el offset
    rms_ch7 = calculate_rms(buffer_ch7, BUFFER_SIZE, offset_ch7);
    rms_ch4 = calculate_rms(buffer_ch4, BUFFER_SIZE, offset_ch4);

    Serial.println(rms_ch4);
    if (rms_ch4 < 0.180) {
      Serial.println("Ganancia RG Activada");
      digitalWrite(GAIN_PIN_1, HIGH);
      digitalWrite(GAIN_PIN_2, HIGH);
      delay(100);

      // Adquirir datos
      for (int i = 0; i < BUFFER_SIZE; i++) {
        buffer_ch7[i] = adc_to_voltage(read_adc(7));  // Voltaje
        buffer_ch4[i] = adc_to_voltage(read_adc(4));  // Corriente
      }

      // Calcular offset DC (promedio)
      double offset_ch7 = calculate_dc_offset(buffer_ch7, BUFFER_SIZE);
      double offset_ch4 = calculate_dc_offset(buffer_ch4, BUFFER_SIZE);

      rms_ch7 = calculate_rms(buffer_ch7, BUFFER_SIZE, offset_ch7);
      rms_ch4 = calculate_rms(buffer_ch4, BUFFER_SIZE, offset_ch4);

      GAIN_CURRENT = 5.07;
      digitalWrite(GAIN_PIN_1, LOW);
      digitalWrite(GAIN_PIN_2, LOW);
      delay(100);
    } else {
      GAIN_CURRENT = 37.9431;
    }

    Serial.println(GAIN_CURRENT);
    // Calcular potencia activa
    double active_power = calculate_active_power(buffer_ch7, buffer_ch4, BUFFER_SIZE, offset_ch7, offset_ch4);

    // Calcular potencia aparente
    double apparent_power = rms_ch7 * rms_ch4;

    // Calcular factor de potencia
    double power_factor = active_power / apparent_power;
    active_power = apparent_power * power_factor;

    // Aplicar las ganancias después del RMS
    active_power *= (GAIN_VOLTAGE * GAIN_CURRENT);  // Aplicamos la ganancia final
    rms_ch7 *= GAIN_VOLTAGE;
    rms_ch4 *= GAIN_CURRENT;

    // Calcular energía activa en Wh
    energy_active += (active_power * elapsedTime) / 1000;

    // Mostrar resultados con formato más claro
    Serial.println("===== MEDICIÓN =====");
    Serial.printf("Voltaje RMS: %.2f V\n", rms_ch7);
    Serial.printf("Corriente RMS: %.3f A\n", rms_ch4);
    Serial.printf("Potencia Activa: %.2f W\n", active_power);
    Serial.printf("Factor de Potencia: %.2f\n", power_factor);
    Serial.printf("Energía Activa: %.5f kWh\n", energy_active);
    Serial.println("====================\n");
    digitalWrite(GAIN_PIN_1, LOW);
    digitalWrite(GAIN_PIN_2, LOW);

    EEPROM.get(addr_energy, energy_stored);
    float total_energy = energy_active + (energy_stored > 0 ? energy_stored : 0);

    String jsonData = crearJSON(rms_ch7, rms_ch4, active_power, power_factor, energy_active, 0);

    if (mqtt.isConnected()) {
      mqtt.publish("variables", jsonData));
      EEPROM.put(addr_energy, energy_active);
      EEPROM.commit();  // Necesario en ESP (como ESP32 o ESP8266)
      energy_active = 0;
    }
    else{
      EEPROM.put(addr_energy, energy_active);
      EEPROM.commit();
    }

    Serial.print("Datos publicados: ");
    Serial.println(jsonData);
  }
}

int read_adc(int channel) {
  if (channel < 0 || channel > 7) return -1;

  digitalWrite(SELPIN, LOW);

  uint8_t command = 0b00000110 | (channel >> 2);
  uint8_t msb = (channel << 6);

  spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  spi.transfer(command);
  int highByte = spi.transfer(msb) & 0x0F;
  int lowByte = spi.transfer(0x00);
  spi.endTransaction();

  digitalWrite(SELPIN, HIGH);

  return (highByte << 8) | lowByte;
}

// Convierte el valor del ADC a voltaje sin aplicar la ganancia aún
double adc_to_voltage(int adc_value) {
  return (adc_value / ADC_RESOLUTION) * VREF;
}

// Calcula el valor promedio (offset DC) del buffer
double calculate_dc_offset(double* buffer, int size) {
  double sum = 0;
  for (int i = 0; i < size; i++) {
    sum += buffer[i];
  }
  return sum / size;
}

// Calcula el RMS eliminando el offset DC
double calculate_rms(double* buffer, int size, double offset) {
  double sum = 0;
  for (int i = 0; i < size; i++) {
    double adjusted = buffer[i] - offset;
    sum += pow(adjusted, 2);
  }
  return sqrt(sum / size);
}

// Calcula la potencia activa usando el producto de muestras
double calculate_active_power(double* voltage, double* current, int size, double offset_v, double offset_c) {
  double sum = 0;
  for (int i = 0; i < size; i++) {
    double v = voltage[i] - offset_v;
    double c = current[i] - offset_c;
    sum += v * c;
  }
  return sum / size;
}


