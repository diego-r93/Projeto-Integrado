#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <Update.h>
#include <WiFi.h>
#include <WiFiUDP.h>

#include <set>

#include "NTPClient.h"
#include "PHMeter.h"
#include "PIDController.h"
#include "SPIFFS.h"
#include "TDSMeter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "hydraulicPumpController.h"
#include "mongoDbAtlas.h"
#include "wifiCredentials.h"

/*
Task                Core  Prio     Descrição
----------------------------------------------------------------------------------------------------
vTaskUpdate          0     3     Atualiza as informações através de um POST no MongoDB Atlas
vTaskCheckWiFi       0     2     Verifica a conexão WiFi e tenta reconectar caso esteja deconectado
vTaskMqttReconnect   0     2     Verifica a conexão MQTT e tenta reconectar caso esteja deconectado
vTaskNTP             0     1     Atualiza o horário com base no NTP
vTaskTurnOnPump      1     3     Liga a bomba quando chegar no seu horário de acionamento
vTaskTdsSensorRead   1     1     Lê o sensor de TDS
vTaskTdsDataProcess  1     2     Processa os dados do sensor de TDS
vTaskPhSensorRead    1     2     Lê o sensor de pH
vTaskPhDataProcess   1     2     Processa os dados do sensor de pH
vTaskTdsMotorControl 1     1     Controla o motor da bomba de TDS
vTaskPhMotorControl  1     1     Controla o motor da bomba de pH

*/

// Pump Timers configuration
#define LED_BUILTIN 25
#define NUMBER_OUTPUTS 4
#define ACTIVE_PUMPS 4
#define UPDATE_DELAY 300000
#define TURN_ON_PUMP_DELAY 100

const uint8_t outputGPIOs[NUMBER_OUTPUTS] = {21, 19, 18, 5};

HydraulicPumpController myPumps[ACTIVE_PUMPS] = {
     HydraulicPumpController("code01", outputGPIOs[0], 900000),
     HydraulicPumpController("code02", outputGPIOs[1], 60000),
     HydraulicPumpController("code03", outputGPIOs[2], 60000),
     HydraulicPumpController("code04", outputGPIOs[3], 60000),
};

// TDS and PH configuration
#define TdsSensorPin 36
#define PhSensorPin 39

const float VRef = 3.3;
const int SCount = 30;
const float Temperature = 25.0;
const float CellConstant = 0.973;

// Instancia a classe TDSMeter
TDSMeter tdsMeter(TdsSensorPin, VRef, SCount, Temperature, CellConstant);

// Instancia a classe PHMeter
PHMeter phMeter(PhSensorPin, VRef, SCount);

// PID Controller configuration

// Motor A
#define motorAPin1 22
#define motorAPin2 23
#define enableAPin 27

// Motor B
#define motorBPin1 12
#define motorBPin2 33
#define enableBPin 32

// Setting PWM properties
const int freq = 30000;
const int pwmChannelA = 0;
const int pwmChannelB = 2;
const int resolution = 8;

// Variáveis para armazenar a saída do PID
volatile int dutyCycleA = 0;
volatile int dutyCycleB = 1;

Pid pidPH;
Pid pidTDS;

// NTP configuration
#define CHECK_WIFI_DELAY 100
#define NTP_DELAY 600000

WiFiUDP udp;
NTPClient ntp(udp, "a.st1.ntp.br", -3 * 3600, 3600000);

// Set your Static IP address
// IPAddress local_IP(192, 168, 1, 2);
// Set your Gateway IP address
// IPAddress gateway(192, 168, 1, 254);
// Set your SubnetMask
// IPAddress subnet(255, 255, 255, 0);

// MQTT configuration
String mqtt_server;
const uint16_t mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// WebServer configuration
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// String formatedTime;

// FreeRTOS configuration
SemaphoreHandle_t xWifiMutex;
SemaphoreHandle_t xPIDControllerMutex;

TaskHandle_t UpdateTaskHandle = NULL;
TaskHandle_t CheckWiFiTaskHandle = NULL;
TaskHandle_t MqttReconnectTaskHandle = NULL;
TaskHandle_t NTPTaskHandle = NULL;
TaskHandle_t TurnOnPumpTaskHandle = NULL;
TaskHandle_t TdsSensorReadTaskHandle = NULL;
TaskHandle_t TdsDataProcessTaskHandle = NULL;
TaskHandle_t PhSensorReadTaskHandle = NULL;
TaskHandle_t PhDataProcessTaskHandle = NULL;
TaskHandle_t TdsMotorControlTaskHandle = NULL;
TaskHandle_t PhMotorControlTaskHandle = NULL;

void vTaskUpdate(void* pvParameters);
void vTaskCheckWiFi(void* pvParametes);
void vTaskMqttReconnect(void* pvParametes);
void vTaskNTP(void* pvParameters);
void vTaskTurnOnPump(void* pvParametes);
void vTaskTdsSensorReadTask(void* pvParameter);
void vTaskTdsDataProcessTask(void* pvParameter);
void vTaskPhSensorReadTask(void* pvParameter);
void vTaskPhDataProcessTask(void* pvParameter);
void vTaskTdsMotorControlTask(void* pvParameter);
void vTaskPhMotorControlTask(void* pvParameter);

void callback(char* topic, byte* payload, unsigned int length) {
   Serial.print("Message arrived on topic: ");
   Serial.print(topic);
   Serial.print(". Message: ");
   String messageTemp;

   for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
      messageTemp += (char)payload[i];
   }
   Serial.println();

   if (String(topic) == String(WiFi.getHostname()) + "/output") {
      DynamicJsonDocument doc(128);
      deserializeJson(doc, messageTemp);

      // Check if the JSON object contains the desired GPIO number
      for (int indice = 0; indice < NUMBER_OUTPUTS; indice++) {
         String gpioString = String(outputGPIOs[indice]);

         if (doc.containsKey(gpioString)) {
            int gpioNumber = outputGPIOs[indice];
            bool state = doc[gpioString];

            Serial.print("Changing output GPIO ");
            Serial.print(gpioNumber);
            Serial.print(" to ");
            Serial.println(state ? "on" : "off");

            digitalWrite(outputGPIOs[indice], state ? HIGH : LOW);
         }
      }

   } else if (String(topic) == "getdevices") {
      if (messageTemp == "get all") {
         StaticJsonDocument<128> jsonPayload;
         jsonPayload["host"] = WiFi.getHostname();
         jsonPayload["ip"] = WiFi.localIP().toString();
         jsonPayload["mac"] = WiFi.macAddress();
         jsonPayload["rssi"] = WiFi.RSSI();

         String jsonStr;
         serializeJson(jsonPayload, jsonStr);

         client.publish("devices", jsonStr.c_str());
      }
   }
}

void controlMotor(int motorPin1, int motorPin2, int pwmChannel, int dutyCycle) {
   // Configura a direção do motor
   digitalWrite(motorPin1, LOW);
   digitalWrite(motorPin2, HIGH);

   // Configura a velocidade do motor (duty cycle)
   ledcWrite(pwmChannel, dutyCycle);
}

int convertControlOutputToDutyCycle(float controlOutput, float maxPIDOutput) {
   // Normaliza a saída do PID para o intervalo de 0 a 'maxPIDOutput'
   float normalizedOutput = fmax(0.0, fmin(controlOutput, maxPIDOutput));

   // Mapeia a saída normalizada para o intervalo de duty cycle (0 a 255)
   int dutyCycle = (int)((normalizedOutput / maxPIDOutput) * 255.0);
   dutyCycle = constrain(dutyCycle, 0, 255);

   return dutyCycle;
}

void restart() {
   yield();
   delay(1000);
   yield();
   ESP.restart();
}

void loadConfigurationCloud(const char* pumperCode, DynamicJsonDocument* jsonData) {
   DynamicJsonDocument response(MAX_SIZE_DOCUMENT);
   DynamicJsonDocument body(512);  // Pode ser pequeno pois é o que será enviado

   const size_t CAPACITY = JSON_OBJECT_SIZE(1);
   StaticJsonDocument<CAPACITY> doc;

   JsonObject object = doc.to<JsonObject>();
   object["pumperCode"] = pumperCode;

   body["dataSource"] = "Tomatoes";
   body["database"] = "tomatoes-database";
   body["collection"] = "boards";
   body["filter"] = object;

   // Serialize JSON document
   String json;
   serializeJson(body, json);

   WiFiClientSecure client;

   client.setCACert(root_ca);

   HTTPClient http;

   http.begin(client, serverName);
   http.addHeader("api-key", apiKey);
   http.addHeader("Content-Type", "application/json");
   http.addHeader("Accept", "application/json");

   int httpResponseCode = http.POST(json);

   Serial.print("HTTP Response code: ");
   Serial.println(httpResponseCode);

   String payload = http.getString();

   // Disconnect
   http.end();

   DeserializationError error = deserializeJson(response, payload);

   if (error)
      Serial.println("Failed to read document, using default configuration");

   serializeJsonPretty(response["document"], Serial);

   *jsonData = response["document"];
}

void updateConfiguration(DynamicJsonDocument inputDocument, std::set<String>* inputDriveTime, uint32_t* inputDuration) {
   JsonArray tempArray = inputDocument["driveTimes"].as<JsonArray>();

   inputDriveTime->clear();
   for (JsonVariant index : tempArray) {
      JsonObject object = index.as<JsonObject>();
      const char* tempTime = object["time"];
      bool tempState = object["state"];

      if (tempState)
         inputDriveTime->insert(tempTime);
   }

   *inputDuration = inputDocument["pulseDuration"].as<uint32_t>();
}

void initSPIFFS() {
   if (!SPIFFS.begin(true)) {
      Serial.println("An error has occurred while mounting SPIFFS");
   }
   Serial.println("SPIFFS mounted successfully");
}

void initWiFi() {
   WiFi.mode(WIFI_STA);

   // Configures static IP address
   // if (!WiFi.config(local_IP, gateway, subnet)) {
   //    Serial.println("STA Failed to configure");
   // }

   Serial.print("Connecting to WiFi ");
   Serial.print(ssid);
   Serial.print(" ...");
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) {
      Serial.print('.');
      delay(1000);
   }
   Serial.println();
   Serial.print("MAC Address:  ");
   Serial.println(WiFi.macAddress());
   Serial.print("Local IP:  ");
   Serial.println(WiFi.localIP());
   Serial.print("Hostname:  ");
   Serial.println(WiFi.getHostname());
   Serial.print("Gateway padrão da rede: ");
   Serial.println(WiFi.gatewayIP().toString());

   if (!MDNS.begin(WiFi.getHostname())) {
      Serial.println("Error setting up MDNS responder!");
      while (1) {
         delay(1000);
      }
   }
   Serial.println("mDNS responder started");
   Serial.print("mDNS Adress:  ");
   Serial.println(WiFi.getHostname());
}

void initConfiguration() {
   for (int indice = 0; indice < ACTIVE_PUMPS; indice++) {
      loadConfigurationCloud(myPumps[indice].pumperCode, myPumps[indice].getJsonDataPointer());
      updateConfiguration(myPumps[indice].getJsonData(), myPumps[indice].getDriveTimesPointer(), myPumps[indice].pulseDurationPointer);
   }
}

void initNTP() {
   ntp.begin();
   ntp.forceUpdate();
}

void initRtos() {
   xWifiMutex = xSemaphoreCreateMutex();
   xPIDControllerMutex = xSemaphoreCreateMutex();

   xTaskCreatePinnedToCore(vTaskCheckWiFi, "taskCheckWiFi", configMINIMAL_STACK_SIZE, NULL, 2, &CheckWiFiTaskHandle, PRO_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskMqttReconnect, "taskMqttReconnect", configMINIMAL_STACK_SIZE + 2048, NULL, 2, &MqttReconnectTaskHandle, PRO_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskNTP, "taskNTP", configMINIMAL_STACK_SIZE + 2048, NULL, 1, &NTPTaskHandle, PRO_CPU_NUM);

   for (int indice = 0; indice < ACTIVE_PUMPS; indice++) {
      xTaskCreatePinnedToCore(vTaskUpdate, "taskUpdate", configMINIMAL_STACK_SIZE + 8192, &myPumps[indice], 3, &UpdateTaskHandle, PRO_CPU_NUM);
      xTaskCreatePinnedToCore(vTaskTurnOnPump, "taskTurnOnPump", configMINIMAL_STACK_SIZE + 1024, &myPumps[indice], 3, &TurnOnPumpTaskHandle, APP_CPU_NUM);
   }

   xTaskCreatePinnedToCore(vTaskTdsSensorReadTask, "TDS Sensor Read Task", 4096, NULL, 1, &TdsSensorReadTaskHandle, APP_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskTdsDataProcessTask, "TDS Data Process Task", 4096, NULL, 2, &TdsDataProcessTaskHandle, APP_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskPhSensorReadTask, "pH Meter Task", 4096, NULL, 2, &PhSensorReadTaskHandle, APP_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskPhDataProcessTask, "pH Data Process Task", 4096, NULL, 2, &PhDataProcessTaskHandle, APP_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskTdsMotorControlTask, "TDS Motor Control", 4096, NULL, 1, &TdsMotorControlTaskHandle, APP_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskPhMotorControlTask, "PH Motor Control", 4096, NULL, 1, &PhMotorControlTaskHandle, APP_CPU_NUM);
}

void reconnect() {
   while (!client.connected()) {
      Serial.print("Attempting MQTT connection... ");

      if (client.connect(WiFi.getHostname(), "diego", "D1993rS*")) {
         Serial.println("connected");
         String topic = String(WiFi.getHostname()) + "/output";
         client.subscribe(topic.c_str());
         client.subscribe("getdevices");
      } else {
         Serial.println("Failed, reconnecting ... ");
         Serial.print("Client State: ");
         Serial.println(client.state());
         Serial.println("Try again in 5 seconds");
      }
   }
}

void initServer() {
   server.serveStatic("/", SPIFFS, "/");

   // DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");

   server.begin();
}

void setup() {
   Serial.begin(115200);

   initSPIFFS();
   initWiFi();
   initNTP();
   initConfiguration();
   initRtos();
   initServer();

   // Obter o gateway padrão da rede
   IPAddress gateway = WiFi.gatewayIP();
   mqtt_server = gateway.toString();

   client.setServer(mqtt_server.c_str(), mqtt_port);
   client.setCallback(callback);

   // Configuração dos pinos do motor A
   pinMode(motorAPin1, OUTPUT);
   pinMode(motorAPin2, OUTPUT);
   pinMode(enableAPin, OUTPUT);

   // Configuração dos pinos do motor B
   pinMode(motorBPin1, OUTPUT);
   pinMode(motorBPin2, OUTPUT);
   pinMode(enableBPin, OUTPUT);

   // Configuração do PWM para ambos os motores
   ledcSetup(pwmChannelA, freq, resolution);
   ledcSetup(pwmChannelB, freq, resolution);

   // Associação do canal PWM aos pinos
   ledcAttachPin(enableAPin, pwmChannelA);
   ledcAttachPin(enableBPin, pwmChannelB);

   // Inicialização dos parâmetros do PID (ajuste conforme necessário)
   pidTDS.setKp(0.1);
   pidTDS.setKi(0.2);
   pidTDS.setKd(0.01);

   pidPH.setKp(0.1);
   pidPH.setKi(0.1);
   pidPH.setKd(0.01);

   pinMode(LED_BUILTIN, OUTPUT);

   for (int indice = 0; indice < ACTIVE_PUMPS; indice++) {
      pinMode(outputGPIOs[indice], OUTPUT);
   }

   // Allow the hardware to sort itself out
   delay(1500);
}

void loop() {
   vTaskDelete(NULL);
}

void vTaskCheckWiFi(void* pvParameters) {
   while (1) {
      if (WiFi.status() != WL_CONNECTED) {
         Serial.println("Reconnecting to WiFi...");
         WiFi.disconnect();
         WiFi.reconnect();
      }

      vTaskDelay(pdMS_TO_TICKS(CHECK_WIFI_DELAY));
   }
}

void vTaskNTP(void* pvParameters) {
   while (1) {
      if (xSemaphoreTake(xWifiMutex, portMAX_DELAY)) {
         ntp.update();
         xSemaphoreGive(xWifiMutex);
      }

      vTaskDelay(pdMS_TO_TICKS(NTP_DELAY));
   }
}

void vTaskMqttReconnect(void* parameter) {
   while (true) {
      if (!client.connected()) {
         reconnect();
      }
      client.loop();
      vTaskDelay(1000);
   }
}

void vTaskUpdate(void* pvParameters) {
   HydraulicPumpController* pump = (HydraulicPumpController*)pvParameters;

   while (1) {
      if (xSemaphoreTake(xWifiMutex, portMAX_DELAY)) {
         loadConfigurationCloud(pump->pumperCode, pump->getJsonDataPointer());
         updateConfiguration(pump->getJsonData(), pump->getDriveTimesPointer(), pump->pulseDurationPointer);
         xSemaphoreGive(xWifiMutex);
      }

      vTaskDelay(pdMS_TO_TICKS(UPDATE_DELAY));
   }
}

void vTaskTurnOnPump(void* pvParameters) {
   HydraulicPumpController* pump = (HydraulicPumpController*)pvParameters;

   while (1) {
      String formatedTime = ntp.getFormattedTime();

      for (String driveTime : pump->getDriveTimes()) {
         if (formatedTime == driveTime)
            pump->startPump();
      }

      vTaskDelay(pdMS_TO_TICKS(TURN_ON_PUMP_DELAY));
   }
}

void vTaskTdsSensorReadTask(void* pvParameter) {
   while (true) {
      tdsMeter.update();
      vTaskDelay(100 / portTICK_PERIOD_MS);
   }
}

void vTaskTdsDataProcessTask(void* pvParameter) {
   while (true) {
      float tdsValue = tdsMeter.getTDSValue();
      float controlOutputTDS = pidTDS.pid_control(tdsValue, 350.0);

      printf("Control Output TDS: %.2f\n", controlOutputTDS);

      xSemaphoreTake(xPIDControllerMutex, portMAX_DELAY);
      dutyCycleA = convertControlOutputToDutyCycle(controlOutputTDS, 350.0);
      printf("Duty Cycle TDS: %d\n", dutyCycleA);
      xSemaphoreGive(xPIDControllerMutex);

      Serial.printf("TDS Value: %.2f uS/cm\n", tdsValue);

      if (xSemaphoreTake(xWifiMutex, portMAX_DELAY)) {
         String tdsTopic = String("sensors/") + String(WiFi.getHostname()) + "/tds";
         String tdsMessage = String(tdsValue, 2);
         if (client.connected()) {
            client.publish(tdsTopic.c_str(), tdsMessage.c_str());
         }
         xSemaphoreGive(xWifiMutex);
      }

      vTaskDelay(1000 / portTICK_PERIOD_MS);
   }
}

void vTaskPhSensorReadTask(void* pvParameter) {
   while (true) {
      phMeter.update();
      vTaskDelay(100 / portTICK_PERIOD_MS);
   }
}

void vTaskPhDataProcessTask(void* pvParameter) {
   while (true) {
      float phValue = phMeter.getPHValue();
      float controlOutputPH = pidPH.pid_control(phValue, 7.0);

      xSemaphoreTake(xPIDControllerMutex, portMAX_DELAY);
      dutyCycleB = convertControlOutputToDutyCycle(controlOutputPH, 7.0);
      printf("Duty Cycle PH: %d\n", dutyCycleB);
      xSemaphoreGive(xPIDControllerMutex);

      Serial.printf("pH Value: %.2f\n", phValue);

      if (xSemaphoreTake(xWifiMutex, portMAX_DELAY)) {
         String phTopic = String("sensors/") + String(WiFi.getHostname()) + "/ph";
         String phMessage = String(phValue, 2);
         if (client.connected()) {
            client.publish(phTopic.c_str(), phMessage.c_str());
         }
         xSemaphoreGive(xWifiMutex);
      }

      vTaskDelay(1000 / portTICK_PERIOD_MS);
   }
}

void vTaskTdsMotorControlTask(void* pvParameter) {
   while (true) {
      xSemaphoreTake(xPIDControllerMutex, portMAX_DELAY);
      int localDutyCycleA = dutyCycleA;
      xSemaphoreGive(xPIDControllerMutex);

      controlMotor(motorAPin1, motorAPin2, pwmChannelA, localDutyCycleA);

      vTaskDelay(100 / portTICK_PERIOD_MS);
   }
}

void vTaskPhMotorControlTask(void* pvParameter) {
   while (true) {
      xSemaphoreTake(xPIDControllerMutex, portMAX_DELAY);
      int localDutyCycleB = dutyCycleB;
      xSemaphoreGive(xPIDControllerMutex);

      controlMotor(motorBPin1, motorBPin2, pwmChannelB, localDutyCycleB);

      vTaskDelay(100 / portTICK_PERIOD_MS);
   }
}