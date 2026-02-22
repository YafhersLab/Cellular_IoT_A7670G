//============================================================
// LIBRERÍAS
//============================================================

// Librerias A7670G
#define TINY_GSM_MODEM_A7670
#include <TinyGsmClient.h>
#include <PubSubClient.h>

// Librerias Sensores
#include <DHT.h>
#include <Wire.h>
#include <RTClib.h>

//===================== DEFINICIONES =========================//

// Pines MODEM
#define MODEM_RESET_PIN   5
#define MODEM_PWKEY       4
#define MODEM_POWER_ON    12
#define MODEM_TX          26
#define MODEM_RX          27
#define MODEM_BAUD        115200
#define SerialGsm         Serial1

// Pines sensores
#define DHT_PIN           25
#define DHT_TYPE          DHT11
#define HUM_PIN           33
#define PIN_BATTERY       35

// Definiciones I2C
#define I2C_SCL           22
#define I2C_SDA           21

// Definiciones Led RGB 
#define RGB_RED 19
#define RGB_GREEN 18 
#define RGB_BLUE 5 

//===================== VARIABLES ===========================//

// Datos APN de Bitel
const char apn[]      = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Parametros del broker MQTT
const char* broker = "broker.emqx.io";
const int   port   = 1883;
const char* topic_pub = "Julian/SM/sensores/pub";
const char* topic_sub = "Julian/SM/sensores/sub";

// Variables de sensores
float TempAmb = 0.0;
float HumAmb = 0.0;
float HumSuelo = 0.0;
float Battery1 = 0.0;

//======================= OBJETOS ===========================//

// Objetos TinyGsm
TinyGsm modem(SerialGsm);
TinyGsmClient gsmClient(modem);

// Objeto cliente MQTT
PubSubClient mqtt(gsmClient);

// Objetos sensores
DHT dht(DHT_PIN, DHT_TYPE);
RTC_DS3231 rtc;

//====================== PROTOTIPOS =========================//

static void MQTT_Callback(char* topic, byte* payload, unsigned int length);
static bool MQTT_Connect(void);

//===================== MAIN CODE ===========================//

// Void Setup
void setup()
{
  // Inicialización serial
  Serial.begin(115200);
  delay(100);

  // Alimentar modem
  pinMode(MODEM_POWER_ON, OUTPUT);  
  digitalWrite(MODEM_POWER_ON, 1);

  // Reiniciar modem
  pinMode(MODEM_RESET_PIN, OUTPUT); 
  digitalWrite(MODEM_RESET_PIN, 0); delay(100);
  digitalWrite(MODEM_RESET_PIN, 1); delay(1000);
  digitalWrite(MODEM_RESET_PIN, 0);

  // Pulso de reinicio en PWRKEY
  pinMode(MODEM_PWKEY, OUTPUT);
  digitalWrite(MODEM_PWKEY, 0); delay(100);
  digitalWrite(MODEM_PWKEY, 1); delay(1000);
  digitalWrite(MODEM_PWKEY, 0);

  // Inicializar modem
  Serial.println("Inicializando modem ... 👀");
  SerialGsm.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);
  
  // Obtener información del módem
  String name = modem.getModemName(); delay(500);
  String modemInfo = modem.getModemInfo();

  // Imprimir información del modem
  Serial.println("================================");
  Serial.println("Información básica del módem 🌟");
  Serial.print("   Name: "); Serial.println(name);
  Serial.print("   Info: "); Serial.println(modemInfo);
  Serial.println("================================");
  
  // Conectarse a la red
  Serial.println("Esperando a la red...");
  if (!modem.waitForNetwork())
  {
    Serial.println("Error: no se pudo conectar a la red ❌");
    delay(10000);
    return;
  }

  // Verificar si esta conectado a la red
  if (modem.isNetworkConnected())
  {
    Serial.println("Red disponible, conectado correctamente ✔");
  }

  // Obtener mas informacion del modem
  String ccid = modem.getSimCCID();           delay(500);
  String imei = modem.getIMEI();              delay(500);
  String operatorName = modem.getOperator();  delay(500);
  int csq = modem.getSignalQuality();         delay(500);

  // Imprimir información del modem
  Serial.println("Información del módem: 📡");
  Serial.print("   CCID (SIM): "); Serial.println(ccid);
  Serial.print("   IMEI: "); Serial.println(imei);
  Serial.print("   Operador: "); Serial.println(operatorName);
  Serial.print("   Calidad de señal (0-31): "); Serial.println(csq);
  Serial.println("================================");

  // Realizar conexión GPRS
  Serial.print("Esperando conexión al APN (");
  Serial.print(apn);
  Serial.println(")...");
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) 
  {
    Serial.println("Error: no se pudo conectar al APN ❌");
  }
  delay(500);

  // Verificar si esta conectado al APN
  if (modem.isGprsConnected()) 
  {
    Serial.println("APN disponible, conectado correctamente ✔");
  }

  // Obtener IP local
  IPAddress local = modem.localIP();
  Serial.print("  IP obtenida: ");
  Serial.println(local);
  Serial.println("================================");
  delay(500);

  // Inicialización de pines RGB 
  pinMode(RGB_RED, OUTPUT); 
  pinMode(RGB_GREEN, OUTPUT); 
  pinMode(RGB_BLUE, OUTPUT); 

  // Inicialización DHT 
  dht.begin(DHT_PIN); 

  // Inicialización sensor capacitivo
  pinMode(HUM_PIN, INPUT); 
  analogReadResolution(10); 

  // Inicialización I2C 
  Wire.begin(I2C_SDA, I2C_SCL);

  // Inicialización RTC DS3231 
  if (!rtc.begin()) 
  { 
    Serial.println("❌ No se encontró el RTC"); 
  } 

  // Configuración conexión MQTT
  mqtt.setServer(broker, port);
  mqtt.setCallback(MQTT_Callback);

  // Realizar conexión MQTT
  if (!MQTT_Connect()) 
  {
    Serial.println("No se pudo conectar al broker MQTT 😢");
  }
}

// Void Loop
void loop()
{
  // Reconexión MQTT en caso de desconexión
  if (!mqtt.connected()) 
  {
    MQTT_Connect();
  }

  // MQTT Loop
  mqtt.loop();

  // Publicar datos al TOPIC
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 5000)
  {
    lastSend = millis();

    // Lectura del DHT11
    TempAmb = dht.readTemperature();
    HumAmb  = dht.readHumidity();

    // Lectura de la batería
    float voltage = analogRead(PIN_BATTERY) / 100.0;
    Battery1 = (voltage - 3.0) * (100.0 / (4.2 - 3.0)); 
    Battery1 = constrain(Battery1, 0, 100);

    // Lectura del sensor capactivo de humedad
    HumSuelo = map(analogRead(HUM_PIN), 100, 820, 100, 0);

    // Crear JSON con valores de los sensores
    char json[200];
    snprintf(json, sizeof(json), "{\"TempAmb\":\"%.2f\",\"HumAmb\":\"%.2f\",\"HumSuelo\":\"%.2f\",\"Battery1\":\"%.2f\"}", TempAmb, HumAmb, HumSuelo, Battery1);

    // Enviar JSON por MQTT
    mqtt.publish(topic_pub, json);
    Serial.print("📤 Enviado JSON: ");
    Serial.println(json);
  }
}

//===================== FUNCIONES =========================//

// Función: Conexión MQTT
static bool MQTT_Connect(void) 
{
  Serial.print("Conectando al broker MQTT...");

  // Conectarse con el ID A7670Client
  if (mqtt.connect("A7670Client")) 
  {
    Serial.println(" conectado ✔");

    // Suscribirse a un topic
    mqtt.subscribe(topic_sub);
    Serial.print("Suscrito a: "); Serial.println(topic_sub);

    return true;
  } 
  else 
  {
    Serial.print(" fallo, rc=");
    Serial.println(mqtt.state());
    return false;
  }
}

// Función: Callback MQTT
static void MQTT_Callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("📩 Mensaje en [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++) 
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}