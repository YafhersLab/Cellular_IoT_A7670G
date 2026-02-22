//===================== LIBRERIAS =========================//

// Librerias A7670E
#define TINY_GSM_MODEM_A7670
#include <TinyGsmClient.h>
#include <PubSubClient.h>

// Librerias Sensores
#include <Wire.h>
#include <RTClib.h>

//===================== DEFINICIONES =========================//

// Pines MODEM
#define MODEM_RESET_PIN   -1
#define MODEM_PWKEY       -1
#define MODEM_POWER_ON    -1
#define MODEM_TX          18
#define MODEM_RX          17
#define MODEM_BAUD        115200
#define SerialGsm         Serial1

// Pines sensores
#define RLY_PIN           41
#define TRIG_PIN          42
#define ECHO_PIN          39

// Definiciones I2C
#define I2C_SCL           1
#define I2C_SDA           21
#define BAT_SCL           2
#define BAT_SDA           3

// Direcciones I2C
#define MAX17048_ADDR     0x36

//===================== VARIABLES ===========================//

// Datos APN de Bitel
const char apn[]      = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Parametros del broker MQTT
const char* broker    = "broker.emqx.io"; 
const int   port      = 1883;                
const char* topic_sub = "Julian/SM/control/sub";
const char* topic_pub = "Julian/SM/control/pub";

// Variables de sensores
float HCSR04_distance = 0.0;
float MAX17048_battery = 0.0;
bool motorState = false;

//======================= OBJETOS ===========================//

// Objetos TinyGsm
TinyGsm modem(SerialGsm);
TinyGsmClient gsmClient(modem);

// Objeto cliente MQTT
PubSubClient mqtt(gsmClient);

// Objetos sensores
RTC_DS3231 rtc;
DateTime rtc_info;
TwoWire I2C_1 = TwoWire(0);
TwoWire I2C_2 = TwoWire(1);

//===================== PROTOTIPOS =========================//

static void MQTT_Callback(char* topic, byte* payload, unsigned int length);
static bool MQTT_Connect(void);
static void readSensors(void);

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

  // Inicialización de pines
  pinMode(RLY_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

   // Inicialización I2C 
  I2C_1.begin(I2C_SDA, I2C_SCL);
  I2C_2.begin(BAT_SDA, BAT_SCL);

  // Inicialización RTC DS3231 
  if (!rtc.begin(&I2C_1)) 
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

    // Lectura de sensores
    readSensors();
    motorState = digitalRead(RLY_PIN);

    // Crear JSON con valores de los sensores
    char json[100];
    snprintf(json, sizeof(json), "{\"Battery2\":\"%.2f\",\"Profundidad\":\"%.2f\",\"State\":\"%d\"}", MAX17048_battery, HCSR04_distance, motorState);
  
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
  // Recibir mensaje
  String msg = "";
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.print("📩 Mensaje recibido: "); Serial.println(msg);

  // Mover motor
  if (msg == "1") digitalWrite(RLY_PIN, 0);
  else if (msg == "0") digitalWrite(RLY_PIN, 1);
  motorState = digitalRead(RLY_PIN); 
}

// Función: Lectura de sensores
static void readSensors(void) 
{
  // Lectura del RTC
  rtc_info = rtc.now();

  // Lectura del HCSR04
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseInLong(ECHO_PIN, HIGH, 30000);
  if (duration > 0) HCSR04_distance = duration * 0.0343 / 2;

  // Batería
  I2C_2.beginTransmission(MAX17048_ADDR);
  I2C_2.write(0x02);
  I2C_2.endTransmission();
  I2C_2.requestFrom(MAX17048_ADDR, 2);
  uint16_t soc = (I2C_2.read() << 8) | I2C_2.read();
  if (soc > 65535) soc = 65535;
  MAX17048_battery = (float)soc / 65535.0 * 5.0;

  // Impresión de datos
  Serial.printf("⏰ %02d/%02d/%04d %02d:%02d:%02d | 👀 %.2f cm | 🔋 %.2f V | Motor: %d\r\n", rtc_info.day(), rtc_info.month(), rtc_info.year(), rtc_info.hour(), rtc_info.minute(), rtc_info.second(), HCSR04_distance, MAX17048_battery, motorState);
}