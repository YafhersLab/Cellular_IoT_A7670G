//======================= LIBRERIAS ==========================//

#include <Wire.h>
#include <RTClib.h>
#include <math.h>
#include <driver/i2s.h>

//===================== DEFINICIONES =========================//

// Definiciones RLY
#define RLY_PIN       41

// Definiciones HCSR04
#define TRIG_PIN      42   
#define ECHO_PIN      39 

// Definiciones I2C
#define I2C_SCL       1
#define I2C_SDA       21

// Direcciones I2C
#define PCF8574_ADDR  0x27 
#define MAX17048_ADDR 0x36

// Definiciones I2S MAX98357A
#define I2S_DOUT      6
#define I2S_BCLK      5
#define I2S_LRC       4

// Definiciones I2C MAX170348
#define BAT_SCL       2
#define BAT_SDA       3 

//====================== VARIABLES ==========================//

// Variables lectura RTC DS3231
DateTime rtc_info;

// Variables lectura HCSR04
float HCSR04_distance = 0.0;

// Variables de comando I2C para PCF8574
byte pcf_state = 0xFF;

// Variable del nivel de batería
float MAX17048_battery = 0.0;

//======================= OBJETOS ===========================//

RTC_DS3231 rtc;
TwoWire I2C_1 = TwoWire(0); 
TwoWire I2C_2 = TwoWire(1);

//====================== PROTOTIPOS =========================//

// Prototipos de Tasks
void readSensors(void* pvParameters);
void moveMotor(void* pvParameters);

// Prototipos de Funciones
void setRGB(bool red, bool green, bool blue);
bool getStateMotor();
void playTone(float frequency, int duration_ms, float volume = 0.5);
float getBatteryLevel();

//====================== MAIN CODE ==========================//

// Void Setup
void setup() 
{
  // Inicialización serial
  Serial.begin(115200);
  while(!Serial);
  
  // Configuración de pines
  pinMode(RLY_PIN,  OUTPUT);
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

  // Verificar la hora del RTC DS3231
  if (rtc.lostPower()) 
  {
    Serial.println("⚠️ RTC sin hora, ajustando a la hora de compilación");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Inicialización del PCF8574
  pcf_state = 0b01111111;
  I2C_1.beginTransmission(PCF8574_ADDR);
  I2C_1.write(pcf_state);
  I2C_1.endTransmission();

  // Inicialización MAX98357A
  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false
  };

  // Configuración de pines MAX98357A
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  // Instalación de driver I2S
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  // Configuración de Tasks
  xTaskCreate(readSensors, "Read Sensors", 4096, NULL, 3, NULL);
  xTaskCreate(moveMotor, "Move Motor", 4096, NULL, 3, NULL);

  // Elimino el loop
  vTaskDelete(NULL);
}

// Task: Lectura de sensores de la planta
void readSensors(void* pvParameters)
{
  while(1)
  {
    // Lectura del RTC DS3231
    rtc_info = rtc.now();

    // Lectura del HCSR04
    digitalWrite(TRIG_PIN, 0);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, 1);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, 0);

    // Calcular distancia del HCSR04
    long duration = pulseIn(ECHO_PIN, HIGH);
    HCSR04_distance = duration * 0.0343 / 2;

    // Obtener porcentaje de batería
    MAX17048_battery = getBatteryLevel();

    // Pintar el led según la batería
    if (MAX17048_battery > 3.5)
    {
      setRGB(0,1,0); // verde
    }
    else
    {
      setRGB(1,0,0); // rojo
    }

    // Imprimir lectura de sensores
    Serial.printf("⏰ %02d/%02d/%04d %02d:%02d:%02d | 👀 %.2f cm | 🔋 %.2f V\r\n",rtc_info.day(), rtc_info.month(), rtc_info.year(), rtc_info.hour(), rtc_info.minute(), rtc_info.second(), HCSR04_distance , MAX17048_battery);

    // Delay entre lecturas
    vTaskDelay(pdMS_TO_TICKS(1000)); 
  }
}

// Task: Controlar movimiento del motor
void moveMotor(void* pvParameters)
{
  while(1)
  {
    // Toggle motor
    digitalWrite(RLY_PIN, !digitalRead(RLY_PIN));

    // Verificar estado del motor
    Serial.printf("Estado del motor: %u\r\n", getStateMotor());

    float notes[] = {261.63, 293.66, 329.63};
    for (int i = 0; i < 3; i++) {
      playTone(notes[i], 400, 0.2);
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Delay entre lecturas
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// Funcion: Setear el color del led RGB
void setRGB(bool red, bool green, bool blue)
{
  // Escribir el color en el led RGB
  bitWrite(pcf_state, 5, red);
  bitWrite(pcf_state, 6, green);
  bitWrite(pcf_state, 7, blue);

  // Enviar comando I2C
  I2C_1.beginTransmission(PCF8574_ADDR);
  I2C_1.write(pcf_state);
  I2C_1.endTransmission();
}

// Funcion: Obtener el estado del motor
bool getStateMotor()
{
  // Enviar solicitud I2C
  I2C_1.requestFrom(PCF8574_ADDR, 1);

  // Verificar datos disponibles
  if (I2C_1.available()) 
  {
    byte readVal = I2C_1.read();
    return (readVal & (1 << 0)); 
  }

  return false;
}

// Funcion: Reproducción de Tono en el MAX98357A
void playTone(float frequency, int duration_ms, float volume) 
{
  // Parametros de reproducción
  const int sampleRate = 44100;
  const int maxAmplitude = 32767;
  int amplitude = maxAmplitude * volume; 

  int numSamples = (sampleRate * duration_ms) / 1000;
  float phase = 0;
  int16_t sample[1];
  size_t bytesWritten;

  // Reproducir Tono
  for (int i = 0; i < numSamples; i++) 
  {
    sample[0] = (int16_t)(amplitude * sin(phase));
    phase += (2.0 * PI * frequency) / sampleRate;
    if (phase > 2.0 * PI) phase -= 2.0 * PI;
    i2s_write(I2S_NUM_0, sample, sizeof(sample), &bytesWritten, portMAX_DELAY);
  }

  // Envía 10 ms de silencio para apagar el sonido residual
  int silentSamples = (sampleRate * 10) / 1000;
  sample[0] = 0;
  for (int i = 0; i < silentSamples; i++)
  {
    i2s_write(I2S_NUM_0, sample, sizeof(sample), &bytesWritten, portMAX_DELAY);
  }

  // Reiniciar buffer DMA
  i2s_zero_dma_buffer(I2S_NUM_0);
}

// Funcion: Obtener nivel de la bateria
float getBatteryLevel()
{
  // Solicitud I2C
  I2C_2.beginTransmission(MAX17048_ADDR);
  I2C_2.write(0x02);
  I2C_2.endTransmission();

  // Solicitar valor de batería
  I2C_2.requestFrom(MAX17048_ADDR, 2);
  uint16_t soc = (I2C_2.read() << 8) | I2C_2.read();
  if (soc > 65535) soc = 65535;
    
  // Retornar valor de la batería
  return (float)soc / 65535.0 * 5;
}

void loop() 
{
}