// ========================================
// ESP32 #1 - TRANSMISSOR (MPU-9250 para PWM)
// VERSÃO DE ALTA PRECISÃO com Filtro Complementar
// ========================================

#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <math.h>

// Pinos SPI para ESP32 - MPU9250
#define HSPI_MISO 12 //SDO ou ADO
#define HSPI_MOSI 13 //SDI ou SDA
#define HSPI_SCLK 14 //SCL
#define HSPI_CS   15

// ========================================
// CONFIGURAÇÃO CRÍTICA: CANAL WI-FI
// ========================================
#define WIFI_CHANNEL 6  // ⚠️ DEVE SER IGUAL AO DO RECEPTOR!

// =================================================================
//  MAC Address do ESP32 receptor (PASSO CRÍTICO!)
// =================================================================
uint8_t receiverMAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // <-- Descubra o MAC ADDRESS do seu receptor e substitua aqui

// Registradores MPU-9250
#define MPU9250_WHO_AM_I    0x75
#define MPU9250_PWR_MGMT_1  0x6B
#define MPU9250_USER_CTRL   0x6A
#define MPU9250_CONFIG      0x1A
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_GYRO_CONFIG  0x1B 
#define MPU9250_SMPLRT_DIV  0x19
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_GYRO_XOUT_H  0x43 

// ========================================
// CONFIGURAÇÕES DO PWM
// ========================================
#define PWM_RESOLUTION 12   // 12 bits (0-4095)
#define DUTY_MINIMO 19      // Duty cycle inicial
#define DUTY_MAXIMO 4095    // Duty cycle final (ajuste se necessário)
#define GAMMA_VALUE 2.0     // Correção gamma (2.0, 2.5 ou 3.0)

// ========================================
// CONFIGURAÇÕES DO FILTRO
// ========================================
// 98% do ângulo vem do Giroscópio (rápido)
// 2% do ângulo vem do Acelerômetro (correção de drift)
#define A_COMP_FILTER_CONST 0.98

SPIClass hspi(HSPI);

// Escala do Giroscópio (para ±250 dps)
// Fatores de conversão (baseados na configuração 0x00)
float accelScale = 16384.0; // Para ±2g (Sensibilidade: 16384 LSB/g)
float gyroScale = 131.0;    // Para ±250 dps (Sensibilidade: 131 LSB/dps)

// Estrutura de dados MÍNIMA para máxima velocidade

uint16_t dutyValue; // duty cycle (2 bytes)

// Variáveis globais para cálculos (NÃO SÃO ENVIADAS)
float accelX_smooth = 0.0;
float accelY_smooth = 0.0;
float accelZ_smooth = 0.0;

// Variáveis do Filtro Complementar
float complementaryAngle = 0.0; // O ângulo final filtrado
unsigned long lastLoopTime = 0;   // Tempo do loop anterior (em microssegundos)

// Variáveis para controle de envio
unsigned long lastSendTime = 0;
volatile bool lastSendSuccess = false; // Flag para monitorar o callback

// ========================================
// CALLBACK DE ENVIO
// ========================================
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  lastSendSuccess = (status == ESP_NOW_SEND_SUCCESS);
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Delay para estabilizar
  
  Serial.println();
  Serial.println("===== ESP32 #1 - TRANSMISSOR (FILTRO COMPLEMENTAR v3 - YAW) =====");
  Serial.println();
  
  // Informações de configuração
  int maxDutyCycle = (1 << PWM_RESOLUTION) - 1;
  Serial.println("CONFIGURAÇÕES:");
  Serial.println("  Resolução PWM: " + String(PWM_RESOLUTION) + " bits (0-" + String(maxDutyCycle) + ")");
  Serial.println("  Duty Mínimo: " + String(DUTY_MINIMO));
  Serial.println("  Duty Máximo: " + String(DUTY_MAXIMO));
  Serial.println("  Gamma: " + String(GAMMA_VALUE, 1));
  Serial.println("  Medindo Ângulo: Yaw (Direção, X/Y)");
  Serial.println();
  
  // Inicialização MPU9250 (com retentativas)
  pinMode(HSPI_CS, OUTPUT);
  digitalWrite(HSPI_CS, HIGH);
  hspi.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_CS);
  hspi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  Serial.println("Inicializando MPU9250...");
  
  bool sensorOK = false;
  const int maxTentativas = 10; 
  
  for (int tentativa = 1; tentativa <= maxTentativas; tentativa++) {
    Serial.print("  Tentativa "); Serial.print(tentativa); Serial.print("/"); Serial.print(maxTentativas);
    
    writeRegisterMPU(MPU9250_PWR_MGMT_1, 0x80); delay(100); // Reset
    writeRegisterMPU(MPU9250_PWR_MGMT_1, 0x00); delay(100); // Sair do sleep
    writeRegisterMPU(MPU9250_USER_CTRL, 0x10); delay(10);

    uint8_t whoami = readRegisterMPU(MPU9250_WHO_AM_I);
    Serial.print(" - WHO_AM_I = 0x"); Serial.print(whoami, HEX);
    
    if (whoami == 0x71 || whoami == 0x73 || whoami == 0x70) {
      Serial.println(" ✓ OK!");
      sensorOK = true;
      break;
    } else {
      Serial.println(" ✗ Falha");
      delay(500); 
    }
  }
  
  if (!sensorOK) {
    Serial.println();
    Serial.println("ERRO: Sensor não detectado após múltiplas tentativas!");
    while (true) { delay(1000); } // Trava se o sensor falhar
  }

  // Configura o Giroscópio
  writeRegisterMPU(MPU9250_GYRO_CONFIG, 0x00);  // Giroscópio em +/- 250 dps
  
  writeRegisterMPU(MPU9250_CONFIG, 0x03);       // DLPF (filtro) em 41Hz
  writeRegisterMPU(MPU9250_ACCEL_CONFIG, 0x00); // Aceleração em +/- 2g
  writeRegisterMPU(MPU9250_SMPLRT_DIV, 0x09);   // Sample Rate = 100Hz (1kHz / (9+1))
  delay(100);
  Serial.println("✓ MPU9250 (Acel + Giro) configurado e pronto!");

  
  Serial.println();
  Serial.println("Inicializando ESP-NOW...");
  WiFi.mode(WIFI_STA);
  
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.print("Canal Wi-Fi: "); Serial.println(WIFI_CHANNEL);
  Serial.print("MAC Address Transmissor: "); Serial.println(WiFi.macAddress());
  Serial.print("MAC Address Receptor: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", receiverMAC[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERRO ao inicializar ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = WIFI_CHANNEL; 
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERRO ao adicionar peer");
    return;
  }

  Serial.println("✓ ESP-NOW OK!");
  Serial.println();
  Serial.println("Configuração concluída!");
  Serial.println("--------------------------------------");
  
  lastLoopTime = micros(); // Inicia o timer do filtro
}

void loop() {
  // ========================================
  // CÁLCULO DE TEMPO (dt) - ESSENCIAL PARA O FILTRO
  // ========================================
  unsigned long currentTime = micros();
  float dt = (currentTime - lastLoopTime) / 1000000.0; // Delta time em segundos
  lastLoopTime = currentTime;


  // ========================================
  // 1. LEITURA DOS SENSORES
  // ========================================
  
  // Ler Acelerômetro
  int16_t ax_raw = readRegister16MPU(MPU9250_ACCEL_XOUT_H);
  int16_t ay_raw = readRegister16MPU(MPU9250_ACCEL_XOUT_H + 2);
  int16_t az_raw = readRegister16MPU(MPU9250_ACCEL_XOUT_H + 4);

  // Ler Giroscópio
  int16_t gx_raw = readRegister16MPU(MPU9250_GYRO_XOUT_H);
  int16_t gy_raw = readRegister16MPU(MPU9250_GYRO_XOUT_H + 2);
  int16_t gz_raw = readRegister16MPU(MPU9250_GYRO_XOUT_H + 4);

  // Converter para g's (Acel) e graus/segundo (Giro)
  float accelX = ax_raw / accelScale;
  float accelY = ay_raw / accelScale;
  float accelZ = az_raw / accelScale;
  float gyroZ = gz_raw / gyroScale;
  
  // Suavização (mantendo sua lógica de filtro 'alpha' no acelerômetro)
  const float alpha = 0.2;
  accelX_smooth = accelX_smooth * (1 - alpha) + accelX * alpha;
  accelY_smooth = accelY_smooth * (1 - alpha) + accelY * alpha;
  accelZ_smooth = accelZ_smooth * (1 - alpha) + accelZ * alpha;

  // ========================================
  // 2. FILTRO COMPLEMENTAR (para YAW)
  // ========================================
  
  // Ângulo do Acelerômetro (Yaw)
  // Mede o ângulo "lento" baseado na direção da gravidade (X e Y)
  float rawAngle = atan2(accelX_smooth, accelY_smooth) * 180.0 / PI;
  float invertedAngle = 180.0 - rawAngle;
  // Normalizar
  if (invertedAngle > 180.0) {
    invertedAngle -= 360.0;
  } else if (invertedAngle < -180.0) {
    invertedAngle += 360.0;
  }
  float accelAngle = invertedAngle;

  // Ângulo do Giroscópio
  // Mede a *mudança* de ângulo "rápida" desde o último loop
  float gyroAngleChange = -gyroZ * dt; 

  // Filtro: 98% (Giro) + 2% (Acel)
  complementaryAngle = A_COMP_FILTER_CONST * (complementaryAngle + gyroAngleChange) 
                       + (1.0 - A_COMP_FILTER_CONST) * (accelAngle);

  // O Yaw é um ângulo de 360°, ele "dá a volta" (wraps around)
  if (complementaryAngle > 180.0) {
    complementaryAngle -= 360.0;
  } else if (complementaryAngle < -180.0) {
    complementaryAngle += 360.0;
  }


  // ========================================
  // 3. MAPEAMENTO (Ângulo -> PWM)
  // ========================================
  float normalized_value = 0.0;
  
  // Lógica ASSIMÉTRICA (0-40 para um lado, 0-90 para outro)
  // aplicada ao 'complementaryAngle' (Yaw)
  
  if (complementaryAngle >= 0 && complementaryAngle <= 40) {
    // Sentido (ex: direita): 0° → 1.0 (máximo), 40° → 0.0 (mínimo)
    normalized_value = 1.0 - (complementaryAngle / 40.0);
  }  
  else if (complementaryAngle > 40) {
    // Acima de 40°: manter valor mínimo
    normalized_value = 0.0;
  }
  else if (complementaryAngle < 0 && complementaryAngle >= -90) {
    // Sentido (ex: esquerda): 0° → 1.0 (máximo), -90° → 0.0 (mínimo)
    normalized_value = 1.0 - (abs(complementaryAngle) / 90.0);
  }
  else if (complementaryAngle < -90) {
    // Abaixo de -90°: manter valor mínimo
    normalized_value = 0.0;
  }

  // Aplicar correção gamma
  float corrected_value = pow(normalized_value, GAMMA_VALUE);

  // Mapear para faixa DUTY_MINIMO até DUTY_MAXIMO
  dutyValue = (uint16_t)(DUTY_MINIMO + (corrected_value * (DUTY_MAXIMO - DUTY_MINIMO)));

  // Garantir limites
  if (dutyValue > DUTY_MAXIMO) dutyValue = DUTY_MAXIMO;
  if (dutyValue < DUTY_MINIMO) dutyValue = DUTY_MINIMO;

  // ========================================
  // 4. ENVIO (Controlado por tempo)
  // ========================================
  unsigned long currentTimeMs = millis();
  if (currentTimeMs - lastSendTime >= 20) { // Enviar a cada 20ms (50Hz)
    esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&dutyValue, sizeof(dutyValue));
    lastSendTime = currentTimeMs;

    // Mostrar no Serial Monitor
    Serial.print("Yaw: "); 
    Serial.print(complementaryAngle, 1); // Imprime o ângulo Yaw
    Serial.print("° | Duty PWM: ");
    Serial.print(dutyValue);
    Serial.print(" (");
    Serial.print((dutyValue * 100.0) / DUTY_MAXIMO, 1);
    Serial.print("%) | API: ");
    Serial.print(result == ESP_OK ? "OK" : "ERRO");
    
    // O callback é assíncrono, então imprimimos o *último* status conhecido
    Serial.print(" | Callback: ");
    Serial.println(lastSendSuccess ? "✓" : "✗");
  }
  delay(1);
}


// ---------- Funções auxiliares ----------
uint8_t readRegisterMPU(uint8_t reg) {
  uint8_t value;
  digitalWrite(HSPI_CS, LOW);
  hspi.transfer(reg | 0x80); // 0x80 para LER
  value = hspi.transfer(0x00);
  digitalWrite(HSPI_CS, HIGH);
  return value;
}

void writeRegisterMPU(uint8_t reg, uint8_t value) {
  digitalWrite(HSPI_CS, LOW);
  hspi.transfer(reg & 0x7F); // 0x7F para ESCREVER
  hspi.transfer(value);
  digitalWrite(HSPI_CS, HIGH);
}

int16_t readRegister16MPU(uint8_t reg) {
  int16_t value;
  digitalWrite(HSPI_CS, LOW);
  hspi.transfer(reg | 0x80); // 0x80 para LER
  value = hspi.transfer(0x00) << 8; // Lê MSB
  value |= hspi.transfer(0x00);     // Lê LSB
  digitalWrite(HSPI_CS, HIGH);
  return value;
}
