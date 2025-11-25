// ========================================
// ESP32 #2 - RECEPTOR (PWM com ESP-NOW)
// Versão compatível com ESP32 Arduino Core 3.x
// ========================================

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ========================================
// CONFIGURAÇÕES DO PWM
// ========================================
#define PWM_PIN 5        // Pino de saída do PWM
#define PWM_FREQ 5000     // 5 kHz
#define PWM_RESOLUTION 12 // 12 bits (0-4095)

// ========================================
// CONFIGURAÇÕES DO RECEPTOR
// ========================================
#define WIFI_CHANNEL 6    // DEVE SER IGUAL AO DO TRANSMISSOR!
#define TIMEOUT_MS 10000 // após 10 segundos zera o resistor (pwm máximo)


// Variáveis de estado
volatile bool newDataReceived = false;
volatile uint16_t lastDutyValue = 0;
unsigned long lastDataTimestamp = 0;
bool isPwmActive = false;


// CALLBACK - Chamado quando um dado chega
// ========================================
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy((void*)&lastDutyValue, incomingData, sizeof(uint16_t));
  lastDataTimestamp = millis();
  newDataReceived = true;
}

// ========================================
// SETUP
// ========================================
void setup() {
  Serial.begin(115200);
  delay(500); // Delay para estabilizar
  
  Serial.println();
  Serial.println("===== ESP32 #2 - RECEPTOR PWM =====");
  Serial.println();

  // 1. Configurar PWM
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PWM_PIN, 4095); // Começa ligado (resistência mínima)
  Serial.println("✓ PWM configurado (ligado)");

  // 2. Configurar Wi-Fi e ESP-NOW
  WiFi.mode(WIFI_STA);
  
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  //Serial.print("Canal Wi-Fi: "); Serial.println(WIFI_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERRO ao inicializar ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("✓ ESP-NOW inicializado.");
  Serial.println();
  Serial.println("Inicialização completa.");
  Serial.println("Aguardando dados do transmissor...");
  Serial.println("--------------------------------------");
}

// ========================================
// LOOP
// ========================================
void loop() {
  
  if (!isPwmActive) ledcWrite(PWM_PIN, 4095);
  // Parte 1: Processar dados se eles chegaram
  if (newDataReceived) {
    newDataReceived = false;

    // FUNÇÃO "ACORDAR": Se este é o PRIMEIRO dado recebido, ativa o PWM
    
    if (!isPwmActive) {
      isPwmActive = true;
      Serial.println();
      Serial.println("======================================");
      Serial.println("PRIMEIRO DADO RECEBIDO! Ativando PWM.");
      Serial.println("======================================");
    }
    
    // Atualiza o PWM
    ledcWrite(PWM_PIN, lastDutyValue);

    Serial.print("Duty Recebido: ");
    Serial.print(lastDutyValue);
    Serial.print(" (");
    Serial.print((lastDutyValue * 100.0) / 4095.0, 1);
    Serial.println("%)");
  }

  // Parte 2: Checar o Timeout
  if (isPwmActive && (millis() - lastDataTimestamp > TIMEOUT_MS)) {
    Serial.println();
    Serial.println("======================================");
    Serial.println("TIMEOUT DE 10 segundos!");
    Serial.println("Nenhum dado recebido. Mudando para PWM máximo.");
    Serial.println("======================================");
    
    isPwmActive = false;
    ledcWrite(PWM_PIN, 4095);
  }

  // Pequeno delay para não sobrecarregar o processador
  delay(5);
}