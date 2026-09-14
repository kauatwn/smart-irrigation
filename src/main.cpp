/*
 * Sistema de Irrigação Inteligente com Arduino Uno
 *
 * Descrição do Projeto:
 * Firmware para monitoramento contínuo de umidade do solo e controle autônomo de irrigação.
 * O sistema avalia as condições do solo em tempo real, aciona um servomotor (atuando como
 * registro de água) para molhar o solo até atingir a meta de 80% e transmite dados de
 * telemetria periodicamente via porta serial.
 *
 * Regras de Negócio e Comportamento Operacional:
 * 1. Umidade do Solo:
 *    - Leitura analógica via sensor no pino A0 (faixa calibrada de 0 a 876).
 *    - Conversão da leitura para porcentagem de umidade (0.0% a 100.0%).
 * 2. Controle do Registro de Água (Servomotor D9):
 *    - Se a umidade estiver abaixo de 80.0%, o registro é aberto (90 graus) para molhar o solo.
 *    - Se a umidade atingir ou ultrapassar a meta de 80.0%, o registro é fechado (0 graus).
 *    - Histerese operacional de 2.0% (acionamento em 78.0%) para evitar oscilações mecânicas no limiar.
 * 3. Sinalização Visual (LEDs):
 *    - LED Verde (D2): Solo com umidade adequada (registro fechado).
 *    - LED Amarelo (D7): Irrigação ativa (registro aberto).
 */

#include <Arduino.h>
#include <Servo.h>

// Mapeamento de pinos do hardware
constexpr uint8_t pin_soil_moisture = A0;  // Entrada analógica: sensor de umidade do solo
constexpr uint8_t pin_led_adequate = 2;    // Saída digital: LED verde (solo adequado)
constexpr uint8_t pin_led_irrigating = 7;  // Saída digital: LED amarelo (irrigação ativa)
constexpr uint8_t pin_servo = 9;           // Saída digital/PWM: sinal de controle do servomotor

// Parâmetros de calibração do sensor de umidade do solo
constexpr int adc_dry_soil = 0;             // Valor ADC para solo completamente seco (0%)
constexpr int adc_wet_soil = 876;           // Valor ADC para solo saturado (100%)
constexpr float min_moisture_pct = 0.0F;    // Limite mínimo de porcentagem
constexpr float max_moisture_pct = 100.0F;  // Limite máximo de porcentagem

// Limiares operacionais e histerese de controle
constexpr float target_moisture_pct = 80.0F;                                  // Meta de umidade requerida: 80.0%
constexpr float hysteresis_pct = 2.0F;                                        // Margem para evitar oscilações mecânicas
constexpr float start_irrigation_pct = target_moisture_pct - hysteresis_pct;  // Limiar de acionamento: 78.0%

// Posicionamentos do atuador (Servomotor)
constexpr int valve_angle_closed = 0;  // 0 graus: registro d'água fechado
constexpr int valve_angle_open = 90;   // 90 graus: registro d'água aberto

// Temporizações e comunicação serial
constexpr unsigned long serial_baud_rate = 9600;       // Velocidade da porta serial (9600 bps)
constexpr unsigned long telemetry_interval_ms = 1000;  // Intervalo de transmissão serial (1 segundo)
constexpr unsigned long sampling_interval_ms = 200;    // Intervalo de amostragem do sensor (200 ms)
constexpr uint8_t telemetry_decimals = 1;              // Casas decimais na exibição de porcentagem

// Variáveis de estado global do sistema
static Servo valve_servo;
static bool is_irrigating = false;
static int current_angle = valve_angle_closed;
static int current_raw_adc = 0;
static float current_moisture_pct = 0.0F;
static unsigned long last_telemetry_ms = 0;
static unsigned long last_sampling_ms = 0;

// Leitura do canal analógico do sensor de umidade do solo
static int read_soil_moisture_adc() { return analogRead(pin_soil_moisture); }

// Conversão da leitura analógica em porcentagem com limitação de escala (clamp)
static float calculate_moisture_percentage(const int raw_adc) {
  if (raw_adc <= adc_dry_soil) {
    return min_moisture_pct;
  }
  if (raw_adc >= adc_wet_soil) {
    return max_moisture_pct;
  }

  constexpr auto adc_span = static_cast<float>(adc_wet_soil - adc_dry_soil);
  const float calculated_pct = static_cast<float>(raw_adc - adc_dry_soil) / adc_span * max_moisture_pct;

  if (calculated_pct < min_moisture_pct) {
    return min_moisture_pct;
  }
  if (calculated_pct > max_moisture_pct) {
    return max_moisture_pct;
  }
  return calculated_pct;
}

// Avalia a necessidade de irrigação com base na histerese operacional
static bool is_irrigation_needed(const float moisture_pct, const bool currently_irrigating) {
  // Se o registro já está aberto (irrigando), continua até atingir a meta de 80.0%
  if (currently_irrigating) {
    return moisture_pct < target_moisture_pct;
  }

  // Se o registro está fechado, inicia nova irrigação apenas se a umidade cair abaixo de 78.0%
  return moisture_pct < start_irrigation_pct;
}

// Posiciona o servomotor no ângulo correspondente ao estado do registro de água
static void control_water_valve(const bool open_valve) {
  const int target_angle = open_valve ? valve_angle_open : valve_angle_closed;

  if (current_angle != target_angle) {
    valve_servo.write(target_angle);
    current_angle = target_angle;
  }

  is_irrigating = open_valve;
}

// Atualização das saídas digitais dos LEDs de sinalização visual
static void update_visual_signaling(const bool irrigating) {
  if (irrigating) {
    digitalWrite(pin_led_irrigating, HIGH);
    digitalWrite(pin_led_adequate, LOW);
    return;
  }

  digitalWrite(pin_led_irrigating, LOW);
  digitalWrite(pin_led_adequate, HIGH);
}

// Retorna o rótulo textual do status do sistema para a telemetria serial
static const __FlashStringHelper* get_system_status_label(const bool irrigating) {
  if (irrigating) {
    return F("IRRIGANDO (REGISTRO ABERTO)");
  }
  return F("SOLO ADEQUADO (REGISTRO FECHADO)");
}

// Transmissão periódica das informações pela porta serial
static void transmit_telemetry(const float moisture_pct, const bool irrigating, const int raw_adc) {
  Serial.print(F("[TELEMETRIA] ADC: "));
  Serial.print(raw_adc);

  Serial.print(F(" | Umidade: "));
  Serial.print(moisture_pct, telemetry_decimals);
  Serial.print(F("% | Meta: "));
  Serial.print(target_moisture_pct, 0);

  Serial.print(F("% | Registro: "));
  Serial.print(irrigating ? F("ABERTO (90 graus)") : F("FECHADO (0 graus)"));

  Serial.print(F(" | Status: "));
  Serial.println(get_system_status_label(irrigating));
}

void setup() {
  Serial.begin(serial_baud_rate);
  Serial.println(F("=================================================="));
  Serial.println(F(" SISTEMA DE IRRIGAÇÃO INTELIGENTE - ARDUINO UNO "));
  Serial.println(F(" Status: Inicializado com Sucesso                 "));
  Serial.println(F("=================================================="));

  pinMode(pin_soil_moisture, INPUT);
  pinMode(pin_led_adequate, OUTPUT);
  pinMode(pin_led_irrigating, OUTPUT);

  // Inicialização do servomotor no pino D9
  valve_servo.attach(pin_servo);

  // Amostragem inicial e determinação do estado de partida seguro
  current_raw_adc = read_soil_moisture_adc();
  current_moisture_pct = calculate_moisture_percentage(current_raw_adc);
  const bool initial_irrigate = current_moisture_pct < target_moisture_pct;

  control_water_valve(initial_irrigate);
  update_visual_signaling(initial_irrigate);
}

void loop() {
  const unsigned long current_ms = millis();

  // Amostragem periódica dos sensores a cada 200 ms
  if (current_ms - last_sampling_ms >= sampling_interval_ms) {
    last_sampling_ms = current_ms;

    current_raw_adc = read_soil_moisture_adc();
    current_moisture_pct = calculate_moisture_percentage(current_raw_adc);

    // Avaliação das regras operacionais da irrigação
    const bool should_irrigate = is_irrigation_needed(current_moisture_pct, is_irrigating);

    // Atualização imediata dos atuadores e LEDs de sinalização visual
    control_water_valve(should_irrigate);
    update_visual_signaling(should_irrigate);
  }

  // Transmissão periódica da telemetria serial a cada 1 segundo (1000 ms)
  if (current_ms - last_telemetry_ms >= telemetry_interval_ms) {
    last_telemetry_ms = current_ms;
    transmit_telemetry(current_moisture_pct, is_irrigating, current_raw_adc);
  }
}
