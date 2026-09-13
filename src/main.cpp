/*
 * Sistema de Irrigacao Inteligente com Arduino Uno
 *
 * Descricao do Projeto:
 * Firmware para monitoramento continuo de umidade do solo e controle autonomo de irrigacao.
 * O sistema avalia as condicoes do solo em tempo real, aciona um servomotor (atuando como
 * registro de agua) para molhar o solo ate atingir a meta de 80% e transmite dados de
 * telemetria periodicamente via porta serial.
 *
 * Regras de Negocio e Comportamento Operacional:
 * 1. Umidade do Solo:
 *    - Leitura analogica via sensor no pino A0 (faixa calibrada de 0 a 876).
 *    - Conversao da leitura para porcentagem de umidade (0.0% a 100.0%).
 * 2. Controle do Registro de Agua (Servomotor D9):
 *    - Se a umidade estiver abaixo de 80.0%, o registro e aberto (90 graus) para molhar o solo.
 *    - Se a umidade atingir ou ultrapassar a meta de 80.0%, o registro e fechado (0 graus).
 *    - Histerese operacional de 2.0% (acionamento em 78.0%) para evitar oscilacoes mecanicas no limiar.
 * 3. Sinalizacao Visual (LEDs):
 *    - LED Verde (D2): Solo com umidade adequada (registro fechado).
 *    - LED Amarelo (D7): Irrigacao ativa (registro aberto).
 */

#include <Arduino.h>
#include <Servo.h>

// Mapeamento de pinos do hardware
constexpr uint8_t pin_soil_moisture = A0;  // Entrada analogica: sensor de umidade do solo
constexpr uint8_t pin_led_adequate = 2;    // Saida digital: LED verde (solo adequado)
constexpr uint8_t pin_led_irrigating = 7;  // Saida digital: LED amarelo (irrigacao ativa)
constexpr uint8_t pin_servo = 9;           // Saida digital/PWM: sinal de controle do servomotor

// Parametros de calibracao do sensor de umidade do solo
constexpr int adc_dry_soil = 0;             // Valor ADC para solo completamente seco (0%)
constexpr int adc_wet_soil = 876;           // Valor ADC para solo saturado (100%)
constexpr float min_moisture_pct = 0.0F;    // Limite minimo de porcentagem
constexpr float max_moisture_pct = 100.0F;  // Limite maximo de porcentagem

// Limiares operacionais e histerese de controle
constexpr float target_moisture_pct = 80.0F;                                  // Meta de umidade requerida: 80.0%
constexpr float hysteresis_pct = 2.0F;                                        // Margem para evitar oscilacoes mecanicas
constexpr float start_irrigation_pct = target_moisture_pct - hysteresis_pct;  // Limiar de acionamento: 78.0%

// Posicionamentos do atuador (Servomotor)
constexpr int valve_angle_closed = 0;  // 0 graus: registro d'agua fechado
constexpr int valve_angle_open = 90;   // 90 graus: registro d'agua aberto

// Temporizacoes e comunicacao serial
constexpr unsigned long serial_baud_rate = 9600;       // Velocidade da porta serial (9600 bps)
constexpr unsigned long telemetry_interval_ms = 1000;  // Intervalo de transmissao serial (1 segundo)
constexpr unsigned long sampling_interval_ms = 200;    // Intervalo de amostragem do sensor (200 ms)
constexpr uint8_t telemetry_decimals = 1;              // Casas decimais na exibicao de porcentagem

// Variaveis de estado global do sistema
static Servo valve_servo;
static bool is_irrigating = false;
static int current_angle = valve_angle_closed;
static int current_raw_adc = 0;
static float current_moisture_pct = 0.0F;
static unsigned long last_telemetry_ms = 0;
static unsigned long last_sampling_ms = 0;

// Leitura do canal analogico do sensor de umidade do solo
static int read_soil_moisture_adc() { return analogRead(pin_soil_moisture); }

// Conversao da leitura analogica em porcentagem com limitacao de escala (clamp)
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

// Avalia a necessidade de irrigacao com base na histerese operacional
static bool is_irrigation_needed(const float moisture_pct, const bool currently_irrigating) {
  // Se o registro ja esta aberto (irrigando), continua ate atingir a meta de 80.0%
  if (currently_irrigating) {
    return moisture_pct < target_moisture_pct;
  }

  // Se o registro esta fechado, inicia nova irrigacao apenas se a umidade cair abaixo de 78.0%
  return moisture_pct < start_irrigation_pct;
}

// Posiciona o servomotor no angulo correspondente ao estado do registro de agua
static void control_water_valve(const bool open_valve) {
  const int target_angle = open_valve ? valve_angle_open : valve_angle_closed;

  if (current_angle != target_angle) {
    valve_servo.write(target_angle);
    current_angle = target_angle;
  }

  is_irrigating = open_valve;
}

// Atualizacao das saidas digitais dos LEDs de sinalizacao visual
static void update_visual_signaling(const bool irrigating) {
  if (irrigating) {
    digitalWrite(pin_led_irrigating, HIGH);
    digitalWrite(pin_led_adequate, LOW);
    return;
  }

  digitalWrite(pin_led_irrigating, LOW);
  digitalWrite(pin_led_adequate, HIGH);
}

// Retorna o rotulo textual do status do sistema para a telemetria serial
static const __FlashStringHelper* get_system_status_label(const bool irrigating) {
  if (irrigating) {
    return F("IRRIGANDO (REGISTRO ABERTO)");
  }
  return F("SOLO ADEQUADO (REGISTRO FECHADO)");
}

// Transmissao periodica das informacoes pela porta serial
static void transmit_telemetry(const float moisture_pct, const bool irrigating, const int raw_adc) {
  Serial.print(F("[TELEMETRIA] ADC: "));
  Serial.print(raw_adc);

  Serial.print(F(" | Umidade: "));
  Serial.print(moisture_pct, telemetry_decimals);
  Serial.print(F("% | Meta: "));
  Serial.print(target_moisture_pct, 0);

  Serial.print(F("% | Registro: "));
  Serial.print(irrigating ? F("ABERTO (90 deg)") : F("FECHADO (0 deg)"));

  Serial.print(F(" | Status: "));
  Serial.println(get_system_status_label(irrigating));
}

void setup() {
  Serial.begin(serial_baud_rate);
  Serial.println(F("=================================================="));
  Serial.println(F(" SISTEMA DE IRRIGACAO INTELIGENTE - ARDUINO UNO   "));
  Serial.println(F(" Status: Inicializado com Sucesso                 "));
  Serial.println(F("=================================================="));

  pinMode(pin_soil_moisture, INPUT);
  pinMode(pin_led_adequate, OUTPUT);
  pinMode(pin_led_irrigating, OUTPUT);

  // Inicializacao do servomotor no pino D9
  valve_servo.attach(pin_servo);

  // Amostragem inicial e determinacao do estado de partida seguro
  current_raw_adc = read_soil_moisture_adc();
  current_moisture_pct = calculate_moisture_percentage(current_raw_adc);
  const bool initial_irrigate = current_moisture_pct < target_moisture_pct;

  control_water_valve(initial_irrigate);
  update_visual_signaling(initial_irrigate);
}

void loop() {
  const unsigned long current_ms = millis();

  // Amostragem periodica dos sensores a cada 200 ms
  if (current_ms - last_sampling_ms >= sampling_interval_ms) {
    last_sampling_ms = current_ms;

    current_raw_adc = read_soil_moisture_adc();
    current_moisture_pct = calculate_moisture_percentage(current_raw_adc);

    // Avaliacao das regras operacionais da irrigacao
    const bool should_irrigate = is_irrigation_needed(current_moisture_pct, is_irrigating);

    // Atualizacao imediata dos atuadores e LEDs de sinalizacao visual
    control_water_valve(should_irrigate);
    update_visual_signaling(should_irrigate);
  }

  // Transmissao periodica da telemetria serial a cada 1 segundo (1000 ms)
  if (current_ms - last_telemetry_ms >= telemetry_interval_ms) {
    last_telemetry_ms = current_ms;
    transmit_telemetry(current_moisture_pct, is_irrigating, current_raw_adc);
  }
}
