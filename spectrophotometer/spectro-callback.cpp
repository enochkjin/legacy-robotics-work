#include <Arduino.h>
#include <ESP32Servo.h>
#include "driver/twai.h"

const int LED_PIN   = 9;
const int SERVO_PIN = 1;
const int PD_PIN    = 2;

#define CAN_TX GPIO_NUM_5
#define CAN_RX GPIO_NUM_4

#define REQUEST_ID  0x4C8   // Pi -> ESP32
#define RESPONSE_ID 0x028   // ESP32 -> Pi

Servo turret;

struct Measurement {
  float voltage;
  float absorbance;
};

// ---- Take one photodiode reading ----
Measurement readPhotodiode(const char* label) {

  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(PD_PIN);
    delay(5);
  }

  float raw = sum / 10.0f;
  float voltage = raw * (3.3f / 4095.0f);
  float absorbance = 1.63f * voltage - 2.34f;

  Serial.print(label);
  Serial.print("  voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" V  absorbance: ");
  Serial.println(absorbance, 4);

  Measurement m;
  m.voltage = voltage;
  m.absorbance = absorbance;
  return m;
}

// ---- Full sweep through cuvettes ----
Measurement runMeasurement() {

  const int cuvetteAngles[] = {-40, 65, -40};
  const int numCuvettes = sizeof(cuvetteAngles) / sizeof(cuvetteAngles[0]);

  float voltageSum = 0;

  for (int i = 0; i < numCuvettes; i++) {
    turret.write(cuvetteAngles[i]);
    delay(1000);
    Measurement m = readPhotodiode("Reading:");
    voltageSum += m.voltage;
  }

  float avgVoltage = voltageSum / numCuvettes;
  float absorbance = 1.63f * avgVoltage - 2.34f;

  Measurement result;
  result.voltage = avgVoltage;
  result.absorbance = absorbance;

  return result;
}

// ---- Send CAN response ----
void sendResponse(float voltage, float absorbance) {

  twai_message_t tx_msg = {};
  tx_msg.identifier = RESPONSE_ID;
  tx_msg.extd = 0;              // 11-bit standard
  tx_msg.rtr = 0;
  tx_msg.data_length_code = 8;

  memcpy(&tx_msg.data[0], &voltage, 4);
  memcpy(&tx_msg.data[4], &absorbance, 4);

  twai_transmit(&tx_msg, pdMS_TO_TICKS(100));
}

// ---- CAN "callback" handler ----
void handleCanRequest() {

  Measurement m = runMeasurement();
  sendResponse(m.voltage, m.absorbance);

  Serial.println("Measurement sent over CAN.");
}

void setup() {

  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  turret.attach(SERVO_PIN);

  // ---- TWAI Setup ----
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);

  twai_timing_config_t t_config =
      TWAI_TIMING_CONFIG_500KBITS();

  twai_filter_config_t f_config =
      TWAI_FILTER_CONFIG_ACCEPT_ALL();

  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();
}

void loop() {

  twai_message_t rx_msg;

  // Poll for incoming CAN frame
  if (twai_receive(&rx_msg, pdMS_TO_TICKS(100)) == ESP_OK) {

    if (rx_msg.identifier == REQUEST_ID) {
      handleCanRequest();   // <-- this is the CAN callback
    }
  }

}
