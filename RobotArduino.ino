#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// ================= WiFi =================
const char* WIFI_SSID = "Seta";
const char* WIFI_PASS = "seta2002";

// ================= WebSocket =================
WebSocketsServer webSocket(80);

// ================= Motor Pins =================
// Left Motor
const int ENB = 25;
const int IN3 = 18;
const int IN4 = 19;

// Right Motor
const int ENA = 26;
const int IN1 = 23;
const int IN2 = 5;

// ================= PWM =================
const int PWM_CH_A = 0;   // Right
const int PWM_CH_B = 1;   // Left
const int PWM_FREQ  = 800;
const int PWM_RES   = 10;
const uint16_t MAX_PWM = 1023;

// ================= Robot Params =================
#define WHEEL_RADIUS 0.05
#define WHEEL_BASE   0.21

#define MIN_RPM      340
#define ZERO_EPS_RPM 25
#define RAMP_GAIN    0.8
#define MAX_RPM_LIM  700

// ================= Utils =================
static inline int16_t shape_deadzone_to_int(float rpm) {
  float s = (rpm >= 0) ? 1.f : -1.f;
  float a = fabs(rpm);

  if (a <= ZERO_EPS_RPM) return 0;

  float out = MIN_RPM + RAMP_GAIN * (a - ZERO_EPS_RPM);
  if (a > MIN_RPM) out = a;
  if (out > MAX_RPM_LIM) out = MAX_RPM_LIM;

  return (int16_t)(s * out + 0.5f);
}

void vwToRPM(float v, float w, float &rpmL, float &rpmR) {
  float wL = (v + (w * WHEEL_BASE / 2.0)) / WHEEL_RADIUS;
  float wR = (v - (w * WHEEL_BASE / 2.0)) / WHEEL_RADIUS;
  rpmL = wL * 60.0 / (2.0 * PI);
  rpmR = wR * 60.0 / (2.0 * PI);
}

// ================= Motor + Log =================
void applyMotors(int left, int right, uint8_t clientID) {
  // right=right-55;
  // Direction
  digitalWrite(IN1, right >= 0);
  digitalWrite(IN2, right <  0);
  digitalWrite(IN3, left  >= 0);
  digitalWrite(IN4, left  <  0);

  // PWM
  int pwmR = constrain(abs(right), 0, MAX_PWM);
  int pwmL = constrain(abs(left),  0, MAX_PWM);
  if (pwmR<75)
    pwmR=75;
  ledcWrite(PWM_CH_A, pwmR-75);
  ledcWrite(PWM_CH_B, pwmL);

  // Log back to client
  StaticJsonDocument<200> log;
  log["L_dir"] = (left  >= 0) ? "FWD" : "REV";
  log["L_pwm"] = pwmL;
  log["R_dir"] = (right >= 0) ? "FWD" : "REV";
  log["R_pwm"] = pwmR-75;

  String out;
  serializeJson(log, out);
  webSocket.sendTXT(clientID, out);
}

// ================= WebSocket Event =================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

  if (type == WStype_TEXT) {
    StaticJsonDocument<200> doc;
    if (deserializeJson(doc, payload)) return;

    if (doc.containsKey("v") && doc.containsKey("w")) {
      float v = doc["v"];
      float w = doc["w"];

      float rpmL, rpmR;
      vwToRPM(v, w, rpmL, rpmR);

      int rpmLi = shape_deadzone_to_int(rpmL);
      int rpmRi =shape_deadzone_to_int(rpmR);

      applyMotors(rpmLi, rpmRi, num);
    }
  }
}

// ================= Setup & Loop =================
void setup() {
  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n[WiFi] Connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, PWM_CH_A);
  ledcAttachPin(ENB, PWM_CH_B);
}

void loop() {
  webSocket.loop();
}
