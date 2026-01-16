#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <PID_v1.h>

// ===== MAX31855 =====
#define MAXDO   12
#define MAXCLK  14
#define MAXCS1  18
#define MAXCS2  19

// ===== SSR =====
#define SSR1_PIN  13
#define SSR2_PIN  4

// ===== Objects =====
Adafruit_MAX31855 tc1(MAXCLK, MAXCS1, MAXDO);
Adafruit_MAX31855 tc2(MAXCLK, MAXCS2, MAXDO);

// ===== PID vars =====
double temp1_raw, temp2_raw;
double temp1_filt = 25, temp2_filt = 25;
double out1, out2;

// User setpoints (serial adjustable)
double setpoint1 = 100.0;
double setpoint2 = 100.0;

// Effective setpoints (USED by PID)
double sp1_eff = 100.0;
double sp2_eff = 100.0;

// ===== VERY SLOW PID =====
double Kp = 3;
double Ki = 0.1;
double Kd = 0.0;

PID pid1(&temp1_filt, &out1, &sp1_eff, Kp, Ki, Kd, DIRECT);
PID pid2(&temp2_filt, &out2, &sp2_eff, Kp, Ki, Kd, DIRECT);

// ===== Time proportional SSR =====
const unsigned long windowSize = 2000;
unsigned long windowStart1, windowStart2;

// ===== Sensor delay model =====
const double tau = 2.0;
unsigned long lastFilterUpdate = 0;

// ===== Heating bias =====
const double heatBias = 25.0; // °C

// ===== Serial =====
char cmd[24];
uint8_t idx = 0;

void setup() {
  Serial.begin(115200);

  pinMode(SSR1_PIN, OUTPUT);
  pinMode(SSR2_PIN, OUTPUT);

  tc1.begin();
  tc2.begin();

  pid1.SetOutputLimits(0, windowSize);
  pid2.SetOutputLimits(0, windowSize);
  pid1.SetMode(AUTOMATIC);
  pid2.SetMode(AUTOMATIC);

  windowStart1 = millis();
  windowStart2 = millis();
  lastFilterUpdate = millis();

  Serial.println("Slow PID Heater Control (fixed)");
  Serial.println("Commands: T1= , T2= , STATUS");
}

void loop() {
  readAndFilterTemps();

  // ===== Apply heating bias =====
  sp1_eff = setpoint1;
  sp2_eff = setpoint2;

  if (temp1_filt < setpoint1)
    sp1_eff = setpoint1 - heatBias;

  if (temp2_filt < setpoint2)
    sp2_eff = setpoint2 - heatBias;

  pid1.Compute();
  pid2.Compute();

  driveSSR(SSR1_PIN, out1, windowStart1);
  driveSSR(SSR2_PIN, out2, windowStart2);

  handleSerial();
  printStatus();
  delay(200);
}

// ===== Functions =====

void readAndFilterTemps() {
  unsigned long now = millis();
  double dt = (now - lastFilterUpdate) / 1000.0;
  lastFilterUpdate = now;

  temp1_raw = tc1.readCelsius();
  temp2_raw = tc2.readCelsius();

  if (!isnan(temp1_raw))
    temp1_filt += (temp1_raw - temp1_filt) * (dt / tau);

  if (!isnan(temp2_raw))
    temp2_filt += (temp2_raw - temp2_filt) * (dt / tau);
}

void driveSSR(int pin, double output, unsigned long &windowStart) {
  unsigned long now = millis();
  if (now - windowStart >= windowSize)
    windowStart += windowSize;

  digitalWrite(pin, (output > (now - windowStart)) ? HIGH : LOW);
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      cmd[idx] = 0;
      parseCommand(cmd);
      idx = 0;
    } else if (idx < sizeof(cmd) - 1) {
      cmd[idx++] = toupper(c);
    }
  }
}

void parseCommand(char *c) {
  if (strncmp(c, "T1=", 3) == 0) {
    setpoint1 = atof(c + 3);
  } 
  else if (strncmp(c, "T2=", 3) == 0) {
    setpoint2 = atof(c + 3);
  } 
  else if (strcmp(c, "STATUS") == 0) {
    printStatus();
    return;
  }
  printStatus();
}

void printStatus() {
  Serial.println("---- STATUS ----");
  Serial.print("Raw1: "); Serial.print(temp1_raw);
  Serial.print("  Filt1: "); Serial.print(temp1_filt);
  Serial.print("  SP1: "); Serial.print(setpoint1);
  Serial.print("  Eff1: "); Serial.println(sp1_eff);

  Serial.print("Raw2: "); Serial.print(temp2_raw);
  Serial.print("  Filt2: "); Serial.print(temp2_filt);
  Serial.print("  SP2: "); Serial.print(setpoint2);
  Serial.print("  Eff2: "); Serial.println(sp2_eff);
}
