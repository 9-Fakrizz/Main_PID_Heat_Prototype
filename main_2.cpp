#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "max6675.h"

// ---------------- Pin Definitions ----------------
// Relays
#define RELAY1  2
#define RELAY2  15
#define RELAY3  26

// Buttons
#define BUTTON1 34   // B1
#define BUTTON2 35   // B2
#define BUTTON3 39   // B3

// Potentiometer
#define POTENTIOMETER 36   // PT3

// Buzzer
#define BUZZER 32

// Thermocouple MAX6675
#define MAX6675_SCK 33
#define MAX6675_SO  25
#define MAX6675_CS  19

// IR Temperature Sensors (I2C)
#define I2C_SDA 21
#define I2C_SCL 22

// Extra Chip Selects
#define CS2 18
#define CS3 5
#define CS4 4

// Extra Potentiometer
#define PT1 17
#define PT2 16

// ---------------- Objects ----------------
MAX6675 thermocouple(MAX6675_SCK, MAX6675_CS, MAX6675_SO);
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);

  // Relays
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);

  // Buttons
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);

  // Potentiometer
  pinMode(POTENTIOMETER, INPUT);

  // Buzzer
  pinMode(BUZZER, OUTPUT);

  // Extra CS
  pinMode(CS2, OUTPUT);
  pinMode(CS3, OUTPUT);
  pinMode(CS4, OUTPUT);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("ESP32 Project Start");
  display.display();

  Serial.println("System Initialized!");
}

// ---------------- Functions ----------------
void relayControl(int relay, bool state) {
  digitalWrite(relay, state ? HIGH : LOW);
}

void buzzerBeep(int duration) {
  digitalWrite(BUZZER, HIGH);
  delay(duration);
  digitalWrite(BUZZER, LOW);
}

float readPot() {
  return analogRead(POTENTIOMETER) * (3.3 / 4095.0); // Convert to volts
}

bool readButton(int buttonPin) {
  return digitalRead(buttonPin) == LOW; // Pressed = LOW
}

double readThermocouple() {
  return thermocouple.readCelsius();
}

// ---------------- Loop ----------------
void loop() {
  // Example: Read buttons
  if (readButton(BUTTON1)) {
    Serial.println("Button1 Pressed -> Relay1 ON");
    relayControl(RELAY1, true);
  } else {
    relayControl(RELAY1, false);
  }

  if (readButton(BUTTON2)) {
    Serial.println("Button2 Pressed -> Relay2 ON");
    relayControl(RELAY2, true);
  } else {
    relayControl(RELAY2, false);
  }

  if (readButton(BUTTON3)) {
    Serial.println("Button3 Pressed -> Relay3 ON + Buzzer");
    relayControl(RELAY3, true);
    buzzerBeep(200);
  } else {
    relayControl(RELAY3, false);
  }

  // Read potentiometer
  float potVal = readPot();
  Serial.print("Potentiometer Voltage: ");
  Serial.println(potVal);

  // Read thermocouple
  double tempC = readThermocouple();
  Serial.print("Thermocouple Temp: ");
  Serial.print(tempC);
  Serial.println(" °C");

  // Show on OLED
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Temp: ");
  display.print(tempC);
  display.println(" C");
  display.print("Pot: ");
  display.print(potVal, 2);
  display.println(" V");
  display.display();

  delay(500);
}
