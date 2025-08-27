#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MAX6675.h>
#include <EEPROM.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int dataPin   = 12;
const int clockPin  = 14;
const int selectPin = 18;
MAX6675 thermoCouple(selectPin, dataPin, clockPin);

const int button1_pin = 25; // Menu up / Back (hold)
const int button2_pin = 26; // Menu down
const int button3_pin = 33; // Select
const int encoderPin = 32;  // Analog input for rotary encoder simulation (placeholder)

const int heaterRelayPin = 2;

const int buzzerPin = 27;  
int adcValue = 0;
bool relayState;

unsigned long lastInteractionTime = 0;
const unsigned long standbyTimeout = 10000;

float currentTemp = 0;
int menuIndex = 0;
int subMenuIndex = 0;

unsigned long lastButtonPress = 0;
const unsigned long buttonDelay = 250; // milliseconds

enum MenuState {
  MENU_STANDBY,
  MENU_MAIN,
  MENU_START_TEMP,
  MENU_MAX_TEMP,
  MENU_IDLE_OFF,
  MENU_LIGHT_SOUND,
  MENU_IR_ALARM,
  MENU_UNIT,
  MENU_ABOUT,
  MENU_SET_CUSTOM_TEMP,
  MENU_MANUAL_TEST       // <--- Added new manual test menu
};

MenuState currentState = MENU_STANDBY;
bool tempUnitIsCelsius = true;

float startTemps[] = {0, 260, 287, 315, 343, 371, 398, 426};
float customStartTemp = 0;
float maxTempLock = 537;
int idleTimeoutOption = 0; // 0=15m, 1=30m, 2=60m, 3=Always On
bool lightOn = false;
bool soundOn = true;
int irAlarm1 = 270;
int irAlarm2 = 270;

void selectMenuOption() {
  switch (currentState) {
    case MENU_MAIN:
      if (menuIndex <= 6) {
        currentState = (MenuState)(MENU_START_TEMP + menuIndex);
      } else if (menuIndex == 7) {
        currentState = MENU_MANUAL_TEST;
      }
      subMenuIndex = 0;
      break;

    case MENU_START_TEMP:
      if (subMenuIndex < 8) {
        customStartTemp = startTemps[subMenuIndex];
      } else {
        currentState = MENU_SET_CUSTOM_TEMP;
        customStartTemp = 0;
      }
      break;

    case MENU_SET_CUSTOM_TEMP:
      if (customStartTemp < 537) customStartTemp++;
      break;

    case MENU_MAX_TEMP:
      if (subMenuIndex < 537) maxTempLock = subMenuIndex;
      break;

    case MENU_UNIT:
      tempUnitIsCelsius = !tempUnitIsCelsius;
      break;

    case MENU_LIGHT_SOUND:
      if (subMenuIndex % 2 == 0) lightOn = !lightOn;
      else soundOn = !soundOn;
      break;

    case MENU_IDLE_OFF:
      idleTimeoutOption = subMenuIndex % 4;
      break;

    case MENU_IR_ALARM:
      irAlarm1 = subMenuIndex % 271;
      irAlarm2 = (subMenuIndex + 50) % 271;
      break;

    case MENU_MANUAL_TEST:
      switch (subMenuIndex) {
        case 0: // Relay
          relayState = !relayState;
          digitalWrite(heaterRelayPin, relayState);
          break;
        case 1: // Buzzer
          tone(buzzerPin, 1000, 200); // 1kHz beep for 200ms
          break;
        case 2: // Buttons -> display only
          break;
        case 3: // Knob/ADC -> display only
          break;
      }
      break;

    default:
      currentState = MENU_MAIN;
      break;
  }
}

void handleInput() {
  if (millis() - lastButtonPress < buttonDelay) return;

  if (digitalRead(button1_pin) == LOW || digitalRead(button2_pin) == LOW || digitalRead(button3_pin) == LOW) {
    if (currentState == MENU_STANDBY) {
      currentState = MENU_MAIN;
      lastInteractionTime = millis();
      lastButtonPress = millis();
      return;
    } 
  }

  // Long press Button1 to go back to main menu
  if (digitalRead(button1_pin) == LOW) {
    unsigned long pressStart = millis();
    while (digitalRead(button1_pin) == LOW) {
      if (millis() - pressStart > 1000) { // Hold > 1 second
        if (currentState != MENU_MAIN && currentState != MENU_STANDBY) {
          currentState = MENU_MAIN;
          subMenuIndex = 0;
          lastInteractionTime = millis();
          lastButtonPress = millis();
          return;
        }
      }
    }

    // Short press behavior
    if (currentState == MENU_MAIN) {
      menuIndex = (menuIndex - 1 + 7) % 7;
    } else {
      subMenuIndex--;
    }
    lastInteractionTime = millis();
    lastButtonPress = millis();
  }

  if (digitalRead(button2_pin) == LOW) {
    if (currentState == MENU_MAIN) {
      menuIndex = (menuIndex + 1) % 7;
    } else {
      subMenuIndex++;
    }
    lastInteractionTime = millis();
    lastButtonPress = millis();
  }

  if (digitalRead(button3_pin) == LOW) {
    ESP_LOGI(TAG,"(handle_input)Current State : %s ",MenuStateToStr(currentState) );
    selectMenuOption();
    lastInteractionTime = millis();
    lastButtonPress = millis();
  }
}

void readTemperature() {
  float currentTemp = thermoCouple.readCelsius();
  if (!tempUnitIsCelsius) {
    currentTemp = currentTemp * 9.0 / 5.0 + 32.0;
  }
}

void controlHeater() {
  float compareTemp = tempUnitIsCelsius ? customStartTemp : (customStartTemp * 9.0 / 5.0 + 32.0);
  float maxTemp     = tempUnitIsCelsius ? maxTempLock : (maxTempLock * 9.0 / 5.0 + 32.0);

  if (currentTemp < compareTemp) {
    digitalWrite(heaterRelayPin, HIGH);
  } else if (currentTemp >= maxTemp) {
    digitalWrite(heaterRelayPin, LOW);
  }
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  switch (currentState) {
    case MENU_STANDBY:
      display.setTextSize(2);
      display.setCursor(10, 10);
      display.println("STANDBY");
      display.setTextSize(1);
      display.setCursor(0, 40);
      display.print("Temp: ");
      display.print(currentTemp);
      display.print(tempUnitIsCelsius ? " C" : " F");
      break;
    case MENU_MAIN:
      display.println(menuIndex == 0 ? "> Start Temp" : "  Start Temp");
      display.println(menuIndex == 1 ? "> Max Temp Lock" : "  Max Temp Lock");
      display.println(menuIndex == 2 ? "> Idle Off" : "  Idle Off");
      display.println(menuIndex == 3 ? "> Light & Sound" : "  Light & Sound");
      display.println(menuIndex == 4 ? "> IR Alarm" : "  IR Alarm");
      display.println(menuIndex == 5 ? "> Unit" : "  Unit");
      display.println(menuIndex == 6 ? "> About" : "  About");
      display.println(menuIndex == 7 ? "> Manual Test" : "  Manual Test"); // <--- added here
      break;
    case MENU_START_TEMP:
      for (int i = 0; i < 8; i++) {
        display.println(subMenuIndex == i ? "> " + String((int)startTemps[i]) + " C" : "  " + String((int)startTemps[i]) + " C");
      }
      display.println(subMenuIndex == 8 ? "> CUSTOM" : "  CUSTOM");
      break;
    case MENU_SET_CUSTOM_TEMP:
      display.setCursor(0, 10);
      display.print("Custom Temp: ");
      display.print(customStartTemp);
      display.println(" C");
      break;
    case MENU_MAX_TEMP:
      display.print("Max Temp Lock: ");
      display.println(maxTempLock);
      break;
    case MENU_UNIT:
      display.println("Temperature Unit");
      display.println(tempUnitIsCelsius ? "> Celsius" : "  Celsius");
      display.println(!tempUnitIsCelsius ? "> Fahrenheit" : "  Fahrenheit");
      break;
    case MENU_LIGHT_SOUND:
      display.println(lightOn ? "> Light: ON" : "> Light: OFF");
      display.println(soundOn ? "> Sound: ON" : "> Sound: OFF");
      break;
    case MENU_IDLE_OFF:
      display.println(idleTimeoutOption == 0 ? "> 15 Min" : "  15 Min");
      display.println(idleTimeoutOption == 1 ? "> 30 Min" : "  30 Min");
      display.println(idleTimeoutOption == 2 ? "> 60 Min" : "  60 Min");
      display.println(idleTimeoutOption == 3 ? "> Always On" : "  Always On");
      break;
    case MENU_IR_ALARM:
      display.print("Heater 1 Alarm: ");
      display.println(irAlarm1);
      display.print("Heater 2 Alarm: ");
      display.println(irAlarm2);
      break;
    case MENU_ABOUT:
      display.println("Contact:");
      display.println("support@example.com");
      break;

    case MENU_MANUAL_TEST:
        display.println(subMenuIndex == 0 ? "> Relay Test"   : "  Relay Test");
        display.println(subMenuIndex == 1 ? "> Buzzer Test"  : "  Buzzer Test");
        display.println(subMenuIndex == 2 ? "> Buttons Test" : "  Buttons Test");
        display.println(subMenuIndex == 3 ? "> Knob/ADC"     : "  Knob/ADC");
        if (subMenuIndex == 2) {
            display.print("B1:"); display.print(digitalRead(button1_pin)==LOW?"P":"-");
            display.print(" B2:"); display.print(digitalRead(button2_pin)==LOW?"P":"-");
            display.print(" B3:"); display.print(digitalRead(button3_pin)==LOW?"P":"-");
        }
        if (subMenuIndex == 3) {
            adcValue = analogRead(encoderPin);
            display.print("ADC: ");
            display.println(adcValue);
        }
        break;

  }
  display.display();
}

void checkStandby() {
  if (millis() - lastInteractionTime > standbyTimeout) {
    if (currentState != MENU_MAIN && currentState != MENU_STANDBY) {
      currentState = MENU_MAIN;
      subMenuIndex = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  SPI.begin();

  pinMode(heaterRelayPin, OUTPUT);
  digitalWrite(heaterRelayPin, LOW);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  display.clearDisplay();
  display.display();

  pinMode(button1_pin, INPUT);
  pinMode(button2_pin, INPUT);
  pinMode(button3_pin, INPUT);

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
}

void loop() {
  handleInput();
  readTemperature();
  controlHeater();
  updateDisplay();
  checkStandby();
}