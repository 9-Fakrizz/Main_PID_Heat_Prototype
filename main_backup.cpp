#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <esp_log.h> 
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <PID_v1.h>
#include <Adafruit_MLX90614.h>


static const char *TAG = "MAIN";

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===== IR temperature (GY-906 / MLX90614) =====
Adafruit_MLX90614 irSensor = Adafruit_MLX90614();

// ===== MAX31855 =====
#define MAXDO   12
#define MAXCLK  14
#define MAXCS1  18
#define MAXCS2  19

// ===== Solid State Pins =====

#define heaterRelayPin = 2;
#define heaterRelayPin2 = 15;
#define heaterRelayPin3 = 13;


// // Create two thermocouple objects
// Adafruit_MAX31855 thermocouple1(MAXCLK, MAXCS1, MAXDO);
// Adafruit_MAX31855 thermocouple2(MAXCLK, MAXCS2, MAXDO);
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


const int button1_pin = 25; // Menu up / Back (hold)
const int button2_pin = 26; // Menu down
const int button3_pin = 33; // Select
const int encoderPin = 32;  // Analog input for rotary encoder simulation (placeholder)

const int buzzerPin = 27;  
int adcValue = 0;
bool relayState;
bool relayState2;
bool relayState3;

unsigned long lastInteractionTime = 0;
const unsigned long standbyTimeout = 10000;

float targetTemp1 = 0;
float targetTemp2 = 0;

float currentTemp1 = 0;
float currentTemp2 = 0;
float currentTemp3 = 0;
float irTemp1 = 0;
float irTemp2 = 0;
float thermoTemp = 0;
bool tempUnitIsCelsius = true;

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
  MENU_MANUAL_TEST,
  RELAY_TEST
};

MenuState currentState = MENU_STANDBY;

float startTemps[] = {0, 260, 287, 315, 343, 371, 398, 426};
float customStartTemp = 0;
float maxTempLock = 537;
int idleTimeoutOption = 0; // 0=15m, 1=30m, 2=60m, 3=Always On
int idle_timeout_arr[3] = {10, 15, 30};
int idle_timeout = 10;
bool idle_always = false;
bool lightOn = false;
bool soundOn = true;
int irAlarm1 = 0;
int irAlarm2 = 0;

void buzzerOn(){
  if(soundOn){
    digitalWrite(buzzerPin, HIGH);
    delay(500);
    digitalWrite(buzzerPin, LOW);
  }
}

void checkUnit() {
  if (!tempUnitIsCelsius) { // convert to Fahrenheit
    currentTemp1 = currentTemp1 * 9.0 / 5.0 + 32.0;
    currentTemp2 = currentTemp2 * 9.0 / 5.0 + 32.0;
    currentTemp3 = currentTemp3 * 9.0 / 5.0 + 32.0;
    irTemp1      = irTemp1      * 9.0 / 5.0 + 32.0;
    irTemp2      = irTemp2      * 9.0 / 5.0 + 32.0;
    thermoTemp   = thermoTemp   * 9.0 / 5.0 + 32.0;
  }
}

void selectMenuOption() {
  switch (currentState) {
    case MENU_MAIN:
      if (menuIndex <= 6) {
        currentState = (MenuState)(MENU_START_TEMP + menuIndex);
      } else if (menuIndex == 7) {
        currentState = MENU_MANUAL_TEST;
      }
      ESP_LOGI(TAG,"Current State : %s ",MenuStateToStr(currentState) );
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
     switch (subMenuIndex) {
        case 0: // Celsius
          break;
        case 1: // Farenhien
          break;
      }
      break;

    case MENU_LIGHT_SOUND:
      if (subMenuIndex % 2 == 0) lightOn = !lightOn;
      else soundOn = !soundOn;
      break;

    case MENU_IDLE_OFF:
      idleTimeoutOption = subMenuIndex;
      switch (subMenuIndex) {
        case 0: // 10min
          break;
        case 1: // 15min
          break;
        case 2: // 30min
          break;
        case 3: // always on
          break;
      }
      break;

    case MENU_IR_ALARM:
      switch (subMenuIndex) {
        case 0: //IR 1
          break;
        case 1: //IR 2
          break;
      }
      break;

    case MENU_MANUAL_TEST:
      switch (subMenuIndex) {
        case 0: // Relay
          break;
        case 1: // Buzzer
          break;
        case 2: // Buttons -> display only
          break;
        case 3: // Knob/ADC -> display only
          break;
      }
      break;
    
    case RELAY_TEST:
      switch (subMenuIndex) {
        case 0: // Relay
          relayState = !relayState;
          digitalWrite(SSR1_PIN, relayState);
          break;
        case 1: // Relay 2
          relayState2 = !relayState2;
          digitalWrite(SSR2_PIN, relayState2);
          break;
        case 2: // Relay 3
          relayState3 = !relayState3;
          // digitalWrite(heaterRelayPin3, relayState3);
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

  // Interaction Check
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
        else if (currentState == MENU_MAIN) {
          currentState = MENU_STANDBY;
          subMenuIndex = 0;
          lastInteractionTime = millis();
          lastButtonPress = millis();
          return;
        }
      }
    }
    // Short press behavior
    if (currentState == MENU_MAIN) {  // Main menu 8 option
      menuIndex = (menuIndex - 1 + 8) % 8;

    } else if(currentState == MENU_MANUAL_TEST) {
      subMenuIndex = (subMenuIndex-1+4)%4;
    }
    else if(currentState == MENU_LIGHT_SOUND) {
      subMenuIndex = (subMenuIndex-1+3)%3;
    }
    else if(currentState == MENU_IR_ALARM) {
      subMenuIndex = (subMenuIndex-1+3)%3;
    }
    else if(currentState == RELAY_TEST) {
      subMenuIndex = (subMenuIndex-1+3)%3;
    }
    else if(currentState == MENU_UNIT) {
      subMenuIndex = (subMenuIndex-1+2)%2;
    }
    else if(currentState == MENU_IDLE_OFF) { 
      subMenuIndex = (subMenuIndex-1+2)%5;
      idleTimeoutOption = subMenuIndex;
    }
    lastInteractionTime = millis();
    lastButtonPress = millis();
  }

  if (digitalRead(button2_pin) == LOW) {
    if (currentState == MENU_MAIN) {
      menuIndex = (menuIndex + 1) % 8;
    } else if(currentState == MENU_MANUAL_TEST){
      subMenuIndex = (subMenuIndex+1)%4;
    }
    else if(currentState == MENU_LIGHT_SOUND) {
      subMenuIndex = (subMenuIndex+1)%3;
    }
    else if(currentState == MENU_IR_ALARM) {
      subMenuIndex = (subMenuIndex+1)%3;
    }
    else if(currentState == RELAY_TEST) {
      subMenuIndex = (subMenuIndex+1)%3;
    }
    else if(currentState == MENU_UNIT) {
      subMenuIndex = (subMenuIndex+1)%2;
    }
    else if(currentState == MENU_IDLE_OFF) { 
      subMenuIndex = (subMenuIndex+1)%5;
      idleTimeoutOption = subMenuIndex;
    }
    lastInteractionTime = millis();
    lastButtonPress = millis();
  }

  // confirm button
  if (digitalRead(button3_pin) == LOW) {
    delay(200);
    ESP_LOGI(TAG,"(handle_input)Current State : %s ",MenuStateToStr(currentState) );
    // main
    if(currentState == MENU_MAIN){
      //menu irAlarm
      if(menuIndex == 4){
        currentState = MENU_IR_ALARM;
        return;
      }
      //menu unit
      else if(menuIndex == 5){
        currentState = MENU_UNIT;
        return;
      }
    }

    // idle off
    if(currentState == MENU_IDLE_OFF){
      if(subMenuIndex == 0){
        idle_timeout = idle_timeout_arr[0];
        Serial.println("set idle :" + String(idle_timeout));
        display.setCursor(18, 48);
        display.println("Set Idle :" + String(idle_timeout));
        display.display();
        delay(1000);
        return;
      }
      else if(subMenuIndex == 1){
        idle_timeout = idle_timeout_arr[1];
        Serial.println("set idle :" + String(idle_timeout));
        display.setCursor(18, 48);
        display.println("Set Idle :" + String(idle_timeout));
        display.display();
        delay(1000);
        return;
      }
      else if(subMenuIndex == 2){
        idle_timeout = idle_timeout_arr[2];
        Serial.println("set idle :" + String(idle_timeout));
        display.setCursor(18, 48);
        display.println("Set Idle :" + String(idle_timeout));
        display.display();
        delay(1000);
        return;
      }
      else if(subMenuIndex == 3){
        idle_always = !idle_always;
        delay(200);
        Serial.println("set non idle");
        display.setCursor(18, 48);
        display.println("Set Non Idle");
        display.display();
        delay(1000);
        return;
      }
      else if(subMenuIndex == 4){
        currentState = MENU_MAIN;
        Serial.println("confirm idle");
        return;
      }
    }

    // Light and Sound
    if(currentState == MENU_LIGHT_SOUND){
      if(subMenuIndex == 0){
        lightOn = !lightOn;
        delay(200);
        return;
      }
      else if(subMenuIndex == 1){
        soundOn = !soundOn;
        delay(200);
        return;
      }
      if(subMenuIndex == 2){
        currentState = MENU_MAIN;
        return;
      }
    }
    // irAlarm
    if(currentState == MENU_IR_ALARM){
      if(subMenuIndex == 2){
        currentState = MENU_MAIN;
        return;
      }
    }

    // unit
    if(currentState == MENU_UNIT){
      if(subMenuIndex == 0){
        tempUnitIsCelsius = true;
        currentState = MENU_MAIN;
        return;
      }
      else if(subMenuIndex == 1){
        tempUnitIsCelsius = false;
        currentState = MENU_MAIN;
        return;
      }
    }

    // menu test
    if (currentState == MENU_MANUAL_TEST) {
      if(subMenuIndex == 0){
        subMenuIndex = 0; 
        currentState = RELAY_TEST;
      }
      else if(subMenuIndex == 1){
        digitalWrite(buzzerPin, HIGH);
        ESP_LOGI(TAG,"Buzzer On !");
        Serial.println("Buzzer On !");
      }
    }
    selectMenuOption();
    lastInteractionTime = millis();
    lastButtonPress = millis();
  }else{
    digitalWrite(buzzerPin, LOW);
  }
}

double round1(double x)
{
    return round(x * 10.0) / 10;
}


void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  switch (currentState) {
    case MENU_STANDBY: {
      display.clearDisplay();
      // --- Box layout constants ---
      int boxWidth = (display.width() / 3) - 4;  // divide width into 3 columns
      int boxHeight = display.height() / 2; // divide height into 2 rows
      // --- Draw 6 boxes (3 top, 3 bottom) ---
      for (int row = 0; row < 2; row++) {
        for (int col = 0; col < 3; col++) {
          int x = col * boxWidth;
          int y = row * boxHeight;
          display.drawRect(x+12, y, boxWidth, boxHeight, WHITE);
        }
      }
      // --- Draw vertical "STANDBY" on the left ---
      display.setTextSize(1);
      display.setTextColor(WHITE);
      int startX = 0;      // X position for vertical text
      int startY = 5;      // starting Y position
      int lineSpacing = 8; // space between each character
      const char* text = "STANDBY";
      for (int i = 0; text[i] != '\0'; i++) {
        display.setCursor(startX + 2, startY + (i * lineSpacing));
        display.write(text[i]);
      }
      // --- Example content inside boxes ---
      display.setTextSize(1);
      display.setTextColor(WHITE);
      // Top row labels
      display.setCursor(15, 5);
      display.print("Coil1");
      display.setCursor(15, 18);
      display.print(round1(targetTemp1),1);
      display.print(tempUnitIsCelsius ? "C" : "F");

      display.setCursor(54, 5);
      display.print("Coil2");
      display.setCursor(54, 18);
      display.print(round1(targetTemp2),1);
      display.print(tempUnitIsCelsius ? "C" : "F");

      display.setCursor(92, 5);
      display.print("Coil3");
      display.setCursor(92, 18);
      display.print(" ---");
      // display.print(tempUnitIsCelsius ? "C" : "F");

      // Bottom row labels
      display.setCursor(18, 38);
      display.print("Temp1");
      display.setCursor(15, 50);
      display.print(round1(currentTemp1),1);
      display.print(tempUnitIsCelsius ? "C" : "F");

      display.setCursor(57, 38);
      display.print("Temp2");
      display.setCursor(54, 50);
      display.print(round1(currentTemp2),1);
      display.print(tempUnitIsCelsius ? "C" : "F");

      display.setCursor(92, 38);
      display.print("IR1");
      display.setCursor(92, 50);
      display.print(round1(irTemp1),1);
      display.print(tempUnitIsCelsius ? "C" : "F");

      display.display();
      break;
    }

    case MENU_MAIN:
      display.println(menuIndex == 0 ? "> Start Temp" : "  Start Temp");
      display.println(menuIndex == 1 ? "> Max Temp Lock" : "  Max Temp Lock");
      display.println(menuIndex == 2 ? "> Idle Off" : "  Idle Off");
      display.println(menuIndex == 3 ? "> Sound" : "  Sound");
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
      display.println(subMenuIndex == 0 ? "> Celsius" : "  Celsius");
      display.println(subMenuIndex == 1 ? "> Fahrenheit" : "  Fahrenheit");
      break;
    case MENU_LIGHT_SOUND:
      display.print(subMenuIndex == 0 ? "> Light: " : "  Light: ");
      display.println(lightOn ? "ON" : "OFF");
      display.print(subMenuIndex == 1 ? "> Sound: " : "  Sound: ");
      display.println(soundOn ? "ON" : "OFF");
      display.print(subMenuIndex == 2 ? "> Confirm: " : "  Confirm: ");
      break;
    case MENU_IDLE_OFF:
      display.println(idleTimeoutOption == 0 ? "> 10 Min" : "  10 Min");
      display.println(idleTimeoutOption == 1 ? "> 15 Min" : "  15 Min");
      display.println(idleTimeoutOption == 2 ? "> 30 Min" : "  30 Min");
      display.println(idleTimeoutOption == 3 ? "> Always " +String(idle_always) : "  Always " +String(idle_always));
      display.println(idleTimeoutOption == 4 ? "> Confirm" : "  Confirm");
      break;
    case MENU_IR_ALARM:
      lastInteractionTime = millis();
      display.println("Set IR Alarm");
      display.print(subMenuIndex == 0 ? "> IR1 Alarm: " : "  IR1 Alarm: ");
      display.println(irAlarm1);
      display.print(subMenuIndex == 1 ? "> IR2 Alarm: " : "  IR2 Alarm: ");
      display.println(irAlarm2);
      display.print(subMenuIndex == 2 ? "> Confirm: " : "  Confirm: ");
      display.setCursor(5, 40);
      display.print("Use Knob to Adjust.");
      if(subMenuIndex == 0){
        irAlarm1 = analogRead(encoderPin)/10;
      }
      if(subMenuIndex == 1){
        irAlarm2 = analogRead(encoderPin)/10;
      }
      break;
    case MENU_ABOUT:
      lastInteractionTime = millis();
      display.println("Contact:");
      display.println("support@example.com");
      break;

    case MENU_MANUAL_TEST:
        //disable standby counter
        lastInteractionTime = millis();
        display.println(subMenuIndex == 0 ? "> Relay Test"   : "  Relay Test");
        display.println(subMenuIndex == 1 ? "> Buzzer Test"  : "  Buzzer Test");
        display.println(subMenuIndex == 2 ? "> Buttons Test" : "  Buttons Test");
        display.println(subMenuIndex == 3 ? "> Knob/ADC"     : "  Knob/ADC");
        if(subMenuIndex == 1){
          display.print("Press B3 to test");
        }
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
    case RELAY_TEST:
        //disable standby counter
        lastInteractionTime = millis();
        display.println(subMenuIndex == 0 ? "> Relay 1"  : "  Relay 1");
        display.println(subMenuIndex == 1 ? "> Relay 2"  : "  Relay 2");
        display.println(subMenuIndex == 2 ? "> Relay 3"  : "  Relay 3");

  }
  display.display();
}

void checkStandby() {
  if (millis() - lastInteractionTime > standbyTimeout) {
    if (currentState != MENU_STANDBY) {
      currentState = MENU_STANDBY;
      subMenuIndex = 0;
    }
  }
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

void readSensor()
{
    unsigned long now = millis();
    static unsigned long lastRead = 0;

    if (now - lastRead < 100) return;
    lastRead = now;

    // ================= MAX31855 =================
    double t1 = tc1.readCelsius();
    delay(100);
    double t2 = tc2.readCelsius();

    if (isnan(t1)) t1 = temp1_raw;
    if (isnan(t2)) t2 = temp2_raw;

    temp1_raw = t1;
    temp2_raw = t2;

    currentTemp1 = temp1_raw;
    currentTemp2 = temp2_raw;

    // ================= IR (GY-906) =================
    double irObj = irSensor.readObjectTempC();
    // double irAmb = irSensor.readAmbientTempC();

    if (!isnan(irObj)) irTemp1 = irObj;
    // if (!isnan(irAmb)) irTemp2 = irAmb;
}


void setup() {
  Serial.begin(115200);
  SPI.begin();

  esp_log_level_set("*", ESP_LOG_INFO);

  // pinMode(heaterRelayPin, OUTPUT);
  // digitalWrite(heaterRelayPin, LOW);
  // pinMode(heaterRelayPin2, OUTPUT);
  // digitalWrite(heaterRelayPin2, LOW);
  // pinMode(heaterRelayPin3, OUTPUT);
  // digitalWrite(heaterRelayPin3, LOW);

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

  if (!irSensor.begin()) {
      ESP_LOGE(TAG, "MLX90614 not found!");
  }

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
  checkUnit();
  handleInput();
  updateDisplay();
  checkStandby();
  readSensor();
}