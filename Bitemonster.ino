/*
  Bite Monster development board
  VL53L0X ToF sensor
*/

//Includes
#include <fishing.h>
#include <Preferences.h>
#include <U8g2lib.h>
#include <VL53L0X.h>
#include <WiFi.h>

//Enum
enum DISPLAY_SM {
  INIT = 0,
  SET_VOL = 1,
  SET_DIST = 2,
  PREPARE = 3,
  BITEWATCH = 4,
  ALARM = 5,
  FINISH = 6,
  RESERVED = 9
};


//Defines
#define HOLD_PIN 6
#define BUZPIN 9
#define LED_PIN 13
#define I2C_SDA 39
#define I2C_SCL 38
#define BUT_UP 7
#define BUT_PUSH 15
#define BUT_DOWN 16
#define BATV 8

#define PWM_FREQ 2700  //2.7 kHz
#define PWM_CHANNEL 0
#define PWM_RESOLUTION 8  //8 bit
#define FREQ_CHANGE 2000  //3 sec

#define TOF_INSENS_MM 6    //[mm] no screen refresh in this range
#define CPU_LOW_FREQ 80    //80 MHz
#define ARRAY_SIZE 21      //values of lookup tables
#define BATTERY_CAP 6700   //6700 mAh
#define CURRCONS 68.0      //68mA in state BITEWATCH
#define MAXVALIDDIST 8189  //Maximal valid value for ToF sensor
#define IIRCOEFF 0.986
#define MENUVALUES 4  //4 menu variables


//Global variables
float voltages[ARRAY_SIZE] = { 3.27, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79, 3.8, 3.82, 3.84, 3.85, 3.87, 3.91, 3.95, 3.98, 4.02, 4.08, 4.11, 4.15, 4.2 };
float percentages[ARRAY_SIZE] = { 0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0, 90.0, 95.0, 100.0 };
float remaintime[ARRAY_SIZE] = { 0, 4.92, 9.85, 14.77, 19.68, 24.6, 29.52, 34.44, 39.36, 44.28, 49.2, 54.12, 59.04, 63.96, 68.88, 73.8, 78.72, 83.64, 88.56, 93.48, 98.4 };
bool BUT_UP_Pressed = false;
bool BUT_DOWN_Pressed = false;
bool BUT_PUSH_Pressed = false;

//Init services
VL53L0X sensor;
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, I2C_SCL, I2C_SDA);  // ESP32 Thing, HW I2C with pin remapping
Preferences preferences;


void PWM_config() {
  ledcAttachPin(BUZPIN, PWM_CHANNEL);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PWM_CHANNEL, 0);
}

void BuzzerSiren(int volume, bool init) {

  static unsigned long startFreq;
  static unsigned int step;
  unsigned int pause[] = { 935, 435, 185, 60, 32 };

  if (init) {
    startFreq = millis();
    step = 0;
  }

  if (millis() - startFreq > FREQ_CHANGE) {
    if (step < 4) {
      step++;
      startFreq = millis();
    }
  }
  ledcWrite(PWM_CHANNEL, volume);
  digitalWrite(LED_PIN, HIGH);
  delay(64);
  ledcWrite(PWM_CHANNEL, 0);
  digitalWrite(LED_PIN, LOW);
  delay(pause[step]);
}

int BatteryPercent(float actualVoltage) {
  int i = 0;
  float actualPercent;

  if (actualVoltage > voltages[ARRAY_SIZE - 1]) {
    return 100;
  }
  if (actualVoltage < voltages[0]) {
    return 0;
  }
  while (actualVoltage > voltages[i]) {
    i++;
  }
  actualPercent = floorf(percentages[i - 1] + 5.0 * (voltages[i] - actualVoltage) / (voltages[i] - voltages[i - 1]));
  return (int)actualPercent;
}

int BatteryRemain(float actualVoltage) {
  int i = 0;
  float actualRemain;

  if (actualVoltage > voltages[ARRAY_SIZE - 1]) {
    return remaintime[ARRAY_SIZE - 1];
  }
  if (actualVoltage < voltages[0]) {
    return 0;
  }
  while (actualVoltage > voltages[i]) {
    i++;
  }
  actualRemain = floorf(remaintime[i - 1] + 4.92 * (voltages[i] - actualVoltage) / (voltages[i] - voltages[i - 1]));
  return (int)actualRemain;
}


bool CheckRange(int inputDistance, int inputSetup) {
  bool retValue;

  if (((inputDistance - inputSetup) > 20) && ((inputDistance + inputSetup) < 200)) {
    return true;
  } else {
    return false;
  }
}

int EEPROMHandler(int mode, byte dataToStore) {
  short storedValue;
  bool successWrite;

  switch (mode) {
    case 1:
      storedValue = preferences.getChar("volume", -1);
      return storedValue;
      break;
    case 2:
      storedValue = preferences.getChar("displacement", -1);
      return storedValue;
      break;
    case 3:
      storedValue = preferences.putChar("volume", dataToStore);
      return storedValue;
      break;
    case 4:
      storedValue = preferences.putChar("displacement", dataToStore);
      return storedValue;
      break;
    default:
      break;
  }
}

int DrawSettings(int val1, int val2, int val3, int val4, DISPLAY_SM mode) {
  static int setValue;
  static int readoutValue;
  static bool newPageNeeded = true;
  static bool readoutNeeded = true;
  int values[MENUVALUES] = { val1, val2, val3, val4 };
  int coord_x[MENUVALUES] = { 0, 0, 90, 90 };
  int coord_y[MENUVALUES] = { 25, 62, 25, 62 };
  static char plotString[3];
  static int actualPos;

  if (readoutNeeded) {
    if (SET_VOL == mode) {
      readoutValue = EEPROMHandler(1, 0);
    }
    if (SET_DIST == mode) {
      readoutValue = EEPROMHandler(2, 0);
    }
    if (readoutValue > -1) {
      setValue = readoutValue;
    }
    readoutNeeded = false;
  }

  if (newPageNeeded) {
    BUT_PUSH_Pressed = false;
    BUT_UP_Pressed = false;
    BUT_DOWN_Pressed = false;
    u8g2.clearBuffer();
    for (int i = 0; i < MENUVALUES; i++) {
      if (setValue == values[i]) {
        u8g2.setDrawColor(0);
        actualPos = i;
      }
      sprintf(plotString, "%02i", values[i]);
      u8g2.drawStr(coord_x[i], coord_y[i], plotString);
      u8g2.setDrawColor(1);
    }
    u8g2.sendBuffer();
    newPageNeeded = false;
  }

  if (BUT_UP_Pressed) {
    setValue = values[++actualPos];
    if (actualPos > 3) {
      actualPos = 0;
      setValue = values[actualPos];
    }
    newPageNeeded = true;
    delay(200);
  }

  if (BUT_DOWN_Pressed) {
    setValue = values[--actualPos];
    if (actualPos < 0) {
      actualPos = 3;
      setValue = values[actualPos];
    }
    newPageNeeded = true;
    delay(200);
  }

  if (BUT_PUSH_Pressed) {
    if (setValue != readoutValue) {
      if (SET_VOL == mode) {
        EEPROMHandler(3, setValue);
      }
      if (SET_DIST == mode) {
        EEPROMHandler(4, setValue);
      }
    }
    newPageNeeded = true;
    readoutNeeded = true;
    BUT_PUSH_Pressed = false;
    delay(400);
    return (setValue);
  }
  return (-1);
}

void DrawMainPage(int actualDistance, int actualPercent, bool distanceColor, int actRemTime) {
  static int lastDrawnDistance;
  char distanceStr[4];
  char remainStr[6];
  char batteryStr[5];

  if (actualDistance < MAXVALIDDIST) {
    dtostrf(actualDistance, 3, 0, distanceStr);
  } else {
    strcpy(distanceStr, "XXX");
  }
  dtostrf(actRemTime, 3, 1, remainStr);
  dtostrf(actualPercent, 3, 0, batteryStr);
  strcat(remainStr, "h");
  strcat(batteryStr, "%");
  if ((abs(lastDrawnDistance - actualDistance)) < TOF_INSENS_MM) {
    return;

  } else {
    u8g2.clearBuffer();
    u8g2.setDrawColor(1);
    u8g2.setFont(u8g2_font_6x13B_tf);  // choose a suitable font
    u8g2.drawStr(10, 10, remainStr);
    u8g2.drawUTF8(100, 10, batteryStr);
    if (distanceColor) {
      u8g2.setDrawColor(1);
    } else {
      u8g2.setDrawColor(0);
    }
    u8g2.setFont(u8g2_font_logisoso38_tf);  // choose a suitable font
    u8g2.drawStr(25, 60, distanceStr);
    u8g2.sendBuffer();
    lastDrawnDistance = actualDistance;
  }
}

float BatteryVoltage() {
  int batADC;
  float batteryVolt;
  static bool firstRun = true;
  static float lastVal;

  for (int i = 0; i < 63; i++) {
    batADC = batADC + analogRead(BATV);
  }
  batADC = batADC >> 6;
  batteryVolt = 1.04 * batADC / 1000;
  if (!firstRun) {
    batteryVolt = lastVal * IIRCOEFF + batteryVolt * (1 - IIRCOEFF);  //IIR filter
    lastVal = batteryVolt;
  } else {
    lastVal = batteryVolt;
    firstRun = false;
  }
  return (batteryVolt);
}

void IRAM_ATTR IsrUp() {
  BUT_UP_Pressed = true;
}

void IRAM_ATTR IsrDown() {
  BUT_DOWN_Pressed = true;
}

void IRAM_ATTR IsrPush() {
  BUT_PUSH_Pressed = true;
}

void setup() {
  pinMode(HOLD_PIN, OUTPUT);
  digitalWrite(HOLD_PIN, HIGH);
  Serial.begin(115200);
  Serial.println();
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUT_PUSH, INPUT_PULLDOWN);
  pinMode(BUT_DOWN, INPUT_PULLDOWN);
  pinMode(BUT_UP, INPUT_PULLDOWN);
  attachInterrupt(BUT_UP, IsrUp, RISING);
  attachInterrupt(BUT_DOWN, IsrDown, RISING);
  attachInterrupt(BUT_PUSH, IsrPush, RISING);
  while (digitalRead(BUT_PUSH)) {
    digitalWrite(LED_PIN, HIGH);
  }
  digitalWrite(LED_PIN, LOW);
  pinMode(BATV, INPUT);
  preferences.begin("bitemonster", false);
  setCpuFrequencyMhz(CPU_LOW_FREQ);
  delay(100);
  btStop();
  delay(100);
  WiFi.mode(WIFI_OFF);
  delay(100);
  PWM_config();
  delay(100);
  u8g2.setI2CAddress(0x78);
  u8g2.begin();
  delay(100);
  //u8g2.setContrast(255);
  u8g2.setFont(u8g2_font_logisoso24_tf);  // choose a suitable font
  delay(100);
  sensor.init();
  delay(100);
}

void loop() {
  static char distString[5];
  int batAdc;
  float batVolt;
  static DISPLAY_SM displayBox = INIT;
  int distance;
  static int setupDistance;
  static int setupVolume;
  int batteryP;
  int remainingTime;
  bool inRange;
  static bool firstRun = true;
  static int settledDistance;
  static bool isBiteIndicated = false;
  static bool firstBite = true;

  switch (displayBox) {
    case INIT:
      {
        u8g2.clearBuffer();
        u8g2.drawXBM(0, 10, logo_width, logo_height, logo_bits);
        u8g2.sendBuffer();
        delay(3000);
        displayBox = SET_VOL;
        break;
      }
    case SET_VOL:
      {
        setupVolume = DrawSettings(0, 33, 66, 99, SET_VOL);
        if (setupVolume > -1) {
          setupVolume = setupVolume / 11 * 14;
          displayBox = SET_DIST;
          BUT_PUSH_Pressed = false;
          delay(400);
        }
        break;
      }
    case SET_DIST:
      {
        setupDistance = DrawSettings(20, 30, 40, 50, SET_DIST);
        if (setupDistance > -1) {
          displayBox = PREPARE;
          BUT_PUSH_Pressed = false;
          delay(400);
        }
        break;
      }
    case PREPARE:
      {
        distance = sensor.readRangeSingleMillimeters();
        inRange = CheckRange(distance, setupDistance);
        batVolt = BatteryVoltage();
        batteryP = BatteryPercent(batVolt);
        remainingTime = BatteryRemain(batVolt);
        DrawMainPage(distance, batteryP, inRange, remainingTime);
        if (BUT_PUSH_Pressed && inRange) {
          settledDistance = distance;
          displayBox = BITEWATCH;
          BUT_PUSH_Pressed = false;
        } else {
          delay(1000);
        }
        break;
      }
    case BITEWATCH:
      {
        if (firstRun) {
          u8g2.clearDisplay();
          firstRun = false;
        }
        distance = sensor.readRangeSingleMillimeters();
        if (abs(distance - settledDistance) > setupDistance) {
          displayBox = ALARM;
        }
        break;
      }
    case ALARM:
      {
        BuzzerSiren(setupVolume, firstBite);
        if (firstBite) {
          u8g2.clearBuffer();
          u8g2.drawXBM(8, 0, fish_width, fish_height, fish_bits);
          u8g2.sendBuffer();
          firstBite = false;
        }
        distance = sensor.readRangeSingleMillimeters();
        if (abs(distance - settledDistance) < setupDistance) {
          ledcWrite(PWM_CHANNEL, 0);
          digitalWrite(LED_PIN, LOW);
          u8g2.clearDisplay();
          firstBite = true;
          displayBox = BITEWATCH;
        }
        if (distance > MAXVALIDDIST) {
          displayBox = FINISH;
        }
        break;
      }
    case FINISH:
      {
        u8g2.clearDisplay();
        ledcWrite(PWM_CHANNEL, 0);
        digitalWrite(LED_PIN, LOW);
        digitalWrite(HOLD_PIN, LOW);
        break;
      }
  }
}