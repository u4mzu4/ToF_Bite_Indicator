/*
  M5StickC Plus development board (https://docs.m5stack.com/en/core/m5stickc_plus)
  VL53L0X ToF sensor
*/

//Includes
#include <EEPROM.h>
#include <fishing.h>
#include <I2C_AXP192.h>
#include <SPIFFS.h>
#include <TFT_eSPI.h>
#include <VL53L0X.h>
#include <WiFi.h>

//Enum
enum DISPLAY_SM {
  INIT        = 0,
  SETTINGS    = 1,
  PREPARE     = 2,
  BITEWATCH   = 3,
  ALARM       = 4,
  FINISH      = 5,
  RSRVD       = 9
};

//Defines
#define BUZZER_PIN      2
#define I2C_SDA_PIN1    21
#define I2C_SDA_PIN2    32
#define I2C_SDA_PIN3    0
#define I2C_SCL_PIN1    22
#define I2C_SCL_PIN2    33
#define I2C_SCL_PIN3    26
#define BUTTON_A_PIN    37
#define BUTTON_B_PIN    39
#define LED_PIN         10
#define FONT_60         "Logisoso60"
#define FONT_36         "Logisoso36"
#define PWM_FREQ1       4000  //4 kHz
#define PWM_FREQ2       7000  //7 kHz
#define PWM_CHANNEL     0
#define PWM_RESOLUTION  10    //10 bit
#define TOF_INSENS_MM   6     //[mm] no screen refresh in this range
#define CPU_LOW_FREQ    40    //40 MHz
#define ARRAY_SIZE      21    //values of lookup tables
#define BRIGHTHNESS     2800  //2.8V LED forward voltage
#define TFT_VOLT        3000  //3V for TFT driver IC
#define FREQ_CHANGE     500   //2Hz
#define EEPROM_SIZE     2     //2 bytes for validity check
#define BATTERY_CAP     7200.0  //120mAh = 7200 mAmin
#define CURRCONS        25.0    //25mA in state BITEWATCH

//Global variables
float voltages[ARRAY_SIZE] = {3.27, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79, 3.8, 3.82, 3.84, 3.85, 3.87, 3.91, 3.95, 3.98, 4.02, 4.08, 4.11, 4.15, 4.2};
float percentages[ARRAY_SIZE] = {0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0, 90.0, 95.0, 100.0};


//Init services
I2C_AXP192 axp192(I2C_AXP192_DEFAULT_ADDRESS, Wire1);
TFT_eSPI tft = TFT_eSPI();
VL53L0X sensor;

//Functions
void DrawMainPage(int actualDistance, int actualPercent, bool distanceColor, int actRemTime)
{
  static int lastDrawnDistance;
  static int lastDrawnPercent;

  if ((abs(lastDrawnDistance - actualDistance)) < TOF_INSENS_MM)
  {
    return;
  }
  else
  {
    tft.fillScreen(TFT_BLACK);
    tft.loadFont(FONT_36);
    tft.drawString(String(actRemTime) + " min", 20, 5);
    tft.drawString(String(actualPercent) + " %", 180, 5);
    tft.loadFont(FONT_60);
    if (distanceColor)
    {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
    }
    else
    {
      tft.setTextColor(TFT_RED, TFT_BLACK);
    }
    tft.drawString(String(actualDistance) + " mm", 20, 40);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.unloadFont();
    tft.unloadFont();
    lastDrawnDistance = actualDistance;
  }
}

int DrawSettings()
{
  static int setValue = 20;
  int readoutValue;
  static bool newPageNeeded = true;
  static bool readoutNeeded = true;

  if (readoutNeeded)
  {
    readoutValue = EEPROMHandler(0, 0);
    if (readoutValue > 0)
    {
      setValue = readoutValue;
    }
    readoutNeeded = false;
  }

  if (newPageNeeded)
  {
    tft.fillScreen(TFT_BLACK);
    if (setValue == 20)
    {
      tft.setTextColor(TFT_BLACK, TFT_WHITE);
    }
    tft.drawString("20", 20, 5);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    if (setValue == 30)
    {
      tft.setTextColor(TFT_BLACK, TFT_WHITE);
    }
    tft.drawString("30", 20, 70);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    if (setValue == 40)
    {
      tft.setTextColor(TFT_BLACK, TFT_WHITE);
    }
    tft.drawString("40", 180, 5);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    if (setValue == 50)
    {
      tft.setTextColor(TFT_BLACK, TFT_WHITE);
    }
    tft.drawString("50", 180, 70);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    newPageNeeded = false;
  }

  if (!digitalRead(BUTTON_A_PIN))
  {
    setValue += 10;
    if (setValue > 50)
    {
      setValue = 20;
    }
    newPageNeeded = true;
  }
  if (!digitalRead(BUTTON_B_PIN))
  {
    EEPROMHandler(1, setValue);
    return (setValue);
  }
  else
  {
    return (0);
  }
}

int BatteryPercent(float actualVoltage)
{
  int i = 0;
  float actualPercent;

  if (actualVoltage > voltages[ARRAY_SIZE - 1])
  {
    return 100;
  }
  if (actualVoltage < voltages[0])
  {
    return 0;
  }
  while (actualVoltage > voltages[i])
  {
    i++;
  }
  actualPercent = floorf(percentages[i - 1] + 5.0 * (voltages[i] - actualVoltage) / (voltages[i] - voltages[i - 1]));
  return (int)actualPercent;
}

void PWM_config()
{
  ledcAttachPin(BUZZER_PIN, PWM_CHANNEL);
  ledcSetup(PWM_CHANNEL, PWM_FREQ1, PWM_RESOLUTION);
  ledcWrite(PWM_CHANNEL, 0);
}

void Display_config()
{
  tft.begin();
  delay(100);
  tft.setRotation(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillScreen(TFT_BLACK);
}

bool CheckRange(int inputDistance, int inputSetup)
{
  bool retValue;

  if (((inputDistance - inputSetup) > 20) && ((inputDistance + inputSetup) < 200))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void BuzzerSiren()
{
  static unsigned long startFreq = millis();
  static bool buzzerIsOn = false;
  static int actualFreq = PWM_FREQ1;

  if (millis() - startFreq > FREQ_CHANGE)
  {
    if (actualFreq == PWM_FREQ1)
    {
      digitalWrite(LED_PIN, LOW);
      ledcWriteTone(PWM_CHANNEL, PWM_FREQ2);
      actualFreq = PWM_FREQ2;
      startFreq = millis();
    }
    else
    {
      digitalWrite(LED_PIN, HIGH);
      ledcWriteTone(PWM_CHANNEL, PWM_FREQ1);
      actualFreq = PWM_FREQ1;
      startFreq = millis();
    }
  }
}

int EEPROMHandler(bool writeMode, byte dataToStore)
{
  byte storedValue[EEPROM_SIZE];
  bool dataIsValid;

  if (!writeMode)
  {
    for (int i = 0; i < EEPROM_SIZE; i++)
    {
      storedValue[i] = EEPROM.read(i);
    }
    if (storedValue[0] ^ storedValue[1] == 0)
    {
      dataIsValid = true;
      return storedValue[0];
    }
    else
    {
      return -1;
    }
  }
  else
  {
    EEPROM.write(0, dataToStore);
    EEPROM.write(1, ~dataToStore);
    EEPROM.commit();
    return 255;
  }
}

int EstimateRemTime (int batteryStatus)
{
  int estRemTime;
  float actualCapacity;
  
  actualCapacity = (float)(batteryStatus) * 0.01 * BATTERY_CAP;
  estRemTime = round(actualCapacity/CURRCONS);
  return estRemTime;
}



void setup() {
  pinMode(BUTTON_A_PIN, INPUT);
  pinMode(BUTTON_B_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  setCpuFrequencyMhz(CPU_LOW_FREQ);
  delay(100);
  Serial.begin(115200);
  delay(100);
  btStop();
  delay(100);
  WiFi.mode(WIFI_OFF);
  delay(100);
  EEPROM.begin(EEPROM_SIZE);
  delay(100);
  Wire1.begin(I2C_SDA_PIN1, I2C_SCL_PIN1);
  delay(100);
  I2C_AXP192_InitDef initDef = {
    .EXTEN  = true,
    .BACKUP = true,
    .DCDC1  = 3300,
    .DCDC2  = 0,
    .DCDC3  = 0,
    .LDO2   = BRIGHTHNESS,
    .LDO3   = TFT_VOLT,
    .GPIO0  = -1,
    .GPIO1  = -1,
    .GPIO2  = -1,
    .GPIO3  = -1,
    .GPIO4  = -1,
  };
  axp192.begin(initDef);
  delay(100);
  Wire.begin(I2C_SDA_PIN2, I2C_SCL_PIN2);
  delay(100);
  sensor.init();
  delay(100);
  SPIFFS.begin();
  delay(100);
  Display_config();
  delay(100);
  PWM_config();
  delay(100);
}


void loop() {
  int distance;
  static int setupDistance;
  float batteryVoltage;
  float currentConsumption;
  int batteryP;
  int remainingTime;
  static DISPLAY_SM displayBox = INIT;
  bool inRange;
  static int brightness = BRIGHTHNESS;
  static bool firstRun = true;
  static int settledDistance;
  static bool isBiteIndicated = false;
  static bool firstBite = true;

  switch (displayBox)
  {
    case INIT:
      {
        /*
          spr1.drawXBitmap(0, 0, logo_bits, logo_width, logo_height, TFT_CYAN);
          delay(3000);
        */
        displayBox = SETTINGS;
        break;
      }
    case SETTINGS:
      {
        tft.loadFont(FONT_60);
        setupDistance = DrawSettings();
        if (setupDistance > 0)
        {
          tft.unloadFont();
          displayBox = PREPARE;
        }
        break;
      }
    case PREPARE:
      {
        distance = sensor.readRangeSingleMillimeters();
        inRange = CheckRange(distance, setupDistance);
        batteryVoltage = (float)axp192.getBatteryVoltage() * 0.001;
        batteryP = BatteryPercent(batteryVoltage);
        remainingTime = EstimateRemTime(batteryP);
        DrawMainPage(distance, batteryP, inRange, remainingTime);
        if (!digitalRead(BUTTON_A_PIN) && inRange)
        {
          settledDistance = distance;
          displayBox = BITEWATCH;
        }
        else
        {
          delay(1000);
        }
        break;
      }
    case BITEWATCH:
      {
        if (firstRun)
        {
          axp192.setLDO2(0);
          axp192.setLDO3(0);
          sensor.setTimeout(500);
          sensor.startContinuous();
          firstRun = false;
        }
        distance = sensor.readRangeSingleMillimeters();
        if (abs(distance - settledDistance) > setupDistance)
        {
          displayBox = ALARM;
        }
        break;

      }
    case ALARM:
      {
        BuzzerSiren();
        if (firstBite)
        {
          axp192.setLDO3(TFT_VOLT);
          axp192.setLDO2(BRIGHTHNESS);
          Display_config();
          tft.drawXBitmap(0, 0, fish_bits, fish_width, fish_height, TFT_WHITE);
          firstBite = false;
        }
        break;
      }
  }
}
