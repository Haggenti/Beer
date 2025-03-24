#pragma GCC optimize("O2") // code optimisation controls - "O2" & "O3" code performance, "Os" code size
#include <Arduino.h>
#include <Wire.h>
#include "I2C_LCD.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "TM1637.h"
#include <Blinkenlight.h>
#include <EEPROM.h>
#include "EMAFilter.h"
//  ==================== LCD & DISPLAY SETTINGS ====================
#define LCD_SPACE_SYMBOL 0x20 // Space symbol from LCD ROM (GDM2004D datasheet p.9)
#define LCD_COLS 20
#define LCD_ROWS 4
#define I2C_BUS_SPEED 400000 // I2C bus speed 400000Hz
#define ARROW_LEFT 0x7F
#define ARROW_RIGHT 0x7E

#define BACKLIGHT_PIN 3
#define En_pin 2
#define Rw_pin 1
#define Rs_pin 0
#define D4_pin 4
#define D5_pin 5
#define D6_pin 6
#define D7_pin 7

// ==================== HARDWARE PIN DEFINITIONS ====================
constexpr uint8_t PIN_BTN = 4;
constexpr uint8_t CLK = 5;          // TM1637 CLK
constexpr uint8_t DIO = 6;          // TM1637 DIO
constexpr uint8_t ONE_WIRE_BUS = 7; // Temperature sensor data wire
constexpr uint8_t BUZZER_PIN = 10;
constexpr uint8_t SSR = 13;
constexpr uint8_t SSR_LED = 11;

// ==================== COMMUNICATION SETTINGS ====================
constexpr uint16_t SERIAL_BAUDRATE = 9600;

// ==================== TEMPERATURE SETTINGS ====================
constexpr uint8_t TEMPERATURE_PRECISION = 11;
float offset_temp = 0.0; // Calibration offset

// Temperature control limits
constexpr float MIN_SETPOINT = 10, MAX_SETPOINT = 99;
float Setpoint = 60;
float prev_temp = 0, temperature = 0.0;
float delta = 0, pwm = 0, old_pwm = -1;

// Sampling settings
constexpr uint32_t SAMPLE_PERIOD = 500;
unsigned lastTempRequest = 0;
uint32_t lastSampleTime = 0;
unsigned delayInMillis = 500; // Wait time for temp reading

// ==================== PWM & CONTROL SETTINGS ====================
float OVERSHOOT_X = -0.1;
float NEAR_LIMIT = 0.3;
float FAR_LIMIT = 3.0;
float PWM_FAR = 80.0, PWM_NEAR = 5.0;
uint32_t PWM_PERIOD = 5000;
bool pwmstate = false;
uint32_t onPWM_PERIOD = 0;

// ==================== MENU & UI SETTINGS ====================
volatile uint8_t menu_select = 0;
bool need_save = false;
uint8_t buttonState;
uint8_t encbutton_state;
unsigned long buttontick = 0;
const char *menu1[] = {"MANUAL", "AUTO", "SETTINGS", "MEMORY", NULL};
const char *menu2[] = {"LOAD", "SAVE", "DEFAULTS", "BACK", NULL};
const char *menu3[] = {"START", "BACK", "EXIT", NULL};
#define VERSION 1.0

// ==================== AUTOMATION VARIABLES ====================
volatile byte mash_rests = 2;
volatile byte rest_stp[] = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
volatile byte rest_dur[] = {2, 2, 2, 5, 5, 5, 5, 5, 5, 5};
volatile byte hops_nmbr = 2;
volatile byte hop_time[] = {5, 5, 5, 5, 5};

// ==================== TIMER VARIABLES ====================
uint32_t chronostart = 0, timelapse = 0;
uint8_t h = 0, m = 0, s = 0, old_s, old_m, old_h;
uint32_t startime = 0;
bool timer_active = false, querry_temp = false, error = false;
bool pauseCountdown = false;  // Variable de pause
unsigned long pausedTime = 0; // Temps restant lors de la pause
// ==================== PRESET TEMPERATURE PROFILES ====================
byte mash_mode = 4;
byte preset = 1, preset_max = 10;
const char *pre_name[] = {
    "Min ", "Temp #1", "Temp #2", "Temp #3", "Temp #4",
    "Temp #5", "Temp #6", "Temp #7", "Temp Out", "Boil"};
float pre_setpt[] = {10, 40, 50, 62, 64, 65, 68, 72, 78, 97};

// ==================== STATE VARIABLES ====================
bool emafilter = true;
bool click_prev = true;
float push_time = 0;
byte countdown_active = 0;
bool fineajust = 0;

// ==================== AUTO MODE STATE VARIABLES ====================
bool automode_started = 0;
int actual_step = -1;
bool stepcomplete = 0;
bool temp_reached = 0;
bool chrono_started = 0;
unsigned long step_time_started = 0;
bool pause = 0;
bool confirm = 0;
byte actual_hop = 0;

// ==================== OBJECT INITIALIZATION ====================
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;
I2C_LCD lcd(39);
EMAFilter tempFilter(0.2); // Initial alpha = 0.2
TM1637 tm;
Blinkenlight buzz(BUZZER_PIN);

// ==================== FUNCTION PROTOTYPES ====================
void buttonstate();

// ==================== CUSTOM CHARACTERS ====================
uint8_t delta_char[8] = {
    B00000, B00000, B00100, B01010, B11111, B00000, B00000, B00000};

uint8_t arrowUp[8] = {
    0b00100, 0b01110, 0b10101, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100};

uint8_t arrowDown[8] = {
    0b00100, 0b00100, 0b00100, 0b00100, 0b10101, 0b01110, 0b00100, 0b00000};

// ==================== EEPROM MANAGEMENT ====================
void writeEEPROM()
{
  int address = 0;
  EEPROM.put(address, offset_temp);
  address += sizeof(offset_temp);
  EEPROM.put(address, OVERSHOOT_X);
  address += sizeof(OVERSHOOT_X);
  EEPROM.put(address, NEAR_LIMIT);
  address += sizeof(NEAR_LIMIT);
  EEPROM.put(address, FAR_LIMIT);
  address += sizeof(FAR_LIMIT);
  EEPROM.put(address, PWM_FAR);
  address += sizeof(PWM_FAR);
  EEPROM.put(address, PWM_NEAR);
  address += sizeof(PWM_NEAR);
  EEPROM.put(address, PWM_PERIOD);
  address += sizeof(PWM_PERIOD);
  EEPROM.put(address, emafilter);
}

void readEEPROM()
{
  int address = 0;
  EEPROM.get(address, offset_temp);
  address += sizeof(offset_temp);
  EEPROM.get(address, OVERSHOOT_X);
  address += sizeof(OVERSHOOT_X);
  EEPROM.get(address, NEAR_LIMIT);
  address += sizeof(NEAR_LIMIT);
  EEPROM.get(address, FAR_LIMIT);
  address += sizeof(FAR_LIMIT);
  EEPROM.get(address, PWM_FAR);
  address += sizeof(PWM_FAR);
  EEPROM.get(address, PWM_NEAR);
  address += sizeof(PWM_NEAR);
  EEPROM.get(address, PWM_PERIOD);
  address += sizeof(PWM_PERIOD);
  EEPROM.get(address, emafilter);
}

// ==================== ENCODER MANAGEMENT ====================
constexpr byte pinA = 2;                 // Hardware interrupt pin (digital pin 2)
constexpr byte pinB = 3;                 // Hardware interrupt pin (digital pin 3)
volatile byte aFlag = 0;                 // Indicates rising edge on pinA (encoder detent reached)
volatile byte bFlag = 0;                 // Indicates rising edge on pinB (encoder detent reached in opposite direction)
volatile byte reading = 0;               // Stores direct values from interrupt pins before validating movement
volatile int encPos = 0;                 // Encoder position counter
volatile bool axcel = 1;                 // acceleration activation
volatile unsigned long lastTurnTime = 0; // Temps du dernier changement d'état
volatile int accelFactor = 1;            // Facteur d'accélération

void updateEncoder(int direction, unsigned long currentTime)
{
  unsigned long timeDiff = currentTime - lastTurnTime; // Calcul du temps entre deux impulsions
  unsigned long maxdif = 250;
  if (timeDiff > maxdif)
  {
    timeDiff = maxdif;
  }
  lastTurnTime = currentTime;

  // Ajuste le facteur d’accélération en fonction de la vitesse de rotation
  if (axcel)
  {
    accelFactor = maxdif / timeDiff;
  }
  else
  {
    accelFactor = 1;
  }
  encPos = direction * accelFactor;
}

void print_space(byte sp)
{
  for (size_t i = 0; i < sp; i++)
  {
    lcd.print(F(" "));
  }
}
void print_deg()
{
  lcd.print(F("\xDF"
              "C")); // Print °C symbol
}

void PinA()
{
  unsigned long currentTime = millis(); // Récupère le temps actuel
  reading = PIND & 0xC;

  if (reading == B00001100 && aFlag)
  {
    updateEncoder(1, currentTime);
    bFlag = 0;
    aFlag = 0;
  }
  else if (reading == B00000100)
  {
    bFlag = 1;
  }
}

void PinB()
{
  unsigned long currentTime = millis();
  reading = PIND & 0xC;

  if (reading == B00001100 && bFlag)
  {
    updateEncoder(-1, currentTime);
    bFlag = 0;
    aFlag = 0;
  }
  else if (reading == B00001000)
  {
    aFlag = 1;
  }
}

void factory_rst()
{
  lcd.clear();
  lcd.print(F("Factory Reset"));
  offset_temp = 0.0;
  OVERSHOOT_X = -0.1;
  NEAR_LIMIT = 0.3;
  FAR_LIMIT = 3;
  PWM_FAR = 80;
  PWM_NEAR = 5;
  PWM_PERIOD = 5000;
  emafilter = true;
  delay(1000);
  lcd.print(F("OK!"));
}

void check_counter()
{
  if (abs(delta) < NEAR_LIMIT && timer_active == false)
  {
    timer_active = true;
    chronostart = millis();
    buzz.pattern(3, false);
  }

  if (abs(delta) > NEAR_LIMIT && timer_active == true)
  {
    timer_active = false;
    lcd.setCursor(0, 3);
    lcd.print(F("00:00:00"));
  }
}

void ssr_mgmt()
{
  // Mise à jour de l'affichage PWM uniquement si changement
  if (old_pwm != pwm)
  {
    lcd.setCursor(5, 1);
    print_space(6);
    lcd.setCursor(5, 1);
    // lcd.print(F("PWM: "));
    lcd.print(pwm, 1);
    lcd.print(F("%"));
    old_pwm = pwm;
    onPWM_PERIOD = (pwm * PWM_PERIOD) / 100; // Calcul direct du temps ON
  }

  // Gestion du cycle PWM
  uint32_t currentMillis = millis();

  // Début d'un nouveau cycle
  if (currentMillis - startime >= PWM_PERIOD)
  {
    startime = currentMillis;
    if (pwm > 0)
    {
      digitalWrite(SSR, HIGH);
      digitalWrite(SSR_LED, HIGH);

      pwmstate = true;
    }
  }

  // Fin de la période ON
  if (pwmstate && (currentMillis - startime >= onPWM_PERIOD))
  {
    digitalWrite(SSR, LOW);
    digitalWrite(SSR_LED, LOW);

    pwmstate = false;
  }
}

// ==================== SETPOINT MANAGEMENT ====================
void setpoint_mgmt()
{
  if (encPos != 0 && !automode_started)
  {
    Setpoint = constrain(Setpoint + (0.1 * encPos), MIN_SETPOINT, MAX_SETPOINT);
    encPos = 0;

    lcd.setCursor(5, 0);
    // lcd.print(F("Set: "));
    lcd.print(Setpoint, 1);
    print_deg();
  }

  delta = Setpoint - temperature;
  pwm = (delta <= OVERSHOOT_X)  ? 0
        : (delta <= NEAR_LIMIT) ? PWM_NEAR
        : (delta < FAR_LIMIT)   ? PWM_NEAR + (delta - NEAR_LIMIT) * ((PWM_FAR - PWM_NEAR) / (FAR_LIMIT - NEAR_LIMIT))
                                : PWM_FAR;
}

void disp_preset()
{
  lcd.setCursor(0, 0);
  print_space(16);
  lcd.setCursor(0, 0);
  lcd.print(pre_name[preset]);
  lcd.write(ARROW_RIGHT);
  lcd.print(pre_setpt[preset], 0);
  Setpoint = pre_setpt[preset];
  print_deg();
  encPos = 0;
}

void preset_mgmt()
{
  if (encPos != 0)
  {
    int oldpreset = preset;
    preset = constrain(preset + encPos, 0, preset_max - 1);
    encPos = 0;
    if (oldpreset != preset)
    {
      disp_preset();
    }
  }
}

void restore_disp_man()
{
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print(F("PWM: "));
  lcd.print(pwm, 1);
  lcd.print("%");
  lcd.setCursor(0, 3);
  lcd.print(F("00:00:00"));
  lcd.setCursor(0, 0);
  lcd.print(F("Set: "));
  lcd.print(Setpoint, 1);
  print_deg();
}

void dis_mode()
{
  if (mash_mode == 1)
  {
    restore_disp_man();
    axcel = 1;
    lcd.setCursor(13, 1);
    lcd.print(F("MASH  "));
    lcd.setCursor(0, 2);
    lcd.clearEOL();

    // print_space(16);
  }
  else if (mash_mode == 2)
  {
    axcel = 1;
    lcd.setCursor(0, 0);
    print_space(11);
    lcd.setCursor(13, 1);
    lcd.print(F("BOIL  "));
    lcd.setCursor(0, 2);
    lcd.clearEOL();

    //  print_space(16);
  }
  else if (mash_mode == 3)
  {
    axcel = 0;
    restore_disp_man();
    lcd.setCursor(13, 1);
    lcd.print(F("PRESET"));
    disp_preset();
  }
  else if (mash_mode == 4)
  {
    axcel = 0;
    restore_disp_man();
    lcd.setCursor(0, 0);
    lcd.print(F("Standby... "));
    lcd.setCursor(13, 1);
    lcd.print(F("IDLE   "));
  }
}

void dis_time()
{
  old_s = s;
  old_m = m;
  old_h = h;

  timelapse = (millis() - chronostart) / 1000;
  h = timelapse / 3600;
  m = (timelapse - (h * 3600)) / 60;
  s = timelapse - h * 3600 - m * 60;
  if (old_h != h)
  {
    lcd.setCursor(0, 3);
    if (h < 10)
      lcd.print('0');
    lcd.print(h);
    lcd.print(F(":"));
  }
  if (old_m != m)
  {
    lcd.setCursor(3, 3);
    if (m < 10)
      lcd.print('0');
    lcd.print(m);
    lcd.print(F(":"));
  }

  if (old_s != s)
  {
    lcd.setCursor(6, 3);
    if (s < 10)
      lcd.print('0');
    lcd.print(s);
  }
}

void boil_mgmt()
{

  if (encPos != 0 && mash_mode != 4)
  {

    pwm = pwm + 0.1 * encPos;
    if (pwm > 100.0)
    {
      pwm = 100.0;
    }
    if (pwm < 0)
    {
      pwm = 0;
    }
    encPos = 0;
  }
}

void fine()
{

  if (encPos != 0)
  {
    PWM_NEAR = constrain(PWM_NEAR + (0.1 * encPos), 0, PWM_FAR);
    pwm = PWM_NEAR;
    ssr_mgmt();
    encPos = 0;
  }
}

void read_temp()
{
  // Check if it's time to request new temperature reading
  if (millis() - lastTempRequest >= delayInMillis)
  {
    // First pass: request temperature reading
    if (!querry_temp)
    {
      sensors.requestTemperatures();
      lastTempRequest = millis();
      querry_temp = true;
    }
    // Second pass: get the temperature value
    else
    {
      if (emafilter)
      {
        float rawTemp = sensors.getTempCByIndex(0);
        temperature = tempFilter.update(rawTemp);
      }
      else
        temperature = sensors.getTempCByIndex(0) + offset_temp;

      if (temperature == -127.00)
      {
        error = true;
        pwm = 0;
      }
      if (temperature != -127.00 && error)
      {
        error = false;
      }
      querry_temp = false;
      if (temperature != prev_temp)
      {
        tm.displayFloat(temperature, 1);
        prev_temp = temperature;
        delta = Setpoint - temperature;
      }
    }
  }
}

void rot_man()
{
  buttonstate();

  if (encbutton_state == 1 && fineajust)
  {
    fineajust = 0;
    encbutton_state = 0;
    lcd.setCursor(19, 3);
    print_space(1);
  }

  if (encbutton_state == 1 && !fineajust)
  {
    // single click
    mash_mode++;
    if (mash_mode > 4)
    {
      mash_mode = 1;
    }
    dis_mode();
    encbutton_state = 0;
  }

  if (encbutton_state == 4)
  {
    // long press
    digitalWrite(SSR, LOW);
    digitalWrite(SSR_LED, LOW);

    pwmstate = false;
    menu_select = 0;
    encbutton_state = 0;
    fineajust = 0;
  }
  if (encbutton_state == 2 && mash_mode == 1)
  {
    if (!fineajust)
    {
      lcd.setCursor(19, 3);
      lcd.print(F("F"));
      pwm = PWM_NEAR;
    }
    // keep pressed
    fineajust = 1;
    encbutton_state = 0;
  }
}

byte menu_mode_flex(const char *menu[], const char *title)
{
  lcd.clear();
  lcd.print(title);

  byte menu_index = 0;
  int num_choices = 0;
  int scroll_offset = 0;

  // Calculer le nombre de choix
  while (menu[num_choices] != NULL)
  {
    num_choices++;
  }

  // Afficher les choix visibles
  for (int i = 0; i < LCD_ROWS - 1; i++)
  {
    int menu_item_index = i + scroll_offset;
    if (menu_item_index < num_choices)
    {
      lcd.setCursor(1, i + 1);
      lcd.print(menu[menu_item_index]);
    }
    else
    {
      lcd.setCursor(1, i + 1);
      print_space(1);
    }
  }

  // Afficher le curseur
  lcd.setCursor(0, menu_index + 1 - scroll_offset);
  lcd.write(ARROW_RIGHT);

  while (1)
  {
    buttonstate();
    read_temp();

    if (encbutton_state == 1)
    {
      // Retourner l'index sélectionné
      encbutton_state = 0;
      return menu_index;
    }

    if (encPos < 0)
    {
      encPos = 0;
      if (menu_index < num_choices - 1)
      {
        lcd.setCursor(0, menu_index + 1 - scroll_offset);
        print_space(1);
        menu_index++;

        if (menu_index >= scroll_offset + LCD_ROWS - 1)
        {
          scroll_offset++;
          // Effacer les lignes avant de les réécrire
          for (int i = 0; i < LCD_ROWS - 1; i++)
          {
            lcd.setCursor(0, i + 1);
            lcd.clearEOL();
            // print_space(19);
          }
          for (int i = 0; i < LCD_ROWS - 1; i++)
          {
            int menu_item_index = i + scroll_offset;
            if (menu_item_index < num_choices)
            {
              lcd.setCursor(1, i + 1);
              lcd.print(menu[menu_item_index]);
            }
            else
            {
              lcd.setCursor(1, i + 1);
              print_space(1);
            }
          }
        }

        lcd.setCursor(0, menu_index + 1 - scroll_offset);
        lcd.write(ARROW_RIGHT);
      }
    }

    if (encPos > 0)
    {
      encPos = 0;

      if (menu_index > 0)
      {
        lcd.setCursor(0, menu_index + 1 - scroll_offset);
        print_space(1);
        menu_index--;

        if (menu_index < scroll_offset)
        {
          scroll_offset--;
          // Effacer les lignes avant de les réécrire
          for (int i = 0; i < LCD_ROWS - 1; i++)
          {
            lcd.setCursor(0, i + 1);
            lcd.clearEOL();

            // print_space(19);
          }
          for (int i = 0; i < LCD_ROWS - 1; i++)
          {
            int menu_item_index = i + scroll_offset;
            if (menu_item_index < num_choices)
            {
              lcd.setCursor(1, i + 1);
              lcd.print(menu[menu_item_index]);
            }
            else
            {
              lcd.setCursor(1, i + 1);
              print_space(1);
            }
          }
        }

        lcd.setCursor(0, menu_index + 1 - scroll_offset);
        lcd.write(ARROW_RIGHT);
      }
    }

    // Afficher les flèches de défilement
    if (scroll_offset > 0)
    {
      lcd.setCursor(LCD_COLS - 1, 1); // Haut à droite
      lcd.write(byte(2));             // Flèche vers le haut
    }
    else
    {
      lcd.setCursor(LCD_COLS - 1, 1);
      print_space(1);
    }

    if (scroll_offset + LCD_ROWS - 1 < num_choices)
    {
      lcd.setCursor(LCD_COLS - 1, LCD_ROWS - 1); // Bas à droite
      lcd.write(byte(3));                        // Flèche vers le bas
    }
    else
    {
      lcd.setCursor(LCD_COLS - 1, LCD_ROWS - 1);
      print_space(1);
    }
  }
  encbutton_state = 0;
  return 0;
}

void splash()
{
  lcd.clear();
  lcd.home();
  lcd.println(F("    FuZZy LoGic"));
  lcd.println(F("  Temp. controller"));
  lcd.println(F("   For BIAB RIMS"));
  lcd.print(F("   Version "));
  lcd.print(VERSION);
  while (encbutton_state == 0)
  {
    buttonstate();
  }
  encbutton_state=0;
}

void setup(void)
{
  // Serial.begin(SERIAL_BAUDRATE);
  tm.begin(CLK, DIO, 4);
  tm.setBrightness(5);
  tm.displayClear();
  lcd.config(39, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin, BACKLIGHT_PIN, POSITIVE);
  Wire.begin();
  Wire.setClock(100000);
  lcd.begin(20, 4);
  lcd.createChar(1, delta_char);
  lcd.createChar(2, arrowUp);
  lcd.createChar(3, arrowDown);

  sensors.begin();
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
  sensors.setWaitForConversion(false);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(SSR, OUTPUT);

  pinMode(pinA, INPUT_PULLUP);      // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP);      // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0, PinA, RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1, PinB, RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
  splash();
    // readEEPROM();

}

void manual_mode()
{
  rot_man();
  check_counter();
  if (timer_active)
    dis_time();

  read_temp();
  if (mash_mode == 1)
  {
    if (fineajust)
    {
      fine();
    }
    else
    {
      setpoint_mgmt();
    }
  }
  else if (mash_mode == 2)
  {
    boil_mgmt();
  }
  else if (mash_mode == 3)
  {
    preset_mgmt();
    setpoint_mgmt();
  }
  else if (mash_mode == 4)
  {
    pwm = 0;
    boil_mgmt();
  }

  ssr_mgmt();
}

float modify(const String title, float variable, float min, float max, float inc, const char *unit, const byte decimals)
{
  lcd.setCursor(0, 0);
  lcd.print(title);
  lcd.setCursor(0, 1);
  lcd.print(variable, decimals);
  lcd.print(unit);

  buttonstate();
  while (encbutton_state == 0)
  {
    buttonstate();
    if (encPos > 0 && variable + inc <= max)
      variable += inc;
    if (encPos < 0 && variable - inc >= min)
      variable -= inc;

    if (encPos != 0)
    {
      lcd.setCursor(0, 1);
      print_space(8);
      lcd.setCursor(0, 1);
      lcd.print(variable, decimals);
      lcd.print(unit);
      encPos = 0;
    }
  }
  lcd.clear();
  return variable;
}

float selector(float variable, float min, float max, float inc, const byte decimals, const byte x, const byte y)
{
  buttonstate();
  lcd.setCursor(x, y);
  lcd.print(variable, decimals);
  while (encbutton_state == 0)
  {
    buttonstate();
    if (encPos > 0 && variable + inc <= max)
      variable += inc;
    if (encPos < 0 && variable - inc >= min)
      variable -= inc;

    if (encPos != 0)
    {
      lcd.setCursor(x, y);
      print_space(5);
      lcd.setCursor(x, y);
      lcd.print(variable, decimals);
      encPos = 0;
    }
  }
  return variable;
}

// ==================== SETTINGS AUTO MODE MENU ====================
void set_m()
{
  lcd.blink();
  lcd.clear();
  offset_temp = modify("Temperature offset", offset_temp, -3, 3, 0.1, "\xDF"
                                                                      "C",
                       1);
  lcd.clear();
  PWM_PERIOD = modify("PWM period", PWM_PERIOD, 1000, 8000, 100, " ms", 0);
  lcd.clear();
  emafilter = modify("EMA Filter", emafilter, 0, 1, 1, " (no:0 - yes:1)", 0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Rule 1:"));
  lcd.setCursor(0, 1);
  lcd.print(F("Overshoot limit"));
  lcd.setCursor(0, 2);
  lcd.print(F("if \x01<"));
  lcd.setCursor(0, 3);
  lcd.print(F("PWM=0"));
  lcd.setCursor(0, 2);
  OVERSHOOT_X = selector(OVERSHOOT_X, -1, 3, 0.1, 1, 5, 2);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Rule 2:"));
  lcd.setCursor(0, 1);
  lcd.print(F("Setpoint zone"));
  lcd.setCursor(0, 2);
  lcd.print("if " + String(OVERSHOOT_X) + "<\x01<");
  lcd.print(NEAR_LIMIT, 1);
  lcd.setCursor(0, 3);
  lcd.print(F("PWM="));
  lcd.print(PWM_NEAR, 1);
  NEAR_LIMIT = selector(NEAR_LIMIT, OVERSHOOT_X, 6, 0.1, 1, 11, 2);
  delay(300);
  PWM_NEAR = selector(PWM_NEAR, 0, 50, 0.1, 1, 4, 3);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Rule 3:"));
  lcd.setCursor(0, 1);
  lcd.print(F("Ramp zone"));
  lcd.setCursor(0, 2);
  lcd.print("if " + String(NEAR_LIMIT) + "<\x01<");
  lcd.print(FAR_LIMIT, 0);
  lcd.setCursor(0, 3);
  lcd.print(F("Ramp PWM"));
  FAR_LIMIT = selector(FAR_LIMIT, NEAR_LIMIT, 50, 0.1, 1, 10, 2);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Rule 4:"));
  lcd.setCursor(0, 1);
  lcd.print(F("Far zone"));
  lcd.setCursor(0, 2);
  lcd.print(F("if "));
  lcd.print("\x01>");
  lcd.print(FAR_LIMIT, 1);
  lcd.setCursor(0, 3);
  lcd.print(F("PMW="));
  PWM_FAR = selector(PWM_FAR, PWM_NEAR, 100, 1, 0, 4, 3);

  lcd.noBlink();
  menu_select = 0;
}

void ctdn(int dur, unsigned long &start)
{
  static bool wasPaused = false; // Indique si on était en pause précédemment

  if (pauseCountdown)
  {
    if (!wasPaused)
    {
      pausedTime = (dur * 60) - ((millis() - start) / 1000); // Sauvegarde le temps restant
      wasPaused = true;
    }
    return; // Bloque le décompte tant qu'on est en pause
  }

  if (wasPaused)
  {
    // Ajuste "start" pour reprendre là où on s'était arrêté
    start = millis() - ((dur * 60) - pausedTime) * 1000;
    wasPaused = false;
  }

  old_s = s;
  old_m = m;
  old_h = h;
  timelapse = (dur * 60) - ((millis() - start) / 1000);
  h = timelapse / 3600;
  m = (timelapse - (h * 3600)) / 60;
  s = timelapse - h * 3600 - m * 60;

  if (old_h != h)
  {
    lcd.setCursor(0, 3);
    if (h < 10)
      lcd.print('0');
    lcd.print(h);
    lcd.print(F(":"));
  }
  if (old_m != m)
  {
    lcd.setCursor(3, 3);
    if (m < 10)
      lcd.print('0');
    lcd.print(m);
    lcd.print(F(":"));
  }

  if (old_s != s)
  {
    lcd.setCursor(6, 3);
    if (s < 10)
      lcd.print('0');
    lcd.print(s);
  }
}

void automatic()
{
  lcd.clear();
  bool brek = 0;

  actual_step = 0;
  stepcomplete = 0;
  temp_reached = 0;
  chrono_started = 0;
  step_time_started = 0;
  pause = 0;
  confirm = 0;
  actual_hop = 0;
  automode_started = 1;
  setpoint_mgmt();
  pwm = -1;
  ssr_mgmt();

  while (((actual_step < mash_rests - 1) && encbutton_state != 2) && brek == 0)
  {

    lcd.setCursor(0, 3);
    lcd.print(F("00:00:00"));
    lcd.setCursor(0, 1);
    lcd.print(F("PWM: "));
    lcd.setCursor(19, 3);
    lcd.print(F("F"));
    Setpoint = int(rest_stp[actual_step]);
    if (actual_step < mash_rests - 1)
    {
      lcd.setCursor(0, 0);
      print_space(12);
      lcd.setCursor(0, 0);
      lcd.print(F("Step #"));
      lcd.print(actual_step + 1);
      lcd.write(ARROW_RIGHT);
      lcd.print(Setpoint, 0);
      print_deg();
    }
    else
    {
      lcd.setCursor(0, 0);
      print_space(12);
      lcd.setCursor(0, 0);
      lcd.print(F("Boil "));
      lcd.write(ARROW_RIGHT);
      lcd.print(Setpoint, 0);
      print_deg();
    }
    while (!stepcomplete && brek != 1)
    {
      buttonstate();

      read_temp();
      if (encbutton_state == 4)
      {
        brek = 1;
      }
      if (encbutton_state == 1 && confirm)
      {
        // clear message & confirm action

        lcd.setCursor(0, 2);
        print_space(12);
        confirm = 0;
        encbutton_state = 0;
        pauseCountdown = 0;
        buzz.off();
      }
      if (encbutton_state == 1 && !pause && !confirm)
      {
        pause = !pause;
        encbutton_state = 0;
        pwm = 0;
        ssr_mgmt();
        lcd.setCursor(0, 2);
        lcd.print(F("PWM Paused"));
        lcd.setCursor(19, 3);
        lcd.print(F("C"));
      }

      if (encbutton_state == 1 && pause && !confirm)
      {
        pause = !pause;
        encbutton_state = 0;
        lcd.setCursor(0, 2);
        print_space(14);
        lcd.setCursor(19, 3);
        lcd.print(F("F"));
      }

      if (!pause)
      {
        setpoint_mgmt();
        ssr_mgmt();
      }

      if (chrono_started)
      {

        if (encPos != 0 && pause)
        {
          // ajust time if in pause && timer started
          rest_dur[actual_step] = constrain(rest_dur[actual_step] + encPos, 1, 120);
          encPos = 0;
        }
        // display counter
        ctdn(rest_dur[actual_step], step_time_started);

        if (encPos != 0 && !pause && delta < PWM_NEAR && delta > OVERSHOOT_X)
        {
          // ajust pwm power
          fine();
          encPos = 0;
        }
      }
      if (delta < NEAR_LIMIT && temp_reached == 0)
      {
        temp_reached = 1;
        chrono_started = 1;
        step_time_started = millis();
        if (actual_step == 0)
        {
          lcd.setCursor(0, 2);
          lcd.print(F("Add Malt"));
          pauseCountdown = 1;
          confirm = 1;
          buzz.pattern(3, true);
        }
        if (actual_step == mash_rests - 1)
        {
          lcd.setCursor(0, 2);
          lcd.print(F("Remove malt"));
          pauseCountdown = 1;
          confirm = 1;
          buzz.on();
          buzz.pattern(3, true);
        }
      }
      if (temp_reached && (millis() - step_time_started) > (rest_dur[actual_step] * 60000))
      {
        stepcomplete = 1;
        lcd.setCursor(0, 3);
        print_space(9);
        buzz.pattern(3, false);
      }
      if (actual_step == (mash_rests - 1))
      {
        // on test si on est bien en boil
        if ((rest_dur[actual_step] - m) == hop_time[actual_hop] && s == 0 && actual_hop < hops_nmbr)
        {
          actual_hop++;
          lcd.setCursor(0, 2);
          lcd.print(F("Add hop #"));
          lcd.print(actual_hop);
          confirm = 1;
          buzz.pattern(3, true);
        }
      }
    }
    encbutton_state = 0;
    actual_step++;
  }

  automode_started = 0;
  lcd.clear();
  if (brek == 1)
  {
    pwm = 0;
    ssr_mgmt();
    lcd.clear();
    lcd.print(F("Canceled !"));
  }
  else
  {
    lcd.print(F("Finished !"));
    menu_select = 2;
  }
  delay(2000);
}

///////////////////// MENUS ////////////////////////////////////////

void start_automation_menu()
{
  byte selected_index = menu_mode_flex(menu3, "---| AUTOMATION |---");

  // Utilisez selected_index pour déterminer l'action à effectuer
  switch (selected_index)
  {
  case 0:
    // START AUTOMATION
    lcd.clear();
    lcd.print(F("Start..."));
    delay(1000);
    automatic();
    start_automation_menu();
    break;
  case 1:
    menu_select = 2;
    break;
  case 2:
    menu_select = 0;
    break;

  default:
    break;
  }
}

void auto_mode()
{
  lcd.blink();
  lcd.clear();
  mash_rests = modify("How many steps ?", mash_rests, 2, 10, 1, "", 0);
  String texte;
  for (size_t i = 0; i < mash_rests; i++)
  {
    int ii;
    texte = "Setpoint " + String(i + 1);
    if (i > 0)
    {
      ii = i - 1;
      rest_stp[i] = rest_stp[ii] + 1;
    }
    else
    {
      ii = 0;
    }
    rest_stp[i] = modify(texte, rest_stp[i], (rest_stp[ii] + 1), 99, 1, "\xDF"
                                                                        "C",
                         0);

    texte = "Duration " + String(i + 1);
    rest_dur[i] = modify(texte, rest_dur[i], 1, 120, 1, " min", 0);
  }
  mash_rests++;
  rest_stp[mash_rests - 1] = 15;
  rest_dur[mash_rests - 1] = 30;
  rest_stp[mash_rests - 1] = modify("Boil temp :", rest_stp[mash_rests - 1], 15, 99, 0.5, "\xDF"
                                                                                          "C",
                                    1);
  rest_dur[mash_rests - 1] = modify("Boil time :", rest_dur[mash_rests - 1], 30, 120, 5, " min", 0);

  hops_nmbr = modify("How many hops adds ?", hops_nmbr, 1, 5, 1, "", 0);
  for (size_t i = 0; i < hops_nmbr; i++)
  {
    texte = "Add Hop n" + String(char(223)) + String(i + 1) + " at";
    lcd.setCursor(8, 1);
    lcd.print(F("from boil"));
    byte time_min = 1;
    if (i != 0)
    {
      time_min = 1 + hop_time[i - 1];
      hop_time[i] = 1 + hop_time[i - 1];
    }
    hop_time[i] = modify(texte, hop_time[i], time_min, rest_dur[mash_rests - 1], 1, " min", 0);
  }

  lcd.noBlink();
}

void memory_menu()
{
  byte selected_index = menu_mode_flex(menu2, "---|MEMORY  MENU|---");

  // Utilisez selected_index pour déterminer l'action à effectuer
  switch (selected_index)
  {
  case 0:
    lcd.clear();
    lcd.print(F("Loading"));
    readEEPROM();
    delay(1000);
    menu_select = 0;
    break;
  case 1:
    lcd.clear();
    lcd.print(F("Saving"));
    writeEEPROM();
    delay(1000);
    menu_select = 0;

    break;
  case 2:
    factory_rst();
    menu_select = 0;
    break;
  case 3:
    menu_select = 0;
    break;
  default:
    menu_select = 0;
    break;
  }
}

void main_menu()
{
  byte selected_index = menu_mode_flex(menu1, "---| FUZZY BREW |---");

  // Utilisez selected_index pour déterminer l'action à effectuer
  switch (selected_index)
  {
  case 0:
    menu_select = 1;
    restore_disp_man();
    dis_mode();
    break;
  case 1:
    menu_select = 2;
    break;
  case 2:
    menu_select = 3;
    break;
  case 3:
    menu_select = 4;
    memory_menu();
    break;

  default:
    break;
  }
}

void loop()
{

  switch (menu_select)
  {
  case 0:
    main_menu();
    break;
  case 1:
    manual_mode();
    break;
  case 2:
    auto_mode();
    start_automation_menu();
    break;
  case 3:
    set_m();
    break;
  default:
    break;
  }
}

void buttonstate()
{
  bool state = digitalRead(PIN_BTN);
  encbutton_state = 0;
  float elapsed = millis() - buttontick;
  buzz.update();
  if (click_prev && !state)
  {
    // detect fall
    buttontick = millis();
  }

  if (!click_prev && !state && elapsed > 2500)
  {
    // when continuous push
    encbutton_state = 4;
  }

  if (!click_prev && state)
  {

    if (elapsed > 400 && elapsed < 2500)
    {
      // long push
      encbutton_state = 2;
    }

    if (elapsed < 400)
    {
      // single click
      encbutton_state = 1;
    }
  }

  click_prev = state;
}
