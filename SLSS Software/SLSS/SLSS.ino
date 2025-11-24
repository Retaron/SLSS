#include <Servo.h>
#include <SimpleDHT.h>
#include <LiquidCrystal.h>
#include <string.h>
#include <ezButton.h>

#define ROTARYENCODER_CLK 2
#define ROTARYENCODER_DT 3
#define ROTARYENCODER_SW 4
#define TOGGLESWITCH 13 // front panel on/off image
#define SERVO_PIN 5
#define LED_R 6
#define LED_G 7
#define LED_B 8
#define CURRENT_SENSOR_PIN A0
#define DHT11_PIN 18 // its pin 18 because i dont think it works on other ones, it would on pin 2 but its occupied
#define FAN_ENABLE_PWM 10
#define PELTIER_PWM_BURNING 11
#define PELTIER_PWM_FREEZING 12
//LCD PINS
#define LCD_RS 27
#define LCD_E 26
#define LCD_D4 25
#define LCD_D5 24
#define LCD_D6 23
#define LCD_D7 22

Servo humidityServo;
SimpleDHT11 dht11(DHT11_PIN);
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
ezButton button(ROTARYENCODER_SW);
ezButton toggleSwitch(TOGGLESWITCH);
int servoPos;
int servoDelay = 10;
const float currentSens = 0.185;  // 5A
const int adcRes; // resoution of anagogue digital converter
const float vcc = 5.0; //operating voltage
//enum LCD_Page {Home, Peltier, RGB};
//enum LCD_Page LCD_Current_Page = Home;
volatile int LCD_Current_Page = 1;
// rotary encoder variables
static byte abOld;
volatile int count;
         int old_count;
float rotaryEncoderTimerTime = 200;
float rotaryEncoderTimer = 0;
float myFloat;
byte humidity;
byte tempC;
bool isPageSelected = false; // for when page is selected to prevent changing page
int RGB[3];
int RGBState; //these two variables are for calculating position of RGB values on the screen and editing them
int RGBThreeIntPosition;
bool rbgValuesSelected = false;
bool rgbIndividualNumberSel = false;
char currentSensorString[10];
char humiditySensorString[10];
char floatString[5] = {0};

void setup() {
  Serial.begin(9600);
  //humidityServo.attach(SERVO_PIN);
  pinMode(ROTARYENCODER_CLK, INPUT);
  pinMode(ROTARYENCODER_DT, INPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  //rotaryEncoderPrevCLK = digitalRead(ROTARYENCODER_CLK);
  button.setDebounceTime(50); 
  toggleSwitch.setDebounceTime(50);
  attachInterrupt(0, pinChangeISR, CHANGE);
  attachInterrupt(1, pinChangeISR, CHANGE);
  abOld = count = old_count = 0;
  lcd.begin(16,2);
  int RGB[3] = {0,0,0};
  currentSensorString[10] = {0};
}

void loop() {
  toggleSwitch.loop();
  //if (toggleSwitch.getState() == LOW) { // switch is on, if it isnt then disable most logic
    button.loop();
    int rotaryEncoderIncrement = RotaryEncoderUpdate();
    LCDLoop(rotaryEncoderIncrement);
    ChangeLCDPage(rotaryEncoderIncrement);
    updateRGBLED();
  //}
  delay(50);
}

void LCDLoop(int rotaryEncoderIncrement) {
  switch (LCD_Current_Page) {
    case 1:
      lcd.noBlink();
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Home Page");
      ReadDHTSensor();
      lcd.setCursor(0,1);
      lcd.print(humiditySensorString);
      PrintRightSide("Pg 1/3", 1);
      break;
    case 2:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Peltier Page");
      lcd.setCursor(0,1);
      myFloat = 1.21;
      floatString[5] = {0};
      dtostrf(myFloat, 3, 2, floatString);
      snprintf(currentSensorString, 10, "%s%s%c", "5V ", floatString, 'A');
      lcd.print(currentSensorString);
      PrintRightSide("Pg 2/3", 1);
      break;
    case 3:
      lcd.blink();
      lcd.clear();
      lcd.print("RGB");
      lcd.setCursor(0,0);
      char* formattedNumber = padRGBNumber('R', RGB[0]);
      lcd.print(formattedNumber);
      free(formattedNumber);
      lcd.setCursor(5,0);
      formattedNumber = padRGBNumber('G', RGB[1]);
      lcd.print(formattedNumber);
      free(formattedNumber);
      lcd.setCursor(10,0);
      formattedNumber = padRGBNumber('B', RGB[2]);
      lcd.print(formattedNumber);
      free(formattedNumber);
      PrintRightSide("Pg 3/3", 1);
      lcd.setCursor(0,0);
      EditRGBValues(rotaryEncoderIncrement); // allows user to edit RGB values if button is pressed
      break;
    default:
      break;
  }
}

void updateRGBLED() {
  analogWrite(LED_R, RGB[0]);
  analogWrite(LED_G, RGB[1]);
  analogWrite(LED_B, RGB[2]);
}

char* padRGBNumber(char prefix, int RBGNumber) {
  char* result = (char*)malloc(6 * sizeof(char));
  if (result == NULL) {
    return NULL;
  }
  snprintf(result,6,"%c%03d", prefix, RBGNumber);
  return result;
}

//int RGBState; //these two variables are for calculating position of RGB values on the screen and editing them
//int RGBThreeIntPosition;
//bool rbgValuesSelected = false;
//bool rgbIndividualNumberSel = false;

void EditRGBValues(int increment) {
  if (RGBState == 0 && RGBThreeIntPosition == 0 && rgbIndividualNumberSel == false) {
    isPageSelected = false;
  }
  else {
    isPageSelected = true;
  }
  
  int LCDPosition = 1 + RGBState * 5 + RGBThreeIntPosition;
  lcd.setCursor(LCDPosition, 0);
  //if (rbgValuesSelected == true) {
    if (button.isPressed()) {
      rgbIndividualNumberSel = !rgbIndividualNumberSel;
    }
    if (rgbIndividualNumberSel == true) {
      // add logic for 10s and 100s
      int changedRGBValue;
      if (RGBThreeIntPosition == 0) {
        changedRGBValue = increment * 100;
      }
      else if (RGBThreeIntPosition == 1) {
        changedRGBValue = increment * 10;
      }
      else {
        changedRGBValue = increment;
      }
      //int changedRGBValue = increment * RGBThreeIntPosition;
      RGB[RGBState] = RGB[RGBState] + changedRGBValue;
    }
    else {
      RGBThreeIntPosition = RGBThreeIntPosition + increment;
      // boundary checks
      if (RGBThreeIntPosition > 2) {
        RGBThreeIntPosition = 0;
        RGBState++;
      }
      else if (RGBThreeIntPosition < 0) {
        RGBThreeIntPosition = 2;
        RGBState--;
      }
      if (RGBState > 2) {
        RGBState = 2;
      }
      else if (RGBState < 0) {
        RGBState = 0;
      }
    }
  //}
  // RGB values boundary check
  RGB[0] = ECCRGB(RGB[0]);
  RGB[1] = ECCRGB(RGB[1]);
  RGB[2] = ECCRGB(RGB[2]);
}

int ECCRGB(int rgbValue) {
  if (rgbValue > 255) {
    return 255;
  }
  else if (rgbValue < 0) {
    return 0;
  }
  return rgbValue;
}

void ChangeLCDPage(int increment) {
  if (isPageSelected == false) {
    LCD_Current_Page = increment + LCD_Current_Page;

    if (LCD_Current_Page > 3){
      LCD_Current_Page = 3;
    }
    if (LCD_Current_Page < 1) {
      LCD_Current_Page = 1;
    }
  }
}

void PrintRightSide(char text[], int column) {
  lcd.setCursor(16-strlen(text), column);
  lcd.print(text);
}

int RotaryEncoderUpdate() {
  //if (rotaryEncoderTimer < millis()) {
  int deltaInput = abs(count - old_count);
  if (deltaInput >= 4) {
    //Serial.println(count);
    //Serial.println(old_count);
    int delta = (count - old_count) / 4;
    //Serial.print("delta: ");
    //Serial.println(delta);
    old_count = count;
    return delta * -1;
  }
  return 0;
}

void pinChangeISR() {
  enum { upMask = 0x66, downMask = 0x99 };
  byte abNew = (digitalRead(ROTARYENCODER_CLK) << 1) | digitalRead(ROTARYENCODER_DT);
  byte criterion = abNew^abOld;
  if (criterion==1 || criterion==2) {
    if (upMask & (1 << (2*abOld + abNew/2)))
      count++;
    else count--;       // upMask = ~downMask
  }
  abOld = abNew;        // Save new state
}

// reads peltier current using ACS712 sensor
int nSamples = 1024;
int currentSensorCount = 0;
float currentReadTime = 1;
float currentReadTimer;
float currentSensorCurrent;
void  ReadCurrentSensor() {
  float val = 0;
  if (currentReadTimer + currentReadTime < millis()){
    val += analogRead(CURRENT_SENSOR_PIN);
    currentSensorCount++;
    currentReadTimer = millis();
  }
  if (currentSensorCount >= nSamples) {
    val = val / adcRes / nSamples;
    currentSensorCurrent = (vcc / 2 - vcc * val) / currentSens;
    snprintf(currentSensorString, 10, "%s%.2f%%c", "I: ", currentSensorCurrent, 'A');
    currentSensorCount = 0;
  }
}

//int humidity;
//float tempC;
void ReadDHTSensor() {
  //humidity = dht.readHumidity();
  //tempC = dht.readTemperature();
  //Serial.print("humidity: ");
  //Serial.println(humidity);
  dht11.read(&tempC, &humidity, NULL);
  snprintf(humiditySensorString, 10, "%i%s%i%c", humidity, "% ", tempC, 'C');
}

void SprayBottle() {
  for (servoPos = 0; servoPos <= 90; servoPos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    humidityServo.write(servoPos);              // tell servo to go to position in variable 'pos'
    delay(servoDelay);                       // waits 15ms for the servo to reach the position
  }
  for (servoPos = 90; servoPos >= 0; servoPos -= 1) { // goes from 180 degrees to 0 degrees
    humidityServo.write(servoPos);              // tell servo to go to position in variable 'pos'
    delay(servoDelay);                       // waits 15ms for the servo to reach the position
  }
}
