#include <Servo.h>
#include "DHT.h"
#include <LiquidCrystal.h>

#define FAN_ENABLE_PWM 2
#define PELTIER_PWM_BURNING 3
#define PELTIER_PWM_FREEZING 4
#define SERVO_PIN 5
#define LED_R 6
#define LED_G 7
#define LED_B 8
#define CURRENT_SENSOR_PIN A0
#define HUMIDITY_PIN 10
#define HUMIDITY_SENSOR_TYPE DHT11
//LCD PINS
#define LCD_RS 27
#define LCD_E 26
#define LCD_D4 25
#define LCD_D5 24
#define LCD_D6 23
#define LCD_D7 22

Servo humidityServo;
DHT dht(HUMIDITY_PIN, HUMIDITY_SENSOR_TYPE);
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
int servoPos;
int servoDelay = 10;
const float currentSens = 0.185;  // 5A
const int adcRes; // resoution of anagogue digital converter
const float vcc = 5.0; //operating voltage

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  dht.begin();
  humidityServo.attach(SERVO_PIN);
  lcd.begin(16,2);
  lcd.print("Kaine has a massive head!");
}

void loop() {
  // put your main code here, to run repeatedly:
  //ReadDHTSensor();
  //SprayBottle();
  //delay(500);
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(millis() / 1000);
}

// reads peltier current using ACS712 sensor
float  ReadCurrentSensor(int nSamples) {
  float val = 0;
  for (int i = 0; i < nSamples;; i++) {
    val += analogRead(CURRENT_SENSOR_PIN);
    delay(1);
  }
  val = val / adcRes / nSamples;
  return (vcc / 2 - v * val) / currentSens;
}

void ReadDHTSensor() {
  float humidity = dht.readHumidity();
  float tempC = dht.readTemperature();
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print("%");
  Serial.print("  |  "); 
  Serial.print("Temperature: ");
  Serial.print(tempC);
  Serial.println("°C ~ ");
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
