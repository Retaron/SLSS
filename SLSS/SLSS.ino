#include <Servo.h>
#include "DHT.h"
#include <LiquidCrystal.h>

#define SERVO_PIN 9
#define HUMIDITY_PIN 10
#define HUMIDITY_SENSOR_TYPE DHT11
//LCD PINS
#define LCD_RS 12
#define LCD_E 13
#define LCD_D4 5
#define LCD_D5 4
#define LCD_D6 3
#define LCD_D7 2
// LCD_RW, LCD_VSS GND
// LCD_VCC 5V
// LCD LED+ 5V THROUGH 220 OHM RESISTOR
// LCD LED- TO GND
// MOTOR CONTROLLER L298N
#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 11

Servo humidityServo;
DHT dht(HUMIDITY_PIN, HUMIDITY_SENSOR_TYPE);
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
int servoPos;
int servoDelay = 10;

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
