#include <DHT.h>

DHT dht(9, DHT11);
float humidity;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  dht.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  //int chk = dht.read(HUMIDITY_PIN);
  float humidity2 = dht.readHumidity();
  //tempC = dht.readTemperature();
  Serial.print("humidity: ");
  Serial.println(humidity);
  delay(2000);
}
