#include <Arduino.h>
#include <WiFi.h>
#include <DHT.h> 
#include <LiquidCrystal.h>

const char* ssid = "Wokwi-GUEST";
const char* password = "";

#define DHTPIN 4     
#define DHTTYPE DHT22
int button = 16;
int mq2_pin = 12;
int greenLed = 35;
int redLed = 47;
int buzzer = 19;

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal lcd(41, 40, 39, 38, 37, 36); 

void wifiConnect() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
}

void setup() {
  Serial.begin(9600);
  
  pinMode(DHTPIN, INPUT_PULLUP); 
  pinMode(button, INPUT_PULLUP);
  pinMode(mq2_pin, INPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(redLed, OUTPUT);
  pinMode(buzzer, OUTPUT);

  dht.begin();
  lcd.begin(20, 4);
  lcd.print("System Ready!");

  Serial.print("Connecting to WiFi");
  wifiConnect();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Reconnecting to WiFi");
    wifiConnect();
  }
  
  // code để test
  delay(2000);

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.println(F("°C"));

  int mq2Value = analogRead(mq2_pin);
  Serial.print("MQ2: ");
  Serial.println(mq2Value);

  int buttonState = digitalRead(button);
  if(buttonState == HIGH){
    Serial.println("Button pressed");
  }

  // digitalWrite(greenLed, HIGH);
  // digitalWrite(redLed, HIGH);

  // delay(50);

  // digitalWrite(greenLed, LOW);
  // digitalWrite(redLed, LOW);

  // tone(buzzer, 5000, 200);
  // delay(1000);
  // noTone(buzzer);
}