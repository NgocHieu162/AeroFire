#include <Arduino.h>
#include <WiFi.h>
#include <DHT.h> 
#include <LiquidCrystal.h>
#include <PubSubClient.h>

const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqttServer = "test.mosquitto.org";
int port = 1883;

// Wifi
WiFiClient espClient;
PubSubClient client(espClient);

#define DHTPIN 4     
#define DHTTYPE DHT22
int button = 16;
int mq2_pin = 12;
int greenLed = 35;
int redLed = 47;
int buzzer = 19;
bool ledBlink = false;
unsigned long lastMillis = 0;
//bool switchMode = 0; //chưa dùng

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

void callback(char* topic, byte* message, unsigned int length) {
  Serial.println(topic);
  String stMessage;
  for(int i = 0; i < length; i++) {
    stMessage += (char)message[i];
  }
  Serial.println(stMessage);
  if(String(topic) == "23CLC09N12/Led") {
    if(stMessage == "true") {
      ledBlink = true;
    }
    else {
      ledBlink = false;
    }
  }
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

  client.setServer(mqttServer, port);
  client.setCallback(callback);
}

void mqttReconnect() {
  while(!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if(client.connect("23CLC09N12")) {
      Serial.println("connected");
      if(client.subscribe("23CLC09N12/Led")){
        Serial.println("Kết nối thành công.");
      }
      //client.subscribe("23CLC09N12/LOCK");
    } else {
      Serial.println(" try again in 5 seconds");
      delay(3000);
    }
  }
}

void loop() {
  if(!client.connected()){
    mqttReconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();
  static bool ledVisible = 0;
  static unsigned long previousBlinkTime = 0;

  if (ledBlink){ 
    if (currentMillis - previousBlinkTime >= 500){
      ledVisible = !ledVisible;
      previousBlinkTime = currentMillis;
    }
    digitalWrite(redLed, ledVisible ? HIGH : LOW);
  }
  else {
    digitalWrite(redLed, LOW);
  }


  // delay(2000);
}