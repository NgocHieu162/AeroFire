#include <Arduino.h>
#include <WiFi.h>
#include <DHT.h> 
#include <LiquidCrystal.h>
#include <PubSubClient.h>
#include "fire_model_data.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

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

float temperature = 0;
float humidity = 0;
float smoke = 500;

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal lcd(41, 40, 39, 38, 37, 36); 

const float mean[3] = {36.1719094, 54.19577833, 570.52825509};
const float std_val[3] = {14.1665273, 16.25330416, 1328.76111158};

#define TENSOR_ARENA_SIZE 2*1024
uint8_t tensor_arena[TENSOR_ARENA_SIZE];

tflite::MicroInterpreter* interpreter;
TfLiteTensor* input;
TfLiteTensor* output;

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

  // Load model
  const tflite::Model* model = tflite::GetModel(fire_model_tflite);
  static tflite::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, TENSOR_ARENA_SIZE, nullptr);
  interpreter = &static_interpreter;

  interpreter->AllocateTensors();

  input  = interpreter->input(0);
  output = interpreter->output(0);
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

void readPublishDHTData() {
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  if(isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  client.publish("23CLC09N12/temperature", String(temperature).c_str());
  client.publish("23CLC09N12/humidity", String(humidity).c_str());

  Serial.print("Temperature: "); Serial.println(temperature);
  Serial.print("Humidity: "); Serial.println(humidity);
}

void loop() {
  if(!client.connected()){
    mqttReconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();
  static bool ledVisible = 0;
  static unsigned long previousBlinkTime = 0;

  if(currentMillis - lastMillis >= 2000){
    lastMillis = currentMillis;
    readPublishDHTData();
    // Scale dữ liệu
    float input_scaled[3];
    float input_raw[3] = {temperature, humidity, smoke};
    for(int i = 0; i < 3; i++){
      input_scaled[i] = (input_raw[i] - mean[i]) / std_val[i];
      input->data.f[i] = input_scaled[i];
    }

    // Run inference
    interpreter->Invoke();

    float pred = output->data.f[0];
    Serial.print("Prediction: ");
    Serial.println(pred);  // gần 0 hoặc 1
  }


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
}