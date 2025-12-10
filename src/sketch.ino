#include <Arduino.h>
#include <WiFi.h>
#include <DHT.h> 
#include <LiquidCrystal.h>
#include <PubSubClient.h>
#include "fire_model_data.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "ThingSpeak.h"
#include "apiKey.h"

const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqttServer = "test.mosquitto.org";
int port = 1883;
const char* server = "api.thingspeak.com";

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
bool toneBuzzer = false;
bool isBuzzing = false;
unsigned long lastMillis = 0;
//   button
bool mode = false;  // false: mode 1, true: mode 2
bool modeSwitched = false;
unsigned long buttonPressTime = 0;
const unsigned long holdTime = 3000;
int lastButtonState = LOW;
int buttonState = LOW;
bool isFire = false;

//cloud related variable
unsigned long lastUpdateTime = 0;

float temperature = 0;
float humidity = 0;
float mq2Value = 0;


DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal lcd(41, 40, 39, 38, 37, 36); 

const float mean[3] = {36.45518212, 54.09627852, 3502.11820286};
const float std_val[3] = {14.38166738 , 16.25418753, 171.23889855};

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
  if (String(topic) == "23CLC09N12/buzzer") {
    if (stMessage == "true") {
      toneBuzzer = true;
      mode = true;
    }
    else if (stMessage == "false") {
      toneBuzzer = false;
    }
  }
}

void mqttReconnect() {
  while(!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if(client.connect("23CLC09N12")) {
      Serial.println("connected");
      if(client.subscribe("23CLC09N12/Led") && client.subscribe("23CLC09N12/buzzer")){
        Serial.println("Kết nối thành công.");
      }
    } else {
      Serial.println(" try again in 5 seconds");
      delay(3000);
    }
  }
}

void handleButton() {
  buttonState = digitalRead(button);

  if (buttonState != lastButtonState) {
    buttonPressTime = millis();
    lastButtonState = buttonState;
    modeSwitched = false;
  }

  if (buttonState == HIGH) {
    if (!modeSwitched && (millis() - buttonPressTime >= holdTime)) {
      mode = !mode;
      modeSwitched = true;
      buttonPressTime = millis();
      Serial.print("Mode changed: ");
      Serial.println(mode ? "Mode 2" : "Mode 1");
      client.publish("23CLC09N12/mode", mode ? "2" : "1");
    }
  }
}

void readPublishSensorsData() {
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  mq2Value = analogRead(mq2_pin);

  if(isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  if (isnan(mq2Value)) {
    Serial.println("Failed to read from MQ2 sensor!");
    return;
  }

  client.publish("23CLC09N12/temperature/c", String(temperature).c_str());
  client.publish("23CLC09N12/humidity/%", String(humidity).c_str());
  client.publish("23CLC09N12/MQ2/ppm", String(mq2Value).c_str());
  client.publish("23CLC09N12/fire", isFire ? "1" : "0");

  Serial.print("Temperature: "); Serial.println(temperature);
  Serial.print("Humidity: "); Serial.println(humidity);
  Serial.print("MQ2 ADC: "); Serial.println(mq2Value);
}

void setup() {
  Serial.begin(115200);
  
  pinMode(button, INPUT);
  pinMode(mq2_pin, INPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(redLed, OUTPUT);
  pinMode(buzzer, OUTPUT);

  dht.begin();
  delay(2000);
  lcd.begin(20, 4);
  lcd.setCursor(0,0);
  lcd.print("Temperature: ");
  lcd.setCursor(0,1);
  lcd.print("Humidity: ");
  lcd.setCursor(0,2);
  lcd.print("Gas: ");
  lcd.setCursor(0,3);
  lcd.print("Mode: ");
  lcd.setCursor(6, 3);
  lcd.print(mode ? "2" : "1");

  Serial.print("Connecting to WiFi");
  wifiConnect();

  client.setServer(mqttServer, port);
  client.setCallback(callback);
  client.setKeepAlive(10);

  // Load model
  const tflite::Model* model = tflite::GetModel(fire_model_tflite);
  static tflite::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, TENSOR_ARENA_SIZE, nullptr);
  interpreter = &static_interpreter;

  interpreter->AllocateTensors();

  input  = interpreter->input(0);
  output = interpreter->output(0);

  // cloud
  ThingSpeak.begin(espClient);
}

void loop() {
  handleButton();
  if(!client.connected()){
    mqttReconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();
  static bool ledVisible = 0;
  static unsigned long previousBlinkTime = 0;
  if(modeSwitched){
    lcd.setCursor(0,3);
    lcd.print("Mode: ");
    lcd.setCursor(6, 3);
    lcd.print(mode ? "2" : "1");
  }

  // LED
  digitalWrite(greenLed, mode ? LOW : HIGH);
  digitalWrite(redLed, mode ? HIGH : LOW);

  if(currentMillis - lastMillis >= 2000){
    lastMillis = currentMillis;
    readPublishSensorsData();
    
    lcd.setCursor(0,0);
    lcd.print("Temperature: ");
    lcd.setCursor(13, 0);
    lcd.print(temperature, 1);
    lcd.setCursor(0,1);
    lcd.print("Humidity: ");
    lcd.setCursor(10, 1);
    lcd.print(humidity, 1);
    lcd.setCursor(0,2);
    lcd.print("Gas: ");
    lcd.setCursor(5, 2);
    lcd.print(mq2Value, 1);

    if(mode){
      // Scale dữ liệu
      float input_scaled[3];
      float input_raw[3] = {temperature, humidity, mq2Value};
      for(int i = 0; i < 3; i++){
        input_scaled[i] = (input_raw[i] - mean[i]) / std_val[i];
        input->data.f[i] = input_scaled[i];
      }

      // Run inference
      interpreter->Invoke();

      float pred = output->data.f[0];
      Serial.print("Prediction: ");
      Serial.println(pred);  // gần 0 hoặc 1

      if(pred > 0.5) isFire = true;
      else isFire = false;
    }
  }

  if (currentMillis - lastUpdateTime > 15000){
    ThingSpeak.setField(1, temperature);
    ThingSpeak.setField(2, humidity);
    ThingSpeak.setField(3, mq2Value);

    int x = ThingSpeak.writeFields(channelId, writeAPIKey);

    if (x == 200){
      Serial.println("Success uploading data to cloud.");
    }
    else{
      Serial.print("Problem updating channel. HTTP error code: ");
      Serial.println(x);
    }

    lastUpdateTime = currentMillis;
  }

  if (isFire || ledBlink){ 
    if (currentMillis - previousBlinkTime >= 500){
      ledVisible = !ledVisible;
      previousBlinkTime = currentMillis;
    }
    digitalWrite(redLed, ledVisible ? HIGH : LOW);
  }
  else {
    digitalWrite(redLed, LOW);
  }

  //web -> buzzer
  if (isFire || toneBuzzer) {
    if(!isBuzzing){ // chỉ bật buzzer nếu isBuzzing == false, nghĩa là buzzer chưa bật
      tone(buzzer, 5000); 
      isBuzzing = true;
    }
  }
  else {
    if (isBuzzing) { // chỉ tắt buzzer nếu isBuzzing == true, nghĩa là buzzer đang bật
      noTone(buzzer);
      isBuzzing = false; // Đánh dấu là đã tắt
    }
  }
}