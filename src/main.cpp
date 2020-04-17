#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const char* ssid = "";
const char* password = "";
const char* mqtt_server = "192.168.1.24"; 
const char* mqtt_id = "Probe_1"; // TODO
const char* mqtt_user = "test";
const char* mqtt_pass = "test";
const char* mqtt_topic = "esp8266_DHT";

#define DHTPIN 4

#define NUMSAMPLES 20
#define WAITTIME 3500
#define MAXREADFAILS 10
#define MIN_COUNT 5

float averageT;

uint8_t failCount;

DHT dht(DHTPIN, DHT22, 11);

WiFiClient espClient;
PubSubClient client(espClient);

void restart() {
  Serial.println("Restarting.");
  ESP.restart();
}

void checkFailCount() {
  if(failCount > MAXREADFAILS) {
    Serial.println("Failure limit met.");
    restart();
  }
}

void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int wifiCount = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    digitalWrite(LED_BUILTIN, LOW);
    if(wifiCount > 10) {
      restart();
    }
    wifiCount++;
  }

  Serial.println("");
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println(WiFi.localIP());
}

void recordTemp() {
  Serial.println("Reading Temperature:");
  uint8_t count = 0;
  float t = 0.0F;
  averageT = 0.0F;

  while (count < NUMSAMPLES) {
    t = dht.readTemperature(true);
    
    if (isnan(t)) {
      Serial.print("DHT Read Failure; ");
      delay(WAITTIME);
      if(dht.read()) {
        t = dht.readTemperature(true);
        if(isnan(t)) {
          failCount++;
          break;
        }
      } else {
        Serial.println("DHT Read false");
        failCount++;
        checkFailCount();
        break;
      }
    }
    
    Serial.print(t);
    Serial.print("; ");

    averageT += t;
    count++;
    
    delay(WAITTIME);
  }
  averageT /= count;

  if(count < MIN_COUNT) {
    averageT = -100.0F;
  }
  
  Serial.println();
}

void checkWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    restart();
  }
}

void connectMQTT() {
  Serial.println("Checking MQTT Connection");
  while (!client.connected()) {
    Serial.print("Not connected. Attempting MQTT Connection...");
    WiFi.mode(WIFI_STA);

    if (client.connect(mqtt_id, mqtt_user, mqtt_pass)) {
      Serial.println("Connected");
    } else {
      int8 mqttClientState = client.state();
      Serial.print("failed, rc=");
      Serial.print(mqttClientState);
      Serial.println(" try again in 5 seconds");
      failCount++;
      checkFailCount();
      delay(5000);
    }
  }
}

void publishMessage() {
  if (averageT <= -100.0){
    Serial.println("Abort MQTT Publish");  
    return;
  }

  char msg[8];
  sprintf(msg, "%3.2f", averageT);

  char message[1000] = "";
  strcat(message, mqtt_id);
  strcat(message, ": ");
  strcat(message, msg);

  client.publish(mqtt_topic, message);  // Topic, message
  Serial.print("Published: ");
  Serial.println(message);
}

void setup() {       
  failCount = 0;         
  Serial.begin(115200);

  delay(100);
  pinMode(LED_BUILTIN, OUTPUT);

  dht.begin();
  
  connectWiFi();

  client.setServer(mqtt_server, 1883);
}

void loop() {
  checkWiFi();

  recordTemp();

  connectMQTT();

  publishMessage();
}
