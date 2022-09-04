#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <ESP8266WiFi.h>
#include <DHT.h>
#include <DFRobot_VEML6075.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>

#define WIFI_SSID "N.N."
#define WIFI_PASSWORD "fatamorgana"

//Setting Raspberry Pi Mosquitto as a MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 126, 4)
#define MQTT_PORT 1883

#define MQTT_PUB_TEMP1 "esp/temperature1"
#define MQTT_PUB_TEMP2 "esp/temperature2"
#define MQTT_PUB_HUM "esp/humidity"
#define MQTT_PUB_UVA "esp/uva"
#define MQTT_PUB_UVB "esp/uvb"

//GPIO where the DS18B20 is connected to
const int oneWireBus = 14;

//Digital pin connected to the DHT sensor
#define DHTPIN 12
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define VEML6075_ADDR 0x10
DFRobot_VEML6075_IIC VEML6075(&Wire, VEML6075_ADDR);

float humidity;
float temperatureFirstDevice;
float temperatureSecondDevice;

//Variable for UV sensor
uint16_t UvaRaw;
uint16_t UvbRaw;
uint16_t comp1Raw;
uint16_t comp2Raw;
float Uva;
float Uvb;
float Uvi;

//Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

//Pass oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

//Saves last time data was published
unsigned long previousMillis = 0;
const long interval = 10000;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  // Ensure device don't reconnect to MQTT while reconnecting to Wi-Fi
  mqttReconnectTimer.detach();
  wifiReconnectTimer.once(2, connectToWifi);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  // Start the Serial Monitor
  Serial.begin(115200);

  // Start DHT11 sensor
  dht.begin();

  // Start the DS18B20 sensors
  sensors.begin();

  //Start VEML sensor
  while (!Serial);

  Serial.println();
  while (VEML6075.begin() != true) {
    Serial.println("VEML6075 begin faild");
    delay(2000);
  }
  Serial.println("VEML6075 begin successed");

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // Username and password settings
  // mqttClient.setCredentials("user", "trudnehaslo");

  connectToWifi();
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    sensors.requestTemperatures();
    temperatureFirstDevice = sensors.getTempCByIndex(0);
    temperatureSecondDevice = sensors.getTempCByIndex(1);

    if (isnan(temperatureFirstDevice)) {
      Serial.println("Failed to read temperature from first device!");
    }
    else {
      Serial.print("Temperature for first device: ");
      Serial.print(temperatureFirstDevice);
      Serial.println("ºC");
    }
    if (isnan(temperatureSecondDevice)) {
      Serial.println("Failed to read temperature from second device!");
    }
    else {
      Serial.print("Temperature for second device: ");
      Serial.print(temperatureSecondDevice);
      Serial.println("ºC");
    }

    humidity = dht.readHumidity();
    if (isnan(humidity)) {
      Serial.println("Failed to read humidity!");
    }
    else {
      Serial.print("Humidity: ");
      Serial.print(humidity);
      Serial.println("%");
    }

    UvaRaw = VEML6075.readUvaRaw();
    UvbRaw = VEML6075.readUvbRaw();
    comp1Raw = VEML6075.readUvComp1Raw();
    comp2Raw = VEML6075.readUvComp2Raw();

    Uva = VEML6075.getUva();
    Uvb = VEML6075.getUvb();
    Uvi = VEML6075.getUvi(Uva, Uvb);
    if (isnan(UvaRaw) || isnan(UvbRaw) || isnan(comp1Raw) || isnan(comp2Raw) || isnan(Uva) || isnan(Uvb)) {
      Serial.println("Failed to read date from UV sensor!");
    }
    else {
      Serial.print("UVA: ");
      Serial.println(Uva, 2);
      Serial.print("UVB: ");
      Serial.println(Uvb, 2);
    }

    // Publish an MQTT message on topic esp/temperature1
    char buffer[50];
    snprintf(buffer, 50, "temp1 temperature1=%f", temperatureFirstDevice);
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP1, 1, true, buffer);
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_TEMP1, packetIdPub1);
    Serial.printf("Message: %.2f \n", temperatureFirstDevice);

    // Publish an MQTT message on topic esp/temperature2
    snprintf(buffer, 50, "temp2 temperature2=%f", temperatureSecondDevice);
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_TEMP2, 1, true, buffer);
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_TEMP2, packetIdPub2);
    Serial.printf("Message: %.2f \n", temperatureSecondDevice);

    // Publish an MQTT message on topic esp/humidity
    snprintf(buffer, 50, "humidity humidity=%f", humidity);
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_HUM, 1, true, buffer);
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub3);
    Serial.printf("Message: %.2f \n", humidity);

    // Publish an MQTT message on topic esp/uva
    snprintf(buffer, 50, "uva uva=%f", Uva);
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_UVA, 1, true, buffer);
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_UVA, packetIdPub4);
    Serial.printf("Message: %.2f \n", Uva);

    // Publish an MQTT message on topic esp/uvb
    snprintf(buffer, 50, "uvb uvb=%f", Uvb);
    uint16_t packetIdPub5 = mqttClient.publish(MQTT_PUB_UVB, 1, true, buffer);
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_UVB, packetIdPub5);
    Serial.printf("Message: %.2f \n", Uvb);
  }
}
