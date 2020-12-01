#include <Arduino.h>
// Basic demo for accelerometer readings from Adafruit MPU6050
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESPAsyncUDP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <FS.h>
#include <LittleFS.h>
#include <SPIFFSEditor.h>

#define MPU_POWER_PIN D7

// data
const char *color = "HydroWort-Green";
const char *pub1 = "HydroWort/Green/TILT"; //send angle
const char *pub2 = "HydroWort/Green/TEMP"; //send temperature
const char *pub3 = "HydroWort/Green/SG";   //send Specific Gravity
const char *pub4 = "HydroWort/Green/BATT"; //send battery Value
const char *sub1 = "HydroWort/Green/TARE"; //recieve tare switch position

struct SensorData
{
  float temperature;
  float tilt;
  float sg;
  byte battery;
  sensors_vec_t acceleration;
  sensors_vec_t gyro;
};

const int usDelay = 3150;
long lastMsg = 0;
long KeepTime = 0;

Adafruit_MPU6050 mpu;
WiFiClient espClient;
PubSubClient client(espClient);
AsyncUDP udp;
AsyncWebServer server(80);
SensorData sensor_data;

const char *ssid = "";
const char *password = "";
const char *mqtt_server = "prod"; // no :1883

void setup_wifi()
{

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    char clientId[20];
    strcpy(clientId, color);
    //clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...

      //Duplicate these as needed
      client.publish(pub1, "0"); //Set up Publising  for each value
      client.publish(pub2, "0");
      client.publish(pub3, "0");
      client.publish(pub4, "0");
      // ... and resubscribe
      client.subscribe(sub1); //Set up Subscription for each value
      // client.subscribe(sub2);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  // char message[length + 1];
  // memset(message, 0, length + 1);
  // strncpy(message, (char *)payload, length);
  // Serial.println(message); //debug chcek for data captured

  // if (strcmp(topic, sub1) == 0)
  // {
  // }
}

void start_mpu()
{
  // Switch on MPU6050
  digitalWrite(MPU_POWER_PIN, HIGH);
  delay(100);

  // Try to initialize MPU6050!
  Serial.println("Initializing MPU6050...");
  while (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    delay(1000);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void stop_mpu()
{
  // Switch on MPU6050
  digitalWrite(MPU_POWER_PIN, LOW);
  delay(10);
}

void read_sensors()
{
  // Switch on MPU6050
  start_mpu();

  sensors_event_t a, g, temp;
  byte readCount = 255;
  double aX = 0, aY = 0, aZ = 0, gX = 0, gY = 0, gZ = 0, tmp = 0;
  for (size_t i = 0; i < readCount; i++)
  {
    mpu.getEvent(&a, &g, &temp);
    aX += a.acceleration.x;
    aY += a.acceleration.y;
    aZ += a.acceleration.z;
    gX += g.gyro.x;
    gY += g.gyro.y;
    gZ += g.gyro.z;
    tmp += temp.temperature;
    delayMicroseconds(usDelay);
  }
  aX = (aX / readCount);
  aY = (aY / readCount);
  aZ = (aZ / readCount);
  gX = (gX / readCount);
  gY = (gY / readCount);
  gZ = (gZ / readCount);
  tmp = (tmp / readCount);

  double conv_factor = 180 / PI;

  double angle_X = atan2(aY, aZ) * conv_factor;
  double angle_Y = atan2(aX, aZ) * conv_factor;

  double x_Buff = float(aX);
  double y_Buff = float(aY);
  double z_Buff = float(aZ);
  double roll = atan2(y_Buff, z_Buff) * conv_factor;
  double pitch = atan2((-x_Buff), sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * conv_factor;

  if (pitch < 0)
    pitch = pitch * -1;
  if (roll < 0)
    roll = roll * -1;

  double calLow = 15.0;
  double calHigh = 80.0;
  double sgLow = 1.000;
  double sgHigh = 1.120;

  float sg = 0.0;
  double OldRange = (calHigh - calLow);
  if (OldRange == 0)
  {
    sg = sgLow;
  }
  else
  {
    double NewRange = (sgHigh - sgLow);
    sg = ((((90 - roll) - calLow) * NewRange) / OldRange) + sgLow;
  }

  sensor_data.sg = sg;
  sensor_data.tilt = roll;
  sensor_data.temperature = tmp;
  sensor_data.battery = 100; // TODO: Create a battery sensor.
  sensor_data.acceleration.pitch = pitch;
  sensor_data.acceleration.roll = roll;
  sensor_data.acceleration.x = aX;
  sensor_data.acceleration.y = aY;
  sensor_data.acceleration.z = aZ;
  sensor_data.gyro.x = gX;
  sensor_data.gyro.y = gY;
  sensor_data.gyro.z = gZ;

  // Switch off MPU6050
  stop_mpu();
}

void print_sensor_data(SensorData data)
{
  /* Print out the values */
  char msg[50];
  Serial.print(data.acceleration.x);
  Serial.print(", ");
  Serial.print(data.acceleration.y);
  Serial.print(", ");
  Serial.print(data.acceleration.z);
  Serial.print(" | ");
  Serial.print(data.gyro.x);
  Serial.print(", ");
  Serial.print(data.gyro.y);
  Serial.print(", ");
  Serial.print(data.gyro.z);

  Serial.print(" | ");
  snprintf(msg, sizeof(msg), "%.1f", (float)data.temperature);
  Serial.print(msg);

  Serial.print(" | ");
  snprintf(msg, sizeof(msg), "%.1f", (float)data.tilt);
  Serial.print(msg);
  Serial.print(", ");
  snprintf(msg, sizeof(msg), "%.3f", (float)data.sg);
  Serial.print(msg);

  Serial.println("");
}

void publish_sensor_data(SensorData data)
{
  char msg[50];
  // Publish to MQTT
  snprintf(msg, sizeof(msg), "%.1f", data.tilt);
  client.publish(pub1, msg); // Publish Tilt
  snprintf(msg, sizeof(msg), "%.1f", data.temperature);
  client.publish(pub2, msg); // Publish Temp
  snprintf(msg, sizeof(msg), "%.3f", data.sg);
  client.publish(pub3, msg); // Publish SG
  snprintf(msg, sizeof(msg), "%ld", (long int)data.battery);
  client.publish(pub4, msg); // Publish SG

  // Publish to UDP
  snprintf(msg, sizeof(msg), "{\"tilt\":%.1f,\"temp\":%.1f,\"sg\":%.3f,\"batt\":%d,}", data.tilt, data.temperature, data.sg, data.battery);
  Serial.println(msg);
  udp.broadcastTo(msg, 13337);
}

// Replaces placeholder with LED state value
String processor(const String &var)
{
  Serial.println(var);
  if (var == "STATE")
  {
    return "Test";
  }
  else if (var == "TEMPERATURE")
  {
    return "getTemperature()";
  }
  else if (var == "HUMIDITY")
  {
    return "getHumidity()";
  }
  else if (var == "PRESSURE")
  {
    return "getPressure()";
  }
}

void start_web_server()
{
  if (!LittleFS.begin())
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  server.addHandler(new SPIFFSEditor("admin", "password"));

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.onNotFound([](AsyncWebServerRequest *request) {
    Serial.printf("NOT_FOUND: ");
    if (request->method() == HTTP_GET)
      Serial.printf("GET");
    else if (request->method() == HTTP_POST)
      Serial.printf("POST");
    else if (request->method() == HTTP_DELETE)
      Serial.printf("DELETE");
    else if (request->method() == HTTP_PUT)
      Serial.printf("PUT");
    else if (request->method() == HTTP_PATCH)
      Serial.printf("PATCH");
    else if (request->method() == HTTP_HEAD)
      Serial.printf("HEAD");
    else if (request->method() == HTTP_OPTIONS)
      Serial.printf("OPTIONS");
    else
      Serial.printf("UNKNOWN");
    Serial.printf(" http://%s%s\n", request->host().c_str(), request->url().c_str());

    if (request->contentLength())
    {
      Serial.printf("_CONTENT_TYPE: %s\n", request->contentType().c_str());
      Serial.printf("_CONTENT_LENGTH: %u\n", request->contentLength());
    }

    int headers = request->headers();
    int i;
    for (i = 0; i < headers; i++)
    {
      AsyncWebHeader *h = request->getHeader(i);
      Serial.printf("_HEADER[%s]: %s\n", h->name().c_str(), h->value().c_str());
    }

    int params = request->params();
    for (i = 0; i < params; i++)
    {
      AsyncWebParameter *p = request->getParam(i);
      if (p->isFile())
      {
        Serial.printf("_FILE[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
      }
      else if (p->isPost())
      {
        Serial.printf("_POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
      }
      else
      {
        Serial.printf("_GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
      }
    }

    request->send(404);
  });

  // server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
  //   request->send(LittleFS, "/index.html", String(), false, processor);
  // });
  // server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *request) {
  //   request->send(LittleFS, "/index.html", String(), false, processor);
  // });
  // server.on("/styles.css", HTTP_GET, [](AsyncWebServerRequest *request) {
  //   request->send(LittleFS, "/styles.css", "text/css");
  // });
  // server.on("/scripts.js", HTTP_GET, [](AsyncWebServerRequest *request) {
  //   request->send(LittleFS, "/scripts.js", "text/javascript");
  // });
  server.begin();
}

void setup(void)
{
  delay(100);
  pinMode(MPU_POWER_PIN, OUTPUT);

  Serial.begin(9600);
  Serial.println("");
  Serial.println("Start");

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  start_web_server();

  Serial.println("Ready!");
  Serial.flush();
  delay(100);
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  long now = millis();
  if ((now - lastMsg) > (10000 + KeepTime))
  {
    lastMsg = now;
    now = millis();

    read_sensors();
    publish_sensor_data(sensor_data);
    print_sensor_data(sensor_data);
  }
}