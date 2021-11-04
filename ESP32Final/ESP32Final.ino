/*
 * Tufts University ME 134, Fall 2021
 * 
 * By: Sawyer Paccione, Akshita Rao, Ronan Gissler
 * Completed: TBD
 *
 * Description: Get Readings from an MPU6050 Accelerometer and Gyroscope, and 
 *              a VL53L0X Lidar Sensor over I2C, and publish to an MQTT Broker.
 */
/**************************************************************************
 *                               Libraries                                *
 *************************************************************************/
#include <Adafruit_MPU6050.h> // Get Data from MPU6050
#include <Adafruit_Sensor.h>  // Get Data from MPU6050
#include <Wire.h>             // I2C Communication
#include "Adafruit_VL53L0X.h" // Get Data from Lidar
#include <ArduinoJson.h>      // Package Data into JSON Objects
#include <PubSubClient.h>     // Library for MQTT Communication
#include <WiFi.h>             // Connect to WiFi

/**************************************************************************
 *                            Global Variables                            *
 *************************************************************************/
// Define Sizes of JSON Objects
const int MPU_CAPACITY = JSON_OBJECT_SIZE(3);
const int LIDAR_CAPACITY = JSON_OBJECT_SIZE(1);
const int TIME_CAPACITY = JSON_OBJECT_SIZE(1);


// Defin JSON Objects
StaticJsonDocument<MPU_CAPACITY> accel;
StaticJsonDocument<MPU_CAPACITY> gyro;
StaticJsonDocument<LIDAR_CAPACITY> lidar;
StaticJsonDocument<TIME_CAPACITY> times;

// Define our sensors 
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;

// Define our MQTT Client
const char* ssid = "Tufts_Wireless";
const char* password = "";

// Tell our client what server to point the messages to
//const char* mqtt_server = "broker.emqx.io";
//String mqtt_user = "pi";
//String mqtt_pass = "me134";
const char* mqtt_server = "10.245.91.207";
String mqtt_user = "";
String mqtt_pass = "";

WiFiClient espClient;
PubSubClient client(espClient);

int MSG_DELAY = 50; //Time between messages in milliseconds CHANGE ME

unsigned long currTime;

/**************************************************************************
 *                            Main Functions                              *
 *************************************************************************/

void setup(void) {
  Serial.begin(115200);
//  while (!Serial)
//    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  delay(100);
  setupMPU();
  setupLidar();
  setupWiFi();
  setupMQTT();
}

void loop() {

  if (!client.connected()) {
    connectMQTT();
  }

  client.loop();
  
  getMPUData();
  getLidarData();
  sendData();
  delay(MSG_DELAY);
}

/**************************************************************************
 *                          Helper Functions                              *
 *************************************************************************/
/* Sensor Section */

/*
 * setupMPU
 * Purpose:     Setup the IMU on I2C and check if it's connected
 * Arguments:   N/A
 * Returns:     N/A
 * Effects:     I2C Bus, Serial Monitor 
 * Notes  :     Possible change the parameters to include variables 
 *              for the Ranges
 */
void setupMPU(){
  Serial.println("Adafruit MPU6050 test!");
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

/*
 * setupLidar
 * Purpose:     Setup the Lidar on I2C and check if it's connected
 * Arguments:   N/A
 * Returns:     N/A
 * Effects:     I2C Bus, Serial Monitor 
 * Notes  :     
 */
void setupLidar(){
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
}

/*
 * getMPUData
 * Purpose:     Get new data from the MPU Chip and store the accelerometer 
 *              and gyroscope data into different JSON objects
 * Arguments:   N/A
 * Returns:     N/A
 * Effects:     Accelerometer and Gyroscope JSON Objects 
 * Notes  :     
 */
void getMPUData(){
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accel["accelx"].set(a.acceleration.x/9.81);
  accel["accely"].set(a.acceleration.y/9.81);
  accel["accelz"].set(a.acceleration.z/9.81);

  gyro["gyrox"].set(g.gyro.x);
  gyro["gyroy"].set(g.gyro.y);
  gyro["gyroz"].set(g.gyro.z);

}

/*
 * getMPUData
 * Purpose:     Get new data from the MPU Chip and store the accelerometer 
 *              and gyroscope data into different JSON objects
 * Arguments:   N/A
 * Returns:     N/A
 * Effects:     Accelerometer and Gyroscope JSON Objects 
 * Notes  :     The Lidar JSON Object has only one element, however, storing 
 *              like this helps to easily serialize the information for
 *              MQTT transfer
 */
void getLidarData(){
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    lidar["milli"].set(measure.RangeMilliMeter);
  } else {
    lidar["milli"].set("-1");
  }
}

/* MQTT Section */
/*
 * setup_wifi
 * Purpose:     Setup the ESP32 on WiFi
 * Arguments:   N/A
 * Returns:     N/A
 * Effects:     WiFi Connection 
 * Notes  :     
 */
void setupWiFi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

/*
 * connectMQTT
 * Purpose:     Connect the MQTT Client on the ESP32
 * Arguments:   N/A
 * Returns:     N/A
 * Effects:     MQTT Connection 
 * Notes  :     
 */
void connectMQTT(){
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()), "pi", "me134") {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("esp32", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }  
}

/*
 * callback
 * Purpose:     Gets called whenever a topic this ESP32 is subscribed to
 *              gets a message
 * Arguments:   N/A
 * Returns:     N/A
 * Effects:     MQTT Connection 
 * Notes  :     
 */
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

/*
 * setupMQTT
 * Purpose:     Send Data over the MQTT protocol to the Raspberry Pi
 * Arguments:   N/A
 * Returns:     N/A
 * Effects:     MQTT Topics
 * Notes  :     
 */
void setupMQTT() {
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  connectMQTT();
}

/*
 * sendData
 * Purpose:     Send Data over the MQTT protocol to the Raspberry Pi
 * Arguments:   N/A
 * Returns:     N/A
 * Effects:     MQTT Topics
 * Notes  :     
 */
void sendData(){
  currTime = millis();
  
  char buffer[256];
  size_t n1 = serializeJson(accel, buffer);
  client.publish("esp32/accel", buffer, n1);
//  Serial.println(buffer);
  size_t n2 = serializeJson(gyro, buffer);
  client.publish("esp32/gyro", buffer, n2);
//  Serial.println(buffer);
  size_t n3 = serializeJson(lidar, buffer);
  client.publish("esp32/lidar", buffer, n3);
//  Serial.println(buffer);
//  Serial.println("data sent");
  times["milliseconds"].set(currTime);
  size_t n4 = serializeJson(times, buffer);
  client.publish("esp32/time", buffer, n4);
  
}
