#include <Adafruit_Sensor.h>
#include <DHT_U.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h> // Include the Servo library
#include <FastLED.h>    // Include the FastLED library
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Defining Pins
#define DHTPIN 12
#define LED 26
#define SERVO_PIN 2 // Servo motor pin
#define LED_PIN 4   // WS2812 LED strip pin
#define NUM_LEDS 16 // Number of LEDs in the strip

// PIR sensor initial cong=fig
int ledPinPIR = 25;    // choose the pin for the LED
int inputPinPIR = 14;  // choose the input pin (for PIR sensor)
int pirStatePIR = LOW; // we start, assuming no motion detected
int valPIR = 0;        // variable for reading the pin status

// Pins for potentiometer
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 // No reset pin
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
const int potPin = 35;

// DHT parameters
#define DHTTYPE DHT22 // DHT 11
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

// Servo motor
Servo servo;

// WS2812 LED strip
CRGB leds[NUM_LEDS];

// MQTT Credentials
const char *ssid = "Wokwi-GUEST"; // Setting your AP SSID
const char *password = "";        // Setting your AP PSK
const char *mqttServer = "broker.hivemq.com";
// const char* mqttUserName = "bqzbdodo";
// const char* mqttPwd = "5oU2W_QN2WD8";
const char *clientID = "ujaisldaaasdfgh;laslksdja1"; // Client ID username+0001
const char *topic = "Tempdata";                      // Publish topic

// Parameters for using non-blocking delay
unsigned long previousMillis = 0;
const long interval = 1000;
String msgStr = ""; // MQTT message buffer
float temp, hum;

// Setting up WiFi and MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi()
{
  delay(10);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect()
{
  while (!client.connected())
  {
    if (client.connect(clientID))
    {
      Serial.println("MQTT connected");
      client.subscribe("lights");
      client.subscribe("servo");           // Subscribe to servo topic
      client.subscribe("lights/neopixel"); // Subscribe to neopixel topic
      Serial.println("Topic Subscribed");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000); // wait 5sec and retry
    }
  }
}

// Subscribe callback
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  String data = "";
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    data += (char)payload[i];
  }
  Serial.println();
  Serial.print("Message size: ");
  Serial.println(length);
  Serial.println();
  Serial.println("-----------------------");
  Serial.println(data);

  if (String(topic) == "lights")
  {
    if (data == "ON")
    {
      Serial.println("LED");
      digitalWrite(LED, HIGH);
    }
    else
    {
      digitalWrite(LED, LOW);
    }
  }
  else if (String(topic) == "servo")
  {
    int degree = data.toInt(); // Convert the received data to an integer
    Serial.print("Moving servo to degree: ");
    Serial.println(degree);
    servo.write(degree); // Move the servo to the specified degree
  }
  else if (String(topic) == "lights/neopixel")
  {
    int red, green, blue;
    sscanf(data.c_str(), "%d,%d,%d", &red, &green, &blue); // Parse the received data into RGB values
    Serial.print("Setting NeoPixel color to (R,G,B): ");
    Serial.print(red);
    Serial.print(",");
    Serial.print(green);
    Serial.print(",");
    Serial.println(blue);
    fill_solid(leds, NUM_LEDS, CRGB(red, green, blue)); // Set all LEDs in the strip to the specified color
    FastLED.show();                                     // Update the LED strip with the new color
    fill_solid(leds, NUM_LEDS, CRGB(red, green, blue));
    FastLED.show();
  }
}

void handlePIRSensor()
{
  valPIR = digitalRead(inputPinPIR); // read input value

  if (valPIR == HIGH)
  {                                // check if the input is HIGH
    digitalWrite(ledPinPIR, HIGH); // turn LED ON
    if (pirStatePIR == LOW)
    {
      Serial.println("Motion detected!");
      pirStatePIR = HIGH;
    }
  }
  else
  {
    digitalWrite(ledPinPIR, LOW); // turn LED OFF
    if (pirStatePIR == HIGH)
    {
      Serial.println("Motion ended!");
      pirStatePIR = LOW;
    }
  }
}

// set up the OLED for potentiometer
void initOLED()
{
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("OLED init failed");
    while (1)
      ; // Stop execution if OLED not found
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED Ready");
  display.display();
}

void displayPotValue(int potValue)
{
  float voltage = potValue * (3.3 / 4095.0);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Pot:");
  display.println(potValue);

  display.print("Volt:");
  display.print(voltage, 2);
  display.display();
}

void setup()
{
  Serial.begin(115200);
  // Initialize device.
  dht.begin();
  // Get temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // Setup servo
  servo.attach(SERVO_PIN, 500, 2400);
  servo.write(0);

  // Setup WS2812 LED strip
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

  setup_wifi();
  client.setServer(mqttServer, 1883); // Setting MQTT server
  client.setCallback(callback);       // Define function which will be called when a message is received.

  // PIR SENSOR INITIAL SETUP
  pinMode(ledPinPIR, OUTPUT);  // declare LED as output
  pinMode(inputPinPIR, INPUT); // declare sensor as input

  // Initialize potentiometer
  initOLED();
}

void loop()
{

  handlePIRSensor(); // Call the PIR handler
  if (!client.connected())
  {              // If client is not connected
    reconnect(); // Try to reconnect
  }
  client.loop();
  unsigned long currentMillis = millis(); // Read current time
  if (currentMillis - previousMillis >= interval)
  { // If current time - last time > 5 sec
    previousMillis = currentMillis;
    // Read temperature and humidity
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature))
    {
      Serial.println(F("Error reading temperature!"));
    }
    else
    {
      Serial.print(F("Temperature: "));
      temp = event.temperature;
      Serial.print(temp);
      Serial.println(F("°C"));
    }
    // Get humidity event and print its value
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity))
    {
      Serial.println(F("Error reading humidity!"));
    }
    else
    {
      Serial.print(F("Humidity: "));
      hum = event.relative_humidity;
      Serial.print(hum);
      Serial.println(F("%"));
    }
    msgStr = String(temp) + "," + String(hum) + ",";
    byte arrSize = msgStr.length() + 1;
    char msg[arrSize];
    Serial.print("PUBLISH DATA: ");
    Serial.println(msgStr);
    msgStr.toCharArray(msg, arrSize);
    client.publish(topic, msg);
    msgStr = "";
    delay(1);
  }

  int value = analogRead(potPin);
  displayPotValue(value);
  delay(500); // Only update twice per second
}
