// RaggedPi Project
// Arduino 2 "Cinnamon"
// Written by david durost <david.durost@gmail.com>

// Includes
#include <DHT.h> // temp sensor
#include <SPI.h> 
#include <SD.h> // sd card reader
//#include <SoftwareSerial.h> // camera
//#include <Adafruit_VC0706.h> // camera

// Defines
#define DHTPIN 4 // arduino digital pin
#define DHTTYPE DHT11
#define DHTTIME 2000 // wait 2 sec between measurements
#define DHTFAHRENHEIT true // fahrenheit (true) or celsius (false)

#define PIRLED 13 // LED output pin
#define PIRPIN 2 // arduino digital pin
#define PIRINIT 30 // sensor intialization time

#define MQ2PIN A0 // arduino analog pin

int ledState = LOW;
bool initialized[2] = { false, false }; // Boolean array denoting if each module has been initialized
  

// Initializations
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  Serial.println("RaggedPi Project Codename Cinnamon Initialized.");
  
  dht.begin();
  initializePIR();
}

// Read DHT sensor
void readDHT() {
  delay(DHTTIME);
  
  // Read data (~250ms)
  float humidity = dht.readHumidity();
  float temp = dht.readTemperature(DHTFAHRENHEIT);
  
  // Check for read fails
  if(isnan(humidity)) {
    Serial.print("Failed to read from DHT sensor. (h)\n");
    return;
  } else if (isnan(temp)) {
    Serial.print("Failed to read from DHT sensor! (t)\n");
    return;
  }

  // Compute heat index
  float hIndex = dht.computeHeatIndex(temp, humidity, DHTFAHRENHEIT);
  
  // Output
  String CF = "";
  if (DHTFAHRENHEIT) {
    CF = " *F\t";
  } else {
    CF = " *C\t";
  }
  
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.print(CF);
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print("%\t");
  Serial.print("Heat Index: ");
  Serial.print(hIndex);
  Serial.print(CF);
  Serial.print("\n");
}

// Initialize PIR sensor
void initializePIR() {
  pinMode(PIRPIN, INPUT);
  pinMode(PIRLED, OUTPUT);
  
  // Set sensor off to start.
  digitalWrite(PIRPIN, LOW);
  
  Serial.print("Initializing PIR sensor.\n");
  for (int i; i < PIRINIT; i++) {
    Serial.print(".");
    delay(1000);
  }
  initialized[0] = true;
  Serial.print("PIR sensor initialized.\n");
  delay(50);
}

// Read PIR sensor
void readPIR() {
  int pir = digitalRead(PIRPIN);
  int state = LOW;
  
  // If there's motion...
  if (pir = HIGH) {
    digitalWrite(PIRLED, HIGH);
    if (state == LOW) {
      // Toggle PIR state
      state = HIGH;
      Serial.print("Motion detected.\n");
      // capturePhoto();
    }
  } else {
    digitalWrite(PIRLED, LOW);
    if (state == HIGH) {
      state == LOW;
      Serial.print("Motion ended.\n");
    }
  }
}

// Read MQ2 sensor
void readMQ2() {
  int gasLvl = analogRead(MQ2PIN);
  int state = LOW;

  // if levels are too high...
  if (gasLvl > 200 || state == HIGH) {
    state = HIGH;
    Serial.print("Gas detected! [");
    Serial.print(gasLvl);
    Serial.print("]\n");

    // trigger alarm
  } else if (gasLvl < 200 && state == HIGH) {
    state = LOW;
    Serial.print("Gas levels nominal. [");
    Serial.print(gasLvl);
    Serial.print("]\n");
    // toggle alarm
  }
}
 
void loop() {
  readDHT();
  readPIR();
  readMQ2();
}
