/*
  RaggedPi Project
  Arduino 2 "Cinnamon" - Basement
  Written by david durost <david.durost@gmail.com>
*/

/* Includes */
#include <DHT.h> // temp sensor
#include <SPI.h> 
// #include <SD.h> // sd card reader
// #include <SoftwareSerial.h> // camera
// #include <Adafruit_VC0706.h> // camera

/* Defines */
// DHT
#define DHTPIN 4 // arduino digital pin
#define DHTTYPE DHT11 // DHT11 or DHT22
#define DHTTIME 2000 // wait 2 sec between measurements
#define DHTFAHRENHEIT true // Fahrenheit (true) or celsius (false)
// PIR
#define PIRLED 13 // LED output pin
#define PIRPIN 2 // arduino digital pin
#define PIRINIT 30 // sensor intialization time
// MQ2
#define MQ2PIN A0 // arduino analog pin
// Fire
#define FIREMIN 0 // min sensor reading
#define FIREMAX 1024 // max sensor reading
#define FIREPIN A1 // sensor analog pin
#define FIRETIME 5000 // time between readings (5s)
// Alarm
#define SPEAKERPIN 10 // pzSpeaker pin
#define FIREALARM 1 
// #define FIRECLOSEALARM 1
// #define FIREDISTANTALARM 2
#define GASALARM 2 // 3

/* Variables */
int ledState = LOW; // LED state
bool initialized[2] = { false, false }; // Boolean array denoting if each module has been initialized
  
/* Initializations */
DHT dht(DHTPIN, DHTTYPE);

/**
 * Float to int hack
 *  Arduino Uno chipset does not support floats being 
 *  used with sprintf this is half of the workaround
 * @param  float f
 * @return int
 */
int floatToIntHack(float f) {
  int i = f * 100;
  return i;
}

/**
 * Setup
 */
void setup() {
  Serial.begin(9600);
  Serial.println("RaggedPi Project Codename Cinnamon Initialized.");
  
  dht.begin();
  initializePIR();
}

/**
 * Read DHT sensor
 * @return null
 */
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
  char* CF;
  if (DHTFAHRENHEIT) {
    CF = " *F\t";
  } else {
    CF = " *C\t";
  }

  // char* output = "";
  // sprintf(
  //   output,
  //   "Temperature: %d %c Humidity: %d %%\t Heat Index: %d %c\n",
  //   floatToIntHack(temp),
  //   CF,
  //   floatToIntHack(humidity),
  //   floatToIntHack(hIndex),
  //   CF
  // );
  // Serial.print(output);

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

/**
 * Read PIR sensor
 */
void readPIR() {
  int pir = digitalRead(PIRPIN);
  int state = LOW;
  
  // If there's motion...
  if (pir == HIGH) {
    if (state == LOW) {
      // Toggle PIR state
      state = HIGH;
      Serial.print("Motion detected.\n");
      pinMode(PIRLED, HIGH);
      // capturePhoto();
    }
  } else {
    digitalWrite(PIRLED, LOW);
    if (state == HIGH) {
      state == LOW;
      Serial.print("Motion ended.\n");
      pinMode(PIRLED, LOW);
    }
  }
}

/**
 * Read MQ2 sensor
 */
void readMQ2() {
  int gasLvl = analogRead(MQ2PIN);
  int state = LOW;

  // if levels are too high...
  if (gasLvl > 200 || state == HIGH) {
    state = HIGH;
    Serial.print("Gas detected! [");
    Serial.print(gasLvl);
    Serial.print("]\n");
//    toggleAlarm(GASALARM);
  } else if (gasLvl <= 200 && state == HIGH) {
    state = LOW;
    Serial.print("Gas levels nominal. [");
    Serial.print(gasLvl);
    Serial.print("]\n");
// toggleAlarm(false);
  }
}

/**
 * Read fire sensor
 */
void readFire() {
  // Map sensor range
  int range = map(analogRead(FIREPIN), FIREMIN, FIREMAX, 0, 3);

  switch (range) {
    case 0: // A fire within 1.5 feet
      Serial.print("Close fire!\n");
      toggleAlarm(FIREALARM);
      break;
    case 1: // A fire between 1.5 and 3 feet
      Serial.print("Distant fire detected.\n");
      toggleAlarm(FIREALARM);
      break;
    case 2: // No fire
      Serial.print("All clear.\n");
      toggleAlarm(false);      
      break;
  }
  delay(FIRETIME);
}

/**
 * Toggle alarms
 * @param  int alarm
 */
void toggleAlarm(int alarm) {
  switch(alarm) {
    case FIREALARM:
      Serial.print("alarm activated. [fire]\n");
/** @todo separate tones for close and distant */
//      digitalWrite(SPEAKERPIN, HIGH);
      playTone(500, 600);
      delay(100);
      playTone(500, 800);
      delay(100);
    break;
    case GASALARM:
      Serial.print("alarm activated. [gas]\n");
//      digitalWrite(SPEAKERPIN, HIGH);
      playTone(500, 400);
      delay(100);
      playTone(500, 600);
      delay(100);
    break;
    case false:
      Serial.print("alarm deactivated.");
//      digitalWrite(SPEAKERPIN, LOW);
      playTone(0, 0);
      delay(300);      
    break;
  }
}

/**
 * Play audible tone from speaker
 * @param  long duration
 * @param  int  freq
 */
void playTone(long duration, int freq) {
  duration *= 1000;
  int period = (1.0 / freq) * 1000000;
  long elapsed_time = 0;
  while (elapsed_time < duration) {
    digitalWrite(SPEAKERPIN,HIGH);
    delayMicroseconds(period / 2);
    digitalWrite(SPEAKERPIN, LOW);
    delayMicroseconds(period / 2);
    elapsed_time += (period);
  }
}
 
/**
 * Loop
 */
void loop() {
  readDHT();
  readPIR();
  readMQ2();
//  readFire();
}
