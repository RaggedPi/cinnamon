/*
  RaggedPi Project
  Arduino 2 "Cinnamon" - Basement
  Written by david durost <david.durost@gmail.com>
*/

/* Includes */
#include <TimeLib.h>        // time
#include <Wire.h>           // 1-wire
#include <DS3231.h>         // rtc
#include <DHT.h>            // temp sensor
#include <SPI.h>            // spi
#include <SD.h>             // sd card reader
//  #include <SoftwareSerial.h>  // camera
// #include <Adafruit_VC0706.h> // camera

/* Defines */
#define DEBUG 1             // uncomment for debug (verbose) mode
// DHT
#define DHTPIN 4 // arduino digital pin
#define DHTTYPE DHT11       // DHT11 or DHT22
#define DHTTIME 1000        // (ms) wait atleast 2 sec between measurements
#define DHTFAHRENHEIT true  // Fahrenheit (true) or Celsius (false)
// PIR
#define PIRLED 13           // LED output pin
#define PIRPIN 2            // arduino digital pin
#define PIRINIT 30          // sensor intialization delay time
// MQ2
#define MQ2PIN A0           // arduino analog pin
// Fire
#define FIREMIN 0           // min sensor reading
#define FIREMAX 1024        // max sensor reading
#define FIREPIN A1          // arduino analog pin
#define FIRETIME 5000       // time between readings (5s)
// Alarm
#define SPEAKERPIN 8        // arduino digital pin
#define GASALARM 1          // alarm delimiter
#define FIREALARM 2         // alarm delimiter
// #define FIRECLOSEALARM 2    // alarm delimiter
// #define FIREDISTANTALARM 3  // alarm delimiter

/* Variables */
int ledState = LOW;         // LED state
int pirState = LOW;         // PIR state
int gasState = LOW;         // gas state
File photo;                 // camera photo
Time t;                     // time instance
  
/* Objects */
DHT dht(DHTPIN, DHTTYPE);   // thermometer
DS3231  rtc(SDA, SCL);      // real time clock

/**
 * Float to int hack
 *  Arduino Uno chipset does not support floats being 
 *  used with sprintf this is half of the workaround
 * @param  float f
 * @return int
 */
int floatToIntHack(float f) {
  #ifdef DEBUG
    Serial.println("[DEBUG] Entered floatToIntHack.");
  #endif

  int i = f * 100;
  return i;
}

/**
 * Initialize RTC
 */
void initializeRTC() {
  #ifdef DEBUG
    Serial.println("[DEBUG] Entered initializeRTC.");
  #endif

  Serial.print("Initializing Real Time Clock...\n");
  // Init RTC in 24 hour mode
  rtc.begin(); 

  // Set inital date/time
  rtc.setTime(13,0,0);
  rtc.setDate(2,4,2016);
  
  Serial.print("RTC Initialized.\n");
}

/**
 * Initialize speaker
 */
void initializeSpeaker() {
  #ifdef DEBUG
    Serial.print("[DEBUG] Entered initializeSpeaker.");
  #endif

  Serial.print("Initializing speaker...\n");

  pinMode(SPEAKERPIN, OUTPUT);
  delay(1000);
  Serial.print("SpeakerInitialized.\n");
}

/**
 * Read DHT sensor
 * @return null
 */
void readDHT() {
  #ifdef DEBUG
    Serial.println("[DEBUG] Entered readDHT.");
  #endif
  
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
  delay(DHTTIME);
}

/**
 * Initialize PIR
 */
void initializePIR() {
  #ifdef DEBUG
    Serial.println("[DEBUG] Entered initializePIR.");
  #endif

  pinMode(PIRPIN, INPUT);
  pinMode(PIRLED, OUTPUT);
  
  // Set sensor off to start.
  digitalWrite(PIRPIN, LOW);
  
  Serial.print("Initializing PIR sensor.");
  for (int i; i < PIRINIT; i++) {
    Serial.print(".");
    delay(1000);
  }
  Serial.print("\nPIR sensor initialized.\n");
  delay(50);
}

/**
 * Read PIR sensor
 */
void readPIR() {
  #ifdef DEBUG
    Serial.println("[DEBUG] Entered readPIR.");
  #endif

  int pir = digitalRead(PIRPIN);
  
  if (pir == HIGH) {        // If motion is detected
    if (pirState == LOW) {  // If there was not previous motion
      pirState = HIGH;      // toggle state
      Serial.print("Motion detected.\n");
      digitalWrite(PIRLED, HIGH); // toggle led
      // capturePhoto();       // capture still image from camera
    }
  } else {                  // No motion detected
    if (pirState == HIGH) { // if there was previous motion
      pirState == LOW;      // toggle state
      Serial.print("Motion ended.\n");
      digitalWrite(PIRLED, LOW);  // toggle led
    }
  }
}

/**
 * Read MQ2 sensor
 */
void readMQ2() {
  #ifdef DEBUG
    Serial.println("[DEBUG] Entered readMQ2.");
  #endif

  int gasLvl = analogRead(MQ2PIN);
  int state = LOW;

  // if levels are too high...
  if (gasLvl > 500 || state == HIGH) {
    state = HIGH;
    Serial.print("Gas detected! [");
    Serial.print(gasLvl);
    Serial.print("]\n");
    toggleAlarm(GASALARM);
  } else if (gasLvl <= 200 && state == HIGH) {
    state = LOW;
    Serial.print("Gas levels nominal. [");
    Serial.print(gasLvl);
    Serial.print("]\n");
    toggleAlarm(false);
  }
}

/**
 * Read fire sensor
 */
void readFire() {
  #ifdef DEBUG
    Serial.println("[DEBUG] Entered readFire.");
  #endif

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
  #ifdef DEBUG
    Serial.println("[DEBUG] Entered toggleAlarm.");
    Serial.print("[DEBUG] Passed param: alarm = ");
    Serial.print(alarm);
    Serial.print("\n");
  #endif

  switch(alarm) {
    case FIREALARM:
      Serial.print("alarm activated. [fire]\n");
/** @todo separate tones for close and distant */
//      digitalWrite(SPEAKERPIN, HIGH);
      playTone(500, 600);
      delay(100);
      playTone(500, 600);
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
  #ifdef DEBUG
    Serial.println("[DEBUG] Entered playTone.");
    Serial.print("[DEBUG] Passed Param: duration = ");
    Serial.print(duration);
    Serial.print("\n[DEBUG] Passed Param: freq = ");
    Serial.print(freq);
    Serial.print("\n");
  #endif

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
  t = rtc.getTime();
  Serial.print("Today is ");
  Serial.print(rtc.getMonthStr());
  Serial.print(" ");
  Serial.print(t.date, DEC);
  Serial.print(", ");
  Serial.print(t.year, DEC);
  Serial.print(".\n");
  Serial.print("It is currently ");
  Serial.print(t.hour);
  Serial.print(":");
  Serial.print(t.min);
  Serial.print(":");
  Serial.print(t.sec);
  Serial.print("\n");

  readDHT();
  readPIR();
  readMQ2();
//  readFire();
  delay(1000);              // wait 1 sec so as not to send massive amounts of data
}

/**
 * Generate filename
 * @return char*
 */
char* generateFileName() {
  #ifdef DEBUG
    Serial.println("[DEBUG] Entered generateFileName.");
  #endif

  char* name;
  
  sprintf(
    name,
    "%d_%d_%d_%d_%d_%d",
    rtc.getMonthStr(),
    t.date,
    t.year,
    t.hour,
    t.min,
    t.sec);

  return name;
}

/**
 * Setup
 */
void setup() {
  Serial.begin(9600);
  Serial.println("RaggedPi Project Codename Cinnamon Initialized.");
  
  SPI.begin();
  dht.begin();
  rtc.begin();
  initializePIR();
  initializeSpeaker();
  //initializeRTC();
}