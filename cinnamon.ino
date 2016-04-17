/*
  RaggedPi Project
  Arduino 2 "Cinnamon" - Basement
  Written by david durost <david.durost@gmail.com>
*/

/* Includes */
#include <TimeLib.h>        // time
#include <DS1307RTC.h>      // rtc
#include <Wire.h>           // 1-wire
#include <DS3231.h>         // rtc
#include <DHT.h>            // temp sensor
#include <SPI.h>            // spi
#include <SD.h>             // sd card reader
#include <SoftwareSerial.h> // camera
#include <Adafruit_VC0706.h>// camera
#include <PrintCascade.h>   // debugging
#include <SerialDebug.h>    // debugging

/* Defines */
#define DEBUG true          // uncomment for debugger
#define VERBOSE false
// DHT
#define DHTPIN 4 // arduino digital pin
#define DHTTYPE DHT11       // DHT11 or DHT22
#define DHTTIME 1000        // (ms) wait atleast 2 sec between measurements
#define DHTFAHRENHEIT true  // Fahrenheit (true) or Celsius (false)
// PIR
#define PIRLED 13           // LED output pin
#define PIRPIN 6            // arduino digital pin
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
// Camera
#define CHIPSELECTPIN 10    // arduino digital pin
#define CAMERATXPIN 2       // arduino digital pin
#define CAMERARXPIN 3       // arduino digital pin
#define CAMERAIMGSIZE VC0706_320x240  // camera image size
#define CAMERAMOTIONDETECT true // camera motion detection
// Data Logger
#define LOG_INTERVAL 10000  // (ms) time between log entries
#define SYNC_INTERVAL 1000  // (ms) interval to write data

/* Variables */
int ledState = LOW;         // LED state
int pirState = LOW;         // PIR state
int gasState = LOW;         // gas state
int fireState = 0;          // fire detected state
File photo;                 // camera photo
File logfile;               // log file
time_t t;                     // time instance
SoftwareSerial cameraconnection = SoftwareSerial(CAMERATXPIN, CAMERARXPIN); // camera connection
int *digitalPins;
int *analogPins;
int *digitalStates;
int *analogStates;
int syncTime;

/* Objects */
DHT dht(DHTPIN, DHTTYPE);   // thermometer
DS1307RTC  rtc;             // real time clock
Adafruit_VC0706 cam = Adafruit_VC0706(&cameraconnection); // Camera

/**
 * Error
 * @param char str
 */
void error(char *str)
{
  Serial.print("[ERROR] ");
  Serial.print(str);
  Serial.print("\n");    
  while(1);
}

/**
 * Generate filename
 * @return char*
 */
char* generateFileName(char* override = "") {
  if (VERBOSE) {
    Serial.println("[VERBOSE] Entered generateFileName.");
  }

  if (override != "") {
    return override;
  }

  char* name;

  sprintf(
    name,
    "%d_%d_%d_%d_%d_%d",
    month(),
    day(),
    year(),
    hour(),
    minute(),
    second());

  return name;
}

/**
 * Initialize SD card
 * @return
 */
void initializeSD() {
  if (VERBOSE) {
    Serial.print("[VERBOSE] Entered initializeSD.\n");
  }

  Serial.print("Initializing SD card...");
  pinMode(CHIPSELECTPIN, OUTPUT);
  
  // Check if card exists
  if (!SD.begin(CHIPSELECTPIN)) {
    error("Card failed, or not present.");
    return;
  }

  Serial.println("card initialized.");
  
  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (!SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;
    }
  }
  
  if (!logfile) {
    error("could not create SD file.");
  }
  
  Serial.print("Logging to: ");
  Serial.print(filename);
  Serial.print("\n");
}

/**
 * Initialize camera
 * @return null
 */
void initializeCamera() {
  if (VERBOSE) {
    Serial.println("[VERBOSE] Entered initializeCamera.");
  }
  
  #if !defined(SOFTWARE_SPI)
    if(CHIPSELECTPIN != 10) pinMode(CHIPSELECTPIN, OUTPUT); // SS
  #endif

  // Location camera
  if (cam.begin()) {
    Serial.print("Camera Found:\n");
  } else {
    Serial.print("No camera found!");
    return;
  }

  // Camera information
  char* reply = cam.getVersion();
  if (reply != 0) { // Output info if present
    Serial.print("---------------\n");
    Serial.print(reply);
    Serial.print("---------------\n");
  }

  // Image size
  cam.setImageSize(CAMERAIMGSIZE);
  uint8_t imgSize = cam.getImageSize();
  char* dimensions;
  Serial.print("Camera image size: ");
  if (imgSize == VC0706_640x480) {
      dimensions = "640x480";
  } else if (imgSize == VC0706_320x240) {
    dimensions = "320x240";
  } else if (imgSize == VC0706_160x120) {
    dimensions = "160x120";
  } else {
    dimensions = "unknown";
  }
  sprintf(dimensions, "Camera image size: %s", dimensions);
  Serial.print(dimensions);  

  // Motion detection
  cam.setMotionDetect(CAMERAMOTIONDETECT);
  Serial.print("Camera motion detection: ");
  if (cam.getMotionDetect()) {
    Serial.print("ON\n");
  }
  else {
    Serial.print("OFF\n");
  }

  Serial.print("Camera initialized.\n");
}

/**
 * Run camera
 * @return null
 */
void runCamera() {
  // Check for motion - if disabled, will return false
  if (cam.motionDetected()) {
   Serial.println("Motion!");   
   cam.setMotionDetect(false);
  
  // Capture photo 
  if (!cam.takePicture()) 
    Serial.print("Failed to snap photo!\n");
  else 
    Serial.print("Picture taken.\n");
  
  char* filename = generateFileName("");
  // create if does not exist, do not open existing, write, sync after write
  if (! SD.exists(filename)) {
    Serial.print("ERROR: filename already exists!\n");
    return;
  }
  
  File imgFile = SD.open(filename, FILE_WRITE);
  
  uint16_t jpglen = cam.frameLength();
  Serial.print(jpglen, DEC);
  Serial.print(" byte image\n");
  Serial.print("Writing image to ");
  Serial.print(filename);
  
  while (jpglen > 0) {
    // read 32 bytes at a time;
    uint8_t *buffer;
    uint8_t bytesToRead = min(32, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
    buffer = cam.readPicture(bytesToRead);
    imgFile.write(buffer, bytesToRead);

    /*
    Serial.print("Read ");
    Serial.print(bytesToRead, DEC);
    Serial.println(" bytes");
    */

    jpglen -= bytesToRead;
  }
  imgFile.close();
  Serial.print("Image saved.\n");
  cam.resumeVideo();
  cam.setMotionDetect(true);
 }
}

/**
 * Float to int hack
 *  Arduino Uno chipset does not support floats being 
 *  used with sprintf this is half of the workaround
 * @param  float f
 * @return int
 */
int floatToIntHack(float f) {
  if (VERBOSE) {
    Serial.println("[VERBOSE] Entered floatToIntHack.");
  }

  int i = f * 100;
  return i;
}

/**
 * Initialize RTC
 */
void initializeRTC() {
  if (VERBOSE) {
    Serial.println("[VERBOSE] Entered initializeRTC.");
  }

  Serial.print("Initializing Real Time Clock...\n");
  // Init RTC in 24 hour mode
  // if(!rtc.begin(CHIPSELECTPIN)) {
  //   logFile.println("RTC failed.");
  // } 
  setSyncProvider(RTC.get);

  Serial.print("RTC Initialized.\n");
}

/**
 * Initialize speaker
 */
void initializeSpeaker() {
  if (VERBOSE) {
    Serial.print("[VERBOSE] Entered initializeSpeaker.");
  }

  Serial.print("Initializing speaker...\n");

  pinMode(SPEAKERPIN, OUTPUT);
  delay(1000);
  Serial.print("Speaker Initialized.\n");
}

/**
 * Read DHT sensor
 * @return null
 */
void readDHT() {
  if (VERBOSE) {
    Serial.println("[VERBOSE] Entered readDHT.");
  }
  
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
  if (VERBOSE) {
    Serial.println("[VERBOSE] Entered initializePIR.");
  }

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
  if (VERBOSE) {
    Serial.println("[VERBOSE] Entered readPIR.");
  }

  int pir = digitalRead(PIRPIN);
  
  if (pir == HIGH) {        // If motion is detected
    if (pirState == LOW) {  // If there was not previous motion
      pirState = HIGH;      // toggle state
      Serial.print("Motion detected.\n");
      logfile.println("Motion detected.");
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
  if (VERBOSE) {
    Serial.println("[VERBOSE] Entered readMQ2.");
  }

  int gasLvl = analogRead(MQ2PIN);

  // if levels are too high...
  if (gasLvl > 500 || gasState == HIGH) {
    gasState = HIGH;
    char *msg;
    sprintf(msg, "Gas detected! [%d]\n", gasLvl);
    Serial.print(msg);
    logfile.print(msg);
    toggleAlarm(GASALARM);
  } else if (gasLvl <= 200 && gasState == HIGH) {
    gasState = LOW;
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
  if (VERBOSE) {
    Serial.println("[VERBOSE] Entered readFire.");
  }

  // Map sensor range
  int range = map(analogRead(FIREPIN), FIREMIN, FIREMAX, 0, 3);
  char *msg;

  switch (range) {
    case 0: // A fire within 1.5 feet
      msg = "Close fire!\n";
      Serial.print(msg);
      logfile.print(msg);
      if(!fireState) {
        fireState = 1;
        toggleAlarm(FIREALARM);
      }
      break;
    case 1: // A fire between 1.5 and 3 feet
      msg = "Distant fire detected.\n";
      Serial.print(msg);
      logfile.print(msg);
      if(!fireState) {
        fireState = 1;
        toggleAlarm(FIREALARM);
      }
      break;
    case 2: // No fire
      if(fireState) {
        msg = "All clear.\n";
        Serial.print(msg);
        logfile.print(msg);
        fireState = 0;
        toggleAlarm(false);
      }
      break;
  }
  delay(FIRETIME);
}

/**
 * Toggle alarms
 * @param  int alarm
 */
void toggleAlarm(int alarm) {
  if (VERBOSE) {
    Serial.println("[VERBOSE] Entered toggleAlarm.");
    Serial.print("[VERBOSE] Passed param: alarm = ");
    Serial.print(alarm);
    Serial.print("\n");
  }

  char* msg;

  switch(alarm) {
    case FIREALARM:
      msg = "alarm activated. [fire]\n";
      Serial.print(msg);
      logfile.print(msg);
/** @todo separate tones for close and distant */
//      digitalWrite(SPEAKERPIN, HIGH);
      playTone(500, 600);
      delay(50);
      playTone(500, 600);
      delay(100);
    break;
    case GASALARM:
      msg = "alarm activated. [gas]\n";
      Serial.print(msg);
      logfile.print(msg);
//      digitalWrite(SPEAKERPIN, HIGH);
      playTone(500, 400);
      delay(100);
      playTone(500, 600);
      delay(100);
    break;
    case false:
      msg = "alarm deactivated.";
      Serial.print(msg);
      logfile.print(msg);
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
  if (VERBOSE) {
    Serial.println("[VERBOSE] Entered playTone.");
    Serial.print("[VERBOSE] Passed Param: duration = ");
    Serial.print(duration);
    Serial.print("\n[VERBOSE] Passed Param: freq = ");
    Serial.print(freq);
    Serial.print("\n");
  }

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
  // Debugging
  if (DEBUG) {
    SerialDebugger.debug(NOTIFICATION,"loop","notifications are enabled");
    if (SerialDebugger.debug(ERROR,"loop","errors are disabled")) {
      /** @TODO reset vars, etc */
    }
  }

  // Wait
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));

  t = now();
  // Log time
  // logfile.print(now.unixtime()); // seconds since 1/1/1970
  // logfile.print(", ");
  logfile.print('"');
  logfile.print(year(), DEC);
  logfile.print("/");
  logfile.print(month(), DEC);
  logfile.print("/");
  logfile.print(day(), DEC);
  logfile.print(" ");
  logfile.print(hour(), DEC);
  logfile.print(":");
  logfile.print(minute(), DEC);
  logfile.print(":");
  logfile.print(second(), DEC);
  logfile.print('"');

  readDHT();
  readPIR();
  readMQ2();
  readFire();
  runCamera();

  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  
  logfile.flush();
}

/**
 * Setup
 */
void setup() {
  if (DEBUG) {
    SerialDebugger.begin(9600);
    SerialDebugger.enable(NOTIFICATION);
    SerialDebugger.enable(ERROR);
  }
  Serial.begin(9600);
  Serial.println("RaggedPi Project Codename Cinnamon Initialized.");
  
  SPI.begin();
  dht.begin();
  Wire.begin();
  initializeRTC();
  initializePIR();
  initializeSpeaker();
  initializeSD();
  initializeCamera();

  logfile.println("millis,stamp,datetime,light,temp,vcc");    

}