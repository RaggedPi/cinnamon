/*
  RaggedPi Project
  Arduino 2 "Cinnamon" - Security Camera
  Written by david durost <david.durost@gmail.com>
*/
#include <Adafruit_VC0706.h>
#include <SecurityCamera.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <DHT.h>
#include <Fire.h>
#include <Gas.h>
#include <Alarm.h>
#include <DS3231.h>
//#include <Utility.h>

// Debugging
// #define DEBUG                   // uncomment for debugger
#define XBEE                    // comment to disable xbee transmission
#define LOG                     // comment to disable datalogging
#define SERIAL                  // comment to mute serial output
// Constants
#define SDA 20                  // sda
#define SDL 21                  // sdl
#define CS 10                   // chipselect
#define LED 13                  // led pin
#define SI 4                    // selectinput
#define DHTPIN 4                // digital pin
#define FIREPIN 1               // analog pin
#define MQ2PIN 0                // analog pin
#define FAHRENHEIT true         // Celcius or fahrenheit
#define LOG_INTERVAL 10000      // (ms) time between log entries
#define SYNC_INTERVAL 1000      // (ms) interval to write data
#define SPEAKER 2
File logfile;                   // data logging
int syncTime;                   // log sync timer
char* msg;                      // reused variable
DHT dht(DHTPIN, DHT11);         // themometer/ barometer
DS3231 rtc(SDA, SCL);           // real time clock
SecurityCamera cam();           // camera
Alarm alarm(SPEAKER);                    // alarm
#ifdef XBEE
SoftwareSerial XBee(SDA, SDL);
#endif

void loop() {
    Time now = rtc.getTime();
    int tmp;
    // Log time
    logfile.print(now);
    cam.run();
    //delay((LOG_INTERVAL-1) - (millis()%LOG_INTERVAL));
    tmp = fire.read();
    if (fire.isOn()) {
        println("Fire detected!");
        alarm = Alarm(FIREPIN, HIGH, tmp);
    }
    tmp = gas.read();
    if (gas.isOn()) {
        sprintf(msg, "Gas levels detected. [%d]", tmp);
        println(msg);
        alarm = Alarm(MQ2PIN, HIGH, GAS)
    }
    dht.read();

    if ((millis() - syncTime) < SYNC_INTERVAL)  return;
    syncTime = millis();

    logfile.flush();
}

void setup() {
    Serial.begin(9600);
    #ifdef XBEE
    XBee.begin(9600);
    #endif

    pinMode(LED, OUTPUT);
    pinMode(CS, OUTPUT);

    rtc.begin();
    dht.begin();
    cam.begin();
    fire.begin();
    gas.begin();

    // SD
    print("Initializing SD card");
    dotdotdot(10, ".", 100, true);
    if (!SD.begin(CS))  error("SD card failed or is missing");
    println("done");
    // Logging
    char filename[] = "LOG00.csv";
    for (uint8_t i = 0; i < 100; i++) {
        filename[3] = i / 10 + '0';
        filename[4] = i % 10 + '0';
        if (!SD.exists(filename)) {
            logfile = SD.open(filename, FILE_WRITE);
            break;
        }
    }
    if (!logfile)   error("log file creation failed.  System will continue.", false);
    else {
        print("Logging to: ");
        println(filename);
    }
    println("System initialized.");
}