// RaggedPi Project
// Arduino 2 "Cinnamon"
// Written by david durost <david.durost@gmail.com>

// Includes
#include <DHT.h>
#include <SD.h>
#include <time.h>
#include <string.h>

// Defines
#define DHTPIN 4 // arduino digital pin
#define DHTTYPE DHT11
#define DHTTIME 2000 // wait 2 sec between measurements
#define DHTFAHRENHEIT true // fahrenheit (true) or celsius (false)

#define PIRLED 13 // LED output pin
#define PIRPIN 2 // arduino digital pin
#define PIRINIT 30 // sensor intialization time

#define MQ2PIN A0 // arduino analog pin

#define VC0706_PROTOCOL_SIGN      0x56
#define VC0706_SERIAL_NUMBER      0x00
#define VC0706_COMMAND_RESET      0x26
#define VC0706_COMMAND_GEN_VERSION    0x11
#define VC0706_COMMAND_TV_OUT_CTRL    0x44
#define VC0706_COMMAND_OSD_ADD_CHAR   0x45
#define VC0706_COMMAND_DOWNSIZE_SIZE    0x53
#define VC0706_COMMAND_READ_FBUF    0x32
#define FBUF_CURRENT_FRAME      0
#define FBUF_NEXT_FRAME       0
#define VC0706_COMMAND_FBUF_CTRL    0x36
#define VC0706_COMMAND_COMM_MOTION_CTRL   0x37
#define VC0706_COMMAND_COMM_MOTION_DETECTED 0x39
#define VC0706_COMMAND_POWER_SAVE_CTRL    0x3E
#define VC0706_COMMAND_COLOR_CTRL   0x3C
#define VC0706_COMMAND_MOTION_CTRL    0x42
#define VC0706_COMMAND_WRITE_DATA   0x31
#define VC0706_COMMAND_GET_FBUF_LEN   0x34
#define READ_DATA_BLOCK_NO      56

unsigned char tx_counter;
unsigned char tx_vcbuffer[20];
bool tx_ready;
bool rx_ready;
unsigned char   rx_counter;
unsigned char   VC0706_rx_buffer[80]; 
uint32_t frame_length=0;
uint32_t vc_frame_address =0;
uint32_t last_data_length=0;
File myFile;
const uint8_t SdChip = SS; // where SD card is connected [seeed = SS, ethernet shield = 4]
char myFileName[16];
int myFileNr=1;

// Initializations
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  Serial.println("RaggedPi Project Codename Cinnamon Initialized.");
  
  dht.begin();
  SD.begin(SdChip);
  initializePIR();
  initializeCamera();

}

void loop() {
  readDHT();
  readPIR();
  readMQ2();
}

// Generate current datetime
char *datetime() {
  char *array = (char*)malloc(sizeof(char)*25);
  time_t t = time(NULL);
  sprintf(array, "%s", asctime(localtime(&result)));
  array[25] = '\0';
  return array;
}

// Get formatted datetime
char *getDateTime(String format = "%y_%m_%d_%H_%M_%S") {
  char *array(char*)malloc(sizeof(char)*20);
  memset (array, 0, 20);
  time_t t = time(NULL);
  struct tm *timeinfo = localtime(&t);
  strftime(array, 20, format, timeinfo);
  array[20] = '\0';
  return array;
}

// Explode a string into an array
char** strToArray(char* a_str, const char a_delim)
{
    char** result    = 0;
    size_t count     = 0;
    char* tmp        = a_str;
    char* last_comma = 0;
    char delim[2];
    delim[0] = a_delim;
    delim[1] = 0;

    /* Count how many elements will be extracted. */
    while (*tmp)
    {
        if (a_delim == *tmp)
        {
            count++;
            last_comma = tmp;
        }
        tmp++;
    }

    /* Add space for trailing token. */
    count += last_comma < (a_str + strlen(a_str) - 1);

    /* Add space for terminating null string so caller
       knows where the list of returned strings ends. */
    count++;

    result = malloc(sizeof(char*) * count);

    if (result)
    {
        size_t idx  = 0;
        char* token = strtok(a_str, delim);

        while (token)
        {
            assert(idx < count);
            *(result + idx++) = strdup(token);
            token = strtok(0, delim);
        }
        assert(idx == count - 1);
        *(result + idx) = 0;
    }

    return result;
}

// Read DHT sensor
void readDHT() {
  delay(DHTTIME);
  
  // Read data (~250ms)
  float humidity = dht.readHumidity();
  float temp = dht.readTemperature(DHTFAHRENHEIT);
  
  // Check for read fails
  if(isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor. (h)");
    return;
  } else if (isnan(temp)) {
    Serial.println("Failed to read from DHT sensor! (t)");
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
  Serial.print(" %\t");
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
  
  Serial.print("Initializing PIR sensor.");
  for (int i; i < PIRINIT; i++) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("PIR sensor initialized.");
  delay(50);
}

// Read PIR sensor
void readPIR() {
  int pir = digitalRead(PIRINPUTPIN);
  int state = LOW;
  
  // If there's motion...
  if (pir = HIGH) {
    digitalWrite(PIRLEDPIN, HIGH);
    if (state == LOW) {
      // Toggle PIR state
      state = HIGH;
      Serial.println("Motion detected.");
      capturePhoto();
    }
  } else {
    digitalWrite(PIRLEDPIN, LOW);
    if (state == HIGH) {
      state == LOW;
      Serial.println("Motion ended");
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

  // Initialize camera module
  void initializeCamera() {
    // Set jpeg quality
    VC0706_compression_ratio(66);
    delay(100);

    // Test photo
    capturePhoto("test.jpg");
  }

  // Capture photograph
  void capturePhoto(char photoName[] = NULL, bool deleteDuplicate = true) {
    
    // Verify/ set photo filename
    if (NULL == photoName) {
      photoName = sprint("%s.jpg", getDateTime());
    }

    // Verify video engine is running
    VC0706_frame_control(3);
    delay(10);
  
    // If the file exists, delete/ rename file
    int x = 2;
    while (SD.exists(photoName)) {
      if (deleteDuplicate) { // Remove old file
        SD.remove(photoName);
      } else { // Rename new file
        temp = strToArray(photoName, '.');
          
        photoName = sprint("%s%d.jpg", temp[0], x);
      } 
      x++;
    }
    
    // Freeze video enginge
    VC0706_frame_control(0);
    delay(10);
    rx_ready=false;
    rx_counter=0;
    
    Serial.end();     // clear all rx buffer
    delay(5);
    
    Serial.begin(115200);

    // Get frame buffer length
    VC0706_get_framebuffer_length(0);
    delay(10);
    buffer_read();
    
    // Store frame buffer length
    frame_length=(VC0706_rx_buffer[5]<<8)+VC0706_rx_buffer[6];
    frame_length=frame_length<<16;
    frame_length=frame_length+(0x0ff00&(VC0706_rx_buffer[7]<<8))+VC0706_rx_buffer[8];

    vc_frame_address =READ_DATA_BLOCK_NO;
      
    photo=SD.open(photoName, FILE_WRITE); 
    while(vc_frame_address<frame_length) { 
      VC0706_read_frame_buffer(vc_frame_address-READ_DATA_BLOCK_NO, READ_DATA_BLOCK_NO);
      delay(9);

      // Get the data with length=READ_DATA_BLOCK_NObytes 
      rx_ready=false;
      rx_counter=0;
      buffer_read();

      // Write data to file
      photo.write(VC0706_rx_buffer+5,READ_DATA_BLOCK_NO);
    
      // Read next READ_DATA_BLOCK_NO bytes from frame buffer
      vc_frame_address=vc_frame_address+READ_DATA_BLOCK_NO;
    
    }

    // Get the last data
    vc_frame_address=vc_frame_address-READ_DATA_BLOCK_NO;

    last_data_length=frame_length-vc_frame_address;

    
    VC0706_read_frame_buffer(vc_frame_address,last_data_length);
    delay(9);
    
    // Get the data 
    rx_ready=false;
    rx_counter=0;
    buffer_read();
        
    photo.write(VC0706_rx_buffer+5,last_data_length);
    
    photo.close();

    // Restart video engine
    VC0706_frame_control(3);
    delay(10);

    Serial.print("Photograph taken. [");
    Serial.print(fileName);
    Serial.print("]\n");
  }
}