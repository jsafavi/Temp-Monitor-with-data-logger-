
/************************ Adafruit IO Configuration *******************************/

// Adafruit info
#define IO_USERNAME    "jsafavi"
#define IO_KEY         "1975f53624e2433a992dbe59abc8b73a"

/******************************* WIFI Configuration *******************************/

#define WIFI_SSID       "agiwifi"
#define WIFI_PASS       "wireless"

#include "AdafruitIO_WiFi.h"
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

/************************ List of Libraries Included ****************************/

#include <ESP8266WiFi.h>          //related to IoT
#include <AdafruitIO.h>
#include <Adafruit_MQTT.h>
#include <ArduinoHttpClient.h>

#include <Arduino.h>            //related to sensors 
#include <Wire.h>
#include "Adafruit_MCP9808.h"
#include "RTClib.h"
#include <SPI.h>
#include <SD.h>

/************************* Initiating feeds and variables ***********************/

// set up the feeds to adafruit io
AdafruitIO_Feed *red_temp_sensor   = io.feed("red_temp_sensor");
AdafruitIO_Feed *calib_tempsensor     = io.feed("calib_tempsensor");
AdafruitIO_Feed *sampling_tempsensor  = io.feed("sampling_tempsensor");

  
// assigning the sensors/boards
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
RTC_PCF8523 rtc;

// initiating constatns and variables 
unsigned long previousMillis = 0;           // used to count time in the loop (sampling)
unsigned long previousMillis_red = 0;       // used to count time in the loop (red LED blink)
unsigned long previousMillis_obs = 0;       // used to count time in the loop (obstacle sensor)
long interval = 60000;                      // default sampling interval of sensor in milli seconds
long interval_led = 2000;                   // blinking interval for red LED
//long interval_obs = 5000;                   // interval to check the obstacle sensor 
long offset_t  = 0;                         // calibration value for temperature  
const int chipSelect = 15;                  // SD CS pin in ESP8266, used for SD card communication     
//int obs_sensor = 16;                        // pin assigned as input for obstacle sensor
char sdFileName[] = "TempLog.txt";      // name of the file created on SD card where data is saved
float minTemp = 100;            // initiating min temp value for minmaxFunction
float maxTemp = -100;           // initiating max temp value for minmaxFunction

int n = 0;
int interval_hours = 1440;
float maxTemp24array[1440];
float minTemp24array[1440];
float maxTemp24 = -100;
float minTemp24 = 100;
unsigned long time_before = 0;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"}; 

/********************************************************************************/

void setup() {

  // start the serial connection
  Serial.begin(115200);
  pinMode(0, OUTPUT);                       // red LED 
  digitalWrite(0, HIGH);                    // turning off the red red LED
  pinMode(2, OUTPUT);                       // blue LED
  //pinMode(obs_sensor, INPUT);               // pin assignement for obstacle sensor

  // connect to io.adafruit.com
  Serial.print("Connecting to Adafruit IO");
  io.connect();

  // set up a message handler for the 'temp_test' feed.
  // the handleMessage function (defined below)
  // will be called whenever a message is
  // received from adafruit io.
  calib_tempsensor->onMessage(handleMessage4);
  sampling_tempsensor->onMessage(handleMessage8);
  
  // wait for a connection to Adafruit IO (code will not proceed untill connected if this block of code is enabled)
//  while (io.status() < AIO_CONNECTED) {
//    Serial.print(".");
//    delay(500);
//  }

  // we are connected to Adafruit IO
  Serial.println();
  Serial.println(io.statusText());

  // initializing temp sensor
  if (!tempsensor.begin()) {
    Serial.print("Could not find MCP9808 (temp sensor)");
    while (1) delay(1);
  }

  // initializing RTC
  if (! rtc.begin()) {
    Serial.println("Could not find RTC");
    while (1);
  }  
  
  // initializing RTC
  if (! rtc.initialized()) { 
    Serial.println("RTC is NOT running!");
    // next line is responsible for setting the time and date for RTC once sketch is compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // initializing SD card
  Serial.println("Initializing SD card ...");
  if (! SD.begin(chipSelect)) {
    Serial.println("SD card failed or not present");
    return;
  }
  
  for (int j=0 ; j<interval_hours ; j++) {
    maxTemp24array[j] = maxTemp24;
    minTemp24array[j] = minTemp24;
  }
  
  
}

/********************************************************************************/

void loop() {
  
  // reading sensor info and time
  float c = tempsensor.readTempC() + offset_t;
  float f = (c * 1.8) + 32;
  DateTime now = rtc.now();
  unsigned long currentMillis = millis();
  minmaxFunction(c);
  
  if (io.status()== 21) {               // WHAT TO DO IF THERE IS CONNECTION TO ADAFRIOT IO
  
    // io.run(); is required for all sketches.
    // it should always be present at the top of your loop
    // function. it keeps the client connected to
    // io.adafruit.com, and processes any incoming data.
    io.run();
    
    // reading sensor info
    //float c = tempsensor.readTempC() + offset_t;
    //float f = (c * 1.8) + 32;
    //DateTime now = rtc.now();

    //unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
      
      //turning on blue LED (sampling indicator)
      digitalWrite(2, LOW);                       
      previousMillis = currentMillis;

      // printing and uploading sensor info to feeds on Adafruit
      
      Serial.println("\nSending data --> connected to io ");
      Serial.println(io.statusText());
      Serial.println(io.status());
      //Serial.println(AIO_CONNECTED);
      //Serial.println(WiFi.status());
      Serial.print("Sampling at (sec) = ");
      Serial.println(interval/1000);
      Serial.print("Offset  (Temp *C) = ");
      Serial.println(offset_t);
      Serial.print("Max Temp in a day *C =");
      Serial.println(maxTemp24);
      Serial.print("Min Temp in a day *C =");
      Serial.println(minTemp24);
      
      if (!isnan(c)) {
        red_temp_sensor->save(c);
        Serial.print("Temperature in *C = "); Serial.println(c);
        Serial.print("Temperature in *F = "); Serial.println(f);


        // open the file. note that only one file can be open at a time,
        // so you have to close this one before opening another.
        File dataFile = SD.open(sdFileName, FILE_WRITE);

        // if the file is available, write to it:
        if (dataFile) {
          dataFile.print(now.month()); dataFile.print("/"); dataFile.print(now.day()); dataFile.print("/"); dataFile.print(now.year());
          dataFile.print("\t"); dataFile.print(daysOfTheWeek[now.dayOfTheWeek()]); 
          dataFile.print("\t\t"); dataFile.print(now.hour()); dataFile.print(":"); dataFile.print(now.minute());
          dataFile.print("\t"); dataFile.print(c); dataFile.print("\t"); dataFile.print(f);
          dataFile.print("\t"); dataFile.print(minTemp24);
          dataFile.print("\t"); dataFile.print(maxTemp24);
          dataFile.print("\t\t"); dataFile.println(io.statusText());                
          dataFile.flush();
          dataFile.close();
        }
        else {
          Serial.println("Error opening TempLog.txt");
        }
        
      }
      else {
        Serial.println("Failed to Read Tempreture");
      }

      Serial.println(); 
    }
    
    //turning off blue LED (sampling indicator) when not sampling
    else {
      digitalWrite(2, HIGH);                     
    }

    // Blinking red LED (power indicator)
    if (currentMillis - previousMillis_red >= interval_led) {
      previousMillis_red = currentMillis;
      digitalWrite(0, LOW);
      delay(500);  
      digitalWrite(0, HIGH);
    }

//    // checking obstacle sensor to see if fridge door is fully closed or not, at a certain time interval 
//    if (currentMillis - previousMillis_obs >= interval_obs) {
//      previousMillis_obs = currentMillis;
//      if (digitalRead(obs_sensor) == LOW) {
//      obstacle_sensor->save("Fridge door is closed!");
//      Serial.println("Fridge door is closed!");
//      Serial.println(io.statusText());
//      }
//      else {
//      obstacle_sensor->save("ALERT! Fridge door is open!");
//      Serial.println("ALERT! Fridge door is open!");
//      Serial.println(io.statusText());
//      } 
//    }   
  } 
          
  
  
  else {                        // WHAT TO DO IF THERE IS NO CONNECTION TO ADAFRUIT IO
    //float c = tempsensor.readTempC() + offset_t;
    //float f = (c * 1.8) + 32;
    //DateTime now = rtc.now();
        
    //unsigned long currentMillis = millis();
        
    if (currentMillis - previousMillis >= interval) {
            
      //turning on blue LED (sampling indicator)
      digitalWrite(2, LOW);                       
            previousMillis = currentMillis;
        
            // printing and uploading sensor info to feeds on Adafruit
            
            Serial.println("\nSending data --> not connected ");
            Serial.println(io.statusText());
            Serial.println(io.status());
            //Serial.println(AIO_CONNECTED);
            //Serial.println(WiFi.status());
            Serial.print("Sampling at (sec) = ");
            Serial.println(interval/1000);
            Serial.print("Offset  (Temp *C) = ");
            Serial.println(offset_t);
      Serial.print("Max Temp in a day *C =");
      Serial.println(maxTemp24);
      Serial.print("Min Temp in a day *C =");
      Serial.println(minTemp24);
            
            if (!isnan(c)) {
        Serial.print("Temperature in *C = "); Serial.println(c);
        Serial.print("Temperature in *F = "); Serial.println(f);
        
        
        // open the file. note that only one file can be open at a time,
        // so you have to close this one before opening another.
        File dataFile = SD.open(sdFileName, FILE_WRITE);
        
        // if the file is available, write to it:
        if (dataFile) {
                dataFile.print(now.month()); dataFile.print("/"); dataFile.print(now.day()); dataFile.print("/"); dataFile.print(now.year());
                dataFile.print("\t"); dataFile.print(daysOfTheWeek[now.dayOfTheWeek()]); 
                dataFile.print("\t\t"); dataFile.print(now.hour()); dataFile.print(":"); dataFile.print(now.minute());
                dataFile.print("\t"); dataFile.print(c); dataFile.print("\t"); dataFile.print(f);
        dataFile.print("\t"); dataFile.print(minTemp24);
        dataFile.print("\t"); dataFile.print(maxTemp24);
                dataFile.print("\t\t"); dataFile.println(io.statusText());                
                dataFile.flush();
                dataFile.close();
                }
        else {
                Serial.println("Error opening TempLog.txt");
                }
                
            }
            else {
              Serial.println("Failed to Read Tempreture");
            }
        
            Serial.println(); 
    }
    
    //turning off blue LED (sampling indicator) when not sampling
    else {
            digitalWrite(2, HIGH);                       
        }
        
    // Blinking red LED (power indicator)
    if (currentMillis - previousMillis_red >= interval_led) {
            previousMillis_red = currentMillis;
            digitalWrite(0, LOW);
            delay(500);  
            digitalWrite(0, HIGH);
        }
        
//    // checking obstacle sensor to see if fridge door is fully closed or not, at a certain time interval 
//    if (currentMillis - previousMillis_obs >= interval_obs) {
//            previousMillis_obs = currentMillis;
//            if (digitalRead(obs_sensor) == LOW) {
//              Serial.println("Fridge door is closed!");
//              Serial.println(io.statusText());
//            }
//            else {
//              Serial.println("ALERT! Fridge door is open!");
//              Serial.println(io.statusText());
//            }
//    } 
            
  }
}

/********************************************************************************/

// This function is called whenever data is recieved 
// from sampling_test feed on Adafruit IO. It can be 
// found in setup() function above.

void handleMessage8(AdafruitIO_Data *data8) {
  
  interval = (data8->toLong())*1000;
  Serial.print("received new Sampling frequency <-- ");
  Serial.println(interval/1000);
  Serial.println();
}

/********************************************************************************/

// This function is called whenever data is received
// from calib_tempsensor feed on Adafruit IO. it can be 
// found in setup() function above.

void handleMessage4(AdafruitIO_Data *data4) {

  offset_t = data4->toInt();
  //Serial.print("received new temp Calib value <-- ");
  //Serial.println(offset_t);
  //Serial.println();
}

/********************************************************************************/

// This function is tasked with keeping the MIN and MAX value of
// data given to it over a specified period of time.
  //  It takes in input data (such as temp or humidity), and time (in hours).
  //  And it outputs (saves) min and max of the inputted data.

void minmaxFunction(float input_data) {
  
  
  unsigned long time_now = millis();
  long hour_to_millis =  60 * 1000;
  
  // if one hour has passed, the following if statement will happen
  if (time_now - time_before > hour_to_millis) {
    time_before = time_now;
    maxTemp24array[n] = maxTemp;
    minTemp24array[n] = minTemp;
    n++;
    minTemp = 100;
    maxTemp = -100;
    if (n == (interval_hours-1)) {
      n = 0;
    } 
  }
  
  // saving the lowest temp
  if (input_data < minTemp) {
    minTemp = input_data;
  }
  
  // saving the highest temp
  if (input_data > maxTemp) {
    maxTemp = input_data;
  }
  
  for (int i = 0; i<interval_hours; i++) {
    
    if (maxTemp24 < maxTemp24array[i]) {
    maxTemp24 = maxTemp24array[i];
    }
    
    if (minTemp24 > minTemp24array[i]) {
    minTemp24 = minTemp24array[i];
    } 
  }
}

/********************************************************************************/

  

