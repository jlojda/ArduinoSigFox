/*
  example_Arduino_temperature.cpp - SigFox SFM10R1 library - example code that is
  intended to be run on an Arduino - gets a temperature and sends it over SigFox
  Copyright (c) 2019 Jakub Lojda.  All right reserved.
*/
/*
// include libs
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <SigFox.h>

//output pins
#define LED_PIN 13
#define SIGFOX_RX 3
#define SIGFOX_TX 4

// helper functions
#define seconds() (millis()/1000)

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // start serial for debugging purposes
    Serial.begin(9600);
}


void loop() {
    // create a SW Serial for SigFox connection purposes
    SoftwareSerial sws(SIGFOX_RX, SIGFOX_TX);
    
    // new SigFox object
    SigFox sf(sws);
    
    // new Frame and ErrorType
    SigFox::Frame f;
    SigFox::ErrorType::type e;

    // variable to store the temperature
    int tempInt;

    // it is important to run the begin method over the SigFox object
    // before we can use it
    sf.begin();

    Serial.println("Started...");

    // infinite loop
    for(;;) {
        // every 10 minutes (600 seconds) do a measure and send
        // the resulting temperature
        if (!(seconds() % 600)) {
            digitalWrite(LED_PIN, HIGH);
            
            e = sf.GetTemperatureInt(&tempInt);
            if (e) Serial.println("GetTemperatureInt: ERROR");
            else Serial.println("GetTemperatureInt: OK");

            
            // print the temperature value
            Serial.print("tempInt=");
            Serial.println(tempInt);

            
            e = f.Format("u16", tempInt);
            if (e) Serial.println("Format: ERROR");
            else Serial.println("Format: OK");

            
            e = sf.SendFrame(&f);
            if (e) Serial.println("SendFrame: ERROR");
            else Serial.println("SendFrame: OK");
            
            digitalWrite(LED_PIN, LOW);
        }
        
        Serial.println("Waiting...");
        
        // check the time every 1 second
        delay(1000);
  }
  
  return;
}

*/