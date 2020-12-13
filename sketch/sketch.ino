#define BLYNK_PRINT Serial // Defines the object that is used for printing
//#define BLYNK_DEBUG        // Optional, this enables more detailed prints

// Web Dependencies 
#include <SPI.h>
#include <Ethernet.h>
#include <BlynkSimpleEthernet.h>
#include <TimeLib.h>
#include <WidgetRTC.h>

// Blynk
BlynkTimer timer;
WidgetRTC rtc;

// Blynk
char auth[] = "G3jTchHrHfKQZm4jcQ3hXmo0Rs3rAujd";

#define W5100_CS  10
#define SDCARD_CS 4

// ********************************

// Pin Assignments
int outputPins[] = {3, 5, 6, 9, A0, A1}; // UNO
//int outputPins[] = {0, 1, 2, 3, 4, 5};
float numOfChannels = 6;
long lengthOfTideCycle = 22350; // Tide cycle ( L -> H  ) in seconds (22350). Shorten for debug

// Tide Logic Variables
// ********************************
// t: seconds elapsed in current cycle (placeholder)
float t = 200;

// Tick rate (5 seconds)
long interval = 2000; 

// Direction
float direction = 1; // 1 = rising | -1 = falling

// Last low tide (Nov 27 @ 10:03am)
// Periodically updated by calibrate()
long lastlow = 1606471380;

unsigned long startTimer;
unsigned long calibrationTimer;
unsigned long currentTime;

BLYNK_CONNECTED() {
  // Synchronize time on connection
  rtc.begin();
  setSyncInterval(60 * 60);
}

// Setup
void setup() {

  Serial.begin(9600);

  setPinModes();

  // Deselect the SD card
  pinMode(SDCARD_CS, OUTPUT);
  digitalWrite(SDCARD_CS, HIGH); 

  Blynk.begin(auth, "blynk-cloud.com", 80);

  // Setup a function to be called every second
  timer.setInterval(2000L, setTidePosition);
  timer.setInterval(2500L, lightPins);

  //calibrate();

}

void loop() {
  Blynk.run();
  timer.run(); 
}

void setTidePosition(){   

   // Set t
   float tVal = t + ((interval/1000) * direction); // * by direction to determine increase or decrease

   t = tVal;       
 
   if(t >= lengthOfTideCycle){
    t = lengthOfTideCycle;
    // Flip direction
    direction = -1;
   }

   else if(t <= 0){
    t = 0;
    // Flip direction
    direction = 1;
   }

   Blynk.virtualWrite(V0, (t / lengthOfTideCycle));
}

// Logic to set channel values from 0 - 255
// depending on t's position in tidal cycle
void lightPins(){
  
  // Loop over pins
  for(int i = 0; i < numOfChannels; ++i) {
    int pin = outputPins[i]; // Pin assignment number 
    float channelNumber = i + 1.0; // Index starts at 0 so we add 1 to get channel number

    // Channel 1 is always on full
    if(channelNumber == 1){
       // Logging
      analogWrite(pin, 255); 
     }
     // Other channels
     else{
        // Each channel is concerned with a band of values in the tide cycle based 
        // on its position in the set of channels.
        float channelUpperBound = (channelNumber/numOfChannels) * lengthOfTideCycle;
        float channelLowerBound = channelUpperBound - (lengthOfTideCycle/numOfChannels);

        // If t (time until next high tide) has passed this channel's upper band, set to full brightness
        if(t > channelUpperBound){ analogWrite(pin, 255);  }

        // If t (time until next high tide) has not reached this channel's lower band, set to off
        if(t < channelLowerBound){ analogWrite(pin, 0);  }

        // If is within channel band
        if(t > channelLowerBound && t < channelUpperBound) {
          // How far is t from the lower band of this channel? 
          float distanceFromLowerBound = t - channelLowerBound;
          // What % through the channel band is t? 
          float percentThroughBand = distanceFromLowerBound / (lengthOfTideCycle / numOfChannels);
          // Set channel to this % of 255
          float value = round(255 * percentThroughBand);
          // Write
          analogWrite(pin, value);  
        }
     }
  }
}

// Set PinModes
void setPinModes() {
  //Loop over pins
  for(int i = 0; i < numOfChannels; ++i) {
    int pin = outputPins[i];
    pinMode(pin, OUTPUT);
  }
}

void calibrate(){

  if( timeStatus() == 0){
    return;
  }

  // current time as recived from NTC just now
  long timeSinceLastLow = now() - lastlow;

  long remainder;
  // Remove finished cycles
  if( timeSinceLastLow > lengthOfTideCycle ){
    remainder = timeSinceLastLow % lengthOfTideCycle;
  }
  else{
    remainder = timeSinceLastLow;
  }

  long cyclesCompleted = (timeSinceLastLow / lengthOfTideCycle);

  // If even, rise
  if (cyclesCompleted % 2 == 0){
    t = remainder;
    direction = 1;
  }
  // if odd, fall from top
  else{
    t = lengthOfTideCycle - remainder;
    direction = -1;
  }
}

/*-------- RAM monitor ----------*/

int freeRam () {
  // Use 1024 with ATmega168
  int size = 2048;
  byte *buf;
  while ((buf = (byte *) malloc(--size)) == NULL);
      free(buf);
  return size;
}
