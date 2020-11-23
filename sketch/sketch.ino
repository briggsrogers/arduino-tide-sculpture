// BR Edit 

// Pin Assignments
int outputPins[] = {3, 5, 6, 9, 10, 11};
float numOfChannels = 6;
float lengthOfTideCycle = 20; // Tide cycle in seconds (44640). Shorten for debug

// Time Remaining (Mins until high tide. Min: 0, max: 744.)
float t = 0;
float interval = 1000; // Tick (ms)

// Direction
float direction = 1; // 1 = rising | -1 = falling

// Setup
void setup() {
  Serial.begin(9600);
  // Set PinMode
  setPinModes();
}

void loop() {

  // Raise and lower t value
  setTidePosition();

  // Set pin values
  lightPins();

  delay(interval);
}

void setTidePosition(){          
 
   if(t > lengthOfTideCycle){
    t = lengthOfTideCycle;
    // Flip direction
    direction = direction * -1;
   }

   else if(t < 0){
    t = 0;
    // Flip direction
    direction = direction * -1;
   }

   // Set t
   float tVal = t + (1 * direction); // Increment or decrement by 1;
   t = tVal;
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

          // Logging
          Serial.print ("Time (seconds): ");
          Serial.println (t);
          Serial.print ("| Channel: ");
          Serial.print (channelNumber);
          Serial.print ("| Value: ");
          Serial.println (value);
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
