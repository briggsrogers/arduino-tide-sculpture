#define BLYNK_PRINT Serial // Defines the object that is used for printing
//#define BLYNK_DEBUG        // Optional, this enables more detailed prints

#include <TimeLib.h>

// Web Dependencies 
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <BlynkSimpleEthernet.h>
#include <ArduinoJson.h>

// Blynk
BlynkTimer timer;

// Internet
// ********************************
byte mac[] = { 0xAE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; 

// NTP Servers:
IPAddress timeServer(216, 239, 35, 0);
const int timeZone = 0;

// Blynk
char auth[] = "G3jTchHrHfKQZm4jcQ3hXmo0Rs3rAujd";

#define W5100_CS  10
#define SDCARD_CS 4


// ********************************
//  Time Dependencies 
// ********************************

unsigned int localPort = 8888;
EthernetUDP Udp;

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

// ********************************

// Setup
void setup() {

  Serial.begin(9600);

  // Deselect the SD card
  pinMode(SDCARD_CS, OUTPUT);
  digitalWrite(SDCARD_CS, HIGH); 
  setPinModes();

  // Serial.println("Getting Time via NTP");
  setSyncProvider(getNtpTime);

  // Calulate t
  calibrate();

  Blynk.begin(auth);

  // Setup a function to be called every second
  timer.setInterval(interval, setTidePosition);
  timer.setInterval((interval * 2), lightPins);
  
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

      if(t < 1/lengthOfTideCycle){
         Serial.print ("Position (%): ");
         Serial.println (t/lengthOfTideCycle );
      }
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
  // Set start time
  startTimer = millis();
  calibrationTimer = millis();

  // Client
  EthernetClient client;

  // Send HTTP request
  if (!Ethernet.begin(mac)) {
    return;
  }

  client.setTimeout(10000);
  if (!client.connect("arduino-tide-sculpture.s3-eu-west-1.amazonaws.com", 80)) {
    Serial.println(F("Connection failed"));
    return;
  }

  Serial.println(F("Connected!"));
  client.println(F("GET /lastlowtide.json HTTP/1.1"));
  client.println(F("Host: arduino-tide-sculpture.s3-eu-west-1.amazonaws.com"));
  client.println(F("Connection: close"));
  if (client.println() == 0) {
    Serial.println(F("Failed to send request"));
    return;
  }

  // Check HTTP status
  char status[32] = {0};
  client.readBytesUntil('\r', status, sizeof(status));
  if (strcmp(status, "HTTP/1.1 200 OK") != 0) {
    Serial.print(F("Unexpected response: "));
    Serial.println(status);
    return;
  }

  // Skip HTTP headers
  char endOfHeaders[] = "\r\n\r\n";
  if (!client.find(endOfHeaders)) {
    Serial.println(F("Invalid response"));
    return;
  }

  // Allocate the JSON document
  // Use arduinojson.org/v6/assistant to compute the capacity.
  const size_t capacity = 210;
  DynamicJsonDocument doc(capacity);

  // Parse JSON object
  DeserializationError error = deserializeJson(doc, client);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  // Extract values
  Serial.println((doc["lastlow"].as<long>()));

  lastlow = doc["lastlow"];

  Serial.println(F("Recieved last low: "));
  Serial.println(lastlow);

  // current time as recived from NTC just now
  long timeSinceLastLow = now() - lastlow;

  Serial.print(F("Seconds since last low: "));
  Serial.println(timeSinceLastLow);

  long remainder;
  // Remove finished cycles
  if( timeSinceLastLow > lengthOfTideCycle ){
    remainder = timeSinceLastLow % lengthOfTideCycle;
  }
  else{
    remainder = timeSinceLastLow;
  }

  Serial.print(F("Completed cycles since low: "));
  Serial.println(timeSinceLastLow / lengthOfTideCycle);

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

  // Disconnect
  client.flush();
  client.stop();

  doc.clear();

  // Light pins 
  // (otherwise we have to wait for first interval)
  lightPins();
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

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response.");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}