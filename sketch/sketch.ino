#include <TimeLib.h>

// Web Dependencies 
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <ArduinoJson.h>
#include <Adafruit_SleepyDog.h>

// Internet
// ********************************
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x82, 0x00 }; 

// NTP Servers:
IPAddress timeServer(216, 239, 35, 0);
const int timeZone = 0;

// ********************************
//  Time Dependencies 
// ********************************

unsigned int localPort = 8888;
EthernetUDP Udp;
EthernetClient client;

// Set the static IP address to use if the DHCP fails to assign
IPAddress ip(192, 168, 0, 177);
IPAddress myDns(192, 168, 0, 1);

// ********************************

// Pin Assignments
int outputPins[] = {3, 5, 6, 9, A0, A1}; // Ethernet Board
//int outputPins[] = {0, 1, 2, 3, 4, 5};
float numOfChannels = 6;
long lengthOfTideCycle = 22350; // Tide cycle ( L -> H  ) in seconds (22350). Shorten for debug

// Tide Logic Variables
// t: seconds elapsed in current cycle (placeholder)
float t = 200;

// Tick rate (5 seconds)
long interval = 5000; 

// 4 hours
long calibrationInterval = (60000 * 60 * 4);

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

setPinModes();

// Set start time
startTimer = millis();
calibrationTimer = millis();

Serial.begin(9600);
Serial.println("Initializing...");

// Emable watchdog
// https://github.com/adafruit/Adafruit_SleepyDog/blob/master/examples/BasicUsage/BasicUsage.ino
Watchdog.enable(8000);

//Board setup
Serial.println("Ethernet begin...");

  // Open ethernet connection
if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.");
      return;
    }
    if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
    // try to congifure using IP address instead of DHCP:
  } else {
    Serial.print("DHCP assigned IP ");
    Serial.println(Ethernet.localIP());

    Udp.begin(localPort);

    Serial.println("Getting Time via NTP");
    setSyncProvider(getNtpTime);
    setSyncInterval( 60 * 60 * 2 ); // sync time every 2 hours 
    getLastLowTide();
  }
  
  // Calulate t
  calibrate();
}

void loop() {

  currentTime = millis();

  // Tick
  if (currentTime - startTimer >= interval) {
    //Reset timer
    startTimer = currentTime;
    setTidePosition();
    lightPins();

     Ethernet.maintain();
  }

  // Calibrate
  if (currentTime - calibrationTimer >= calibrationInterval) {
    //Reset timer
    calibrationTimer = currentTime;
    
    getLastLowTide();
    calibrate();
  }

  Watchdog.reset();
 
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

          // Logging
          Serial.print ((t/lengthOfTideCycle) * 100);
          Serial.print ("% | Value: ");
          Serial.print (value);
          Serial.print (" | RAM: ");
          Serial.println (freeRam());    
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

  // Disable SD SPI
  pinMode(4,OUTPUT);
  digitalWrite(4,HIGH);
}

void getLastLowTide(){

  client.stop();

  Serial.println(F("Getting last low"));

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.");
    delay(10000); // Trigger watchdog
    return;
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
    delay(10000); // Trigger watchdog
    return;
  }
    
  Watchdog.reset();

  if (!client.connect("arduino-tide-sculpture.s3-eu-west-1.amazonaws.com", 80)) {
    Serial.println(F("Connection failed"));
    delay(10000); // Trigger watchdog
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

  Watchdog.reset();

  Serial.println(F("Parsing response..."));

  // Check HTTP status
  char status[32] = {0};
  client.readBytesUntil('\r', status, sizeof(status));
  if (strcmp(status, "HTTP/1.1 200 OK") != 0) {
    Serial.print(F("Unexpected response: "));
    Serial.println(status);
    return;
  }

  Watchdog.reset();

  // Skip HTTP headers
  char endOfHeaders[] = "\r\n\r\n";
  if (!client.find(endOfHeaders)) {
    Serial.println(F("Invalid response"));
  }

  Serial.println(F("Handling JSON..."));

  Watchdog.reset();

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

  Serial.println(F("Setting lastlow..."));
  lastlow = doc["lastlow"];

  doc.clear();

  return;
}

void calibrate(){

  if(!timeSet){ Serial.print(F("Time not set. Not calibrating."));  return; }

  // current time as recived from NTP just now
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