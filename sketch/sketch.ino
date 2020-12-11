
// Web Dependencies 
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <ArduinoJson.h>

// Internet
// ********************************
  byte mac[] = { 0xAE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; 

  // Set the static IP address to use if the DHCP fails to assign
  IPAddress ip(192, 168, 0, 177);
  IPAddress myDns(192, 168, 0, 1);
// ********************************

//  Time Dependencies 
// ********************************

unsigned int localPort = 8888;
const char timeServer[] = "time.nist.gov";
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];

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

// Tick rate (10 seconds)
long interval = 30000; 

// One hour
long calibrationInterval = 3600000;

// Direction
float direction = 1; // 1 = rising | -1 = falling

// Last low tide (Nov 27 @ 10:03am)
// Periodically updated by calibrate()
long lastlow = 1606471380;

unsigned long startTimer;
unsigned long calibrationTimer;
unsigned long currentTime;

boolean timeSet = false;
boolean calibrated = false;
long bootTime;
long systemTime;

// ********************************

// Setup
void setup() {

  Serial.begin(9600);

  //Board setup
  Ethernet.init(10);

  // Set PinModes
  //setPinModes();
  bootUp(); 
}

void bootUp(){
  Serial.println("Initializing...");
  getTimeFromNTP();
  calibrate();
}

void loop() {

  while(!timeSet){return;}

  currentTime = millis();

  // Calibrate
  if ( millis() - calibrationTimer >= calibrationInterval ){
    log();
    calibrationTimer = millis();
  }

  if (millis() - startTimer >= interval) {
     // Raise and lower t value
    setTidePosition(interval);
    lightPins();

    systemTime = bootTime
    
    //Reset timer
    startTimer = (millis()/1000) + bootTime;
  }
  
}

void setTidePosition(int interval){   

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

          // Logging
          Serial.print ("Position (%): ");
          Serial.print ((t/lengthOfTideCycle) * 100);
          Serial.print (" | Channel: ");
          Serial.print (channelNumber);
          Serial.print (" | Value: ");
          Serial.println (value);
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
}

void calibrate(){
  // Set start time
  startTimer = millis();
  calibrationTimer = millis();

  // Send HTTP request
  if (!Ethernet.begin(mac)) {
    Serial.println(F("Failed to configure Ethernet"));
    return;
  }
  // Connect to HTTP server
  EthernetClient client;
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
  long timeSinceLastLow = bootTime - lastlow;

  Serial.print(F("Seconds since last low: "));
  Serial.println(timeSinceLastLow);

  long remainder;
  // Remove finished cycles
  if( timeSinceLastLow > lengthOfTideCycle ){
    remainder = timeSinceLastLow % lengthOfTideCycle;
  }
  else{
    Serial.print(F("Setting remainder directly: "));
    Serial.println(timeSinceLastLow);
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

  calibrated = true;

  // Disconnect
  client.flush();
  client.stop();

  doc.clear();
}

// Request time from Network Time Protocol
// https://www.arduino.cc/en/Tutorial/LibraryExamples/UdpNtpClient
void getTimeFromNTP(){

  Serial.println(F("Getting time from NTP..."));

  // start Ethernet and UDP
  if (Ethernet.begin(mac) == 0) {
      Serial.println("Failed to configure Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
      
    } else if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }

    Serial.println("Ethernet did not init.");
  }

  Udp.begin(localPort);

  delay(2000);

  sendNTPpacket(timeServer); 
  delay(2000);

  if (Udp.parsePacket()) {
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    // the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, extract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
   
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);
    bootTime = epoch;

    //Set boolean
    timeSet = true;

    Udp.stop();
  }
  else{
    Ethernet.maintain();
    delay(1000);
    getTimeFromNTP();
  }
}

// send an NTP request to the time server at the given address
void sendNTPpacket(const char * address) {
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
  Udp.beginPacket(address, 123); // NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void log(){

  Serial.println(F("Logging..."));

  // Send HTTP request
  if (!Ethernet.begin(mac)) {
    Serial.println(F("Failed to configure Ethernet"));
    return;
  }

  // Connect to HTTP server
  EthernetClient client;
  client.setTimeout(1000);

  if (!client.connect("dweet.io", 80)) {
    Serial.println(F("Connection failed"));
    return;
  }

  // Prepare JSON document
  DynamicJsonDocument doc(255);
  doc["position"] = (t / lengthOfTideCycle) * 100;
  doc["millis"] = millis();
  doc["memory"] = freeRam();
  
  client.println(F("POST /dweet/for/arduino-tide-metrics-v1 HTTP/1.1"));
  client.println(F("Host: dweet.io:443"));
  client.println(F("Content-Type: application/json"));
  client.println(F("Connection: close"));
  client.print(F("Content-Length: "));
  client.println(measureJsonPretty(doc));
  client.println();
  
  serializeJsonPretty(doc, client);

  // Disconnect
  client.flush();
  client.stop();

  // Clear memory
  doc.clear();

  return;

}

int freeRam () {
  // Use 1024 with ATmega168
  int size = 2048;
  byte *buf;
  while ((buf = (byte *) malloc(--size)) == NULL);
      free(buf);
  return size;
}