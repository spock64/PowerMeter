/*
 PJR - PowerMeter ...


 Sends voltage and power from China-style energy meter to the Internet.
 Collecting data by eavesdropping on the MOSI-line (master in slave out)
 between the energy monitoring chip (ECH1560) and the main processor.

 Based on the amazing work of Karl Hagström at http://gizmosnack.blogspot.co.uk/2014/10/power-plug-energy-meter-hack.html

*/
#include <Arduino.h>
#include <FS.h>
#include <ESP8266WiFi.h>
extern "C" {
  #include "user_interface.h" // for wifi_station_connect
}

//these for WifiManager
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
          // ??? WifiManager bug needs to be unreleased MASTER due to https://github.com/tzapu/WiFiManager/issues/81

// these for OTA upport
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ESP8266HTTPUpdateServer.h>

#include <ArduinoJson.h>
#include <Ticker.h>                                           // For LED status



#define CONFIG_BUTTON 12

const int CLKPin = D2;//4; // Pin connected to CLK (D2 & INT0)
const int MISOPin = D1;//5;  // Pin connected to MISO (D5)

const int SEND_INTERVAL = 1 * 60 * 1000; // 1 minute

//All variables that are changed in the interrupt function must be volatile to make sure changes are saved.
volatile int Ba = 0;   //Store MISO-byte 1
volatile int Bb = 0;   //Store MISO-byte 2
volatile int Bc = 0;   //Store MISO-byte 2
float U = 0;    //voltage
float P = 0;    //power

volatile long CountBits = 0;      //Count read bits
volatile long ClkHighCount = 0;   //Number of CLK-highs (find start of a Byte)
volatile boolean inSync = false;  //as long as we ar in SPI-sync
volatile boolean NextBit = true;  //A new bit is detected

volatile unsigned int isrTriggers; // for debugging to see if ISR routine is being called

float avgVolts, minVolts, maxVolts;
float avgWatts, minWatts, maxWatts;
int numReadings;

unsigned long lastSend;

unsigned long debugOps;

const char* host = "pjr-pm";
const char* ssid = "ROGERS";
const char* password = "*jaylm123456!";

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

// are we running in config or normal run mode ?
#define CONFIG 0
#define RUN 1
int op_mode;

#define CONFIG_MAX_TIME 30000 // 30 secs ...
long t;   // used to remember when we entered config mode ...

bool shouldSaveConfig = false;

const char *wifi_config_name = "RemoteIR"; // Should be Name and number ...
int port = 80;
char passcode[20] = "";
char host_name[20] = "";
char port_str[6] = "80";
char user_id[60] = "";

// Change each time the format changes ...
const char * CONFIG_FILE_VERSION = "0.1";

// Could this be a local in the function?
DynamicJsonBuffer jsonBuffer;
JsonObject& deviceState = jsonBuffer.createObject();

ESP8266WebServer *server = NULL;
HTTPClient http;
Ticker ticker;

const int configpin = D2;//10;          // GPIO10 can't be used on NodeMCU !
const int ledpin = D4;//BUILTIN_LED;    // Built in LED defined for WEMOS people

//+=============================================================================
// Toggle LED state
//
void tick()
{
  int state = digitalRead(ledpin);  // get the current state of BUILTIN_LED pin
  digitalWrite(ledpin, !state);     // set pin to the opposite state
}

//+=============================================================================
// Callback notifying us of the need to save config
//
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

//+=============================================================================
// Gets called when WiFiManager enters configuration mode
//
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

//+=============================================================================
// Gets called when device loses connection to the accesspoint
//
void lostWifiCallback (const WiFiEventStationModeDisconnected& evt) {
  Serial.println("Lost Wifi");
  // reset and try again, or maybe put it to deep sleep

  // PJR - should be better way than this?
  ESP.reset();
  delay(1000);
}

//+=============================================================================
// First setup of the Wifi.
// If return true, the Wifi is well connected.
// Should not return false if Wifi cannot be connected, it will loop    PJR - wtf?
//
bool setupWifi(bool resetConf) {
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.5, tick);

  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  // reset settings - for testing
  if (resetConf)
    wifiManager.resetSettings();

  // set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);
  // set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  // Reset device if on config portal for greater than 3 minutes
  wifiManager.setConfigPortalTimeout(180);

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);

        // Locals ...
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());

        if (json.success())
        {
          Serial.print("json parse complete - ");
          json.printTo(Serial);
          Serial.println(" ...");

          // Check for version keys
          if (json.containsKey("version"))
          {
            if(strcmp(json["version"], CONFIG_FILE_VERSION) == 0)
            {
              // File is OK
              Serial.println("\nConfig file version matches - loading");

              if (json.containsKey("hostname")) strncpy(host_name, json["hostname"], 20);
              if (json.containsKey("passcode")) strncpy(passcode, json["passcode"], 20);
              if (json.containsKey("user_id")) strncpy(user_id, json["user_id"], 60);
              if (json.containsKey("port_str")) {
                strncpy(port_str, json["port_str"], 6);
                port = atoi(json["port_str"]);
              }
            }
            else
            {
              Serial.println("\nConfig file is wrong version\n");
              // *** PJR - so ?
              //Serial.println(json["version"]);
            }
          }
          else
          {
            // No version key ...
            Serial.println("No version key found ...");
            // *** PJR - so ?
          }

        }
        else
        {
          Serial.println("failed to load json config");
          // *** PJR - so ?
        }
      }
    }
  } else {
    // Should do something with the LED?
    Serial.println("failed to mount FS");
  }

  WiFiManagerParameter custom_hostname("hostname", "Choose a hostname to this IR Controller", host_name, 20);
  wifiManager.addParameter(&custom_hostname);
  WiFiManagerParameter custom_passcode("passcode", "Choose a passcode", passcode, 20);
  wifiManager.addParameter(&custom_passcode);
  WiFiManagerParameter custom_port("port_str", "Choose a port", port_str, 6);
  wifiManager.addParameter(&custom_port);

  // fetches ssid and pass and tries to connect
  // if it does not connect it starts an access point with the specified name
  // and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect(wifi_config_name)) {
    Serial.println("Failed to connect and hit timeout");
    // reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  // if you get here you have connected to the WiFi
  strncpy(host_name, custom_hostname.getValue(), 20);
  strncpy(passcode, custom_passcode.getValue(), 20);
  strncpy(port_str, custom_port.getValue(), 6);
  strncpy(user_id,"**PJR NO UID***",60);                      // PJR Remove when HTML sorted ...
  port = atoi(port_str);

// ???
  if (server != NULL) {
    delete server;
  }
  server = new ESP8266WebServer(port);

  // Reset device if lost wifi Connection
  WiFi.onStationModeDisconnected(&lostWifiCallback);

  Serial.println("WiFi connected! User chose hostname '" + String(host_name) + String("' passcode '") + String(passcode) + "' and port '" + String(port_str) + "'");

  // save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println(" config...");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["hostname"] = host_name;
    json["passcode"] = passcode;
    json["port_str"] = port_str;
    json["user_id"] = user_id;
    json["ip"] = WiFi.localIP().toString();
    json["gw"] = WiFi.gatewayIP().toString();
    json["sn"] = WiFi.subnetMask().toString();
    json["version"] = CONFIG_FILE_VERSION;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    Serial.println("");
    Serial.println("Writing config file");
    json.printTo(configFile);
    configFile.close();
    jsonBuffer.clear();
    Serial.println("Config written successfully");
  }
  ticker.detach();

  // keep LED on
  digitalWrite(ledpin, LOW);
  return true;
}



void doOTA() {

  MDNS.begin(host);

  httpUpdater.setup(&httpServer);
  httpServer.begin();

  MDNS.addService("http", "tcp", 80);
  Serial.printf("HTTPUpdateServer ready! Open http://%s.local/update in your browser\n", host);

}

void doWifiManager() {
  WiFiManager wifiManager;
  wifiManager.setTimeout(180);
  if(!wifiManager.autoConnect("PowerMeter-" + ESP.getChipId())) {
    Serial.println("failed to connect and hit timeout. Restarting...");
    delay(3000);
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.print("WiFi connected to "); Serial.print(WiFi.SSID());
  Serial.print(", IP address: "); Serial.println(WiFi.localIP());
}

void doConfig() {
   doWifiManager();
   doOTA();
}

void clearTallys() {
  numReadings = 0;
  minVolts = 9999;
  maxVolts = -9999;
  minWatts = 9999;
  maxWatts = -9999;
}

void updateTallys(float volts, float watts) {

  // a running average calculation
  avgVolts = (volts + (numReadings * avgVolts)) / (numReadings + 1);
  avgWatts = (watts + (numReadings * avgWatts)) / (numReadings + 1);

// min and max appear not to be working on ESP8266! See https://github.com/esp8266/Arduino/issues/398
//  minVolts = min(minVolts, volts);
//  maxVolts = max(maxVolts, volts);
  if (volts < minVolts) minVolts = volts;
  if (volts > maxVolts) maxVolts = volts;
//  minWatts = min(minWatts, watts);
//  maxWatts = max(maxWatts, watts);
  if (watts < minWatts) minWatts = watts;
  if (watts > maxWatts) maxWatts = watts;

  numReadings += 1;

  Serial.print("Readings="); Serial.println(numReadings);
  Serial.print("Volts: "); Serial.print(volts);Serial.print(" avg="); Serial.print(avgVolts); Serial.print(" min="); Serial.print(minVolts); Serial.print(" max="); Serial.println(maxVolts);
  Serial.print("Watts: "); Serial.print(watts); Serial.print(" avg="); Serial.print(avgWatts); Serial.print(" min="); Serial.print(minWatts); Serial.print(" max="); Serial.println(maxWatts);
}

void doInSync() {
    CountBits = 0;  //CLK-interrupt increments CountBits when new bit is received
    while (CountBits < 40) {} //skip the uninteresting 5 first bytes
    CountBits = 0;
    Ba = 0;
    Bb = 0;
    while (CountBits < 24) { //Loop through the next 3 Bytes (6-8) and save byte 6 and 7 in Ba and Bb
      if (NextBit == true) { //when rising edge on CLK is detected, NextBit = true in in interrupt.
        if (CountBits < 9) { //first Byte/8 bits in Ba
          Ba = (Ba << 1);  //Shift Ba one bit to left and store MISO-value (0 or 1) (see http://arduino.cc/en/Reference/Bitshift)
          //read MISO-pin, if high: make Ba[0] = 1
          if (digitalRead(MISOPin) == HIGH) {
            Ba |= (1 << 0); //changes first bit of Ba to "1"
          }   //doesn't need "else" because BaBb[0] is zero if not changed.
          NextBit = false; //reset NextBit in wait for next CLK-interrupt
        }
        else if (CountBits < 17) { //bit 9-16 is byte 7, stor in Bb
          Bb = Bb << 1;  //Shift Ba one bit to left and store MISO-value (0 or 1)
          //read MISO-pin, if high: make Ba[0] = 1
          if (digitalRead(MISOPin) == HIGH) {
            Bb |= (1 << 0); //changes first bit of Bb to "1"
          }
          NextBit = false; //reset NextBit in wait for next CLK-interrupt
        }
      }
    }
    if (Bb != 3) { //if bit Bb is not 3, we have reached the important part, U is allready in Ba and Bb and next 8 Bytes will give us the Power.

      //Voltage = 2*(Ba+Bb/255)
      U = 2.0 * ((float)Ba + (float)Bb / 255.0);

      //Power:
      CountBits = 0;
      while (CountBits < 40) {} //Start reading the next 8 Bytes by skipping the first 5 uninteresting ones

      CountBits = 0;
      Ba = 0;
      Bb = 0;
      Bc = 0;
      while (CountBits < 24) { //store byte 6, 7 and 8 in Ba and Bb & Bc.
        if (NextBit == true) {
          if (CountBits < 9) {
            Ba = (Ba << 1);  //Shift Ba one bit to left and store MISO-value (0 or 1)
            //read MISO-pin, if high: make Ba[0] = 1
            if (digitalRead(MISOPin) == HIGH) {
              Ba |= (1 << 0); //changes first bit of Ba to "1"
            }
            NextBit = false;
          }
          else if (CountBits < 17) {
            Bb = Bb << 1;  //Shift Ba one bit to left and store MISO-value (0 or 1)
            //read MISO-pin, if high: make Ba[0] = 1
            if (digitalRead(MISOPin) == HIGH) {
              Bb |= (1 << 0); //changes first bit of Bb to "1"
            }
            NextBit = false;
          }
          else {
            Bc = Bc << 1;  //Shift Bc one bit to left and store MISO-value (0 or 1)
            //read MISO-pin, if high: make Bc[0] = 1
            if (digitalRead(MISOPin) == HIGH) {
              Bc |= (1 << 0); //changes first bit of Bc to "1"
            }
            NextBit = false;
          }
        }

      }

      //Power = (Ba*255+Bb)/2
      P = ((float)Ba * 255 + (float)Bb + (float)Bc / 255.0) / 2;

      if (U > 200 && U < 300 && P >= 0 && P < 4000) { // ignore spurious readings with voltage or power out of normal range
         updateTallys(U, P);
      } else {
        Serial.print(".");
      }

      inSync = false; //reset sync variable to make sure next reading is in sync.
    }

    if (Bb == 0) { //If Bb is not 3 or something else than 0, something is wrong!
      inSync = false;
      Serial.println("Nothing connected, or out of sync!");
    }
}

//Function that triggers whenever CLK-pin is rising (goes high)
void CLK_ISR() {
  isrTriggers += 1;
  //if we are trying to find the sync-time (CLK goes high for 1-2ms)
  if (inSync == false) {
    ClkHighCount = 0;
    //Register how long the ClkHigh is high to evaluate if we are at the part wher clk goes high for 1-2 ms
    while (digitalRead(CLKPin) == HIGH) {
      ClkHighCount += 1;
      delayMicroseconds(30);  //can only use delayMicroseconds in an interrupt.
    }
    //if the Clk was high between 1 and 2 ms than, its a start of a SPI-transmission
    if (ClkHighCount >= 33 && ClkHighCount <= 67) {
      inSync = true;
    }
  }
  else { //we are in sync and logging CLK-highs
    //increment an integer to keep track of how many bits we have read.
    CountBits += 1;
    NextBit = true;
  }
}

void wifiOff() {
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);
  Serial.println("Wifi off");
}

boolean wifiOn() {
  WiFi.forceSleepWake();
  WiFi.mode(WIFI_STA);
  wifi_station_connect();
  WiFi.begin();
  int timeout = 40; // 20 seconds
  Serial.println("Connecting to WiFi");
  while ((WiFi.status() != WL_CONNECTED) && timeout-- > 0) {
     delay(500);
     Serial.print(".");
  }
  if (timeout < 1) {
     Serial.println("***FAILED");
     wifiOff();
     return false;
  }

  Serial.print("connected, IP address: "); Serial.println(WiFi.localIP());
  return true;
}

boolean sendMQTT() {
  // WiFiClient client;
  // int retries = 6;
  // Serial.print("connecting to:"); Serial.print(host);
  // while (!!!client.connect(host, 80) && (retries-- > 0)) {
  //   Serial.print(".");
  // }
  // Serial.println();
  //
  // if (!!!client.connected()) {
  //   Serial.println("*** connection failed");
  //   return false;
  // }
  //
  // String url = "/input/";
  // url += streamId;
  // url += "?private_key=";
  // url += privateKey;
  // url += "&avgvolts=";
  // url += avgVolts;
  // url += "&minvolts=";
  // url += minVolts;
  // url += "&maxvolts=";
  // url += maxVolts;
  // url += "&avgwatts=";
  // url += avgWatts;
  // url += "&minwatts=";
  // url += minWatts;
  // url += "&maxwatts=";
  // url += maxWatts;
  // url += "&readings=";
  // url += numReadings;
  //
  // Serial.print("Requesting URL: ");
  // Serial.println(url);
  //
  // client.print(String("GET ") + url +
  //              " HTTP/1.1\r\n" +
  //              "Host: " + host + "\r\n" +
  //              "Connection: close\r\n\r\n");
  //
  // int timeout = 20*100; // 20 seconds
  // while(!!!client.available() && (timeout-- > 0)){
  //   delay(10);
  // }
  // if (timeout <= 0) {
  //   return false;
  // }
  //
  // // Read response from server and print to Serial
  // while(client.available()){
  //   Serial.write(client.read());
  // }
  // Serial.println();
  return true;
}

/* Only seems to run reliably with WiFi off while  monitoring the SPI signals.
 * Guessing thats due to the the WiFi timing and SPI monitoring timing interfering with each other
 * resulting in random WDT resets. Its possible that could be fixed with some yield() calls but for
 * now it works ok batching up the WiFi sends instead of streaming the individual power use readings.
 */
boolean sendReading() {
  detachInterrupt(digitalPinToInterrupt(CLKPin));

  boolean sentOk = false;
  if (wifiOn()) {
   sentOk = sendMQTT();
   wifiOff();
  }

  attachInterrupt(digitalPinToInterrupt(CLKPin), CLK_ISR, RISING);
  return sentOk;
}

void setup()
{
  // Initialize serial
  Serial.begin(115200); Serial.println();

  Serial.println("ESP8266 PowerMeter with OTA updates");

  // set led pin as output
  pinMode(ledpin, OUTPUT);

   pinMode(CONFIG_BUTTON, INPUT_PULLUP);
   if (digitalRead(CONFIG_BUTTON) == LOW) {
      Serial.println("In config mode");
      doConfig();
      // Should drop out of setup, and spin around the loop in setup mode ...
      // The loop could force the meter out of config after (e.g. 30 secs)
      op_mode = CONFIG;
      t = millis();
   }
   else
   {
     op_mode = RUN;

     Serial.println("In run mode ...");

     wifiOff();

     //Setting up interrupt ISR on D2 (INT0), trigger function "CLK_ISR()" when INT0 (CLK)is rising
     attachInterrupt(digitalPinToInterrupt(CLKPin), CLK_ISR, RISING);

     //Set the CLK-pin (D5) to input
     pinMode(CLKPin, INPUT);

     //Set the MISO-pin (D5) to input
     pinMode(MISOPin, INPUT);

     clearTallys();
   }
}

void loop()
{
  if(op_mode == RUN)
  {
    // Should look for the button and enter config mode if it's held down

    if ((millis() - lastSend) > SEND_INTERVAL) {
      if (sendReading()) {
         clearTallys();
      }
      lastSend = millis();
    }

    if ((millis() - debugOps) > 10000) { // debug output every 10 secs to see its running
      Serial.print("ISR triggers = "); Serial.println(isrTriggers);
      debugOps = millis();
    }

    //do nothing until the CLK-interrupt occurs and sets inSync=true
    if (inSync == true) {
       doInSync();
    }
  }
  else
  {
    // We are in config mode ...
    if(millis() > (t + CONFIG_MAX_TIME))
      ESP.reset();
  }
}
