//      Arduino MQTT power monitor, Code by Thomas Bergo - November 2017
//      

#include "arduino.h"
#include <avr/wdt.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

#define REPORTING_INTERVAL_MS  10000    // Reporting interval (ms)
#define MIN_ELAPSED_TIME 200000         // Filtering min elapsed time (microSec)
#define PPWH 1                          // pulses per watt hour
#define MQTT_VERSION MQTT_VERSION_3_1_1

// Comment this out for not printing data to the serialport
#define DEBUG

// unique MAC address on our LAN (0x53 => .83)
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x53 };

// MQTT broker connection properties
byte mqttBroker[] = { 192, 168, 1, 170 };
char mqttClientId[] = "emontx";
//char mqttUsername[] = "USERNAME";
//char mqttPassword[] = "PASSWORD";

// publish to "/emontx/<variable>".
char mqttTopic[] = "/emontx";
//char mqttTopicLwt[] = "/clients/emontx";
//int  mqttLwtQos = 0;
//int  mqttLwtRetain = 1;

EthernetClient ethClient;
PubSubClient mqttClient(ethClient);
                 

// ----------- Pinout assignments  -----------
//
// digital input pins:
// dig pin 0 is for Serial Rx
// dig pin 1 is for Serial Tx
// dig pin 4 is for the ethernet module (IRQ) 
// dig pin 10 is for the ethernet module (SEL) 
// dig pin 11 is for the ethernet module (SDI) 
// dig pin 12 is for the ethernet module (SDO) 
// dig pin 13 is for the ethernet module (CLK) 
const byte INT_PIN = 1;

// Pulse counting settings 
volatile int pulseCount = 0;                // Number of pulses, used to measure energy.
volatile unsigned long power[50] = {};      // Array to store pulse power values
volatile unsigned long pulseTime =0;        // Used to measure time between pulses.
unsigned long previousSendTime = 0;         // Used to mesure time between sending data

void mqtt_callback(char* topic, byte* payload, unsigned int length) 
{ 
  // no incoming messages to process
}

void setup() { 
  
  wdt_disable();                           // ensure the watchdog is disabled

  Serial.begin(9600);                      // initialize Serial interface
  while (!Serial) {
    delay(200);                            // wait for serial port to connect. Needed for native USB
  }
  
  #ifdef DEBUG
    Serial.println();
    Serial.println("-------------------------------------");
    Serial.println("Sketch ID:      PowerMeterMQTT.ino");
    Serial.print("Getting IP address...");  
  #endif

  mqttClient.setServer(mqttBroker, 1883);
  //mqttClient.setCallback(mqtt_callback);
  
  while (Ethernet.begin(mac) != 1) {        // Get an IP address
    delay(500);
    #ifdef DEBUG
      Serial.print(".");
    #endif
  }

  #ifdef DEBUG
    Serial.println("");
    Serial.println("Your are connected!");  
    Serial.print("Your IP address is: ");
    Serial.println(Ethernet.localIP());
  #endif

  mqtt_connect();                          // Connecting to MQTT server

  // Attach interupt for capturing light pulses on powercentral
  attachInterrupt(INT_PIN, onPulse, FALLING);

  wdt_enable(WDTO_8S);                     // enable the watchdog timer - 8s timeout
  wdt_reset();
}

void loop() {
   
  wdt_reset();                            // reset the watchdog timer
  Ethernet.maintain();                    // check our DHCP lease is still ok

  if (!mqttClient.connected()) {          // check connection to mqtt server
    mqtt_connect();
  }
  
  mqttClient.loop();
  
  if (millis() - previousSendTime >= REPORTING_INTERVAL_MS) {   // check if it's time to publish data
    previousSendTime = millis();
    send_data();
  }
}

void mqtt_connect() {
  
  while (!mqttClient.connected()) {         // Loop until we're reconnected
    #ifdef DEBUG
      Serial.print("Attempting MQTT connection...");
    #endif
   
    if (mqttClient.connect(mqttClientId)) { // Attempt to connect
      #ifdef DEBUG
        Serial.println("connected");
      #endif
    } 
    else {
      #ifdef DEBUG
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" try again in 5 seconds");
      #endif
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void send_data() {
  
  // Calculate average over the last power meassurements before sending
  int _txpulse = pulseCount;         // number of pulses to send
  unsigned long _sum = 0;
  
  for(int i=1; i<=_txpulse; i++) {
    _sum += power[i];
  }

  pulseCount = 0;
  power[50] = { };
  
  unsigned int _txpower = (_sum / _txpulse);
   
  publishData("power", _txpower);
  publishData("pulse", _txpulse);

  #ifdef DEBUG
    Serial.print("W: ");
    Serial.print(_txpower);
    Serial.print(" - Pulse: ");
    Serial.println(_txpulse);
  #endif 
}

void publishData(const char * name, unsigned int value) {
    
  // build the MQTT topic
  char topic[32];
  snprintf(topic, 32, "%s/%s", mqttTopic, name);

  // build the MQTT payload
  char payload[16];
  snprintf(payload, 16, "%i", value);

  // publish to the MQTT broker 
  mqttClient.publish(topic, payload);
}


// The interrupt routine - runs each time a falling edge of a pulse is detected
void onPulse() {
   
  unsigned long _elapsedTimePulse = micros() - pulseTime;
  if (_elapsedTimePulse > MIN_ELAPSED_TIME) {

    pulseTime = micros();
    
    pulseCount++;                   // Increase pulseCounter
    
    // Size of array to avoid runtime error
    if (pulseCount < 50) {
      power[pulseCount] = long((3600000000.0 / _elapsedTimePulse) / PPWH);  //Calculate power
      
    #ifdef DEBUG
      Serial.print("Power: ");
      Serial.print(power[pulseCount]);
      Serial.print(" W - Count: ");
      Serial.println(pulseCount);
    #endif
    }
    else {
      Serial.println("Pulsecount over 50. Not logging....");
    }
  }
}

