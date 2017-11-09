//      Arduino MQTT power monitor, Code by Thomas Bergo - November 2017
//      

#include "arduino.h"
#include <avr/wdt.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <Timer.h>

#define REPORTING_INTERVAL_MS  5000
#define MQTT_VERSION MQTT_VERSION_3_1_1

// Comment this out for not printing data to the serialport
#define DEBUG

// We are sending calculated results to an MQTT topic via ethernet

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

// Timer used for timing callbacks
Timer callback_timer;
                 

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
int pulseCount = 0;                        // Number of pulses, used to measure energy.
int power[25] = {};                        // Array to store pulse power values
int txpower = 0;                           // powernumber to send
int txpulse = 0;                           // number of pulses to send
volatile unsigned long pulseTime,lastTime;  // Used to measure power.
int ppwh = 1;                              // Pulses per Watt hour 

//----- Interupt filtering variables ---------
int minElapsed = 100;
volatile unsigned long previousTime;

void mqtt_callback(char* topic, byte* payload, unsigned int length) 
{ 
  // no incoming messages to process
}

void setup() 
{ 
  // ensure the watchdog is disabled
  wdt_disable();

  Serial.begin(9600);              // initialize Serial interface
  while (!Serial) {
    delay(200);                    // wait for serial port to connect. Needed for native USB
  }
  
  #ifdef DEBUG
    Serial.println();
    Serial.println("-------------------------------------");
    Serial.println("Sketch ID:      PowerMeterMQTT.ino");
    Serial.print("Getting IP address...");  
  #endif

  mqttClient.setServer(mqttBroker, 1883);
//  mqttClient.setCallback(mqtt_callback);
  
  // get an IP address
  while (Ethernet.begin(mac) != 1) {
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

  // Setup for report event timing
  int reportEvent = callback_timer.every(REPORTING_INTERVAL_MS, send_data);

  // Attach interupt for capturing light pulses on powercentral
  attachInterrupt(INT_PIN, onPulse, FALLING);

  // enable the watchdog timer - 8s timeout
  wdt_enable(WDTO_8S);
  wdt_reset();
}

void loop()
{ 
  // reset the watchdog timer
  wdt_reset();
  callback_timer.update();
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();
  // check our DHCP lease is still ok
  Ethernet.maintain();
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    #ifdef DEBUG
      Serial.print("Attempting MQTT connection...");
    #endif
    // Attempt to connect
    if (mqttClient.connect(mqttClientId)) {
      #ifdef DEBUG
        Serial.println("connected");
      #endif
    } else {
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

void send_data()
{
  // Calculate average over the last power meassurements before sending
  long _sum = 0;
  int _pulsecount = pulseCount;
  
  for(int i=1; i<=_pulsecount; i++) {
    _sum += power[i];
  }

  txpower = int(_sum / _pulsecount);
  txpulse = _pulsecount;
   
  pulseCount=0;
  power[25] = { };
  
  publishData("power", txpower);
  publishData("pulse", txpulse);

  #ifdef DEBUG
    Serial.print("W: ");
    Serial.print(txpower);
    Serial.print(" - Pulse: ");
    Serial.println(txpulse);
  #endif 
}

void publishData(const char * name, int value)
{  
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
void onPulse() 
{
  unsigned long elapsedTime = millis() - previousTime;

  if (elapsedTime >= minElapsed) {  //in range
  
    previousTime = millis();
    
    lastTime = pulseTime;           //used to measure time between pulses.
    pulseTime = micros();

    // Increase pulseCounter
    pulseCount++;
    
    // Size of array to avoid runtime error
    if (pulseCount < 25) {
      power[pulseCount] = int((3600000000.0 / (pulseTime - lastTime))/ppwh);  //Calculate power
      
    #ifdef DEBUG
      Serial.print("Power: ");
      Serial.print(power[pulseCount]);
      Serial.print(" W - Count: ");
      Serial.println(pulseCount);
    #endif
    }
    else {
      Serial.println("Pulsecount over 25. Not logging....");
    }
  }
}
