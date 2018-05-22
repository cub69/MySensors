/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 *
 * DESCRIPTION
 * The ESP8266 MQTT gateway sends radio network (or locally attached sensors) data to your MQTT broker.
 * The node also listens to MY_MQTT_TOPIC_PREFIX and sends out those messages to the radio network
 *
 * LED purposes:
 * - To use the feature, uncomment any of the MY_DEFAULT_xx_LED_PINs in your sketch
 * - RX (green) - blink fast on radio message recieved. In inclusion mode will blink fast only on presentation recieved
 * - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
 * - ERR (red) - fast blink on error during transmission error or recieve crc error
 *
 * See http://www.mysensors.org/build/esp8266_gateway for wiring instructions.
 * nRF24L01+  ESP8266
 * VCC        VCC
 * CE         GPIO4
 * CSN/CS     GPIO15
 * SCK        GPIO14
 * MISO       GPIO12
 * MOSI       GPIO13
 *
 * Not all ESP8266 modules have all pins available on their external interface.
 * This code has been tested on an ESP-12 module.
 * The ESP8266 requires a certain pin configuration to download code, and another one to run code:
 * - Connect REST (reset) via 10K pullup resistor to VCC, and via switch to GND ('reset switch')
 * - Connect GPIO15 via 10K pulldown resistor to GND
 * - Connect CH_PD via 10K resistor to VCC
 * - Connect GPIO2 via 10K resistor to VCC
 * - Connect GPIO0 via 10K resistor to VCC, and via switch to GND ('bootload switch')
 *
  * Inclusion mode button:
 * - Connect GPIO5 via switch to GND ('inclusion switch')
 *
 * Hardware SHA204 signing is currently not supported!
 *
 * Make sure to fill in your ssid and WiFi password below for ssid & pass.
 */

// Enable debug prints to serial monitor
#define MY_DEBUG

// Use a bit lower baudrate for serial prints on ESP8266 than default in MyConfig.h
#define MY_BAUD_RATE 9600

// Enables and select radio type (if attached)
//#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95

#define MY_GATEWAY_MQTT_CLIENT
#define MY_GATEWAY_ESP8266

// Set this node's subscribe and publish topic prefix
#define MY_MQTT_PUBLISH_TOPIC_PREFIX "garage-out"
#define MY_MQTT_SUBSCRIBE_TOPIC_PREFIX "garage-in"

// Set MQTT client id
#define MY_MQTT_CLIENT_ID "garage"

// Enable these if your MQTT broker requires usenrame/password
//#define MY_MQTT_USER "username"
//#define MY_MQTT_PASSWORD "password"

// Set WIFI SSID and password
#define MY_ESP8266_SSID "linkzilla"
#define MY_ESP8266_PASSWORD "ZAQ12wsx"

// Set the hostname for the WiFi Client. This is the hostname
// it will pass to the DHCP server if not static.
// #define MY_ESP8266_HOSTNAME "mqtt-sensor-gateway"

// Enable MY_IP_ADDRESS here if you want a static ip address (no DHCP)
//#define MY_IP_ADDRESS 192,168,178,87

// If using static ip you can define Gateway and Subnet address as well
//#define MY_IP_GATEWAY_ADDRESS 192,168,178,1
//#define MY_IP_SUBNET_ADDRESS 255,255,255,0

// MQTT broker ip address.
#define MY_CONTROLLER_IP_ADDRESS 192, 168, 15, 20

// The MQTT broker port to to open
#define MY_PORT 1883

// Enable inclusion mode
//#define MY_INCLUSION_MODE_FEATURE
// Enable Inclusion mode button on gateway
//#define MY_INCLUSION_BUTTON_FEATURE
// Set inclusion mode duration (in seconds)
//#define MY_INCLUSION_MODE_DURATION 60
// Digital pin used for inclusion mode button
//#define MY_INCLUSION_MODE_BUTTON_PIN  3

// Set blinking period
//#define MY_DEFAULT_LED_BLINK_PERIOD 300

// Flash leds on rx/tx/err
//#define MY_DEFAULT_ERR_LED_PIN 16  // Error led pin
//#define MY_DEFAULT_RX_LED_PIN  16  // Receive led pin
//#define MY_DEFAULT_TX_LED_PIN  16  // the PCB, on board LED
#include <SPI.h>

#include <ESP8266WiFi.h>
#include <MySensors.h>
#include <Bounce2.h>

#define RELAY_ON 1  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0 // GPIO value to write to turn off attached relay
#define RELAY_ID 2  // ID for Garage Relay
#define RELAY_PIN 5 // PIN for Relay for triggering garage door opener

// Setting multiple child ID's for each switch and it's GPIO pin
//Switch 1
#define CHILD_ID_1 1
#define BUTTON_PIN_1  4  // Arduino Digital I/O pin for button/reed switch

//Switch 2
//#define CHILD_ID_2 2
//#define BUTTON_PIN_2  5  // Arduino Digital I/O pin for button/reed switch

//Switch 3
//#define CHILD_ID_3 3
//#define BUTTON_PIN_3  10  // Arduino Digital I/O pin for button/reed switch

//switch 1
Bounce debouncer_1 = Bounce(); 
int oldValue_1=-1;

//switch 2
//Bounce debouncer_2 = Bounce(); 
//int oldValue_2=-1;

//switch 3
//Bounce debouncer_3 = Bounce(); 
//int oldValue_3=-1;

// Setting mysensor message string (ChildID, V type)
// Change to V_LIGHT if you use S_LIGHT in presentation below
MyMessage msg1(CHILD_ID_1,V_ARMED);
//MyMessage msg2(CHILD_ID_2,V_TRIPPED);
//MyMessage msg3(CHILD_ID_3,V_TRIPPED);


//Presenting each child ID
void presentation() {
  // Register binary input sensor to gw (they will be created as child devices)
  // You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage. 
  // If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
    present(CHILD_ID_1, S_DOOR);
//  present(CHILD_ID_2, S_DOOR);
//  present(CHILD_ID_3, S_DOOR);   
    // Present the relay pin
    present(RELAY_ID, S_BINARY);

}

void before()
{
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, loadState(RELAY_ID)?RELAY_OFF:RELAY_ON);

}
//Setting up each GPIO pin as an input and turning on internal pull up resistor
void setup()
{  
  // Setup the relay pin
//  pinMode(RELAY_PIN, OUTPUT);

  // Setup the button 1
  pinMode(BUTTON_PIN_1,INPUT);
  // Activate internal pull-up
  digitalWrite(BUTTON_PIN_1,HIGH);
  // After setting up the button, setup debouncer
  debouncer_1.attach(BUTTON_PIN_1);
  debouncer_1.interval(5);

    // Setup the button 2
//  pinMode(BUTTON_PIN_2,INPUT);
  // Activate internal pull-up
//  digitalWrite(BUTTON_PIN_2,HIGH);
  // After setting up the button, setup debouncer
//  debouncer_2.attach(BUTTON_PIN_2);
//  debouncer_2.interval(5);

    // Setup the button 3
//  pinMode(BUTTON_PIN_3,INPUT);
  // Activate internal pull-up
//  digitalWrite(BUTTON_PIN_3,HIGH);
  // After setting up the button, setup debouncer
//  debouncer_3.attach(BUTTON_PIN_3);
//  debouncer_3.interval(5);
}

void loop() 
{

  // Switch 1
  debouncer_1.update();
  // Get the update value
  int value_1 = debouncer_1.read();
  if (value_1 != oldValue_1) {
     // Send in the new value
     send(msg1.set(value_1==HIGH ? 0 : 1));
     oldValue_1 = value_1;
  }
  // Switch 2
//  debouncer_2.update();
  // Get the update value
//  int value_2 = debouncer_2.read();
//  if (value_2 != oldValue_2) {
     // Send in the new value
//     send(msg2.set(value_2==HIGH ? 1 : 0));
//     oldValue_2 = value_2;
//  }
  // Switch 3
//  debouncer_3.update();
  // Get the update value
//  int value_3 = debouncer_3.read();
//  if (value_3 != oldValue_3) {
     // Send in the new value
//     send(msg3.set(value_3==HIGH ? 1 : 0));
//     oldValue_3 = value_3;
//  }
}

void receive(const MyMessage &message)
{
  if (message.type == 3) {

     digitalWrite(RELAY_PIN, message.getBool()?RELAY_OFF:RELAY_ON);
     delay(600);
     digitalWrite(RELAY_PIN, message.getBool()?RELAY_ON:RELAY_OFF);

     // Change relay state.
     //digitalWrite(RELAY_PIN, message.getBool() ? 0 : 1);
     //delay(600);
     //digitalWrite(RELAY_PIN, message.getBool() ? 1 : 0);
  }
}

