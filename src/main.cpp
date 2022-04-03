// Arduino uno


#include <Arduino.h>
// These variables store the flash pattern
// and the current state of the LED

int ledPin =  13;                 // the number of the LED pin
int ledState = LOW;               // ledState used to set the LED
unsigned long previousMillis = 0; // will store last time LED was updated
long OnTime = 100;                // milliseconds of on-time
long OffTime = 400;               // milliseconds of off-time



void setup() 
{
  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 0);
  Serial.begin(115200);
  delay(3000);   
}

void loop()
{
  // check to see if it's time to change the state of the LED
  unsigned long currentMillis = millis();
 
  if((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
  {
    previousMillis += OnTime;          // on time is over, set new start
    ledState = LOW;                    // Turn it off
    digitalWrite(ledPin, ledState);    // Update the actual LED
    Serial.println(previousMillis);
  }
  else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime))
  {
    previousMillis += OffTime;           // off time is over, set new start
    ledState = HIGH;                     // turn it on
    digitalWrite(ledPin, ledState);	     // Update the actual LED
    Serial.println(previousMillis);
  }
}


; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:uno]
platform = atmelavr
board = uno
framework = arduino

targets = upload, monitor

monitor_speed = 115200



//   Arduino nano


#include <Arduino.h>
// These variables store the flash pattern
// and the current state of the LED

int ledPin =  13;                 // the number of the LED pin
int ledState = LOW;               // ledState used to set the LED
unsigned long previousMillis = 0; // will store last time LED was updated
long OnTime = 50;                // milliseconds of on-time
long OffTime = 100;               // milliseconds of off-time


/**
 * @brief initialisieren
 * 
 */
void setup() 
{
  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 0);
  Serial.begin(115200);
  delay(3000);   
}

/**
 * @brief Hauptschleife
 * 
 */
void loop()
{
  // check to see if it's time to change the state of the LED
  unsigned long currentMillis = millis();
 
  if((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
  {
    previousMillis += OnTime;          // on time is over, set new start
    ledState = LOW;                    // Turn it off
    digitalWrite(ledPin, ledState);    // Update the actual LED
    Serial.println(previousMillis);
  }
  else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime))
  {
    previousMillis += OffTime;           // off time is over, set new start
    ledState = HIGH;                     // turn it on
    digitalWrite(ledPin, ledState);	     // Update the actual LED
    Serial.println(previousMillis);
  }
}



; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:nanoatmega328]
platform = atmelavr
board = nanoatmega328new
framework = arduino
monitor_speed = 115200
; upload_speed = 115200
upload_speed = 57600





// Arduino Leonardo


#include <Arduino.h>
// RX Led ist bei Pin 7, TX Led ist bei Pin 14 !!!!
// bei micro LEDs invertiert


//  On and Off Times (as int, max=32secs)  // micro
/*
const unsigned int onTime1 = 750;
const unsigned int offTime1 = 10;
const unsigned int onTime2 = 10;
const unsigned int offTime2 = 700;
*/


// On and Off Times (as int, max=32secs)  // pro micro und Leonardo
const unsigned int onTime1 = 50;
const unsigned int offTime1 = 750;
const unsigned int onTime2 = 700;
const unsigned int offTime2 = 50;


// Tracks the last time event fired
unsigned long previousMillis1=0;
unsigned long previousMillis2=0;
 
// Interval is how long we wait
int interval1 = onTime1;
int interval2 = onTime2;
 
// Used to track if LED should be on or off
boolean RXLed = true;
boolean TXLed = true;
 
int RXLED = 17;

// Usual Setup Stuff
void setup() {

 Serial.begin(9600); //This pipes to the serial monitor
 Serial1.begin(9600); //This is the UART, pipes to sensors attached to board

pinMode(RXLED, OUTPUT);  // Set RX LED as an output
 // TX LED is set as an output behind the scenes

digitalWrite(RXLED, HIGH);   // set the LED on
TXLED0; //TX LED is not tied to a normally controlled pin, 0 = an!!  


}
 
void loop() {

  // Grab snapshot of current time, this keeps all timing
  // consistent, regardless of how much code is inside the next if-statement
  unsigned long currentMillis = millis();

  // Compare to previous capture to see if enough time has passed
  if ((unsigned long)(currentMillis - previousMillis1) >= interval1) {
    // Change wait interval, based on current LED state
    if (RXLed) {
      // LED is currently on, set time to stay off
      interval1 = offTime1;
    } else {
      // LED is currently off, set time to stay on
      interval1 = onTime1;
    }
    // Toggle the LED's state, Fancy, eh!?
    RXLed = !(RXLed);
    digitalWrite(RXLED, !RXLed); 
    // Save the current time to compare "later"
    previousMillis1 = currentMillis;
  }
   if ((unsigned long)(currentMillis - previousMillis2) >= interval2) {
    // Change wait interval, based on current LED state
    if (TXLed) {
      // LED is currently on, set time to stay off
      interval2 = offTime2;
    } else {
      // LED is currently off, set time to stay on
      interval2 = onTime2;
    }
    // Toggle the LED's state, Fancy, eh!?
    TXLed = !(TXLed);
      if (TXLed) TXLED0;
      else TXLED1;//TX LED is not tied to a normally controlled pin
    // Save the current time to compare "later"
    previousMillis2 = currentMillis;
  } 

// hier Code der immer ausgeführt wird!!!!
  
  
}  // ende von loop


 


/*
und hier das blink mit der onboard LED

#include <Arduino.h>
// These variables store the flash pattern
// and the current state of the LED

int ledPin =  13;                 // the number of the LED pin
int ledState = LOW;               // ledState used to set the LED
unsigned long previousMillis = 0; // will store last time LED was updated
long OnTime = 50;                // milliseconds of on-time
long OffTime = 450;               // milliseconds of off-time



void setup() 
{
  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 0);
  Serial.begin(115200);
  delay(3000);   
}

void loop()
{
  // check to see if it's time to change the state of the LED
  unsigned long currentMillis = millis();
 
  if((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
  {
    previousMillis += OnTime;          // on time is over, set new start
    ledState = LOW;                    // Turn it off
    digitalWrite(ledPin, ledState);    // Update the actual LED
    Serial.println(previousMillis);
  }
  else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime))
  {
    previousMillis += OffTime;           // off time is over, set new start
    ledState = HIGH;                     // turn it on
    digitalWrite(ledPin, ledState);	     // Update the actual LED
    Serial.println(previousMillis);
  }
}

*/


; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:leonardo]
platform = atmelavr
board = leonardo
framework = arduino

; targets = upload, monitor

monitor_speed = 115200




//  Arduino pro mini


#include <Arduino.h>
// These variables store the flash pattern
// and the current state of the LED

int ledPin =  13;                 // the number of the LED pin
int ledState = LOW;               // ledState used to set the LED
unsigned long previousMillis = 0; // will store last time LED was updated
long OnTime = 50;                // milliseconds of on-time
long OffTime = 200;               // milliseconds of off-time



void setup() 
{
  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 0);
  Serial.begin(115200);
  delay(3000);   
}

void loop()
{
  // check to see if it's time to change the state of the LED
  unsigned long currentMillis = millis();
 
  if((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
  {
    previousMillis += OnTime;          // on time is over, set new start
    ledState = LOW;                    // Turn it off
    digitalWrite(ledPin, ledState);    // Update the actual LED
    Serial.println(previousMillis);
  }
  else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime))
  {
    previousMillis += OffTime;           // off time is over, set new start
    ledState = HIGH;                     // turn it on
    digitalWrite(ledPin, ledState);	     // Update the actual LED
    Serial.println(previousMillis);
  }
}


; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:pro16MHzatmega168]
platform = atmelavr
board = pro16MHzatmega168
framework = arduino

; upload_protocol = serial

monitor_speed = 115200




//  Arduino micro

#include <Arduino.h>
// RX Led ist bei Pin 7, TX Led ist bei Pin 14 !!!!
// bei micro LEDs invertiert


//  On and Off Times (as int, max=32secs)  // micro
const unsigned int onTime1 = 50;
const unsigned int offTime1 = 750;
const unsigned int onTime2 = 50;
const unsigned int offTime2 = 700;

/*
// On and Off Times (as int, max=32secs)  // pro micro und Leonardo
const unsigned int onTime1 = 50;
const unsigned int offTime1 = 750;
const unsigned int onTime2 = 700;
const unsigned int offTime2 = 50;
*/

// Tracks the last time event fired
unsigned long previousMillis1=0;
unsigned long previousMillis2=0;
 
// Interval is how long we wait
int interval1 = onTime1;
int interval2 = onTime2;
 
// Used to track if LED should be on or off
boolean RXLed = true;
boolean TXLed = true;
 
int RXLED = 17;

// Usual Setup Stuff
void setup() {

 Serial.begin(115200); //This pipes to the serial monitor
 Serial1.begin(115200); //This is the UART, pipes to sensors attached to board

pinMode(RXLED, OUTPUT);  // Set RX LED as an output
 // TX LED is set as an output behind the scenes

digitalWrite(RXLED, LOW);   // set the LED on
TXLED0; //TX LED is not tied to a normally controlled pin, 0 = an!!  


}
 
void loop() {

  // Grab snapshot of current time, this keeps all timing
  // consistent, regardless of how much code is inside the next if-statement
  unsigned long currentMillis = millis();

  // Compare to previous capture to see if enough time has passed
  if ((unsigned long)(currentMillis - previousMillis1) >= interval1) {
    // Change wait interval, based on current LED state
    if (RXLed) {
      // LED is currently on, set time to stay off
      interval1 = onTime1;
    } else {
      // LED is currently off, set time to stay on
      interval1 = offTime1;
    }
    // Toggle the LED's state, Fancy, eh!?
    RXLed = !(RXLed);
    digitalWrite(RXLED, !RXLed); 
    // Save the current time to compare "later"
    previousMillis1 = currentMillis;
  }
   if ((unsigned long)(currentMillis - previousMillis2) >= interval2) {
    // Change wait interval, based on current LED state
    if (TXLed) {
      // LED is currently on, set time to stay off
      interval2 = offTime2;
    } else {
      // LED is currently off, set time to stay on
      interval2 = onTime2;
    }
    // Toggle the LED's state, Fancy, eh!?
    TXLed = !(TXLed);
      if (TXLed) TXLED1;
      else TXLED0;//TX LED is not tied to a normally controlled pin
    // Save the current time to compare "later"
    previousMillis2 = currentMillis;
  } 

// hier Code der immer ausgeführt wird!!!!
  
  
}  // ende von loop


 ; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:micro]
platform = atmelavr
board = micro
framework = arduino




//   Arduino pro micro


#include <Arduino.h>
// RX Led ist bei Pin 7, TX Led ist bei Pin 14 !!!!
// bei micro LEDs invertiert

/*
//  On and Off Times (as int, max=32secs)  // micro
const unsigned int onTime1 = 750;
const unsigned int offTime1 = 10;
const unsigned int onTime2 = 10;
const unsigned int offTime2 = 700;
*/


// On and Off Times (as int, max=32secs)  // pro micro und Leonardo
const unsigned int onTime1 = 50;
const unsigned int offTime1 = 750;
const unsigned int onTime2 = 700;
const unsigned int offTime2 = 50;


// Tracks the last time event fired
unsigned long previousMillis1=0;
unsigned long previousMillis2=0;
 
// Interval is how long we wait
int interval1 = onTime1;
int interval2 = onTime2;
 
// Used to track if LED should be on or off
boolean RXLed = true;
boolean TXLed = true;
 
int RXLED = 17;

// Usual Setup Stuff
void setup() {

 Serial.begin(115200); //This pipes to the serial monitor
 Serial1.begin(115200); //This is the UART, pipes to sensors attached to board

pinMode(RXLED, OUTPUT);  // Set RX LED as an output
 // TX LED is set as an output behind the scenes

digitalWrite(RXLED, HIGH);   // set the LED on
TXLED0; //TX LED is not tied to a normally controlled pin, 0 = an!!  


}
 
void loop() {

  // Grab snapshot of current time, this keeps all timing
  // consistent, regardless of how much code is inside the next if-statement
  unsigned long currentMillis = millis();

  // Compare to previous capture to see if enough time has passed
  if ((unsigned long)(currentMillis - previousMillis1) >= interval1) {
    // Change wait interval, based on current LED state
    if (RXLed) {
      // LED is currently on, set time to stay off
      interval1 = offTime1;
    } else {
      // LED is currently off, set time to stay on
      interval1 = onTime1;
    }
    // Toggle the LED's state, Fancy, eh!?
    RXLed = !(RXLed);
    digitalWrite(RXLED, !RXLed); 
    // Save the current time to compare "later"
    previousMillis1 = currentMillis;
  }
   if ((unsigned long)(currentMillis - previousMillis2) >= interval2) {
    // Change wait interval, based on current LED state
    if (TXLed) {
      // LED is currently on, set time to stay off
      interval2 = offTime2;
    } else {
      // LED is currently off, set time to stay on
      interval2 = onTime2;
    }
    // Toggle the LED's state, Fancy, eh!?
    TXLed = !(TXLed);
      if (TXLed) TXLED0;
      else TXLED1;//TX LED is not tied to a normally controlled pin
    // Save the current time to compare "later"
    previousMillis2 = currentMillis;
  } 

// hier Code der immer ausgeführt wird!!!!
  
  
}  // ende von loop


 
 ; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:sparkfun_promicro16]
platform = atmelavr
board = sparkfun_promicro16
framework = arduino



//  WeMos D1R2

#include <Arduino.h>
// Led1 ist bei Pin 14, Led2 ist bei Pin 2 !!!!


// On and Off Times (as int, max=32secs)  // pro micro und Leonardo
const unsigned int onTime1 = 50;
const unsigned int offTime1 = 750;
const unsigned int onTime2 = 740;
const unsigned int offTime2 = 50;


// Tracks the last time event fired
unsigned long previousMillis1=0;
unsigned long previousMillis2=0;
 
// Interval is how long we wait
int interval1 = onTime1;
int interval2 = onTime2;
 
// Used to track if LED should be on or off
boolean Led1 = true;
boolean Led2 = true;
 
int LED1 = 14;
int LED2 = 2;

// Usual Setup Stuff
void setup() {

 Serial.begin(115200); //This pipes to the serial monitor
 Serial1.begin(115200); //This is the UART, pipes to sensors attached to board

pinMode(LED1, OUTPUT); 
pinMode(LED2, OUTPUT); 

digitalWrite(LED1, HIGH);  
digitalWrite(LED2, HIGH);   

}
 
void loop() {

  // Grab snapshot of current time, this keeps all timing
  // consistent, regardless of how much code is inside the next if-statement
  unsigned long currentMillis = millis();

  // Compare to previous capture to see if enough time has passed
  if ((unsigned long)(currentMillis - previousMillis1) >= interval1) {
    // Change wait interval, based on current LED state
    if (Led1) {
      // LED is currently on, set time to stay off
      interval1 = offTime1;
    } else {
      // LED is currently off, set time to stay on
      interval1 = onTime1;
    }
    // Toggle the LED's state, Fancy, eh!?
    Led1 = !(Led1);
    digitalWrite(LED1, Led1); 
    // Save the current time to compare "later"
    previousMillis1 = currentMillis;
  }
   if ((unsigned long)(currentMillis - previousMillis2) >= interval2) {
    // Change wait interval, based on current LED state
    if (Led2) {
      // LED is currently on, set time to stay off
      interval2 = offTime2;
    } else {
      // LED is currently off, set time to stay on
      interval2 = onTime2;
    }
    // Toggle the LED's state, Fancy, eh!?
    Led2 = !(Led2);
    digitalWrite(LED2, Led2); 

    previousMillis2 = currentMillis;
  } 

// hier Code der immer ausgeführt wird!!!!
  
  
}  // ende von loop


; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:d1_mini]
platform = espressif8266
board = d1_mini
framework = arduino

monitor_speed = 115200



//  ESP8266


#include <Arduino.h>

/*
 ESP8266 Blink by Simon Peter
 Blink the blue LED on the ESP-01 module
 This example code is in the public domain

 The blue LED on the ESP-01 module is connected to GPIO1 
 (which is also the TXD pin; so we cannot use Serial.print() at the same time)
*/

// #include <ESP8266WiFi.h>            // we need wifi to get internet access
// #include <time.h>                   // time() ctime()

// These variables store the flash pattern
// and the current state of the LED

int ledPin =  1;                  // GPIO1 = blaue LED
int ledState = LOW;               // ledState used to set the LED
unsigned long previousMillis = 0; // will store last time LED was updated
long OnTime = 600;                 // milliseconds of on-time
long OffTime = 150;               // milliseconds of off-time



void setup() 
{
  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 0);
  //Serial.begin(115200);
  //delay(3000);   
}

void loop()
{
  // check to see if it's time to change the state of the LED
  unsigned long currentMillis = millis();
 
  if((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
  {
    previousMillis += OnTime;          // on time is over, set new start
    ledState = LOW;                    // Turn it off
    digitalWrite(ledPin, ledState);    // Update the actual LED
    //Serial.println(previousMillis);
  }
  else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime))
  {
    previousMillis += OffTime;           // off time is over, set new start
    ledState = HIGH;                     // turn it on
    digitalWrite(ledPin, ledState);	     // Update the actual LED
    //Serial.println(previousMillis);
  }
}


; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:esp01_1m]
platform = espressif8266
board = esp01_1m
framework = arduino

monitor_speed = 115200



//  ESP32


#include <Arduino.h>
#include <time.h>
/*
LED an Pin 27!!!!!
*/

// These variables store the flash pattern
// and the current state of the LED

int ledPin =  27;                  // GPIO1 = blaue LED
int ledState = LOW;               // ledState used to set the LED
unsigned long previousMillis = 0; // will store last time LED was updated
long OnTime = 600;                 // milliseconds of on-time
long OffTime = 150;               // milliseconds of off-time



void setup() 
{
  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 0);
  //Serial.begin(115200);
  //delay(3000);   
}

void loop()
{
  // check to see if it's time to change the state of the LED
  unsigned long currentMillis = millis();
 
  if((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
  {
    previousMillis += OnTime;          // on time is over, set new start
    ledState = LOW;                    // Turn it off
    digitalWrite(ledPin, ledState);    // Update the actual LED
    //Serial.println(previousMillis);
  }
  else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime))
  {
    previousMillis += OffTime;           // off time is over, set new start
    ledState = HIGH;                     // turn it on
    digitalWrite(ledPin, ledState);	     // Update the actual LED
    //Serial.println(previousMillis);
  }
}


; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino

monitor_speed = 115200



//   STM32

/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */
#include <Arduino.h>
#include <time.h>

// These variables store the flash pattern
// and the current state of the LED

int ledPin =  PC13;                  // PC13 onboard LED
int ledState = LOW;               // ledState used to set the LED
unsigned long previousMillis = 0; // will store last time LED was updated

unsigned long OnTime = 400;                 // milliseconds of on-time
unsigned long OffTime = 100;               // milliseconds of off-time

 int LED = PA5;

void setup()
{
  // initialize LED digital pin as an output.
 
  pinMode(LED, OUTPUT);
}

void loop()
{
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED, HIGH);
  // wait for a second
  delay(1000);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED, LOW);
   // wait for a second
  delay(1000);
}



; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:genericSTM32F103C8]
platform = ststm32
board = genericSTM32F103C8
framework = arduino
; build_type = debug
build_type = release

debug_tool = stlink

; upload_protocol = stlink
upload_protocol = serial
; upload_protocol = dfu


monitor_speed = 115200







