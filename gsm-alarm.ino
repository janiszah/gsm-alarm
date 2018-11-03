
// --- Includes ---
#include <stdint.h>
#include <stdbool.h>

#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#include "LowPower.h"

#include <EEPROM.h>


#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

Adafruit_LIS3DH lis = Adafruit_LIS3DH();



#include <Adafruit_FONA.h>

#define FONA_RX   11
#define FONA_TX   10
#define FONA_RST  12
#define FONA_PWR  9


char replybuffer[255];
char fonaNotificationBuffer[64];          //for notifications from the FONA
char smsBuffer[250];

#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);


#define SURFACE_PIN   2

#define WIRE_IN_PIN   3
#define WIRE_OUT_PIN  4


bool accelo_alarm = false;
bool wire_alarm = false;
bool surface_alarm = false;
bool battery_alarm = false;



long status_time_cnt = 0;


void wakeUp()
{
  surface_alarm = true;
  detachInterrupt(0);
}


void setup() {
  Serial.begin(115200);
  Serial.println("GSM Alarm");


  pinMode(13, OUTPUT);

  pinMode(FONA_PWR, OUTPUT);
  digitalWrite(FONA_PWR, 0);

  pinMode(FONA_RX, INPUT);
  pinMode(FONA_TX, INPUT);
  pinMode(FONA_RST, INPUT);

  pinMode(SURFACE_PIN, INPUT);

  pinMode(WIRE_IN_PIN, INPUT_PULLUP);
  pinMode(WIRE_OUT_PIN, INPUT);
  digitalWrite(WIRE_OUT_PIN, 0);

  if (! lis.begin(0x19)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");
  lis.setDataRate(LIS3DH_DATARATE_1_HZ);


  Surface_check();
  Wire_check();
  Accelometer_check();



}

void loop() {
  //  Serial.print("t:");
  //  Serial.print(temperature_read());
  //  Serial.print(", b:");
  //  Serial.println(battery_read());
  //  delay(10);


  surface_alarm = Surface_check();
  wire_alarm = Wire_check();
  accelo_alarm = Accelometer_check();


  if (surface_alarm) {
    Serial.println("NOT ON SURFACE!!!");
    delay(10);
  }

  if (wire_alarm) {
    Serial.println("WIRE CUT!!!");
    delay(10);

    if(Phone_On()) {
      Serial.println("GSM OK!");
      delay(1000);

      Sms_send("22145533", "ALARM: WIRE CUT!!!");
      
    } else {
      Serial.println("Error: Failed to start GSM!");
    }
    
    delay(5000);
    Phone_Off();
  }

  if (accelo_alarm) {
    Serial.println("DEVICE MOVED!!!");
    delay(10);
  }



//  status_time_cnt++;
//  if(status_time_cnt >= (24 * 3600 / 8)) {
//    status_time_cnt == 0;
//
//    
//  }



  if (digitalRead(SURFACE_PIN) == 1)
    attachInterrupt(0, wakeUp, LOW);
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  detachInterrupt(0);
}


