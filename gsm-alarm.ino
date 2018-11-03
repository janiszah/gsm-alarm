#define ALARM_CALLERID    "22145533"
#define STATUS_SMS_TIME   ((long)(10L * 3600L + 0 * 60 + 0))




//#define DEBUG   1

// --- Includes ---
#include <stdint.h>
#include <stdbool.h>

#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#include "LowPower.h"

//#include <EEPROM.h>


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


//char replybuffer[255];
//char fonaNotificationBuffer[64];          //for notifications from the FONA
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

long uptime = 0;

int surface_state;
int wire_state;

long sms_cooldown = 0;


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


  if (Phone_On()) {
    Serial.println(F("GSM OK!"));
    delay(1000);

    status_time_cnt = Time_sync();

  } else {
    Serial.println(F("Error: Failed to start GSM!"));
  }

  delay(100);
  Phone_Off();
}

void loop() {

  surface_alarm |= Surface_check();
  wire_alarm |= Wire_check();
  accelo_alarm = Accelometer_check();


  if (surface_alarm) {
#ifdef DEBUG
    Serial.println(F("NOT ON SURFACE!!!"));
    delay(10);
#endif

    if (sms_cooldown == 0) {
      if (Phone_sendMessage(ALARM_CALLERID, "NOT ON SURFACE!!!") == true)
        surface_alarm = false;
      sms_cooldown = 40;
    }
  }

  if (wire_alarm) {
#ifdef DEBUG
    Serial.println(F("WIRE CUT!!!"));
    delay(10);
#endif

    if (sms_cooldown == 0) {
      if (Phone_sendMessage(ALARM_CALLERID, "WIRE CUT!!!") == true)
        wire_alarm = 0;
      sms_cooldown = 40;
    }
  }

  if (accelo_alarm) {
#ifdef DEBUG
    Serial.println(F("DEVICE MOVED!!!"));
    delay(10);
#endif

    if (sms_cooldown == 0) {
      Phone_sendMessage(ALARM_CALLERID, "DEVICE MOVED!!!");
      sms_cooldown = 40;
    }
  }



  /* -------
      Send Status every 24h
  */

  if (sms_cooldown > 0)
    sms_cooldown--;

  status_time_cnt--;
  if (status_time_cnt <= 0) {
    uptime++;

    snprintf(smsBuffer, sizeof(smsBuffer), "I'm OK!\nBattery: %dmV\nUptime: %ld days\nWire: %s\nSurface: %s",
             battery_read(), uptime, wire_state == 0 ? "OK" : "Cut", surface_state == 1 ? "On" : "Off");

    if (Phone_On()) {
      Serial.println(F("GSM OK!"));
      delay(1000);

      status_time_cnt = Time_sync();

      Sms_send(ALARM_CALLERID, smsBuffer);

    } else {
      Serial.println(F("Error: Failed to start GSM!"));
    }

    delay(100);
    Phone_Off();
  }



  if (digitalRead(SURFACE_PIN) == 1)
    attachInterrupt(0, wakeUp, LOW);
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  detachInterrupt(0);
}


