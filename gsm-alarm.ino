#define ALARM_CALLERID    "22145533"
#define STATUS_SMS_TIME   ((long)(8L * 3600L + 0 * 60 + 0))


#define DEBUG   1

// --- Includes ---
#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#include <LowPower.h>
#include <M2M_LM75A.h>


#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FONA.h>


Adafruit_LIS3DH lis = Adafruit_LIS3DH();
M2M_LM75A lm75;


// --- FONA ---
#define FONA_RX   11
#define FONA_TX   10
#define FONA_RST  12
#define FONA_PWR  9




char smsBuffer[250];

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);


#define ALARM_PIN         2
#define SURFACE_PIN       3

#define WIRE_IN_PIN       6
#define WIRE_OUT_PIN      7

#define BAT_LEVEL_EN_PIN  8
#define BAT_LEEL_PIN      A0

#define GPIO1_PIN         4
#define GPIO2_PIN         5
#define ANALOG1_PIN       A1
#define ANALOG2_PIN       A2

#define BUTTON_PIN        A3
#define LED_PIN           13




bool accelo_alarm = false;
bool wire_alarm = false;
bool surface_alarm = false;
bool battery_alarm = false;



long status_time_cnt = 0;

long uptime = 0;

int surface_state;
int wire_state;

long sms_cooldown = 0;


bool accelo_present = false;


const int cooldown_period = 40;


void wakeUp()
{
  surface_alarm = true;
  detachInterrupt(1);
}


void setup() {
  Serial.begin(115200);
  Serial.println("GSM Alarm");


  pinMode(LED_PIN, OUTPUT);

  pinMode(FONA_PWR, OUTPUT);
  digitalWrite(FONA_PWR, 0);

  pinMode(FONA_RX, INPUT);
  pinMode(FONA_TX, INPUT);
  pinMode(FONA_RST, INPUT);

  pinMode(SURFACE_PIN, INPUT);

  pinMode(WIRE_IN_PIN, INPUT_PULLUP);
  pinMode(WIRE_OUT_PIN, INPUT);
  digitalWrite(WIRE_OUT_PIN, 0);


  pinMode(BAT_LEVEL_EN_PIN, OUTPUT);
  digitalWrite(BAT_LEVEL_EN_PIN, 0);
  

  if (lis.begin(0x19)) {   // change this to 0x19 for alternative i2c address
    accelo_present = true;

    Serial.println("LIS3DH found!");
    lis.setDataRate(LIS3DH_DATARATE_1_HZ);
  }


  lm75.begin();
  lm75.shutdown();


  Surface_check();
  Wire_check();
  Accelometer_check();


  if (Phone_On()) {
    Serial.println(F("GSM OK!"));
    delay(1000);

    status_time_cnt = 0;//Time_sync();

  } else {
    Serial.println(F("Error: Failed to start GSM!"));
  }

  delay(100);
  Phone_Off();
}

void loop() {

#ifdef DEBUG
    Serial.println(F("Checking sensors..."));
    delay(10);
#endif

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
      sms_cooldown = cooldown_period;
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
      sms_cooldown = cooldown_period;
    }
  }

  if (accelo_alarm) {
#ifdef DEBUG
    Serial.println(F("DEVICE MOVED!!!"));
    delay(10);
#endif

    if (sms_cooldown == 0) {
      Phone_sendMessage(ALARM_CALLERID, "DEVICE MOVED!!!");
      sms_cooldown = cooldown_period;
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


    if (Phone_On()) {
      Serial.println(F("GSM OK!"));
      delay(1000);

// Get Battery voltage
      uint16_t vbat;
      if (! fona.getBattVoltage(&vbat)) {
#ifdef DEBUG
        Serial.println(F("Failed to read Batt"));
#endif
        vbat = battery_read();
      } 
#ifdef DEBUG
      else {
        Serial.print(F("VBat = ")); Serial.print(vbat); Serial.println(F(" mV"));
      }
#endif


// Get temperature
      lm75.wakeup();
      delay(10);
      uint16_t temp = lm75.getTemperature();
      lm75.shutdown();

      snprintf(smsBuffer, sizeof(smsBuffer), "I'm OK!\nBattery: %dmV\nUptime: %ld days\nTemp: %d*C\nWire: %s\nSurface: %s",
             vbat, uptime, temp, wire_state == 0 ? "OK" : "Cut", surface_state == 1 ? "On" : "Off");

      status_time_cnt = Time_sync();

      Sms_send(ALARM_CALLERID, smsBuffer);

    } else {
      Serial.println(F("Error: Failed to start GSM!"));
    }

    delay(100);
    Phone_Off();
  }



  if (digitalRead(SURFACE_PIN) == 1)
    attachInterrupt(1, wakeUp, LOW);
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  detachInterrupt(1);
}


