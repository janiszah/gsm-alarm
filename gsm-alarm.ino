
// --- Includes ---
#include <stdint.h>
#include <stdbool.h>

#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#include <EEPROM.h>


#include <Adafruit_FONA.h>
#include <Adafruit_LIS3DH.h>



#define FONA_RX 3
#define FONA_TX 4
#define FONA_RST 5

char replybuffer[255];
char fonaNotificationBuffer[64];          //for notifications from the FONA
char smsBuffer[250];

// --- For testing reasons software serial
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
//  HardwareSerial *fonaSerial = &Serial1;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// Accelerometer sensor
//Adafruit_LIS3DH lis = Adafruit_LIS3DH();


void setup() {
  Serial.begin(115200);
  Serial.println("GSM Alarm");

  sleep_init();

  if(gsm_init()) {
    Serial.println("GSM: OK");
  } else {
    Serial.println("GSM: failed init");
  }
}

void loop() {

//  Serial.print("t:");
//  Serial.print(temperature_read());
//  Serial.print(", b:");
//  Serial.println(battery_read());
//  delay(500);

  char* bufPtr = fonaNotificationBuffer;    //handy buffer pointer

  if (fona.available())      //any data available from the FONA?
  {
    int slot = 0;            //this will be the slot number of the SMS
    int charCount = 0;
    //Read the notification into fonaInBuffer
    do  {
      *bufPtr = fona.read();
      Serial.write(*bufPtr);
      delay(1);
    } while ((*bufPtr++ != '\n') && (fona.available()) && (++charCount < (sizeof(fonaNotificationBuffer) - 1)));

    //Add a terminal NULL to the notification string
    *bufPtr = 0;

    //Scan the notification string for an SMS received notification.
    //  If it's an SMS message, we'll get the slot number in 'slot'
    if (1 == sscanf(fonaNotificationBuffer, "+CMTI: " FONA_PREF_SMS_STORAGE ",%d", &slot)) {
      Serial.print("slot: "); Serial.println(slot);

      char callerIDbuffer[32];  //we'll store the SMS sender number in here

      // Retrieve SMS sender address/phone number.
      if (! fona.getSMSSender(slot, callerIDbuffer, 31)) {
        Serial.println("Didn't find SMS message in slot!");
      }
      Serial.print(F("FROM: ")); Serial.println(callerIDbuffer);

      // Retrieve SMS value.
      uint16_t smslen;
      if (fona.readSMS(slot, smsBuffer, 250, &smslen)) { // pass in buffer and max len!
        Serial.println(smsBuffer);
      }

      //Send back an automatic response
      Serial.println("Sending reponse...");
      snprintf(smsBuffer, sizeof(smsBuffer), "I'm OK. Status:\nbat: %dmV\ntemp: %d*C\nuptime: 10h", 
        battery_read(), temperature_read());
      if (!fona.sendSMS(callerIDbuffer, smsBuffer)) {
        Serial.println(F("Failed"));
      } else {
        Serial.println(F("Sent!"));
      }

      // delete the original msg after it is processed
      //   otherwise, we will fill up all the slots
      //   and then we won't be able to receive SMS anymore
      if (fona.deleteSMS(slot)) {
        Serial.println(F("OK!"));
      } else {
        Serial.print(F("Couldn't delete SMS in slot ")); Serial.println(slot);
        fona.print(F("AT+CMGD=?\r\n"));
      }
    }
  }
}


ISR(WDT_vect) {

}
