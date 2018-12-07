
// -----------------------------------------
// --- GSM ---
// -----------------------------------------
int gsm_init(void)
{
  fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
  fonaSerial = &fonaSS;

  fonaSerial->begin(2400);
  if (! fona.begin(*fonaSerial)) {
    //    Serial.println(F("Couldn't find FONA"));
    return 0;
  }
  //  Serial.println(F("FONA is OK"));

  // Print SIM card IMEI number.
  //  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  //  uint8_t imeiLen = fona.getIMEI(imei);
  //  if (imeiLen > 0) {
  //    Serial.print("SIM card IMEI: "); Serial.println(imei);
  //  }

  fonaSerial->print("AT+CNMI=2,1\r\n");  //set up the FONA to send a +CMTI notification when an SMS is received

  //  Serial.println("FONA Ready");
  return 1;
}




// -----------------------------------------
// --- Battery voltage ---
// -----------------------------------------
uint16_t battery_read(void)
{
  uint32_t timeout;
  uint16_t adc, v;
  uint32_t temp;


  ADMUX = (1 << REFS0) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1); // Set AVCC reference and 1.1V bandap channel
  ADCSRA |= (1 << ADEN); // Enable ADC
  delay(20);
  ADCSRA |= (1 << ADSC); // Start conversation

  // Wait for conversation to complete
  timeout = millis();
  while (ADCSRA & (1 << ADSC)) {
    if ((millis() - timeout) > 5)
      return 0;
  }

  adc = ADCW;

  //v = 1126400 / adc;
  v = 1105920 / adc;

  return v;
}





bool Accelometer_check(void)
{
  static int x, y, z;

  if (accelo_present) {
    lis.read();

    int dx = abs(lis.x - x);
    int dy = abs(lis.y - y);
    int dz = abs(lis.z - z);
    x = lis.x;
    y = lis.y;
    z = lis.z;

    if (dx > 5000 || dy > 5000 || dz > 5000)
      return true;
  }

  return false;
}


bool Surface_check(void)
{
  //  static int surface_state;
  int new_state = digitalRead(SURFACE_PIN);
  bool result = false;

  if (surface_state == 1 && new_state == 0) {
    result = true;
  }

  surface_state = new_state;

  return result;
}



bool Wire_check(void) {
  //  static int wire_state;
  int x = 0;
  bool result = false;

  // check state
  pinMode(WIRE_OUT_PIN, OUTPUT);
  x = digitalRead(WIRE_IN_PIN);
  pinMode(WIRE_OUT_PIN, INPUT);

  if (wire_state != x)
    result = true;

  wire_state = x;

  return result;
}


bool Battery_check(void) {

}


bool Phone_On(void)
{
  digitalWrite(FONA_PWR, 1);
  delay(10);

  if (gsm_init()) {
    int i;
    for (i = 0; i < 100; i++) {

      uint8_t n = fona.getNetworkStatus();

      if (n == 1)
        return true;

      delay(500);
    }

    return false;

  } else {
    return false;
  }
}

void Phone_Off(void)
{
  digitalWrite(FONA_PWR, 0);

  pinMode(FONA_RX, INPUT);
  pinMode(FONA_TX, INPUT);
  pinMode(FONA_RST, INPUT);
}


bool Sms_send(char* callerId, char* content)
{
  if (!fona.sendSMS(callerId, content)) {
    Serial.println(F("Failed"));
    return false;
  } else {
    Serial.println(F("Sent!"));
    return true;
  }
}


long Phone_getTime(void)
{
  char buf[32];
  bool result;

  fona.getTime(buf, sizeof(buf));

  int h = (buf[10] - '0') * 10 + (buf[11] - '0');
  int m = (buf[13] - '0') * 10 + (buf[14] - '0');
  int s = (buf[16] - '0') * 10 + (buf[17] - '0');

  if (h > 23 || m > 59 || s > 59)
    return false;

#ifdef DEBUG
  Serial.print(h);
  Serial.print(":");
  Serial.print(m);
  Serial.print(":");
  Serial.println(s);
#endif

  long t = (long)h * 3600L + (long)m * 60L + (long)s;

  return t;
}


long Time_sync(void)
{
  long t = Phone_getTime();
  long period_cnt = 0;

#ifdef DEBUG
  Serial.print(F("Set time: "));
  Serial.println(STATUS_SMS_TIME);
  Serial.print(F("Time: "));
  Serial.println(t);
#endif

  if (t < STATUS_SMS_TIME) {
    if ((STATUS_SMS_TIME - t) < 3600) {
      period_cnt = (86400 +  (STATUS_SMS_TIME - t)) / 8;
    } else {
      period_cnt = (STATUS_SMS_TIME - t) / 8;
    }
  } else {
    period_cnt = ((86400 - t) + STATUS_SMS_TIME) / 8;
  }

#ifdef DEBUG
  Serial.print(F("Next time after 8s period: "));
  Serial.println(period_cnt * 8);
#endif

  return period_cnt;
}

// h*3600 + m*60 + s


bool Phone_sendMessage(char* callerId, char* message)
{
  bool result = false;

  if (Phone_On()) {
    Serial.println(F("GSM OK!"));
    delay(1000);

    result = Sms_send(callerId, message);

  } else {
    Serial.println(F("Error: Failed to start GSM!"));
    result = false;
  }

  delay(100);
  Phone_Off();

  return result;
}


