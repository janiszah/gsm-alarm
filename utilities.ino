
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
// --- Sleep ---
// -----------------------------------------
void sleep_init(void) {
  cli();        // Disable interrupts
  MCUSR = 0;    // Reset status register

  WDTCSR |= (1 << WDCE) | (1 << WDE); // Set WDCE and WDE to enable changes
  WDTCSR = 0b100001;                  // Set the prescaler bit values (8s)
  WDTCSR |= (1 << WDIE);              // Enable only watchdog interrupts

  sei();        // Enable interrupts
}


uint32_t sleep_cycle_cnt = 0;
void sleep(uint32_t cycles) {

  sleep_cycle_cnt = cycles;

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  power_adc_disable();

  while (sleep_cycle_cnt > 0) {
    sleep_mode();
    sleep_disable();
    sleep_cycle_cnt--;
  }

  power_all_enable();
}


// -----------------------------------------
// --- Internal temperature ---
// -----------------------------------------
int16_t temperature_read(void)
{
  uint32_t timeout;
  uint16_t adc;
  int16_t t;

  ADMUX = (1 << REFS1) | (1 << REFS0) | (1 << MUX3); // Set internal reference and channel 8
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

  int32_t temp = ((adc * 100u) - 32431u) / 100;

  t = temp;

  return t;
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



void accelo_read(void)
{
  static int x, y, z;

  lis.read();

  int dx = abs(lis.x - x);
  int dy = abs(lis.y - y);
  int dz = abs(lis.z - z);
  x = lis.x;
  y = lis.y;
  z = lis.z;

  Serial.print("x:");
  Serial.print(dx);
  Serial.print(", y:");
  Serial.print(dy);
  Serial.print(", z:");
  Serial.println(dz);


#ifdef DEBUG
  Serial.print("x:");
  Serial.print(lis.x);
  Serial.print(", y:");
  Serial.print(lis.y);
  Serial.print(", z:");
  Serial.println(lis.z);
#endif

}



bool Accelometer_check(void)
{
  static int x, y, z;

  lis.read();

  int dx = abs(lis.x - x);
  int dy = abs(lis.y - y);
  int dz = abs(lis.z - z);
  x = lis.x;
  y = lis.y;
  z = lis.z;

  if (dx > 5000 || dy > 5000 || dz > 5000)
    return true;

  return false;
}

bool Surface_check(void)
{
  static int surface_state;
  int new_state = digitalRead(SURFACE_PIN);
  bool result = false;

  if (surface_state == 1 && new_state == 0) {
    result = true;
  }

  surface_state = new_state;

  return result;
}

bool Wire_check(void) {
  static int wire_state;
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
  
  if(gsm_init()) {
    int i;
    for(i=0; i<100; i++) {

      uint8_t n = fona.getNetworkStatus();

      if(n == 1)
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


void Sms_send(char* callerId, char* content)
{
  if (!fona.sendSMS(callerId, content)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("Sent!"));
  }
}



