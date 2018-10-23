
// -----------------------------------------
// --- GSM ---
// -----------------------------------------
int gsm_init(void)
{
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

  v = 1126400 / adc;

  return v;
}
