// DCF_TH
// Displays time and date from DCF signal 
// and temperature and humidity from a DHT22 / AM2302 sensor
// on a 16x02 character LCD via I2C

// Arduino Pro Mini
// ATmega328P 5V
// Pin A4: SDA (for LCD I2C module)
// Pin A5: SCL (for LCD I2C module)

// Clock states:
// useless  = 0, waiting for good enough signal
// dirty    = 1, time data available but unreliable
// free     = 2, clock was once synced but now may deviate more than 200 ms, must not re-lock if valid phase is detected
// unlocked = 3, lock was once synced, inaccuracy below 200 ms, may re-lock if a valid phase is detected
// locked   = 4, clock driven by accurate phase, time is accurate but not all decoder stages have sufficient quality for sync
// synced   = 5  best possible quality, clock is 100% synced

#include "DHT.h"
#include <LiquidCrystal_I2C.h>
#include <dcf77.h>

#define DHTPIN 10       // Pin where the DHT sensor is connected to
#define DHTTYPE DHT22   // DHT22 (AM2302)

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
const uint8_t dhtDelayInSeconds = 5;


// Pins and settings for DCF module
const uint8_t dcf77_sample_pin = 9;
const uint8_t dcf77_inverted_samples = 0;
//const uint8_t dcf77_pin_mode = INPUT;  // disable internal pull up
const uint8_t dcf77_pin_mode = INPUT_PULLUP;  // enable internal pull up
const uint8_t dcf77_monitor_led = 13;  // A4 == d18


#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif


void setup()
{
  pinMode(dcf77_monitor_led, OUTPUT);
  pinMode(dcf77_sample_pin, dcf77_pin_mode);
  
  using namespace Clock;
  
  Serial.begin(9600);
  lcd.init();
  dht.begin();

  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("DCF init");
  lcd.setCursor(0,1);
  lcd.print("st=0 m=0 ");

  serialPrintDcfSetupInfo();
  readAndPrintDht();

  DCF77_Clock::setup();
  DCF77_Clock::set_input_provider(dcf_sample_input_pin);

  uint8_t minutes = 0;

  // Wait till clock is synced, depending on the signal quality this may take
  // rather long. About 5 minutes with a good signal, 30 minutes or longer
  // with a bad signal
  for (uint8_t state = Clock::useless;
    state == Clock::useless || state == Clock::dirty;
    state = DCF77_Clock::get_clock_state()) {

    // wait for next sec
    Clock::time_t now;
    DCF77_Clock::get_current_time(now);

    // render one dot per second while initializing
    static uint8_t count = 0;
    sprint('.');
    ++count;
    
    if (count == 60) {
      ++minutes;
      count = 0;
      sprint(minutes);
      sprintln();
      
      lcd.setCursor(3,1);
      lcd.print(state);
      lcd.setCursor(7,1);
      lcd.print(minutes);
    }
  }
}


void loop()
{
  Clock::time_t now;

  DCF77_Clock::get_current_time(now);
  if (now.month.val > 0) {
    switch (DCF77_Clock::get_clock_state()) {
      case Clock::useless: sprint(F("useless ")); break;
      case Clock::dirty:   sprint(F("dirty:  ")); break;
      case Clock::synced:  sprint(F("synced: ")); break;
      case Clock::locked:  sprint(F("locked: ")); break;
    }
    lcdPrintDateTime(now);
    dcfSerialPrint(now);
  }
    
  static uint8_t seconds = 0;
  ++seconds;
  
  // Wait a few seconds between measurements.
  if (seconds >= dhtDelayInSeconds) {    
    seconds = 0;    
    readAndPrintDht();
  }
}


uint8_t dcf_sample_input_pin() {
  const uint8_t sampled_data = dcf77_inverted_samples ^ digitalRead(dcf77_sample_pin);

  digitalWrite(dcf77_monitor_led, sampled_data);
  return sampled_data;
}

void serialPrintDcfSetupInfo() {
  sprintln();
  sprintln(F("Simple DCF77 Clock V3.1.1"));
  sprintln(F("(c) Udo Klein 2016"));
  sprintln(F("www.blinkenlight.net"));
  sprintln();
  sprint(F("Sample Pin:      ")); sprintln(dcf77_sample_pin);
  sprint(F("Sample Pin Mode: ")); sprintln(dcf77_pin_mode);
  sprint(F("Inverted Mode:   ")); sprintln(dcf77_inverted_samples);
  sprint(F("Monitor Pin:     ")); sprintln(dcf77_monitor_led);
  sprintln();
  sprintln();
  sprintln(F("Initializing..."));
}

void lcdPrintDateTime(Clock::time_t now) {
  lcdPrintTime(now);
  lcdPrintDate(now);  
}

void lcdPrintTime(Clock::time_t now) {
  char time[8] = {
    now.hour.digit.hi,
    now.hour.digit.lo,
    ':',
    now.minute.digit.hi,
    now.minute.digit.lo,
    ':',
    now.second.digit.hi,
    now.second.digit.lo
  };
  
  lcd.setCursor(0,0);
  lcd.print(time);
}

void lcdPrintDate(Clock::time_t now) {
  char date[10] = {
    '2',
    '0',
    now.year.digit.hi,
    now.year.digit.lo,
    '-',
    now.month.digit.hi,
    now.month.digit.lo,
    '-',
    now.day.digit.hi,
    now.day.digit.lo
  };
  
  lcd.setCursor(0,1);
  lcd.print(date);
}

void readAndPrintDht() {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius
  float t = dht.readTemperature();

  lcdPrintTemp(t);
  lcdPrintHumidity(h);
}

void lcdPrintTemp(float t) {
  char str[4];
  if (isnan(t)) {
    strcat(str, " ERR");
  } else {
    dtostrf(t, 4, 1, str);
  }
  lcd.setCursor(10,0);
  lcd.print(str); lcd.printByte(0xDF); lcd.print('C');
}

void lcdPrintHumidity(float h) {
  char str[4];
  if (isnan(h)) {
    strcat(str, " ERR");
  } else {
    dtostrf(h, 4, 1, str);
  }
  lcd.setCursor(11,1);
  lcd.print(str); lcd.print('%');
}

void dcfSerialPrint(Clock::time_t now) {
  sprint(' ');

  sprint(F("20"));
  paddedPrint(now.year);
  sprint('-');
  paddedPrint(now.month);
  sprint('-');
  paddedPrint(now.day);
  sprint(' ');

  paddedPrint(now.hour);
  sprint(':');
  paddedPrint(now.minute);
  sprint(':');
  paddedPrint(now.second);

  sprint("+0");
  sprint(now.uses_summertime? '2': '1');
  sprintln();
}

void paddedPrint(BCD::bcd_t n) {
  sprint(n.digit.hi);
  sprint(n.digit.lo);
}
