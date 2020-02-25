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
// synced   = 5, best possible quality, clock is 100% synced

#include <LiquidCrystal_I2C.h>
#include <dcf77.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 10       // Pin where the DHT sensor is connected to
#define DHTTYPE DHT22   // DHT22 (AM2302)

DHT_Unified dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pins and settings for DCF module
const uint8_t dcf77_sample_pin = 9;
const uint8_t dcf77_inverted_samples = 0;
//const uint8_t dcf77_pin_mode = INPUT;  // disable internal pull up
const uint8_t dcf77_pin_mode = INPUT_PULLUP;  // enable internal pull up
const uint8_t dcf77_monitor_led = 13;  // A4 == d18

const uint8_t dcfStateDelayInSeconds = 10;
const uint8_t dhtDelayInSeconds = 20;

bool showDateToggle = true;


#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif


void setup() {  
    using namespace Clock;
    
    Serial.begin(9600);    
    pinMode(dcf77_monitor_led, OUTPUT);
    pinMode(dcf77_sample_pin, dcf77_pin_mode);
    
    lcd.init();  
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("DCF st=0");
    lcd.setCursor(0,1);
    lcd.print("m=0");
  
    serialPrintDcfSetupInfo();
    serialPrintDhtSensorInfo();
    
    dht.begin();
    readAndPrintDht();
  
    DCF77_Clock::setup();
    DCF77_Clock::set_input_provider(dcf_sample_input_pin);

    Clock::time_t now;
    uint16_t minutes = 0;
    uint8_t seconds = 0;
  
    // Wait till clock is synced, depending on the signal quality this may take
    // rather long. About 5 minutes with a good signal, 30 minutes or longer
    // with a bad signal
    for (uint8_t state = Clock::useless;
        state == Clock::useless || state == Clock::dirty;
        state = DCF77_Clock::get_clock_state()) {
    
        // wait for next sec
        DCF77_Clock::get_current_time(now);
    
        // render one char per second while initializing
        sprint(state);
        
        ++seconds;
        
        if (seconds == 60) {
            ++minutes;
            seconds = 0;
            
            sprint(" - m=");
            sprint(minutes);
            sprint(", s=");
            sprint(state);
            sprintln();

            readAndPrintDht();

            lcd.setCursor(7,0);
            lcd.print(state);

            lcd.setCursor(2,1);
            lcd.print(minutes);
        }
    }
}


void loop() {
    Clock::time_t now;
    DCF77_Clock::get_current_time(now);
    
    if (now.month.val > 0) {
        serialPrintDateTimeWithState(now);

        lcdPrintTime(now);
        
        uint8_t second = BCD::bcd_to_int(now.second);
        if (second % dcfStateDelayInSeconds == 0) {
            showDateToggle = !showDateToggle;
        }

        if (showDateToggle) {
            lcdPrintDate(now);
        } else {
            lcdPrintState(DCF77_Clock::get_clock_state());
        }

        // Do a measurement every x seconds
        if (second % dhtDelayInSeconds == 0) {
            readAndPrintDht();
        }
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
    sprint(F("DHT Sensor Pin:  ")); sprintln(DHTPIN);
    sprint(F("DHT Sensor Type: ")); sprintln(DHTTYPE);
    sprintln();
    sprintln(F("Clock states:"));
    sprintln(F("useless  = 0, waiting for good enough signal"));
    sprintln(F("dirty    = 1, time data available but unreliable"));
    sprintln(F("free     = 2, clock was once synced but now may deviate more than 200 ms, must not re-lock if valid phase is detected"));
    sprintln(F("unlocked = 3, lock was once synced, inaccuracy below 200 ms, may re-lock if a valid phase is detected"));
    sprintln(F("locked   = 4, clock driven by accurate phase, time is accurate but not all decoder stages have sufficient quality for sync"));
    sprintln(F("synced   = 5, best possible quality, clock is 100% synced"));
    sprintln();
}

void serialPrintDhtSensorInfo() {
    sensor_t sensor;
    
    // Print temperature sensor details.
    dht.temperature().getSensor(&sensor);
    Serial.println(F("------------------------------------"));
    Serial.println(F("Temperature Sensor"));
    Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
    Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
    Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
    Serial.println(F("------------------------------------"));
    
    // Print humidity sensor details.
    dht.humidity().getSensor(&sensor);
    Serial.println(F("Humidity Sensor"));
    Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
    Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
    Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
    Serial.println(F("------------------------------------"));
}

void serialPrintDateTimeWithState(const Clock::time_t now) {
    uint8_t state = DCF77_Clock::get_clock_state();
    switch (state) {
      case Clock::useless:  sprint(F("st=0, useless:  ")); break;
      case Clock::dirty:    sprint(F("st=1, dirty:    ")); break;
      case Clock::free:     sprint(F("st=2, free:     ")); break;
      case Clock::unlocked: sprint(F("st=3, unlocked: ")); break;
      case Clock::locked:   sprint(F("st=4, locked:   ")); break;
      case Clock::synced:   sprint(F("st=5, synced:   ")); break;
    }
    sprint(' ');

    sprint(F("20"));
    DCF77_Clock::print(now);
    sprintln();
}

void lcdPrintTime(const Clock::time_t now) {
    lcd.setCursor(0,0);
    lcdPaddedPrint(now.hour);
    lcd.print(':');
    lcdPaddedPrint(now.minute);
    lcd.print(':');
    lcdPaddedPrint(now.second);
}

void lcdPrintDate(const Clock::time_t now) {
    lcd.setCursor(0,1);
    lcd.print("20");
    lcdPaddedPrint(now.year);
    lcd.print('-');
    lcdPaddedPrint(now.month);
    lcd.print('-');
    lcdPaddedPrint(now.day);
}

void lcdPrintState(const uint8_t state) {
    lcd.setCursor(0,1);
  
    switch (state) {
        case Clock::useless:  lcd.print("0, useless"); break;
        case Clock::dirty:    lcd.print("1, dirty  "); break;
        case Clock::free:     lcd.print("2, free   "); break;
        case Clock::unlocked: lcd.print("3, unlockd"); break;
        case Clock::locked:   lcd.print("4, locked "); break;
        case Clock::synced:   lcd.print("5, synced "); break;
    }
}

void readAndPrintDht() {
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      Serial.println(F("Error reading temperature!"));
    }
    else {
      Serial.print(F("Temperature: "));
      Serial.print(event.temperature);
      Serial.println(F("°C"));
      lcdPrintTemp(event.temperature);
    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      Serial.println(F("Error reading humidity!"));
    }
    else {
      Serial.print(F("Humidity: "));
      Serial.print(event.relative_humidity);
      Serial.println(F("%"));
      lcdPrintHumidity(event.relative_humidity);
    }
}

void lcdPrintTemp(const float t) {
    char str[4];
    if (isnan(t)) {
        strcat(str, " ERR");
        sprint("Error reading temperature: ");
        sprint(t);
        sprintln();
    } else {
        dtostrf(t, 4, 1, str);
    }
    lcd.setCursor(10,0);
    lcd.print(str); lcd.printByte(0xDF); lcd.print('C');
}

void lcdPrintHumidity(const float h) {
    char str[4];
    if (isnan(h)) {
        strcat(str, " ERR");
        sprint("Error reading humidity: ");
        sprint(h);
        sprintln();
    } else {
        dtostrf(h, 4, 1, str);
    }
    lcd.setCursor(11,1);
    lcd.print(str); lcd.print('%');
}

void lcdPaddedPrint(const BCD::bcd_t n) {
    lcd.print(n.digit.hi);
    lcd.print(n.digit.lo);
}
