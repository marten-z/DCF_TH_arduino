// DCF_TH
// Displays time and date from DCF signal and temperature and humidity (DHT22 / AM2302)
// on a 16x02 character LCD display via I2C

// Arduino Pro Mini
// ATmega328P 5V
// Pin A4: SDA (for LCD I2C module)
// Pin A5: SCL (for LCD I2C module)

#include "DHT.h"
#include <LiquidCrystal_I2C.h>

#define DHTPIN 10       // Pin where the DHT sensor is connected to
#define DHTTYPE DHT22   // DHT22 (AM2302)

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);


#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif


void setup()
{
  Serial.begin(9600);
  lcd.init();
  dht.begin();

  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("xx:xx:xx  xx.x"); lcd.printByte(0xDF); lcd.print("C");
  lcd.setCursor(0,1);
  lcd.print("xxxx-xx-xx xx.x%");
}


void loop()
{
  // Wait a few seconds between measurements.
  delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius
  float t = dht.readTemperature();

  printTemp(t);
  printHumidity(h);
}


void printTemp(float t) {
  char str[4];
  if (isnan(t)) {
    strcat(str, " ERR");
  } else {
    dtostrf(t, 4, 1, str);
  }
  lcd.setCursor(10,0);
  lcd.print(str);
}


void printHumidity(float h) {
  char str[4];
  if (isnan(h)) {
    strcat(str, " ERR");
  } else {
    dtostrf(h, 4, 1, str);
  }
  lcd.setCursor(11,1);
  lcd.print(str);
}
