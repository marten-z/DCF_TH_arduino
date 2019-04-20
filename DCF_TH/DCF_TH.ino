// DCF_TH
// Displays time and date from DCF signal and temperature and humidity
// on a 16x02 character LCD display

// Arduino Pro Mini
// ATmega328P 5V
// Pin A4: SDA
// Pin A5: SCL

#include <LiquidCrystal_I2C.h>

//LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27,16, 2);

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

void setup()
{
  lcd.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("19:49:21    20"); lcd.printByte(0xDF); lcd.print("C");
  lcd.setCursor(0,1);
  lcd.print("2019-04-19   80%");
}


void loop()
{
  
}
