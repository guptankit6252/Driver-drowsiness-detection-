#include <LiquidCrystal.h>
int serial;
char array1[]="  Alert!                       ";                  //the string to print on the LCD

int tim = 1;  //the value of delay time

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(4, 6, 10, 11, 12, 13);

void setup()
{ Serial.begin(1);
  lcd.begin(16, 2);  // set up the LCD's number of columns and rows: 
}

void loop() 
{   if(Serial.available()>0){
      serial=Serial.read();
      delay(10);
      Serial.println(serial);
      if(serial=='1'){
    lcd.clear();  //Clears the LCD screen and positions the cursor in the upper-left corner 
    lcd.setCursor(0,0);                    // set the cursor to column 15, line 0
    for (int p = 0; p < 16; p++)
    {
      lcd.print(array1[p]); // Print a message to the LCD.
      delay(tim);                          //wait for 250 microseconds
    }
    delay(10000);
    lcd.clear();}}
}
