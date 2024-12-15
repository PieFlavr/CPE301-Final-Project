// CPE 301 Final Project
// Written by Lucas Pinto and Finn Campbell

//LCD Display Includes !!!NEED DOCUMENTATION/REFERENCE SHEET!!!
//LCD additionallu uses a 330 OHM resistor
#include <LiquidCrystal.h>
//const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5;
const int RS = 39, EN = 41, D4 = 31, D5 = 33, D6 = 35, D7 = 37; 
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);


void setup(){
    lcd.begin(16,2);
    lcd.clear();
    
    //Testing Display
    lcd.write("Hello World!");
    
}

void loop(){
    
}

ISR(TIMER1_OVF_vect)
{

}