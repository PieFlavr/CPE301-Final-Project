// CPE 301 Final Project
// Written by Lucas Pinto and Finn Campbell

//LCD Display Includes !!!NEED DOCUMENTATION/REFERENCE SHEET!!!
//LCD additionallu uses a 330 OHM resistor
#include <LiquidCrystal.h>
//const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5;

const int RS = 39, EN = 41, D4 = 31, D5 = 33, D6 = 35, D7 = 37; 
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

//LED System
//temporary pins, replace with register macros later
const int LED_R = 30, LED_Y = 32, LED_G = 34, LED_B = 36;

void setup(){
    lcd.begin(16,2);
    lcd.clear();
    
    //Testing Display
    lcd.write("Hello World!");

    //Temporary LED Pin Testing
    pinMode(LED_R, OUTPUT);
    pinMode(LED_Y, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);

    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_Y, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);
}

void loop(){

}

ISR(TIMER1_OVF_vect)
{

}