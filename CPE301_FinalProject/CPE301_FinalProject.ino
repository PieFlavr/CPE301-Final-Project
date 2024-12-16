/**
 * @file CPE301_FinalProject.ino
 * @author Lucas Pinto and Finn Campbell
 * @brief Final project for CPE301 Fall 2024. 
 *          TO-DO-LIST : clean up everything, implement systems mechanics, push-button interrupt, and potentiometer reading for stepper motor adjust.
 * @version 1.0
 * @date 2024-12-15
 * 
 */

/**
 * LCD Display Includes !!!NEED DOCUMENTATION/REFERENCE SHEET!!!
 * ..> LCD additionallu uses a 330 OHM resistor
 */
#include <LiquidCrystal.h>
const int RS = 39, EN = 41, D4 = 31, D5 = 33, D6 = 35, D7 = 37; 
LiquidCrystal display(RS, EN, D4, D5, D6, D7);

/**
 * LED System !!!NEED DOCUMENTATION/REFERENCE SHEET NEEDED!!!
 * ..> const int LED_R = 30, LED_Y = 32, LED_G = 34, LED_B = 36
 * DIGITAL PINs 30-37 correspond to BITs 7-0 
 * ..> (30~7 ; 31~6 ; 32~5 ; 33~4 ; 34~3 ; 35~2 ; 36~1 ; 37~0)
 */
volatile unsigned char* PORT_C = (unsigned char*) 0x28; //WRITE address, 0x08 is READ!!!
volatile unsigned char* DDR_C = (unsigned char*) 0x27; //WRITE address, 0x07 is READ!!!

/**
 * Stepper Motor !!!NEED DOCUMENTATION/REFERENCE SHEET NEEDED!!!
 */
#include <Stepper.h>
const int stepsPerRevolution = 2038;
Stepper  ventMotor = Stepper(stepsPerRevolution, 45, 47, 49, 51);

/**
 * Fan Motor !!!NEED DOCUMENTATION/REFERENCE SHEET NEEDDED!!!
 */
const int fanSpeedPIN = 12, fanDIR1 = 11, fanDIR2 = 13, fanSpeed = 90;

/**
 * DHT11 Water Sensor !!!NEED DOCUMENTATION/REFERENCE SHEET NEEDED!!!
 */
#include <dht.h>
const int DHT11_PIN = 54; //Pin A0
volatile unsigned char* PORT_G = (unsigned char*) 0x34; //WRITE adddres, 0x14 is READ!!!
volatile unsigned char* DDR_G = (unsigned char*) 0x33; //WRITE address, 0x13 is READ!!!
dht DHT_Sensor;

/**
 * RTC Clock
 */
#include <RTClib.h>
RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday",
"Friday", "Saturday"};

/**
 * @brief Water Level Sensor !!!NEED DOCUMENTATION/REFERENCE SHEET NEEDED!!!
 * 
 */
// ADC Registers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;    // ADC Multiplexer Selection Register
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;   // ADC Control and Status Register B
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;   // ADC Control and Status Register A
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;   // ADC Data Register

/**
 * Function Definitions
 */
// Register Manipulators
void writeRegister(unsigned char* address, int bit, int value);
unsigned int readRegister(unsigned char* address, int bit);

void setup(){
    Serial.begin(9600);

    rtc.begin();

    display.begin(16,2);
    display.clear();
    
    // Testing Display
    display.write("Hello World!");

    // LED Testing (DIGITAL 30,32,34,36 respectively)
    writeRegister(DDR_C, 7, 1); //Set to OUTPUT
    writeRegister(DDR_C, 5, 1);
    writeRegister(DDR_C, 3, 1);
    writeRegister(DDR_C, 1, 1);

    writeRegister(PORT_C, 7, 0); //Set to OFF
    writeRegister(PORT_C, 5, 0);
    writeRegister(PORT_C, 3, 0);
    writeRegister(PORT_C, 1, 0);

    //Stepper Motor Testing
    ventMotor.setSpeed(5);
    ventMotor.step(stepsPerRevolution);

    //Fan Motor Testing, !!!not sure if library is allowed ???!!!
    pinMode(fanSpeedPIN, OUTPUT);
    pinMode(fanDIR1, OUTPUT);
    pinMode(fanDIR2, OUTPUT);

    //Analog Read Setup
    ADC_setup();

    //Water Level Sensor
    writeRegister(DDR_G, 1, 1); //Set to OUTPUT
    writeRegister(PORT_G, 1, 0); //Set to OFF
}

void loop(){
    //Temperature Sensing
    int chk = DHT_Sensor.read11(DHT11_PIN);
    int temperature = DHT_Sensor.temperature; // Already at integer precision
    int humidity = DHT_Sensor.humidity; // So no information lost doing this

    //LCD Printout
    display.clear();

    display.setCursor(0,0);
    display.print("TEMP:: ");
    display.print(temperature);
    display.print("C");

    display.setCursor(0,1);
    display.print("HUMIDITY:: ");
    display.print(humidity);
    display.print("%");

    //TEMPORARY WATER LEVEL TESTING
    writeRegister(PORT_G, 1, 1); //Set to ON
    int testValue = adc_read(1);
    Serial.println(testValue); //100 lower bound, 300 upper bound calibration
    writeRegister(PORT_G, 1, 0); //Set to OFF

    //Fan Motor Testing
    digitalWrite(fanDIR1,LOW);
    digitalWrite(fanDIR2,HIGH);
    analogWrite(fanSpeedPIN,0);

    //RTC Testing
    DateTime now = rtc.now();
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    delay(500);
}

ISR(TIMER1_OVF_vect)
{

}

/**
 * @brief Initializes the ADC primarily for the water level sensor.
 */
void ADC_setup()
{
  *my_ADCSRA |= 0b10000000;   // Enable ADC by setting bit 7
  *my_ADCSRA &= 0b11011111;   // Disable ADC trigger mode (bit 6)
  *my_ADCSRA &= 0b11110111;   // Disable ADC interrupt (bit 5)
  *my_ADCSRA &= 0b11111000;   // Reset prescaler
  *my_ADCSRA |= 0b00000111;   // Set prescaler to 128

  *my_ADCSRB &= 0b11110111;   // Reset channel and gain bits in ADCSRB
  *my_ADCSRB &= 0b11111000;   // Set ADC to free running mode

  *my_ADMUX  &= 0b01111111;   // Set AVCC as analog reference by clearing bit 7
  *my_ADMUX  |= 0b01000000;   // Set AVCC as analog reference by setting bit 6
  *my_ADMUX  &= 0b11011111;   // Right adjust ADC result (clear bit 5)
  *my_ADMUX  &= 0b11100000;   // Clear channel selection bits in ADMUX
}

/**
 * @brief Reads a specific bit from a register. 
 * 
 * @param address Pointer to the register's address.
 * @param bit The bit position to read (0-7).
 * @return int Returns 1 if the bit is set, 0 otherwise.
 */
unsigned int readRegister(unsigned char* address, int bit) {
    //Returns 1 if the bit is set, 0 otherwise
    return (*address & (1 << bit)) ? 1 : 0;  
}

/**
 * @brief Uses the ADC to read and return the analog data !!!REDO COMMENTS!!!
 * 
 * @param ADC_PIN The specific analog pin number being read from.
 * @return unsigned int Returns a range from 0 to 1023
 */
unsigned int adc_read(unsigned char adc_channel_num)
{
  *my_ADMUX  &= 0b11100000;   // Clear channel selection bits (MUX 4:0)
  *my_ADCSRB &= 0b11110111;   // Clear MUX 5 bit

  if(adc_channel_num > 7)
  {
    adc_channel_num -= 8;        // Adjust channel number for high channels
    *my_ADCSRB |= 0b00001000;    // Set MUX 5 bit for high channels
  }
  
  *my_ADMUX  += adc_channel_num; // Set MUX bits to select the channel
  *my_ADCSRA |= 0x40;            // Start ADC conversion by setting bit 6

  while((*my_ADCSRA & 0x40) != 0); // Wait for conversion to complete

  return *my_ADC_DATA;           // Return ADC result
}

/**
 * @brief Writes a specific value (1 or 0) to a bit in a register.
 * 
 * @param address Pointer to the register's address.
 * @param bit The bit position to modify (0-7).
 * @param value The value to write (1 to set the bit, 0 to clear it).
 */
void writeRegister(unsigned char* address, int bit, int value) {
    if (value == 1) {
        *address |= (1 << bit);  //Set the bit to 1 (HIGH)
    } else {
        *address &= ~(1 << bit);  //Set the bit to 0 (LOW)
    }
}
