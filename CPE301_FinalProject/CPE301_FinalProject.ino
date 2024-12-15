// CPE 301 Final Project
// Written by Lucas Pinto and Finn Campbell

/**
 * LCD Display Includes !!!NEED DOCUMENTATION/REFERENCE SHEET!!!
 * ..> LCD additionallu uses a 330 OHM resistor
 */
#include <LiquidCrystal.h>
const int RS = 39, EN = 41, D4 = 31, D5 = 33, D6 = 35, D7 = 37; 
LiquidCrystal display(RS, EN, D4, D5, D6, D7);

/**
 * LED System !!!NEED DOCUMENTATION/REFERENCE SHEET NEEDED!!!
 * ..> const int LED_R = 30, LED_Y = 32, LED_G = 34, LED_B = 36;
 * DIGITAL PINs 30-37 correspond to BITs 7-0 
 * ..> (30~7 ; 31~6 ; 32~5 ; 33~4 ; 34~3 ; 35~2 ; 36~1 ; 37~0)
 */
volatile unsigned char* PORT_C = (unsigned char*) 0x28;
volatile unsigned char* DDR_C = (unsigned char*) 0x27; 

/**
 * Stepper Motor !!!NEED DOCUMENTATION/REFERENCE SHEET NEEDED!!!
 */
#include <Stepper.h>
const int stepsPerRevolution = 2038;
Stepper  ventMotor = Stepper(stepsPerRevolution, 45, 47, 49, 51);

/**
 * DHT11 Water Sensor !!!NEED DOCUMENTATION/REFERENCE SHEET NEEDED!!!
 */
#include <dht.h>
const int DHT11_PIN = 54; //Pin A0
dht DHT_Sensor;

/**
 * Function Definitions
 */
//Register Manipulators
void writeRegister(unsigned char* address, int bit, int value);
int readRegister(unsigned char* address, int bit);



void setup(){
    Serial.begin(9600);

    display.begin(16,2);
    display.clear();
    
    //Testing Display
    display.write("Hello World!");

    //LED Testing (DIGITAL 30,32,34,36 respectively)
    writeRegister(DDR_C, 7, 1);
    writeRegister(DDR_C, 5, 1);
    writeRegister(DDR_C, 3, 1);
    writeRegister(DDR_C, 1, 1);

    writeRegister(PORT_C, 7, 0);
    writeRegister(PORT_C, 5, 0);
    writeRegister(PORT_C, 3, 0);
    writeRegister(PORT_C, 1, 0);

    //Motor Testing
    ventMotor.setSpeed(5);
    ventMotor.step(stepsPerRevolution);
}

void loop(){
    int chk = DHT_Sensor.read11(DHT11_PIN);
    int temperature = DHT_Sensor.temperature; //Already at integer precision
    int humidity = DHT_Sensor.humidity; //So no information lost doing this

    display.clear();

    display.setCursor(0,0);
    display.print("TEMP:: ");
    display.print(temperature);
    display.print("C");

    display.setCursor(0,1);
    display.print("HUMIDITY:: ");
    display.print(humidity);
    display.print("%");

    delay(100);
}

ISR(TIMER1_OVF_vect)
{

}

/**
 * @brief Reads a specific bit from a register. 
 * 
 * @param address Pointer to the register's address.
 * @param bit The bit position to read (0-7).
 * @return int Returns 1 if the bit is set, 0 otherwise.
 */
int readRegister(unsigned char* address, int bit) {
    //Returns 1 if the bit is set, 0 otherwise
    return (*address & (1 << bit)) ? 1 : 0;  
}

/// IMPLEMENT ANALOG READ LATER

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
