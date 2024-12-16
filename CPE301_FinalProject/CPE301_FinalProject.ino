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
 * LCD1602 16x2 Display (Definitions/Includes)
 * ..> LCD additionallu uses a 330 OHM resistor
 */
#include <LiquidCrystal.h>
const int RS = 39, EN = 41, D4 = 31, D5 = 33, D6 = 35, D7 = 37; 
LiquidCrystal display(RS, EN, D4, D5, D6, D7);

/**
 * LED System (Definitions/Includes)
 * ..> const int LED_R = 30, LED_Y = 32, LED_G = 34, LED_B = 36
 * DIGITAL PINs 30-37 correspond to BITs 7-0 
 * ..> (30~7 ; 31~6 ; 32~5 ; 33~4 ; 34~3 ; 35~2 ; 36~1 ; 37~0)
 */
volatile unsigned char* PORT_C = (unsigned char*) 0x28; //WRITE address, 0x08 is READ!!!
volatile unsigned char* DDR_C = (unsigned char*) 0x27; //WRITE address, 0x07 is READ!!!

/**
 * 28BYJ-48 Stepper Motor + ULN2004 Driver Module (Definitions/Includes)
 */
#include <Stepper.h>
const int stepsPerRevolution = 2038;
Stepper  ventMotor = Stepper(stepsPerRevolution, 45, 49, 47, 51); // Initialize stepper motor with pins

/**
 * Fan Motor + L293D Driver (Definitions/Includes)
 */
const int fanSpeedPIN = 12, fanDIR1 = 11, fanDIR2 = 13, fanSpeed = 90; // Pins for controlling fan motor speed and direction

/**
 * DHT11 Water Sensor (Definitions/Includes)
 */
#include <dht.h>
const int DHT11_PIN = 54; //Analog Pin A0 for DHT11 sensor
volatile unsigned char* PORT_G = (unsigned char*) 0x34; //WRITE address, 0x14 is READ!!!
volatile unsigned char* DDR_G = (unsigned char*) 0x33; //WRITE address, 0x13 is READ!!!
dht DHT_Sensor; // Initialize DHT11 sensor

/**
 * DS1307 RTC Module (Definitions/Includes)
 */
#include <RTClib.h>
RTC_DS1307 rtc; // RTC DS1307 object
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday",
"Friday", "Saturday"}; // Array of days of the week for RTC display

/**
 * Water Level Detection Sensor Module (Definitions/Includes)
 */
 // ADC Registers for analog-to-digital conversion
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;    // ADC Multiplexer Selection Register
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;   // ADC Control and Status Register B
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;   // ADC Control and Status Register A
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;   // ADC Data Register

/**
 * Interrupt/Reset Button (Definitions/Includes)
 */
volatile unsigned char* DDR_D = (unsigned char*) 0x2A;  // PWM PIN 2
volatile unsigned char* PORT_D = (unsigned char*) 0x2B;
const int resetPin = 2; // Reset button pin
volatile int resetti = 0; // Reset counter

/**
 * UART Definitions (Definitions/Includes)
 */
#define RDA 0x80  
#define TBE 0x20  
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0; // UART Status Register A
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1; // UART Status Register B
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2; // UART Status Register C
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4; // UART Baud Rate Register
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6; // UART Data Register

/**
 * Function Definitions
 */
 // Register Manipulators
void writeRegister(unsigned char* address, int bit, int value); // Function to write to register
unsigned int adc_read(unsigned char adc_channel_num); // Function to read ADC data
unsigned int readRegister(unsigned char* address, int bit); // Function to read from register

// Interrupt Function
void reset(); // Interrupt function for reset button

// UART Functions
void U0init(int U0baud); // Function to initialize UART
unsigned char U0kbhit(); // Function to check if data is available in UART
unsigned char U0getchar(); // Function to receive a character from UART
void U0putchar(unsigned char U0pdata); // Function to send a character through UART
void UART_print(unsigned char* cstring); // Function to print string over UART
void ADC_setup(); // Function to initialize ADC conversion

void setup(){
    U0init(9600); //Setting the UART Baud Rate

    rtc.begin(); // Start RTC object

    display.begin(16,2); // Initialize LCD (16 columns, 2 rows)
    display.clear(); // Clear LCD screen

    // LED Testing (DIGITAL 30,32,34,36 respectively)
    writeRegister(DDR_C, 7, 1); // Set pin to OUTPUT
    writeRegister(DDR_C, 5, 1);
    writeRegister(DDR_C, 3, 1);
    writeRegister(DDR_C, 1, 1);

    writeRegister(PORT_C, 7, 0); // Set pin to OFF
    writeRegister(PORT_C, 5, 0);
    writeRegister(PORT_C, 3, 0);
    writeRegister(PORT_C, 1, 0);

    // Fan Motor Testing
    pinMode(fanSpeedPIN, OUTPUT); // Set fan speed pin as OUTPUT
    pinMode(fanDIR1, OUTPUT); // Set fan direction pin 1 as OUTPUT
    pinMode(fanDIR2, OUTPUT); // Set fan direction pin 2 as OUTPUT

    // Analog Read Setup (for water level sensor and potentiometer)
    ADC_setup();

    // Water Level Sensor (ANALOG A1)
    writeRegister(DDR_G, 1, 1); // Set pin A1 to OUTPUT
    writeRegister(PORT_G, 1, 0); // Set pin A1 to OFF

    // Vent Motor Init
    ventMotor.setSpeed(5); // Set vent motor speed

    // Reset Button Setup (attach interrupt)
    writeRegister(DDR_D, 2, 0); // Set reset pin as input
    writeRegister(PORT_D, 2, 1); // Enable pull-up resistor
    attachInterrupt(digitalPinToInterrupt(resetPin), reset, FALLING); // Attach interrupt to reset pin

    // RTC time sync
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    display.write("Hello World!"); // Display a Hello World message on LCD
}

void loop(){
    unsigned char value_buffer[50]; // Buffer to convert numbers to printable strings

    // Temperature Sensing (DHT11 sensor)
    int chk = DHT_Sensor.read11(DHT11_PIN); // Read sensor data
    int temperature = DHT_Sensor.temperature; // Get temperature reading
    int humidity = DHT_Sensor.humidity; // Get humidity reading

    // Interrupt Testing (output reset count)
    sprintf(value_buffer, "%d", resetti); // Convert reset count to string
    UART_print("The hell???:: ");
    UART_println(value_buffer);

    // LCD Printout
    display.clear(); // Clear display

    display.setCursor(0, 0); // Set cursor to row 0, column 0
    display.print("TEMP:: ");
    display.print(temperature); // Display temperature
    display.print("C");

    display.setCursor(0, 1); // Set cursor to row 1, column 0
    display.print("HUMIDITY:: ");
    display.print(humidity); // Display humidity
    display.print("%");

    // TEMPORARY WATER LEVEL TESTING
    writeRegister(PORT_G, 1, 1); // Set water level sensor (DIGITAL PIN 40) to ON

    int rawWaterLevel = adc_read(1); // 100 lower bound, 300 upper bound calibration

    sprintf(value_buffer, "%d", rawWaterLevel); // Convert raw water level to string
    UART_print("Water Level Raw Value:: ");
    //UART_println(value_buffer); // Print water level value
    Serial.println(rawWaterLevel);
    

    writeRegister(PORT_G, 1, 0); // Set water level sensor pin to OFF

    // Fan Motor Testing (Run fan)
    digitalWrite(fanDIR1, LOW); // Set fan direction
    digitalWrite(fanDIR2, HIGH);
    analogWrite(fanSpeedPIN, 255); // Set fan speed to full

    // Stepper Motor Testing (Control stepper motor based on potentiometer)
    int stepperControl = adc_read(2); // Read potentiometer (analog A2)
    if(stepperControl >= 800){
        ventMotor.step(100); // Step motor forward
    } else if (stepperControl <= 200){
        ventMotor.step(-100); // Step motor backward
    }

    // RTC Testing (optional code for RTC display)
    // DateTime now = rtc.now();
    // Serial.print(now.year(), DEC);
    // Serial.print('/');
    // Serial.print(now.month(), DEC);
    // Serial.print('/');
    // Serial.print(now.day(), DEC);
    // Serial.print(" (");
    // Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    // Serial.print(") ");
    // Serial.print(now.hour(), DEC);
    // Serial.print(':');
    // Serial.print(now.minute(), DEC);
    // Serial.print(':');
    // Serial.print(now.second(), DEC);
    // Serial.println();

    delay(500);
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

/**
 * @brief 
 * 
 */
void reset(){
    resetti++; // Increment reset count on button press
}

/**
 * @brief 
 * 
 * @param U0baud 
 */
void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;           // CPU frequency
 unsigned int tbaud = (FCPU / 16 / U0baud - 1); // Calculate baud rate

 *myUCSR0A = 0x20; // Set UART Control and Status Register A
 *myUCSR0B = 0x18; // Enable receiver and transmitter
 *myUCSR0C = 0x06; // Set UART mode and data format
 *myUBRR0  = tbaud; // Set calculated baud rate
}

/**
 * @brief 
 * 
 * @return unsigned char 
 */
unsigned char U0kbhit()
{
  return *myUCSR0A & RDA; // Check RDA flag
}

/**
 * @brief 
 * 
 * @return unsigned char 
 */
unsigned char U0getchar()
{
  return *myUDR0; // Return data from UART Data Register
}

/**
 * @brief 
 * 
 * @param U0pdata 
 */
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE) == 0); // Wait until transmit buffer is empty
  *myUDR0 = U0pdata;             // Load data into UART Data Register
}

/**
 * @brief 
 * 
 * @param cstring 
 */
void UART_print(unsigned char* cstring){
  int i = 0;
  while(cstring[i] != '\0'){      // Loop until null terminator
    U0putchar((unsigned char)(cstring[i])); // Transmit each character
    i++;
  }
}

/**
 * @brief 
 * 
 * @param cstring 
 */
void UART_println(unsigned char* cstring){
  int i = 0;
  while(cstring[i] != '\0'){      // Loop until null terminator
    U0putchar((unsigned char)(cstring[i])); // Transmit each character
    i++;
  }
  U0putchar('\n');
}