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
 * Interrupt/Reset/ON-OFF Button (Definitions/Includes)
 */
volatile unsigned char* DDR_D = (unsigned char*) 0x2A;  // For PWM PIN 2 and 3
volatile unsigned char* PORT_D = (unsigned char*) 0x2B; // OUT
volatile unsigned char* PIN_D = (unsigned char*) 0x09; // READ
volatile const int resetPin = 2; // Reset button pin
volatile const int togglePin = 3; //ON-OFF button pin
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

/**
 * System State Information
 */
volatile int stateNum = 0; //DISABLED(Y) = 0, IDLE(G) = 1,RUNNING(B) = 2, ERROR(R) = 3
volatile const int waterLevelThreshold = 75; //Errors if below water level; based on 100 and 300 lower/upper bound measurement
volatile const int tempThreshold = 21; //Going above should turn fan motor ON

volatile int waterLevel = 0; //Current System State Information
volatile int temperature = 0;
volatile int humidity = 0;

volatile bool fanState = 0; //For printing out information on fan/stepper transition.
volatile bool previousFanState = 0; //OFF = 0, ON = 1 
volatile bool stepperState = 0;
volatile bool previousStepperState = 0;

volatile unsigned long previousTime = 0;
volatile const long updateTime = 60000;

void setup(){
    U0init(9600); //Setting the UART Baud Rate

    rtc.begin(); // Start RTC object

    display.begin(16,2); // Initialize LCD (16 columns, 2 rows)
    display.clear(); // Clear LCD screen

    // LED Testing (DIGITAL 30,32,34,36 respectively) RYGB order
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

    digitalWrite(fanDIR1, LOW); // Set fan direction
    digitalWrite(fanDIR2, HIGH);

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

    writeRegister(DDR_D, 3, 0); // Set ON/OFF pin as input
    writeRegister(PORT_D, 3, 1); // Enable pull-up resistor
    attachInterrupt(digitalPinToInterrupt(togglePin), toggleState, FALLING); // Attach interrupt to ON/OFF pin

    // RTC time sync
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    display.write("Hello World!"); // Display a Hello World message on LCD
}

void loop(){
    unsigned long currentTime = millis(); //Time check

    unsigned char value_buffer[50]; // Buffer to convert numbers to printable strings

    // Temperature Sensing (DHT11 sensor)
    int chk = DHT_Sensor.read11(DHT11_PIN); // Read sensor data
    temperature = DHT_Sensor.temperature; // Get temperature reading
    humidity = DHT_Sensor.humidity; // Get humidity reading

    // LCD Printout
    display.clear(); // Clear display

    if(stateNum == 0){ //DISABLED MESSAGED
      display.setCursor(0, 0); // Set cursor to row 0, column 0
      display.print("DISABLED");
    } else if(stateNum == 1 || stateNum == 2){
      if(currentTime - previousTime >= updateTime){
        display.setCursor(0, 0); // Set cursor to row 0, column 0
        display.print("TEMP:: ");
        display.print(temperature); // Display temperature
        display.print("C");

        display.setCursor(0, 1); // Set cursor to row 1, column 0
        display.print("HUMIDITY:: ");
        display.print(humidity); // Display humidity
        display.print("%");

        previousTime = currentTime; //Save last update time 
      }
      
    } else if (stateNum == 3) { //ERROR MESSAGE
      display.setCursor(0, 0); // Set cursor to row 0, column 0
      display.print("Water level");
      display.setCursor(0, 1); // Set cursor to row 1, column 0
      display.print("is too low!");
    }

    // Water Level Measuring
    writeRegister(PORT_G, 1, 1); // Set water level sensor (DIGITAL PIN 40) to ON

    waterLevel = adc_read(1); // 100 lower bound, 300 upper bound calibration

    writeRegister(PORT_G, 1, 0); // Set water level sensor pin to OFF

    // LED State Display (DISABLED(Y) = 0, IDLE(G) = 1,RUNNING(B) = 2, ERROR(R) = 3)
    if(stateNum == 0){  // DISABLED (YELLOW)
      writeRegister(PORT_C, 5, 1);
      writeRegister(PORT_C, 7, 0); 
      writeRegister(PORT_C, 3, 0);
      writeRegister(PORT_C, 1, 0);
    } else if(stateNum == 1) { // IDLE (GREEN)
      writeRegister(PORT_C, 3, 1);
      writeRegister(PORT_C, 7, 0); 
      writeRegister(PORT_C, 5, 0);
      writeRegister(PORT_C, 1, 0);
    } else if(stateNum == 2){ // RUNNING (BLUE)
      writeRegister(PORT_C, 1, 1);
      writeRegister(PORT_C, 7, 0); 
      writeRegister(PORT_C, 5, 0);
      writeRegister(PORT_C, 3, 0);
    } else if(stateNum == 3){ // ERROR (RED)
      writeRegister(PORT_C, 7, 1); 
      writeRegister(PORT_C, 5, 0);
      writeRegister(PORT_C, 3, 0);
      writeRegister(PORT_C, 1, 0);
    }

    if(stateNum == 1){ //IDLE STATE
      if(waterLevel <= waterLevelThreshold){
        stateNum = 3;
      } else if(temperature > tempThreshold){
        stateNum = 2;
      }
    } else if(stateNum == 2){ //RUNING STATE
      if(waterLevel <= waterLevelThreshold){
        stateNum = 3;
      } else if(temperature <= tempThreshold){
        stateNum = 1;
      }
    } //ERROR and DISABLED state handled by reset() and toggleState()
    
    if(stateNum == 2){ // Fan Motor Testing (Run fan)
      fanState = 1;
      analogWrite(fanSpeedPIN, 255); // Set fan speed to full
    } else {
      fanState = 0;
      analogWrite(fanSpeedPIN, 0); // Stop fan
    }
    
    if(stateNum != 3){ // Stepper Motor Testing (Control stepper motor based on potentiometer)
      int stepperControl = adc_read(2); // Read potentiometer (analog A2)
      if(stepperControl >= 800){
          stepperState = 1;
          ventMotor.step(100); // Step motor forward
      } else if (stepperControl <= 200){
          stepperState = 1;
          ventMotor.step(-100); // Step motor backward
      } else {
          stepperState = 0;
      }
    }

    // RTC Time Record for Fan/Stepper Transition
    DateTime now = rtc.now();
    
    if(previousFanState != fanState){ //Check for fan transition for RTC printout
      dateTimePrint(now);
      UART_print(": ");
      if(previousFanState == 0){
        UART_println("Fan was turned ON!");
      } else {
        UART_println("Fan was turned OFF!");
      }
    } 

    if(previousStepperState != stepperState){ //Check for fan transition for RTC printout
      dateTimePrint(now);
      UART_print(": ");
      if(previousStepperState == 0){
        UART_println("Vent started changing position/Stepper Motor turned on!");
      } else {
        UART_println("Vent stopped changing position/Stepper Motor turned off!");
      }
    } 

    previousFanState = fanState;
    previousStepperState = stepperState;
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
 * INTERRUPT FUNCTIONs
 */

/**
 * @brief Resets the reset counter on a button press.
 *        This function is triggered by an interrupt from a reset button.
 *        It increments a global variable `resetti` each time it is called.
 */
void reset(){
  if(stateNum == 3){
    stateNum = 1;
  }
}

/**
 * @brief 
 * 
 */
void toggleState(){
  if(stateNum == 0){
    stateNum = 1;
  } else {
    stateNum = 0;
  }
}

/**
 * PRINT FUNCTIONs 
 * 
 */
 
 /**
  * @brief 
  * 
  * @param now 
  */
 void dateTimePrint(DateTime now){
    unsigned char value_buffer[50];
    sprintf(value_buffer, "%d", now.year()); // Convert raw water level to string
    UART_print(value_buffer);
    UART_print("/");
    sprintf(value_buffer, "%d", now.month()); // Convert raw water level to string
    UART_print(value_buffer);
    UART_print("/");
    sprintf(value_buffer, "%d", now.day()); // Convert raw water level to string
    UART_print(value_buffer);
    UART_print(" (");
    UART_print(daysOfTheWeek[now.dayOfTheWeek()]);
    UART_print(") ");
    sprintf(value_buffer, "%d", now.hour()); // Convert raw water level to string
    UART_print(value_buffer);
    UART_print(":");
    sprintf(value_buffer, "%d", now.minute()); // Convert raw water level to string
    UART_print(value_buffer);
    UART_print(":");
    sprintf(value_buffer, "%d", now.second()); // Convert raw water level to string
    UART_print(value_buffer);
    UART_print("");
 }

/**
 * @brief 
 * 
 * @param now 
 */
 void dateTimePrintln(DateTime now){
    dateTimePrint(now);
    UART_println("");
 }


/**
 *  UART FUNCTIONs
 */

/**
 * @brief Initializes the UART0 communication with a specified baud rate.
 *        This function configures the UART registers, calculates the baud rate,
 *        and sets up the transmission and reception mode.
 * 
 * @param U0baud The desired baud rate for UART communication (e.g., 9600).
 */
void U0init(int U0baud)
{
    unsigned long FCPU = 16000000;           // CPU frequency (16 MHz)
    unsigned int tbaud = (FCPU / 16 / U0baud - 1); // Calculate baud rate setting based on desired baud rate

    *myUCSR0A = 0x20; // Set UART Control and Status Register A to configure the UART settings
    *myUCSR0B = 0x18; // Enable the receiver and transmitter for UART communication
    *myUCSR0C = 0x06; // Set UART mode and data format (8 data bits, 1 stop bit, no parity)
    *myUBRR0  = tbaud; // Set the calculated baud rate into the UART Baud Rate Register
}

/**
 * @brief Checks if there is data available in the UART receive buffer.
 *        This function checks the RDA (Receive Data Available) flag in the UART status register.
 * 
 * @return unsigned char Returns a non-zero value if data is available to be read.
 */
unsigned char U0kbhit()
{
    return *myUCSR0A & RDA; // Check RDA flag in UART status register to see if data is available
}

/**
 * @brief Reads a byte of data from the UART Data Register.
 *        This function retrieves the byte of data from the UART receiver.
 * 
 * @return unsigned char The byte of data received from the UART buffer.
 */
unsigned char U0getchar()
{
    return *myUDR0; // Return the data stored in the UART Data Register (received byte)
}

/**
 * @brief Sends a byte of data via UART.
 *        This function waits for the transmit buffer to be empty and then sends the data byte.
 * 
 * @param U0pdata The byte of data to be transmitted via UART.
 */
void U0putchar(unsigned char U0pdata)
{
    while((*myUCSR0A & TBE) == 0); // Wait until the transmit buffer is empty (TBE flag is set)
    *myUDR0 = U0pdata;             // Load the data byte into the UART Data Register to transmit
}

/**
 * @brief Prints a string via UART.
 *        This function transmits each character of the input string until the null terminator is reached.
 * 
 * @param cstring A pointer to the null-terminated string to be printed.
 */
void UART_print(unsigned char* cstring){
    int i = 0;
    while(cstring[i] != '\0'){      // Loop until null terminator is reached
        U0putchar((unsigned char)(cstring[i])); // Transmit each character of the string
        i++;
    }
}

/**
 * @brief Prints a string followed by a newline via UART.
 *        This function transmits each character of the input string and then sends a newline character.
 * 
 * @param cstring A pointer to the null-terminated string to be printed.
 */
void UART_println(unsigned char* cstring){
    int i = 0;
    while(cstring[i] != '\0'){      // Loop until null terminator is reached
        U0putchar((unsigned char)(cstring[i])); // Transmit each character of the string
        i++;
    }
    U0putchar('\n'); // Send a newline character at the end of the string
}
