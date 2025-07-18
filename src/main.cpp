#include <avr/io.h>
#include <avr/interrupt.h>
#include "Arduino.h"
#include <util/delay.h>
#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h> 
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <SPI.h>

#define R_SENSE 0.075f // Match to your driver

using namespace std;

#define F_CPU 16000000UL // Adjust clock frequency as needed
#define MAX_STRING_LENGTH 100 // Adjust this value as needed
#define BAUD 115200
#define UBRR0_VALUE ((F_CPU / (8UL * BAUD)) - 1) 

#define MISO_PIN 50
#define MOSI_PIN 51
#define SCK_PIN 52

#define BASE_STEP_PIN 34
#define BASE_DIR_PIN 30
#define BASE_ENA_PIN 22
#define BASE_CS_PIN 26

#define SHOULDER_STEP_PIN 35
#define SHOULDER_DIR_PIN 31
#define SHOULDER_ENA_PIN 23
#define SHOULDER_CS_PIN 27

#define ELBOW_STEP_PIN 36
#define ELBOW_DIR_PIN 32
#define ELBOW_ENA_PIN 24
#define ELBOW_CS_PIN 28

#define FOREARM_STEP_PIN 37
#define FOREARM_DIR_PIN 33
#define FOREARM_ENA_PIN 25
#define FOREARM_CS_PIN 29

#define LASER_PIN 38
#define LASER_TRIGGER 39

#define WRIST1_PWM_PIN  A15//
#define WRIST2_PWM_PIN A14//

#define GRIPPER_PIN 40//

#define FLOAT_TOLERANCE 1e-5 // Tolerance for floating-point comparison

// blue wire outlet is yellow, redwire outlet is purple

char *inputString;
String receivedString;
volatile unsigned long timermicros = 0;

TMC5160Stepper BASEDriver = TMC5160Stepper(BASE_CS_PIN, R_SENSE);
TMC5160Stepper SHOULDERDriver = TMC5160Stepper(SHOULDER_CS_PIN, R_SENSE);
TMC5160Stepper ELBOWDriver = TMC5160Stepper(ELBOW_CS_PIN, R_SENSE);
TMC5160Stepper FOREARMDriver = TMC5160Stepper(FOREARM_CS_PIN, R_SENSE);

struct JOINTStruct {
    uint8_t STEP_PIN;
    uint8_t DIR_PIN;
    uint8_t ENA_PIN;
    int DIR;
    long STEPS; // amount of moves
    long SPEED; // interval
    bool STATE; // active or not
    float ANGLE_TRUE;
    float ANGLE_IDEAL;
    bool HOMESTATUS;
    unsigned long LASTSTEP;
    float STEP_FACTOR; // the amount of steps to change 1 degree
};

JOINTStruct BaseJoint;
JOINTStruct ShoulderJoint;
JOINTStruct ElbowJoint;
JOINTStruct ForearmJoint;

JOINTStruct* BASE = &BaseJoint;
JOINTStruct* SHOULDER = &ShoulderJoint;
JOINTStruct* ELBOW = &ElbowJoint;
JOINTStruct* FOREARM = &ForearmJoint;

JOINTStruct* JOINTARRAY[4] = {BASE, SHOULDER, ELBOW, FOREARM};


void init_timer3() {
    // Set Timer3 to normal mode (counts from 0 to 65535)
    TCCR3A = 0x00;

    // Set prescaler to 64
    TCCR3B = (1 << CS31); //| (1 << CS30);

    // Enable Timer3 overflow interrupt
    TIMSK3 = (1 << TOIE3);

    // Initialize Timer3 for 1 ms overflow
    TCNT3 = 65535 - (F_CPU / 8 / 1000000) + 1;

    // Enable global interrupts
    sei();
}

ISR(TIMER3_OVF_vect) {
    // Increment the millis counter
    timermicros++;

    // Reset Timer3 for 1 ms overflow
    TCNT3 = 65535 - (F_CPU / 64 / 1000) + 1;
}

unsigned long microseconds() {
    unsigned long micros;
    uint8_t oldSREG = SREG; // Save the current interrupt status
    cli(); // Disable interrupts
    micros = timermicros; // Read the volatile variable
    SREG = oldSREG; // Restore the interrupt status
    return micros;
}


void jointAssignment(){
    // assign base values

    BASE -> STEP_PIN = BASE_STEP_PIN;
    BASE -> DIR_PIN = BASE_DIR_PIN;
    BASE -> ENA_PIN = BASE_ENA_PIN;
    BASE -> DIR = 0;
    BASE -> STEPS = 0;
    BASE -> SPEED = 0;
    BASE -> ANGLE_TRUE = 0;
    BASE -> ANGLE_IDEAL = 0;
    BASE -> HOMESTATUS = false;
    BASE -> STEP_FACTOR = 300; // placeholder, means number of steps per degree


    //assign shoulder values 
    SHOULDER -> STEP_PIN = SHOULDER_STEP_PIN;
    SHOULDER -> DIR_PIN = SHOULDER_DIR_PIN;
    SHOULDER -> ENA_PIN = SHOULDER_ENA_PIN;
    SHOULDER -> DIR = 0;
    SHOULDER -> STEPS = 0;
    SHOULDER -> SPEED = 0;
    SHOULDER -> ANGLE_TRUE = 0;
    SHOULDER -> ANGLE_IDEAL = 0;
    SHOULDER -> HOMESTATUS = false;
    SHOULDER -> STEP_FACTOR = 300; //placeholder

    ELBOW -> STEP_PIN = ELBOW_STEP_PIN; 
    ELBOW -> DIR_PIN = ELBOW_DIR_PIN; 
    ELBOW -> ENA_PIN = ELBOW_ENA_PIN; 
    ELBOW -> DIR = 0;
    ELBOW -> STEPS = 0; 
    ELBOW -> SPEED = 0; 
    ELBOW -> ANGLE_TRUE = 0; 
    ELBOW -> ANGLE_IDEAL = 0; 
    ELBOW -> HOMESTATUS = false;
    ELBOW -> STEP_FACTOR = 300;

    FOREARM -> STEP_PIN = FOREARM_STEP_PIN; 
    FOREARM -> DIR_PIN = FOREARM_DIR_PIN; 
    FOREARM -> ENA_PIN = FOREARM_ENA_PIN; 
    FOREARM -> DIR = 0;
    FOREARM -> STEPS = 0; 
    FOREARM -> SPEED = 0; 
    FOREARM -> ANGLE_TRUE = 0; 
    FOREARM -> ANGLE_IDEAL = 0; 
    FOREARM -> HOMESTATUS = false;
    FOREARM -> STEP_FACTOR = 300;
    
}

void ftoa(float value, char* buffer, int decimalPlaces) {
    // Handle negative numbers
    if (value < 0) {
        *buffer++ = '-';
        value = -value;
    }

    // Extract the integer part
    int intPart = (int)value;
    itoa(intPart, buffer, 10); // Convert integer part to string
    while (*buffer) buffer++; // Move pointer to the end of the integer part

    // Add decimal point
    *buffer++ = '.';

    // Extract the fractional part
    float fractionalPart = value - intPart;
    for (int i = 0; i < decimalPlaces; i++) {
        fractionalPart *= 10;
        int digit = (int)fractionalPart;
        *buffer++ = '0' + digit;
        fractionalPart -= digit;
    }

    // Null-terminate the string
    *buffer = '\0';
}

void USART_Init() {
    // Set baud rate
    UBRR0H = (uint8_t)(UBRR0_VALUE >> 8);
    UBRR0L = (uint8_t)(UBRR0_VALUE);
    // Enable double-speed mode
    UCSR0A = (1 << U2X0);
    // Enable transmitter and receiver
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void USART_Transmit(char data) {
    // Wait for the transmit buffer to be empty
    while (!(UCSR0A & (1 << UDRE0)));
    // Put data into the buffer, sends the data
    UDR0 = data;
}

void USART_SendString(const char* str) {
    while (*str) {
        USART_Transmit(*str++);
    }
}

char USART_Receive() {
    // Wait for data to be received
    while (!(UCSR0A & (1 << RXC0)));
    // Return received data
    return UDR0;
}

void USART_ReceiveString(char* buffer, uint8_t max_length) {
    uint8_t index = 0;
    char received_char;

    // Read characters until newline or buffer is full
    while (index < max_length) {
        received_char = USART_Receive();

        // Check for newline (end of string)
        if (received_char == '\n' || received_char == '\r') {
            break;
        }
        // Store the received character
        buffer[index++] = received_char;
    }
    // Null-terminate the string
    buffer[index+1] = '\0';
    USART_SendString(buffer);
    USART_SendString("\n");
}

int integerExtract(const char* str, uint8_t start, uint8_t end) {
    char substring[10]; // Buffer to hold the substring
    uint8_t length = end - start + 1; // Length of the substring
    //USART_SendString(str);
    USART_SendString("\n");

    // Copy the substring from the original string
    strncpy(substring, str + start, length);
    substring[length+1] = '\0'; // Null-terminate the substring

    // Convert the substring to an integer
    return atoi(substring);
}

bool USART_Available() {
    return (UCSR0A & (1 << RXC0)); // Check if data is available in the receive buffer
}

void stateCheck(){
    for(int joint=1; joint<5; joint++){
        JOINTStruct* CURRENTJOINT = JOINTARRAY[joint];

        int ANGLE = CURRENTJOINT->ANGLE_IDEAL;
        int SPEEDSETTING = CURRENTJOINT->SPEED;
        char buffer[10]; // Adjust the size as needed
        char buffer1[10];
        char jointid[10];

        itoa(joint, jointid, 10);
        itoa(ANGLE, buffer, 10); // Convert integer to string
        itoa(SPEEDSETTING, buffer1, 10);

        USART_SendString("Joint ID: "); 
        USART_SendString(jointid);            
        USART_SendString("\r\n"); 
        USART_SendString("Joint Angle: "); 
        USART_SendString(buffer);            
        USART_SendString("\r\n"); 

        USART_SendString("Joint Speed: "); 
        USART_SendString(buffer1);
        USART_SendString("\r\n"); 
        USART_SendString("\r\n"); 
    }
}

void stateControls(JOINTStruct* JOINT) {
    float Ideal_Angle = JOINT->ANGLE_IDEAL;
    float Actual_Angle = JOINT->ANGLE_TRUE;
    float Ratio = JOINT->STEP_FACTOR;

    // Check if the angles are approximately equal
    if (fabsf(Ideal_Angle - Actual_Angle) <= FLOAT_TOLERANCE) {
        return; // Angles are close enough, no action needed
    }
    else {
        // Determine which angle is larger
        if (Ideal_Angle > Actual_Angle) {
            USART_SendString("current angle too low\n");
            //int direction = HIGH;
            JOINT->DIR = HIGH;
        }
        else if (Ideal_Angle < Actual_Angle) {
            USART_SendString("current angle too high\n");
            //int direction = LOW;
            JOINT->DIR = LOW;
        }

        // Calculate the angle delta and steps
        float angleDelta = fabsf(Ideal_Angle - Actual_Angle);
        unsigned long steps = (unsigned long)(angleDelta * Ratio);

        // Convert values to strings for printing
        char currentAngleholder[12];
        char idealAngleholder[12];
        char printableDelta[12];
        char stepsholder[12];
        char ratioHolder[12];

        ltoa(steps, stepsholder, 10);
        ltoa((long)Ratio, ratioHolder, 10); // Cast Ratio to long for ltoa
        ftoa(angleDelta, printableDelta, 4);
        ftoa(Ideal_Angle, idealAngleholder, 4);
        ftoa(Actual_Angle, currentAngleholder, 4);

        // Send data via USART
        USART_SendString(stepsholder);
        USART_SendString("\n");
        USART_SendString("Current Angle: ");
        USART_SendString(currentAngleholder);
        USART_SendString("\n");
        USART_SendString("Ideal Angle: ");
        USART_SendString(idealAngleholder);
        USART_SendString("\n");
        USART_SendString(printableDelta);
        USART_SendString("\n");
        USART_SendString(ratioHolder);
        USART_SendString("\n");

        // Update the joint's steps
        JOINT->STEPS = steps;
    }
}

void motorDriver(JOINTStruct* JOINT) {  
    // this function will live in the main loop and manipulate motors on a per need basis
    // get STEP COUNT, DIRECTION, and SPEED, and last step
    unsigned long LAST_STEP = JOINT->LASTSTEP;
    unsigned long CurrentTime = microseconds();
    int DIR = JOINT -> DIR;
    // int SPEED = JOINT -> SPEED;
    unsigned long STEPCOUNT = JOINT -> STEPS;
    uint8_t STEP_PIN = JOINT->STEP_PIN;
    uint8_t DIR_PIN = JOINT->DIR_PIN;
    // uint8_t DIR_PIN = BASE->DIR_PIN;
    //uint8_t STEPPINGPIN = (JOINT->STEP_PIN);
    // char PINNUMBER[12];
    // char SECONDPINNUMBER[12];
    // USART_SendString(PINNUMBER);
    // USART_SendString("\n");

    //itoa(STEPPINGPIN, PINNUMBER, 10);

    float ratio = JOINT->STEP_FACTOR; //use this to calculate the angle
    // speed should be given in 0-10 settings but what dimensions???
    // use this to calculate interval 0-99 rpm?
    unsigned long INTERVAL = 0;

    if(STEPCOUNT > 0) {
    // investigate this  interval thing
        digitalWrite(DIR_PIN, DIR);
        if(CurrentTime - LAST_STEP > INTERVAL) {
            // USART_SendString("available");
            // USART_SendString("\n");
            //do the step thing, i guess toggle the pin state of the step pin
            int pinState = !digitalRead(STEP_PIN);
            digitalWrite(STEP_PIN, pinState);

            // itoa(BASE_STEP_PIN, PINNUMBER, 10);
            // itoa(STEPPINGPIN, SECONDPINNUMBER, 10);

            // USART_SendString("Pin Number1: ");
            // USART_SendString(PINNUMBER);
            // USART_SendString("\n");
            // USART_SendString("Pin Number2: ");
            // USART_SendString(SECONDPINNUMBER);
            // USART_SendString("\n");


            STEPCOUNT --;
            // char printableSteps[12];
            // char printableAngle[12];
            // char printableDir[12];

            // ltoa(STEPCOUNT, printableSteps, 10);

            // USART_SendString("Step Count: ");
            // USART_SendString(printableSteps);
            // USART_SendString("\n");

            JOINT->STEPS = STEPCOUNT;
            JOINT->LASTSTEP = CurrentTime;            

            // itoa(directionValue, printableDir, 10);
            // USART_SendString("directional value: ");
            // USART_SendString(printableDir);
            // USART_SendString("\n");

            // but how do we handle the joint angles?
            // calculate it every time the joint steps but do state management only when a command enters
            // lets assume 6400 steps per revolution, use the 32 division factor

            float Angle = JOINT->ANGLE_TRUE;

            if (DIR < 1) {
                Angle -= 1/ratio; // Decrease Angle by the ratio
                // USART_SendString("direction is LOW");
                // USART_SendString("\n");
            } else if (DIR > 0) {
                Angle += 1/ratio; // Increase Angle by the ratio
                // USART_SendString("direction is HIGH");
                // USART_SendString("\n");
            }
            //ftoa(Angle, printableAngle, 2);
            // USART_SendString("Current Angle: ");
            // USART_SendString(printableAngle); 
            // USART_SendString("\n");
            JOINT->ANGLE_TRUE = Angle;
        }
    } 
}

void servoDriver(JOINTStruct *JOINT) {
    
}
//homing functions

void wristHoming() {
    // rotate wrist until it hits the home position
    // turn on laser pin, start receiving laser data 
    digitalWrite(LASER_PIN, HIGH);


    // rotate until laser blockage is detected 
    for(;;) {
        int Trigger_Status = digitalRead(LASER_TRIGGER);
        digitalWrite(FOREARM_STEP_PIN, HIGH);
        delay(25);
        digitalWrite(FOREARM_STEP_PIN, LOW);
        delay(25);
        if(Trigger_Status == 1) {
            Serial.println("okay Stanman we detected the stopping pin");
            break;
        }
    }
    // take steps and count until blockage passes 
    for(;;){
        int Trigger_Status = digitalRead(LASER_TRIGGER);
    }
    
}

int JointHoming(JOINTStruct* JOINT){
    // using a serial port to check for the values of sensorless homing
    // we are using sda and scl to read these pins correct? 
    // once homing is complete set the homed status as "good"

    //some use serial some use i2c....
    uint8_t pin = JOINT->DIR_PIN;
}

void initialize() {

    SPI.begin();

    USART_Init();
    init_timer3();

    jointAssignment();
    
    pinMode(LASER_PIN, OUTPUT);
    pinMode(LASER_TRIGGER, INPUT);

    pinMode(BASE_STEP_PIN, OUTPUT);
    pinMode(BASE_DIR_PIN, OUTPUT);

    pinMode(SHOULDER_STEP_PIN, OUTPUT);
    pinMode(SHOULDER_DIR_PIN, OUTPUT);
  
    pinMode(ELBOW_STEP_PIN, OUTPUT);
    pinMode(ELBOW_DIR_PIN, OUTPUT);
  
    pinMode(FOREARM_STEP_PIN, OUTPUT);
    pinMode(FOREARM_DIR_PIN, OUTPUT);

    BASEDriver.begin();
    BASEDriver.en_pwm_mode(1);
    BASEDriver.rms_current(2000); // Changed to 1000
    BASEDriver.microsteps(16);
    BASEDriver.TCOOLTHRS(0xFFFFF);
    BASEDriver.COOLCONF(0);
    BASEDriver.sgt(10);
  
    SHOULDERDriver.begin();
    SHOULDERDriver.en_pwm_mode(1);
    SHOULDERDriver.rms_current(2000); // Changed to 1000
    SHOULDERDriver.microsteps(16);
    SHOULDERDriver.TCOOLTHRS(0xFFFFF);
    SHOULDERDriver.COOLCONF(0);
    SHOULDERDriver.sgt(10);
  
    ELBOWDriver.begin();
    ELBOWDriver.en_pwm_mode(1);
    ELBOWDriver.rms_current(2000); // Changed to 1000
    ELBOWDriver.microsteps(16);
    ELBOWDriver.TCOOLTHRS(0xFFFFF);
    ELBOWDriver.COOLCONF(0);
    ELBOWDriver.sgt(10);
  
    FOREARMDriver.begin();
    FOREARMDriver.en_pwm_mode(1);
    FOREARMDriver.rms_current(1600); // Changed to 800
    FOREARMDriver.microsteps(16);
    FOREARMDriver.TCOOLTHRS(0xFFFFF);
    FOREARMDriver.COOLCONF(0);
    FOREARMDriver.sgt(10);
  
} 

void inputHandler(char *inputString) {
    if(USART_Available()){ 
        USART_ReceiveString(inputString, MAX_STRING_LENGTH);
        // command types
        // J1090005 joint 1 to 90.0 speed of 05rpm
        // J2180010 Joint 2 to 180.0 speed of 10rpm

        // implement speed later

        if (inputString[0] == 'H'){
            // do the homing
            char JOINTIDChar = inputString[1] - '0';

            int JOINTID = JOINTIDChar;
        }

        if (inputString[0] == 'S'){
            // this command checks states of everything
            stateCheck();
        }

        if (inputString[0] == 'J') {
            // i say we say fuck it and just construct here. 

            char testString[2]; 

            char JOINTIDChar = inputString[1] - '0'; 
            int JOINTID = JOINTIDChar; 

            itoa(JOINTID, testString, 10); 

            USART_SendString("joint id number: "); 
            USART_SendString(testString); 
            USART_SendString("\r\n"); 

            // you need to convert this to an integer before calling a string 
            // angle contructor 
            int commandAngle = integerExtract(inputString, 2, 4);
            int commandSpeed = integerExtract(inputString, 5, 7);

            // send it to joint

            JOINTARRAY[JOINTID]->ANGLE_IDEAL = commandAngle;
            JOINTARRAY[JOINTID]->SPEED = commandSpeed;
            // apply changes to joint
            stateControls(JOINTARRAY[JOINTID]);
            // clear input string
            inputString = "";

        } else {
            inputString = "";
        }
    }
    }

int main(void) {
    char receivedString[MAX_STRING_LENGTH];

    initialize();

    // main loop
    while(true) {
        // Com Handling Code

        inputHandler(inputString);
        // motor handling code
        motorDriver(BASE);
        motorDriver(SHOULDER);
        motorDriver(ELBOW);
        motorDriver(FOREARM);

        // unsigned long time1 = microseconds();
        // char timeHolder[12];

        // ltoa(time1, timeHolder, 10);
        // USART_SendString(timeHolder);
        // USART_SendString("\n");


    }

    return(0);
}

// you need the structures to serve as a state bank.
// if current and target is not the same keep rotating until you hit the target
// use a steps to angle calculator to figure out where your true angle is.