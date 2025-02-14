#include <avr/io.h>
#include "Arduino.h"
#include <util/delay.h>
#include "motordriver.h"
#include "TMCStepper.h"
#include <string.h>
#include <stdlib.h>

using namespace std;

#define F_CPU 16000000UL // Adjust clock frequency as needed
#define MAX_STRING_LENGTH 100 // Adjust this value as needed
#define BAUD 115200
#define UBRR0_VALUE ((F_CPU / (8UL * BAUD)) - 1) 

// these addresses may no longer be relevant
#define TEST_PIN PORTB7

#define BASE_STEP_PIN PORTB0 // 53
#define BASE_DIR_PIN PORTB2 // 51
#define BASE_ENA_PIN PORTL0 // 49

#define SHOULDER_STEP_PIN PORTG0 //41
#define SHOULDER_DIR_PIN PORTG2 // 39
#define SHOULDER_ENA_PIN PORTC0 // 37

#define ELBOW_STEP_PIN PORTC2 //35
#define ELBOW_DIR_PIN  PORTC4 // 33
#define ELBOW_ENA_PIN PORTC6 // 31

#define FOREARM_STEP_PIN PORTL2 // 47
#define FOREARM_DIR_PIN PORTL4 // 45
#define FOREARM_ENA_PIN PORTL6 // 43

#define WRIST1_PWM_PIN  //
#define WRIST2_PWM_PIN //

#define GRIPPER_PIN //

#define LASER_PIN 30
#define LASER_TRIGGER 32

// blue wire outlet is yellow, redwire outlet is purple

// Pin Definitions
// Stepper Motor 1: Base Rotation --> PIN22
// Stepper Motor 2: Base Hinge --> PIN23
// Stepper Motor 3: Elbow Driver --> PIN24
// Stepper Motor 4: Forearm Rotator --> PIN25 

// Function to initialize UART with a specific baud rate

//#define CONTROL_BIT 1 // Corresponds to bit 1 of Port B

String receivedString;

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
    float STEP_FACTOR; // the amount of degrees each step will change the joint
};

JOINTStruct BaseJoint;
JOINTStruct ShoulderJoint;
JOINTStruct ElbowJoint;
JOINTStruct ForearmJoint;

JOINTStruct INCOMINGJointCommand;

JOINTStruct* BASE = &BaseJoint;
JOINTStruct* SHOULDER = &ShoulderJoint;
JOINTStruct* ELBOW = &ElbowJoint;
JOINTStruct* FOREARM = &ForearmJoint;
JOINTStruct* INCOMING = &INCOMINGJointCommand;

void jointAssignment(){
    // assign base values
    BASE -> STEP_PIN = BASE_STEP_PIN;
    BASE -> DIR_PIN = BASE_DIR_PIN;
    BASE -> ENA_PIN = BASE_ENA_PIN;
    BASE -> STEPS = 0;
    BASE -> SPEED = 0;
    BASE -> ANGLE_TRUE;
    BASE -> ANGLE_IDEAL;
    BASE -> HOMESTATUS = false;

    //assign shoulder values
    SHOULDER -> STEP_PIN = SHOULDER_STEP_PIN;
    SHOULDER -> DIR_PIN = SHOULDER_DIR_PIN;
    SHOULDER -> ENA_PIN = SHOULDER_ENA_PIN;
    SHOULDER -> STEPS = 0;
    SHOULDER -> SPEED = 0;
    SHOULDER -> ANGLE_TRUE;
    SHOULDER -> ANGLE_IDEAL;
    SHOULDER -> HOMESTATUS = false;

    ELBOW -> STEP_PIN = ELBOW_STEP_PIN; 
    ELBOW -> DIR_PIN = ELBOW_DIR_PIN; 
    ELBOW -> ENA_PIN = ELBOW_ENA_PIN; 
    ELBOW -> STEPS = 0; 
    ELBOW -> SPEED = 0; 
    ELBOW -> ANGLE_TRUE; 
    ELBOW -> ANGLE_IDEAL; 
    ELBOW -> HOMESTATUS = false;

    FOREARM -> STEP_PIN = FOREARM_STEP_PIN; 
    FOREARM -> DIR_PIN = FOREARM_DIR_PIN; 
    FOREARM -> ENA_PIN = FOREARM_ENA_PIN; 
    FOREARM -> STEPS = 0; 
    FOREARM -> SPEED = 0; 
    FOREARM -> ANGLE_TRUE; 
    FOREARM -> ANGLE_IDEAL; 
    FOREARM -> HOMESTATUS = false;
    
}

void motorDriver(JOINTStruct* JOINT) {  
    // this function will live in the main loop and manipulate motors on a per need basis
    // get STEP COUNT, DIRECTION, and SPEED, and last step
    unsigned long CurrentTime = micros();
    int DIR = JOINT -> DIR;
    int SPEED = JOINT -> SPEED;
    unsigned long STEPCOUNT = JOINT -> STEPS;
    uint8_t STEP_PIN = JOINT->STEP_PIN;
    float ratio = JOINT->STEP_FACTOR;
    unsigned long LAST_STEP = JOINT->LASTSTEP;
    unsigned long STEPS = JOINT->STEPS;


    // speed should be given in 0-10 settings but what dimensions???
    // use this to calculate interval 0-99 rpm?

    long INTERVAL = 60/SPEED*STEPS;

    if(STEPCOUNT > 0){
        if(CurrentTime - LAST_STEP > INTERVAL) {
            //do the step ting, i guess toggle the pin state of the step pin
            int pinState = digitalRead(STEP_PIN);
            digitalWrite(STEP_PIN, pinState);
            STEPCOUNT -= 1;

            // but how do we handle the joint angles?
            // calculate it every time the joint steps but do state management only when a command enters
            // lets assume 6400 steps per revolution, use the 32 division factor

            float Angle = JOINT->ANGLE_TRUE;
            if (DIR == HIGH) {
                Angle -= ratio; // Decrease Angle by the ratio
            } else if (DIR == LOW) {
                Angle += ratio; // Increase Angle by the ratio
            }
            Serial.println(Angle); // Print the Angle value
        }
    }
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

void stateControls(JOINTStruct* JOINT){
    // this code will guess the approx angle of the joint
    // check if each joint is up to date
    // only call this code when new state is requested
    // this will compare ideal with actual state and apply the speed settings
    // this will also apply the direction
    
    float Ideal_Angle = JOINT->ANGLE_IDEAL;
    float Actual_Angle = JOINT->ANGLE_TRUE;
    float Ratio = JOINT->STEP_FACTOR;
    if (Ideal_Angle == Actual_Angle) {
        return;
    }
    else {
        unsigned long steps = (Ideal_Angle-Actual_Angle)*Ratio;
        JOINT->STEPS = steps;
        Serial.println(steps);

        if(Ideal_Angle>Actual_Angle){
            JOINT->DIR = 1;
        }
        if(Ideal_Angle<Actual_Angle){
            JOINT->DIR = 0;
        }
    }
    
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

char *inputString;


void USART_ReceiveString(char* buffer, uint8_t max_length) {
    uint8_t index = 0;
    char received_char;

    // Read characters until newline or buffer is full
    while (index < max_length - 1) {
        received_char = USART_Receive();

        // Check for newline (end of string)
        if (received_char == '\n' || received_char == '\r') {
            break;
        }
        // Store the received character
        buffer[index++] = received_char;
    }
    // Null-terminate the string
    buffer[index] = '\0';
}

int ExtractIntegerFromSubstring(const char* str, uint8_t start, uint8_t end) {
    char substring[10]; // Buffer to hold the substring
    uint8_t length = end - start + 1; // Length of the substring

    // Copy the substring from the original string
    strncpy(substring, str + start, length);
    substring[length] = '\0'; // Null-terminate the substring

    // Convert the substring to an integer
    return atoi(substring);
}


void initialize() {

    USART_Init();


    // DDRB |= (1 << DDB0);
    // DDRB |= (1 << DDB7);
    // DDRG |= (1 << DDG0);
    // DDRC |= (1 << DDC2);
    // DDRC |= (1 << DDC4);
    // DDRC |= (1 << DDC5);
    // DDRL |= (1 << DDL2);
    pinMode(LASER_PIN, OUTPUT);
    pinMode(LASER_TRIGGER, INPUT);
    
} 

int inputHandler(char *inputString) {
        if (read_uart_string(inputString)) {
        // Grip/Release commands to engage hand
        // command types
        // J1P Joint 1 Position
        // J1090005 joint 1 to 90.0 speed of 05rpm
        // J2180010 Joint 2 to 180.0 speed of 10rpm
        // J2180099 Joint 2 to 180.0 speed of 99 rpm
        // clean this code later, make it work first. 

        // this only sets ideal state and speed

        // we can calculate angles in post for 0 degrees and etc. 

        if (inputString[0] == 'T') {
            // generally this command is just going to be for tool head control
            // commands taken in will be pretty dependent on the kind of tool head mounted though
            // example commands for the claw: 
            uart_transmit(second_test_message);

            char character = inputString[0];
            char* character_ptr = &inputString[2]; 
            
        }

        if (inputString[0] == 'J') {
            send_hello_world();
            JOINTStruct INCOMINGJointCommand;
            // i say we say fuck it and just construct here. 
            int JOINTID = inputString[1];
            // angle contructor 
            int Angle_Hundred = 0;//inputString[2];
            char Angle_Ten = inputString[3];
            int Angle_One = 1;//inputString[4];
            int Angle_Tenth = 0;//inputString[5];

            int ANGLE = (Angle_Hundred*100)+(Angle_Ten*10)+(Angle_One);

            // Convert the float to a character array
            char angleTest[10];

            char* Angle_Tenx = &Angle_Ten;

            itoa(ANGLE, angleTest, 10);


            // start passing data from input string to joint 
            
            //INCOMING -> SPEED = Speed;

            // gather the speed param 

        }
    }
}

int inputHandler0(){
    //Serial.println("shalom stanley");
    if (Serial.available() > 0) {   // check if data is available
    char inChar = Serial.read();  // read the incoming byte


    if (inChar != '\n') {         // if it's not a newline character
      receivedString += inChar;      // add the incoming byte to the string
    } else {
      Serial.println(receivedString);

      if (receivedString == "V1H") {
        // turn on Valve 1
        Serial.println("moshi moshi");

        }

        receivedString = "";


        }
    }

    return(0);
}


int main(void) {

    char receivedString[MAX_STRING_LENGTH];
    initialize();

    // main loop
    while(true) {
        // Com Handling Code
        inputHandler(inputString);
        // motor handling code
    }
    return(0);
}

// you need the structures to serve as a state bank.
// if current and target is not the same keep rotating until you hit the target
// use a steps to angle calculator to figure out where your true angle is.