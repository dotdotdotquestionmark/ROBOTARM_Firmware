#include <avr/io.h>
#include "Arduino.h"
#include <util/delay.h>
#include "motordriver.h"
#include "TMCStepper.h"
#include "string.h"

using namespace std;

#define F_CPU 16000000UL // Adjust clock frequency as needed
#define MAX_STRING_LENGTH 64 // Adjust this value as needed

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


struct JOINTStruct {
    uint8_t STEP_PIN;
    uint8_t DIR_PIN;
    uint8_t ENA_PIN;
    int DIR;
    long STEPS; // amount of moves
    long SPEED; // interval
    bool STATE; // active or not
    int ANGLE_TRUE;
    int ANGLE_IDEAL;
    bool HOMESTATUS;
    unsigned long LASTSTEP;
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

    // speed should be given in 0-10 settings but what dimensions???
    // use this to calculate interval 0-99 rpm?

    


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

void stateControls(){
    // this code will guess the approx angle of the joint
    // check if each joint is up to date
    // only call this code when new state is requested
    // this will compare ideal with actual state and apply the speed settings
    // this will also apply the direction

    
}

void uart0_init(uint32_t baud) {
    // Calculate UBRR value based on baud rate and clock frequency
    uint16_t ubrr = (F_CPU / (16UL * baud)) - 1;
    // Set baud rate
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    // Configure data format (8N1)
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    // Enable transmitter and receiver
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
}

void uart0_transmit(unsigned char data) {
  // Wait for empty transmit buffer
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = data;
}

char uart0_receive() {
    // Wait for data to be received
    while (!(UCSR0A & (1 << RXC0)));
    // Read and return data
    return UDR0;
}

const char *hello_world_message = "Hello World\r\n";
const char *second_test_message = "shalom stan\r\n";
const char *third_test_message = "test3 successful\r\n";

char*inputstring = "";



char read_uart_string(char *buffer) {
  int index = 0;
  unsigned char received_char;

  // Read characters until newline or buffer full
  while ((received_char = uart0_receive()) != '\n' && index < MAX_STRING_LENGTH - 1) {
    buffer[index++] = received_char;
  }

  // Add null terminator if string is not empty
  if (index > 0) {
    buffer[index] = '\0';
    return 1; // Indicate successful string reception
  } else {
    return 0; // Indicate no characters received or buffer full
  }
}

void send_hello_world(void) {
  const char *message_ptr = hello_world_message;
  while (*message_ptr) {
    uart0_transmit(*message_ptr++);
  }
}

void setup() {

    uart0_init(9600);
    // set register input output mode 
    // setting data directions
    DDRB |= (1 << DDB0);
    DDRB |= (1 << DDB7);
    DDRG |= (1 << DDG0);
    DDRC |= (1 << DDC2);
    DDRC |= (1 << DDC4);
    DDRC |= (1 << DDC5);
    DDRL |= (1 << DDL2);
    pinMode(LASER_PIN, OUTPUT);
    pinMode(LASER_TRIGGER, INPUT);

} 

int inputHandler(char *inputString) {
        if (read_uart_string(inputString)) {
        // Grip/Release commands to engage hand
        
        // Check if the first character is 'A'
        // command structure example for axis1, 
        // A1M360H100

        // command types
        // J1P Joint 1 Position
        // J1090005 joint 1 to 90.0 speed of 05rpm
        // J2180010 Joint 2 to 180.0 speed of 10rpm
        // J2180099 Joint 2 to 180.0 speed of 99 rpm

        // clean this code later, make it work first. 

        // this only sets ideal state and speed

        if (inputString[0] == 'J') {
            JOINTStruct INCOMINGJointCommand;

            send_hello_world(); 
            if (inputString[1] == '1') { 
                send_hello_world();
            }

            char Speed = inputString[6];


            // start passing data from input string to joint 
            INCOMING -> SPEED = Speed;

            // gather the speed param 


        }

        
    }
}

int main(void) {
    char receivedString[MAX_STRING_LENGTH];
    setup();
    // main loop
    while(1) {
        inputHandler(receivedString);
        // motor handling code
    }
    return 0;
}

// you need the structures to serve as a state bank.
// if current and target is not the same keep rotating until you hit the target
// use a steps to angle calculator to figure out where your true angle is. 
