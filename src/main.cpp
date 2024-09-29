#include <avr/io.h>
#include <util/delay.h>
#include "motordriver.h"

#include "Arduino.h"

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

// blue wire outlet is yellow, redwire outlet is purple

// Pin Definitions
// Stepper Motor 1: Base Rotation --> PIN22
// Stepper Motor 2: Base Hinge --> PIN23
// Stepper Motor 3: Elbow Driver --> PIN24
// Stepper Motor 4: Forearm Rotator --> PIN25 

// Function to initialize UART with a specific baud rate

//#define CONTROL_BIT 1 // Corresponds to bit 1 of Port B


struct JOINTS {
    uint8_t STEP_PIN;
    uint8_t DIR_PIN;
    uint8_t ENA_PIN;
    int DIR;
    long STEPS; // amount of moves
    long SPEED; // interval
    bool STATE; // active or not
};

JOINTS baseJOINT;
JOINTS shoulderJOINT;
JOINTS elbowJOINT;
JOINTS forearmJOINT;


JOINTS*BASE = &baseJOINT;
JOINTS*SHOUDER = &shoulderJOINT;
JOINTS*ELBOW = &elbowJOINT;
JOINTS*FOREARM =&forearmJOINT;


void motorDriver(JOINTS *motor) {
    uint8_t stepPin = motor->STEP_PIN;
    uint8_t dirPin = motor->DIR_PIN;
    uint8_t enaPin = motor->ENA_PIN;
}

void loop() {
    // Set PB7 (Pin 13 on Arduino) as an output

    while(1)
    {
        PORTB |= (1 << TEST_PIN);
        PORTB |= (1 << BASE_STEP_PIN);
        PORTC |= (1 << ELBOW_STEP_PIN);
        PORTG |= (1 << SHOULDER_STEP_PIN);
        PORTL |= (1 << FOREARM_STEP_PIN);
        // Wait for some time
        _delay_us(1000);
        PORTB &= ~(1 << TEST_PIN);
        PORTB &= ~(1 << BASE_STEP_PIN);
        PORTC &= ~(1 << ELBOW_STEP_PIN);
        PORTG &= ~(1 << SHOULDER_STEP_PIN);
        PORTL &= ~(1 << FOREARM_STEP_PIN);
        _delay_us(1000);
    }
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

void send_hello_world(void) {
  const char *message_ptr = hello_world_message;
  while (*message_ptr) {
    uart0_transmit(*message_ptr++);
  }
}

void send_second(void) {
    const char *second_message = second_test_message;
    while(*second_message){
        uart0_transmit(*second_message++);
    }
}

void send_third(void){
    const char *third_message = third_test_message;
    while(*third_message){
        uart0_transmit(*third_message++);
    }
}

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
} 

int inputHandler(char *inputString) {
        if (read_uart_string(inputString)) {
        // Grip/Release commands to engage hand
        
        // Check if the first character is 'A'
        // command structure example for axis1, 
        // A1M360H100
        if (inputString[0] == 'A') {
            send_hello_world(); 
            if (inputString[1] == 'H') {
                send_second();
            }
        }
        }
    }

int main(void) {
    char receivedString[MAX_STRING_LENGTH];
    setup();
    // main loop
    while(1) {
        inputHandler(receivedString);
    }
    return 0;
}
