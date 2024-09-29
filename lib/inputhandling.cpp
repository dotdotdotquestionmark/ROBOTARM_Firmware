#include <avr/io.h>
#include <util/delay.h>

void USART_Init(unsigned int ubrr) {
    // Set baud rate
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;

    // Enable receiver and transmitter
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    // Set frame format: 8 data, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Function to transmit a byte via UART
void USART_Transmit(unsigned char data) {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)));
    // Put data into buffer, sends the data
    UDR0 = data;
}

int main(void) {
    // Set the baud rate (for example, 9600 bps at 16 MHz)
    USART_Init(103);

    // Set LED pin as output
    DDRB |= (1 << DDB7);

    while (1) {
        // Wait for data to be received
        while (!(UCSR0A & (1 << RXC0)));

        // Read the received data
        char receivedData = UDR0;

        // Check the received data and adjust blink speed accordingly
        if (receivedData == '1') {
            _delay_ms(100);
        } else if (receivedData == '2') {
            _delay_ms(500);
        } else if (receivedData == '3') {
            _delay_ms(1000);
        }

        // Toggle the state of the LED
        PORTB ^= (1 << PORTB7);
    }

    return 0;
}