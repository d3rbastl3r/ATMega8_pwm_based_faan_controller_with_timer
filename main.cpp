/**
 * PWM Based fan controller with features:
 * - Activate a fan for defined time with defined time delay.
 * - Activate a fan only on a day (light sensor)
 * - Controle a speed of a fan with poti
 * - Display current state vi LED
 *     - Off: Night mode, no counter is running
 *     - Slow blinking: Timer is running, more then 1 hour is left to start the fan
 *     - Fast blinking: Timer is running, less then 1 hour before starting the fan
 *     - On: Fan is running or was active today. Timer will reset after a day/night cycle
 *
 * PWM Output: PB1 (OC1A)
 *
 * @author Igor Martens
 * @since 11.11.2018
 */

#define F_CPU 16000000

#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t pwmValue = 128;

void setupValDisplay() {
    // Setup output ports
    DDRD |= (1<<DDD5);  // SER Port
    DDRD |= (1<<DDD6);  // SRCLK Port
    DDRD |= (1<<DDD7);  // RCLK Port
}

/**
 * Push the given byte to the register and finally execute latch.
 * The left bit will be pushed first.
 */
void pushByteAndLatch(uint8_t byte) {
    for (uint8_t i=0; i<8; ++i) {
        (byte & 128) ? PORTD |= (1 << PD5) : PORTD &= ~(1 << PD5);
        PORTD |= (1 << PD6);
        PORTD &= ~(1 << PD6);
        byte = byte << 1;
    }

    PORTD |= (1 << PD7);
    PORTD &= ~(1 << PD7);
}

void setupPWM() {
    DDRB |= (1 << DDB1); // Setup the Output for PWM (OC1A)


    TCCR1B |=
        // Set Timer/Counter1 prescaler to clock/64.
        // At 16 MHz this is 250 kHz.
        (0 << CS12) |
        (1 << CS11) |
        (1 << CS10) |

        // Fast PWM, 8-bit
        (0 << WGM13) |
        (1 << WGM12);


    TCCR1A |=
        // Fast PWM, 8-bit
        (0 << WGM11) |
        (1 << WGM10) |

        // Clear OC1A/OC1B on Compare Match, set OC1A/OC1B at BOTTOM, (non-inverting mode)
        (1 << COM1A1) |
        (0 << COM1A0);

    OCR1A=pwmValue;
}

void setup() {
    setupPWM();
    setupValDisplay();
}

int main(void) {
    setup();

    while(1) {
        pushByteAndLatch(pwmValue);
    }

    return 0;
}
