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

    // If the value '128' is reached, the PWM signal will set to LOW
    OCR1A=128; // 50% duty cycle
}

void setup() {
    pwmSetup();
}

int main(void) {
    setup();

    while(1) {
    }

    return 0;
}
