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

const uint8_t fanStepValues[7] = {63, 95, 127, 159, 191, 223, 255};
uint8_t currentFanStep = 0;

volatile uint8_t fanSpeedPwmValue = 0;

volatile uint64_t timerValue = 0; // time in uSeconds

ISR (TIMER0_OVF_vect) {
    // On 16.000.000 / 1024 / 256 = 61,03515635 = 16384uS every interrupt cycle
    timerValue += 16384;
}

void setupValDisplay() {
    // Setup output ports
    DDRD |= (1<<DDD5);  // SER Port
    DDRD |= (1<<DDD6);  // SRCLK Port
    DDRD |= (1<<DDD7);  // RCLK Port
}

void initOverflowInterruptCounter() {
    // Set Timer/Counter0 prescaler to clock/1024.
    TCCR0 |= (1 << CS02) | (0 << CS01) | (1 << CS00);

    // Enable overflow interrupt for Timer/Counter0
    TIMSK |= (1 << TOIE0);
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

    OCR1A=fanSpeedPwmValue;
}

void initADC() {
    ADMUX |=
        // left shift result
        (1 << ADLAR) |

        // AREF, Internal V ref turned off
        (0 << REFS1) |
        (1 << REFS0) |

        // use ADC0 for input (PC0), MUX3..0 = 0000
        (0 << MUX3)  |
        (0 << MUX2)  |
        (0 << MUX1)  |
        (0 << MUX0);

    ADCSRA =
        // Enable ADC
        (1 << ADEN)  |

        // no prescaler is required, we use only 8bit resolution
        (0 << ADPS2) |
        (0 << ADPS1) |
        (0 << ADPS0);
}

void setup() {
    setupPWM();
    initADC();
    initOverflowInterruptCounter();
    setupValDisplay();
    sei();
}

void setFanStep(uint8_t potiVal) {
    if (potiVal > fanStepValues[currentFanStep]) {
        ++currentFanStep;
        fanSpeedPwmValue = fanStepValues[currentFanStep];

    } else if (currentFanStep > 0 && potiVal <= fanStepValues[currentFanStep-1]) {
        --currentFanStep;
        fanSpeedPwmValue = fanStepValues[currentFanStep];
    } else {
        fanSpeedPwmValue = fanStepValues[currentFanStep];
    }
}

int main(void) {
    setup();

    pushByteAndLatch(0x00000000);

    while(1) {
        ADCSRA |= (1 << ADSC);         // start ADC measurement
        while (ADCSRA & (1 << ADSC));  // wait till conversion complete
        setFanStep(ADCH); // read value from ADC

        // If timer is less or equals one hour then let the fan run
        if (timerValue <= 3600000000ULL) {
            OCR1A=fanSpeedPwmValue;
        }

        // If the tmer is more then one hour then the fan should be off
        else {
            OCR1A=0;
        }

        // Reset timer if the value is more then 24 hours
        if (timerValue > 86400000000ULL) {
            timerValue = 0;
        }

        pushByteAndLatch(timerValue / 60000000);
    }

    return 0;
}
