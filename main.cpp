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

#define F_CPU 16000000UL

#define MIN_FAN_SPEED_VALUE 60
#define FAN_SPEED_FACTOR (255 - MIN_FAN_SPEED_VALUE) / 255

#define FAN_ACTIVE_TIMER_VALUE_LESS_THEN 3600000000ULL

#define TIMER_VALUE_RESET 86400000000ULL

#define STATUS_LED_MODE0_DELAY_VALUE 0ULL
#define STATUS_LED_MODE1_DELAY_VALUE 5000000ULL
#define STATUS_LED_MODE2_DELAY_VALUE 500000ULL

#include <avr/io.h>
#include <avr/interrupt.h>

bool isFanOn = false;
uint8_t fanSpeedPwmValue = 0;

volatile uint64_t timerValue = 0; // time in uSeconds

bool isStatusLedOn = false;
uint64_t statusLedDelayValue = 0;
uint64_t statusLedTimerValue = 0;

ISR (TIMER0_OVF_vect) {
    // On 16.000.000 / 1024 / 256 = 61,03515635 = 16384uS every interrupt cycle
    timerValue += 16384;
}

void initOverflowInterruptCounter() {
    // Set Timer/Counter0 prescaler to clock/1024.
    TCCR0 |= (1 << CS02) | (0 << CS01) | (1 << CS00);

    // Enable overflow interrupt for Timer/Counter0
    TIMSK |= (1 << TOIE0);
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

    DDRB |= (1 << DDB0); // Setup the Output fan status LED

    sei();
}

uint8_t computeFanSpeedValue(uint8_t potiVal) {
    return potiVal * FAN_SPEED_FACTOR + MIN_FAN_SPEED_VALUE;
}

void setStatusLedDelay() {
    const uint64_t THREE_HOURS_VALUE = 10800000000ULL;

    // Fan is active
    if (isFanOn) {
        statusLedDelayValue = STATUS_LED_MODE0_DELAY_VALUE;
    }

    // Less then 3 hours to fan start
    else if ((timerValue + THREE_HOURS_VALUE) >= TIMER_VALUE_RESET) {
        statusLedDelayValue = STATUS_LED_MODE2_DELAY_VALUE;
    }

    // Fan is off
    else {
        statusLedDelayValue = STATUS_LED_MODE1_DELAY_VALUE;
    }
}

void setFanStatus() {
    isFanOn = timerValue <= FAN_ACTIVE_TIMER_VALUE_LESS_THEN;
}

int main(void) {
    setup();

    uint64_t timerNextStopValue = 0;
    bool timerTick = false;

    while(1) {
        if (timerNextStopValue < timerValue) {
            timerTick = true;
            timerNextStopValue += 250000;
            statusLedTimerValue += 250000;
        }

        if (isStatusLedOn) {
            PORTB |= (1<<PB0);
        } else {
            PORTB &= ~(1<<PB0);
        }

        if (timerTick) {
            ADCSRA |= (1 << ADSC);         // start ADC measurement
            while (ADCSRA & (1 << ADSC));  // wait till conversion complete
            uint8_t potiVal = ADCH;        // read value from ADC
            fanSpeedPwmValue = computeFanSpeedValue(potiVal);

            setFanStatus();
            setStatusLedDelay();

            isFanOn ? OCR1A=fanSpeedPwmValue : OCR1A=0;

            if (statusLedDelayValue == STATUS_LED_MODE0_DELAY_VALUE) {
                isStatusLedOn = true;

            } else if (statusLedTimerValue >= statusLedDelayValue) {
                isStatusLedOn = !isStatusLedOn;
                statusLedTimerValue = 0;
            }

            // Reset timer if the value is more then 24 hours
            if (timerValue > TIMER_VALUE_RESET) {
                timerValue = 0;
                timerNextStopValue = 0;
                timerTick = false;
            }
        }

        timerTick = false;
    }

    return 0;
}
