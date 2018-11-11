# ATMega8_pwm_based_faan_controller_with_timer
 PWM Based fan controller with features:
  - Activate a fan for defined time with defined time delay.
  - Activate a fan only on a day (light sensor)
  - Controle a speed of a fan with poti
  - Display current state vi LED
      - Off: Night mode, no counter is running
      - Slow blinking: Timer is running, more then 1 hour is left to start the fan
      - Fast blinking: Timer is running, less then 1 hour before starting the fan
      - On: Fan is running or was active today. Timer will reset after a day/night cycle
