// Assumes mikroC for PIC syntax (Delay_ms, Delay_us, etc.)
// PIC16F877A @ 8 MHz
// RC0 = IR input
// RC2 = CCP1 / PWM output to servo

unsigned int angle_us = 1500;  // start centered (1.5 ms pulse)
const unsigned int rate = 50;  // change in µs per cycle

//------------------------------------------------------------------------------
// Send a 1–2 ms pulse via hardware PWM on CCP1
void set_servo_position1(int degrees) {
    // Map -90..+90° to 500..2500 µs
    int pulse_width = (degrees + 90) * 10 + 500;
    // pulse_width now ranges 500..2500

    // CCP1 is PWM mode, so:
    // CCPR1L = pulse_width/4
    // CCP1CON<5:4> = (pulse_width%4)
    CCPR1L = pulse_width >> 2;
    CCP1CON = (CCP1CON & 0xCF) | ((pulse_width & 0x03) << 4);

    Delay_ms(50);  // let servo reach position
}

//------------------------------------------------------------------------------
// Map your angle_us (1000–2000) back to -90..+90°
int us_to_degrees(unsigned int us) {
    // 1000 -> -90°, 1500 -> 0°, 2000 -> +90°
    // (us - 1500) / (500/90) = (us - 1500) * 90/500
    return (int)(((us - 1500) * 90) / 500);
}

//------------------------------------------------------------------------------
// Configure CCP1 (RC2) for PWM, TMR2 for 1 kHz base ? 20 ms period
void pwm1_init() {
    // Set PR2 so that PWM period = (PR2 + 1) × 4 × Tosc × TMR2_prescale
    // We want 20 ms = 20000 µs. With Fosc=8 MHz, Tosc=0.125 µs, prescale=16:
    // PR2 = (20000 / (4 × 0.125 × 16)) - 1 ˜ 249
    PR2 = 249;
    T2CON = 0b00000110;  // TMR2 on, prescale 16

    // Set RC2/CCP1 as output
    TRISC = TRISC | 0X04
    // Configure CCP1 in PWM mode (CCP1CON<3:0> = 0b1100)
    CCP1CON = 0b00001100;
    // The top two bits of CCP1CON (5:4) will hold the LSBs of the duty cycle
}

int open_angle   =  35;   // +35° = half of your 70° swing when IR ON
int closed_angle = -35;   // -35° = half of your 70° swing when IR OFF
int step_delay   =  10;   // ms pause per 1° step (10ms ? ~0.7s full 70°)

void main() {
    int current = 0;      // start centered at 0°
    int target  = 0;

    // ===== Configure TRISC =====
    STATUS |= 0x20;       // bank 1
    TRISC = TRISC | 0X01 ; // bit 0 is input, bit 2 is output
    STATUS &= 0xDF;       // bank 0

    pwm1_init();          // set up CCP1 PWM on RC2
    set_servo_position1(current);

    while (1) {
        // choose target based on IR
        if (PORTC & 0X01)
            target = open_angle;
        else
            target = closed_angle;

        // step current toward target
        if (current < target) {
            current++;
            set_servo_position1(current);
            Delay_ms(10);
        }
        else if (current > target) {
            current--;
            set_servo_position1(current);
            Delay_ms(10);
        }
        // if current==target, do nothing until IR changes
    }
}
