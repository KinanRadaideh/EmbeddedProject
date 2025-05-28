/* ==========================================================
   PIC16F877A • 8 MHz crystal • mikroC PRO
   Pump subsystem  +  water-level distance check
   ----------------------------------------------------------
   • RB1 – IR sensor (active-LOW, hand present)
   • RB2 – Level LED  (ON when distance > 100 cm)
   • RC7 – Status LED (your original indicator)
   • RA0 – Potentiometer (pump speed)
   • RC1 – PWM (EN) to H-bridge
   • RC4 – IN3  (HIGH  → forward)
   • RC5 – IN4  (LOW   → forward)
   • RC2 – HC-SR04 TRIG
   • RC3 – HC-SR04 ECHO
   ========================================================== */

#define PWM_FREQ_HZ   500u
#define PR2_VALUE     249u          /* 8 MHz, 1:16 prescale → 500 Hz   */

/* ---------- simple helpers -------------------------------- */
unsigned char pot_to_pwm(unsigned int v) {   /* 0-1023 → 0-249   */
    return (unsigned char)((v * 249UL) / 1023UL);
}

unsigned int adc_read_ra0(void) {
    ADCON0 |= 0x04;                 /* start conversion            */
    while (ADCON0 & 0x04);          /* wait until done             */
    return ((ADRESH << 8) | ADRESL);
}

/* ---------- HC-SR04 distance (cm)  ------------------------ */

void mydelay() {
    unsigned int i = 0;
    unsigned int j = 0;
    for (; i < 0xFFFF; i++) {
        for (j = 0; j < 0xFF; j++) {}  // reduced inner loop for practical delay
    }
}
void delay10us(void){
    char i;
    for(i = 0; i < 10; i++){
        asm NOP;
        asm NOP;
    }
}
int dist_cm(void) {
    int d;

    /* clear Timer1 */
    TMR1H = 0;
    TMR1L = 0;

    /* 10 µs trigger on RC2 (bit 2) */
    PORTC |=  0b00000100;
    delay10us();
    PORTC &= ~0b00000100;

    /* wait for echo HIGH on RC3 (bit 3) */
    while (!(PORTC & 0b00001000));
    T1CON |=  0x01;                 /* start Timer1               */
    while (  (PORTC & 0b00001000));
    T1CON &= ~0x01;                 /* stop Timer1                */

    d  = (TMR1H << 8) | TMR1L;      /* counts (µs @ Fosc/4)       */
    d  = d / 58;                    /* ~58 µs per cm              */
    return d;
}




/* ------------------------------ MAIN ---------------------- */
void main(void) {
    unsigned int pot;
    unsigned char duty;
    int distance;
 OPTION_REG &= 0x7F;

    /* TRIS settings */
    TRISB = 0x02;                   /* RB1 input, RB2 output            */
    TRISC = 0x08;                   /* RC3 input, others outputs        */
    TRISA = 0x01;                   /* RA0 analogue input               */

    /* ADC: only AN0 used */
    ADCON1 = 0xCE;                  /* AN0 analogue, others digital     */
    ADCON0 = 0x41;                  /* ADC ON, ch0, Fosc/16             */

    /* PWM on RC1 / CCP2 */
    T2CON   = 0x07;                 /* Timer2 ON, 1:16 prescale         */
    PR2     = PR2_VALUE;
    CCP2CON = 0x0C;                 /* CCP2 PWM mode                    */
    CCPR2L  = 0x00;                 /* duty 0                           */

    /* Timer1 set up for echo timing (internal clk, 1:1 prescale, OFF) */
    T1CON   = 0x00;

    while (1) {
        /* --- pump speed from potentiometer ---------------- */
        pot  = adc_read_ra0();           /* 0-1023                   */
        duty = pot_to_pwm(pot);          /* 0-249  ( ≤ PR2 )         */

        /* --- IR sensor controls the pump ------------------ */
        if ( !(PORTB & 0x02) ) {         /* RB1 LOW → hand present   */
            CCPR2L = duty;               /* run pump at set speed    */
        } else {                         /* no hand                  */
            CCPR2L = 0;                  /* pump off                 */
        }

        /* --- water-level check with ultrasonic ------------- */
        distance = dist_cm();            /* blocking read (≈ 2 ms)   */

        if (distance > 100) {            /* tank low? (>100 cm)      */
            PORTB |=  0b00000100;        /* RB2 HIGH  → LED ON       */
        } else {
            PORTB &= ~0b00000100;        /* RB2 LOW   → LED OFF      */
        }

        /* optional pause to reduce duty-cycle of HC-SR04      */
        mydelay();
    }
}
