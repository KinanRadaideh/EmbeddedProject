/* ==========================================================
   PIC16F877A • 8 MHz crystal • mikroC PRO

   • RB1  – IR sensor (active-LOW)
   • RC7  – LED indicator
   • RA0  – Potentiometer (speed set)
   • RC1  – PWM (EN) to H-bridge
   • RC4  – IN3  (set HIGH for forward)
   • RC5  – IN4  (set LOW  for forward)
   • Pump red  wire ? OUT3
   • Pump black wire ? OUT4
   ========================================================== */

#define PWM_FREQ_HZ  500u
#define PR2_VALUE    249u          /* 8 MHz, 1:16 prescale ? 500 Hz */

/* ---------- simple helpers ---------------------------------- */
unsigned char pot_to_pwm(unsigned int v) {   /* 0-1023 ? 0-249   */
    return (unsigned char)((v * 249UL) / 1023UL);
}

unsigned int adc_read_ra0(void) {
    ADCON0 |= 0x04;                /* start conversion          */
    while (ADCON0 & 0x04);         /* wait until done           */
    return ((ADRESH << 8) | ADRESL);
}

/* ---------- INITIALISATION ---------------------------------- */
void init_io_adc_pwm(void) {
    /* RB pull-ups ON */
    OPTION_REG &= 0x7F;

    /* TRIS settings */
    TRISB = 0x02;                  /* RB1 input                 */
    TRISC &= 0x00;                 /* RC7,RC5,RC4,RC1 outputs   */
    TRISA = 0x01;                  /* RA0 analogue input        */

    /* ADC: only AN0 used */
    ADCON1 = 0xCE;                 /* AN0 analogue, others dig. */
    ADCON0 = 0x41;                 /* ADC ON, ch 0, Fosc/16     */

    /* PWM on RC1 / CCP2 */
    T2CON  = 0x07;                 /* Timer2 ON, 1:16 prescale  */
    PR2    = PR2_VALUE;            /* sets PWM frequency        */
    CCP2CON = 0x0C;                /* CCP2 PWM mode             */
    CCPR2L  = 0x00;                /* duty starts at 0          */

    /* H-bridge direction: forward (IN3=1, IN4=0) */
    RC4_bit = 1;                   /* IN3 HIGH                  */
    RC5_bit = 0;                   /* IN4 LOW                   */
}

/* ------------------------------ MAIN ------------------------ */
void main(void) {
    unsigned int  pot;
    unsigned char duty;

    init_io_adc_pwm();

    while (1) {
        pot  = adc_read_ra0();           /* 0–1023                  */
        duty = pot_to_pwm(pot);          /* 0–249   (<=PR2)         */

        if (!(PORTB & 0x01) {              /* IR triggered (LOW)      */
            CCPR2L   = duty;             /* enable PWM, set speed   */
        } else {                         /* no object               */
            CCPR2L   = 0;                /* PWM duty 0 ? pump off   */
        }
    }
}
