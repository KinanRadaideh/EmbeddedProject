/* ==========================================================
   PIC16F877A • 8 MHz • mikroC PRO
   RB0 active-LOW IR  ?  RC2 servo (0 ° / 180 °) + RC6 LED
   ========================================================== */

#define SERVO_MIN    1000u        // 0.50 ms  = 1000 × 0.5 µs  (0°)
#define SERVO_MAX    5000u        // 2.50 ms  = 5000 × 0.5 µs  (180°)
#define SERVO_FRAME  40000u       // 20 ms frame

/* --- SFR shortcuts --- */
#define TMR1H   ((volatile unsigned char)0x0F)
#define TMR1L   ((volatile unsigned char)0x0E)
#define T1CON   ((volatile unsigned char)0x10)
#define CCPR1H  ((volatile unsigned char)0x16)
#define CCPR1L  ((volatile unsigned char)0x15)
#define CCP1CON ((volatile unsigned char)0x17)
#define PIR1    ((volatile unsigned char)0x0C)
#define PIE1    ((volatile unsigned char)0x8C)
#define INTCON  ((volatile unsigned char)0x0B)
#define OPTION_REG ((volatile unsigned char)0x81)

/* --- Bit masks --- */
#define CCP1IF 0x04
#define CCP1IE 0x04
#define PEIE   0x40
#define GIE    0x80

volatile unsigned int  pulseTicks = SERVO_MIN;
volatile unsigned char highPhase  = 1;

/* --- CCP1 compare ISR (50 Hz PWM) --- */
void interrupt() {
    if (PIR1 & CCP1IF) {
        if (highPhase) {
            CCPR1H  = (unsigned char)(pulseTicks >> 8);
            CCPR1L  = (unsigned char)pulseTicks;
            CCP1CON = 0x09;        // clear RC2 on match
            highPhase = 0;
        } else {
            unsigned int rest = SERVO_FRAME - pulseTicks;
            CCPR1H  = (unsigned char)(rest >> 8);
            CCPR1L  = (unsigned char)rest;
            CCP1CON = 0x08;        // set RC2 on match
            highPhase = 1;
        }
        TMR1H = 0;  TMR1L = 0;
        PIR1 &= ~CCP1IF;
    }
}

/* ------------ MAIN ------------ */
void main(void)
{
    OPTION_REG &= 0x7F;      // enable PORTB pull-ups (RBPU = 0)

    TRISB = 0x01;            // RB0 input
    TRISC &= 0x00;
    PORTC &= 0x00;

    /* CCP1 / Timer-1 setup */
    TMR1H = 0;  TMR1L = 0;
    CCP1CON = 0x08;                      // first compare sets RC2
    CCPR1H  = (unsigned char)(pulseTicks >> 8);
    CCPR1L  = (unsigned char)pulseTicks;
    T1CON   = 0x01;                      // Timer-1 ON, 1:1 prescale

    PIE1   |= CCP1IE;
    INTCON |= PEIE | GIE;

    while(1) {
        if (!(PORTB & 0x02) {       // RB1 LOW ? object detected
            pulseTicks = SERVO_MAX;      // 2.50 ms  (180°)
        } else {                         // RB0 HIGH ? no object
            pulseTicks = SERVO_MIN;      // 0.50 ms  (0°)
        }
        /* PWM width updated automatically by ISR next frame */
    }
}
