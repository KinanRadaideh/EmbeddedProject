/* ==========================================================
   PIC16F877A • 8 MHz crystal • mikroC PRO
   ----------------------------------------------------------
   SUBSYSTEM 1  (trash-can lid)
     • RB0 – IR sensor (active-LOW ? open lid)
     • RC2 – Servo (CCP1, 0°?180°)
     • RC6 – Status LED (optional)

   SUBSYSTEM 2  (water tank)
     • RB1 – IR sensor (active-LOW ? run pump)
     • RB2 – “Tank-low” LED  (ON when level >100 cm)
     • RA0 – Potentiometer (pump speed)
     • RC1 – PWM (CCP2) ? H-bridge EN
     • RC4 – IN3  (HIGH ? forward)
     • RC5 – IN4  (LOW  ? forward)

   Water-level sensing (HC-SR04)
     • *RC5 – TRIG  (moved from RC2)*
     • RC3 – ECHO  (unchanged)

   ----------------------------------------------------------
   Timer / peripheral use
     • CCP1 + Timer-1 ? 50 Hz servo pulses
     • CCP2 + Timer-2 ? 500 Hz pump PWM
     • Timer-1 is borrowed briefly (CCP1 interrupt masked)
       during each HC-SR04 measurement, then restored.
   ========================================================== */

/* -------------------  SERVO CONSTANTS  ------------------- */
#define SERVO_MIN    1000u          /* 0.5 ms × 0.5 µs = 0°  */
#define SERVO_MAX    5000u          /* 2.5 ms                */
#define SERVO_FRAME  40000u         /* 20 ms frame           */

/* -------------------  PUMP CONSTANTS  -------------------- */
#define PWM_FREQ_HZ  500u
#define PR2_VALUE    249u           /* 8 MHz, 1:16 presc.    */

/* -------------------  SFR SHORTCUTS  --------------------- */
#define TMR1H        ((volatile unsigned char)0x0F)
#define TMR1L        ((volatile unsigned char)0x0E)
#define T1CON        ((volatile unsigned char)0x10)
#define CCPR1H       ((volatile unsigned char)0x16)
#define CCPR1L       ((volatile unsigned char)0x15)
#define CCP1CON      ((volatile unsigned char)0x17)
#define PIR1         ((volatile unsigned char)0x0C)
#define PIE1         ((volatile unsigned char)0x8C)
#define INTCON       ((volatile unsigned char)0x0B)
#define OPTION_REG   ((volatile unsigned char)0x81)

/* -------------------  BIT MASKS  ------------------------- */
#define CCP1IF       0x04
#define CCP1IE       0x04
#define PEIE         0x40
#define GIE          0x80

/* -------------------  GLOBALS  --------------------------- */
volatile unsigned int  pulseTicks = SERVO_MIN;   /* current width */
volatile unsigned char highPhase  = 1;           /* ISR state     */

/* --------------  SIMPLE HELPERS (unchanged) -------------- */
unsigned char pot_to_pwm(unsigned int v) {       /* 0-1023 ? 0-249 */
    return (unsigned char)((v * 249UL) / 1023UL);
}

unsigned int adc_read_ra0(void) {
    ADCON0 |= 0x04;                 /* start conversion        */
    while (ADCON0 & 0x04);          /* wait                    */
    return ((ADRESH << 8) | ADRESL);
}

void delay10us(void){              /* ~10 µs by NOPs          */
    char i;
    for(i = 0; i < 10; i++){ asm NOP; asm NOP; }
}

/* blocking “rest” between HC-SR04 pings – keeps the
   ultrasonic duty-cycle low without using Delay_ms() */
void mydelay(void){
    unsigned int i,j;
    for(i=0;i<0x7FFF;i++){ for(j=0;j<3;j++){} }
}

/* --------------  CCP1 COMPARE ISR  (servo) --------------- */
void interrupt() {
    if (PIR1 & CCP1IF) {
        if (highPhase) {                    /* HIGH?LOW edge */
            CCPR1H  = (unsigned char)(pulseTicks >> 8);
            CCPR1L  = (unsigned char)pulseTicks;
            CCP1CON = 0x09;                 /* clear RC2     */
            highPhase = 0;
        } else {                            /* LOW?HIGH edge */
            unsigned int rest = SERVO_FRAME - pulseTicks;
            CCPR1H  = (unsigned char)(rest >> 8);
            CCPR1L  = (unsigned char)rest;
            CCP1CON = 0x08;                 /* set RC2       */
            highPhase = 1;
        }
        TMR1H = 0;  TMR1L = 0;              /* restart frame */
        PIR1 &= ~CCP1IF;
    }
}

/* --------------  HC-SR04 DISTANCE (cm)  ------------------ */
/*   – masks CCP1 interrupt for the measurement
     – reuses Timer-1, then restores everything            */
int dist_cm(void){
    unsigned char saveIE = PIE1 & CCP1IE;    /* remember state */
    PIE1 &= ~CCP1IE;                         /* servo off      */

    /* reset & start Timer-1 */
    T1CON &= ~0x01;  TMR1H = 0; TMR1L = 0;
    T1CON |=  0x01;

    /* 10 µs trigger on *RC5* */
    PORTC |=  0b00100000;
     delay10us();
    PORTC &= ~0b00100000;

    /* wait for echo HIGH on RC3 */
    while(!(PORTC & 0b00001000));
    TMR1H = 0; TMR1L = 0;                     /* zero on edge  */
    while(  PORTC & 0b00001000);              /* wait LOW      */
    T1CON &= ~0x01;                           /* stop Timer1   */

    unsigned int ticks = (TMR1H << 8) | TMR1L;/* 0.5 µs ticks  */
    PIE1 |= saveIE;                           /* servo back on */

    return (int)(ticks / 58);  /* ˜ (ticks*0.5 µs) / 58 µs/cm */
}

/* ---------------------------  MAIN  ---------------------- */
void main(void)
{
    unsigned int pot;    unsigned char duty;
    int distance;

    OPTION_REG &= 0x7F;            /* enable PORTB pull-ups */

    /*  TRIS settings  */
    TRISB = 0x03;                  /* RB0,RB1 inputs, RB3 out  */
    TRISA = 0x01;                  /* RA0 analogue            */
    TRISC = 0x08;                  /* RC3 input, rest outputs */

    /*  ADC: AN0 only  */
    ADCON1 = 0xCE;                 /* AN0 analogue            */
    ADCON0 = 0x41;                 /* ADC ON, ch0, Fosc/16    */

    /*  CCP1 (servo)  */
    TMR1H = 0;  TMR1L = 0;
    CCP1CON = 0x08;                /* first compare sets RC2  */
    CCPR1H  = (unsigned char)(pulseTicks >> 8);
    CCPR1L  = (unsigned char)pulseTicks;
    T1CON   = 0x01;                /* Timer-1 ON, 1:1 presc.  */

    PIE1   |= CCP1IE;
    INTCON |= PEIE | GIE;

    /*  CCP2 (pump)  */
    T2CON   = 0x07;                /* Timer-2 ON, 1:16 presc. */
    PR2     = PR2_VALUE;
    CCP2CON = 0x0C;                /* PWM mode                */
    CCPR2L  = 0x00;                /* duty 0                  */

    while(1)
    {
        /* ----------  PUMP SPEED  ---------- */
        pot  = adc_read_ra0();          /* 0-1023 */
        duty = pot_to_pwm(pot);         /* 0-249  */

        if(!(PORTB & 0x02)){            /* RB1 LOW ? hand */
            CCPR2L = duty;              /* run pump       */
        } else {
            CCPR2L = 0;                 /* stop pump      */
        }

        /* ----------  WATER LEVEL  --------- */
        distance = dist_cm();           /* ˜2 ms blocking */
        if(distance > 100){             /* tank low?      */
            PORTB |=  0b00001000;       /* RB3 LED ON     */
        } else {
            PORTB &= ~0b00001000;       /* LED OFF        */
        }

        /* ----------  SERVO CONTROL -------- */
        if(!(PORTB & 0x01)){            /* RB0 LOW ? open */
            pulseTicks = SERVO_MAX;     /* 180°           */
        } else {                        /* no object      */
            pulseTicks = SERVO_MIN;     /* 0°             */
        }

        mydelay();                      /* idle ~few ms   */
    }
}
