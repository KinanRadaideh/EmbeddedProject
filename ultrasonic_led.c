// Ultrasonic TRIG ? RD0, ECHO ? RD1
// LED output   ? RD5

unsigned int Distance;

int dist() {
    // Reset Timer1
    TMR1H = 0;
    TMR1L = 0;

    // Send 10µs trigger on RD0
    PORTD |=  0b00000001; // RD0 = 1
    delay_us(10);
    PORTD &= ~0b00000001; // RD0 = 0

    // Wait for echo start on RD1
    while (!(PORTD & 0b00000010));
    T1CON |= 0b00000001;  // Start Timer1

    // Wait for echo end on RD1
    while ( PORTD & 0b00000010);
    T1CON &= ~0b00000001; // Stop Timer1

    // Convert Timer1 ticks to cm
    Distance = (TMR1L | (TMR1H << 8)) / 58.82;
    return Distance;
}

void main() {
    // TMR0 prescaler=256 on INT OSC; not used here
    OPTION_REG = 0x07;
    TMR0       = 0;

    // PORTC unused
    TRISC = 0x00;
    PORTC = 0x00;

    // RD0 output (TRIG), RD1 input (ECHO), RD5 output (LED), others outputs
    TRISD = 0b00000010;
    PORTD = 0x00;

    // Timer1 on, Fosc/4 source
    T1CON = 0b00000001;

    // No interrupts needed
    INTCON  = 0x00;
    PIE1    = 0x00;
    CCP1CON = 0x00;

    while (1) {
        Distance = dist();
        // Light RD5 if = 10 cm
        if (Distance >= 40cm){
         PORTD = PORTD |  0x10;
        }
        delay_ms(100);
    }
}

