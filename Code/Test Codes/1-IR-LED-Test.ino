/*
  Filename: 1-IR-LED-Test.ino
  Purpose: Test if 1 IR LED can reliably send a signal at 38kHz
  Date: 2025-06-06
  Setup:
    - IR LED connected to D10
  Notes:
    - All parameters of the Timer is adjusted so that the signal frequency is approximately 38kHz
    - TSOP works fine with 38.46kHz, it is in the acceptable frequency input range
*/



/* ------------- Setup ------------ */
const byte irLedPin = 10;        // Arduino Mega D10 → ATmega2560 PB4 --> OC2A


void setup()
{
  pinMode(irLedPin, OUTPUT);     // Make OC2A output

  /* --- Stop Timer2, and reset the timer--- */
  TCCR2A = 0;                    
  TCCR2B = 0;                   
  TCNT2  = 0;                    

  /* --- Toogle OC2A at every Compare Match --- */
  TCCR2A |= (1 << COM2A0);      
  TCCR2A |= (1 << WGM21);        

  /* --- Frequency setting --- */
  OCR2A  = 25;                   // TOP = 25 --> ≈ 38,46 kHz output
  TCCR2B |= (1 << CS21);         // CS22:0 = 010 --> timer = F_CPU / 8
}

/* ---------- Main Loop ---------- */
void loop()
{
  /* PWM works itself without the need of any loop. Moving forward, there will be the main loop code here.*/
}
