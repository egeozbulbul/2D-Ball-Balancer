/*
  Filename: 1-TSOP-Test.ino
  Purpose: Test if one TSOP4838 IR sensor can reliably read signal at 38kHz
  Date: 2025-06-06
  Setup:
    - TSOP sensor connected to D22
    - Sampling period 50ms
  Notes:
    - I will manually distrupt the IR signal by placing my hand between the IR LED and the TSOP and check whether TSOP actually understands that there is a object
*/





// -------------  PIN and Time Setup  -------------
const int  sensorPin = 22;        // Input pin for TSOP4838: D22
const unsigned long Ts_ms = 50;   // Sampling period (ms)  --> Ts = 0.05 s


void setup()
{
  Serial.begin(9600);             // Initialize serial communication at 9600 baud speed
  pinMode(sensorPin, INPUT);      
}
// -------------------------------------

// -------------  Main Loop -------------
void loop()
{
  int sensorState = digitalRead(sensorPin);   // Read logic level at D22

  if (sensorState == HIGH)                    // If TSOP doesn't detect the IR LED (HIGH)
  {
    Serial.println("Object Detected");          
  }
  else                                        // If TSOP detects the IR LED (LOW)
  {
    Serial.println("Object Not Detected");      
  }

  delay(Ts_ms);                               // Wait 50 ms (sampling period)
}
// -------------------------------------
