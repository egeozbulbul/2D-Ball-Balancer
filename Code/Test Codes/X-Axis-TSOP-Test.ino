/*
  Filename: X-Axis-TSOP-Test.ino
  Purpose: Test if 8 TSOP4838 IR sensors on the x-axis can reliably read signal at 38kHz
  Date: 2025-06-06
  Setup:
    - TSOP sensors connected to D22–D29
    - Sampling period = 50ms
  Notes:
    - I will manually block the IR signals by placing my hand between the IR LEDs and the TSOP sensors so that 
    it blocks 1 or 2 IR LEDS, and move my hand along the x-axis. Then I will check whether the TSOP 
    sensors actually detects that there is a object
*/


// ---------------- Setup ----------------
const byte NUM_SENS = 8;                         // Number of sensors
const byte sensorPins[NUM_SENS] = {22,23,24,25,26,27,28,29};
const char* sensorNames[NUM_SENS] = {            // Seral monitor labels
  "X=1","X=2","X=3","X=4","X=5","X=6","X=7","X=8"
};

const unsigned long Ts_ms = 50;                  // Sampling period


void setup()
{
  Serial.begin(9600);                           
  for (byte i = 0; i < NUM_SENS; i++)
    pinMode(sensorPins[i], INPUT);      
}


// -------------------------------------------

// ---------------- Main Loop ----------------
void loop()
{
  byte activeCount = 0;                          // HIGH (Object Detected) counter
  float positionSum = 0;                         // Initializing weighted summation
  bool sensorState[NUM_SENS];                 

  for (byte i = 0; i < NUM_SENS; i++)
  {
    sensorState[i] = digitalRead(sensorPins[i]);

    if (sensorState[i] == HIGH)                  // Object detected
    {
      activeCount++;
      positionSum += (i + 1);                    
      Serial.print(sensorNames[i]);
      Serial.println(": Object Detected");
    }
    else                                         // Object not detected
    {
      Serial.print(sensorNames[i]);       
      Serial.println(": Object Not Detected");
    }
  }

  /* ---------- Interpret the Position ---------- */
  if (activeCount == 0)
  {
    Serial.println("Object Not On X Sensors");
  }
  else if (activeCount == 1)
  {
    /* Only 1 sensor is HIGH --> Exact position */
    byte idx = (byte)(positionSum) - 1;          
    Serial.print("Object Position → ");
    Serial.println(sensorNames[idx]);
  }
  else if (activeCount == 2)
  {
    /* 2 sensors are HIGH --> Average = (i1+i2)/2  */
    float xPos = positionSum / 2.0;              
    Serial.print("Object Position → X=");
    Serial.println(xPos, 1);                   
  }
  else
  {
    /* More than 2 sensors or no sensors are HIGH --> Error */
    Serial.println("Error");
  }

  Serial.println("---");                        
  delay(Ts_ms);                                  // Sampling period
}
