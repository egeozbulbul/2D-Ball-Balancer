/*
  Filename: 8x8-Optical-Grid-TSOP-Test.ino
  Purpose: Test whether the 2D (X-Axis & Y-Axis) 8x8 Optical Grid with TSOP4838 IR sensors can reliably read signal at 38kHz
  Date: 2025-06-04
  Setup:
    - TSOP sensors from D22 to D29 is on the X-Axis
    - TSOP sensors form D30 to D37 is on the Y-Axis
  Notes:
    - This code still works to detect if the 2D 8x8 sensor array can detect an object or not. Its purpose isn't to calculate the positions accurately. It only gives an estimate by interpreting the sensor values. 

*/

//-------------------------Setup------------------------

const byte NUM_SENS = 8;

/* X-Axis: D22–D29 */
const byte xSensorPins[NUM_SENS]  = {22,23,24,25,26,27,28,29};
const char* xSensorNames[NUM_SENS] = {
  "X=1","X=2","X=3","X=4","X=5","X=6","X=7","X=8"
};

/* Y-Axis: D30–D37 */
const byte ySensorPins[NUM_SENS]  = {30,31,32,33,34,35,36,37};
const char* ySensorNames[NUM_SENS] = {
  "Y=1","Y=2","Y=3","Y=4","Y=5","Y=6","Y=7","Y=8"
};

const unsigned long Ts_ms = 50;          // Sampling period


void setup()
{
  Serial.begin(9600);
  /* Tüm X+Y pinlerini girişe al */
  for (byte i = 0; i < NUM_SENS; i++)
  {
    pinMode(xSensorPins[i], INPUT);
    pinMode(ySensorPins[i], INPUT);
  }
}

// --------------------------------------------------------

/* ------------------ YARDIMCI FONKSİYON ------------------
   Reads the axis values; number of sensors that reads a HIGH (activeCount), 
   is converted to a weighted summation (positionSum)                   */
void readAxis(const byte pinArr[], const char* nameArr[],
              byte& activeCount, float& positionSum)
{
  activeCount = 0;
  positionSum = 0;

  for (byte i = 0; i < NUM_SENS; i++)
  {
    bool s = digitalRead(pinArr[i]);

    if (s)   /* HIGH --> Object detected */
    {
      Serial.print(nameArr[i]);
      Serial.println(": Object Detected");
      activeCount++;
      positionSum += (i + 1);   
    }
    else
    {
      Serial.print(nameArr[i]);
      Serial.println(": Object Not Detected");
    }
  }
}

/* -------------- Position Interpretation -----------------
   Calculates a position value from activeCount & positionSum
   - 0  : error   (noObject=false, pos=0)
   - 1  : 1 sensor index
   - 2  : 2 consecutive sensors --> average
   - >2 : error  (noObject=false, pos=0, ambiguous=true)   */
bool solveAxis(byte activeCount, float positionSum,
               float& pos, bool& ambiguous)
{
  ambiguous = false;

  if (activeCount == 0)                 // no HIGH
    return false;                       // noObject

  else if (activeCount == 1)            // 1 sensor
  {
    pos = positionSum;              
  }
  else if (activeCount == 2)            // 2 sensors
  {
    pos = positionSum / 2.0f;           // average
  }
  else                                  // more than 2 sensors
  {
    ambiguous = true;
    return false;                       // error
  }
  return true;                        
}

/* -------------------- Main Loop -------------------- */
void loop()
{
  // Read X-Axis
  byte  actX;
  float sumX;
  readAxis(xSensorPins, xSensorNames, actX, sumX);

  // Read Y-Axis
  byte  actY;
  float sumY;
  readAxis(ySensorPins, ySensorNames, actY, sumY);

  // Interpret
  float xPos = 0, yPos = 0;
  bool ambX = false, ambY = false;
  bool okX  = solveAxis(actX, sumX, xPos, ambX);
  bool okY  = solveAxis(actY, sumY, yPos, ambY);

  // Report
  if (okX && okY)
  {
    Serial.print("Ball position → (");
    Serial.print(xPos, 1);   
    Serial.print(", ");
    Serial.print(yPos, 1);
    Serial.println(")");
  }
  else
  {
    if (!okX)
    {
      if (ambX) Serial.println("X-axis ambiguous – multiple sensors HIGH");
      else      Serial.println("Object Not on X Sensors");
    }
    if (!okY)
    {
      if (ambY) Serial.println("Y-axis ambiguous – multiple sensors HIGH");
      else      Serial.println("Object Not on Y Sensors");
    }
  }

  Serial.println("=== Frame End ===");
  delay(Ts_ms);               // Sampling period
}
