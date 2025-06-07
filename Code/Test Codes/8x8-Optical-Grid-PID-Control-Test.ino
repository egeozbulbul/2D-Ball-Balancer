/*
  Filename: 8x8-Optical-Grid-TSOP-PID-Control-Test.ino
  Purpose: Measure 2D position on an 8×8 TSOP4838 sensor grid and apply PID control to reach setpoints
  Date: 2025-06-07
  Setup:
    - X-axis TSOP sensors connected to digital pins D22–D29
    - Y-axis TSOP sensors connected to digital pins D30–D37
    - Sampling period: 50 ms
    - Sensor spacing: 12.5mm
    - Offset: 6.25mm
  Notes:
    - Two adjacent sensors high --> position = lower-index × sensor spacing
    - Single end sensor high --> position = edge offset
    - Invalid readings retain last valid position
    - PID parameters Kp, Ki, Kd can be tuned later
    - PID is used for the position, the accurate angle that will be sent to the servo motors will be
    calculated later with either linear mapping or Euler-Lagrange equations
*/

 // ----------- Setup ------------
const int NUM_TSOP = 8;
const int tsopPinX[NUM_TSOP] = {22, 23, 24, 25, 26, 27, 28, 29};
const int tsopPinY[NUM_TSOP] = {30, 31, 32, 33, 34, 35, 36, 37};
const float Ts = 0.05;           // Sampling period (s)
const unsigned long Ts_ms = Ts * 1000;
const float d = 12.5;            // Sensor spacing (mm)
const float edgeLeft = 9.375;    // Position if only sensor 1 is triggered (mm)
const float edgeRight = 90.625;  // Position if only sensor 8 is triggered (mm)

// PID Parameters (to be tuned later)
float Kp = 2.0;   
float Ki = 0.2;
float Kd = 0.5;

// Setpoints (center of the square 10cmX10 cm plate)
float xSetpoint = 50.0; // Target X position (mm)
float ySetpoint = 50.0; // Target Y position (mm)

// Working Variables
int xSensorState[NUM_TSOP];
int ySensorState[NUM_TSOP];

float xPos = 0.0;
float yPos = 0.0;
float xPos_last = 50.0;
float yPos_last = 50.0;

// Variables for PID
float xError = 0.0, xIntegral = 0.0, xDerivative = 0.0, xPrevError = 0.0;
float yError = 0.0, yIntegral = 0.0, yDerivative = 0.0, yPrevError = 0.0;

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < NUM_TSOP; i++) {
    pinMode(tsopPinX[i], INPUT);
    pinMode(tsopPinY[i], INPUT);
  }
}

 // -------------- Main Loop ----------------

void loop() {
  unsigned long tStart = millis();

  int xHighCount = 0, xFirst = -1, xSecond = -1;
  int yHighCount = 0, yFirst = -1, ySecond = -1;

  // --- Read X Sensors ---
  for (int i = 0; i < NUM_TSOP; i++) {
    xSensorState[i] = digitalRead(tsopPinX[i]);
    if (xSensorState[i]) {
      xHighCount++;
      if (xFirst == -1) xFirst = i + 1;      // 1-based index
      else if (xSecond == -1) xSecond = i + 1;
    }
  }
  // --- Read Y Sensors ---
  for (int i = 0; i < NUM_TSOP; i++) {
    ySensorState[i] = digitalRead(tsopPinY[i]);
    if (ySensorState[i]) {
      yHighCount++;
      if (yFirst == -1) yFirst = i + 1;
      else if (ySecond == -1) ySecond = i + 1;
    }
  }

// For example: if x sensors 2 and 3 are HIGH then the x-coordinate is:
// d/2 + d (distance between sensors 1-2) + d/2 (middle of the sensors 2 and 3) = xFirst * d = 25mm


  // --- Determine X Position ---
  bool xValid = false;
  if (xHighCount == 2 && xSecond == xFirst + 1) {
    xPos = xFirst * d;
    xValid = true;
  } else if (xHighCount == 1 && xFirst == 1) {
    xPos = edgeLeft;
    xValid = true;
  } else if (xHighCount == 1 && xFirst == 8) {
    xPos = edgeRight;
    xValid = true;
  } else {
    xPos = xPos_last; // Use last valid value
  }

  // For example: if y sensors 3 and 4 are HIGH then the y-coordinate is: 
  //d/2 + 2*d (distance between sensors 1-2; and 2-3) + d/2 (middle of the sensors 2 and 3) = yFirst * d = 37.5mm

  // --- Determine Y Position ---
  bool yValid = false;
  if (yHighCount == 2 && ySecond == yFirst + 1) {
    yPos = yFirst * d;
    yValid = true;
  } else if (yHighCount == 1 && yFirst == 1) {
    yPos = edgeLeft;
    yValid = true;
  } else if (yHighCount == 1 && yFirst == 8) {
    yPos = edgeRight;
    yValid = true;
  } else {
    yPos = yPos_last;
  }

  // Update Last Valid Values
  if (xValid) xPos_last = xPos;
  if (yValid) yPos_last = yPos;

  // --- PID ALGORITHM ---

  // X-Axis PID
  xError = xSetpoint - xPos;
  xIntegral += xError * Ts;
  xDerivative = (xError - xPrevError) / Ts;
  float outputX = Kp * xError + Ki * xIntegral + Kd * xDerivative;
  xPrevError = xError;

  // Y-Axis PID
  yError = ySetpoint - yPos;
  yIntegral += yError * Ts;
  yDerivative = (yError - yPrevError) / Ts;
  float outputY = Kp * yError + Ki * yIntegral + Kd * yDerivative;
  yPrevError = yError;

  // --- Print Outputs ---
  Serial.print("(x,y) = (");
  Serial.print(xPos, 3);
  Serial.print(", ");
  Serial.print(yPos, 3);
  Serial.print(")  ");
  Serial.print("PIDout(x,y): (");
  Serial.print(outputX, 3);
  Serial.print(", ");
  Serial.print(outputY, 3);
  Serial.println(")");


  while (millis() - tStart < Ts_ms);
}
