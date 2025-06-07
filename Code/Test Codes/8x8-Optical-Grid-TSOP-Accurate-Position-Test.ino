/*
  Filename: 8x8-Optical-Grid-TSOP-Accurate-Position-Test.ino
  Purpose: Read and compute the 2D position of a small ball on an 8×8 TSOP4838 sensor grid using sample-and-hold
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
    - If readings are invalid, the last valid position is retained
*/



// --------- Setup ----------
const int NUM_TSOP = 8;
const int tsopPinX[NUM_TSOP] = {22, 23, 24, 25, 26, 27, 28, 29};
const int tsopPinY[NUM_TSOP] = {30, 31, 32, 33, 34, 35, 36, 37};
const float Ts = 0.05;           // Sampling period (s)
const unsigned long Ts_ms = Ts * 1000;
const float d = 12.5;            // Distance between sensors (mm)
const float edgeLeft = 9.375;    // Estimated position if only sensor 1 is triggered (mm)
const float edgeRight = 90.625;  // Estimated position if only sensor 8 is triggered (mm)


int xSensorState[NUM_TSOP];
int ySensorState[NUM_TSOP];

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < NUM_TSOP; i++) {
    pinMode(tsopPinX[i], INPUT);
    pinMode(tsopPinY[i], INPUT);
  }
}

float xPos = 0.0;
float yPos = 0.0;
float xPos_last = 50.0; // Initialize at center
float yPos_last = 50.0;

void loop() {
  unsigned long tStart = millis();

  int xHighCount = 0, xFirst = -1, xSecond = -1;
  int yHighCount = 0, yFirst = -1, ySecond = -1;

  // --- Read X sensors ---
  for (int i = 0; i < NUM_TSOP; i++) {
    xSensorState[i] = digitalRead(tsopPinX[i]);
    if (xSensorState[i]) {
      xHighCount++;
      if (xFirst == -1) xFirst = i + 1;      // 1-based index
      else if (xSecond == -1) xSecond = i + 1;
    }
  }
  // --- Read Y sensors ---
  for (int i = 0; i < NUM_TSOP; i++) {
    ySensorState[i] = digitalRead(tsopPinY[i]);
    if (ySensorState[i]) {
      yHighCount++;
      if (yFirst == -1) yFirst = i + 1;
      else if (ySecond == -1) ySecond = i + 1;
    }
  }

  // --- Determine X position ---
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
    xPos = xPos_last; // Use last valid value for x-coordinate if more than 2 sensors or 0 sensors are HIGH
  }

  // --- Determine Y position ---
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
    yPos = yPos_last; // Use last valid value for y-coordinate if more than 2 sensors or 0 sensors are HIGH
  }

  // --- Update Last Valid Values ---
  if (xValid) xPos_last = xPos;
  if (yValid) yPos_last = yPos;

  // --- Print Result ---
  Serial.print("(x,y) = (");
  Serial.print(xPos, 3);
  Serial.print(", ");
  Serial.print(yPos, 3);
  Serial.println(")");

  
  while (millis() - tStart < Ts_ms);
}
