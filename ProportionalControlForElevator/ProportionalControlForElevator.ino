#include <math.h>

//Lab00 Variables To Modify ******************************
float KpA =0.5; // Proportional Gain for Angle Error
float KpV = 0.0;  // Proportional Gain for BackEMF Error
float nominalBackEMF = 0.5;
float direct = 2.5;  
float desiredDeltaBackEMF = 0.0; // Alternate + and - offset from nominal
float desiredHeight = 25.0;
// motorCmd=direct+KpA*errorAngle+KpV*errorDeltaBackEMF

unsigned int deltaT = 10000;      // Sample period in microseconds.

// Definition based on 6302 wiring.
#define alternatePeriod 1.0 // Time Between alts in seconds
#define pwmVoltageSensorPin  A3
#define motorVoltageSensorPin  A4
#define angleSensorPin  A5
#define hbMode 4
#define hbIn1A 5
#define hbIn2A 6
#define hbIn1B 9
#define hbIn2B 10
#define motorOutPWM  hbIn2A
#define monitorPin 2
#define dacRes 12   // Teensy 12-bit dac resolution, max val 4095
#define dacMax 4095
#define adcRes 14
#define adcCenter 8492 //Teensy ADC set to 14 bits, 16384/2 = 8192
#define adcMax 16383
#define analogAverages 5
#define irSensorPin A8

// Circuit Specific Definitions.
#define scaleVadc 5.0 // All signals 0->5 volts.
#define vDrive 5.0 // Driver peak voltage.

// Variables for loop control
elapsedMicros loopTime; // loopTime used to force precise deltaT between starts
unsigned int headroom;  // Headroom=post execution time before next loop start.
boolean switchFlag;  // Used to monitor loop timing.
unsigned int alternateCount;  // Used to count loops since last alternate
float realTime = deltaT * 1.0e-6;

// Set up pins
void setup() {
  Serial.begin(115200);

  // Loop period Monitor
  pinMode(monitorPin, OUTPUT);

  // Set up inputs
  analogReadResolution(adcRes);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);

  // Set up outputs
  analogWriteResolution(dacRes);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

   // Set up side B for external bidirectional, A for unidirectional.
  digitalWrite(hbIn1B,HIGH);
  digitalWrite(hbIn2B, HIGH);
  digitalWrite(hbIn1A, LOW);
  digitalWrite(hbIn2A, LOW);
  digitalWrite(hbMode, HIGH);

  // Set PWM frequency
  analogWriteFrequency(motorOutPWM,23437.5); // Changes several timers!!!
  analogWrite(motorOutPWM, LOW);  
}

void loop() {  // Main code, runs repeatedly

  // Make sure loop starts deltaT microsecs since last start
  unsigned int newHeadroom = max(int(deltaT) - int(loopTime), 0);
  headroom = min(headroom, newHeadroom);

  while (int(loopTime) < int(deltaT)) {}; // looptime is an elapsedtime var.
  loopTime = 0;

  // Monitor Output should be an exact square wave with frequency = 1/(2*deltaT)
  switchFlag = !switchFlag;
  digitalWrite(monitorPin, switchFlag);

  // Read analog values and average to reduce noise.
  int motorI = 0;
  int pwmI = 0;
  int heightI = 0;
  for (int i = 0; i < analogAverages; i++) {
    motorI += analogRead(motorVoltageSensorPin);
    pwmI += analogRead(pwmVoltageSensorPin);
    heightI += analogRead(irSensorPin);
  }
  float motorV = scaleVadc * float(motorI) / float(analogAverages*adcMax);
  float pwmV = scaleVadc * float(pwmI) / float(analogAverages*adcMax);
  float heightV = scaleVadc * float(heightI) / float(analogAverages*adcMax);

  // backEMF = valtage across motor - voltage across motor internal resistance.
  // The voltage across the motor internal inductance assumed neglible.
  // We measure the voltage across an external current sense resistor, 
  //                      rsenseV = pwmV - motorV,
  // and a fraction of rsenseV is subtracted from motorV to get the backEMF.
  // That fraction is the ratio of the sense resistor to the internal
  // motor resistor (and must be calibrated by experiment).
  #define RmotorOverRsense 0.7
  float BackEMF = motorV - RmotorOverRsense * (pwmV - motorV);

  // Error is the difference between desired deltaBackEMF and 
  // the measured deltaBackEMF. 
  float measuredDeltaBackEMF = (BackEMF - nominalBackEMF);
  float errorV = (desiredDeltaBackEMF - measuredDeltaBackEMF);

  // Angle error
  float actualHeight = 20/(heightV-0.25);
  float errorH = (desiredHeight - actualHeight);

  // The Motor Command is a sum of a direct term and a gain*error
  float motorCmd = KpA * errorH;
  /*if (motorCmd >0 )  {
    digitalWrite(hbIn1A, LOW);
  } else {
    digitalWrite(hbIn1A, HIGH);
  }*/

  // Limit the motor command and write it out.
  float motorCmdLim = min(abs(motorCmd), vDrive);
  analogWrite(motorOutPWM, int((motorCmdLim / vDrive)*dacMax));
  //analogWrite(A14,int((motorCmdLim/vDrive)*dacMax));

  // If the alternate flag is set, alternate every alternate period.
  alternateCount += 1;
  if (float(alternateCount)*realTime > alternatePeriod) {
    alternateCount = 0;
    desiredDeltaBackEMF = -desiredDeltaBackEMF;
    headroom = deltaT;
  };

  // Print out BackEMF in millivolts so that the serial plotter autoscales.
  Serial.println(errorH);
}
