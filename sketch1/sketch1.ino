// ***************************************************************
// Denfinition based on 6302 wiring.
#define alternatePeriod 2.0 // Time Between alts in seconds
#define angleSensorPin  A5
#define pwmVoltageSensorPin  A3
#define motorVoltageSensorPin  A4
#define irSignalPin A8
#define hbMode 8
#define hbIn1A 9
#define hbIn2A 10
#define hbIn1B 11
#define hbIn2B 12
#define motorOutPWM  hbIn2A
#define monitorPin 2
#define dacRes 12   // Teensy 12-bit dac resolution, max val 4095
#define dacMax 4095
#define adcRes 14
#define adcCenter 8492 //Teensy ADC set to 14 bits, 16384/2 = 8192
#define adcMax 16383
#define analogAverages 20


// Circuit Specific Definitions.
#define scaleVadc 5.0 // All signals 0->5 volts.
#define vDrive 5.0 // Driver peak voltage.

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
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

}

void loop() {
// put your main code here, to run repeatedly:

  // Read analog values and average to reduce noise.
  int irRead = 0;
  for (int i = 0; i < analogAverages; i++) {
    irRead += analogRead(irSignalPin);
  }
  float irV = scaleVadc * float(irRead) / float(analogAverages*adcMax);

  Serial.println(irV);

}
