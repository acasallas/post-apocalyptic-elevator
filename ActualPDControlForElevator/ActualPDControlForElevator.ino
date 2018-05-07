#include <math.h>

// Variables To Modify ******************************
float direct = 2.5;
float upValues[] = {1.250, 1.500, 1.750, 2.000, 2.250, 2.500, 2.750, 3.000, 3.250, 3.500, 3.750, 4.000, 4.250, 4.500, 4.750, 5.000};
float downValues[] = {1.250, -1.500, -1.750, -2.000, -2.250, -2.500, -2.750, -3.000, -3.250, -3.500, -3.750, -4.000, -4.250, -4.500, -4.750, -5.000};
int numValues = 16;
int valIndex = 0;
int moving = 0; //0: still, 1: down, 2: up (can make this enum)
int wait = 100;

// Variables to set using sliders
float Kp = 0.0; // Proportional Gain for Angle Error
float Kd = 0.0; // Delta gain
float Kbemf = 0.0;
float Ku = 0.0;
float Ki = 0.0;
float desired = 0.0;
float sum = 0.0;
float sumMax = 2.0;

// Loop timing, Derivative and integral variables
unsigned int deltaT = 35000;         // Sample period in microseconds.

#define pastSize 1                 // interval for delta, larger=less noise, more delay.
float dTsec = 1.0e-6*deltaT;       // The period in seconds. 
float scaleDeriv = 1.0/(dTsec*pastSize); // Divide deltas by interval.   

float errorVintegral;                // Variable for integrating angle errors.
float integralMax = 200;            // Maximum value of the integral

// ***************************************************************
// Variables for loop control 
elapsedMicros loopTime; // loopTime used to force precise deltaT between starts
unsigned int headroom;  // Headroom=post execution time before next loop start.
boolean switchFlag;  // Used to monitor loop timing.
unsigned int alternateCount;  // Used to count loops since last alternate
int loopCounter;
#define numSkip 5
bool first_time = false;


// Pick Arduino or Browser Monitor **********************************
boolean useBrowser = true;
String config_message_30_bytes = "&A~DesiredAng~5&C&S~DesiredAng~A~-1~1~0.1&S~Kp~P~0~20~0.1&S~Kd~D~0~5~0.1&S~Ki~I~0~5~0.1&T~Angle~F4~-2.5~2.5&T~Error~F4~-5~5&T~Deriv~F4~-10~10&T~Sum~F4~-100~100&T~Cmd~F4~0~5&D~100&H~4&";
String config_message = config_message_30_bytes; 

// Storage for past values.
float pastHeight[pastSize]; 

// Storage for browser communication
char buf[60]; 
float desiredAngleV = 0;

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
#define irSignalPin A8

// Circuit Specific Definitions.
#define scaleVadc 5.0 // All signals 0->5 volts.
#define vDrive 5.0 // Driver peak voltage.

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
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(A14, OUTPUT);
 
  // Set up side B for external bidirectional, A for unidirectional.
  digitalWrite(hbIn1B,HIGH);
  digitalWrite(hbIn2B, HIGH);
  digitalWrite(hbIn1A, HIGH);
  digitalWrite(hbIn2A, LOW);
  digitalWrite(hbMode, HIGH);

  // Set PWM frequency
  analogWriteFrequency(motorOutPWM,23437.5); // Changes several timers!!!
  analogWrite(motorOutPWM, LOW);
}

void loop() {  // Main code, runs repeatedly 
  // Reinitializes or updates from sliders on GUI.
  startup();
  
  // Make sure loop starts deltaT microsecs since last start
  unsigned int newHeadroom = max(int(deltaT) - int(loopTime), 0);
  headroom = min(headroom, newHeadroom);
  
  while (int(loopTime) < int(deltaT)) {};
  loopTime = 0;
  
  // Monitor Output should be a square wave with frequency = 1/(2*deltaT) 
  switchFlag = !switchFlag;
  digitalWrite(monitorPin, switchFlag);

  // *************calculate the motor command signal here***********************
  // States are angleV (angle), omegaV (d(angleV)/dt), deltaVemf, Input is desiredAngleV
    // Read analog values and average to reduce noise.
  int irRead = 0;
  for (int i = 0; i < analogAverages; i++) {
    irRead += analogRead(irSignalPin);
  }
  float irV = scaleVadc * float(irRead) / float(analogAverages*adcMax);
  float elev_h = sqrt(268.31/(irV-.7301));
  float errorH = ((desired*10)+20.0) - elev_h;
  float errorDiff = (errorH-pastHeight[pastSize-1]);
  sum = max(min(sum+errorH,sumMax),-sumMax);
  

  //float motorCmd = Kp*errorH + Kd*errorDiff + Ki*sum;

  
  float motorCmd = 0.0;
  if (moving == 0) {
    if (wait > 0) {
      wait--;
    } else {
      if (valIndex >= numValues-1) {
        valIndex = 0;
      } else {
        valIndex++;
      }
      
      if (elev_h < 25.0) {
        moving = 1;
      } else {
        moving = 2;
      }
    }
  } else if (moving == 1) {
    if (elev_h > 32.0) {
      moving = 0;
      wait = 100;
    } else {
      motorCmd = upValues[valIndex];
    }
  } else if (moving == 2) {
    if (elev_h < 18.0) {
      moving = 0;
      wait = 100;
    } else {
      motorCmd = downValues[valIndex];
    }    
  }

  if (motorCmd >0 )  {
    digitalWrite(hbIn1A, LOW);
  } else {
    digitalWrite(hbIn1A, HIGH);
  }

  
  float motorCmdLim = min(abs(motorCmd), vDrive);
  analogWrite(motorOutPWM,int((motorCmdLim/vDrive)*dacMax));
  //analogWrite(A14,int((motorCmdLim/vDrive)*dacMax));
  
  // Update previous errors for next time.
  for (int i = pastSize-1; i > 0; i--) pastHeight[i] = pastHeight[i-1];
  pastHeight[0] = errorH;

  if (loopCounter == numSkip) {  
    if (useBrowser) {
      packStatus(buf, elev_h, errorH, errorDiff, 0.0, motorCmdLim, float(headroom));
      Serial.write(buf,26);
    } else {
      // Print out in millivolts so that the serial plotter autoscales.
      Serial.println(elev_h);
    }
    loopCounter = 0;
    headroom = deltaT;
  } else loopCounter += 1;
}

void init_loop() {
  // Initialize loop variables
  loopCounter = 0; 
  headroom = deltaT;
  // Zero past errors
  for (int i = pastSize-1; i >= 0; i--) pastHeight[i] = 0;
}


void processString(String inputString) {
char St = inputString.charAt(0);
  inputString.remove(0,1);
  float val = inputString.toFloat();
  switch (St) {
    case 'P': 
      Kp = val;
      break;
    case 'D':
      Kd = val;
      break;  
    case 'E':
      Kbemf = val;
      break;
    case 'U':
      Ku = val;
      break;
    case 'I':
      Ki = val;
      break;  
    case 'O':  
      direct = val;
      break;
    case 'A':
      desired = val;
      break;
    case '~':
      first_time = true;
      break;
    default:
    break;  
  }
}

// Load the serial output buffer.
void packStatus(char *buf, float a, float b, float c, float d, float e, float f) {
  
  // Start Byte.
  buf[0] = byte(0);
  int n = 1; 
  
  memcpy(&buf[n],&a,sizeof(a));
  n+=sizeof(a);
  memcpy(&buf[n],&b,sizeof(b));
  n+=sizeof(b);
  memcpy(&buf[n],&c,sizeof(c));
  n+=sizeof(c);
  memcpy(&buf[n],&d,sizeof(d));
  n+=sizeof(d);
  memcpy(&buf[n],&e,sizeof(e));
  n+=sizeof(e);
  memcpy(&buf[n],&f,sizeof(f));
  n+=sizeof(f);
  /*
  memcpy(&buf[n],&g,sizeof(g));
   n+=sizeof(g);
   */

  // Stop byte (255 otherwise unused).
  buf[n] = byte(255); 
}

// Initializes the loop if this is the first time, or if reconnect sent
// from GUI.  Otherwise, just look for serial event.
void startup(){
  if (first_time) {
    while(Serial.available() > 0) Serial.read(); // Clear out rcvr.
    Serial.println(config_message);  // Send the configuration files.
    while (! Serial.available()) {}  // Wait for serial return.
    while(Serial.available() > 0) {   // Clear out rcvr.
        Serial.read();
        //char inChar = (char) Serial.read(); 
        //if (inChar == '\n') break;
    }
    init_loop();
    first_time = false;
  } else {
    serialEvent();
  }
}


// Simple serial event, only looks for disconnect character, resets loop if found.

// Simple serial event, only looks for disconnect character, resets loop if found.
void serialEvent() {
  String inputString = ""; 
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    // if the incoming character is a newline, ready to process.
    if (inChar == '\n') {
      processString(inputString);
      inputString = "";
      break;
    }
  }
}

