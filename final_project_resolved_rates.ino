#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "MultiDriver.h"
#include "SyncDriver.h"
#include <Stepper.h>

const int stepsPerRevolution = 2038;

float MATLAB_Angle[164];
float Angle1[41];
float Angle2[41];
float Angle3[41];
float Angle3new[41];
float Angle3ph;
float Angle4[41];
float Angle4div[41];
// using a 200-step motor Nema 17 is also 200 steps/revolution with the gearbox *5.18:1, it is 1028
#define MOTOR_STEPS 200
// configure the pins connected
#define STEP1 11
#define DIR1 10

#define STEP2 9
#define DIR2 8

#define STEP3 7
#define DIR3 6
// #define MS1 14
// #define MS2 15
// #define MS3 16

BasicStepperDriver stepper1(1028, DIR1, STEP1);
BasicStepperDriver stepper2(1028, DIR2, STEP2); 
BasicStepperDriver stepper3(MOTOR_STEPS, DIR3, STEP3);
// BasicStepperDriver stepper3(MOTOR_STEPS, DIR3, STEP3, MS1, MS2, MS3);
Stepper stepper4 = Stepper(stepsPerRevolution, 2,3,4,5);


void setup() {  
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  stepper1.begin(1,1);
  stepper2.begin(1,1);
  stepper3.begin(1,1);
  // stepper3.begin(1,16);
  stepper4.setSpeed(5);
}

void loop() {
  while (Serial.available() == 0) {}
  for(int i=0; i<164;i++)
  {
    MATLAB_Angle[i] = Serial.parseFloat(SKIP_ALL);
  }

  for(int k=0; k<41;k++)
  {
    Angle1[k]=MATLAB_Angle[k];
    Angle2[k]=MATLAB_Angle[41+k];
    Angle3[k]=MATLAB_Angle[82+k];
    Angle4div[k]=360/MATLAB_Angle[123+k];
    Angle4[k]= 2038/Angle4div[k];
  }
  
  Angle3ph = Angle3[0];

  for(int j=0; j<41; j++){
    if(abs(Angle3ph)<1.8){
      Angle3new[j]=0;
      if(j+1<41){
        Angle3ph = Angle3ph + Angle3[j+1];
      }
    }    
    else {
      Angle3new[j] = Angle3ph;
      Angle3ph = Angle3[j+1];
    }
  }

  for (int t=0; t<41;t++){
    Serial.print("Angle1["); Serial.print(t+1); Serial.print("] = "); Serial.println(Angle1[t]);
    Serial.print("Angle2["); Serial.print(t+1); Serial.print("] = "); Serial.println(Angle2[t]);
    Serial.print("Angle3["); Serial.print(t+1); Serial.print("] = "); Serial.println(Angle3new[t]);
    Serial.print("Angle4["); Serial.print(t+1); Serial.print("] = "); Serial.println(360/Angle4div[t]);

    stepper3.rotate(Angle3new[t]);
    delay(1000);
    stepper1.rotate(-Angle1[t]);
    stepper4.step(Angle4[t]);
    stepper2.rotate(Angle2[t]);
    delay(1000);
  }
}