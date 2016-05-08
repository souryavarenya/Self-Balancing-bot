//--------------------------Included Libraries----------------------------------//

#include "Wire.h"                                                     // For I2C communication
#include "I2Cdev.h"                                                   // For MPU6050
#include "MPU6050.h"                                                  // For MPU6050
#include <PID_v1.h>                                                   // PID Library

//-----------------------Initializing Variables---------------------------------//

float angle = 0;                                                      // Observed angle 

int forward = 8;                                                      // Motor Digital Pin
int backward = 7;                                                     // Motor Digital Pin
int motor = 9;                                                        // Motor PWM Pin

float multi = 0.6;                                                    // Multiplication factor for Kp, Ki and Kd
double Kp=120*multi, Ki=1*multi, Kd=7*multi;                          // Sub-Values of Kp, Ki and Kd
double Setpoint=0, Input, Output;                                     // Inputs for PID class
PID MotorPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);         // Defining MotorPID of PID class - used to implement PID   

MPU6050 accelgyro(0x68);                                              // Defining accelgyro of MPU6050 class - used to take input from gyro
int16_t ax1, ay1, az1;                                                // acceleration components
int16_t gx1, gy1, gz1;                                                // angular velocity components

float angle_accln;                                                    // Accln. due to angle
long t1=0;                                                            // variable to store time in millisecond

float dt1,anv1,angle1=0,an1;                                          // Some other intermediate variables

//--------------function for finding angle (Complementary Filter)---------------//

float find_angle(long tt){

  accelgyro.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);           // Calling getMotion6 method to find raw values of all 6 components
  
  angle_accln=(double(ay1)/double(az1)*180)/3.1416;                   // Angle calculated from orientation of acceleration
      
  dt1=float(millis()-tt)/1000;                                        // finding 'dt' in seconds
  anv1=(float(gx1))/131;                                              // angular velocity from Gyroscope
  angle1=angle1+(anv1*dt1);                                           // Angle calculated from gyroscope measurements
  
  an1=((0.96)*(an1+(anv1*dt1)))+(angle_accln*0.04);                   // COMPLEMENTARY FILTER IMPLEMENTATION
  
  return an1;
}

//---------------------------Setup/Initialize------------------------------------//

void setup() {
  Wire.begin();                                                       // Initialise I2C Bus
  Serial.begin(38400);                                                // Initialise Serial Communication
  accelgyro.initialize();                                             // Initialize MPU6050
  
  pinMode(motor, OUTPUT);                                             // Motor speed variable
  pinMode(forward, OUTPUT);                                           // Motor Direction
  pinMode(backward, OUTPUT);                                          // Motor Direction
  
  MotorPID.SetMode(AUTOMATIC);                                        // PID configuration
  MotorPID.SetOutputLimits(-255, 255);
  MotorPID.SetSampleTime(1);
}

//------------------------------------Main Loop----------------------------------//

void loop() {

  angle = find_angle(t1);                                             // Querying the angle
 
  if (angle>=0){                                                      // Conditional statement for checking angle
    Input = angle+7;                                                  // Input for PID is Theta
    MotorPID.Compute();                                               // Calling PID output for given input
    
    digitalWrite(forward, HIGH);                                      // Motor Control statements
    digitalWrite(backward, LOW);
    analogWrite(motor,(-1)*Output);
  }
  else{
    Input = angle+7;
    MotorPID.Compute();
    digitalWrite(forward, LOW);
    digitalWrite(backward, HIGH);
    analogWrite(motor,Output);
  }

  Serial.print(Input);                                                // Print input angle on Serial Interface - For DEBUGGING
  Serial.print("\t");
  Serial.print(Output);                                               // Print PID motor output on Serial Interface - For DEBUGGING
  Serial.println("\t");
  
  t1 = millis()+1;                                                    // Time compensation for next statement
  delay(1);                                                           // Delay 1 ms for processor stability
}
