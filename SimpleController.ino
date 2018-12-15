/*
 * Simple controller
 * 
 * -> A simple flight controller for an arduino uno based tricopter
 * 
 * AUTHOR: Kevin McAndrew
 * DATE: 8 December 2018
 * 
 */

#include <Servo.h>
#include <movingAvg.h>
#include <EnableInterrupt.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>

//////////////////////////////////////////////////////////////////////////
// Receiver
// -> The receiver is a Flysky FC-RSB with 6 pwm channels
//////////////////////////////////////////////////////////////////////////
#define RX_PIN_THROTTLE  6 //Receiver input pins
#define RX_PIN_ROLL  5  
#define RX_PIN_PITCH  4  
#define RX_PIN_YAW  7

PROGMEM const byte rxPins[6]={RX_PIN_YAW,RX_PIN_ROLL,RX_PIN_PITCH,RX_PIN_THROTTLE};
volatile int rxState[4]={0,0,0,0};  //Temporary receiver input variables
volatile int rxPrev[4]={0,0,0,0};   //Used for smoothing

float pwm_yaw, pwm_throttle, pwm_pitch, pwm_roll; //Receiver input variables

//Smoothing of the PWM inputs
movingAvg pitchAvg(10), rollAvg(10), yawAvg(10), throttleAvg(10);

//////////////////////////////////////////////////////////////////////////
// Gyro and accelerometer
//////////////////////////////////////////////////////////////////////////
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();  //LSM9DS0 object
float gyro_yaw, gyro_pitch, gyro_roll;      //Gyro input variables

//Angular position based on accelerometer
double accel_angle_pitch, accel_angle_roll, accel_angle_yaw;

//Raw gyro offsets to prevent drift (Different value for different gyros)
float offset_yaw    = -4;
float offset_pitch  = 0;
float offset_roll   = 1.5;

float level_roll   = 102.2;  //Value of roll angle when level
float level_pitch    = 101.2; //Value of pitch angle when level

//////////////////////////////////////////////////////////////////////////
// BLDC objects
//////////////////////////////////////////////////////////////////////////
Servo tailESC, leftESC, rightESC, tailServo; //Motor objects
float tail, left, right, yawServo; //Motor outputs

//////////////////////////////////////////////////////////////////////////
// PID gain and limits
//////////////////////////////////////////////////////////////////////////
float p_pitch, i_pitch, d_pitch, pid_pitch; //PID variables for pitch
float gyro_pitch_prev, level_pitch_prev;    //Derivative previous pitch variables

float p_roll, i_roll, d_roll, pid_roll; //PID variables for roll
float gyro_roll_prev, level_roll_prev;  //Derivative previous roll variables

float d_yaw, pid_yaw, gyro_yaw_prev;

float p_gain_pitch = 2.5;              //Gain setting for the pitch P-controller
float i_gain_pitch = 0.025;            //Gain setting for the pitch I-controller
float d_gain_pitch = 100;              //Gain setting for the pitch D-controller
int pid_max_pitch = 150;               //Maximum output of the PID-controller (+/-)

float p_gain_roll = p_gain_pitch;      //Gain setting for the roll P-controller
float i_gain_roll = i_gain_pitch;      //Gain setting for the roll I-controller
float d_gain_roll = d_gain_pitch;      //Gain setting for the roll D-controller
int pid_max_roll = pid_max_pitch;      //Maximum output of the PID-controller (+/-)

float d_gain_yaw = 50;                 //Gain setting for the yaw D-controller
int pid_max_yaw = 30;                  //Maximum output of the PID-controller

//////////////////////////////////////////////////////////////////////////
// Comparative filter
//////////////////////////////////////////////////////////////////////////
double tim1, dt;                        //Comparative filter timing for integrator
double compx, compy, compz;             //Comparative filter outputs
double compx_old, compy_old, compz_old;

/************************************************************************/
// Setup
/************************************************************************/
void setup() 
{
  Serial.begin(230400); //Start serial at 9600 baud

  //Setup receiver pwm reads
  rxInit();
  pitchAvg.begin();
  rollAvg.begin();
  yawAvg.begin();
  throttleAvg.begin();

  //Setup gyro, accelerometer and compass (include the full scale deflection)
  //If there is a low full scale deflection set the IMU could freeze on large spikes
  lsm.setupAccel  (lsm.LSM9DS0_ACCELRANGE_16G);
  lsm.setupMag    (lsm.LSM9DS0_MAGGAIN_12GAUSS);
  lsm.setupGyro   (lsm.LSM9DS0_GYROSCALE_2000DPS);

  //Try to initialise IMU and warn if we couldn't detect the chip
  if (!lsm.begin()){
    Serial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
    while (1);}

  //Intialise comparative filter
  //This is so that there are some intial previous values
  lsm.read();
  //Calculate accelerometer angle
  compx = atan2(lsm.accelData.z,lsm.accelData.y)*(180/PI);
  compy = atan2(lsm.accelData.z,lsm.accelData.x)*(180/PI);
  gyro_pitch_prev = compx;
  level_pitch_prev = level_pitch;
  gyro_roll_prev = compy;
  level_roll_prev = level_roll;

  //Attach motors
  tailESC.attach(9);
  leftESC.attach(10);
  rightESC.attach(11);
  tailServo.attach(3);

  //Initialise comparitive filter timer
  tim1 = millis();
}

/************************************************************************/
// Main loop
/************************************************************************/

void loop() 
{ 
  // Receiver inputs from ISR
  pwm_yaw =       rxState[3];
  pwm_throttle =  rxState[0];
  pwm_pitch =     rxState[2];
  pwm_roll =      rxState[1];

  // Normalize receiver inputs to full scale angles
  pwm_yaw = (pwm_yaw-1500)/12.5;      //+-20deg
  pwm_pitch = -1*(pwm_pitch-1500)/22.22; //+-45deg
  pwm_roll = -1*(pwm_roll-1500)/22.22;  //+-45deg

  //Smooth noising pwm receive inputs
  pwm_pitch = pitchAvg.reading(pwm_pitch);
  pwm_roll = rollAvg.reading(pwm_roll);
  pwm_yaw = yawAvg.reading(pwm_yaw);
  pwm_throttle = throttleAvg.reading(pwm_throttle); 

  // Timer for integrator in comparative filters
  dt = (millis() - tim1)/1000;
  tim1 = millis();

  // Read the raw gyro inputs
  lsm.read();
  gyro_pitch  = (lsm.gyroData.x)/(100) - offset_pitch;
  gyro_roll   = (lsm.gyroData.y)/(100) - offset_roll;
  gyro_yaw    = (lsm.gyroData.z)/(100) - offset_yaw;

  //Calculate angles using accelerometer
  //Scale the angle between 360deg and 0deg
  /* accel_angle_pitch = tan-1(z/y) */
  accel_angle_pitch = atan2(lsm.accelData.z,lsm.accelData.y)*(180/PI);
  if(accel_angle_pitch < 0) accel_angle_pitch = 360 + accel_angle_pitch;
  /* accel_angle_pitch = tan-1(z/x) */
  accel_angle_roll = atan2(lsm.accelData.z,lsm.accelData.x)*(180/PI);
  if(accel_angle_roll < 0) accel_angle_roll = 360 + accel_angle_roll;

  //Update previous values
  compx_old = compx;
  compy_old = compy;
  compz_old = compz;

  //Comparative filters
  /* [gyroGain * (prevAngle - change in angle)] + [accelGain * accelAngle]*/
  /* Change in angle is calculated by integrating - gyro(deg per sec)*sec = degrees */
  compx = 0.99 * (compx - (gyro_pitch * dt)) + (0.01 * (accel_angle_pitch));
  if(abs(compx - accel_angle_pitch) > 180) compx = accel_angle_pitch;
  compy = 0.99 * (compy + (gyro_roll * dt)) + (0.01 * (accel_angle_roll));
  if(abs(compy - accel_angle_roll) > 180) compy = accel_angle_roll;
  compz = 1.00 * (compz + (gyro_yaw * dt * (-1))) + (0.00 * (accel_angle_yaw));

  //Pitch PID
  /* targAngle is a function of the level value and input from the transmitter*/
  /* p = (realAngle - targAngle) * pGain */
  /* i += (realAngle - targAngle) * iGain */
  /* d = [(realAngle - targAngle) - (prevError)] * pGain */
  p_pitch = (compx - (level_pitch + pwm_pitch)) * p_gain_pitch;
  i_pitch += (compx - (level_pitch + pwm_pitch)) * i_gain_pitch;
  if(i_pitch > 40) i_pitch = 40;  //Integration limits to prevent overcompensation
  if(i_pitch < -40) i_pitch = -40;
  d_pitch = (compx - (level_pitch + pwm_pitch) - (gyro_pitch_prev - level_pitch_prev)) * d_gain_pitch;
  //Combine P, I and D
  pid_pitch = p_pitch + i_pitch + d_pitch;
  //PID controller limits
  if(pid_pitch > pid_max_pitch) pid_pitch = pid_max_pitch;
  if(pid_pitch < -pid_max_pitch) pid_pitch = -pid_max_pitch;
  // Save derivative values
  gyro_pitch_prev = compx;
  level_pitch_prev = level_pitch + pwm_pitch;

  //Roll PID
  /* targAngle is a function of the level value and input from the transmitter*/
  /* p = (realAngle - targAngle) * pGain */
  /* i += (realAngle - targAngle) * iGain */
  /* d = [(realAngle - targAngle) - (prevError)] * pGain */
  p_roll = (compy - (level_roll + pwm_roll)) * p_gain_roll;
  i_roll += (compy - (level_roll + pwm_roll)) * i_gain_roll;
  if(i_roll > 40) i_roll = 40;  //Integration limits to prevent overcompensation
  if(i_roll < -40) i_roll = -40;
  d_roll = ((compy - (level_roll + pwm_roll)) - (gyro_roll_prev - level_roll_prev)) * d_gain_roll;
  //Combine P, I and D
  pid_roll = p_roll + i_roll + d_roll;
  //PID controller limits
  if(pid_roll > pid_max_roll) pid_roll = pid_max_roll;
  if(pid_roll < -pid_max_roll) pid_roll = -pid_max_roll;
  // Save derivative values
  gyro_roll_prev = compy;
  level_roll_prev = level_roll + pwm_roll;

  //Yaw PID
  //There is no need for P or I
  //D is used to stop drift and prevent the drone from twisting
  /* d = [(realAngle - targAngle) - (prevError)] * pGain */
  d_yaw = (compz - gyro_yaw_prev) * d_gain_yaw;
  pid_yaw = d_yaw;
  //PID controller limits
  if(pid_yaw > pid_max_yaw) pid_yaw = pid_max_yaw;
  if(pid_yaw < -pid_max_yaw) pid_yaw = -pid_max_yaw;
  // Save derivative values
  gyro_yaw_prev = compz;

  //Calculate tail motor value (add pitch)
  tail = pwm_throttle - pid_pitch;
  if(tail < 1000) tail = 1000;
  if(tail > 2000) tail = 2000;

  //Calculate left motor value (subtract pitch; subtract roll)
  left = pwm_throttle - pid_roll + (pid_pitch/2);
  if(left < 1000) left = 1000;
  if(left > 2000) left = 2000;

  //Calculate left motor value (subtract pitch; add roll)
  right = pwm_throttle + pid_roll + (pid_pitch/2);
  if(right < 1000) right = 1000;
  if(right > 2000) right = 2000;

  //Calculate left motor value
  yawServo = 60 - pid_yaw + pwm_yaw;
  if(yawServo < 30) yawServo = 30;
  if(yawServo > 110) yawServo = 110;

  //Set motor values
  tailESC.write(tail);
  leftESC.write(left);
  rightESC.write(right);
  tailServo.write(yawServo);

  delay(2);

  /* Used for debugging purposes*/
  /* WARNING: Each print causes a significant delay ~10ms which 
              throws off the PID integral and derivative controllers
  */
  //Serial.print(" compy: "); Serial.println(compy);
  //Serial.print(" compx: "); Serial.print(compx);
  //Serial.print(" dt: "); Serial.println(dt*1000);
  //Serial.print(" tail: "); Serial.println(tail);
}

//////////////////////////////////////////////////////////////////////////
// Helper functions - Mostly for reading the PWM inputs using interrupts
//////////////////////////////////////////////////////////////////////////

void rxInit(){
  for(byte i=0;i<6;i++){
    pinMode(pgm_read_byte(&rxPins[i]),INPUT); 
    digitalWrite(pgm_read_byte(&rxPins[i]),HIGH);
  }
  enableInterrupt(RX_PIN_THROTTLE,rxGoesHigh1,RISING);
  enableInterrupt(RX_PIN_ROLL,rxGoesHigh2,RISING);
  enableInterrupt(RX_PIN_PITCH,rxGoesHigh3,RISING);
  enableInterrupt(RX_PIN_YAW,rxGoesHigh4,RISING);
}

void rxGoesHigh1(){
  enableInterrupt(RX_PIN_THROTTLE,rxGoesLow1,FALLING);
  rxPrev[0]=micros();
}
void rxGoesLow1(){
  enableInterrupt(RX_PIN_THROTTLE,rxGoesHigh1,RISING);
  rxState[0]=micros()-rxPrev[0];
}
void rxGoesHigh2(){
  enableInterrupt(RX_PIN_ROLL,rxGoesLow2,FALLING);
  rxPrev[1]=micros();
}
void rxGoesLow2(){
  enableInterrupt(RX_PIN_ROLL,rxGoesHigh2,RISING);
  rxState[1]=micros()-rxPrev[1];
}
void rxGoesHigh3(){
  enableInterrupt(RX_PIN_PITCH,rxGoesLow3,FALLING);
  rxPrev[2]=micros();
}
void rxGoesLow3(){
  enableInterrupt(RX_PIN_PITCH,rxGoesHigh3,RISING);
  rxState[2]=micros()-rxPrev[2];
}
void rxGoesHigh4(){
  enableInterrupt(RX_PIN_YAW,rxGoesLow4,FALLING);
  rxPrev[3]=micros();
}
void rxGoesLow4(){
  enableInterrupt(RX_PIN_YAW,rxGoesHigh4,RISING);
  rxState[3]=micros()-rxPrev[3];
}
