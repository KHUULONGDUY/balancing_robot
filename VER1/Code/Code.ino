
#include "I2Cdev.h"
#include "math.h"
#include <Kalman.h>
#include "Wire.h"
#include <MPU6050_tockn.h>
#define leftMotorPWMPin   6
#define leftMotorDirPin1   3
#define leftMotorDirPin2   4
#define rightMotorPWMPin  5
#define rightMotorDirPin1  7
#define rightMotorDirPin2  8

#define Kp  23
#define Kd  0.001
#define Ki  25
#define sampleTime  0.005
#define targetAngle 0

//MPU6050 mpu;
MPU6050 mpu6050(Wire);


int16_t accY=0, accZ=0, gyroX=0;
int16_t accY_pre=0, accZ_pre=0, gyroX_pre=0;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error=0, prevError=0, errorSum=0, error_pre=0;
volatile bool exitISR = false;
int x, y, z;

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  if(leftMotorSpeed >= 0) {
    analogWrite(leftMotorPWMPin, leftMotorSpeed);
    digitalWrite(leftMotorDirPin1, LOW);
    digitalWrite(leftMotorDirPin2, HIGH);
  }
  else {
    leftMotorSpeed=-leftMotorSpeed;
    analogWrite(leftMotorPWMPin, leftMotorSpeed);
    digitalWrite(leftMotorDirPin1, HIGH);
    digitalWrite(leftMotorDirPin2, LOW);

  }
  if(rightMotorSpeed >= 0) {
    analogWrite(rightMotorPWMPin, rightMotorSpeed);
    digitalWrite(rightMotorDirPin1, HIGH);
    digitalWrite(rightMotorDirPin2, LOW);
  }
  else {
    rightMotorSpeed=-rightMotorSpeed;
    analogWrite(rightMotorPWMPin, rightMotorSpeed);
    digitalWrite(rightMotorDirPin1, LOW);
    digitalWrite(rightMotorDirPin2, HIGH);
  }
}

void init_PID() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();          // enable global interrupts
}

void setup() {
  Serial.begin(9600);
  // set the motor control and PWM pins to output mode
  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin1, OUTPUT);
  pinMode(leftMotorDirPin2, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin1, OUTPUT);
  pinMode(rightMotorDirPin2, OUTPUT);
  // initialize the MPU6050 and set offset values
  /*
  mpu.initialize();
  mpu.setYAccelOffset(-2884);
  mpu.setZAccelOffset(-426);
  mpu.setXGyroOffset(71);
  */
  // 
  Serial.begin(9600);  
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  init_PID();
 
}
void loop() {
 
  mpu6050.update();
  x = mpu6050.getAngleX();
  //y = mpu6050.getAngleY();
  //z = mpu6050.getAngleZ();
  
  /*
  Serial.print("X: "); Serial.print(x); Serial.print("      ");
  Serial.print("Y: "); Serial.print(y); Serial.print("      ");
  Serial.print("Z: "); Serial.println(z);
  */
  // GENERATE PULSE TO MOTOR
  motorPower = constrain(motorPower, -200, 200);
  setMotors(motorPower, motorPower);
  //Serial.println(motorPower);

}
// The ISR will be called every 5 milliseconds

ISR(TIMER1_COMPA_vect)
{  
  // calculate the angle of inclination
  error = x - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300,300);
  //calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(error-error_pre)/sampleTime;
  prevAngle = currentAngle;
  error_pre=error;
}
