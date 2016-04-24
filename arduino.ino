#ifndef QUADARDU
#define QUADARDU
#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>
#include <PinChangeInt.h>

/* Motor Configuration
 
A  O    O  B     . 
    \  /        /|\ is the North of the UAV(Axis along which it pitches)]
    /  \         |
D  O    O  C

*/

#define ESC_A 11
#define ESC_B 10
#define ESC_C 6
#define ESC_D 5

#define RC_PWR 13
#define ESC_MIN 1000
#define ESC_MAX 1800            // We need some room to keep full control at full throttle.
#define ESC_TAKEOFF_OFFSET 1100 // #TODO will need some increments depending upon the weight of the UAV.
#define ESC_ARM_DELAY 5000

#define RC_HIGH_CH1 1000
#define RC_LOW_CH1 2000
#define RC_HIGH_CH2 1000
#define RC_LOW_CH2 2000
#define RC_HIGH_CH3 1000
#define RC_LOW_CH3 2000 
#define RC_HIGH_CH4 1000
#define RC_LOW_CH4 2000
#define RC_HIGH_CH5 1000
#define RC_LOW_CH5 2000
#define RC_ROUNDING_BASE 50
#define PITCH_P_VAL 0.5
#define PITCH_I_VAL 0
#define PITCH_D_VAL 1
#define ROLL_P_VAL 2
#define ROLL_I_VAL 5
#define ROLL_D_VAL 1
#define YAW_P_VAL 2
#define YAW_I_VAL 5
#define YAW_D_VAL 1
#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -30
#define ROLL_MAX 30
#define YAW_MIN -180
#define YAW_MAX 180
#define PID_PITCH_INFLUENCE 20
#define PID_ROLL_INFLUENCE 20
#define PID_YAW_INFLUENCE 20

MPU6050 mpu;                           // mpu interface object
uint8_t mpuIntStatus;                  // mpu statusbyte
uint8_t devStatus;                     // device status    
uint16_t packetSize;                   // estimated packet size  
uint16_t fifoCount;                    // fifo buffer size   
uint8_t fifoBuffer[64];                // fifo buffer 
Quaternion q;                          // quaternion for mpu output
VectorFloat gravity;                   // gravity vector for ypr
float ypr[3] = {0.0f,0.0f,0.0f};       // yaw pitch roll values
float yprLast[3] = {0.0f, 0.0f, 0.0f};
volatile bool mpuInterrupt = false;    //interrupt flag
boolean interruptLock = false;          //semaphore

// Roll,Pitch,Throttle,Yaw,Misc
float ch1=1500;
float ch2=1500;
float ch3=1000;
float ch4=1500;
float ch5=0;         // RC channel inputs

int throttle;                          // global throttle
float roll_output, pitch_output;                 // motor balances can vary between -100 & 100
float yaw_output;                       // throttle balance between axes -100:ac , +100:bd
int va, vb, vc, vd;                    //velocities

Servo a,b,c,d;
PID pitchReg(&ypr[1], &pitch_output, &ch2, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE);
PID rollReg(&ypr[2], &roll_output, &ch1, ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL, REVERSE);
PID yawReg(&ypr[0], &yaw_output, &ch4, YAW_P_VAL, YAW_I_VAL, YAW_D_VAL, DIRECT);
float ch1Last, ch2Last, ch4Last, throttleLast;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

unsigned long previousMillis = 0;        // will reset ch[1,2,4] values after 500ms to 1500(ms).
const long interval = 500;    

void setup(){
    initRC();                            // Self explaining
    initMPU();
    initESCs();
    initStabilization();
    initRegulators();
    Serial.begin(9600);                 // Serial from ESP8266
    Serial.flush();
    delay(10);
    inputString.reserve(200);
}
void loop(){
    while(!mpuInterrupt && fifoCount < packetSize){         /* Do nothing while MPU is not working     * This should be a VERY short period     */        }
    getYPR();                          
    computePID();
    calculateVelocities();
    updateMotors();
    if (stringComplete) {
      String valueString=inputString.substring(1);
      int valueInt=valueString.toInt();
      switch(inputString[0]){
      case '!': ch1= valueInt;
      break;
      case '@': ch2= valueInt;
      break;
      case '#': ch3= valueInt;
      break;
      case '$': ch4= valueInt;
      break;
      case '%': ch5= valueInt;
      break;
        }
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
  unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
       previousMillis = currentMillis;
       ch1=1500;
        ch2=1500;
         ch4=1500;
    }
  }
void computePID(){

  acquireLock();
  
  ch1 = floor(ch1/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  ch2 = floor(ch2/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  ch4 = floor(ch4/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;

  ch2 = map(ch2, RC_LOW_CH2, RC_HIGH_CH2, PITCH_MIN, PITCH_MAX);
  ch1 = map(ch1, RC_LOW_CH1, RC_HIGH_CH1, ROLL_MIN, ROLL_MAX);
  ch4 = map(ch4, RC_LOW_CH4, RC_HIGH_CH4, YAW_MIN, YAW_MAX);
  
  if((ch2 < PITCH_MIN) || (ch2 > PITCH_MAX)) ch2 = ch2Last;
  if((ch1 < ROLL_MIN) || (ch1 > ROLL_MAX)) ch1 = ch1Last;
  if((ch4 < YAW_MIN) || (ch4 > YAW_MAX)) ch4 = ch4Last;
  
  ch1Last = ch1;
  ch2Last = ch2;
  ch4Last = ch4;


  //To Degrees
  ypr[0] = ypr[0] * 180/M_PI;
  ypr[1] = ypr[1] * 180/M_PI;
  ypr[2] = ypr[2] * 180/M_PI;
  
  if(abs(ypr[0]-yprLast[0])>30) ypr[0] = yprLast[0];
  if(abs(ypr[1]-yprLast[1])>30) ypr[1] = yprLast[1];
  if(abs(ypr[2]-yprLast[2])>30) ypr[2] = yprLast[2];
  
  yprLast[0] = ypr[0];
  yprLast[1] = ypr[1];
  yprLast[2] = ypr[2];

  pitchReg.Compute();
  rollReg.Compute();
  yawReg.Compute();
  
  releaseLock();

}
void getYPR(){
  
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if((mpuIntStatus & 0x10) || fifoCount >= 1024){ 
      
      mpu.resetFIFO(); 
    
    }else if(mpuIntStatus & 0x02){
    
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      fifoCount -= packetSize;
    
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    }

}
void calculateVelocities(){

  acquireLock();

  ch3 = floor(ch3/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  
  throttle = constrain(ch3, RC_LOW_CH3, RC_HIGH_CH3);
  
  releaseLock();

  if((throttle < RC_LOW_CH3) || (throttle > RC_HIGH_CH3)) throttle = throttleLast;
  
  throttleLast = throttle;

va=throttle + roll_output + pitch_output - yaw_output;
vb=throttle - roll_output + pitch_output + yaw_output;
vc=throttle - roll_output - pitch_output - yaw_output;
vd=throttle + roll_output - pitch_output + yaw_output;

 
  if(throttle < ESC_TAKEOFF_OFFSET){
  
    va = ESC_MIN;
    vb = ESC_MIN;
    vc = ESC_MIN;
    vd = ESC_MIN;
  }
}
inline void updateMotors(){

  a.writeMicroseconds(va);
  c.writeMicroseconds(vc);
  b.writeMicroseconds(vb);
  d.writeMicroseconds(vd);

}
inline void arm(){

  a.write(ESC_MIN);
  b.write(ESC_MIN);
  c.write(ESC_MIN);
  d.write(ESC_MIN);
  
  delay(ESC_ARM_DELAY);

}
inline void dmpDataReady() {
    mpuInterrupt = true;
}
inline void initRC(){
  pinMode(RC_PWR, OUTPUT);
  digitalWrite(RC_PWR, HIGH);
  
}
void initMPU(){
  
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if(devStatus == 0){
  
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
    
  }
}
inline void initESCs(){

  a.attach(ESC_A);
  b.attach(ESC_B);
  c.attach(ESC_C);
  d.attach(ESC_D);
  
  delay(100);
  
  arm();

}
inline void initStabilization(){

  yaw_output = 0;
  roll_output = 0;
  pitch_output = 0;

}
inline void initRegulators(){

  pitchReg.SetMode(AUTOMATIC);
  pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);
  
  rollReg.SetMode(AUTOMATIC);
  rollReg.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);
  
  yawReg.SetMode(AUTOMATIC);
  yawReg.SetOutputLimits(-PID_YAW_INFLUENCE, PID_YAW_INFLUENCE);

}
inline void acquireLock(){
  interruptLock = true; 
}
inline void releaseLock(){
  interruptLock = false;
}
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
#endif
