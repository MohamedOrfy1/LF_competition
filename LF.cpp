#include <Arduino.h>
#include "LF.h"
#include <PID_v1.h>
double startangle = 0, newangle = 0, Output = 0;
String tmp;
int previous_error = 0;
int oldtime=0;
int newtime=0;
int dt;
int dE;
int error = 0;
float errorslope;
float errorarea=0;
int a;
double correction=0;
double heading = 0;
int setpoint=0;
int pos;
int sensor1 , sensor2 , sensor3 , sensor4 , sensor5 ;
PID myPID(&newangle, &Output, &startangle, Kp, Ki, Kd, DIRECT);
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
   ========================================================================= */



#define OUTPUT_READABLE_YAWPITCHROLL




#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void forward(double* SpeedR,double* SpeedL) {
  analogWrite(EN1, (int)*SpeedR);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN2, (int)*SpeedL);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println(*SpeedR);
  Serial.println(*SpeedL);
}
void Forward_PID()
{
  double SpeedR = INITIAL_SPEEDR, SpeedL = INITIAL_SPEEDL;
  myPID.Compute();
  SpeedR = SpeedR - Output;
  SpeedL = SpeedL + Output;
  SpeedR = constrain(SpeedR, 30, 255);
  SpeedL = constrain(SpeedL, 30, 255);
  forward(&SpeedR,&SpeedL);
//  Serial.println(Output);
//  Serial.println(SpeedR);
//  Serial.println(SpeedL);
  delay(1000);
}
void Forward_PID2(){
       double SpeedR = INITIAL_SPEEDR, SpeedL = INITIAL_SPEEDL;
       
       Read(&sensor1,&sensor2,&sensor3,&sensor4,&sensor5);
       pos=((((2*~(sensor1))+(1*~(sensor2))+(-1*~(sensor4))+(-2*~(sensor5)))*1000)/(sensor1+sensor2+sensor4+sensor5));
       if(heading == 90)
       {
          startangle = 90;
       }
       else if(heading == -90)
       {
          startangle = -90;
       }
       else if(heading == 180)
       {
          startangle = 175 ;
       }

       oldtime=newtime;
       newtime=millis();
       dt=0.001*(newtime-oldtime);
       error=setpoint-pos;
       dE=error-previous_error;
       errorslope=dE/dt;
       errorarea=errorarea+error*dt;
       correction=Kp*error+Kd*errorslope+Ki*errorarea;
       SpeedR -=correction;
       SpeedL +=correction;
       SpeedR=constrain(SpeedR,0,255);
       SpeedL=constrain(SpeedL,0,255);
       forward(&SpeedR,&SpeedL );
      Serial.println(error);
      Serial.println(correction);
      Serial.println(SpeedR);
      Serial.println(SpeedL);

      //Serial.println(SpeedR);
      //Serial.println(SpeedL);
      previous_error=error;
  
  }
void left() {
  newangle=ReadMPU();
  if (heading==0){
  while(newangle > - 90){
  analogWrite(EN1, 80);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN2,80);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4,LOW);
  newangle=ReadMPU();
  }}else if (heading==-90){
  while(newangle > - 175){
  analogWrite(EN1, 80);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN2, 80);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  newangle=ReadMPU();
  }}else if (heading==90){
  while(newangle > 0){
  analogWrite(EN1, 80);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN2, 80);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  newangle=ReadMPU();
  }}else if (heading==180){
  while(newangle > 90){
  analogWrite(EN1, 80);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN2, 80);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  newangle=ReadMPU();
  }}
  switch((int)heading){
    case -90 : 
     heading=180;
     break;
    default : 
     heading-=90;
     break;
  }
 
}
void right() {
  if (heading==0){
  while(newangle < 30){
    analogWrite(EN1, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(EN2, 80);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    newangle = ReadMPU();
  }}else if (heading==-90){
  while(newangle < -30){
    analogWrite(EN1, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(EN2, 80);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    newangle = ReadMPU();
  }}else if (heading==90){
  while(newangle < 90){
    analogWrite(EN1, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(EN2, 80);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    newangle = ReadMPU();
  }}if (heading==180){
  while(newangle < -30){
   analogWrite(EN1, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(EN2, 80);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    newangle = ReadMPU();
  }}
  switch((int)heading){
    case 180 : 
     heading=-90;
     break;
    default : 
     heading+=90;
     break;
  }
}
void Uturn() {
  if (heading==0){
  while(newangle < 175){
    analogWrite(EN1, 120);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(EN2, 80);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    newangle = ReadMPU();
  }}if (heading==90){
  while(newangle < -75){
    analogWrite(EN1, 120);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(EN2, 80);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    newangle = ReadMPU();
  }}if (heading==-90){
  while(newangle < 75){
    analogWrite(EN1, 120);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(EN2, 80);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    newangle = ReadMPU();
  }}if (heading==180){
  while(newangle < 0){
    analogWrite(EN1, 120);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(EN2, 80);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    newangle = ReadMPU();
  }}
  
  switch((int)heading){
    case 180 : 
     heading=0;
     break;
    case 90 : 
     heading=-90;
     break;
    default : 
     heading+=180;
     break;
  }
}
void backward() {
  analogWrite(EN1, 255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(EN2, 255);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void Stop() {
  analogWrite(EN1, 255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(EN2, 255);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
void Read(int* S1, int* S2, int* S3, int* S4, int* S5) {
  *S1 = digitalRead (3) ;
  delay ( ADC_stabilize ) ;  //stabilize
  *S1 = digitalRead (3) ;
  delay ( ADC_stabilize ) ;
  *S2 = digitalRead (4) ;
  delay ( ADC_stabilize ) ;
  *S2 = digitalRead (4) ;
  delay (ADC_stabilize) ;
  *S3 = digitalRead (5) ;
  delay (ADC_stabilize) ;
  *S3 = digitalRead (5) ;
  delay (ADC_stabilize) ;
  *S4 = digitalRead (6) ;
  delay (ADC_stabilize) ;
  *S4 = digitalRead (6) ;
  delay (ADC_stabilize) ;
  *S5 = digitalRead(7) ;
  delay (ADC_stabilize);
  *S5 = digitalRead (7) ;
  delay (ADC_stabilize) ;
}
double ReadMPU() {
  //Read(&sensor_1 ,&sensor_2 ,&sensor_3 ,&sensor_4 ,&sensor_5);
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("YAW\t");
    Serial.println(ypr[0] * 180 / M_PI);
#endif
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
  return ypr[0] * 180 / M_PI;
}
void INIT() {
  newtime=millis();
  startangle=ReadMPU();
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN1, OUTPUT); 
  pinMode(EN2, OUTPUT);
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  delay(1000);

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);


  myPID.SetOutputLimits(OUTPUT_LIMIT * -1, OUTPUT_LIMIT);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  startangle=ReadMPU();


}
