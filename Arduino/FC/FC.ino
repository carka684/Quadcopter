#include <PID_v1.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include <microsmooth.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <printf.h>
#include <Wire.h>
#include "Kalman.h" 
#define MAX_SIZE 6
#define CE_PIN 7
#define CSN_PIN 8
#define THIS_NODE 0
#define OTHER_NODE 1
#define TYPE_ANGLE 0 
#define TYPE_CONTROL 1
#define BAUD_RATE 115200
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
RF24 radio(CE_PIN,CSN_PIN);
RF24Network network(radio);

struct payload_t
{
  double dataVector[MAX_SIZE];
};

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

<<<<<<< HEAD
=======
int16_t sums[6];
int16_t values[6];

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

#define LED_PIN 13
bool blinkState = false;
uint16_t *history = ms_init(SMA);
bool virgin = true;
>>>>>>> origin/master
uint32_t timer,gyroTimer,radioTimer;

int numberOfSuccess = 0;
double compAngleRoll, compAnglePitch;
double kalAngleX, kalAngleY;
double consKp=1, consKi=0.05, consKd=0.25;
double PIDOutputX,PIDOutputY;
double setPointX,setPointY;
double angleVec[10]; //X - Y
double PIDVec[10]; // X - Y 
int sampleTime,radioTime,currentSample = 0;
PID PIDX(&compAngleRoll, &PIDOutputX, &setPointX, consKp, consKi, consKd, DIRECT);
PID PIDY(&compAnglePitch, &PIDOutputY, &setPointY, consKp, consKi, consKd, DIRECT);
void setup() {
    pinMode(0, OUTPUT); 
    digitalWrite(0,HIGH);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(BAUD_RATE);
    
    printf_begin();
    SPI.begin();
    radio.begin();
    network.begin(/*channel*/ 90, /*node address*/ this_node);
    
    accelgyro.initialize();
    accelgyro.setXGyroOffset(3);
    accelgyro.setYGyroOffset(16);
    accelgyro.setZGyroOffset(31);
    accelgyro.setXAccelOffset(-554);
    accelgyro.setYAccelOffset(-1144);
    accelgyro.setZAccelOffset(1646);
    
    delay(100);
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    double roll  = atan2(ay, az) * RAD_TO_DEG;
    double pitch = atan(-ax / sqrt((double) ay * ay + (double) az * az)) * RAD_TO_DEG;
    compAngleRoll  = roll;
    compAnglePitch = pitch;
    kalmanX.setAngle(roll); // Set starting angle
    kalmanY.setAngle(pitch);
    setPointX = setPointY = 0;
    sampleTime = 10;
    radioTime = 20;
    PIDX.SetSampleTime(sampleTime);
    PIDX.SetMode(AUTOMATIC);
    PIDX.SetOutputLimits(-125,125);
    
    PIDY.SetSampleTime(sampleTime);
    PIDY.SetMode(AUTOMATIC);
    PIDY.SetOutputLimits(-125,125);
    pinMode(3, OUTPUT); 
    analogWrite(3,80);

}

void loop() {
  network.update();
  if( network.available() )
  {
    readData(); 
  } 
  
  if(millis() - gyroTimer > sampleTime)
  {
    //readGyro();
    readGyroKalmann();
    gyroTimer = millis();
  }
  if(millis() - radioTimer > radioTime)
  {
    sendData();
    radioTimer = millis();
  }

 
  
}
void sendData()
{
  network.update();
  payload_t payload;

  payload.dataVector[0] = angleVec[0];  
  payload.dataVector[1] = angleVec[1]; 
  payload.dataVector[2] = PIDVec[0];
  payload.dataVector[3] = PIDVec[1];
  int type = 0;
  RF24NetworkHeader header(/*to node*/ other_node,type);
  bool ok = network.write(header,&payload,sizeof(payload));
  if(!ok)
  {
//    Serial.print("failed sendning:\t");
//    Serial.println(numberOfSuccess);
//    numberOfSuccess = 0;
  }
  else
  {
    numberOfSuccess++;
  }

}
void readData()
{
   RF24NetworkHeader header;
    payload_t payload;
    network.read(header,&payload,sizeof(payload));

    switch (header.type)
    {
      case TYPE_ANGLE:          
        Serial.print(payload.dataVector[0]);
        Serial.print(" \t");
        Serial.print(payload.dataVector[1]);
        Serial.print(" \t");
        Serial.print(payload.dataVector[2]);
        Serial.print(" \t");
        Serial.print(payload.dataVector[3]);
        Serial.println("");
        break;
      case TYPE_CONTROL:
        Serial.println(payload.dataVector[0]);
        analogWrite(3,payload.dataVector[0]);
        
        break;
    }
}
void readGyro()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  double roll  = atan2((double)  ay,(double)  az) * RAD_TO_DEG;
  double pitch = atan(-ax / sqrt( (double)  ay * ay +  (double)  az * az)) * RAD_TO_DEG;
  double gyroXrate = (double) gx / 131.0; // Convert to deg/s
  double gyroYrate = (double) gy / 131.0; // Convert to deg/s
  

  if ((roll < -90 && compAngleRoll > 90) || (roll > 90 && compAngleRoll < -90)) {
    compAnglePitch = roll;
  }
  if (abs(compAngleRoll) > 90)
  {
    gyroXrate = -gyroXrate; 
  }
  
  compAngleRoll = 0.93 * (compAngleRoll + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAnglePitch = 0.93 * (compAnglePitch + gyroYrate * dt) + 0.07 * pitch;
  PIDX.Compute();
  PIDY.Compute();
  angleVec[0] = compAngleRoll;
  angleVec[1] = compAnglePitch;
  PIDVec[0] = PIDOutputX;
  PIDVec[1] = PIDOutputY;
}
void readGyroKalmann()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  double roll  = atan2((double)  ay,(double)  az) * RAD_TO_DEG;
  double pitch = atan(-ax / sqrt( (double)  ay * ay +  (double)  az * az)) * RAD_TO_DEG;
  double gyroXrate = (double) gx / 131.0; // Convert to deg/s
  double gyroYrate = (double) gy / 131.0; // Convert to deg/s
  
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
  } 
  else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  
  angleVec[0] = kalAngleX;
  angleVec[1] = kalAngleY;
  PIDVec[0] = PIDOutputX;
  PIDVec[1] = PIDOutputY;
}
void printGyro()
{
  Serial.print(compAngleRoll); Serial.print("\t");
  Serial.print(compAnglePitch); Serial.print("\t");
  Serial.print(PIDOutputX); Serial.print("\t");
  Serial.print(PIDOutputY); Serial.print("\t");
  Serial.println();
}

