#include <PID_v1.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include <microsmooth.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <printf.h>
#define MAX_SIZE 6
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
/*
 * RADIO
 */
RF24 radio(6,7);
RF24Network network(radio);
// Address of our node
const uint16_t this_node = 1;

// Address of the other node
const uint16_t other_node = 0;
unsigned long last_sent;
unsigned long packets_sent;
struct payload_t
{
  double dataVector[MAX_SIZE];
};

/*
 * IMU
 */
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

int16_t sums[6];
int16_t values[6];


#define LED_PIN 13
bool blinkState = false;
uint16_t *history = ms_init(SMA);
bool virgin = true;
uint32_t timer,gyroTimer,radioTimer;

double compAngleX, compAngleY;
double consKp=1, consKi=0.05, consKd=0.25;
double PIDOutputX,PIDOutputY;
double setPointX,setPointY;
double angleVec[10]; //X - Y
double PIDVec[10]; // X - Y 
int sampleTime,radioTime,currentSample = 0;
PID PIDX(&compAngleX, &PIDOutputX, &setPointX, consKp, consKi, consKd, DIRECT);
PID PIDY(&compAngleY, &PIDOutputY, &setPointY, consKp, consKi, consKd, DIRECT);
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(115200);
    
    printf_begin();
    SPI.begin();
    radio.begin();
    network.begin(/*channel*/ 90, /*node address*/ this_node);
    
    // initialize device
   // Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    //Serial.println("Testing device connections...");
    //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

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
    compAngleX  = roll;
    compAngleY = pitch;
    setPointX = setPointY = 0;
    sampleTime = 5;
    radioTime = 100;
    PIDX.SetSampleTime(sampleTime);
    PIDX.SetMode(AUTOMATIC);
    PIDX.SetOutputLimits(-125,125);
    
    PIDY.SetSampleTime(sampleTime);
    PIDY.SetMode(AUTOMATIC);
    PIDY.SetOutputLimits(-125,125);
    pinMode(LED_PIN, OUTPUT); 

}

void loop() {
  if(millis() - gyroTimer > sampleTime)
  {
   
    readGyro();
    //printGyro();
    sendData();
    gyroTimer = millis();
  }

  
  if(Serial.available())
  {
   char c = Serial.read();
   readSerial(c); 
  }
  
}
void sendData()
{
   network.update();
  Serial.print("Sending...");
  payload_t payload;

  payload.dataVector[0] = angleVec[0];  
  payload.dataVector[1] = angleVec[1]; 
  payload.dataVector[2] = PIDVec[0];
  payload.dataVector[3] = PIDVec[1];
  int type = 0;
  RF24NetworkHeader header(/*to node*/ other_node,type);
  bool ok = network.write(header,&payload,sizeof(payload));
  if (ok)
    Serial.println("ok.");
  else
    Serial.println("failed.");
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
  

  if ((roll < -90 && compAngleX > 90) || (roll > 90 && compAngleX < -90)) {
    compAngleY = roll;
  }
  if (abs(compAngleX) > 90)
  {
    gyroXrate = -gyroXrate; 
  }
  
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  PIDX.Compute();
  PIDY.Compute();
  angleVec[0] = compAngleX;
  angleVec[1] = compAngleY;
  PIDVec[0] = PIDOutputX;
  PIDVec[1] = PIDOutputY;
}
void printGyro()
{
  Serial.print(compAngleX); Serial.print("\t");
  Serial.print(compAngleY); Serial.print("\t");
  Serial.print(PIDOutputX); Serial.print("\t");
  Serial.print(PIDOutputY); Serial.print("\t");
  Serial.println();
}
void readSerial(char c)
{
 switch (c){
  case 'a':
   digitalWrite(LED_PIN,0);
   break;
  case 'b':
   digitalWrite(LED_PIN,1);
   break;
 }
}
