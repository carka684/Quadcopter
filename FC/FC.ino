#include <PID_v1.h>



// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <MPU6050.h>
#include <I2Cdev.h>
#include <microsmooth.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

int16_t sums[6];
int16_t values[6];

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
//#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
#define OUTPUT_BINARY_ACCELGYRO


#define LED_PIN 13
bool blinkState = false;
uint16_t *history = ms_init(SMA);
bool virgin = true;
uint32_t timer,timer2;
double compAngleX, compAngleY;
double consKp=1, consKi=0.05, consKd=0.25;
double PIDOutputX,PIDOutputY;
double setPointX,setPointY;
int sampleTime;
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
    PIDX.SetSampleTime(sampleTime);
    PIDX.SetMode(AUTOMATIC);
    PIDX.SetOutputLimits(-125,125);
    
    PIDY.SetSampleTime(sampleTime);
    PIDY.SetMode(AUTOMATIC);
    PIDY.SetOutputLimits(-125,125);
    pinMode(LED_PIN, OUTPUT); 

}

void loop() {
  if(millis() - timer2 > sampleTime)
  {
    printGyro();
    readGyro();
    timer2 = millis();
  }
  if(Serial.available())
  {
   char c = Serial.read();
   readSerial(c); 
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
