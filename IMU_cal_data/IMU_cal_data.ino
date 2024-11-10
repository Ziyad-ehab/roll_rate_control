#include <Wire.h>
#include <Servo.h>
long accelx, accely, accelz;
float gforcex, gforcey, gforcez;
long gyrox, gyroy, gyroz;
float rotX, rotY, rotZ;
float Ax,Ay,Az,Gx,Gy,Gz;
long timer = 0;

void setup()
{
Serial.begin(9600);
Wire.begin();
setupMPU(); 
}

void loop()
{
recordAccelRegisters();
recordGyroRegisters();
processData();
printData();
delay(100);
}

void setupMPU()
{
Wire.beginTransmission(0b1101000);  // I2C addreas of the MPU
Wire.write(0x6B); // accessing the register 6b that controls the power management
Wire.write(0b00000000);  // setting sleep register to 0
Wire.endTransmission();

Wire.beginTransmission(0b1101000);
Wire.write(0x1B);   // accessing the register 1B - gyroscope configuration
Wire.write(0b00000000);  // setting to full scale of 250 degree per second
Wire.endTransmission();

Wire.beginTransmission(0b1101000);
Wire.write(0x1C);  // accesing the register 1C - accelerometer configuration
Wire.write(0b00000000);  // setting the accel to +/- 2g
Wire.endTransmission();
}

void recordAccelRegisters()
{
Wire.beginTransmission(0b1101000);
Wire.write(0x3B);
Wire.endTransmission();
Wire.requestFrom(0b1101000,6);
while(Wire.available() <6);
accelx = Wire.read()<<8|Wire.read();
accely = Wire.read()<<8|Wire.read();
accelz = Wire.read()<<8|Wire.read();
}

void recordGyroRegisters()
{
Wire.beginTransmission(0b1101000);
Wire.write(0x43);
Wire.endTransmission();
Wire.requestFrom(0b1101000,6);
while(Wire.available() <6);
gyrox = Wire.read()<<8|Wire.read();
gyroy = Wire.read()<<8|Wire.read();
gyroz = Wire.read()<<8|Wire.read();
}

void processData(){  // this fucton removes the bias and scale factor that was computed from analyzing the sensor raw data
                                              // My sensor bias = 30.1135          scale factor = 1.63411e+4
  Ax=(accelx-30.1135)/1.63411e+4;
  Ay=(accely-164.6765)/1.6356e+4;
  Az=(accelz-3.3960e+3)/1.64951e+4;
  Gx=(gyrox+72.09)/128.95;
  Gy=(gyroy-100.5)/131.8;
  Gz=(gyroz+86.42)/129.95;
}

void printData()
{
Serial.print("Acc(x,y,z)= ");
Serial.print(Ax);
Serial.print(",");
Serial.print(Ay);
Serial.print(",");
Serial.print(Az);
Serial.print("  ");
Serial.print("Gyro(x,y,z)= ");
Serial.print(Gx);
Serial.print(",");
Serial.print(Gy);
Serial.print(",");
Serial.println(Gz);
}
