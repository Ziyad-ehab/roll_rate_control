#include <Wire.h>
#include <Servo.h>
long accelx, accely, accelz;
float gforcex, gforcey, gforcez;
long gyrox, gyroy, gyroz;
float rotX, rotY, rotZ;
float Ax, Ay, Az, Gx, Gy, Gz, Grx, Gry, Grz, Grxr, Gryr, Grzr;
long delta_t, t_old;
float Gy_old, Gy_int,reqG=0,gyro_error;
float Kp = 2.5, Ki = 0, Kd = 0.7;
long timer = 0;
int servopin1 = 10;                    // Servo pin
int servopin2 = 11;
Servo servo1;                          // Servo object
Servo servo2;
int pos = 0;//position

void setup() 
{
  Serial.begin(38400);
  Wire.begin();
  setupMPU();
  servo1.attach(servopin1);
  servo2.attach(servopin2);    //attach object to pin
   Serial.print("Enter these values $required_g,kp,ki,kd* ");
   delay(3000);
}

void loop() 
{
  bt_getData();
  recordAccelRegisters();
  recordGyroRegisters();
  processData();
  printData();
  rollControl();
  delay(150);
}

void bt_getData() 
{
  if (Serial.available())
    if (Serial.read() == '$')
    { 
      Kp = Serial.readStringUntil(',').toFloat();
      Ki = Serial.readStringUntil(',').toFloat();
      Kd = Serial.readStringUntil('*').toFloat();
    }
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
  Wire.requestFrom(0b1101000, 6);
  while (Wire.available() < 6);
  accelx = Wire.read() << 8 | Wire.read();
  accely = Wire.read() << 8 | Wire.read();
  accelz = Wire.read() << 8 | Wire.read();
}

void recordGyroRegisters()
{
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);
  while (Wire.available() < 6);
  gyrox = Wire.read() << 8 | Wire.read();
  gyroy = Wire.read() << 8 | Wire.read();
  gyroz = Wire.read() << 8 | Wire.read();
}

void processData() {
  Ax = (accelx - 30.1135) / 1.63411e+4;
  Ay = (accely - 164.6765) / 1.6356e+4;
  Az = (accelz - 3.3960e+3) / 1.64951e+4;
  Gx = (gyrox + 72.09) / 128.95;    // in degree and will be changed to radian
  Gy = (gyroy - 100.5) / 131.8;
  Gz = (gyroz + 86.42) / 129.95;
  Grx = Gx * (3.14159 / 180);         //  angle in radian
  Gry = Gy * (3.14159 / 180);
  Grz = Gz * (3.14159 / 180);
  Grxr = Grx * 7;
  Gryr = (Gry * Kp);          // minus the 5 degree offset
  Grzr = Grz * 7;
}

void rollControl() 
{
  delta_t = t_old - millis();
  t_old = millis();
  gyro_error=Gy-reqG;
  Gryr += Kd * ((gyro_error - Gy_old) / delta_t);
  Gy_old = Gy;
  //y-axis / roll
  servo2.write(80 - Gryr);                                //unstable angle
  servo1.write(85 - Gryr);                              //unstable angle
}

void printData()
{
  //  Serial.print("Acc(x,y,z)= ");
  //  Serial.print(Ax);
  //  Serial.print(",");
  //  Serial.print(Ay);
  //  Serial.print(",");
  //  Serial.print(Az);
  //  Serial.print("  ");
  Serial.print("fin_def= ");
  //  Serial.print(Grxr);
  //  Serial.print(",");
  Serial.print(Gryr);
  Serial.print(",PID= ");
  //  Serial.println(Grzr);
  Serial.print(Kp);
  Serial.print(",");
  Serial.print(Ki);
  Serial.print(",");
  Serial.print(Kd);
  Serial.println(",");
}
