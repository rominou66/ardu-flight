// ardu-flight
// Manual Flight Control with IMU stabilization and training mode by
// rominou66
// Program is designed to run on an Arduino UNO or an Arduino NANO
// GNU GPLv3

#include <LSM9DS1_Types.h>
#include <LSM9DS1_Registers.h>
#include <SparkFunLSM9DS1.h>

#include <Wire.h>
#include <SPI.h>
#include <Servo.h>

#define LSM9DS1_M  0x1E
#define LSM9DS1_AG  0x6B
#define PRINT_CALCULATED
#define PRINT_SPEED 250

// Add the declination for your location
#define DECLINATION 0.16

LSM9DS1 imu;

float pitch;
float roll;
float heading;

unsigned long lastUpdate = 0;
unsigned int interval = 66;

class servoControl
{
    int channelPin;
    int in;
    int out;
    int lastch;
    unsigned long lastUpdate;

    Servo servo;

  public:
    servoControl(int CHPin)
    {
      channelPin = CHPin;
      in = 0;
    }

    void Attach(int pin)
    {
      servo.attach(pin);
    }

    void UpdateP()
    {
      in = pulseIn(channelPin, HIGH);
      in = map(in, 1005, 1990, 150, 50);
      out = in - (int)pitch;

      if (out > 150)
      {
        servo.write(150);
      }
      else if (out < 50)
      {
        servo.write(50);
      }
      else
      {
        servo.write(out);
      }
    }

    void UpdateR()
    {
      in = pulseIn(channelPin, HIGH);
      in = map(in, 1005, 1990, 130, 50);
      out = in - (int)roll;

      if (out > 130)
      {
        servo.write(130);
      }
      else if (out < 50)
      {
        servo.write(50);
      }
      else
      {
        servo.write(out);
      }
    }

    void UpdateM()
    {
      in = pulseIn(channelPin, HIGH);
      in = map(in, 1005, 1990, 50, 140);
      //out = in - (int)heading;

      servo.write(in);
    }

    void UpdateMotor()
    {
      in = pulseIn(channelPin, HIGH);
      in = map(in, 1005, 1990, 50, 80);

      servo.write(in);
    }

    void test(int value)
    {
      servo.write(value);
    }
};

servoControl aileronL(5); //ch4
servoControl aileronR(5); //ch4
servoControl elevator(3); //ch2
servoControl rudder(2);   //ch1
//servoControl motor(4);    //ch3

void imuUpdate()
{
  printAccel();
  printMag();
  printAttitude(imu.ax, imu.ay, imu.az, imu.mx, imu.my);
  //Serial.print(pitch);
  //Serial.print(" , ");
  //Serial.print(roll);
  //Serial.print(" , ");
  //Serial.println(heading);
}

void printGyro()
{
  imu.readGyro();

#ifdef PRINT_CALCULATED
#endif
}

void printAccel()
{
  imu.readAccel();

#ifdef PRINT_CALCULATED
#endif

}

void printMag()
{
  imu.readMag();

#ifdef PRINT_CALCULATED
#endif
}

void printAttitude(float ax, float ay, float az, float mx, float my)
{
  pitch = atan2(ay, az);
  roll = atan2(-ax, sqrt(ay * ay + az * az));

  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;

  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
}

void setup()
{

  Serial.begin(115200);

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  imu.settings.gyro.enabled = true;
  imu.settings.gyro.scale = 245;
  imu.settings.gyro.sampleRate = 1;
  imu.settings.accel.enabled = true;
  imu.settings.mag.enabled = true;

  pinMode(2, INPUT);  //ch1
  pinMode(3, INPUT);  //ch2
  pinMode(4, INPUT);  //ch3
  pinMode(5, INPUT);  //ch4

  aileronL.Attach(6); //ch4
  aileronR.Attach(7); //ch4
  elevator.Attach(8); //ch2
  rudder.Attach(9);   //ch1
  //motor.Attach(10);   //ch3

  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Looping to infinity.");
    while (1);
  }
}

void loop()
{
  if ((millis() - lastUpdate)  > interval)
  {
    lastUpdate = millis();
    //motor.UpdateMotor();
    //motor.test(10);
    aileronL.UpdateR();
    aileronR.UpdateR();
    elevator.UpdateP();
    rudder.UpdateM();

    imuUpdate();
  }
}
