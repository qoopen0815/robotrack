#include "mbed.h"
#include "motordriver.h"
#include "QEI.h"
#include "MPU6050.h"
#include "MadgwickFilter.hpp"
#include "Servo.h"
#include "PIDcontroller.h"
#include "Eigen/Dense.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#define M_PI 3.14159265358979323846
#define AXIS 3

#ifdef TARGET_LPC1768
#define MOTOR_PWM   p21
#define MOTOR_FWD   p19
#define MOTOR_REV   p20
#define MOTOR_BRAKE 1
#define QEI_PHASEA  p25
#define QEI_PHASEB  p26
#define QEI_PHASEC  p24
#define QEI_PULSE   205
#define QEI_CODE    QEI::X2_ENCODING
#define MPU_SDA     p28
#define MPU_SDL     p27
#define SERVO_PIN   p29
#else
#error "You need to specify a pin for the sensor"
#endif

//debug matrix
#define PRINT_MATRIX(MATRIX)  \
 std::printf(#MATRIX "_matrix\r\n"); \
 for (int i = 0; i < MATRIX.rows(); ++i) \
 { \
   for (int j = 0; j < MATRIX.cols(); ++j) \
   { \
     std::printf("%.3f\t", MATRIX(i, j)); \
   } \
   std::printf("\r\n");             \
 } \
using namespace Eigen;

//rosserial
ros::NodeHandle nh;
DigitalOut myled(LED1);
void messageCb(const std_msgs::Empty& toggle_msg)
{
    myled = !myled;   // blink the led
}
ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb);

/*------ constructor------*/
//PC-Serial
Serial pc(USBTX, USBRX);
//motordriver
Motor motor1(MOTOR_PWM, MOTOR_FWD, MOTOR_REV, MOTOR_BRAKE);
//encoder
QEI wheel(QEI_PHASEA, QEI_PHASEB, QEI_PHASEC, QEI_PULSE, QEI_CODE);
//IMU
MPU6050 mpu(MPU_SDA, MPU_SDL);
//servo
//Servo servo(SERVO_PIN);
//Timer
Timer t1;  //total time
//EKF > imu
MadgwickFilter attitude(1.0);

void calibrateIMU(float bias[6])
{
  float gyro[AXIS];
  float acc[AXIS];
  
  for(int i=1; i<=50; i++)
    {
      mpu.getGyro(gyro);
      mpu.getAccelero(acc);

      for(int j=0; j<AXIS; j++)
	{
	  //average
	  bias[j] = (i-1)*bias[j]/i + gyro[j]/i;
	  bias[j+3] = (i-1)*bias[j+3]/i + acc[j]/i;
	}
    }
}

int main()
{
  pc.printf("\r\n---------------------start program---------------------");
  pc.printf("\r\n");

  t1.start();
  float gyro[AXIS];
  float acc[AXIS];
  float bias[6];
  float rpyEuler[AXIS];

  calibrateIMU(bias);
  
  while(true)
    {
      mpu.getGyro(gyro);
      mpu.getAccelero(acc);

      for(int i=0; i<AXIS; i++)
	{
	  gyro[i] = gyro[i] - bias[i];
	  acc[i] = acc[i] - bias[i+3];
	}
      
      attitude.MadgwickAHRSupdateIMU(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);
      attitude.getEulerAngle(rpyEuler);

      pc.printf("%f\t%f\t%f\t%f\r\n", bias[0], rpyEuler[0], rpyEuler[1], rpyEuler[2]);
    }
}
