#include "mbed.h"
#include "motordriver.h"
#include "QEI.h"
#include "MPU6050.h"
#include "Servo.h"
#include "PIDcontroller.h"
#include "Eigen/Dense.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#define M_PI 3.14159265358979323846
#define N 3     // mpuDeg_param => 0:Pitch, 1:Roll

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
Timer t1;  //全体経過時間
Timer t2;  //pid用

double e[3] = {};  //e[0]=k-1, e[1]=k, e[2]=sigma

//PID
double input_pid(double e[], double y, double r, double Kp, double Ki, double Kd)
{
    double ts = t2.read();
    t2.reset();
    
    e[0] = e[1];
    e[1] = r - y;
    if (isnan(e[1]) != true) e[2] += e[1];
    
    double u = Kp * e[1] + Ki * ts * e[2] + Kd * (e[1] - e[0]) / ts;
    
    pc.printf("error = %4.2f\t", e[1]);
    
    return -u;
}

void speedControll(void)    //速度制御
{   
    motor1.speed(1.0);
    pc.printf("input = %4.2f\t", motor1.state());    
    pc.printf("RPS = %4.2f\r\n", wheel.getRPS());
} //input_pid(e, wheel.getRPS(), 2.0, 0.3, 0.3, 0)

int main()
{
    // //初期化 initialize
    // float mpuDeg[N] = {};
    // wheel.reset();
    // t1.start();
    // t2.start();
    // //double e[3] = {};    //e[0]=k-1, e[1]=k, e[2]=sigma
    
    // //実行ループ
    // while(true)
    // {
    //     //update
    //     mpu.getAccelero(mpuDeg);
        
    //     //速度制御
    //     speedControll();   
    //     motor2.speed(1.0);
    
    //     //motor1.speed(input_pid(e, mpuDeg[0]*9, 40, 15, 0.15, 250));
    //     //pc.printf("input = %4.2f\t", motor1.state());    
    //     //pc.printf("deg = %4.2f\r\n", mpuDeg[0]*9);
    // }
}
