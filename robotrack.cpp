#include "mbed.h"
#include "motordriver.h"
#include "QEI.h"
#include "MPU6050.h"
#include "Servo.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

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
//#define SERVO_PIN   p29
#elif defined(TARGET_KL25Z) || defined(TARGET_NUCLEO_F401RE)
#define MOTOR_PWM   D1
#define MOTOR_FWD   D2
#define MOTOR_REV   D3
#define MOTOR_BRAKE 1
#define QEI_PHASEA  D4
#define QEI_PHASEB  D5
#define QEI_PHASEC  D6
#define QEI_PULSE   205
#define QEI_CODE    QEI::X2_ENCODING
#define MPU_SDA     D7
#define MPU_SDL     D8
//#define SERVO_PIN   D9
#else
#error "You need to specify a pin for the sensor"
#endif

ros::NodeHandle nh;
DigitalOut myled(LED1);

void messageCb(const std_msgs::Empty& toggle_msg)
{
    myled = !myled;   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb);

/*
TA7291Pのpin配置
pin1  - GND
pin2  - motor(+ or -)
pin3  - not connect
pin4  - PwmOut motorSpeed(p21);
pin5  - DigitalOut dir1(p19);
pin6  - DigitalOut dir2(p20);
pin7  - Vout
pin8  - external power supply
pin9  - not connect
pin10 - motor(+ or -)
*/

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
