#include "mbed.h"
#include "motordriver.h"
#include "QEI.h"
#include "MPU6050.h"
#include <ros.h>
#include <string>
#include <std_msgs/Empty.h>

// #define N 3     // mpuDeg_param => 0:Pitch, 1:Roll

// ros::NodeHandle nh;
// ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb);

// DigitalOut myled(LED1);

// void messageCb(const std_msgs::Empty& toggle_msg)
// {
//     myled = !myled;   // blink the led
// }

// /*
// TA7291Pのpin配置
// pin1  - GND
// pin2  - motor(+ or -)
// pin3  - not connect
// pin4  - PwmOut motorSpeed(p21);
// pin5  - DigitalOut dir1(p19);
// pin6  - DigitalOut dir2(p20);
// pin7  - Vout
// pin8  - external power supply
// pin9  - not connect
// pin10 - motor(+ or -)
// */

// //PCシリアル通信
// Serial pc(USBTX, USBRX);

// //モータードライバ
// Motor motor1(p21, p19, p20, 1); // pwm, fwd, rev, can brake ==> wheel
// Motor motor2(p22, p17, p18, 1); // pwm, fwd, rev, can brake ==> steering

// //エンコーダ
// QEI wheel(p25, p26, p24, 205, QEI::X2_ENCODING);  //QEI name(phase A, phase B, phase Z, pulse, CODE)

// //ジャイロ
// MPU6050 mpu(p28, p27);      //SDA, SDL

// //Timer
// Timer t1; //全体経過時間
// Timer t2; //pid用

// double e[3] = {};    //e[0]=k-1, e[1]=k, e[2]=sigma

// //PID制御出力
// double input_pid(double e[], double y, double r, double Kp, double Ki, double Kd)
// {
//     double ts = t2.read();
//     t2.reset();
    
//     e[0] = e[1];
//     e[1] = r - y;
//     if (isnan(e[1]) != true) e[2] += e[1];
    
//     double u = Kp * e[1] + Ki * ts * e[2] + Kd * (e[1] - e[0]) / ts;
    
//     pc.printf("error = %4.2f\t", e[1]);
    
//     return -u;
// }

// void speedControll(void)    //速度制御
// {   
//     motor1.speed(1.0);
//     pc.printf("input = %4.2f\t", motor1.state());    
//     pc.printf("RPS = %4.2f\r\n", wheel.getRPS());
// } //input_pid(e, wheel.getRPS(), 2.0, 0.3, 0.3, 0)

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
