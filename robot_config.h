// robot_config.h
#ifndef _ROBOT_CONFIG_H
#define _ROBOT_CONFIG_H

/*******************************************************************************
* constants
*******************************************************************************/

// L289 pins for left and right motors
#define EN_R  5 // Enable pin
#define IN_R1 6 // IN1 pin
#define IN_R2 7 // IN2 pin

#define EN_L 2 // Enable pin
#define IN_L1 3 // IN1 pin
#define IN_L2 4 // IN2 pin 

// Encoders pins 
#define LEFT_ENCODER_PINA 24
#define LEFT_ENCODER_PINB 22

#define RIGHT_ENCODER_PINA 28
#define RIGHT_ENCODER_PINB 26

// other constants 
#define SAMPLE_TIME 10 // sample time = 10 mili seconds 
#define L_over_2  0.156 // the distance between the tyres/2 
#define CONTROL_ACTION_LIMIT 255 // control action limit 
#define TICKS_PER_METER 3500
#define ENCODER_PUB_DURATION 100 // publish encoders every 100 mili seconds 

/*******************************************************************************
* includes
*******************************************************************************/

#define USE_USBCON // this is required because of a bug in rosserial with Arduino Due boards, remove it if you use Arduino Mega or something else
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include "Arduino.h"
#include<PID_v1.h>
#include "motor.h"

/*******************************************************************************
* functions prototypes
*******************************************************************************/

void setup();
void setupEncoders();
void setupMotors();
void setupPIDs();
void updateLeftEncoder();
void updateRightEncoder();
void cmd_vel_cb(const geometry_msgs::Twist& twist);
void updateMotors();
void publishEncoders();
void loop();

/*******************************************************************************
* globals
*******************************************************************************/

ros::NodeHandle nh;

// Left encoder
volatile long left_encoder_ticks = 0;
long left_encoder_ticks_prev = 0;
//Variable to read current state of left encoder pin
volatile bool left_encoder_Bset;

//Right Encoder
volatile long right_encoder_ticks = 0;
long right_encoder_ticks_prev = 0;
//Variable to read current state of right encoder pin
volatile bool right_encoder_Bset;

// msgs 
std_msgs::Float32 lwheel_ticks_msg;
std_msgs::Float32 rwheel_ticks_msg;

// Publishers 
// encoders publishers
ros::Publisher left_encoder_pub("lwheel", &lwheel_ticks_msg);
ros::Publisher right_encoder_pub("rwheel", &rwheel_ticks_msg);

// Subscribers
ros::Subscriber<geometry_msgs::Twist>sub("cmd_vel", cmd_vel_cb );

// PIDs 
double left_kp = 3.8 , left_ki = 0 , left_kd = 0.0;            
double right_kp = 4 , right_ki = 0 , right_kd = 0.0;

double right_input = 0, right_output = 0, right_setpoint = 0;
PID right_PID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT);  

double left_input = 0, left_output = 0, left_setpoint = 0;
PID left_PID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);  

float desired_linear = 0.0;
float desired_angular = 0.0;
float desired_speed_left = 0.0;
float desired_speed_right = 0.0;

// motors
Motor right_motor(IN_R1, IN_R2, EN_R);
Motor left_motor(IN_L1, IN_L2, EN_L);

#endif // robot_config.h
