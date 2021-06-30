#include "robot_config.h"

/*******************************************************************************
* setups
*******************************************************************************/
void setup()
{
  //Initialise node and subscribe to necessary topics
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(left_encoder_pub);
  nh.advertise(right_encoder_pub);
  nh.subscribe(sub);
  Serial.begin(57600);
  setupEncoders();
  setupMotors();
  setupPIDs();
}

void setupMotors() {
  right_motor.init();
  left_motor.init(); 
}

void setupEncoders()
{
  // Quadrature encoders
  // Left encoder
  pinMode(LEFT_ENCODER_PINA, INPUT); // sets pin A pullup
  pinMode(LEFT_ENCODER_PINB, INPUT); // sets pin B pullup
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PINA), updateLeftEncoder, RISING);

  // Right encoder
  pinMode(RIGHT_ENCODER_PINA, INPUT); // sets pin A pullup
  pinMode(RIGHT_ENCODER_PINB, INPUT); // sets pin B pullup
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PINA), updateRightEncoder, RISING);
}

void setupPIDs(){
  right_PID.SetMode(AUTOMATIC);
  right_PID.SetSampleTime(1);
  right_PID.SetOutputLimits(-CONTROL_ACTION_LIMIT, CONTROL_ACTION_LIMIT);

  left_PID.SetMode(AUTOMATIC);
  left_PID.SetSampleTime(1);
  left_PID.SetOutputLimits(-CONTROL_ACTION_LIMIT, CONTROL_ACTION_LIMIT);
}


/*******************************************************************************
* subscribers callbacks
*******************************************************************************/

void cmd_vel_cb(const geometry_msgs::Twist& twist){
  desired_linear = twist.linear.x;
  desired_angular = twist.angular.z;
}


/*******************************************************************************
* control loop 
*******************************************************************************/

void loop()
{
  
  updateMotors();    
  publishEncoders();
}


/*******************************************************************************
* encoders interrupts
*******************************************************************************/

void updateLeftEncoder()
{
  left_encoder_Bset = digitalRead(LEFT_ENCODER_PINB); // read the input pin
  left_encoder_ticks += left_encoder_Bset ? -1 : +1;
}
void updateRightEncoder()
{
  right_encoder_Bset = digitalRead(RIGHT_ENCODER_PINB); // read the input pin
  right_encoder_ticks += right_encoder_Bset ? +1 : -1;
}

/*******************************************************************************
* motor control
*******************************************************************************/

void updateMotors(){
    unsigned long cur_t = millis();
    static unsigned long prev_t= 0;
    unsigned long delta_t = 0;
    delta_t = cur_t - prev_t;
    if (delta_t < SAMPLE_TIME)
      return;
    prev_t = cur_t;
    
    desired_speed_left = desired_linear - (desired_angular*L_over_2);
    desired_speed_right = desired_linear + (desired_angular*L_over_2);
    
    left_input = left_encoder_ticks - left_encoder_ticks_prev; 
    right_input = right_encoder_ticks - right_encoder_ticks_prev;

    left_encoder_ticks_prev = left_encoder_ticks;
    right_encoder_ticks_prev = right_encoder_ticks;
  
    left_setpoint = desired_speed_left*TICKS_PER_METER/delta_t;  //Setting required speed as a mul/frac of 1 m/s
    right_setpoint = desired_speed_right*TICKS_PER_METER/delta_t;
    
    left_PID.Compute();
    right_PID.Compute();
    
    left_motor.writeVelocity(left_output);
    right_motor.writeVelocity(right_output);
}

/*******************************************************************************
* publish encoders
*******************************************************************************/

void publishEncoders(){
    unsigned long cur_t = millis();
    unsigned long delta_t = 0;
    static unsigned long prev_encoder_t = 0;
    delta_t = cur_t - prev_encoder_t;
    if (delta_t < ENCODER_PUB_DURATION)
      return;
    prev_encoder_t = cur_t;
    
    // publishing the encoders ticks 
    lwheel_ticks_msg.data = left_encoder_ticks;
    rwheel_ticks_msg.data = right_encoder_ticks;
    left_encoder_pub.publish(&lwheel_ticks_msg);
    right_encoder_pub.publish(&rwheel_ticks_msg);
    nh.spinOnce();
}
