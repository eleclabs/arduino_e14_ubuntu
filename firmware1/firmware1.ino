#include <micro_ros_arduino.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include <rmw_microros/rmw_microros.h>

//================pin setup==============
#define LED_PIN 2
#define RCCHECK(fn) {  rcl_ret_t temp_rc = fn;  if ((temp_rc != RCL_RET_OK)) { return false; } }
#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t init = -1; if (init == -1) { init = uxr_millis(); } if (uxr_millis() - init > MS) { X; init = uxr_millis(); } } while(0)  

//================PWM SETUP==============
const int freq = 50;
const int l_pwm_channel = 1;
const int r_pwm_channel = 2;
const int resolution = 8;

//=======================================
#define l_dir1_pin 33
#define l_dir2_pin 25
#define l_pwm_pin 32

#define r_dir1_pin 12
#define r_dir2_pin 14
#define r_pwm_pin 13

#define l_encoder_a 34
#define l_encoder_b 35

#define r_encoder_a 26
#define r_encoder_b 27

////////////////Variables/////////////////////
rcl_allocator_t allocator;
rclc_support_t support;
rcl_timer_t timer;
rcl_node_t node;
rclc_executor_t executor;

rcl_publisher_t left_tick_pub;
std_msgs__msg__Int32 left_tick;

rcl_publisher_t right_tick_pub;
std_msgs__msg__Int32 right_tick;

rcl_publisher_t feedback_vel_pub;
geometry_msgs__msg__Twist feedback_vel_msg;

rcl_subscription_t wr_command_sub;
std_msgs__msg__Float32 wr_command;

rcl_subscription_t wl_command_sub;
std_msgs__msg__Float32 wl_command;

rcl_subscription_t pid_gain_sub;
geometry_msgs__msg__Vector3 pid_gain;

bool micro_ros_init_successful;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

unsigned int SPIN_FREQ = 20;
float windup_guard = 20;

// PID control parameters
float kp = 0.02;
float ki = 0.004;
float kd = 0.0;

//Motor pulse parameters
float PPR = 550;  //PPR = 700 (From manufacturer)
long pl, pr;

//Left Wheel control parameter
unsigned long curTimeL, prevTimeL, dtL;
long curTickL, prevTickL, diffTickL;
double errL, prev_errL, sumErrL, dErrL, setRPML;
double control_outL;
double measuredRPML;
double desiredRPMR, desiredRPML;

/*Right Wheel Control*/
unsigned long curTimeR, prevTimeR, dtR;
long curTickR, prevTickR, diffTickR;
double errR, prev_errR, sumErrR, dErrR, setRPMR;
double control_outR;
double measuredRPMR;

//Find max value function
float max(float num1, float num2) {
  return (num1 > num2) ? num1 : num2;
}

//Find min value function
float min(float num1, float num2) {
  return (num1 > num2) ? num2 : num1;
}


//Left Motor Controller Function
void computePIDL(double control_cmd, long inTick) {
  int dirs = 1;  //1 for forward direction
  //int dirm = 1;  //1 for forward direction

  curTickL = inTick;
  curTimeL = millis();

  setRPML = control_cmd;
  dirs = control_cmd / abs(control_cmd);

  diffTickL = curTickL - prevTickL;
  dtL = (curTimeL - prevTimeL);

  measuredRPML = ((diffTickL / PPR) / (dtL * 0.001)) * 60;

  errL = abs(setRPML) - abs(measuredRPML);

  sumErrL += errL * dtL;

  dErrL = (errL - prev_errL) / dtL;

  control_outL = kp * errL + ki * sumErrL + kd * dErrL;

  if (control_outL < 0) {
    control_outL = 0;
  }
  prev_errL = errL;
  prevTickL = curTickL;
  prevTimeL = curTimeL;

  if (dirs > 0.5) {
    digitalWrite(l_dir1_pin, HIGH);
    digitalWrite(l_dir2_pin, LOW);
    //ledcWrite(l_pwm_channel, control_outL);
    analogWrite(l_pwm_pin, control_outL);
  } else if (dirs < -0.5) {
    digitalWrite(l_dir1_pin, LOW);
    digitalWrite(l_dir2_pin, HIGH);
    //ledcWrite(l_pwm_channel, control_outL);
    analogWrite(l_pwm_pin, control_outL);
  } else {
    digitalWrite(l_dir1_pin, LOW);
    digitalWrite(l_dir2_pin, LOW);
    //ledcWrite(l_pwm_channel, 0);
    analogWrite(l_pwm_pin, control_outL);
  }
}


//Right Motor Controller Function
void computePIDR(double control_cmd, long inTick) {
  int dirs = 1;
  //int dirm = 1;

  curTickR = inTick;
  curTimeR = millis();

  setRPMR = control_cmd;
  dirs = control_cmd / abs(control_cmd);

  diffTickR = curTickR - prevTickR;
  dtR = (curTimeR - prevTimeR);

  measuredRPMR = ((diffTickR / PPR) / (dtR * 0.001)) * 60;

  errR = abs(setRPMR) - abs(measuredRPMR);

  sumErrR += errR * dtR;

  dErrR = (errR - prev_errR) / dtR;

  control_outR = kp * errR + ki * sumErrR + kd * dErrR;

  prev_errR = errR;

  prevTickR = curTickR;
  prevTimeR = curTimeR;

  if (dirs > 0.5) {
    digitalWrite(r_dir1_pin, HIGH);
    digitalWrite(r_dir2_pin, LOW);
    //ledcWrite(r_pwm_channel, control_outR);
    analogWrite(r_pwm_pin, control_outR);
  } else if (dirs < -0.5) {
    digitalWrite(r_dir1_pin, LOW);
    digitalWrite(r_dir2_pin, HIGH);
    //ledcWrite(r_pwm_channel, control_outR);
    analogWrite(r_pwm_pin, control_outR);
  } else {
    digitalWrite(r_dir1_pin, LOW);
    digitalWrite(r_dir2_pin, LOW);
    //ledcWrite(r_pwm_channel, 0);
    analogWrite(r_pwm_pin, control_outR);
  }
}


void pid_gain_callback(const void *msgin) {
  const geometry_msgs__msg__Vector3 *gain_value = (const geometry_msgs__msg__Vector3 *)msgin;

  //Uncomment for tuning
  /*
    kp = gain_value->x;
    ki = gain_value->y;
    kd = gain_value->z;
  */
}


//Right wheel command callback function
void rightwheel_callback(const void *msgin) {
  const std_msgs__msg__Float32 *power = (const std_msgs__msg__Float32 *)msgin;
  desiredRPMR = power->data;
}


//Left Wheel Command Callback Function
void leftwheel_callback(const void *msgin) {
  const std_msgs__msg__Float32 *power = (const std_msgs__msg__Float32 *)msgin;
  desiredRPML = power->data;
}


//Timer callback function
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    rcl_publish(&feedback_vel_pub, &feedback_vel_msg, NULL);
    RCLC_UNUSED(timer);
  }
}


//Create entities function
bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "base_control_node", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &pid_gain_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "pid_gain"));

  RCCHECK(rclc_subscription_init_default(
    &wl_command_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "rpm_left"));

  RCCHECK(rclc_subscription_init_default(
    &wr_command_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "rpm_right"));

  RCCHECK(rclc_publisher_init_default(
    &left_tick_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "left_tick"));

  RCCHECK(rclc_publisher_init_default(
    &right_tick_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "right_tick"));

  RCCHECK(rclc_publisher_init_default(
    &feedback_vel_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "feedback_vel"));

  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(50),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &pid_gain_sub, &pid_gain, &pid_gain_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &wr_command_sub, &wr_command, &rightwheel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &wl_command_sub, &wl_command, &leftwheel_callback, ON_NEW_DATA));
  
  return true;
}


void destroy_entities() {
  rcl_publisher_fini(&left_tick_pub, &node);
  rcl_publisher_fini(&right_tick_pub, &node);
  rcl_publisher_fini(&feedback_vel_pub, &node);
  rcl_subscription_fini(&wr_command_sub, &node);
  rcl_subscription_fini(&wl_command_sub, &node);
  rcl_subscription_fini(&pid_gain_sub, &node);

  rcl_node_fini(&node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);

  micro_ros_init_successful = false;
}


void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  //PWM pin setup
  pinMode(r_dir1_pin, OUTPUT);
  pinMode(r_dir2_pin, OUTPUT);

  pinMode(l_dir1_pin, OUTPUT);
  pinMode(l_dir2_pin, OUTPUT);

  ledcSetup(l_pwm_channel, freq, resolution);
  ledcSetup(r_pwm_channel, freq, resolution);
  ledcAttachPin(l_pwm_pin, l_pwm_channel);
  ledcAttachPin(r_pwm_pin, r_pwm_channel);

  pinMode(l_encoder_a, INPUT_PULLUP);
  pinMode(l_encoder_b, INPUT_PULLUP);
  pinMode(r_encoder_a, INPUT_PULLUP);
  pinMode(r_encoder_b, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(l_encoder_a), l_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(r_encoder_a), r_encoder_isr, CHANGE);

  state = WAITING_AGENT;
}


long curRPM_time;
long prevRPM_time;
long curPl;
long curPr;
long prevPl;
long prevPr;
long diffRPM_time;
float RPM_l, RPM_r;


void counter_RPM(long inPl, long inPr) {
  curPl = inPl;
  curPr = inPr;
  curRPM_time = millis();
  diffRPM_time = curRPM_time - prevRPM_time;
  RPM_l = (((curPl - prevPl) / PPR) / (diffRPM_time * 0.001)) * 60;
  RPM_r = (((curPr - prevPr) / PPR) / (diffRPM_time * 0.001)) * 60;


  prevPl = curPl;
  prevPr = curPr;
  prevRPM_time = curRPM_time;
  feedback_vel_msg.linear.x = RPM_l;  // RPM_l;
  feedback_vel_msg.linear.y = RPM_r;
  feedback_vel_msg.angular.x = control_outL;
  feedback_vel_msg.angular.y = control_outR;
  feedback_vel_msg.angular.z = ki;
}


void l_encoder_isr() {
  int A = digitalRead(l_encoder_a);
  int B = digitalRead(l_encoder_b);

  if ((A == HIGH) != (B == LOW)) {
    pl--;
  } else {
    pl++;
  }
}


void r_encoder_isr() {
  int A = digitalRead(r_encoder_a);
  int B = digitalRead(r_encoder_b);

  if ((A == HIGH) != (B == LOW)) {
    pr--;
  } else {
    pr++;
  }
}


void counter_tick() {
  left_tick.data = pl;
  right_tick.data = pr;
  counter_RPM(pl, pr);
  rcl_publish(&left_tick_pub, &left_tick, NULL);
  rcl_publish(&right_tick_pub, &right_tick, NULL);
}


void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT; );
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED; );
      if (state == AGENT_CONNECTED) {
        computePIDR(desiredRPMR, pr);
        computePIDL(desiredRPML, pl);
        counter_tick();
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
}


