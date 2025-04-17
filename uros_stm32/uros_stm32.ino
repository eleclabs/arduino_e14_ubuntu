#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int64.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//Variable subsciption
rcl_subscription_t LEDs_subscriber;
std_msgs__msg__Int8 LEDs_msg;

rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;

//variable publisher
rcl_publisher_t left_encoder_publisher;
std_msgs__msg__Int32 left_encoder_msg;

rcl_publisher_t right_encoder_publisher;
std_msgs__msg__Int32 right_encoder_msg;

// Pins LED Motor Encoder 
#define LED_PIN PC13
#define PWMA PA0     //PWMA Motor1
#define IN1_PIN PA1 //left forward
#define IN2_PIN PA2  //left backward
#define LeftEncoder_C1 PA3
#define LeftEncoder_C2 PA4

#define PWMB PA5     //PWMB Motor2
#define IN3_PIN PA7  //right forward
#define IN4_PIN PB0 //right backward
#define RightEncoder_C1 PB1
#define RightEncoder_C2 PB2

// Motor control variables
int motorSpeedLeft = 0;
int motorSpeedRight = 0;

// Encoder variables
long LeftEncoderCount = 0;
long RightEncoderCount = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {error_loop();}}
//--------------------
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

bool micro_ros_init_successful;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

//--------------------
//Function prototypes
void LeftEncoderCallback();
void RightEncoderCallback();

void setMotorSpeed(int speedLeft, int speedRight);

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

int limitToMaxValue(int value, int maxLimit) {
  if (value > maxLimit) {
    return maxLimit;
  } else {
    return value;
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&left_encoder_publisher, &left_encoder_msg, NULL));
    RCSOFTCHECK(rcl_publish(&right_encoder_publisher, &right_encoder_msg, NULL));

    right_encoder_msg.data = RightEncoderCount;
    left_encoder_msg.data = -LeftEncoderCount;
  }
}

void setMotorSpeed(int speedLeft, int speedRight) {
// Set left motor direction and speed  
  if (speedLeft > 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(PWMA, abs(limitToMaxValue(speedLeft, 250)));  
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    analogWrite(PWMA, abs(limitToMaxValue(speedLeft, 250)));  
  }

  // Set right motor direction and speed
  if (speedRight > 0) {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    analogWrite(PWMB, abs(limitToMaxValue(speedRight, 250)));  
  } else {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    analogWrite(PWMB, abs(limitToMaxValue(speedRight, 250)));  
  }

  if (speedLeft == 0 && speedRight == 0) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
    analogWrite(PWMA, abs(limitToMaxValue(speedLeft, 250)));  
    analogWrite(PWMB, abs(limitToMaxValue(speedLeft, 250)));  
  }
}

void LEDs_subscription_callback(const void *msgin) {
  const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8 *)msgin;
  int8_t value = msg->data;
  digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  //LED ขา 2
}

// Twist message callback
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  // Calculate motor speeds based on twist message
  float linear = msg->linear.x;
  float angular = msg->angular.z;

  // Calculate individual motor speeds
  motorSpeedLeft = (int)((linear - angular / 2) * 1000);
  motorSpeedRight = (int)((linear + angular / 2) * 1000);

  if (motorSpeedLeft > 0) {
    motorSpeedLeft = motorSpeedLeft + 40;  //spd=40
  }
  if (motorSpeedLeft < 0) {
    motorSpeedLeft = motorSpeedLeft - 40;
  }

  if (motorSpeedRight > 0) {
    motorSpeedRight = motorSpeedRight + 40;
  }
  if (motorSpeedRight < 0) {
    motorSpeedRight = motorSpeedRight - 40;
  }
  // Set motor speeds
  setMotorSpeed(motorSpeedLeft, motorSpeedRight);
}

//Create entities function
bool create_entities()
{
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "base_control_node", "", &support)); //create node

//create left_encoder publisher
  RCCHECK(rclc_publisher_init_default(
    &left_encoder_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "left_motor_ticks"));

//create right_encoder publisher
  RCCHECK(rclc_publisher_init_default(
    &right_encoder_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "right_motor_ticks"));

//LEDs subscriber
  RCCHECK(rclc_subscription_init_default(
    &LEDs_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "LEDs"));

//Create twist subscriber
  RCCHECK(rclc_subscription_init_default(
    &twist_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

//create timer,
  const unsigned int timer_timeout = 10; //100
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));  

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &LEDs_subscriber, &LEDs_msg, &LEDs_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
}

void destroy_entities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCCHECK(rcl_subscription_fini(&LEDs_subscriber, &node));
  RCCHECK(rcl_subscription_fini(&twist_subscriber, &node));
  RCCHECK(rcl_publisher_fini(&left_encoder_publisher, &node));
  RCCHECK(rcl_publisher_fini(&right_encoder_publisher, &node));

  RCCHECK(rcl_node_fini(&node));
  RCCHECK(rcl_timer_fini(&timer));
  RCCHECK(rclc_executor_fini(&executor));
  RCCHECK(rclc_support_fini(&support));

}

void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(LeftEncoder_C1, INPUT_PULLUP);
  pinMode(LeftEncoder_C2, INPUT_PULLUP);

  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);  
  pinMode(RightEncoder_C1, INPUT_PULLUP);
  pinMode(RightEncoder_C2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LeftEncoder_C1), LeftEncoderCallback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RightEncoder_C1), RightEncoderCallback, CHANGE);

 state = WAITING_AGENT;

 left_encoder_msg.data = 0;
 right_encoder_msg.data = 0;

}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(50, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;); //500
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(20, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;); //200
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); //100
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

void LeftEncoderCallback() {
  if (digitalRead(LeftEncoder_C1) == digitalRead(LeftEncoder_C2)) {
    LeftEncoderCount++;
  } else {
    LeftEncoderCount--;
  }
}

void RightEncoderCallback() {
  if (digitalRead(RightEncoder_C1) == digitalRead(RightEncoder_C2)) {
    RightEncoderCount++;
  } else {
    RightEncoderCount--;
  }
}