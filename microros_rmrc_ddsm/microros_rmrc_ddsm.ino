#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>

#include <rmw_microros/rmw_microros.h>

#include <ddsm_ctrl.h>

DDSM_CTRL dc;
Servo myservo;  // create servo object to control a servo
const int servoPin = 12;

#define BATTERY_PIN 36
#define DDSM_RX 18
#define DDSM_TX 19

rcl_subscription_t LEDs_subscriber;
std_msgs__msg__Int8 LEDs_msg;

rcl_subscription_t servo_subscriber;
std_msgs__msg__Int8 servo_msg;

rcl_publisher_t battery_publisher;
std_msgs__msg__Int8 battery_msg;

rcl_publisher_t left_encoder_publisher;
std_msgs__msg__Int32 left_encoder_msg;

rcl_publisher_t right_encoder_publisher;
std_msgs__msg__Int32 right_encoder_msg;

rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LEFT_LED_PIN 17
#define RIGHT_LED_PIN 16

// Encoder Pins
#define LeftEncoder_C1 23
#define LeftEncoder_C2 22
#define RightEncoder_C1 19
#define RightEncoder_C2 18

// Pin Definitions
#define IN1_PIN 33  //left forward
#define IN2_PIN 25  //left backward
#define PWMA 32     //left

#define IN3_PIN 12  //right forward
#define IN4_PIN 14  //right backward
#define PWMB 13     //right

#define LED_PIN 2

// Motor control variables
int motorSpeedLeft = 0;
int motorSpeedRight = 0;

// Encoder variables
int LeftEncoderCount = 0;
int RightEncoderCount = 0;

void LeftEncoderCallback();
void RightEncoderCallback();


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn;  if((temp_rc != RCL_RET_OK)) {} }

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


// Function prototypes
void setMotorSpeed(int speedLeft, int speedRight);
int8_t get_battery_percentage();
int map_to_percentage(int raw_value);


void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
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

    int8_t battery_percentage = get_battery_percentage();
    battery_msg.data = battery_percentage;
    RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));
  }
}


int8_t get_battery_percentage() {
  // Read the voltage from the BATTERY_PIN
  int raw_value = analogRead(BATTERY_PIN);

  // Convert the raw value to battery percentage (0 to 100)
  int battery_percentage = map_to_percentage(raw_value);

  return static_cast<int8_t>(battery_percentage);
}


int map_to_percentage(int raw_value) {
  // Assuming the raw_value represents the battery voltage in the range of 0 to 4095
  // Adjust the following values based on your battery voltage range and voltage divider setup (if any).
  int min_voltage = 2400;  // Minimum voltage reading (corresponding to 0% battery)
  int max_voltage = 3720;  // Maximum voltage reading (corresponding to 100% battery)

  // Map the raw value to the battery percentage
  int battery_percentage = map(raw_value, min_voltage, max_voltage, 0, 100);
  return battery_percentage;
}


void LEDs_subscription_callback(const void *msgin) {
  const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8 *)msgin;
  int8_t value = msg->data;

  switch (value) {
    case 0:
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
      break;
    case 1:
      digitalWrite(LEFT_LED_PIN, HIGH);
      digitalWrite(RIGHT_LED_PIN, LOW);
      break;
    case 2:
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, HIGH);
      break;
    case 3:
      digitalWrite(LEFT_LED_PIN, HIGH);
      digitalWrite(RIGHT_LED_PIN, HIGH);
      break;
    default:
      break;
  }
  //digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);
}


void servo_callback(const void *msgin) {
  const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8 *)msgin;
  int8_t angle = msg->data;
  int servo_position;
  servo_position = limitToMaxValue(angle, 40);
  myservo.write(servo_position);
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
    motorSpeedLeft = motorSpeedLeft ;
  }
  if (motorSpeedLeft < 0) {
    motorSpeedLeft = motorSpeedLeft;
  }

  if (motorSpeedRight > 0) {
    motorSpeedRight = motorSpeedRight;
  }
  if (motorSpeedRight < 0) {
    motorSpeedRight = motorSpeedRight;
  }
  // Set motor speeds
  setMotorSpeed(motorSpeedLeft, motorSpeedRight);
}


void setMotorSpeed(int speedLeft, int speedRight) {
// Set left motor direction and speed

  if (speedLeft > 0) {
    dc.ddsm_ctrl(1, speedLeft , 2); 
    dc.ddsm_ctrl(3, speedLeft , 2);
  } else {
    dc.ddsm_ctrl(1, speedLeft , 2);   //-
    dc.ddsm_ctrl(3, speedLeft , 2);   //-
  }

  // Set right motor direction and speed
  if (speedRight > 0) {
    dc.ddsm_ctrl(2, speedRight*(-1) , 2);  //-
    dc.ddsm_ctrl(4, speedRight*(-1) , 2);  //-
  } else {
    dc.ddsm_ctrl(2, speedRight*(-1) , 2); 
    dc.ddsm_ctrl(4, speedRight*(-1) , 2);
  }
// Set stop motor 
  if (speedLeft == 0 && speedRight == 0) {
    dc.ddsm_ctrl(1, 0, 2); 
    dc.ddsm_ctrl(2, 0, 2);
    dc.ddsm_ctrl(3, 0, 2);
    dc.ddsm_ctrl(4, 0, 2);
  }
}



bool create_entities(){
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "RMRC_2025", "", &support));

  // LED subscriber
  RCCHECK(rclc_subscription_init_default(
    &LEDs_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "LEDs"));

  // servo subscriber
  RCCHECK(rclc_subscription_init_default(
    &servo_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "/servo"));

  // create twist subscriber
  RCCHECK(rclc_subscription_init_default(
    &twist_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create battery publisher
  RCCHECK(rclc_publisher_init_default(
    &battery_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "battery"));

  // create left encoder publisher
  RCCHECK(rclc_publisher_init_default(
    &left_encoder_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "left_motor_ticks"));

 // create right encoder publisher
  RCCHECK(rclc_publisher_init_default(
    &right_encoder_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "right_motor_ticks"));

  // create timer,
  const unsigned int timer_timeout = 1000; //1 sec
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &LEDs_subscriber, &LEDs_msg, &LEDs_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_subscriber, &servo_msg, &servo_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));

  return true;
}


void destroy_entities(){
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&battery_publisher, &node);
  rcl_publisher_fini(&left_encoder_publisher, &node);
  rcl_publisher_fini(&right_encoder_publisher, &node);

  rcl_subscription_fini(&LEDs_subscriber, &node);
  rcl_subscription_fini(&servo_subscriber, &node);
  rcl_subscription_fini(&twist_subscriber, &node);

  rcl_node_fini(&node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);

  micro_ros_init_successful = false;

}



void setup() {
  //Serial.begin(115200);
  //set_microros_wifi_transports("Pixel_5234", "deyz1234", "192.168.148.143", 8888);
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);

  pinMode(LEFT_LED_PIN, OUTPUT);
  digitalWrite(LEFT_LED_PIN, HIGH);

  pinMode(RIGHT_LED_PIN, OUTPUT);
  digitalWrite(RIGHT_LED_PIN, HIGH);
  pinMode(BATTERY_PIN, INPUT);

  pinMode(LeftEncoder_C1, INPUT_PULLUP);
  pinMode(LeftEncoder_C2, INPUT_PULLUP);
  pinMode(RightEncoder_C1, INPUT_PULLUP);
  pinMode(RightEncoder_C2, INPUT_PULLUP);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  digitalWrite(PWMA,HIGH);
  digitalWrite(PWMB,HIGH);

  attachInterrupt(digitalPinToInterrupt(LeftEncoder_C1), LeftEncoderCallback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RightEncoder_C1), RightEncoderCallback, CHANGE);

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);            // standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000);  // attaches the servo on pin 18 to the servo object
  delay(200);


  // ddsm init.
  Serial1.begin(DDSM_BAUDRATE, SERIAL_8N1, DDSM_RX, DDSM_TX);
  dc.pSerial = &Serial1;  

  // config the type of ddsm. 
  dc.set_ddsm_type(115);

  // clear ddsm serial buffer.
  dc.clear_ddsm_buffer();


  left_encoder_msg.data = 0;
  right_encoder_msg.data = 0;

  state = WAITING_AGENT;

}


void loop() {
  delay(100);

    switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
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

