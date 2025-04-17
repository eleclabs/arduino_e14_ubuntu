#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/joint_state.h>

#include <Arduino.h>

// --- กำหนดค่าพินมอเตอร์และเอนโคดเดอร์สำหรับ ESP32 ---
#define MOTOR_LEFT_PWM_PIN 12
#define MOTOR_LEFT_DIR_PIN 16
#define MOTOR_RIGHT_PWM_PIN 13
#define MOTOR_RIGHT_DIR_PIN 17
#define ENCODER_LEFT_A_PIN 2
#define ENCODER_LEFT_B_PIN 3
#define ENCODER_RIGHT_A_PIN 4
#define ENCODER_RIGHT_B_PIN 5

// --- กำหนดค่าทางกายภาพของหุ่นยนต์ ---
#define WHEEL_DIAMETER 0.065    // เส้นผ่านศูนย์กลางล้อ (เมตร)
#define WHEEL_BASE 0.20         // ระยะห่างระหว่างล้อ (เมตร)
#define ENCODER_CPR 2000.0      // จำนวนพัลส์ต่อรอบของเอนโคดเดอร์

// --- กำหนดค่า PID ---
#define KP 1.0
#define KI 0.1
#define KD 0.01

// --- ความถี่ในการวนลูปควบคุม (Hz) ---
#define CONTROL_LOOP_FREQUENCY 100
#define CONTROL_LOOP_PERIOD (1000.0 / CONTROL_LOOP_FREQUENCY) // ms

// --- โครงสร้างข้อมูลสำหรับ PID Controller ---
typedef struct {
    float setpoint;
    float proportional;
    float integral;
    float derivative;
    float last_error;
    float output;
    float kp;
    float ki;
    float kd;
} PIDController;

// --- ตัวแปร Global ---
rcl_node_t node;
rcl_publisher_t odom_publisher;
rcl_publisher_t joint_state_publisher;
rcl_subscription_t cmd_vel_subscriber;
rclc_executor_t executor;
rclc_support_t support;

geometry_msgs__msg__Twist cmd_vel_msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__JointState joint_state_msg;

PIDController left_pid_controller;
PIDController right_pid_controller;

volatile long long left_encoder_count = 0;
volatile long long right_encoder_count = 0;
long long last_left_encoder_count = 0;
long long last_right_encoder_count = 0;
unsigned long last_time = 0;

float target_left_vel = 0.0;  // เป้าหมายความเร็วเชิงเส้นของล้อซ้าย (m/s)
float target_right_vel = 0.0; // เป้าหมายความเร็วเชิงเส้นของล้อขวา (m/s)

// --- ฟังก์ชันสำหรับอ่านค่าเอนโคดเดอร์ (ใช้ Interrupt) ---
void IRAM_ATTR left_encoder_A_changed() {
  if (digitalRead(ENCODER_LEFT_A_PIN) == digitalRead(ENCODER_LEFT_B_PIN)) {
    left_encoder_count++;
  } else {
    left_encoder_count--;
  }
}

void IRAM_ATTR left_encoder_B_changed() {
  if (digitalRead(ENCODER_LEFT_A_PIN) != digitalRead(ENCODER_LEFT_B_PIN)) {
    left_encoder_count++;
  } else {
    left_encoder_count--;
  }
}

void IRAM_ATTR right_encoder_A_changed() {
  if (digitalRead(ENCODER_RIGHT_A_PIN) == digitalRead(ENCODER_RIGHT_B_PIN)) {
    right_encoder_count++;
  } else {
    right_encoder_count--;
  }
}

void IRAM_ATTR right_encoder_B_changed() {
  if (digitalRead(ENCODER_RIGHT_A_PIN) != digitalRead(ENCODER_RIGHT_B_PIN)) {
    right_encoder_count++;
  } else {
    right_encoder_count--;
  }
}

// --- ฟังก์ชันสำหรับตั้งค่าความเร็วมอเตอร์ ---
void set_motor_speed(int pwm_pin, int dir_pin, float speed) {
  int pwm_value = (int)(abs(speed) * 255.0); // แปลง speed (-1 ถึง 1) เป็น PWM (0 ถึง 255)
  if (speed > 0) {
    digitalWrite(dir_pin, HIGH); // หมุนไปข้างหน้า (ต้องตรวจสอบทิศทางมอเตอร์)
  } else if (speed < 0) {
    digitalWrite(dir_pin, LOW);  // หมุนถอยหลัง (ต้องตรวจสอบทิศทางมอเตอร์)
  } else {
    pwm_value = 0;
  }
  analogWrite(pwm_pin, pwm_value);
}

// --- ฟังก์ชัน PID Controller ที่เขียนเอง ---
float compute_pid(PIDController *pid, float error, float delta_time) {
    pid->proportional = error;
    pid->integral += error * delta_time;
    pid->derivative = (error - pid->last_error) / delta_time;
    pid->last_error = error;
    pid->output = (pid->kp * pid->proportional) + (pid->ki * pid->integral) + (pid->kd * pid->derivative);
    return pid->output;
}

// --- Callback ฟังก์ชันเมื่อได้รับข้อความ /cmd_vel ---
void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  cmd_vel_msg = *msg;
  Serial.printf("Received cmd_vel: linear.x = %f, angular.z = %f\n", msg->linear.x, msg->angular.z);

  // คำนวณความเร็วเชิงเส้นเป้าหมายของล้อแต่ละข้าง (m/s)
  target_left_vel = msg->linear.x - (msg->angular.z * WHEEL_BASE / 2.0);
  target_right_vel = msg->linear.x + (msg->angular.z * WHEEL_BASE / 2.0);

  // แปลงความเร็วเชิงเส้นเป้าหมายเป็นความเร็วเชิงมุมเป้าหมาย (rad/s)
  left_pid_controller.setpoint = target_left_vel / (WHEEL_DIAMETER / 2.0);
  right_pid_controller.setpoint = target_right_vel / (WHEEL_DIAMETER / 2.0);
}

// --- ฟังก์ชันสำหรับคำนวณ Odometry และควบคุมมอเตอร์ ---
void control_loop() {
  unsigned long current_time = millis();
  float delta_time = (float)(current_time - last_time) / 1000.0; // แปลงเป็นวินาที
  last_time = current_time;

  long long current_left_encoder_count = left_encoder_count;
  long long current_right_encoder_count = right_encoder_count;

  long long delta_left_ticks = current_left_encoder_count - last_left_encoder_count;
  long long delta_right_ticks = current_right_encoder_count - last_right_encoder_count;

  last_left_encoder_count = current_left_encoder_count;
  last_right_encoder_count = current_right_encoder_count;

  // คำนวณความเร็วเชิงมุมจริงของล้อ (rad/s)
  float left_wheel_vel = (2.0 * PI / ENCODER_CPR) * delta_left_ticks / delta_time;
  float right_wheel_vel = (2.0 * PI / ENCODER_CPR) * delta_right_ticks / delta_time;

  // --- PID Control ---
  float left_error = left_pid_controller.setpoint - left_wheel_vel;
  float right_error = right_pid_controller.setpoint - right_wheel_vel;

  float left_motor_output = compute_pid(&left_pid_controller, left_error, delta_time);
  float right_motor_output = compute_pid(&right_pid_controller, right_error, delta_time);

  // จำกัดค่า Output ให้อยู่ในช่วงที่เหมาะสม (-1 ถึง 1 โดยประมาณ)
  left_motor_output = constrain(left_motor_output, -1.0, 1.0);
  right_motor_output = constrain(right_motor_output, -1.0, 1.0);

  // ตั้งค่าความเร็วมอเตอร์
  set_motor_speed(MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_DIR_PIN, left_motor_output);
  set_motor_speed(MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_DIR_PIN, right_motor_output);

  // --- คำนวณ Odometry ---
  // คำนวณระยะทางที่ล้อแต่ละข้างเคลื่อนที่ (เมตร)
  float left_wheel_distance = left_wheel_vel * (WHEEL_DIAMETER / 2.0) * delta_time;
  float right_wheel_distance = right_wheel_vel * (WHEEL_DIAMETER / 2.0) * delta_time;

  // คำนวณความเร็วเชิงเส้นและความเร็วเชิงมุมของหุ่นยนต์
  float linear_velocity = (left_wheel_distance + right_wheel_distance) / (2.0 * delta_time);
  float angular_velocity = (right_wheel_distance - left_wheel_distance) / (WHEEL_BASE * delta_time);

  // อัปเดต Odometry Message
  odom_msg.header.stamp.sec = current_time / 1000;
  odom_msg.header.stamp.nanosec = (current_time % 1000) * 1000000;
  odom_msg.pose.pose.position.x += linear_velocity * cos(odom_msg.pose.pose.orientation.z) * delta_time;
  odom_msg.pose.pose.position.y += linear_velocity * sin(odom_msg.pose.pose.orientation.z) * delta_time;
  odom_msg.pose.pose.orientation.z += angular_velocity * delta_time;

  odom_msg.twist.twist.linear.x = linear_velocity;
  odom_msg.twist.twist.angular.z = angular_velocity;

  // Publish Odometry Message
  rcl_publish(&odom_publisher, &odom_msg, NULL);

  // --- อัปเดต Joint State Message ---
  joint_state_msg.header.stamp.sec = current_time / 1000;
  joint_state_msg.header.stamp.nanosec = (current_time % 1000) * 1000000;
  joint_state_msg.velocity.data[0] = left_wheel_vel;  // ความเร็วเชิงมุมล้อซ้าย (rad/s)
  joint_state_msg.velocity.data[1] = right_wheel_vel; // ความเร็วเชิงมุมล้อขวา (rad/s)
  rcl_publish(&joint_state_publisher, &joint_state_msg, NULL);
}

void setup() {
  Serial.begin(115200);
  //set_microros_serial_transports(Serial);
  //set_microros_serial_transports();
  set_microros_transports();
  delay(2000);

  pinMode(MOTOR_LEFT_PWM_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR_PIN, OUTPUT);

  pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B_PIN, INPUT_PULLUP);

  // ติดตั้ง Interrupt สำหรับ Encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), left_encoder_A_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B_PIN), left_encoder_B_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A_PIN), right_encoder_A_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B_PIN), right_encoder_B_changed, CHANGE);

  // --- Initial ค่า PID Controller ---
  left_pid_controller.setpoint = 0.0;
  left_pid_controller.proportional = 0.0;
  left_pid_controller.integral = 0.0;
  left_pid_controller.derivative = 0.0;
  left_pid_controller.last_error = 0.0;
  left_pid_controller.output = 0.0;
  left_pid_controller.kp = KP;
  left_pid_controller.ki = KI;
  left_pid_controller.kd = KD;

  right_pid_controller.setpoint = 0.0;
  right_pid_controller.proportional = 0.0;
  right_pid_controller.integral = 0.0;
  right_pid_controller.derivative = 0.0;
  right_pid_controller.last_error = 0.0;
  right_pid_controller.output = 0.0;
  right_pid_controller.kp = KP;
  right_pid_controller.ki = KI;
  right_pid_controller.kd = KD;

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  // --- สร้าง Node ---
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_ret_t ret = rclc_node_init_default(&node, "differential_drive_controller", "", &support);

/*
  if (ret != RCL_OK) {
    Serial.println("Error creating node");
    return;
  }
*/

  // --- สร้าง Publisher สำหรับ Odometry ---
  rcl_publisher_options_t odom_pub_ops = rcl_publisher_get_default_options();
  ret = rclc_publisher_init_default(
      &odom_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "odom");

/*
  if (ret != RCL_OK) {
    Serial.println("Error creating odom publisher");
    return;
  }
*/

  // --- สร้าง Publisher สำหรับ Joint State ---
  rcl_publisher_options_t joint_state_pub_ops = rcl_publisher_get_default_options();
  ret = rclc_publisher_init_default(
      &joint_state_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "joint_states");

/*
  if (ret != RCL_OK) {
    Serial.println("Error creating joint_state publisher");
    return;
  }
*/

  // --- สร้าง Subscriber สำหรับ /cmd_vel ---
  rcl_subscription_options_t cmd_vel_sub_ops = rcl_subscription_get_default_options();
  ret = rclc_subscription_init_default(
      &cmd_vel_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel");

/*      
  if (ret != RCL_OK) {
    Serial.println("Error creating cmd_vel subscriber");
    return;
  }
*/

  // --- เตรียม Odometry Message ---
  const char * frame_id = "odom";
  const char * child_frame_id = "base_link";
  odom_msg.header.frame_id.data = (char*)malloc(strlen(frame_id) + 1);
  strcpy(odom_msg.header.frame_id.data, frame_id);
  odom_msg.header.frame_id.size = strlen(frame_id);
  odom_msg.header.frame_id.capacity = strlen(frame_id) + 1;
  odom_msg.child_frame_id.data = (char*)malloc(strlen(child_frame_id) + 1);
  strcpy(odom_msg.child_frame_id.data, child_frame_id);
  odom_msg.child_frame_id.size = strlen(child_frame_id);
  odom_msg.child_frame_id.capacity = strlen(child_frame_id) + 1;
  odom_msg.pose.pose.position.x = 0.0;
  odom_msg.pose.pose.position.y = 0.0;

  odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    // --- เตรียม Joint State Message ---
    const char * joint_names[2] = {"left_wheel_joint", "right_wheel_joint"};
    //joint_state_msg.name.data = (char**)malloc(sizeof(char*) * 2);
    //joint_state_msg.name.data[0] = (char*)malloc(strlen(joint_names[0]) + 1);
    //strcpy(joint_state_msg.name.data[0], joint_names[0]);
    //joint_state_msg.name.data[1] = (char*)malloc(strlen(joint_names[1]) + 1);
    //strcpy(joint_state_msg.name.data[1], joint_names[1]);
    joint_state_msg.name.size = 2;
    joint_state_msg.name.capacity = 2;

    joint_state_msg.position.data = (double*)malloc(sizeof(double) * 2);
    joint_state_msg.position.size = 2;
    joint_state_msg.position.capacity = 2;
    joint_state_msg.position.data[0] = 0.0;
    joint_state_msg.position.data[1] = 0.0;

    joint_state_msg.velocity.data = (double*)malloc(sizeof(double) * 2);
    joint_state_msg.velocity.size = 2;
    joint_state_msg.velocity.capacity = 2;
    joint_state_msg.velocity.data[0] = 0.0;
    joint_state_msg.velocity.data[1] = 0.0;

    joint_state_msg.effort.data = (double*)malloc(sizeof(double) * 2);
    joint_state_msg.effort.size = 2;
    joint_state_msg.effort.capacity = 2;
    joint_state_msg.effort.data[0] = 0.0;
    joint_state_msg.effort.data[1] = 0.0;

    // --- สร้าง Executor ---
    rclc_executor_init(&executor, &support.context, 1, &allocator); // เพิ่มขนาด Executor เป็น 1 สำหรับ Subscription เดียว
    rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); // ประมวลผล callbacks ทุก 100 ms
  control_loop(); // เรียกใช้ loop ควบคุมมอเตอร์และ publish odometry
  delay(CONTROL_LOOP_PERIOD);
}