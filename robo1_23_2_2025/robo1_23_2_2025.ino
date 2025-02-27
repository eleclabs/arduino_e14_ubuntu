#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <tf2_msgs/msg/tf_message.h>
#include <std_msgs/msg/header.h>

// กำหนดขา GPIO
#define MOTOR_LEFT_DIR1 33
#define MOTOR_LEFT_DIR2 25
#define MOTOR_LEFT_PWM  32

#define MOTOR_RIGHT_DIR1 12
#define MOTOR_RIGHT_DIR2 14
#define MOTOR_RIGHT_PWM  13

#define ENCODER_LEFT_A  34
#define ENCODER_LEFT_B  35
#define ENCODER_RIGHT_A 26
#define ENCODER_RIGHT_B 27

// ค่าพื้นฐาน
#define WHEEL_RADIUS 0.05   // รัศมีล้อ (เมตร)
#define WHEEL_BASE   0.20   // ระยะห่างล้อซ้าย-ขวา (เมตร)
#define ENCODER_TICKS_PER_REV  2048  // จำนวน ticks ต่อรอบ

// ตัวแปร Encoder
volatile int left_encoder_count = 0;
volatile int right_encoder_count = 0;

// ความเร็วล้อซ้าย/ขวา
float left_wheel_speed = 0;
float right_wheel_speed = 0;

// ROS Node และ Publisher
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t sub_cmd_vel;
rcl_publisher_t pub_odom;
rcl_publisher_t pub_tf;
geometry_msgs__msg__Twist cmd_vel_msg;
nav_msgs__msg__Odometry odom_msg;
tf2_msgs__msg__TFMessage tf_msg;

// ฟังก์ชันควบคุมมอเตอร์
void setMotorSpeed(float left_speed, float right_speed) {
    // คำนวณ PWM
    int left_pwm = abs(left_speed) * 255;
    int right_pwm = abs(right_speed) * 255;

    // ควบคุมล้อซ้าย
    if (left_speed > 0) {
        digitalWrite(MOTOR_LEFT_DIR1, HIGH);
        digitalWrite(MOTOR_LEFT_DIR2, LOW);
    } else {
        digitalWrite(MOTOR_LEFT_DIR1, LOW);
        digitalWrite(MOTOR_LEFT_DIR2, HIGH);
    }
    analogWrite(MOTOR_LEFT_PWM, left_pwm);

    // ควบคุมล้อขวา
    if (right_speed > 0) {
        digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
        digitalWrite(MOTOR_RIGHT_DIR2, LOW);
    } else {
        digitalWrite(MOTOR_RIGHT_DIR1, LOW);
        digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
    }
    analogWrite(MOTOR_RIGHT_PWM, right_pwm);
}

// Callback สำหรับ cmd_vel
void cmd_vel_callback(const void *msg) {
    geometry_msgs__msg__Twist *twist = (geometry_msgs__msg__Twist *)msg;
    float linear_x = twist->linear.x;
    float angular_z = twist->angular.z;

    // คำนวณความเร็วล้อซ้าย-ขวา
    left_wheel_speed = linear_x - (angular_z * WHEEL_BASE / 2);
    right_wheel_speed = linear_x + (angular_z * WHEEL_BASE / 2);

    // ส่งค่าไปขับมอเตอร์
    setMotorSpeed(left_wheel_speed, right_wheel_speed);
}

// ฟังก์ชันคำนวณ Odometry
void calculateOdometry() {
    float left_distance = (left_encoder_count / (float)ENCODER_TICKS_PER_REV) * (2 * PI * WHEEL_RADIUS);
    float right_distance = (right_encoder_count / (float)ENCODER_TICKS_PER_REV) * (2 * PI * WHEEL_RADIUS);
    
    float distance = (left_distance + right_distance) / 2;
    float delta_theta = (right_distance - left_distance) / WHEEL_BASE;

    // คำนวณตำแหน่งใหม่
    static float x = 0, y = 0, theta = 0;
    theta += delta_theta;
    x += distance * cos(theta);
    y += distance * sin(theta);

    // ส่งข้อมูล Odometry
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.orientation.z = sin(theta / 2);
    odom_msg.pose.pose.orientation.w = cos(theta / 2);
    rcl_publish(&pub_odom, &odom_msg, NULL);
}

// ตั้งค่า Encoder Interrupt
void IRAM_ATTR left_encoder_ISR() { left_encoder_count++; }
void IRAM_ATTR right_encoder_ISR() { right_encoder_count++; }

void setup() {
    //Serial.begin(115200);
    set_microros_transports();

    // ตั้งค่า GPIO
    pinMode(MOTOR_LEFT_DIR1, OUTPUT);
    pinMode(MOTOR_LEFT_DIR2, OUTPUT);
    pinMode(MOTOR_LEFT_PWM, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR2, OUTPUT);
    pinMode(MOTOR_RIGHT_PWM, OUTPUT);
    pinMode(ENCODER_LEFT_A, INPUT);
    pinMode(ENCODER_RIGHT_A, INPUT);
    attachInterrupt(ENCODER_LEFT_A, left_encoder_ISR, RISING);
    attachInterrupt(ENCODER_RIGHT_A, right_encoder_ISR, RISING);

    // ตั้งค่า ROS
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_node", "", &support);
    rclc_subscription_init_default(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
    rclc_publisher_init_default(&pub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    calculateOdometry();
}