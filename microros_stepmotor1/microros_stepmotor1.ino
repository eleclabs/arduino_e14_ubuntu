#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// กำหนดขา
#define STEP_PIN 13
#define DIR_PIN 17

rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t sub;
std_msgs__msg__Int32 step_msg;

// ฟังก์ชันสั่งหมุน Stepper Motor
void moveStepper(int steps, bool direction) {
    digitalWrite(DIR_PIN, direction);
    for (int i = 0; i < abs(steps); i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(800);  // ปรับความเร็วที่นี่
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(800);
    }
}

// Callback รับข้อความจาก ROS 2
void step_callback(const void *msg) {
    const std_msgs__msg__Int32 *data = (const std_msgs__msg__Int32 *)msg;
    int steps = data->data;
    bool direction = (steps >= 0);
    moveStepper(steps, direction);
}

void setup() {
    // ตั้งค่าขา
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    
    // เริ่มต้น Micro-ROS
    set_microros_transports();
    
    rclc_support_t support;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    // สร้าง Node และ Subscription
    rclc_node_init_default(&node, "stepper_node", "", &support);
    rclc_subscription_init_default(&sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "stepper_cmd");

    // ตั้งค่า Executor
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &sub, &step_msg, &step_callback, ON_NEW_DATA);
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
