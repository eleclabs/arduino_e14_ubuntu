#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <rclc/executor.h>

// ตั้งค่าขา Servo
#define SERVO1_PIN  15
#define SERVO2_PIN  4
#define SERVO3_PIN  16

Servo servo1, servo2, servo3;
int selected_servo = 1;  // Servo ที่ถูกเลือก (1, 2, 3)
int servo_angles[3] = {90, 90, 90};  // มุมของ Servo

rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;

// ฟังก์ชันปรับมุม Servo
void set_servo_angle(int servo, int angle) {
    angle = constrain(angle, 0, 180);  // จำกัดค่ามุมระหว่าง 0-180 องศา
    servo_angles[servo - 1] = angle;

    switch (servo) {
        case 1: servo1.write(angle); break;
        case 2: servo2.write(angle); break;
        case 3: servo3.write(angle); break;
    }
}

// Callback เมื่อได้รับข้อมูลจาก ROS 2
void servo_callback(const void *msg) {
    const std_msgs__msg__Int32 *data = (const std_msgs__msg__Int32 *)msg;
    int command = data->data;

    if (command == 1) selected_servo = 1;  // เลือก Servo 1
    else if (command == 2) selected_servo = 2;  // เลือก Servo 2
    else if (command == 3) selected_servo = 3;  // เลือก Servo 3
    else if (command == 10) set_servo_angle(selected_servo, servo_angles[selected_servo - 1] + 5);  // เพิ่มองศา
    else if (command == -10) set_servo_angle(selected_servo, servo_angles[selected_servo - 1] - 5);  // ลดองศา

    Serial.printf("Servo %d set to %d degrees\n", selected_servo, servo_angles[selected_servo - 1]);
}

void setup() {
    //Serial.begin(115200);
    set_microros_transports();

    // ตั้งค่า Servo
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);

    rclc_support_t support;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_servo_node", "", &support);
    rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "servo_command");
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &servo_callback, ON_NEW_DATA);
}

void loop() {
    delay(100);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
