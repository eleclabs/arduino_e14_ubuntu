#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#include <rmw_microros/rmw_microros.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

// กำหนดขาสำหรับ Stepper Motor
#define STEP_PIN 13
#define DIR_PIN 17
#define LED_PIN 2

rclc_support_t support;
rcl_timer_t timer;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;

rcl_subscription_t sub;

std_msgs__msg__Int32 step_msg;

bool micro_ros_init_successful;
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {
    //rcl_publish(&publisher, &msg, NULL);
    //msg.data++;
  }
}


bool create_entities(){
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "int32_publisher_rclc", "", &support));

  // create publisher
  /*
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "std_msgs_msg_Int32"));
  */

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
}


void destroy_entities(){
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  //rcl_publisher_fini(&publisher, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  micro_ros_init_successful = false;
}


// ตัวแปรควบคุม Stepper
volatile long target_steps = 0;  // จำนวนสเต็ปที่ต้องหมุน
volatile bool motorMoving = false;

// ฟังก์ชันหมุน Stepper Motor
void moveStepper(int steps) {
    bool direction = (steps > 0);
    digitalWrite(DIR_PIN, direction); // กำหนดทิศทาง

    for (long i = 0; i < abs(steps); i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(800);  // ปรับความเร็วได้ที่นี่
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(800);
    }

    motorMoving = false; // หยุดเมื่อหมุนครบ
}

// Callback รับข้อมูลจาก ROS 2
void step_callback(const void *msg) {
    const std_msgs__msg__Int32 *data = (const std_msgs__msg__Int32 *)msg;
    target_steps = data->data; // รับจำนวนสเต็ป
    motorMoving = true;        // เริ่มหมุน
}

void setup() {
    // ตั้งค่าขา Stepper
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

    // เริ่มต้น Micro-ROS
    set_microros_transports();
    rclc_support_t support;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    // สร้าง Node
    rclc_node_init_default(&node, "stepper_node", "", &support);

    // สร้าง Subscription รับคำสั่งจาก ROS 2
    rclc_subscription_init_default(&sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "stepper_cmd");

    // ตั้งค่า Executor
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &sub, &step_msg, &step_callback, ON_NEW_DATA);
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
        //computePIDR(desiredRPMR, pr);
        //computePIDL(desiredRPML, pl);
        //counter_tick();
        // ประมวลผล Executor

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
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
    // หมุน Stepper หากมีคำสั่ง
    if (motorMoving && target_steps != 0) {
        moveStepper(target_steps);
        target_steps = 0; // เคลียร์ค่าหลังหมุนเสร็จ
    }
  } else {
    digitalWrite(LED_PIN, 0);
  }

}
