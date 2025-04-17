///////////////////
///    Servo    ///
///////////////////
#include <ESP32Servo.h>
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

uint16_t pwm_min = 1000;
uint16_t pwm_mid = 1500;
uint16_t pwm_max = 2000;

int servo1_pin = 15;
int servo2_pin = 4;
int servo3_pin = 16;
int servo4_pin = 17;

////////////////////////
/// SBUS RC Receiver ///
////////////////////////
/*
#include <driver/uart.h>
#define SBUS_UART UART_NUM_2
#define SBUS_RX 16
#define SBUS_TX 17
#define BUF_SIZE (1024 * 2)
static QueueHandle_t uart2_queue;
uint16_t ch[16];
uint16_t checksum = 0;
uint16_t sbus_ch[16];
uint16_t sbus_min = 368;
uint16_t sbus_mid = 1024;
uint16_t sbus_max = 1680;
uint16_t sbus_db = 20;
uint16_t sbus_min_db = sbus_mid - sbus_db;
uint16_t sbus_max_db = sbus_mid + sbus_db;
static const char * TAG = "";
bool lost_frame;
bool failsafe;

//////////////////////
///   BNO055 IMU   ///
//////////////////////
#include <Wire.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
double qw, qx, qy, qz;
double ax, ay, az;
double gx, gy, gz;
*/
/////////////////////
/// For Micro ROS ///
/////////////////////
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

/// TODO : include your desired msg header file

#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/u_int16_multi_array.h>
#include <std_msgs/msg/bool.h>
//#include <std_msgs/msg/int16.h>

/*
 * Optional,
 * LED pin to check connection
 * between micro-ros-agent and ESP32
*/

#define LED_PIN 2
//#define PILOT_LED 17      // subscription LED
bool led_state = false;

// #define LED_PIN_TEST 18   // subscription LED
// bool led_test_state = false;

#define BTN_PIN 36
bool btn_state = false;

/// Helper functions to help reconnect
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;


/// Declare rcl object
rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;

/*
 * TODO : Declare your 
 * publisher & subscription objects below
*/

rcl_publisher_t btn_pub;
//rcl_publisher_t sbus_pub;
//rcl_publisher_t imu_pub;
rcl_subscription_t pwmArray_sub;
rcl_subscription_t led_sub;

//rcl_publisher_t int16_pub;
//rcl_publisher_t int16array_pub;
//rcl_subscription_t int16array_sub;
/*
 * TODO : Define your necessary Msg
 * that you want to work with below.
*/

//std_msgs__msg__UInt16MultiArray sbus_msg;
std_msgs__msg__UInt16MultiArray pwmArray_msg;
std_msgs__msg__Bool btn_msg;
std_msgs__msg__Bool led_msg;
//sensor_msgs__msg__Imu imu_msg;

/*
std_msgs__msg__Int16 int16_msg;
std_msgs__msg__Int16MultiArray int16array_send_msg;
std_msgs__msg__Int16MultiArray int16array_recv_msg;
std_msgs__msg__Bool led_msg;
*/

/*
 * TODO : Define your subscription callbacks here
 * leave the last one as timer_callback()
*/
void led_callback(const void *msgin) {
  std_msgs__msg__Bool * led_msg = (std_msgs__msg__Bool *)msgin;

  /// Do something with your receive message

  led_state = led_msg->data;
  digitalWrite(LED_PIN, led_state);  

  //led_test_state = led_msg->data;
  //digitalWrite(LED_PIN_TEST, led_test_state);
}

void pwmArray_callback(const void *msgin) {
  std_msgs__msg__UInt16MultiArray * pwmArray_msg = (std_msgs__msg__UInt16MultiArray *)msgin;
  /// Do something with your receive message
  servo1.writeMicroseconds(pwmArray_msg->data.data[0]);
  servo2.writeMicroseconds(pwmArray_msg->data.data[1]);
  servo3.writeMicroseconds(pwmArray_msg->data.data[2]);
  servo4.writeMicroseconds(pwmArray_msg->data.data[3]);
}

/*
void int16array_callback(const void *msgin) {
  std_msgs__msg__Int16MultiArray * int16array_recv_msg = (std_msgs__msg__Int16MultiArray *)msgin;

  /// Do something with your receive message
   
  int16array_send_msg.data.data[0] = int16array_recv_msg->data.data[0];
  int16array_send_msg.data.data[1] = int16array_recv_msg->data.data[1];
}
*/

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {
    /*
     * TODO : Publish anything inside here
     * For example, we are going to echo back
     * the int16array_sub data to int16array_pub data,
     * so we could see the data reflect each other.
     * And also keep incrementing the int16_pub
    */

    /*
    rcl_publish(&int16array_pub, &int16array_send_msg, NULL);
    int16_msg.data++;
    rcl_publish(&int16_pub, &int16_msg, NULL);
    */

    /// Publish btn state  ///
    btn_msg.data = btn_state;
    rcl_publish(&btn_pub, &btn_msg, NULL);

    ///  Publish SBUS value  ///
    // memcpy(sbus_msg.data.data, ch, sizeof(ch));
    // rcl_publish(&sbus_pub, &sbus_msg, NULL);

    ///  Publish IMU data  ///
    /*
    imu_msg.header.stamp.sec = int(millis() / 1000);
    imu_msg.header.stamp.nanosec = RCL_MS_TO_NS(millis());
    imu_msg.orientation.x = qx;
    imu_msg.orientation.y = qy;
    imu_msg.orientation.z = qz;
    imu_msg.orientation.w = qw;
    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;
    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;
    rcl_publish(&imu_pub, &imu_msg, NULL);
    */
  }
}

/*
   Create object (Initialization)
*/
bool create_entities()
{
  /*
   * TODO : Define your
   * - ROS node name
   * - namespace
   * - ROS_DOMAIN_ID
  */
  const char * node_name = "esp32_microros_node";
  const char * ns = "";
  const int domain_id = 0;
  
  /// Initialize node

  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, domain_id);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, node_name, ns, &support);

  /// TODO : Init your publisher and subscriber 

  rclc_publisher_init(
    &btn_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/esp/btn", &rmw_qos_profile_default);

/*
  rclc_publisher_init(
    &sbus_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
    "/esp/sbus", &rmw_qos_profile_default);

  rclc_publisher_init(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/esp/imu", &rmw_qos_profile_default);
*/

  rclc_subscription_init(
    &pwmArray_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
    "/esp/pwm_array", &rmw_qos_profile_default);

  rclc_subscription_init(
    &led_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/esp/led", &rmw_qos_profile_default);

/*
  rclc_publisher_init(
    &int16array_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "/esp/int16array_pub", &rmw_qos_profile_default);

  rclc_publisher_init(
    &int16_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/esp/int_pub", &rmw_qos_profile_default);

  rclc_subscription_init(
    &int16array_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "/esp/int16array_sub", &rmw_qos_profile_default);

  rclc_subscription_init(
    &led_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/esp/led", &rmw_qos_profile_default);
*/

  /*
   * Init timer_callback
   * TODO : change timer_timeout
   * 50ms : 20Hz
   * 20ms : 50Hz
   * 10ms : 100Hz
  */
  const unsigned int timer_timeout = 50;
  rclc_timer_init_default(&timer,&support, RCL_MS_TO_NS(timer_timeout), timer_callback);

  /*
   * Init Executor
   * TODO : make sure the num_handles is correct
   * num_handles = total_of_subscriber + timer
   * publisher is not counted
   * 
   * TODO : make sure the name of sub msg and callback are correct
   */
  unsigned int num_handles = 3;
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  // rclc_executor_add_subscription(&executor, &int16array_sub, &int16array_recv_msg, &int16array_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &pwmArray_sub, &pwmArray_msg, &pwmArray_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &led_sub, &led_msg, &led_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  return true;
}

 /// Clean up all the created objects
void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_init_options_fini(&init_options);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  /// TODO : Make sue the name of publisher and subscriber are correct
  rcl_publisher_fini(&btn_pub, &node);
  //rcl_publisher_fini(&sbus_pub, &node);
  //rcl_publisher_fini(&imu_pub, &node);
  rcl_subscription_fini(&pwmArray_sub, &node);  
  rcl_subscription_fini(&led_sub, &node);

/*
  rcl_publisher_fini(&int16array_pub, &node);
  rcl_publisher_fini(&int16_pub, &node);
  rcl_subscription_fini(&int16array_sub, &node);
*/ 

}

void setup() {
  set_microros_transports();
  // set_microros_wifi_transports("WIFI-SSID", "WIFI-PW", "HOST_IP", 8888);

  /// Optional, setup output pin for LEDs

  //pinMode(LED_PIN_TEST, OUTPUT);   
  pinMode(LED_PIN, OUTPUT);
  //pinMode(PILOT_LED, OUTPUT);
  pinMode(BTN_PIN, INPUT);

  
  /// TODO : Initialze the message data variable
  
 ///  Init PWM msg  ///
  uint16_t pwm_mid_array[4] = {pwm_mid, pwm_mid, pwm_mid, pwm_mid};
  pwmArray_msg.data.capacity = 4; // number of array length
  pwmArray_msg.data.size = 4;
  pwmArray_msg.data.data = pwm_mid_array;

  /// Init button msg ///
  btn_msg.data = false;

  ///   Init LED msg ///
  led_msg.data = false;

  ///  Init SBUS msg  ///
/*
  static uint16_t array_data[16];
  for(int i; i < 16; i++){
    array_data[i] = 1024;
  }
  sbus_msg.data.capacity = 16;
  sbus_msg.data.size = 16;
  sbus_msg.data.data = array_data;

///  Init IMU msg  ///
  imu_msg.header.frame_id.data = "imu_link";
  imu_msg.header.stamp.sec = int(millis()/1000);
  imu_msg.header.stamp.nanosec = RCL_MS_TO_NS(millis());

  int16_t array_datas[2] = {0, 0};
  int16array_send_msg.data.capacity = 2; // number of array length
  int16array_send_msg.data.size = 2;     // number of array length
  int16array_send_msg.data.data = array_datas;

  int16array_recv_msg.data.capacity = 2; // number of array length
  int16array_recv_msg.data.size = 2;     // number of array length
  int16array_recv_msg.data.data = array_datas;

  int16_msg.data = 0;
*/


  /// Setup first state

  state = WAITING_AGENT;

  /////////////////////////
  ///  ESP32 PWM Servo  ///
  /////////////////////////
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  servo4.setPeriodHertz(50);
  servo1.attach(servo1_pin, pwm_min, pwm_max);
  servo2.attach(servo2_pin, pwm_min, pwm_max);
  servo3.attach(servo3_pin, pwm_min, pwm_max);
  servo4.attach(servo4_pin, pwm_min, pwm_max);

  /////////////////
  ///    SBUS   ///
  /////////////////
  /*
  uart_config_t uart2_config = {
    .baud_rate = 100000,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_EVEN,
    .stop_bits = UART_STOP_BITS_2,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };

  uart_param_config(SBUS_UART, &uart2_config);
  esp_log_level_set(TAG, ESP_LOG_INFO);
  uart_set_pin(SBUS_UART, SBUS_TX, SBUS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_set_line_inverse(SBUS_UART, UART_SIGNAL_RXD_INV);
  uart_driver_install(SBUS_UART, BUF_SIZE, BUF_SIZE, 20, &uart2_queue, 0);

  xTaskCreate(uart2_rx_task, "uart2_rx_task", 2048, NULL, 12, NULL);
*/

  ////////////////
  ///  BNO055  ///
  ////////////////
  /*
  if(!bno.begin()){
    //Serial.print("no BNO005 detected ... Check your wiring or I2C ADDT!!")
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  */

}

void loop() {

   /// Try ping the micro-ros-agent (HOST PC), then switch the state 
   /// https://github.com/micro-ROS/micro_ros_arduino/blob/galactic/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
 
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

   /// Output LED when in AGENT_CONNECTED state

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }

  /*
   * TODO : 
   * Do anything else you want to do here,
   * like read sensor data,  
   * calculate something, etc.
   */

  /// Read button ///
  btn_state = digitalRead(BTN_PIN);

  /// Read BNO055 data ///
  /*
  sensors_event_t orientationData, angVelocityData, linearAccelData;
  //bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  imu::Quaternion quat = bno.getQuat();
  qw = quat.w();
  qx = quat.x();
  qy = quat.y();
  qz = quat.z();

  ax = linearAccelData.acceleration.x;
  ay = linearAccelData.acceleration.y;  
  az = linearAccelData.acceleration.z;  

  gx = angVelocityData.gyro.x;
  gy = angVelocityData.gyro.y;  
  gz = angVelocityData.gyro.z;  
  */
}

/*
static void uart2_rx_task(void *pvParameters){
  uart_event_t event;
  size_t buffered_size;
  bool exit_condition = false;

  while(1){
    if(xQueueReceive(uart2_queue, (void *)&event, (portTickType)portMAX_DELAY)){
      /// Handle received event
      if(event.type == UART_DATA){
        uint8_t buf_[128];
        int UART2_data_length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&UART2_data_length));
        UART2_data_length = uart_read_bytes(UART_NUM_2, buf_, UART2_data_length, 100);

        /// from  https://github.com/bolderflight/sbus/tree/main
        if(buf_[0] == 0x0F){
          ch[0] = static_cast<uint16_t>(buf_[1] | ((buf_[2] << 8) & 0x07FF));
          ch[1] = static_cast<uint16_t>(buf_[2] >> 3 | ((buf_[3] << 5) & 0x07FF));
          ch[2] = static_cast<uint16_t>(buf_[3] >> 6 | (buf_[4] << 2) | ((buf_[5] << 10) & 0x07FF));
          ch[3] = static_cast<uint16_t>(buf_[5] >> 1 | ((buf_[6] << 7) & 0x07FF));
          ch[4] = static_cast<uint16_t>(buf_[6] >> 4 | ((buf_[7] << 4) & 0x07FF));
          ch[5] = static_cast<uint16_t>(buf_[7] >> 7 | (buf_[8] << 1) | ((buf_[9] << 9) & 0x07FF));
          ch[6] = static_cast<uint16_t>(buf_[9] >> 2 | ((buf_[10] << 6) & 0x07FF));
          ch[7] = static_cast<uint16_t>(buf_[10] >> 5 | ((buf_[11] << 3) & 0x07FF));
          ch[8] = static_cast<uint16_t>(buf_[12] | ((buf_[13] << 8) & 0x07FF));
          ch[9] = static_cast<uint16_t>(buf_[13] >> 3 | ((buf_[14] << 5) & 0x07FF));
          ch[10] = static_cast<uint16_t>(buf_[14] >> 6 | (buf_[15] << 2) | ((buf_[16] << 10) & 0x07FF));
          ch[11] = static_cast<uint16_t>(buf_[16] >> 1 | ((buf_[17] << 7) & 0x07FF));
          ch[12] = static_cast<uint16_t>(buf_[17] >> 4| ((buf_[18] << 4) & 0x07FF));
          ch[13] = static_cast<uint16_t>(buf_[18] >> 7 | (buf_[19] << 1) | ((buf_[20] << 9) & 0x07FF));
          ch[14] = static_cast<uint16_t>(buf_[20] >> 2 | ((buf_[21] << 6) & 0x07FF));
          ch[15] = static_cast<uint16_t>(buf_[21] >> 5| ((buf_[22] << 3) & 0x07FF));

          /// Grap the lost frame 
          lost_frame = buf_[23] & 0x04;
          /// Grap the failsafe 
          failsafe = buf_[23] & 0x08;
        }
      }
    }

    /// If you want to break out of the loop due to certain condition, set exit condition to true
    if(exit_condition){
      break;
    }
  }

  /// Out side of loop now. Task needs to clean up and self teminate before returning
  vTaskDelete(NULL);

}

*/