#include "ros_interface.h"
#include "shared_resources.h"

//Setup a time handle
TimerHandle_t watchdog_ros_timer;
portMUX_TYPE rosTickMux = portMUX_INITIALIZER_UNLOCKED;

//Micro-Ros objects
rcl_subscription_t control_subscriber;
rcl_subscription_t state_cmd_subscriber;

rcl_publisher_t imu_pub;
rcl_publisher_t mag_pub;
rcl_publisher_t state_pub;

// Message objects
std_msgs__msg__UInt8 state_msg;
geometry_msgs__msg__Twist ctrl_msg;
sensor_msgs__msg__Imu msg_imu;
sensor_msgs__msg__MagneticField msg_mag;
rclc_executor_t exec_state, exec_ctrl;
rclc_support_t support;
rcl_allocator_t ros_allocator;
rcl_node_t node;
rcl_timer_t timer;

ControlCommand cmd;

static volatile uint32_t last_msg_tick = 0;
#define TIMEOUT_ROS_MS 1000
char imu_frame_id_buffer[16] = "imu_link";

void init_ROS(){

    //Setup control data mutex
    controlDataMutex = xSemaphoreCreateMutex();
    if(controlDataMutex == NULL){
        //digitalWrite(LED_PIN, LOW);
        error_loop();
    }

    Serial.begin(921600);
    Serial.setRxBufferSize(4096);

    set_microros_serial_transports(Serial);
    
    delay(1000);

    ros_allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &ros_allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "saurcon_rc", "", &support));
    
    // create subscriber with best-effort Qos
    RCCHECK(rclc_subscription_init_best_effort(
        &control_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "ctrl_output"));

    // create subscriber with best-effort Qos
    RCCHECK(rclc_subscription_init_best_effort(
        &state_cmd_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "rc_state_cmd"));

    // create IMU Publisher
    RCCHECK(rclc_publisher_init_best_effort(
      &imu_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu/data_raw"));
    
    // create magnetometer Publisher
    RCCHECK(rclc_publisher_init_best_effort(
      &mag_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
      "imu/mag"));

    // create state Publisher
    RCCHECK(rclc_publisher_init_default(
      &state_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
      "saurcon/state"
    ));

    // create executor
    RCCHECK(rclc_executor_init(&exec_state, &support.context, 2, &ros_allocator));
    RCCHECK(rclc_executor_init(&exec_ctrl,  &support.context, 2, &ros_allocator));

    // attach
    RCCHECK(rclc_executor_add_subscription(&exec_ctrl, &control_subscriber, &ctrl_msg, &ctrl_sub_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&exec_state, &state_cmd_subscriber, &state_msg, &state_cmd_sub_callback, ON_NEW_DATA));
}

void setup_watchdog_ros_timer(){
  watchdog_ros_timer = xTimerCreate(
    "msg_watchdog",
    pdMS_TO_TICKS(30), //check every 30ms
    pdTRUE,
    NULL,
    watchdog_ros_callback
  );
  xTimerStart(watchdog_ros_timer, 0);
}

//Setup watchdog_ros timer callback
void watchdog_ros_callback(TimerHandle_t xTimer){
  static uint8_t miss_count = 0;
  uint32_t tick_copy = 0;

  portENTER_CRITICAL(&rosTickMux);
  tick_copy = last_msg_tick;
  portEXIT_CRITICAL(&rosTickMux);

  //Bypass if we havent hit a message yet.
  if(tick_copy == 0){return;}

  uint32_t now = xTaskGetTickCount();
  if((now-tick_copy) > pdMS_TO_TICKS(TIMEOUT_ROS_MS)) {
    if(++miss_count > 5) {
      stateMachine->setFault(ROS_CTRL_WDOG);
    }
  } else {
    miss_count = 0;
  }
}

// Define the Micro-ROS objects declared in the header file
void error_loop(){
  while(1){
    stateMachine->setFault(ROS_CONNECTION_LOSS);
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP.restart();
    
  }
}

void ctrl_sub_callback(const void * msgin)
{  
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  static bool led_state = false;

  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state;

  portENTER_CRITICAL(&rosTickMux);
  last_msg_tick = xTaskGetTickCount();
  portEXIT_CRITICAL(&rosTickMux);

  /*
  SerialDBug.print("Time since last msg: ");
  SerialDBug.print(xTaskGetTickCount() - prnt_last_msg_time);
  SerialDBug.println(" ms");
  */

  // Lock the mutex before updating shared variable
  if(xSemaphoreTake(controlDataMutex, portMAX_DELAY) == pdTRUE)
  {
    // Extract the velocity and steering commands from the message
    velCommand   = msg->linear.x;
    steerCommand = msg->angular.z;

    cmd.steer    = steerCommand;
    cmd.throttle = velCommand;

    xSemaphoreGive(controlDataMutex);
  };

  BaseType_t sent = xQueueSend(controlQueue, &cmd, pdMS_TO_TICKS(5));  // Send to queue without waiting  
  if (sent!= pdTRUE) {
    stateMachine->setFault(SaurconFaults::ROS_QUEUE_FULL);
  }
}

// uRos state command subscriber callback
void state_cmd_sub_callback(const void * msgin)
{
  static uint32_t last_processed_tick = 0;
  uint32_t now = xTaskGetTickCount();

  // Only process every 100 ms
  if ((now - last_processed_tick) < pdMS_TO_TICKS(300)) {
    return;  // too soon, skip this one
  }

  last_processed_tick = now;

  const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
  SaurconState commandState = static_cast<SaurconState>(msg->data);
  stateMachine->setCommandState(commandState);
}

// uRos Subscribe Task
void ros_ctrl_sub_task(void *pvParameters) 
{
  while(1) {
    // Spin the executor to process incoming messages with a timeout
    rcl_ret_t ret = rclc_executor_spin_some(&exec_ctrl, RCL_MS_TO_NS(5));

    if (ret != RCL_RET_OK) {stateMachine->setFault(ROS_COM_LOSS);}

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// uRos Subscribe Task
void ros_state_sub_task(void *pvParameters) 
{
  while(1) {
    // Spin the executor to process incoming messages with a timeout
    rcl_ret_t ret = rclc_executor_spin_some(&exec_state, RCL_MS_TO_NS(5));

    if (ret != RCL_RET_OK) {stateMachine->setFault(ROS_COM_LOSS);}

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// uRos State Publish Task
void ros_state_publisher_task(void *pvParameters)
{
  while(1){
    state_msg.data = static_cast<uint8_t>(stateMachine->getState());

    rcl_ret_t ok_pub = rcl_publish(&state_pub, &state_msg, NULL);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
// uRos Sensor Publish Task
void ros_sensor_publisher_task(void *pvParameters)
{
  while(1){
    imu->update(); // semaphore protected call

    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float qx, qy, qz, qw;

    imu->getAccel(ax, ay, az);
    imu->getGyro(gx, gy, gz);
    imu->getMag(mx, my, mz);
    imu->getQuaternion(qx, qy, qz, qw);

    //Fill in the imu message
    fill_msg_header(msg_imu.header, "imu_link");
    msg_imu.linear_acceleration.x = ax * 9.80665;
    msg_imu.linear_acceleration.y = ay * 9.80665;
    msg_imu.linear_acceleration.z = az * 9.80665;

    //From MPU 6050 datasheet +- 2G
    msg_imu.linear_acceleration_covariance[0] = -1;

    msg_imu.angular_velocity.x = gx * DEG_TO_RAD;
    msg_imu.angular_velocity.y = gy * DEG_TO_RAD;
    msg_imu.angular_velocity.z = gz * DEG_TO_RAD;

    //From MPU 6050 datasheet +-250 deg/s
    msg_imu.angular_velocity_covariance[0] = -1;

    msg_imu.orientation.w = qw;
    msg_imu.orientation.x = qx;
    msg_imu.orientation.y = qy;
    msg_imu.orientation.z = qz;

   for (int i = 0; i < 9; i++) {
      msg_imu.linear_acceleration_covariance[i] = -1.0;
      msg_imu.angular_velocity_covariance[i] = -1.0;
      msg_imu.orientation_covariance[i] = -1.0;
    }

    //Fill in the Mag message
    fill_msg_header(msg_mag.header, "imu_link");

    msg_mag.magnetic_field.x = mx;
    msg_mag.magnetic_field.y = my;
    msg_mag.magnetic_field.z = mz;

    for (int i = 0; i < 9; i++) {
      msg_mag.magnetic_field_covariance[i] = -1.0;
    }


    rcl_ret_t rc_imu = rcl_publish(&imu_pub, &msg_imu, NULL);
    rcl_ret_t rc_mag = rcl_publish(&mag_pub, &msg_mag, NULL);

    /*** 
    if(rc_imu != RCL_RET_OK || rc_mag != RCL_RET_OK) {
      stateMachine->setFault(SaurconFaults::ROS_CONNECTION_LOSS);
      stateMachine->setState(SaurconState::FAULT_ROS_SCON);
    }
    ***/

    vTaskDelay(pdMS_TO_TICKS(50)); //Delay 10ms for 100hz 
  }
}

// Fill header with time + frame_id
void fill_msg_header(std_msgs__msg__Header &header, const char *frame_id_str) {
  header.stamp = get_time();
  header.frame_id.data = imu_frame_id_buffer;
  header.frame_id.size = strlen(imu_frame_id_buffer);
  header.frame_id.capacity = sizeof(imu_frame_id_buffer);
}

// Time for message
builtin_interfaces__msg__Time get_time() {
  builtin_interfaces__msg__Time t;
  uint64_t us = (uint32_t) (esp_timer_get_time());
  t.sec = us / 1000000;
  t.nanosec =  (us % 1000000) * 1000;
  return t;
}