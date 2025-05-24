#include "ros_interface.h"
#include "shared_resources.h"

//Setup a time handle
TimerHandle_t watchdog_ros_timer;

//Micro-Ros objects
rcl_subscription_t subscriber;
rcl_publisher_t imu_pub;
rcl_publisher_t mag_pub;

// Message objects
geometry_msgs__msg__Twist msg;
sensor_msgs__msg__Imu msg_imu;
sensor_msgs__msg__MagneticField msg_mag;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t ros_allocator;
rcl_node_t node;
rcl_timer_t timer;

ControlCommand cmd;

static volatile uint32_t last_msg_tick = 0;
#define TIMEOUT_ROS_MS 120
char imu_frame_id_buffer[16] = "imu_link";

void init_ROS(){

    //Setup control data mutex
    controlDataMutex = xSemaphoreCreateMutex();
    if(controlDataMutex == NULL){
        //digitalWrite(LED_PIN, LOW);
        error_loop();
    }

    Serial.begin(921600);
    Serial.setRxBufferSize(1024);

    set_microros_serial_transports(Serial);
    
    delay(2000);

    ros_allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &ros_allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "saurcon_rc", "", &support));
    
    // create subscriber with best-effort Qos
    RCCHECK(rclc_subscription_init_best_effort(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "ctrl_output"));

    // create IMU Publisher
    RCCHECK(rclc_publisher_init_default(
      &imu_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu/data_raw"));
    
    // create magnetometer Publisher
    RCCHECK(rclc_publisher_init_default(
      &mag_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
      "imu/mag"));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &ros_allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
    //setup_watchdog_ros_timer();
}

void setup_watchdog_ros_timer(){
  watchdog_ros_timer = xTimerCreate(
    "msg_watchdog",
    pdMS_TO_TICKS(10), //check every 10ms
    pdTRUE,
    NULL,
    watchdog_ros_callback
  );
  xTimerStart(watchdog_ros_timer, 0);
}

//Setup watchdog_ros timer callback
void watchdog_ros_callback(TimerHandle_t xTimer){
  uint32_t now = xTaskGetTickCount();
  if((now-last_msg_tick) > pdMS_TO_TICKS(TIMEOUT_ROS_MS)) {
    //stateMachine->setFault(ROS_CONNECTION_LOSS);
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

void subscription_callback(const void * msgin)
{  
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  last_msg_tick = xTaskGetTickCount();

  // Lock the mutex before updating shared variable
  if(xSemaphoreTake(controlDataMutex, portMAX_DELAY) == pdTRUE)
  {
    // Extract the velocity and steering commands from the message
    velCommand = msg->linear.x;
    steerCommand = msg->angular.z;

    cmd.steer    = steerCommand;
    cmd.throttle = velCommand;

    xSemaphoreGive(controlDataMutex);
  };

  xQueueSend(controlQueue, &cmd, pdMS_TO_TICKS(5));  // Send to queue without waiting  
}

// uRos Subscribe Task
void ros_subscriber_task(void *pvParameters) 
{
  while(1) {
    // Spin the executor to process incoming messages with a timeout
    rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));

    if (ret != RCL_RET_OK) {stateMachine->setFault(ROS_CONNECTION_LOSS);}

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// uRos Publish Task
void ros_publisher_task(void *pvParameters)
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

    vTaskDelay(pdMS_TO_TICKS(10)); //Delay 10ms for 100hz 
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