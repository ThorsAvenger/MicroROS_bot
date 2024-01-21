#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
// #include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/temperature.h>

#include <ICM_20948.h>
#include <TMC2209.h>

// #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
// #error This example is only avaliable for Arduino framework with serial transport.
// #endif

//publisher
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg_pub;

rcl_publisher_t publisher_imu;
sensor_msgs__msg__Imu msg_imu;

rcl_publisher_t publisher_mag;
sensor_msgs__msg__MagneticField msg_mag;

rcl_publisher_t publisher_temp;
sensor_msgs__msg__Temperature msg_temp;

// subscriber
rcl_subscription_t subscriber;
// std_msgs__msg__Int32 msg_sub;
// geometry_msgs__msg__Twist msg_sub;
nav_msgs__msg__Odometry msg_sub;
// publisher and subscriber common
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_timer_t timer;
unsigned int num_handles = 5;   // 1 subscriber, 1 publisher

#define LED_PIN 13
#define WIRE_PORT Wire
#define AD0_VAL 1
ICM_20948_I2C myICM; 

#define RCCHECK(fn){rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    if(myICM.dataReady()) { 
      myICM.getAGMT(); 
       
      
    }

    msg_imu.linear_acceleration.x = myICM.accX();
    msg_imu.linear_acceleration.y = myICM.accY();
    msg_imu.linear_acceleration.z = myICM.accZ();
    msg_imu.angular_velocity.x = myICM.gyrX();
    msg_imu.angular_velocity.y = myICM.gyrY();
    msg_imu.angular_velocity.z = myICM.gyrZ();
    msg_mag.magnetic_field.x = myICM.magX();
    msg_mag.magnetic_field.y = myICM.magY();
    msg_mag.magnetic_field.z = myICM.magZ();
    msg_temp.temperature = myICM.temp();
    RCSOFTCHECK(rcl_publish(&publisher_temp, &msg_temp, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_imu, &msg_imu, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_mag, &msg_mag, NULL));

    //msg.data++;
  }
}

// void subscription_callback(const void * msgin)
// {  
//   const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
//   //digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH); 
  
//   Serial.print("I heard: ");
//   Serial.println(msg->data); 
// }
void subscription_callback(const void * msgin)
{  
  nav_msgs__msg__Odometry * msg = (nav_msgs__msg__Odometry *)msgin;
  //digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH); 
  
  Serial.print("I heard: ");
  Serial.println(msg->twist.twist.linear.x); 
}

void setup() {
  // Configure serial transport
  IPAddress agent_ip(192, 168, 2, 13);
  size_t agent_port = 8888;

  char ssid[] = "HyruleCastle";
  char psk[]= "M3G4M4N_X";

  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  Serial.begin(115200);
//   set_microros_serial_transports(Serial);
 
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); 

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_imu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "micro_ros_platformio_node_publisher_imu"));

   RCCHECK(rclc_publisher_init_default(
    &publisher_mag,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    "micro_ros_platformio_node_publisher_mag"));   

    RCCHECK(rclc_publisher_init_default(
    &publisher_temp,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
    "micro_ros_platformio_node_publisher_temp"));   

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "micro_ros_platformio_node_publisher"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));

  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));  
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));   

  msg_pub.data = 0;
   WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  myICM.begin(WIRE_PORT, AD0_VAL);
  Serial.println("Finished initialization.");
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}