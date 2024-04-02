#include <Arduino.h>
// #include <Wire.h>
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// #include <std_msgs/msg/int32.h>
// #include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h> 
#include <nav_msgs/msg/odometry.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/temperature.h>
#include <sensor_msgs/msg/joint_state.h>


#include <geometry_msgs/msg/transform_stamped.h>
// #include <tf2_msgs/msg/tf_message.h>


#include <ICM_20948.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h> 


// #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
// #error This example is only avaliable for Arduino framework with serial transport.
// #endif

// #define USE_WIFI_TRANSPORT

#ifndef TOPIC_PREFIX
#define TOPIC_PREFIX
#endif
#ifdef WDT_TIMEOUT
#include <esp_task_wdt.h>
#endif
#ifdef USE_WIFI_TRANSPORT
// remove wifi initialization code from wifi transport
static inline void set_microros_net_transports(IPAddress agent_ip, uint16_t agent_port)
{
    static struct micro_ros_agent_locator locator;
    locator.address = agent_ip;
    locator.port = agent_port;

    rmw_uros_set_custom_transport(
        false,
        (void *) &locator,
        platformio_transport_open,
        platformio_transport_close,
        platformio_transport_write,
        platformio_transport_read
    );
}
#endif

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)
// #ifdef ARDUINO_LOLIN_D32
//   #define DIR_PIN_R   12
//   #define STEP_PIN_R  14
//   #define DIR_PIN_L   27
//   #define STEP_PIN_L  26
//   #define EN_PIN      13
//   #define LED_PIN     5
// #endif

//#ifdef ARDUINO_DFROBOT_FIREBEETLE_2_ESP32E
  #define DIR_PIN_L   23
  #define STEP_PIN_L  19
  #define DIR_PIN_R   26
  #define STEP_PIN_R  0
  #define EN_PIN      25
  #define LED_PIN     4
//#endif 
#define SERIAL_PORT Serial2
#define STALL_VALUE 255
#define DRIVER_L_ADDR 0b11  // 16
#define DRIVER_R_ADDR 0b10  // 64
#define R_SENSE 0.11f 

int AMPS = 2000; 
int micro = 16; 
int Maccell = 10000; 
int Mspeed = 100; 
int MMperRev = 3.14*80;
float StepsPerRot = 200; 

float Vl; 
float Vr; 
float prev_L = 0; 
float prev_R = 0;
float alpha = 0;
float theta = 0; 

TMC2209Stepper driver_l(&SERIAL_PORT, R_SENSE, DRIVER_L_ADDR);
TMC2209Stepper driver_r(&SERIAL_PORT, R_SENSE, DRIVER_R_ADDR);
using namespace TMC2209_n; 

FastAccelStepperEngine engine = FastAccelStepperEngine(); 
// FastAccelStepperEngine engine_r = FastAccelStepperEngine(); 
FastAccelStepper *stepper_l = NULL; 
FastAccelStepper *stepper_r = NULL; 



//publisher
// rcl_publisher_t publisher;
rcl_publisher_t publisher_imu;
rcl_publisher_t publisher_mag;
rcl_publisher_t publisher_temp;
rcl_publisher_t publisher_odom;
rcl_publisher_t publisher_joint_state; 

// std_msgs__msg__Int32 msg_pub;
sensor_msgs__msg__Imu msg_imu;
sensor_msgs__msg__MagneticField msg_mag;
sensor_msgs__msg__Temperature msg_temp;
nav_msgs__msg__Odometry msg_odom;
sensor_msgs__msg__JointState msg_joint_state;


// subscriber
rcl_subscription_t subscriber;
// std_msgs__msg__Int32 msg_sub;
geometry_msgs__msg__Twist msg_sub;
// sensor_msgs__msg__JointState msg_sub;
sensor_msgs__msg__JointState msg_joint_cmd;


// publisher and subscriber common
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_timer_t timer;

// unsigned long long time_offset = 0;
int64_t time_offset = 0;

unsigned int num_handles = 6;   // 1 subscriber, 1 publisher

#define WIRE_PORT Wire
#define AD0_VAL     1
ICM_20948_I2C myICM;



#define RCCHECK(fn){rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define _POSIX_TIMERS 0


// Error handle loop
void error_loop() {
  while(1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    vTaskDelay(500);
  }
}

struct timespec getTime()
{
    struct timespec tp = {0};
#if (_POSIX_TIMERS > 0)
    clock_gettime(CLOCK_REALTIME, &tp);
#else
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
#endif
    return tp;
}

void timer_callback_odom(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if(timer != NULL) { 
    struct timespec time_stamp = getTime();
    msg_odom.header.stamp.sec = time_stamp.tv_sec;
    msg_odom.header.stamp.nanosec = time_stamp.tv_nsec;

    msg_joint_state.header.stamp.sec = time_stamp.tv_sec;
    msg_joint_state.header.stamp.nanosec = time_stamp.tv_nsec;

    //stepper_l->getCurrentSpeedInMilliHz();
    int mL = stepper_l->getCurrentPosition();
    int mR = stepper_r->getCurrentPosition();
    int vL = stepper_l->getCurrentSpeedInUs();
    int vR = stepper_r->getCurrentSpeedInUs();

    // Serial.printf("vL: %i\t vR: %i\n", vL, vR);
    float V[2] = {Vl/(M_PI*0.08),Vr/(M_PI*0.08)}; 
    float L = (mL/(micro*StepsPerRot))*2*M_PI; 
    float R = (mR/(micro*StepsPerRot))*2*M_PI;

    float dL = (L - prev_L)*0.04;
    float dR = (R - prev_R)*0.04;
    prev_L = L;
    prev_R = R; 
    float P[2] = {L,R}; 

    // Serial.printf("mL:%i\t dL: %f\t dR: %f\n",mL, dL, dR);

    
    float d = (dL+dR)/2; 
    float d_theta = (dR-dL)/(2*0.081);
    alpha = alpha + d_theta/2;
   

    float x = cos(theta+d_theta/2)*d;
    float y = sin(theta+d_theta/2)*d;

    theta = theta +d_theta;

    float z = sin(theta/2);
    float w = cos(theta/2);


     
    // Serial.printf("jsp: Pos x: %f\t Pos y: %f\n");


    msg_odom.pose.pose.position.x = msg_odom.pose.pose.position.x + x;
    msg_odom.pose.pose.position.y = msg_odom.pose.pose.position.y + y;
    msg_odom.pose.pose.orientation.z = z;
    msg_odom.pose.pose.orientation.w = w;


    for(int i = 0;i<2;i++) { 
      msg_joint_state.velocity.data[i] = V[i];
      msg_joint_state.position.data[i] = P[i];
    }
    



    RCSOFTCHECK(rcl_publish(&publisher_joint_state, &msg_joint_state, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_odom, &msg_odom, NULL));
    // Serial.printf("ODOM: Pos x: %f\t Pos y: %f\tjsp:Pos x: %f\t Pos y: %f\n", msg_odom.pose.pose.position.x, msg_odom.pose.pose.position.y, P[0], P[1]);
    Serial.printf("ODOM:Theta: %f\t Alpha: %f\n", theta, alpha);
  }

}


void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    if(myICM.dataReady()) { 
      Serial.println("Got imu data");
      myICM.getAGMT(); 
    }
    Serial.println(myICM.accX());
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

    struct timespec time_stamp = getTime();
    msg_imu.header.stamp.sec = time_stamp.tv_sec;
    msg_imu.header.stamp.nanosec = time_stamp.tv_nsec;

    msg_mag.header.stamp.sec = time_stamp.tv_sec;
    msg_mag.header.stamp.nanosec = time_stamp.tv_nsec;

    msg_temp.header.stamp.sec = time_stamp.tv_sec;
    msg_temp.header.stamp.nanosec = time_stamp.tv_nsec;


    RCSOFTCHECK(rcl_publish(&publisher_temp, &msg_temp, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_imu, &msg_imu, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_mag, &msg_mag, NULL));
  }
}

// void subscription_callback(const void * msgin)
// {  
//   const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
//   //digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH); 
  
//   Serial.print("I heard: ");
//   Serial.println(msg->data); 
// }

bool syncTime()
{
    
    const int timeout_ms = 1000;
    if (rmw_uros_epoch_synchronized()) {
      // Serial.printf("In syncTime(): Synched.\n\r");
      // while(1);
      return true; // synchronized previously
    }
    // get the current time from the agent
    RCCHECK(rmw_uros_sync_session(timeout_ms));
    if (rmw_uros_epoch_synchronized()) {
#if (_POSIX_TIMERS > 0)
        // Get time in milliseconds or nanoseconds
        int64_t time_ns = rmw_uros_epoch_nanos();
	timespec tp;
	tp.tv_sec = time_ns / 1000000000;
	tp.tv_nsec = time_ns % 1000000000;
	clock_settime(CLOCK_REALTIME, &tp);
#else
	int64_t ros_time_ms = rmw_uros_epoch_millis();
  Serial.printf("In syncTime(): ros_time_ms: %ld ms.\n\r", ros_time_ms);
	// unsigned long long ros_time_ms = rmw_uros_epoch_millis();
	// now we can find the difference between ROS time and uC time
	time_offset = ros_time_ms - millis();
#endif
	return true;
    }
    return false;
}



void subscription_callback(const void * msgin)
{  
    geometry_msgs__msg__Twist * msg = (geometry_msgs__msg__Twist *)msgin;
  float V_linx = msg->linear.x; // m/s to rad/s by dividing by r
  float V_angz = msg->angular.z;
  //digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH); 
  Vl = (V_linx-V_angz);
  Vr = (V_linx+V_angz);

  // sensor_msgs__msg__JointState * msg = (sensor_msgs__msg__JointState *)msgin; 
  // double Vle = msg->velocity.data[0];
  // double Vri = msg->velocity.data[1];
  // Serial.printf("Vl: %f\tVr: %f\n", Vl, Vr);
  int32_t stepsL = (int32_t)(Vl*200*micro);
  int32_t stepsR = (int32_t)(Vr*200*micro);
  // printf("%f, %f\n\r", V_linx, V_angz);
  // Serial.print(V_linx);
  // Serial.print(" ");
  // Serial.println(V_angz);
  if(stepsL < 0) { 
      stepper_l->runBackward();
      
 } else {
      stepper_l->runForward();
  }  
  if(stepsR < 0) { 
	// stepper_driver_R.disableInverseMotorDirection();
      stepper_r->runBackward();
  } else {
	// stepper_driver_R.enableInverseMotorDirection();
      stepper_r->runForward();
 }
  if((abs(stepsL) > 10 )|( abs(stepsR)  > 10)) {
    // driver_l.VACTUAL(Vl);
    stepper_l->setSpeedInHz(abs(stepsL));
    stepper_l->applySpeedAcceleration();
    
    stepper_r->setSpeedInHz(abs(stepsR));
    stepper_r->applySpeedAcceleration();

    // stepper_driver_R.enable();
    digitalWrite(5, HIGH);
  }
  else {
    // Serial.println("motor off");
    stepper_l->stopMove();
    stepper_r->stopMove();
    // stepper_driver_R.disable();
    digitalWrite(5, LOW);
  }
}

void setup() {
  // Configure serial transport
  IPAddress agent_ip(192, 168, 2, 38);  // Laptop IP Address :)
  // IPAddress agent_ip(192,168,228,223);
  size_t agent_port = 8888;

  char ssid[] = "HyruleCastle";
  char psk[]= "M3G4M4N_X";
  Serial.begin(115200);
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  
  // set_microros_serial_transports(Serial);
  // TMC2209 setup
  driver_l.begin(); 
  driver_r.begin(); 

  driver_l.toff(0); // 2
  driver_l.blank_time(24); 
  driver_r.toff(0); // 2
  driver_r.blank_time(24);

  driver_l.hysteresis_start(1);
  driver_l.hysteresis_end(12);
  driver_r.hysteresis_start(1);
  driver_r.hysteresis_end(12);

  driver_l.rms_current(AMPS, 0.01);
  driver_r.rms_current(AMPS, 0.01);

  driver_l.microsteps(micro); 
  driver_l.en_spreadCycle(false);
  driver_r.microsteps(micro); 
  driver_r.en_spreadCycle(false);
  driver_r.shaft(0);

  pinMode(LED_PIN, OUTPUT);
  pinMode(5,OUTPUT);
  digitalWrite(LED_PIN, HIGH); 

  engine.init();
  stepper_l = engine.stepperConnectToPin(STEP_PIN_L);
  stepper_r = engine.stepperConnectToPin(STEP_PIN_R);
  stepper_l->setDirectionPin(DIR_PIN_L);
  stepper_r->setDirectionPin(DIR_PIN_R);
  stepper_l->setEnablePin(EN_PIN);
  stepper_r->setEnablePin(EN_PIN);
  stepper_l->setAcceleration(1*Maccell);
  stepper_r->setAcceleration(1*Maccell);
  stepper_l->setAutoEnable(true);
  stepper_r->setAutoEnable(true);
  
  // driver_l.VACTUAL();
  

  delay(100);

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
    TOPIC_PREFIX "imu/data"));

   RCCHECK(rclc_publisher_init_default(
    &publisher_mag,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    TOPIC_PREFIX "imu/mag"));   

    RCCHECK(rclc_publisher_init_default(
    &publisher_temp,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
    TOPIC_PREFIX "imu/temp"));

    RCCHECK(rclc_publisher_init_default( //init_default
    &publisher_joint_state, 
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states"));

    RCCHECK(rclc_publisher_init_default(
    &publisher_odom,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    TOPIC_PREFIX "odom")); //diif_cont/odom


    msg_odom.header.frame_id.data = "odom";
    msg_odom.header.frame_id.size = sizeof("odom");
    
  // create subscriber
  // RCCHECK(rclc_subscription_init_default(
  //   &subscriber,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
  //   "odom"));

    RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), 
    "/diff_cont/cmd_vel_unstamped")); //"/cmd_vel"));

    // RCCHECK(rclc_subscription_init_default(
    // &subscriber,
    // &node,
    // ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), 
    // "/diffbot_joint_commands")); //"/cmd_vel"));

  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  const unsigned int timer_timeout_odom = 100;
    RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback_odom));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));  
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));   
  // RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_joint_cmd, &subscription_callback, ON_NEW_DATA));


     // initialize measured joint state message memory
    micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    &msg_joint_state,
    (micro_ros_utilities_memory_conf_t) {});

    //      // initialize measured joint state message memory
    // micro_ros_utilities_create_message_memory(
    // ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    // &msg_joint_cmd,
    // (micro_ros_utilities_memory_conf_t) {});

    // populate fixed message fields - size, frame ID and joint names for measured joint state
  msg_joint_state.header.frame_id = micro_ros_string_utilities_set(msg_joint_state.header.frame_id, "");
  msg_joint_state.name.size = msg_joint_state.position.size = msg_joint_state.velocity.size = 2;
  msg_joint_state.name.data[0] = micro_ros_string_utilities_set(msg_joint_state.name.data[0], "left_wheel_joint");
  msg_joint_state.name.data[1] = micro_ros_string_utilities_set(msg_joint_state.name.data[1], "right_wheel_joint");

  // msg_joint_cmd.header.frame_id = micro_ros_string_utilities_set(msg_joint_cmd.header.frame_id, "");
  // msg_joint_cmd.name.size = msg_joint_cmd.position.size = msg_joint_cmd.velocity.size = 2;
  // msg_joint_cmd.name.data[0] = micro_ros_string_utilities_set(msg_joint_cmd.name.data[0], "left_wheel_joint");
  // msg_joint_cmd.name.data[1] = micro_ros_string_utilities_set(msg_joint_cmd.name.data[1], "right_wheel_joint");


  // msg_pub.data = 0;
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  myICM.begin(WIRE_PORT, AD0_VAL);
  Serial.println("Finished initialization.");
}

void loop() {
  int t = millis(); 
  // delay(100);
  syncTime();
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  
  // Serial.printf("Pulse L: %i\tPulse R: %i\n\r", stepper_l->getCurrentPosition(), stepper_r->getCurrentPosition()); 

  vTaskDelay(10);  

}