#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/temperature.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/battery_state.h>
#include "Driver.hpp"
#include "Stepper.hpp"

#include <geometry_msgs/msg/transform_stamped.h>

#include <ICM_20948.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include <TMC2209.h>

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

  rmw_uros_set_custom_transport(false, (void*)&locator, platformio_transport_open, platformio_transport_close,
                                platformio_transport_write, platformio_transport_read);
}
#endif

#define EXECUTE_EVERY_N_MS(MS, X)                                                                                      \
  do                                                                                                                   \
  {                                                                                                                    \
    static volatile int64_t init = -1;                                                                                 \
    if (init == -1)                                                                                                    \
    {                                                                                                                  \
      init = uxr_millis();                                                                                             \
    }                                                                                                                  \
    if (uxr_millis() - init > MS)                                                                                      \
    {                                                                                                                  \
      X;                                                                                                               \
      init = uxr_millis();                                                                                             \
    }                                                                                                                  \
  } while (0)

#define LED_PIN 4

TickType_t xLastWakeTime;
int prev_time = 0;

int AMPS = 2000;
int micro = 16;
int Maccell = 20000;
int Mspeed = 100;
int MMperRev = 3.14 * 80;
float StepsPerRot = 200;

float Vl;
float Vr;
float prev_L = 0;
float prev_R = 0;
float alpha = 0;
float theta = 0;



Driver driver_l(DRIVER_L_ADDR,AMPS,micro);
Driver driver_r(DRIVER_R_ADDR,AMPS,micro);

// Stepper123 steppert;
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* stepper_l = NULL;
FastAccelStepper* stepper_r = NULL;

HardwareSerial& serial_stream = Serial2;
TMC2209 DRV09_L;
TMC2209 DRV09_R;

// publisher
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
geometry_msgs__msg__Twist msg_sub;
sensor_msgs__msg__JointState msg_joint_cmd;

// publisher and subscriber common
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_timer_t timer;
rcl_timer_t timer_odom;

int64_t time_offset = 0;

unsigned int num_handles = 6;  // 1 subscriber, 1 publisher

#define WIRE_PORT Wire
#define AD0_VAL 1
ICM_20948_I2C myICM;

#define RCCHECK(fn)                                                                                                    \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
      error_loop();                                                                                                    \
    }                                                                                                                  \
  }
#define RCSOFTCHECK(fn)                                                                                                \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
    }                                                                                                                  \
  }
#define _POSIX_TIMERS 0

/// start new driver lib ///
// // Pin configuration
// const int pwmPin = 0;         // Change this to the appropriate pin
// const int inputPin = pwmPin;  // Change this to the appropriate pin
// const int dirPin = 26;
// const int enPin = 25;

// volatile int pulseCount = 0;

// void IRAM_ATTR handleInterrupt()
// {
//   if (digitalRead(dirPin))
//     pulseCount++;
//   else
//     pulseCount--;
// }

// void pwmTaskSetup(void) // stepPin, stepPin, handleCntr_int, enPin
// {
//                                                      // Default frequency is 1 kHz
//   ledcSetup(10, 0, 8);                            // channel 0, 8-bit resolution
//   ledcAttachPin(pwmPin, 10);                        // Attach PWM channel to pin
//   attachInterrupt(digitalPinToInterrupt(inputPin), handleInterrupt, RISING);  // Attach interrupt
//   // digitalWrite(enPin, LOW);
//   pinMode(dirPin, OUTPUT);
//   pinMode(enPin, OUTPUT);
//   // Generate PWM signal with 50% duty cycle
//   ledcWrite(10, 8); // 128
// }

//// End of new driver lib


// Error handle loop
void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    vTaskDelay(500);
  }
}

struct timespec getTime()
{
  struct timespec tp = { 0 };
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

void timer_callback_odom(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {

    int Vbat = analogRead(A0);
    Serial.printf("> Vbat: %f\n", (float)Vbat*(3.3/4095));
    struct timespec time_stamp = getTime();
    msg_odom.header.stamp.sec = time_stamp.tv_sec;
    msg_odom.header.stamp.nanosec = time_stamp.tv_nsec;

    msg_joint_state.header.stamp.sec = time_stamp.tv_sec;
    msg_joint_state.header.stamp.nanosec = time_stamp.tv_nsec;

    // stepper_l->getCurrentSpeedInMilliHz();
    // int mL = DRV09_L.getMicrostepCounter();   
    // int mR = DRV09_R.getMicrostepCounter();
    // int vL = DRV09_L.getInterstepDuration();
    // int vR = DRV09_R.getInterstepDuration();
    int mL = stepper_l->getCurrentPosition();
    int mR = stepper_r->getCurrentPosition();
    int vL = stepper_l->getCurrentSpeedInUs();
    int vR = stepper_r->getCurrentSpeedInMilliHz();

    Serial.printf("vL: %i\t vR: %f\n", vL, (float)vR/(1000*micro*200));
    
    // Serial.printf("vL: %f\t vR: %i\n", (float)(18.75 / (vL*M_PI)), vR);
    // float V[2] = { 18.75 / (vL*M_PI), Vr / (M_PI * 18.75) }; // 243.375*M_PI * 0.08
    float V[2] = {Vl/(M_PI*0.08),Vr/(M_PI*0.08)}; 
    float L = (mL / (micro * StepsPerRot)) * 2 * M_PI;
    float R = (mR / (micro * StepsPerRot)) * 2 * M_PI;

    float dL = (L - prev_L) * 0.04;
    float dR = (R - prev_R) * 0.04;
    prev_L = L;
    prev_R = R;
    float P[2] = { L, R };

    float d = (dL + dR) / 2;
    float d_theta = (dL - dR) / (2 * 0.081);
    alpha = alpha + d_theta / 2;

    float x = cos(theta + d_theta / 2) * d;
    float y = sin(theta + d_theta / 2) * d;

    theta = theta + d_theta;

    float z = sin(theta / 2);
    float w = cos(theta / 2);

    msg_odom.pose.pose.position.x = msg_odom.pose.pose.position.x + x;
    msg_odom.pose.pose.position.y = msg_odom.pose.pose.position.y + y;
    msg_odom.pose.pose.orientation.z = z;
    msg_odom.pose.pose.orientation.w = w;

    for (int i = 0; i < 2; i++)
    {
      msg_joint_state.velocity.data[i] = V[i];
      msg_joint_state.position.data[i] = P[i];
    }

    RCSOFTCHECK(rcl_publish(&publisher_joint_state, &msg_joint_state, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_odom, &msg_odom, NULL));
    // Serial.printf("ODOM:Theta: %f\t Alpha: %f\n", theta, alpha);
  }
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {

    if (myICM.dataReady())
    {
      // Serial.println("Got imu data");
      myICM.getAGMT();
    }
    
    // Serial.println(myICM.accX());
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

bool syncTime()
{
  const int timeout_ms = 1000;
  if (rmw_uros_epoch_synchronized())
  {
    return true;  // synchronized previously
  }
  // get the current time from the agent
  RCCHECK(rmw_uros_sync_session(timeout_ms));
  if (rmw_uros_epoch_synchronized())
  {
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

void subscription_callback(const void* msgin)
{
  geometry_msgs__msg__Twist* msg = (geometry_msgs__msg__Twist*)msgin;
  float V_linx = msg->linear.x;  // m/s to rad/s by dividing by r
  float V_angz = msg->angular.z*2*M_PI;
  // digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);
  Vl = (V_linx + V_angz*(0.162/2));
  Vr = (V_linx - V_angz*(0.162/2));

  int32_t stepsL = (int32_t)(Vl * 200 * micro); // *1.39);
  int32_t stepsR = (int32_t)(Vr * 200 * micro); // *1.39);
  // Serial.printf("steps/s: %i\n", stepsL);

  // DRV09_L.moveAtVelocity(stepsL);
  // DRV09_R.moveAtVelocity(stepsR);
  
  
  if (stepsL < 0)
  {
    stepper_l->runBackward();
  }
  else
  {
    stepper_l->runForward();
  }
  if (stepsR < 0)
  {
    stepper_r->runBackward();
    // digitalWrite(dirPin, LOW);
  }
  else
  {
    stepper_r->runForward();
    // digitalWrite(dirPin, HIGH);
  }
  if ((abs(stepsL) > 10) | (abs(stepsR) > 10))
  {
    
    // driver_l.VACTUAL(Vl);
    stepper_l->setSpeedInHz(abs(stepsL));
    stepper_l->applySpeedAcceleration();
    

    stepper_r->setSpeedInHz(abs(stepsR));
    stepper_r->applySpeedAcceleration();

    // stepper_driver_R.enable();
    digitalWrite(4, HIGH);
  }
  else
  {
    // Serial.println("motor off");
    stepper_l->stopMove();
    stepper_r->stopMove();

    // stepper_driver_R.disable();
    digitalWrite(4, LOW);
  }
}

void setup()
{
  // Configure serial transport
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(A0, INPUT); 

  engine.init();
  // steppert.engine->init();
  stepper_l = engine.stepperConnectToPin(STEP_PIN_L); //steppert.
  stepper_r = engine.stepperConnectToPin(STEP_PIN_R); //steppert.
  stepper_l->setDirectionPin(DIR_PIN_L);
  stepper_r->setDirectionPin(DIR_PIN_R);
  stepper_l->setEnablePin(EN_PIN);
  stepper_r->setEnablePin(EN_PIN);
  stepper_l->setAcceleration(1 * Maccell);
  stepper_r->setAcceleration(1 * Maccell);
  stepper_l->setAutoEnable(true);
  stepper_r->setAutoEnable(true);

  DRV09_L.setup(serial_stream, 256000, TMC2209::SERIAL_ADDRESS_1, 17, 16);
  DRV09_L.setRunCurrent(100); 
  DRV09_L.enableAutomaticCurrentScaling();
  DRV09_L.enableAutomaticGradientAdaptation();
  DRV09_L.enableAnalogCurrentScaling();
  DRV09_L.setMicrostepsPerStep(micro); 
  DRV09_L.setStandstillMode(TMC2209::NORMAL);
  // DRV09_L.enableInverseMotorDirection();
  TMC2209::Settings sett_left = DRV09_L.getSettings();
  if(sett_left.is_setup) {
    Serial.printf("Left Driver all set up\n");
    DRV09_L.enable();
  }
  else{
    Serial.printf("Left Driver not setup.\n Stayning here forever");
    // while(1);
  }
  delay(100);
  DRV09_R.setup(serial_stream, 256000, TMC2209::SERIAL_ADDRESS_3, 17, 16);
  DRV09_R.setRunCurrent(100); 
  DRV09_R.enableAutomaticCurrentScaling();
  DRV09_R.enableAutomaticGradientAdaptation();
  DRV09_R.enableAnalogCurrentScaling();
  DRV09_R.setMicrostepsPerStep(micro); 
  DRV09_R.setStandstillMode(TMC2209::NORMAL);
  // DRV09_R.enableInverseMotorDirection();
  TMC2209::Settings sett_right = DRV09_R.getSettings();
  if(sett_right.is_setup) {
    Serial.printf("Right Driver all set up\n");
    DRV09_R.enable();
  }
  else{
    Serial.printf("Right Driver not setup.\n Stayning here forever");
    // while(1);
  }

  delay(100);

  IPAddress agent_ip(192, 168, 2, 38);  // Laptop IP Address :)
  size_t agent_port = 8888;

  char ssid[] = "HyruleCastle";
  char psk[] = "M3G4M4N_X";

  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(&publisher_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                      TOPIC_PREFIX "imu/data"));

  RCCHECK(rclc_publisher_init_default(
      &publisher_mag, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), TOPIC_PREFIX "imu/mag"));

  RCCHECK(rclc_publisher_init_default(
      &publisher_temp, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), TOPIC_PREFIX "imu/temp"));

  RCCHECK(rclc_publisher_init_default(  // init_default
      &publisher_joint_state, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/joint_states"));

  RCCHECK(rclc_publisher_init_default(&publisher_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                                      TOPIC_PREFIX "odom"));  // diif_cont/odom

  msg_odom.header.frame_id.data = (char*)"odom";
  msg_odom.header.frame_id.size = sizeof("odom");

  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                         "/cmd_vel")); //"/diff_cont/cmd_vel_unstamped"));  //

  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  const unsigned int timer_timeout_odom = 100;
  RCCHECK(rclc_timer_init_default(&timer_odom, &support, RCL_MS_TO_NS(timer_timeout_odom), timer_callback_odom));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_odom));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));

  // initialize measured joint state message memory
  micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), &msg_joint_state,
                                            (micro_ros_utilities_memory_conf_t){});

  // populate fixed message fields - size, frame ID and joint names for measured joint state
  msg_joint_state.header.frame_id = micro_ros_string_utilities_set(msg_joint_state.header.frame_id, "");
  msg_joint_state.name.size = msg_joint_state.position.size = msg_joint_state.velocity.size = 2;
  msg_joint_state.name.data[0] = micro_ros_string_utilities_set(msg_joint_state.name.data[0], "left_wheel_joint");
  msg_joint_state.name.data[1] = micro_ros_string_utilities_set(msg_joint_state.name.data[1], "right_wheel_joint");

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  myICM.begin(WIRE_PORT, AD0_VAL);
  // pwmTaskSetup();
  xLastWakeTime = xTaskGetTickCount();
  Serial.println("Finished initialization.");
}

void loop()
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  syncTime();
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  xTaskDelayUntil(&xLastWakeTime, 10/ portTICK_PERIOD_MS);
}