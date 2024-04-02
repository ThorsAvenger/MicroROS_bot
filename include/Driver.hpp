#pragma once

#include <TMCStepper.h>
#include <FastAccelStepper.h>

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
//#endif 
#define SERIAL_PORT Serial2
#define STALL_VALUE 255
#define DRIVER_L_ADDR 0b11  // 16
#define DRIVER_R_ADDR 0b10  // 64
#define R_SENSE 0.11f

class Driver {
    public: 
    Driver (uint8_t address);

    TMC2209Stepper driver;
};

Driver::Driver(uint8_t address): driver(&SERIAL_PORT,R_SENSE, address){

}
