#include "Driver.hpp"

Driver::Driver(uint8_t address, int amps, int micro) : driver(&SERIAL_PORT, R_SENSE, address)
{
// source of magic numbers: TMCStepper libraries
  driver.begin();
  driver.toff(0);
  driver.blank_time(24);
  driver.hysteresis_start(1);
  driver.hysteresis_end(12);
  driver.rms_current(amps, 0.01);
  driver.microsteps(micro);
  driver.en_spreadCycle(false);
}