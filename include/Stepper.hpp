#pragma once
#include <FastAccelStepper.h>

class Stepper123
{
public:
  Stepper123(FastAccelStepperEngine* eng);
  FastAccelStepper* stepper = NULL;
  FastAccelStepperEngine* engine;
//   int pizzS;
// private:

};

Stepper123::Stepper123(FastAccelStepperEngine* eng)
{
    engine = eng;
    // pizzS+1;
}