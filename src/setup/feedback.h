
#ifndef feedback_h
#define feedback_h

#include <Arduino.h>
#include <BasicLinearAlgebra.h>

namespace Feedback{
  BLA::Matrix<1, 1> output(BLA::Matrix<4, 1> x);
};
#endif
