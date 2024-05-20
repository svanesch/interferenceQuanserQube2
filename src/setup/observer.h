
#ifndef observer_h
#define observer_h

#include <Arduino.h>
#include <BasicLinearAlgebra.h>

namespace Observer{
  BLA::Matrix<4, 1> stateEstimates(BLA::Matrix<4, 1> x, BLA::Matrix<2,1> y, BLA::Matrix<1, 1> u);
};
#endif
