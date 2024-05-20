
#ifndef stateSpace_h
#define stateSpace_h

#include <Arduino.h>
#include <BasicLinearAlgebra.h>

namespace StateSpace{
  BLA::Matrix<4, 1> stateEstimates(BLA::Matrix<4, 1> x, BLA::Matrix<2, 1> u);

  BLA::Matrix<1, 1> output(BLA::Matrix<4, 1> x, BLA::Matrix<2, 1> u);
};
#endif
