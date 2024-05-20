
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include "feedback.h"

BLA::Matrix<1, 1> Feedback::output(BLA::Matrix<4, 1> x){
  BLA::Matrix<1, 4> K = {-11.5443,   57.7947,   -3.2832,    4.9444};

  BLA::Matrix<1, 1> u = -K*x;

  return u;
}
