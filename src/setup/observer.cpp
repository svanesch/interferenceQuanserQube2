
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include "observer.h"

BLA::Matrix<4, 1> Observer::stateEstimates(BLA::Matrix<4, 1> x, BLA::Matrix<2,1> y, BLA::Matrix<1, 1> u){

  BLA::Matrix<4,2> L = {1.4039,    0.0130,
                        -0.0047,    1.4164,
                        81.0695,    2.6459,
                        -2.2762,   85.7399};

  BLA::Matrix<4, 4> A = {0.9994,    0.0047,    0.0099,    0.0000,
                        -0.0007,    1.0112,   -0.0001,    0.0101,
                        -0.1233,    0.9465,    0.9728,    0.0047,
                        -0.1319,    2.2480,   -0.0291,    1.0111};

  BLA::Matrix<4, 1> B = {0.0014,
                        0.0015,
                        0.2809,
                        0.3006};

   BLA::Matrix<2, 4> C = {0.9997,    0.0024,    0.0049,    0.0000,
                          -0.0003,    1.0056,   -0.0001,    0.0050};

  BLA::Matrix<4, 1> xhat = (A - L*C)*x + B*u + L*y;

  return xhat;
}




