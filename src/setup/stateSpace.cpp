
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include "stateSpace.h"

BLA::Matrix<4, 1> StateSpace::stateEstimates(BLA::Matrix<4, 1> x, BLA::Matrix<2, 1> u){
 
  BLA::Matrix<4, 4> A = {-0.1898,   -0.3750,    0.0050,   -0.0033,
    0.2348,   -0.8033,    0.0022,   -0.0005,
  -77.3460,  -18.5609,    1.3018,   -1.1167,
    6.5711, -102.2421,    0.7700,   -0.6075};

  BLA::Matrix<4, 2> B = {1.1931,    0.3577,
   -0.2312,    1.7864,
   79.7883,    6.5712,
   -3.9563,   90.3835};


  BLA::Matrix<4, 1> xd = A*x + B*u;

  return xd;
}

BLA::Matrix<1, 1> StateSpace::output(BLA::Matrix<4, 1> x, BLA::Matrix<2, 1> u){

  BLA::Matrix<1, 4> C = {11.5443,  -57.7947,    3.2832,   -4.9444};

  BLA::Matrix<1, 2> D = {0,   0};

  BLA::Matrix<1, 1> y = C*x + D*u;

  return y;
}



