#include "wpose.h"
#include <ros/ros.h>
#include <angles/angles.h>

// Static variables
double wpose::vSigma_    = 1.0;
double wpose::wSigma_    = 0.1;

wpose::wpose()
{
  set(0.0, 0.0, 0.0, 0.0);
}

wpose::wpose(const double &X, const double &Y, const double &Theta, const double &W)
{
  set(X, Y, Theta, W);
}

void wpose::move(const double &v, const double &w, const double &dt)
{
  P[0] += (v + Ran.gaussian(0.0, vSigma_)) * dt * cos(P[2]);  
  P[1] += (v + Ran.gaussian(0.0, vSigma_)) * dt * sin(P[2]); 
  P[2] +=  (w + Ran.gaussian(0.0, wSigma_)) * dt;
  P[2] = atan2(sin(P[2]), cos(P[2]));
}
