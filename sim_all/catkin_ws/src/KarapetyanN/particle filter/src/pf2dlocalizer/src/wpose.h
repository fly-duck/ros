#ifndef WPOSE_H
#define WPOSE_H
#include <cmath>
#include <iostream>
#include <cfloat>
#include <random_numbers/random_numbers.h>

using namespace std;
extern random_numbers::RandomNumberGenerator Ran;

// Class for handling particles
class wpose 
{
  public:
    wpose();
    wpose(const double &X, const double &Y, const double &Theta, const double &W);
    ~wpose() {}

    double x() { return P[0]; }
    double y() { return P[1]; }
    double theta() { return P[2]; }
    double w() { return P[3]; }
    void setX(const double &X) { P[0] = X; } 
    void setY(const double &Y) { P[1] = Y; } 
    void setTheta(const double &Theta) { P[2] = Theta; } 
    void setW(const double &W) { P[3] = W; } 
    void set(const double &X, const double &Y, const double &Theta, const double &W) { P[0] = X; P[1] = Y; P[2] = Theta; P[3] = W; }
    void setvSigma(const double &vSigma) { vSigma_ = vSigma; }
    void setwSigma(const double &wSigma) { wSigma_ = wSigma; }

    void move(const double &v, const double &w, const double &dt); 

    friend bool operator<(const wpose& a, const wpose& b) 
    { return a.P[3] < b.P[3]; }
    friend bool operator>(const wpose& a, const wpose& b) 
    { return a.P[3] > b.P[3]; }
    friend bool operator==(const wpose& a, const wpose& b) 
    { return fabs((a.P[3])-(b.P[3])) <= (DBL_EPSILON)*fabs((a.P[3])); }
    friend bool operator!=(const wpose& a, const wpose& b) 
    { return a.P[3] != b.P[3]; }
    friend ostream &operator<<(ostream &stream, wpose const &p)
    { 
      stream <</* std::setprecision(5) << */ p.P[0] <<" "<<  p.P[1] << " " << p.P[2]<< " ";
      stream/* <<std::setprecision(5) */ << p.P[3];
      return stream;
    }
    friend istream &operator>>(istream &is, wpose  &p) 
    {
      if (is >> p.P[0] >> p.P[1] >> p.P[2] >> p.P[3])
      {
      }
      return is; 
    }

  private:
    double P[4];
    static double vSigma_;
    static double wSigma_;
};
#endif
