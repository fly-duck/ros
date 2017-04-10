#ifndef PF_H
#define PF_H
#include "wpose.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <ctime>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
  
using namespace std;

class pf {

 public:
  // Constructors for different initial conditions
  // Instantiate N particles uniformly inside  [X1,Y1,X2,Y2]
  pf(const unsigned int &N, const std::string &fname, 
    const double &sensor_accuracy, 
    const double &processed_scans_number, 
    const bool &consider_inf_scans,
    const double &vSigma, 
    const double &wSigma); 
  
  // Destructor
  ~pf(void);
  
  // Load map
  void loadMap(const std::string &fname);

	// Draw functions
  void draw();
  void drawParticle(wpose P, const int &value);

  // Move the particle
  void move(const double &v, const double &w, const double &dt);

  // Update according to whether particles fall outside the freespace
  void update();

  // Update according to scans
  void update_uniform(const std::vector<float> &R, const double &angle_min, const double &angle_increment, const double &sensor_range);

  // Normalize the weights
  void Normalize();

  // Resample functions
  void Resample(void);
  void Resample1(void);
  void Resample2(void);
  void Resample3(void);
  void Resample4(void);
  
  // Effective sample size
  int ESS(double &cv2);

  // Statistics for the weights
  void StatWeight(const char *s);
  void StatWeight(std::string path, int i);
  
  // Estimate the position of the robot
  wpose Mean(void);
  void WCov(double C[3][3]);
  wpose WeightedMean(void);
  wpose Best(void);

  // Functions to get parameters of the particle filter
  unsigned int size() const {return pf_size_;}
  
  // Find the point which lies on the line segment that starts from starting_point, with angle and distance length
  cv::Point2d PointGivenStartingPointAngleDistance(const cv::Point2d &starting_point, const double &angle, const double &distance);

  // Parameters
  double sensor_accuracy_; // sigma of the laser
  double processed_scans_number_; // percentage of scans to process
  bool consider_inf_scans_; // flag to consider or not inf scans

 private:
  // <N> function that draws line to estimate particle weight
  // returns the distance of a particle from the landmarks
  // with a given direction
  double sense(wpose particle, double angle, double sensor_range);

  // Get the sum of the weights of the particles
  double TotalWeight();
  // Get a Gaussian weight
  double gaussianWeight(const double &mu, const double &sigma, const double &value);
  // Get the tail of the Gaussian weight
  double gaussianTailWeight(const double &mu, const double &sigma, const double &value);
  // Binary search
  unsigned int BinSearch(double *C,double V);
  
  // Transformation function from the world coordinates to the grid ones
  cv::Point2i convertWorldPointToPixel(const double &X, const double &Y);
  // Transformation functions from the grid coordinates to the world ones
  cv::Point2d convertPixelToWorldPoint(const cv::Point2i &pixel);
  // Check whether (x,y) is outside world bounding box
  bool isOutsideWorldBoundingBox(const double &x, const double &y);  
  // Check whether px is outside the freespace
  bool isOutsideGridFreespace(const cv::Point2i &px);
  
  // Euclidean distance between two points
  double EuclideanDistance(const cv::Point2d &p1, const cv::Point2d &p2);

  // Occupancy grid
  cv::Mat map_; // Occupancy grid canvas
  cv::Mat obstacles_; // Occupancy grid obstacles canvas

  // Size of the occupancy grid
  int map_x_;
  int map_y_;
  double resolution_; // Resolution of the grid
  
  // Boundaries of the world (meters)
  double x_min_;
  double y_min_;
  double x_max_;
  double y_max_;
  
  // Particles
  unsigned int pf_size_; // Number of particles
  std::vector<wpose> Sam_; // Vector with particles
  
};

#endif
