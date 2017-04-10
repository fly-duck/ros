#define PARTICLES_C

/**********************
     Include Files
 **********************/

#include "pf.h"
#include <fstream>
#include <sstream> // stringstream
#include <string>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h> // YAML parser
#include <boost/foreach.hpp> // BOOST_FOREACH
#include <angles/angles.h> // normalize_angle, normalize_angle_positive

#define EPS 0.00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001

random_numbers::RandomNumberGenerator Ran(123);

pf::pf (const unsigned int &N, const std::string &fname, 
  const double &sensor_accuracy,
  const double &processed_scans_number,
  const bool &consider_inf_scans,
  const double &vSigma, 
  const double &wSigma):
    sensor_accuracy_(sensor_accuracy),
    processed_scans_number_(processed_scans_number),
    consider_inf_scans_(consider_inf_scans)
{
  // Load occupancy grid
  loadMap(fname);

  // Instantiate N particles uniformly inside the free space (white) of the occupancy grid
  if (N > 0)
  {
    pf_size_ = N;
    double weight = 1.0 / pf_size_;
    Sam_.reserve(pf_size_);
    cv::Point2i pixel;
    cv::Point2d point;
    double theta;
    for (unsigned int i = 0; i < pf_size_; ++i)
    {
      // TODO Uniform initialization of particles over the freespace
      // Hint : there are a set of functions to transform the world point to pixel and viceversa
      // (convertPixelToWorlDPoint(...), ...)
      // Hint : isOutsideGridFreespace can be used for checking whether a pixel is in the freespace or not  
      //////////////////// ANSWER CODE BEGIN //////////////////////
      do {
          point = cv::Point2d( Ran.uniformReal(x_min_, x_max_),
              Ran.uniformReal(y_min_, y_max_));
          pixel = convertWorldPointToPixel(point.x, point.y);
      } while(isOutsideGridFreespace(pixel));
      theta = Ran.uniformReal(0, 2*M_PI);

      //////////////////// ANSWER CODE END //////////////////////      

      wpose P(point.x, point.y, theta, weight);
      Sam_.push_back(P);
    }
    Sam_[0].setvSigma(vSigma);
    Sam_[0].setwSigma(wSigma);
  }
  else
  {
    pf_size_ = 0;
  }
}


pf::~pf(void) {
  Sam_.clear();
}

void pf::loadMap(const std::string &fname)
{
  // Reading YAML file
  YAML::Node config = YAML::LoadFile(fname.c_str());
  
  // Reading the image file
  obstacles_ = cv::imread(config["image"].as<std::string>(), CV_LOAD_IMAGE_GRAYSCALE);

  // Resolution assumed to be meters
  resolution_ = config["resolution"].as<double>(); 

  if (!obstacles_.data) 
  {
    std::cout <<  "Could not open or find the image" << std::endl ;
    //exit(0);
    return;
  }
  
  // Size of the map (in pixels)
  map_ = obstacles_.clone();
  map_x_ = obstacles_.cols;
  map_y_ = obstacles_.rows;
  map_ = cv::Mat(map_x_, map_y_, CV_8UC1);

  // Boundaries of the world (meters)
  x_min_ = (-map_x_ / 2.0) * resolution_;
  x_max_ = (map_x_ / 2.0) * resolution_;
  y_min_ = (-map_y_ / 2.0) * resolution_;
  y_max_ = (map_y_ / 2.0) * resolution_;

  ROS_INFO_STREAM(map_x_ << " " << map_y_ <<  " " << x_min_ << " " << x_max_ << " " << y_min_ << " " << y_max_);
  cv::namedWindow("Map", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
}

void pf::draw() 
{
  int key = 0;
  if (obstacles_.empty())
  {
    std::cout << "Empty obstacle map" << std::endl;
    map_ = cv::Scalar(255);
  }
  else
  {
    cv::cvtColor(obstacles_, map_, CV_GRAY2RGB);
  }

  for (unsigned int i = 0; i < pf_size_; ++i)
  {
    if (Sam_[i].w() > 0.0)
      drawParticle(Sam_[i], 255);
  }
  cv::imshow("Map", map_);
  key = cv::waitKey(1000 / 300); // Obtain keypress from user; wait at most N milliseconds
  if (key == 'q' || key == 'Q')
  {
    exit(0);
  } 
}

void pf::drawParticle(wpose P, const int &value)
{ 
  if(isOutsideWorldBoundingBox(P.x(), P.y())){
	  return;
  }
  cv::Point2i pixel = convertWorldPointToPixel(P.x(), P.y());
  cv::Vec3b color = map_.at<cv::Vec3b>(pixel);
  color[0]=0;
  color[1]=0;
  color[2]=value;
  if (!isOutsideGridFreespace(pixel))
  {
      map_.at<cv::Vec3b>(pixel) = color;
  }
}

void pf::move(const double &v, const double &w, const double &dt)
{
  for (unsigned int i = 0; i < pf_size_; i++)
  {
    Sam_[i].move(v, w, dt);
  }
}


void pf::update() 
{
  cv::Point2i px;
  for(unsigned int i = 0; i < pf_size_; ++i)
  {
    px = convertWorldPointToPixel(Sam_[i].x(), Sam_[i].y());
    if (isOutsideGridFreespace(px)) 
    {
      Sam_[i].setW(0.0);
    }
  }
  Normalize();
}

//particle_filter_->update_uniform(msg->ranges, msg->angle_min, msg->angle_increment, msg->range_max);
void pf::update_uniform(const std::vector<float> &R, const double &angle_min, const double &angle_increment, const double &sensor_range)
{
  // TODO: Update step according to the measurements
  // Hint: Use an algorithm to draw a line for computing the expected measurement
  //////////////////// ANSWER CODE BEGIN //////////////////////
  
    int step = R.size()/processed_scans_number_;
    for(unsigned int i = 0; i < pf_size_; ++i) {
        double w = 0;
        for(unsigned int currIndex = 0; currIndex < R.size(); currIndex +=step){
            double angle = angle_min + currIndex*angle_increment;
            if(processed_scans_number_ == 1) {
                currIndex = R.size()/2;
                angle = 0;
            }
            double sens_read_value = R[currIndex]; //the actua value received from the sensor
            w += Sam_[i].w() * gaussianWeight(sens_read_value, sensor_accuracy_, sense(Sam_[i], angle, sensor_range));
        }
        Sam_[i].setW(w/double(processed_scans_number_));
    }

  //}
  //////////////////// ANSWER CODE END //////////////////////

  Normalize();
}
double pf::sense(wpose particle, double angle, double sensor_range)
{
      float x = particle.x();
      float y = particle.y();
      float theta = (particle.theta() + angle);//%(2 *M_PI);
      cv::Point2d starting_point(x, y);
      
      int step = 1;

      cv::Point2d next_point = PointGivenStartingPointAngleDistance(starting_point, theta, step*resolution_);

      cv::Point2i px = convertWorldPointToPixel(next_point.x, next_point.y);
      double dist = 0;
      while(!isOutsideGridFreespace(px) || dist <= sensor_range) 
      {
          ++step;
          next_point = PointGivenStartingPointAngleDistance(starting_point, theta, step*resolution_);
          px = convertWorldPointToPixel(next_point.x, next_point.y);
          dist = EuclideanDistance(starting_point, next_point);
      }
      return dist;
}


void pf::Normalize()
{
  double W = TotalWeight();
  if (fabs(W - 1.0) > EPS)
  {
    for (unsigned int i = 0; i < pf_size_; i++)
      Sam_[i].setW((Sam_[i].w()) / W);
  }
}

void pf::Resample(void)
{
  //std::vector<wpose> BackUpSam=Sam_;
  Normalize();
  //StatWeight("Before Resampling");
  Resample1();
}

void pf::Resample1(void) 
{
  //   Particles::Resample1  samples from a set weighted by m in linear time
  //   Can be used for the resampling step in Condensation algorithm
  //   Algorithm found in Carpenter, Clifford and Fernhead:
  //   "An Improved Particle Filter for Non-linear Problems"

//       % generate N+1 exponentially distributed variables t_0,...,T_N;
//       t = -log(rand(1,N+1));
//       % calculate the running totals T_j=sum_{l=0}^j t_l;
//       T = cumsum(t);
//       % normalize T to TN;
//       TN = T/T(N+1);
//       i=1; j=1; 
//       S=choose(Q,TN',i,j);
  std::vector<double> Q; // Cumulative sum of the Weights.
  std::vector<double> TN; // A vector of Random Numbers...
  Q.reserve(pf_size_);
  TN.reserve(pf_size_+1);
  Q.push_back(Sam_[0].w());
  double C = Sam_[0].w();
  double CTN = -log(Ran.uniform01());
  TN.push_back(CTN);
  for (unsigned int i = 1; i < pf_size_; ++i)
  {
    C += Sam_[i].w();
    Q.push_back(C);
    CTN += -log(Ran.uniform01());
    TN.push_back(CTN);
    //ROS_INFO_STREAM(C);
  }
  CTN += -log(Ran.uniform01());
  TN.push_back(CTN);

  //for (unsigned int i = 0; i <= pf_size_; ++i) // Normalize TN so TN[pf_size_+1]=1.
  //  TN[i] /= TN[pf_size_];

  std::vector<wpose> NewSam;
  NewSam.reserve(pf_size_);
  // Select from TN 
  unsigned int i = 0, j = 0;
  while (i < pf_size_)
  {
    //ROS_INFO_STREAM(j);
    if (TN[i] / TN[pf_size_] < Q[j])
    {
      //ROS_INFO_STREAM(Sam_[j]);
      NewSam.push_back(Sam_[j]);
      NewSam[i].setW(1.0 / pf_size_);
      ++i;
    }
    else 
      ++j;
  }
  Sam_ = NewSam;
}

void pf::Resample2()
{
  unsigned int count = 0, mcnt = 0, pcnt = 0, dcnt = 0, ucnt = 0;
  std::vector<wpose> NewSam;
  for (unsigned int k = 0; k < pf_size_; ++k) 
    Sam_[k].setW(sqrt(Sam_[k].w()));
  Normalize();
  for (unsigned int k = 0; k < pf_size_; ++k)
  {
    unsigned int Kweight = (int)(Sam_[k].w() * pf_size_);
    if (Kweight > 1)
    {
      for (unsigned int l = 0; l < Kweight; l++)
      {
	      NewSam.push_back(Sam_[k]);
	      NewSam[count].setW(1.0 / ((double)pf_size_));
      	if(++count >= pf_size_)
      	  break;
      }
      ++mcnt;
    }
    else 
    { 
      if (Ran.uniform01() < Kweight)
      {
      	NewSam.push_back(Sam_[k]);
      	NewSam[count].setW(1.0 / ((double)pf_size_));
	      ++count;
	      ++pcnt;
      }
      else
        ++dcnt;
    }
    if (count >= pf_size_)
      break;
  }
  if (count < pf_size_ - 1)
  {
    for (unsigned int l = count; l < pf_size_; ++l)
    {
      NewSam.push_back(Sam_[Ran.uniformInteger(0, pf_size_ - 1)]);
      NewSam[l].setW(1.0 / ((double)pf_size_));
      ++ucnt;    
    }
  }
  cout << " mcnt: " << mcnt << " pcnt: " << pcnt
       << " dcnt: " << dcnt << " ucnt: " << ucnt << endl;
  Sam_ = NewSam;
}

void pf::Resample3(void)
{
  unsigned int count = 0, mcnt = 0, pcnt = 0, dcnt = 0, ucnt = 0;
  std::vector<wpose> NewSam;
  for (unsigned int k = 0; k < pf_size_; ++k)
  {
    unsigned int Kweight = (int)(Sam_[k].w() * pf_size_);
    if (Kweight > 1)
    {
      for (unsigned int l = 0; l < Kweight; l++)
      {
        NewSam.push_back(Sam_[k]);
      	//NewSam.insert(NewSam.end(), Sam_[k]);

	      NewSam[count].setW(1.0 / ((double)pf_size_));
	      if (++count >= pf_size_) 
	        break;
      }
      ++mcnt;
    }
    else 
    { 
      if (Ran.uniform01() < Kweight)
      {
          NewSam.push_back(Sam_[k]);
	      //NewSam.insert(NewSam.end(),Sam_[k]);
	      NewSam[count].setW(1.0/((double )pf_size_));
	      ++count;
	      ++pcnt;
      }
      else 
        ++dcnt;
    }
    if (count >= pf_size_)
      break;
  }
  //std::vector<wpose> UNewSam;
  if (count < pf_size_ - 1)
  {
    for (unsigned int l = count; l < pf_size_; ++l)
    {
      //NewSam.insert(NewSam.end(), Sam_[Ran.uniformInteger(0, pf_size_ - 1)]);
      NewSam.push_back(Sam_[Ran.uniformInteger(0, pf_size_ - 1)]);
      NewSam[l].setW(1.0 / ((double)pf_size_));
      ++ucnt;
    }
  }
  cout << " mcnt: " << mcnt << " pcnt: " << pcnt
       << " dcnt: " << dcnt << " ucnt: " << ucnt << endl;
  Sam_ = NewSam;
}

void pf::Resample4()
{
  // Select with Replacement N times. Create  Q a cumulative sum vector of 
  // the weights and P a sorted vector with random values uniformly 
  // distributed in [0,1]. Use a while loop to run through P and select 
  // from Q.

  std::vector<double> P; // An vector of Random Numbers...
  P.reserve(pf_size_ + 1);
  for (unsigned int i = 0; i <= pf_size_; ++i)
    P.push_back(Ran.uniform01());
  sort(P.begin(), P.end());   // Sorted

  std::vector<double> Q; // Cumulative sum of the Weights.
  Q.reserve(pf_size_);
  Q.push_back(Sam_[0].w());
  double C = Sam_[0].w();
  for (unsigned int i = 1; i < pf_size_; ++i)
  {
    C += Sam_[i].w();
    Q.push_back(C);
  }

  std::vector<wpose> NewSam;
  NewSam.reserve(pf_size_);

  // Select from Q 

  unsigned int i = 0, j = 0;
  while (i < pf_size_)
  {
    if(P[i] < Q[j])
    {
      NewSam.push_back(Sam_[j]);
      NewSam[i].setW(1.0 / pf_size_);
      ++i;
    }
    else
      ++j;
  }
  Sam_ = NewSam;
}

int pf::ESS(double &cv2)
{
  cv2 = 0.0;
  for (unsigned int i = 0; i < pf_size_; ++i)
    cv2 += (Sam_[i].w() * pf_size_ - 1.0) * (Sam_[i].w() * pf_size_ - 1.0);
  cv2 /= pf_size_;
  return floor(pf_size_ / (1.0 + cv2));
}

void pf::StatWeight(const char *s="")
{
  double mean, max, min, sum, std = 0;
  max = Sam_[0].w();
  min = Sam_[0].w();
  sum = Sam_[0].w();
  for (unsigned int i = 1; i < pf_size_; ++i) 
  {
    double w = Sam_[i].w();
    if (max < w)
      max = w;
    if(min > w)
      min = w;
    sum += w;
  }
  mean = sum / pf_size_;
  for (unsigned int i = 0; i < pf_size_; ++i)
    std += (Sam_[i].w() - mean) * (Sam_[i].w() - mean);
  cerr << "STD CALCULATIONS " << std;
  std /= (double)(pf_size_ - 1);
  cerr << " "<< std << " ";
  std = sqrt(std);
  cerr << std << endl;

  cout << "Statistics for the weights" << endl;
  cout <<"Max       Min       Sum       Mean       Std" << endl;
  cout << max << " " << min << " " << sum << " " << mean << " " << std << endl;
  vector<wpose>::iterator Mn = min_element(Sam_.begin(), Sam_.end());
  vector<wpose>::iterator Mx = max_element(Sam_.begin(), Sam_.end());
  wpose Mxp = *Mx;
  wpose Mnp = *Mn;

  cout << "--------------" << s << "------------------------" << endl;
  cout << "Poses" << endl;
  cout << "Mean          :" << Mean() <<endl;
  cout << "WeightedMean  :" << WeightedMean() <<endl;
  cout << "Best Particle :" << Mxp << endl;
  cout << "Sam_[0]        :" <<Sam_[0] << endl;
  cout << "--------------------------------------" << endl;
}

void pf::StatWeight(std::string path, int i)
{
  std::ofstream outputFile;
  outputFile.open(std::string(path + std::string("stat") + std::to_string(i) + std::string(".txt")), std::ios::trunc);

  double mean, max, min, sum, std = 0;
  max = Sam_[0].w();
  min = Sam_[0].w();
  sum = Sam_[0].w();
  for (unsigned int i = 1; i < pf_size_; ++i) 
  {
    double w = Sam_[i].w();
    if (max < w)
      max = w;
    if(min > w)
      min = w;
    sum += w;
  }
  mean = sum / pf_size_;
  for (unsigned int i = 0; i < pf_size_; ++i)
    std += (Sam_[i].w() - mean) * (Sam_[i].w() - mean);
  cerr << "STD CALCULATIONS " << std;
  std /= (double)(pf_size_ - 1);
  cerr << " "<< std << " ";
  std = sqrt(std);
  cerr << std << endl;

  outputFile << "Statistics for the weights" << endl;
  outputFile <<"Max       Min       Sum       Mean       Std" << endl;
  outputFile << max << " " << min << " " << sum << " " << mean << " " << std << endl;
  vector<wpose>::iterator Mn = min_element(Sam_.begin(), Sam_.end());
  vector<wpose>::iterator Mx = max_element(Sam_.begin(), Sam_.end());
  wpose Mxp = *Mx;
  wpose Mnp = *Mn;

  
  outputFile << "Poses" << endl;
  outputFile << "Mean          :" << Mean() <<endl;
  outputFile << "WeightedMean  :" << WeightedMean() <<endl;
  outputFile << "Best Particle :" << Mxp << endl;
  outputFile << "Sam_[0]       :" << Sam_[0] << endl;

  outputFile.close();
}


wpose pf::Mean(void)
{
  double P[4] = {0, 0, 0, 0};
  for(unsigned int i = 0; i < pf_size_; ++i)
  {
    P[0] += Sam_[i].x();
    P[1] += Sam_[i].y();
    P[2] += Sam_[i].theta();
    P[3] += Sam_[i].w();
  }

  P[0] /= pf_size_;
  P[1] /= pf_size_;
  P[2] /= pf_size_;
  P[3] /= pf_size_;
  return (wpose(P[0], P[1], P[2], P[3]));
}


void pf::WCov(double C[3][3]) 
{
    wpose WMP = WeightedMean();
    double WM[3] = {WMP.x(), WMP.y(), WMP.theta()};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            C[i][j] = 0.0;

    for (unsigned int i = 0; i < pf_size_; ++i)
    {
        double S[4] = {Sam_[i].x(), Sam_[i].y(), Sam_[i].theta(), Sam_[i].w()};
        for (int k = 0; k < 3; ++k) 
        {
            C[k][k] += Sam_[i].w() * (WM[k]-S[k]) * (WM[k]-S[k]);
            for(int l = k + 1; l < 3; ++l)
                C[k][l] += Sam_[i].w() * (WM[k]-S[k]) * (WM[l]-S[l]);
        }
    }
    for (int k = 0; k < 3; ++k) 
        for (int l = k + 1; l < 3; ++l)
            C[l][k] = C[k][l];

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
            cout << C[i][j] << " ";
        cout << endl;
    }
}

wpose pf::WeightedMean(void)
{
  double P[4] = {0, 0, 0, 0};
  for (unsigned int i = 0; i < pf_size_; ++i)
  {
    P[0] += Sam_[i].x()     * Sam_[i].w();
    P[1] += Sam_[i].y()     * Sam_[i].w();
    P[2] += Sam_[i].theta() * Sam_[i].w();
    P[3] += Sam_[i].w();
  }
  return (wpose(P[0], P[1], P[2], P[3]));
}

wpose pf::Best()
{
  return(Sam_[0]);
}


double pf::TotalWeight()
{
  double S = 0.0;
  for (unsigned int i = 0; i < pf_size_; i++)
    S += Sam_[i].w();
  return(S);
}

double pf::gaussianWeight(const double &mu, const double &sigma, const double &value)
{
  double difference = value - mu;
  return exp(-(difference * difference)/ (2 * sigma * sigma)) / (sigma * sqrt(2 * M_PI));
}

unsigned int pf::BinSearch(double *C, double V)
{
  unsigned int i = 0, j = pf_size_ - 1;
  while (1)
  {
    if (i == j)
      return(i);
    unsigned int mid = (i + j) / 2; 
    if (V < C[mid])
      j = mid;
    else
      i = mid + 1;
  }    
}


cv::Point2i pf::convertWorldPointToPixel(const double &X, const double &Y)
{
  return cv::Point2i(round((X - x_min_) / resolution_), round((y_max_ - Y) / resolution_)); 
}

cv::Point2d pf::convertPixelToWorldPoint(const cv::Point2i &pixel)
{
  return cv::Point2d(x_min_ + (pixel.x * resolution_), y_max_ - (pixel.y * resolution_));
}

// Check whether the point is outside the world bounding box
bool pf::isOutsideWorldBoundingBox(const double &x, const double &y)
{
  return (x < x_min_ || x > x_max_ || y < y_min_ || y > y_max_);
}

// Check whether the pixel is into an obstacle or outside freespace
bool pf::isOutsideGridFreespace(const cv::Point2i &px)
{
   return (px.x < 0 || px.x >= map_x_ || px.y < 0 || px.y >= map_y_ || obstacles_.at<uchar>(px.y, px.x) == 0);
   
}

double pf::EuclideanDistance(const cv::Point2d &p1, const cv::Point2d &p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return sqrt(dx * dx + dy * dy);
}

cv::Point2d pf::PointGivenStartingPointAngleDistance(const cv::Point2d &starting_point, const double &angle, const double &distance)
{
  return cv::Point2d(starting_point.x + distance * cos(angle), starting_point.y + distance * sin(angle)); 
}

