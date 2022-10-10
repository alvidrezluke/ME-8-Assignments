// File:          PanTiltV2.cpp
// Date:          10/10/2022
// Description:   A basic program to rotate the pan and tilt camera to the apple.
// Author:        Luke Alvidrez

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>

#define TIME_STEP 50
#define MAX_SPEED 2

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>

using namespace webots;
using namespace std;



double calc_theta(double theta_initial, double theta_final, double time, double time_initial, double time_final) {
  double theta = theta_initial + (((theta_final - theta_initial) / 2) * (1-cos(M_PI * ((time - time_initial) / (time_final - time_initial)))));
  return theta;
}

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  
  // Initialize and setup motors
  Motor *panMotor = robot->getMotor("pan");
  Motor *tiltMotor = robot->getMotor("tilt");
  
  Camera *cam = robot->getCamera("camera");
  cam->enable(TIME_STEP);
  
  double Apan  = 1.0;
  double Atilt = 0.5;
  double init_pan_pos = 0.0;
  double init_tilt_pos = 0.0;
  
  vector<double> vec_pan_pos;
  vector<double> vec_tilt_pos;

  // Main loop:
  while (robot->step(TIME_STEP) != -1) {
    double t = robot->getTime();
    if (t >= 1 && t <= 3) {
      double new_pan_pos = calc_theta(init_pan_pos, -1 * (M_PI / 4), t, 1.0, 3.0);
      double new_tilt_pos = calc_theta(init_tilt_pos, -1 * (M_PI / 6) + 0.2, t, 1.0, 3.0);
      cout << "Time: " << t << "New Pan Angle: " << new_pan_pos << "New Tilt Angle: " << new_tilt_pos << endl;
      vec_pan_pos.push_back(new_pan_pos);
      vec_tilt_pos.push_back(new_tilt_pos);
      panMotor->setPosition(new_pan_pos);
      tiltMotor->setPosition(new_tilt_pos);
    } else if (t >= 4) {
      break;
    } else {
      continue;
    };
  };
  
  cout << "Pan: [ ";
  for (int i = 0; i < vec_pan_pos.size(); i++) {
    cout << vec_pan_pos[i] << ", ";
  }
  cout << "]" << endl;
  
  cout << "Tilt: [ ";
  for (int i = 0; i < vec_tilt_pos.size(); i++) {
    cout << vec_tilt_pos[i] << ", ";
  }
  cout << "]" << endl;
  
  delete robot;
  return 0;
}
