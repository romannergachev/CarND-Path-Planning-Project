//
// Created by Roman Nergachev on 07/08/2017.
//

#include <cmath>
#include <vector>

using namespace std;

#ifndef PATH_PLANNING_HELPER_H
#define PATH_PLANNING_HELPER_H

//safe distance in meters
#define SAFE_FOLLOWING_DISTANCE 9

#define SAFE_DELTA 15
//speed in meters per second
#define SPEED_LIMIT 20.5
//acceleration max = from 0 to 50 in 20 sec
#define TIME_TO_MAX 5.0

double laneToD(int lane);

vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

double polyEval(const vector<double> &coeffs, double x);
vector<double> derivative(const vector<double> &coeffs);

constexpr double pi() { return M_PI; }

// The max s value before wrapping around the track back to 0
const double maximumS = 6945.554;

const double step = 0.02;

const int trajectoryPoints = 100;
const double trajectoryLength = trajectoryPoints * step;


#endif //PATH_PLANNING_HELPER_H
