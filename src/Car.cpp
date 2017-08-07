#include <vector>
#include "Car.h"
#include <cmath>

using namespace std;

/*
 * Implementation of Car.h
 */

Car::Car(const std::vector<double> &f) {
  id = (int) f[0];
  x = f[1];
  y = f[2];
  vx = f[3];
  vy = f[4];
  s = f[5];
  d = f[6];
}

Car::Car() : id(-1) {}

double Car::distanceTo(double s2) {
  double s_dist;
  if (s > s2) {
    s_dist = s - s2;
  } else {
    s_dist = (maximumS - s2) + s;
  }
  return s_dist;
}

bool Car::sameLane(int lane) {
  return abs(d - laneToD(lane)) <= 2.0;
}

double Car::getVS() {
  return sqrt(vx * vx + vy * vy);
}

Car::operator bool() const {
  return id != -1;
}

double Car::getS() {
  return s;
}
