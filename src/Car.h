#include "helper.h"


#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

/*
 * Class is responsible for Car maintenance
 */
class Car {
public:
  Car();

  explicit Car(const std::vector<double> &f);

  double distanceTo(double s2);

  bool sameLane(int lane);

  double getVS();

  double getS();

  operator bool() const;

private:
  int id;
  double x, y, vx, vy, s, d;
};


#endif //PATH_PLANNING_CAR_H
