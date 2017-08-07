#include "BehaviorPlanner.h"
#include "Car.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Behavior planner class implementation
 */
BehaviorPlanner::BehaviorPlanner() {
  sCoeffs = vector<double>();
  dCoeffs = vector<double>();
}

bool BehaviorPlanner::isSafeToSwitch(int lane, double s, double us, const vector<vector<double>> &sensor_fusion) {
  // Iterate over vehicles on the lane of interest
  for (const vector<double> &carDetail: sensor_fusion) {
    Car car(carDetail);
    if (car.sameLane(lane)) {
      // Could it collide in a lane change ?

      // check behind closest car
      if (s - car.getS() > 0 && s - car.getS() < SAFE_FOLLOWING_DISTANCE)
        return false;
      // check behind fast car
      if (s - car.getS() > 0 && s - car.getS() < 2 * SAFE_FOLLOWING_DISTANCE && car.getVS() >= us)
        return false;
      // check in front closest car
      if (car.getS() - s > 0 && car.getS() - s < 5 * SAFE_FOLLOWING_DISTANCE)
        return false;
    }
  }
  return true;
}

void BehaviorPlanner::initState(double s, double d, int previousPathLength) {

  carS = s;
  carD = d;
  if (previousPathLength == 0) {
    //init state
    this->s = s;
    us = 0;
    as = 0;
    this->d = d;
    ud = 0;
    ad = 0;
  } else {
    double deltaTime = (trajectoryPoints - previousPathLength) * step;

    this->s = polyEval(sCoeffs, deltaTime);
    this->d = polyEval(dCoeffs, deltaTime);

    // get velocities and accelerations
    us = polyEval(derivative(sCoeffs), deltaTime);
    as = polyEval(derivative(derivative(sCoeffs)), deltaTime);
    ud = polyEval(derivative(dCoeffs), deltaTime);
    ad = polyEval(derivative(derivative(dCoeffs)), deltaTime);
  }

  // Lap ended
  if (this->s > 6000 && carS < 100) {
    this->s = carS;
    this->d = carD;
  }
}

void BehaviorPlanner::perform(const vector<vector<double>> &sensor_fusion) {

  // Trajectory end configuration
  double resultS, resultVs;
  double resultVd = 0;
  double resultD;

  if (state == CarStates::CHANGE_LEFT || state == CarStates::CHANGE_RIGHT) {
    // Completed switch ??
    int newLane = currentLane + (state == CarStates::CHANGE_RIGHT ? 1 : -1);
    if (abs(laneToD(newLane) - d) < 2.0) {
      currentLane = newLane;
      state = CarStates::KEEP_LANE;
    }
  }

  if (state == CarStates::KEEP_LANE) {

    // Sensor fusion
    double minimalDistanceToTheNearestCar = maximumS;
    Car car_front;
    for (const std::vector<double> &carinfo: sensor_fusion) {
      Car car(carinfo);
      if (car.sameLane(currentLane)) {
        double s_dist = car.distanceTo(s);
        if (s_dist < minimalDistanceToTheNearestCar) {
          car_front = car;
          minimalDistanceToTheNearestCar = s_dist;
        }
      }
    }

    // Prepare trajectory params
    double targetSpeed = SPEED_LIMIT;
    double acceleration;
    if (minimalDistanceToTheNearestCar < 0 || minimalDistanceToTheNearestCar > 3 * SAFE_FOLLOWING_DISTANCE) {
      // No car in front, max speed
      acceleration = min((targetSpeed - us) * (SPEED_LIMIT / TIME_TO_MAX), (SPEED_LIMIT / TIME_TO_MAX));

    } else {
      // Car in front, optimize speed
      double delta = minimalDistanceToTheNearestCar - SAFE_FOLLOWING_DISTANCE;
      if (delta < 0)  {
        acceleration = -SPEED_LIMIT / TIME_TO_MAX;
      }
      else {
        double t = delta / us;
        acceleration = (car_front.getVS() - us) / t;
      }
    }

    resultS = s + us * trajectoryLength + 0.5 * acceleration * trajectoryLength * trajectoryLength;
    resultVs = us + acceleration * trajectoryLength;

    // Keep lane
    resultD = laneToD(currentLane);
    resultVd = (resultD - carD) / trajectoryLength;

    if (minimalDistanceToTheNearestCar - SAFE_FOLLOWING_DISTANCE < SAFE_DELTA) {
      state = CarStates::PREPARE_TO_CHANGE;
    }
  } else if (state == CarStates::CHANGE_LEFT || state == CarStates::CHANGE_RIGHT) {

    int newLane = currentLane + (state == CarStates::CHANGE_RIGHT ? 1 : -1);
    double targetSpeed = SPEED_LIMIT;
    double accel = min((targetSpeed - us) * (SPEED_LIMIT / TIME_TO_MAX), (SPEED_LIMIT / TIME_TO_MAX));
    resultS = s + us * trajectoryLength + 0.5 * accel * trajectoryLength * trajectoryLength;
    resultVs = us + accel * trajectoryLength;

    resultD = laneToD(newLane);
    resultVd = (resultD - carD) / trajectoryLength;
  }


  // Trajectory Generation
  vector<double> Si = {s, us, as};
  vector<double> Sf = {resultS, resultVs, 0};

  vector<double> Di = {d, ud, ad};
  vector<double> Df = {resultD, resultVd, 0};

  JMT(sCoeffs, Si, Sf, trajectoryLength);
  JMT(dCoeffs, Di, Df, trajectoryLength);
}

void BehaviorPlanner::JMT(vector<double> &cffs, const vector<double> &start, const vector<double> &end, double T) {
  /*
  Calculate the Jerk Minimizing Trajectory that connects the initial state
  to the final state in time T.

  start - the vehicles start location given as a length three array
      corresponding to initial values of [s, s_dot, s_double_dot]

  end   - the desired end state for vehicle. Like "start" this is a
      length three array.

  T     - The duration, in seconds, over which this maneuver should occur.

  OUTPUT
  an array of length 6, each value corresponding to a coefficent in the polynomial
  s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
  */
  double t2   = T * T;
  double t3   = t2 * T;
  double Sf   = end[0];
  double dSf  = end[1];
  double d2Sf = end[2];
  double Si   = start[0];
  double dSi  = start[1];
  double d2Si = start[2];

  MatrixXd Tm(3, 3);
  MatrixXd Sfmat(3, 1);

  Tm    << t3, t2 * t2, t3 * t2, 3 * t2, 4 * t3, 5 * t2 * t2, 6 * T, 12 * t2, 20 * t3;
  Sfmat << Sf - (Si + dSi * T + 0.5 * d2Si * T * T), dSf - (dSi + d2Si * T), d2Sf - d2Si;

  MatrixXd coeffs = Tm.inverse() * Sfmat;

  cffs.assign({Si, dSi, 0.5 * d2Si, coeffs(0), coeffs(1), coeffs(2)});
}

void BehaviorPlanner::decideState(const vector<vector<double>> &sensor_fusion) {
  if (state == CarStates::PREPARE_TO_CHANGE) {
    if (currentLane == 0) {
      if (isSafeToSwitch(1, s, us, sensor_fusion))
        state = CarStates::CHANGE_RIGHT;
      else
        state = CarStates::KEEP_LANE;
    } else if (currentLane == 1) {
      if (isSafeToSwitch(0, s, us, sensor_fusion))
        state = CarStates::CHANGE_LEFT;
      else if (isSafeToSwitch(2, s, us, sensor_fusion))
        state = CarStates::CHANGE_RIGHT;
      else
        state = CarStates::KEEP_LANE;
    } else if (currentLane == 2) {
      if (isSafeToSwitch(1, s, us, sensor_fusion))
        state = CarStates::CHANGE_LEFT;
      else
        state = CarStates::KEEP_LANE;
    }
  }
}


void BehaviorPlanner::trajectorySmooth(vector<double> &next_x_vals, vector<double> &next_y_vals, vector<double> vectT, vector<double> vectX, vector<double> vectY) {
  tk::spline Xspline, Yspline;
  Xspline.set_points(vectT, vectX);
  Yspline.set_points(vectT, vectY);

  for (int i = 0; i < trajectoryPoints; i++) {
    double x = Xspline(i * step);
    double y = Yspline(i * step);

    next_x_vals.push_back(x);
    next_y_vals.push_back(y);
  }
}
