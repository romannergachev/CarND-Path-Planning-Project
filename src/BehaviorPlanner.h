#include <vector>
#include "CarStates.h"

#ifndef PATH_PLANNING_BEHAVIORPLANNER_H
#define PATH_PLANNING_BEHAVIORPLANNER_H

using namespace std;

/*
 * Behavior planner class
 */
class BehaviorPlanner {
public:
  vector<double> sCoeffs, dCoeffs;

  BehaviorPlanner();
  bool isSafeToSwitch(int lane, double s, double us, const vector<vector<double> > &sensor_fusion);
  void JMT(vector<double> &cffs, const vector<double> &start, const vector<double> &end, double T);
  void trajectorySmooth(vector<double> &next_x_vals, vector<double> &next_y_vals, vector<double> vectT, vector<double> vectX, vector<double> vectY);
  void decideState(const vector <vector<double>> &sensor_fusion);
  void initState(double s, double d, int previousPathLength);
  void perform(const vector <vector<double>> &sensor_fusion);

private:
  double s, d, us, ud, as, ad, carS, carD;
  CarStates state = CarStates::KEEP_LANE;
  int currentLane = 1;
};


#endif //PATH_PLANNING_BEHAVIORPLANNER_H
