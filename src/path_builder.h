#ifndef PATH_BUILDER_H
#define PATH_BUILDER_H

#include <vector>
#include "predictor.h"

using std::vector;

struct LaneAndSpeed {
  int lane;
  double speed;
};

class PathBuilder {
  private:
    vector<double> maps_s;
    vector<double> maps_x;
    vector<double> maps_y;

  public:
    PathBuilder(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y):
     maps_s { maps_s }, maps_x { maps_x }, maps_y { maps_y } {}
    ~PathBuilder() {}
    vector<vector<double>> build_path(
      const vector<double> &previous_path_x,
      const vector<double> &previous_path_y,
      EgoCar ego_car,
      int target_lane,
      double ref_vel
    );
    vector<LaneAndSpeed> get_possible_lanes_and_speeds(Prediction prediction, int target_lane, double ref_vel);
    LaneAndSpeed get_best_lane_and_speed(vector<LaneAndSpeed> lanes_and_speeds, int current_lane, double current_speed);
};

#endif // PATH_BUILDER_H