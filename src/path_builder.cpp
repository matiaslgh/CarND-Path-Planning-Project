#include "path_builder.h"

#include <vector>
#include "spline.h"
#include "helpers.h"

using std::vector;

void shiftToCarCoordinates(
  vector<double> &pts_x,
  vector<double> &pts_y,
  double ref_x,
  double ref_y,
  double ref_yaw
) {
  for(int i=0; i < pts_x.size(); i++){
    double shift_x = pts_x[i] - ref_x;
    double shift_y = pts_y[i] - ref_y;

    pts_x[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
    pts_y[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
  }
}

void findTwoPointsInThePast(
  vector<double> &pts_x,
  vector<double> &pts_y,
  const vector<double> &previous_path_x,
  const vector<double> &previous_path_y,
  double car_x,
  double car_y,
  double car_yaw,
  double &ref_x,
  double &ref_y,
  double &ref_yaw
) {
  int prev_size = previous_path_x.size();
  
  if (prev_size < 2) {
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);
    pts_x.push_back(prev_car_x);
    pts_x.push_back(car_x);

    pts_y.push_back(prev_car_y);
    pts_y.push_back(car_y);
  } else {
    // Redefine reference point to previous point
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];
    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
    pts_x.push_back(ref_x_prev);
    pts_x.push_back(ref_x);

    pts_y.push_back(ref_y_prev);
    pts_y.push_back(ref_y); 
  }
}

void findPointsInTheFuture(
  vector<double> &pts_x,
  vector<double> &pts_y,
  int lane,
  double car_s,
  vector<double> maps_s,
  vector<double> maps_x,
  vector<double> maps_y
) {
  // Find another 3 points in the future
  for(int i = 1; i < 4; i++){
    vector<double> next_wp = getXY(car_s + 30*i, 2 + 4 * lane, maps_s, maps_x, maps_y);
    pts_x.push_back(next_wp[0]);
    pts_y.push_back(next_wp[1]);
  }
}

vector<vector<double>> getSmoothTransition(
  const vector<double> &pts_x,
  const vector<double> &pts_y,
  const vector<double> &previous_path_x,
  const vector<double> &previous_path_y,
  double ref_vel,
  double ref_yaw,
  double ref_x,
  double ref_y
) {
  int prev_size = previous_path_x.size();

  tk::spline s;
  s.set_points(pts_x, pts_y);

  vector<double> next_x_vals;
  vector<double> next_y_vals;
  
  // Adding previous path points
  for ( int i = 0; i < prev_size; i++ ) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  for( int i = 1; i < 50 - prev_size; i++ ) {
    double N = target_dist / (0.02 * ref_vel);
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  vector<vector<double>> built_path;
  built_path.push_back(next_x_vals);
  built_path.push_back(next_y_vals);
  return built_path;
}

vector<vector<double>> PathBuilder::build_path(
  const vector<double> &previous_path_x,
  const vector<double> &previous_path_y,
  double car_s,
  double car_x,
  double car_y,
  double car_yaw
) {
  int lane = 1;
  double ref_vel = 20;

  int prev_size = previous_path_x.size();
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);
  vector<double> pts_x;
  vector<double> pts_y;

  findTwoPointsInThePast(pts_x, pts_y, previous_path_x, previous_path_y, car_x, car_y, car_yaw, ref_x, ref_y, ref_yaw);

  findPointsInTheFuture(pts_x, pts_y, lane, car_s, maps_s, maps_x, maps_y);

  shiftToCarCoordinates(pts_x, pts_y, ref_x, ref_y, ref_yaw);
  
  return getSmoothTransition(pts_x, pts_y, previous_path_x, previous_path_y, ref_vel, ref_yaw, ref_x, ref_y);
}