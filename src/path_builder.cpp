#include "path_builder.h"

#include <iostream>
#include <vector>
#include "spline.h"
#include "helpers.h"
#include "car.h"

using std::vector;
using std::max;
using std::min;

double MAX_SPEED = 49.5 / 2.24;

void shiftToCarCoordinates(
  vector<double> &pts_x,
  vector<double> &pts_y,
  double ref_x,
  double ref_y,
  double ref_yaw
) {
  for (int i = 0; i < pts_x.size(); i++) {
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
  EgoCar ego_car,
  double &ref_x,
  double &ref_y,
  double &ref_yaw
) {
  int prev_size = previous_path_x.size();
  
  if (prev_size < 2) {
    double prev_car_x = ego_car.x - cos(ego_car.yaw);
    double prev_car_y = ego_car.y - sin(ego_car.yaw);
    pts_x.push_back(prev_car_x);
    pts_x.push_back(ego_car.x);

    pts_y.push_back(prev_car_y);
    pts_y.push_back(ego_car.y);
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
  for (int i = 1; i < 4; i++) {
    vector<double> next_wp = getXY(car_s + 30 * i, 2 + 4 * lane, maps_s, maps_x, maps_y);
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
  for (int i = 0; i < prev_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  double x_add_on = 0;

  for (int i = 1; i < 10 - prev_size; i++ ) {
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
  EgoCar ego_car,
  int lane,
  double ref_vel
) {
  int prev_size = previous_path_x.size();
  double ref_x = ego_car.x;
  double ref_y = ego_car.y;
  double ref_yaw = deg2rad(ego_car.yaw);
  vector<double> pts_x;
  vector<double> pts_y;

  findTwoPointsInThePast(pts_x, pts_y, previous_path_x, previous_path_y, ego_car, ref_x, ref_y, ref_yaw);

  findPointsInTheFuture(pts_x, pts_y, lane, ego_car.s, maps_s, maps_x, maps_y);

  shiftToCarCoordinates(pts_x, pts_y, ref_x, ref_y, ref_yaw);
  
  return getSmoothTransition(pts_x, pts_y, previous_path_x, previous_path_y, ref_vel, ref_yaw, ref_x, ref_y);
}

LaneAndSpeed PathBuilder::get_best_lane_and_speed(Predictor predictor, int current_lane, double current_speed, EgoCar ego_car) {
  if (predictor.is_front_free()) {
    double speed = min(MAX_SPEED, current_speed + 8 * 0.02);
    return { current_lane, speed };
  }
  
  if (predictor.is_left_free() && predictor.is_right_free()) {
    if (predictor.get_car_left_front().is_null()) {
      double speed = min(MAX_SPEED, current_speed + 0.02);
      return { current_lane - 1, speed };
    }
    if (predictor.get_car_right_front().is_null()) {
      double speed = min(MAX_SPEED, current_speed + 0.02);
      return { current_lane + 1, speed };
    }
    double left_front_speed = predictor.get_car_left_front().get_speed();
    double right_front_speed = predictor.get_car_right_front().get_speed();
    if (left_front_speed > right_front_speed) {
      double speed = max(left_front_speed, left_front_speed - 0.02);
      return { current_lane - 1, speed };
    }
    double speed = max(right_front_speed, right_front_speed - 0.02);
    return { current_lane - 1, speed };
  }

  if (predictor.is_left_free()) {
    Car car_left_front = predictor.get_car_left_front();
    if (car_left_front.is_null() || (!car_left_front.is_null() && car_left_front.get_speed() > current_speed)) {
      double speed = min(MAX_SPEED, current_speed + 0.02);
      return { current_lane - 1, speed };
    }
  }

  if (predictor.is_right_free()) {
    Car car_right_front = predictor.get_car_right_front();
    if (car_right_front.is_null() || (!car_right_front.is_null() && car_right_front.get_speed() > current_speed)) {
      double speed = min(MAX_SPEED, current_speed + 0.02);
      return { current_lane + 1, speed };
    }
  }

  Car car_in_front = predictor.get_car_in_front();
  if (ego_car.s + 25 > car_in_front.get_s() && car_in_front.get_speed() <= current_speed) {
    double speed = min(current_speed, current_speed - 0.02);
    return { current_lane, speed };
  }

  if (ego_car.s + 15 > car_in_front.get_s() && car_in_front.get_speed() <= current_speed) {
    double speed = min(current_speed, current_speed - 4 * 0.02);
    return { current_lane, speed };
  }

  if (ego_car.s + 10 > car_in_front.get_s() && car_in_front.get_speed() <= current_speed) {
    double speed = min(current_speed, current_speed - 8 * 0.02);
    return { current_lane, speed };
  }

  if (ego_car.s + 5 > car_in_front.get_s() && car_in_front.get_speed() <= current_speed) {
    return { current_lane, car_in_front.get_speed() - 0.02 };
  }


  double speed = max(car_in_front.get_speed(), current_speed - 2 * 0.02);
  return { current_lane, speed };
}