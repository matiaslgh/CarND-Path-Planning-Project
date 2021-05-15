#include "path_builder.h"

#include <iostream>
#include <vector>
#include "spline.h"
#include "helpers.h"
#include "car.h"

using std::vector;
using std::max;
using std::min;

double MAX_SPEED = 49.5;
double MAX_ACCELERATION = 0.224;

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
    double N = target_dist / (0.02 * ref_vel / 2.24);
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

void log(
  vector<double> pts_x,
  vector<double> pts_y,
  const vector<double> &previous_path_x,
  const vector<double> &previous_path_y,
  double ref_vel
) {

  std::cout << "pts_x: ";
  for (int i=0; i < pts_x.size(); i++) {
    std::cout << pts_x[i] << ", ";
  }
  std::cout << std::endl;
  std::cout << "pts_y: ";
  for (int i=0; i < pts_y.size(); i++) {
    std::cout << pts_y[i] << ", ";
  }
  std::cout << std::endl;
  
  std::cout << "ref_vel: " << ref_vel << std::endl;

  std::cout << "previous_path_points: ";
  for (int i=0; i < previous_path_x.size(); i++) {
    std::cout << "(" << previous_path_x[i] << "," << previous_path_y[i] << "), ";
  }
  std::cout << std::endl;
}

void testFixForSplineAssertionFailure(vector<double> &pts_x) {
  for(int i=0; i < pts_x.size() - 1; i++) {
    if (pts_x[i] >= pts_x[i+1]) {
      pts_x[i] = pts_x[i] - 0.001; // Hack to spline assertion to fail in really weird scenarios
    }
  }
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

  log(pts_x, pts_y, previous_path_x, previous_path_y, ref_vel);

  testFixForSplineAssertionFailure(pts_x); // Note for Udacity Reviewer: Comment this line out to test original implementation
  
  return getSmoothTransition(pts_x, pts_y, previous_path_x, previous_path_y, ref_vel, ref_yaw, ref_x, ref_y);
}

void logCarInfoIfNotNull(Car car, string log_prefix) {
  if (!car.is_null()) {
    std::cout << log_prefix << ".get_lane(): " << car.get_lane() << std::endl;
    std::cout << log_prefix << ".get_s(): " << car.get_s() << std::endl;
    std::cout << log_prefix << ".get_speed(): " << car.get_speed() << std::endl;
  }
}

void log(Predictor predictor, int current_lane, double current_speed, EgoCar ego_car) {
  std::cout << "--------------- Debugging data ---------------" << std::endl;
  std::cout << "current_lane: " << current_lane << std::endl;
  std::cout << "ego_car.get_lane(): " << ego_car.get_lane() << std::endl;
  std::cout << "ego_car.s: " << ego_car.s << std::endl;
  std::cout << "ego_car.speed: " << ego_car.speed << std::endl;
  std::cout << "ego_car.x: " << ego_car.x << std::endl;
  std::cout << "ego_car.y: " << ego_car.y << std::endl;
  std::cout << "ego_car.yaw: " << ego_car.yaw << std::endl;
  std::cout << "current_speed: " << current_speed << std::endl;
  std::cout << "is_front_free(): " << predictor.is_front_free() << std::endl;
  std::cout << "is_left_free(): " << predictor.is_left_free() << std::endl;
  std::cout << "is_right_free(): " << predictor.is_right_free() << std::endl;
  logCarInfoIfNotNull(predictor.get_car_in_front(), "car_in_front");
  logCarInfoIfNotNull(predictor.get_car_left_behind(), "car_left_behind");
  logCarInfoIfNotNull(predictor.get_car_left_front(), "car_left_front");
  logCarInfoIfNotNull(predictor.get_car_right_behind(), "car_right_behind");
  logCarInfoIfNotNull(predictor.get_car_right_front(), "car_right_front");
}

LaneAndSpeed PathBuilder::get_best_lane_and_speed(Predictor predictor, int current_lane, double current_speed, EgoCar ego_car) {
  log(predictor, current_lane, current_speed, ego_car);
  if (predictor.is_front_free()) {
    double speed = min(MAX_SPEED, current_speed + MAX_ACCELERATION);
    return { current_lane, speed };
  }
  
  if (predictor.is_left_free() && predictor.is_right_free()) {
    if (predictor.get_car_left_front().is_null()) {
      std::cout << "Change lane to the left" << std::endl;
      double speed = min(MAX_SPEED, current_speed - MAX_ACCELERATION / 4);
      return { current_lane - 1, speed };
    }
    if (predictor.get_car_right_front().is_null()) {
      std::cout << "Change lane to the right" << std::endl;
      double speed = min(MAX_SPEED, current_speed - MAX_ACCELERATION / 4);
      return { current_lane + 1, speed };
    }
    double left_front_speed = predictor.get_car_left_front().get_speed();
    double right_front_speed = predictor.get_car_right_front().get_speed();
    if (left_front_speed > right_front_speed) {
      std::cout << "Change lane to the left because its faster than the right" << std::endl;
      double speed = max(left_front_speed, current_speed - MAX_ACCELERATION / 4);
      return { current_lane - 1, speed };
    }
    double speed = max(right_front_speed, current_speed - MAX_ACCELERATION / 4);
    std::cout << "Change lane to the right because its faster than the left" << std::endl;
    return { current_lane + 1, speed };
  }

  if (predictor.is_left_free()) {
    Car car_left_front = predictor.get_car_left_front();
    if (car_left_front.is_null() || (!car_left_front.is_null() && car_left_front.get_speed() > current_speed)) {
      std::cout << "Change lane to the left because car_left_front is null or its speed is faster than current_speed" << std::endl;
      double speed = min(MAX_SPEED, current_speed - MAX_ACCELERATION / 4);
      return { current_lane - 1, speed };
    }
  }

  if (predictor.is_right_free()) {
    Car car_right_front = predictor.get_car_right_front();
    if (car_right_front.is_null() || (!car_right_front.is_null() && car_right_front.get_speed() > current_speed)) {
      std::cout << "Change lane to the right because car_right_front is null or its speed is faster than current_speed" << std::endl;
      double speed = min(MAX_SPEED, current_speed - MAX_ACCELERATION / 4);
      return { current_lane + 1, speed };
    }
  }

  Car car_in_front = predictor.get_car_in_front();
  if (ego_car.s + 5 > car_in_front.get_s()) {
    std::cout << "About to crash! Expand distance from car in front. No matter jerk violation" << std::endl;
    double speed = min(current_speed, car_in_front.get_speed() - MAX_ACCELERATION / 2);
    return { current_lane, speed };
  }

  if (ego_car.s + 10 > car_in_front.get_s()) {
    std::cout << "Too close to car in front (less than 10 meters)" << std::endl;
    double speed = min(current_speed, current_speed - MAX_ACCELERATION);
    return { current_lane, speed };
  }

  if (ego_car.s + 15 > car_in_front.get_s()) {
    std::cout << "Too close to car in front (less than 15 meters)" << std::endl;
    double speed = min(current_speed, current_speed - MAX_ACCELERATION / 2);
    return { current_lane, speed };
  }

  if (ego_car.s + 25 > car_in_front.get_s()) {
    std::cout << "Too close to car in front (less than 25 meters)" << std::endl;
    double speed = min(current_speed, current_speed - MAX_ACCELERATION / 4);
    return { current_lane, speed };
  }

  double speed = max(car_in_front.get_speed(), current_speed - MAX_ACCELERATION / 4);
  return { current_lane, speed };
}