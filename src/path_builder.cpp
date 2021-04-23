#include "path_builder.h"

#include <vector>
#include "spline.h"
#include "helpers.h"
#include "car.h"

using std::vector;
using std::max;
using std::min;

double MAX_SPEED = 49.5 / 2.24;
int LEFT_LANE = 0;
int CENTER_LANE = 1;
int RIGHT_LANE = 2;

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

  for (int i = 1; i < 50 - prev_size; i++ ) {
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

vector<LaneAndSpeed> PathBuilder::get_possible_lanes_and_speeds(Prediction prediction, int target_lane, double ref_vel) {
  vector<LaneAndSpeed> lanes_and_speeds;
  if (prediction.front_car_data.is_free) {
    lanes_and_speeds.push_back({ target_lane, min(MAX_SPEED, ref_vel + 8 * 0.02) });
  } else {
    lanes_and_speeds.push_back({ target_lane, max(prediction.front_car_data.speed, ref_vel - 2 * 0.02) });

    if (prediction.left_car_data.is_free) {
      lanes_and_speeds.push_back({
        max(target_lane - 1, LEFT_LANE),
        min(MAX_SPEED, ref_vel + 5 * 0.02)
      });
    }

    if (prediction.right_car_data.is_free) {
      lanes_and_speeds.push_back({
        min(target_lane + 1, RIGHT_LANE),
        min(MAX_SPEED, ref_vel + 5 * 0.02)
      });
    }
  }

  return lanes_and_speeds;
}

double calculate_lane_change_cost(int lane, int potential_lane) {
  return std::abs(lane - potential_lane) / 3;
}

double calculate_acceleration_cost(double car_speed, double potential_ref_vel) {
  double acc = (potential_ref_vel - car_speed) / 0.02;
  if (acc < -9.8 || acc > 9.8) {
    return 1;
  }
  return std::abs(acc) / 9.8;
}

double calculate_speed_cost(double car_speed, double potential_ref_vel) {
  if (potential_ref_vel < MAX_SPEED) {
    return 0.8 * (MAX_SPEED - potential_ref_vel) / MAX_SPEED;
  } else {
    if (car_speed > 50) {
      return 1;
    } else {
      return (potential_ref_vel - MAX_SPEED) / 0.5;
    }
  }
}

double calculate_cost(int lane, double car_speed, int potential_lane, double potential_ref_vel) {
  double cost_lane_change = calculate_lane_change_cost(lane, potential_lane);

  double cost_acc = calculate_acceleration_cost(car_speed, potential_ref_vel);

  double cost_speed = calculate_speed_cost(car_speed, potential_ref_vel);

  return 10 * cost_lane_change + 10 * cost_acc + 10 * cost_speed;
}

LaneAndSpeed PathBuilder::get_best_lane_and_speed(vector<LaneAndSpeed> lane_and_speeds, int current_lane, double current_speed) {
  double lowest_cost = 999999;
  int best_lane = current_lane;
  double best_speed = current_speed;

  for (int i = 0; i < lane_and_speeds.size(); i++) {
    int potential_lane = lane_and_speeds[i].lane;
    double potential_ref_vel = lane_and_speeds[i].speed;
    double cost = calculate_cost(current_lane, current_speed, potential_lane, potential_ref_vel);

    if (cost < lowest_cost) {
      lowest_cost = cost;
      best_lane = potential_lane;
      best_speed = potential_ref_vel;
    }
  }

  return { best_lane, best_speed };
}