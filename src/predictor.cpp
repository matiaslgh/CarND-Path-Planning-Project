#include "predictor.h"
#include <vector>

using std::vector;

Predictor::Predictor(int target_lane_, EgoCar ego_car_, json raw_sensor_fusion_data, int previous_path_size) {
  target_lane = target_lane_;
  ego_car = ego_car_;
  previous_path_size = previous_path_size;

  for (int i = 0; i < raw_sensor_fusion_data.size(); i++) {
    vector<double> raw_sensor_fusion_data_i = raw_sensor_fusion_data[i];
    cars_in_road.push_back(Car { raw_sensor_fusion_data_i });
  }
}

double Predictor::predict_future_car_s(Car car) {
  return car.get_s() + previous_path_size * 0.02 * car.get_speed();
}

bool is_distance_under_x_meters(Car car, EgoCar ego_car, double x_meters) {
  return abs(car.get_s() - ego_car.s) < x_meters;
}

bool Predictor::is_close_to_ego_car(Car car) {
  int LIMIT = 100;
  return is_distance_under_x_meters(car, ego_car, LIMIT);
}

bool is_in_range(Car car, EgoCar ego_car, double limit_distance_behind, double limit_distance_in_front) {
  bool is_in_front_or_equal = car.get_s() >= ego_car.s;
  bool is_behind = car.get_s() < ego_car.s;
  
  // TODO: Take speed into consideration.
  // E.g if is behind and it goes too fast probably the limit_distance_behind should be modified
  // E.f. if is in front and it goes too slow probably the limit_distance_in_front should be modified
  return (
    (is_in_front_or_equal && is_distance_under_x_meters(car, ego_car, limit_distance_in_front))
    ||
    (is_behind && is_distance_under_x_meters(car, ego_car, abs(limit_distance_behind)))
  );
}

bool is_same_lane(Car car, int lane) {
  return car.get_lane() == lane;
}

bool Predictor::is_in_front_of_ego_car(Car car) {
  bool is_in_same_lane = is_same_lane(car, target_lane);
  bool is_in_front = car.get_s() > ego_car.s;
  int METERS_OF_DIFFERENCE = 50;
  bool is_close = car.get_s() < ego_car.s + METERS_OF_DIFFERENCE;

  return is_in_same_lane && is_in_front && is_close;
}

bool Predictor::is_on_the_left_of_ego_car(Car car) {
  bool is_on_the_left = is_same_lane(car, target_lane - 1);
  bool is_in_limit_left_lane = target_lane == 0;

  return  is_in_limit_left_lane || ( is_on_the_left && is_in_range(car, ego_car, -30, 40) );
}

bool Predictor::is_on_the_right_of_ego_car(Car car) {
  bool is_on_the_right = is_same_lane(car, target_lane - 1);
  bool is_in_limit_right_lane = target_lane == 2;

  return  is_in_limit_right_lane || ( is_on_the_right && is_in_range(car, ego_car, -30, 40) );
}


// TODO: Check if instead of target_lane I should use ego_car_d (to lane)
Prediction Predictor::get_prediction() {
  struct Prediction prediction;
  prediction.left_car_data.is_free = true;
  prediction.front_car_data.is_free = true;
  prediction.right_car_data.is_free = true;

  for (int i = 0; i < cars_in_road.size(); i++) {
    Car &car = cars_in_road[i];
    if (!is_close_to_ego_car(car)) {
      continue;
    };

    car.set_s(predict_future_car_s(car));

    if (is_in_front_of_ego_car(car)) {
      prediction.front_car_data.is_free = false;
      prediction.front_car_data.speed = car.get_speed();
    }

    if (is_on_the_left_of_ego_car(car)) {
      prediction.left_car_data.is_free = false;
      prediction.left_car_data.speed = car.get_speed();
    }

    if (is_on_the_right_of_ego_car(car)) {
      prediction.right_car_data.is_free = false;
      prediction.right_car_data.speed = car.get_speed();
    }

  }

  return prediction;
}
