#include "predictor.h"
#include <vector>

using std::vector;

int LEFT_LANE = 0;
int CENTER_LANE = 1;
int RIGHT_LANE = 2;

int METERS_OF_DIFFERENCE_FRONT = 40;
int METERS_OF_DIFFERENCE_BEHIND = 20;

bool is_distance_under_x_meters(Car car, EgoCar ego_car, double x_meters) {
  return abs(car.get_s() - ego_car.s) < x_meters;
}

Predictor::Predictor(int target_lane_, EgoCar ego_car_, json raw_sensor_fusion_data, int previous_path_size) {
  target_lane = target_lane_;
  ego_car = ego_car_;
  previous_path_size = previous_path_size;

  for (int i = 0; i < raw_sensor_fusion_data.size(); i++) {
    vector<double> raw_sensor_fusion_data_i = raw_sensor_fusion_data[i];
    Car car { raw_sensor_fusion_data_i };
    if (!is_close_to_ego_car(car)) {
      continue;
    };

    car.set_s(predict_future_car_s(car));

    // TODO: Check if instead of target_lane I should use ego_car.get_lane()
    if (is_in_front_of_ego_car(car) && (car_in_front.is_null() || (car_in_front.get_s() > car.get_s()))) {
      car_in_front = car;
    } else if (is_on_the_left_of_ego_car(car)) {
      if (car.get_s() >= ego_car.s) {
        if (car_left_front.is_null() || (car_left_front.get_s() > car.get_s())) {
          car_left_front = car;
        }
      } else {
        if (car_left_behind.is_null() || (car_left_behind.get_s() < car.get_s())) {
          car_left_behind = car;
        }
      }
    } else if (is_on_the_right_of_ego_car(car)) {
      if (car.get_s() >= ego_car.s) {
        if (car_right_front.is_null() || (car_right_front.get_s() > car.get_s())) {
          car_right_front = car;
        }
      } else {
        if (car_right_behind.is_null() || (car_right_behind.get_s() < car.get_s())) {
          car_right_behind = car;
        }
      }
    }
  }
}

bool Predictor::is_close_to_ego_car(Car car) {
  int LIMIT = 100;
  return is_distance_under_x_meters(car, ego_car, LIMIT);
}

double Predictor::predict_future_car_s(Car car) {
  return car.get_s() + previous_path_size * 0.02 * car.get_speed();
}

bool is_same_lane(Car car, int lane) {
  return car.get_lane() == lane;
}

bool Predictor::is_in_front_of_ego_car(Car car) {
  bool is_in_same_lane = is_same_lane(car, target_lane);
  bool is_in_front = car.get_s() > ego_car.s;
  return is_in_same_lane && is_in_front;
}

bool Predictor::is_on_the_left_of_ego_car(Car car) {
  return is_same_lane(car, target_lane - 1);
}

bool Predictor::is_on_the_right_of_ego_car(Car car) {
  return is_same_lane(car, target_lane + 1);
}

bool Predictor::is_front_free() {
  if (car_in_front.is_null()) {
    return true;
  }
  return !(car_in_front.get_s() < ego_car.s + METERS_OF_DIFFERENCE_FRONT);
}

// TODO: Consider car_left_behind.get_speed() vs ego_car.speed
bool Predictor::is_left_free() {
  if (target_lane == LEFT_LANE) {
    return false;
  }

  if (car_left_behind.is_null() && car_left_front.is_null()) {
    return true;
  }

  bool is_left_front_free = car_left_front.is_null() || !(car_left_front.get_s() < ego_car.s + METERS_OF_DIFFERENCE_FRONT);
  bool is_left_behind_free = car_left_behind.is_null() || !(car_left_behind.get_s() > ego_car.s - METERS_OF_DIFFERENCE_BEHIND);

  return is_left_front_free && is_left_behind_free;
}

// TODO: Consider car_right_behind.get_speed() vs ego_car.speed
bool Predictor::is_right_free() {
  if (target_lane == RIGHT_LANE) {
    return false;
  }

  if (car_right_behind.is_null() && car_right_front.is_null()) {
    return true;
  }

  bool is_right_front_free = car_right_front.is_null() || !(car_right_front.get_s() < ego_car.s + METERS_OF_DIFFERENCE_FRONT);
  bool is_right_behind_free = car_right_behind.is_null() || !(car_right_behind.get_s() > ego_car.s - METERS_OF_DIFFERENCE_BEHIND);

  return is_right_front_free && is_right_behind_free;
}

// TODO: Check if I can remove this
void Predictor::clean_state() {
  car_left_front = Car {};
  car_left_behind = Car {};
  car_in_front = Car {};
  car_right_front = Car {};
  car_right_behind = Car {};
}