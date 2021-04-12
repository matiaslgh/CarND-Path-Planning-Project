#ifndef PREDICTOR_H
#define PREDICTOR_H

#include "json.hpp"
#include "car.h"

using nlohmann::json;

struct CarData {
  bool is_free;
  double speed;
};

struct Prediction {
  CarData left_car_data;
  CarData front_car_data;
  CarData right_car_data;
};

class Predictor {
  private:
    EgoCar ego_car;
    int target_lane;
    int previous_path_size;
    vector<Car> cars_in_road;
    bool is_close_to_ego_car(Car car);
    double predict_future_car_s(Car car);
    bool is_in_front_of_ego_car(Car car);
    bool is_on_the_left_of_ego_car(Car car);
    bool is_on_the_right_of_ego_car(Car car);

  public:
    Predictor(int target_lane, EgoCar ego_car, json raw_sensor_fusion_data, int previous_path_size);
    Prediction get_prediction();
};

#endif  // PREDICTOR_H