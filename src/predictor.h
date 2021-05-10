#ifndef PREDICTOR_H
#define PREDICTOR_H

#include "json.hpp"
#include "car.h"

using nlohmann::json;

class Predictor {
  private:
    EgoCar ego_car;
    int target_lane;
    int previous_path_size;
    Car car_in_front;
    Car car_left_behind;
    Car car_left_front;
    Car car_right_behind;
    Car car_right_front;
    bool is_close_to_ego_car(Car car);
    double predict_future_car_s(Car car);
    bool is_in_front_of_ego_car(Car car);
    bool is_on_the_left_of_ego_car(Car car);
    bool is_on_the_right_of_ego_car(Car car);

  public:
    Predictor(int target_lane, EgoCar ego_car, json raw_sensor_fusion_data, int previous_path_size);
    Car get_car_in_front() { return car_in_front; };
    Car get_car_left_behind() { return car_left_behind; };
    Car get_car_left_front() { return car_left_front; };
    Car get_car_right_behind() { return car_right_behind; };
    Car get_car_right_front() { return car_right_front; };
    bool is_front_free();
    bool is_left_free();
    bool is_right_free();
    void clean_state();
};

#endif  // PREDICTOR_H