#include "car.h"
#include <cmath>

double Car::get_speed() {
  return std::sqrt(std::pow(vx, 2) + std::pow(vy, 2));
}

int Car::get_lane() {
  return (int) d / 4;
}