#include "car.h"

double Car::get_speed() {
  return sqrt(pow(vx, 2) + pow(vy, 2));
}

int Car::get_lane() {
  return (int) d / 4;
}