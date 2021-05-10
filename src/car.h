#ifndef CAR_H
#define CAR_H

#include <vector>

using std::vector;

class Car {
  private:
    double vx;
    double vy;
    double s;
    double d;
    bool is_null_;

  public:
    Car(vector<double> sensor_data):
      vx { sensor_data[3] },
      vy { sensor_data[4] },
      s { sensor_data[5] },
      d { sensor_data[6] },
      is_null_ { false }
    {};
    Car():
      vx { 0 },
      vy { 0 },
      s { 0 },
      d { 0 },
      is_null_ { true }
    {};
    bool is_null() { return is_null_; };
    double get_speed();
    double get_s() { return s; };
    void set_s(double s) { s = s; };
    int get_lane();
};

struct EgoCar {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  int get_lane() { return (int) d / 4; };
};

#endif