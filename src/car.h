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

  public:
    Car(vector<double> sensor_data):
      vx { sensor_data[3] },
      vy { sensor_data[4] },
      s { sensor_data[5] },
      d { sensor_data[6] }
    {};
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
};

#endif