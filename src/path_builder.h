#include <vector>

using std::vector;

class PathBuilder {
  private:
    vector<double> maps_s;
    vector<double> maps_x;
    vector<double> maps_y;

  public:
    PathBuilder(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y):
     maps_s { maps_s }, maps_x { maps_x }, maps_y { maps_y } {}
    ~PathBuilder() {}
    vector<vector<double>> build_path(
      const vector<double> &previous_path_x,
      const vector<double> &previous_path_y,
      double car_s,
      double car_x,
      double car_y,
      double car_yaw
    );
};