"""Compile actual controller methods with dependency stubs; no ROS controller build."""
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


@unittest.skipUnless(shutil.which("c++"), "requires a C++17 compiler")
class ControllerBounds(unittest.TestCase):
    def test_actual_rotation_method_respects_caps_and_zero(self):
        source = (Path(__file__).resolve().parents[1] / "nav2_graceful_controller/src/graceful_controller.cpp").read_text()
        start = source.index("geometry_msgs::msg::Twist GracefulController::rotateToTarget")
        end = source.index("bool GracefulController::inCollision", start)
        method = source[start:end]
        fixture = r"""
#include <algorithm>
#include <cassert>
#include <cmath>
namespace geometry_msgs { namespace msg {
struct Twist { struct Vector { double x=0, y=0, z=0; }; Vector linear, angular; };
}}
struct Params { double rotation_scaling_factor=0.5, v_angular_max=0.6, v_angular_min_in_place=0.3; };
struct GracefulController {
  Params value;
  Params *params_=&value;
  geometry_msgs::msg::Twist rotateToTarget(double);
};
"""
        fixture += method + r"""
int main() {
  GracefulController c;
  for (double cap : {0.6, 0.1}) {
    c.value.v_angular_max=cap;
    for (double angle : {-3.141592653589793, -0.1, 0.0, 0.1, 3.141592653589793}) {
      auto cmd=c.rotateToTarget(angle);
      assert(std::abs(cmd.angular.z)<=cap);
      assert(cmd.linear.x==0.0);
      if (angle==0.0) { assert(cmd.angular.z==0.0); }
      else { assert(cmd.angular.z*angle>0.0); }
    }
  }
}
"""
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)
            (path / "check.cpp").write_text(fixture)
            subprocess.run(["c++", "-std=c++17", str(path / "check.cpp"), "-o", str(path / "check")], check=True, capture_output=True)
            subprocess.run([str(path / "check")], check=True, capture_output=True)

    def test_outside_costmap_is_rejected(self):
        source = (Path(__file__).resolve().parents[1] / "nav2_graceful_controller/src/graceful_controller.cpp").read_text()
        start = source.index("bool GracefulController::inCollision")
        end = source.index("void GracefulController::computeDistanceAlongPath", start)
        fixture = r"""
#include <cassert>
#define RCLCPP_WARN(...) ((void)0)
namespace nav2_costmap_2d { enum { LETHAL_OBSTACLE=254, INSCRIBED_INFLATED_OBSTACLE=253, NO_INFORMATION=255 }; }
struct Map { bool inside=false; bool worldToMap(double, double, unsigned int &, unsigned int &) { return inside; } };
struct Layer { bool isTrackingUnknown() { return false; } };
struct Ros { Map map; Layer layer; Map *getCostmap() { return &map; }
  Layer *getLayeredCostmap() { return &layer; } bool getUseRadius() { return true; }
  int getRobotFootprint() { return 0; } };
struct Checker { double cost=0; double pointCost(unsigned int, unsigned int) { return cost; }
  double footprintCostAtPose(double, double, double, int) { return cost; } };
struct GracefulController { Ros ros; Checker checker; Ros *costmap_ros_=&ros;
  Checker *collision_checker_=&checker; int logger_=0;
  bool inCollision(const double &, const double &, const double &); };
"""
        fixture += source[start:end] + r"""
int main() {
  GracefulController c;
  assert(c.inCollision(0,0,0));
  c.ros.map.inside=true;
  assert(!c.inCollision(0,0,0));
  c.checker.cost=nav2_costmap_2d::LETHAL_OBSTACLE;
  assert(c.inCollision(0,0,0));
}
"""
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)
            (path / "check.cpp").write_text(fixture)
            subprocess.run(["c++", "-std=c++17", str(path / "check.cpp"), "-o", str(path / "check")], check=True, capture_output=True)
            subprocess.run([str(path / "check")], check=True, capture_output=True)

    def test_negative_heading_variation_is_preserved(self):
        source = (Path(__file__).resolve().parents[1] / "nav2_graceful_controller/src/graceful_controller.cpp").read_text()
        start = source.index("void GracefulController::validateOrientations")
        end = source.index("}  // namespace", start)
        fixture = r"""
#include <cassert>
#include <cmath>
#include <vector>
namespace geometry_msgs { namespace msg {
struct PoseStamped { struct Pose { struct Point { double x=0,y=0,z=0; } position;
  double orientation=0; } pose; };
}}
namespace tf2 { double getYaw(double yaw) { return yaw; } }
namespace angles { double shortest_angular_distance(double a,double b) { return std::remainder(b-a, 6.283185307179586); } }
namespace nav2_util { namespace geometry_utils { double orientationAroundZAxis(double yaw) { return yaw; } } }
struct GracefulController { void validateOrientations(std::vector<geometry_msgs::msg::PoseStamped> &); };
"""
        fixture += source[start:end] + r"""
int main() {
  GracefulController c;
  std::vector<geometry_msgs::msg::PoseStamped> path(4);
  for (int i=0; i<4; ++i) { path[i].pose.position.x=i; }
  for (double sign : {-1.0, 1.0}) {
    path[1].pose.orientation=0;
    path[2].pose.orientation=sign*0.3;
    c.validateOrientations(path);
    assert(path[2].pose.orientation==sign*0.3);
  }
}
"""
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)
            (path / "check.cpp").write_text(fixture)
            subprocess.run(["c++", "-std=c++17", str(path / "check.cpp"), "-o", str(path / "check")], check=True, capture_output=True)
            subprocess.run([str(path / "check")], check=True, capture_output=True)
