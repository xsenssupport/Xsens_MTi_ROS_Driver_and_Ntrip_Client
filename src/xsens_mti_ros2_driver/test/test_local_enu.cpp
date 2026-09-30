#include "local_enu.h"
#include <iostream>
#include <limits>
#include <stdexcept>

static void check(bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
}
static void near(const Eigen::Vector3d &a, const Eigen::Vector3d &b, double tolerance, const char *message) {
    check((a-b).norm() < tolerance, message);
}
int main() {
    xsens::LocalEnu frame;
    check(!frame.initialized(), "origin starts unset");
    frame.reset(0, 0, 0);
    near(frame.position(0,0,0), Eigen::Vector3d::Zero(), 1e-9, "first position is zero");
    near(frame.position(0,0,10), {0,0,10}, 1e-8, "ellipsoidal height is up");
    near(frame.position(0,0.00001,0), {1.1131949079,0,0}, 1e-6, "east axis");
    near(frame.position(0.00001,0,0), {0,1.1057427582,0}, 1e-6, "north axis");
    const double pi = std::acos(-1.0);
    const Eigen::Quaterniond yaw(Eigen::AngleAxisd(pi/2, Eigen::Vector3d::UnitZ()));
    const auto result = frame.orientation(0,0,yaw);
    near(result * Eigen::Vector3d::UnitX(), {0,1,0}, 1e-12, "90 degrees must not become 180");
    near(yaw.conjugate() * Eigen::Vector3d::UnitX(), {0,-1,0}, 1e-12, "ENU velocity to sensor axes");
    frame.reset(52, 6, 40);
    const Eigen::Quaterniond tilted(Eigen::AngleAxisd(0.7, Eigen::Vector3d(1,2,3).normalized()));
    check(std::abs(frame.orientation(52,6,tilted).dot(tilted)) > 1-1e-12, "arbitrary origin preserves attitude");
    // The same ECEF orientation at two positions must have the same fixed-frame attitude.
    const Eigen::Quaterniond at_origin(xsens::LocalEnu::ecefToEnu(52,6));
    const Eigen::Quaterniond at_other(xsens::LocalEnu::ecefToEnu(53,7));
    check(std::abs(frame.orientation(53,7,at_other).dot(at_origin)) > 1-1e-12, "fixed tangent orientation");
    frame.reset(0,179.99999,0);
    check(frame.position(0,-179.99999,0).norm() < 3, "dateline continuity");
    frame.reset(-0.00001,30,0);
    check(frame.position(0.00001,30,0).norm() < 3, "equator continuity");
    frame.reset(90,0,0);
    check(frame.position(89.99999,120,0).allFinite(), "polar coordinates remain finite");
    check(!xsens::LocalEnu::valid(91,0,0), "invalid latitude");
    check(!xsens::LocalEnu::valid(0,181,0), "invalid longitude");
    check(!xsens::LocalEnu::valid(0,0,std::numeric_limits<double>::quiet_NaN()), "invalid altitude");
    std::cout << "Local ENU regression checks passed\n";
}
