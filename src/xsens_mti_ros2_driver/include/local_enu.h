// Distributed under the BSD license of the Xsens MTi ROS 2 driver.
#ifndef XSENS_LOCAL_ENU_H
#define XSENS_LOCAL_ENU_H
#include <Eigen/Geometry>
#include <cmath>

namespace xsens {
// WGS84 ellipsoidal height; fixed tangent frame at the first valid sample.
class LocalEnu {
    bool ready_ = false;
    Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rotation_ = Eigen::Matrix3d::Identity();
public:
    static bool valid(double lat, double lon, double height) {
        return std::isfinite(lat) && std::isfinite(lon) && std::isfinite(height) &&
            lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0;
    }
    static Eigen::Matrix3d ecefToEnu(double lat, double lon) {
        const double rad = std::acos(-1.0) / 180.0;
        const double s = std::sin(lat * rad), c = std::cos(lat * rad);
        const double sl = std::sin(lon * rad), cl = std::cos(lon * rad);
        Eigen::Matrix3d r;
        r << -sl, cl, 0, -s*cl, -s*sl, c, c*cl, c*sl, s;
        return r;
    }
    static Eigen::Vector3d ecef(double lat, double lon, double height) {
        const double rad = std::acos(-1.0) / 180.0;
        const double a = 6378137.0, f = 1.0 / 298.257223563;
        const double e2 = f * (2.0 - f);
        const double s = std::sin(lat * rad), c = std::cos(lat * rad);
        const double n = a / std::sqrt(1.0 - e2*s*s);
        return {(n+height)*c*std::cos(lon*rad), (n+height)*c*std::sin(lon*rad),
                (n*(1.0-e2)+height)*s};
    }
    bool initialized() const { return ready_; }
    void reset(double lat, double lon, double height) {
        origin_ = ecef(lat, lon, height);
        rotation_ = ecefToEnu(lat, lon);
        ready_ = true;
    }
    Eigen::Vector3d position(double lat, double lon, double height) const {
        return rotation_ * (ecef(lat, lon, height) - origin_);
    }
    Eigen::Quaterniond orientation(double lat, double lon, const Eigen::Quaterniond &q) const {
        return (Eigen::Quaterniond(rotation_ * ecefToEnu(lat, lon).transpose()) * q).normalized();
    }
};
}
#endif
