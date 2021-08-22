#ifndef LASER_ODOM_ERROR_TERM_H
#define LASER_ODOM_ERROR_TERM_H

#include "synchronizer.h"
#include "csm/math_utils.h"

class LaserOdomErrorTerm {
 public:
  LaserOdomErrorTerm(const cSynchronizer::sync_data& data)
      : calibration_data_(data) {}
  bool operator()(const double* const l_x, const double* const l_y,
                  const double* const l_theta,
                  const double* const left_wheel_radius,
                  const double* const right_wheel_radius,
                  const double* const bias, double* residuals) const {
    double J11 = *left_wheel_radius / 2.0;
    double J12 = *right_wheel_radius / 2.0;
    double J21 = -*left_wheel_radius / *bias;
    double J22 = *right_wheel_radius / *bias;

    double speed = J11 * calibration_data_.velocity_left +
                   J12 * calibration_data_.velocity_right;
    double omega = J21 * calibration_data_.velocity_left +
                   J22 * calibration_data_.velocity_right;

    double o_theta = calibration_data_.T * omega;

    double t1, t2;
    if (fabs(o_theta) > 1e-12) {
      t1 = sin(o_theta) / o_theta;
      t2 = (1 - cos(o_theta)) / o_theta;
    } else {
      t1 = 1;
      t2 = 0;
    }

    double odom[3];

    odom[0] = t1 * speed * calibration_data_.T;
    odom[1] = t2 * speed * calibration_data_.T;
    odom[2] = o_theta;

    double l_plus_s[3];
    double o_plus_l[3];
    double l_odom_to_laser[3] = {*l_x, *l_y, *l_theta};

    oplus_d(l_odom_to_laser, calibration_data_.scan_match_results, l_plus_s);
    oplus_d(odom, l_odom_to_laser, o_plus_l);

    residuals[0] = l_plus_s[0] - o_plus_l[0];
    residuals[1] = l_plus_s[1] - o_plus_l[1];
    residuals[2] = angleDiff(l_plus_s[2], o_plus_l[2]);

    return true;
  }

  static ceres::CostFunction* Create(
      const cSynchronizer::sync_data& calibration_data) {
    return (
        new ceres::NumericDiffCostFunction<LaserOdomErrorTerm, ceres::CENTRAL,
                                           3, 1, 1, 1, 1, 1, 1>(
            new LaserOdomErrorTerm(calibration_data)));
  }

 private:
  const cSynchronizer::sync_data calibration_data_;
};

#endif  // LASER_ODOM_ERROR_TERM_H
