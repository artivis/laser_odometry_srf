#include "laser_odometry_srf/laser_odometry_srf.h"

#include <laser_odometry_core/laser_odometry_utils.h>

#include <pluginlib/class_list_macros.h>

namespace laser_odometry {

OdomType LaserOdometrySrf::odomType() const noexcept
{
  return OdomType::Odom2DCov;
}

bool LaserOdometrySrf::configureImpl()
{
  operation_mode_ = private_nh_.param("operation_mode", 2);

  if (operation_mode_ < 0 || operation_mode_ > 2) {
    ROS_ERROR(
      "Unknown operation mode for the SRF odometry!\n"
      "Using default: 2 - Hybrid"
    );
    operation_mode_ = 2;
  }

  return true;
}

bool LaserOdometrySrf::processImpl(
  const sensor_msgs::LaserScanConstPtr& laser_msg,
  const Transform& /*prediction*/
)
{
  for (unsigned int i = 0; i<laser_msg->ranges.size(); ++i)
  {
    srf_.range_wf(i) = laser_msg->ranges[i];
  }

  srf_.odometryCalculation();

  const auto& increment_2D = srf_.getIncrement();

  increment_ = Isometry3s(
    Eigen::Translation<Scalar, 3>(
      increment_2D.translation()(0), increment_2D.translation()(1), 0
    )
  );
  increment_.rotate(
    Eigen::AngleAxis<Scalar>(
      utils::getYaw(increment_2D.matrix()), Eigen::Matrix<Scalar, 3, 1>::UnitZ()
    )
  );

  increment_covariance_ = utils::covariance2dTo3d(
    srf_.getIncrementCovariance().cast<Scalar>()
  );

  return true;
}

bool LaserOdometrySrf::initialize(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  const unsigned int scan_size = scan_msg->ranges.size();
  const float fov = std::abs(scan_msg->angle_max - scan_msg->angle_min);

  ROS_INFO_STREAM(
    "Initializing with "
    "scan-size: " << scan_size <<
    " FoV " << fov <<
    " operation_mode " << operation_mode_
  );

  srf_.initialize(scan_size, fov, operation_mode_);

  for (unsigned int i = 0; i<scan_size; ++i)
    srf_.range_wf(i) = scan_msg->ranges[i];
  srf_.createScanPyramid();

  return true;
}

} // namespace laser_odometry

PLUGINLIB_EXPORT_CLASS(
  laser_odometry::LaserOdometrySrf, laser_odometry::LaserOdometryBase
);
