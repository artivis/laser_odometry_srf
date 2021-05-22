#ifndef _LASER_ODOMETRY_SRF_LASER_ODOMETRY_SRF_H_
#define _LASER_ODOMETRY_SRF_LASER_ODOMETRY_SRF_H_

#include <laser_odometry_core/laser_odometry_base.h>
#include <srf_laser_odometry/laser_odometry_refscans.h>

namespace laser_odometry {

class LaserOdometrySrf : public LaserOdometryBase
{
  using Base = LaserOdometryBase;

public:

  LaserOdometrySrf()  = default;
  ~LaserOdometrySrf() = default;

  OdomType odomType() const noexcept override;

protected:

  bool processImpl(const sensor_msgs::LaserScanConstPtr& laser_msg,
                   const Transform& /*prediction*/) override;

protected:

  // method: {0:CS} {1:KS} {2:Hybrid}
  int operation_mode_ = 2;

  srf::SRF_RefS srf_;

  bool configureImpl() override;

  bool initialize(const sensor_msgs::LaserScanConstPtr& scan_msg) override;
};

} // namespace laser_odometry

#endif // _LASER_ODOMETRY_SRF_LASER_ODOMETRY_SRF_H_
