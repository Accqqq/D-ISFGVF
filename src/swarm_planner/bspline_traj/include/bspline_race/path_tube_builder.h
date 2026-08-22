#ifndef BSPLINE_RACE_PATH_TUBE_BUILDER_H
#define BSPLINE_RACE_PATH_TUBE_BUILDER_H

#include <Eigen/Core>

#include <memory>
#include <vector>

#include <ros/time.h>

#include <plan_env/sdf_map.h>

#include <bspline_race/continuous_phase_path.h>
#include <bspline_race/phase_offset_types.h>

namespace FLAG_Race
{

struct TubeSample
{
  double w = 0.0;
  double lower = 0.0;
  double upper = 0.0;
  double lower_dw = 0.0;
  double upper_dw = 0.0;
  double regularity = 1.0;
  double beta = 1.0;
  // True when the path point and both +/-N rays are inside the map and the
  // sampled voxels are observed (not unknown).  A certified sample may still
  // have a zero-width interval (narrow corridor); that is a valid compressed
  // tube.  Uncertified samples (out-of-map / unknown) must not be claimed
  // passable and are skipped by the conservative lookahead query.
  bool certified = true;
};

struct TubeProfile
{
  std::vector<TubeSample> samples;
  double w0 = 0.0;
  double w1 = 0.0;
  uint32_t epoch = 0;
  ros::Time stamp;
  bool valid = false;
};

struct TubeBuilderParams
{
  double sample_step_w = 0.10;
  double lookahead_w = 2.00;
  double ray_step = 0.05;
  double max_offset = 1.20;
  double reference_clearance = 0.50;
  double boundary_slope_max = 0.80;
  double open_width = 1.60;
  double mu_regular = 0.20;
};

class PathTubeBuilder
{
public:
  void setMap(const std::shared_ptr<SDFMap>& map) { map_ = map; }

  bool rebuild(const ContinuousPhasePath& path,
               double current_w,
               double current_delta,
               TubeProfile& profile);

  bool query(const TubeProfile& profile, double w, TubeBounds& bounds) const;

  bool queryConservativeLookahead(const TubeProfile& profile,
                                  double w0,
                                  double w1,
                                  TubeBounds& bounds) const;

  bool lineOfSight(const Eigen::Vector3d& a,
                   const Eigen::Vector3d& b) const;
  bool mapReady() const { return map_ && map_->mapReady(); }

  TubeBuilderParams params;

private:
  std::shared_ptr<SDFMap> map_;
  uint32_t epoch_ = 0;
};

}  // namespace FLAG_Race

#endif  // BSPLINE_RACE_PATH_TUBE_BUILDER_H
