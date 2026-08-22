#include "bspline_race/integration/phase_offset_tube_markers.h"

#include <visualization_msgs/Marker.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace FLAG_Race {
namespace {

using TubeSampleVector = std::vector<phase_offset_navigation::TubeRawSample,
                                     Eigen::aligned_allocator<phase_offset_navigation::TubeRawSample>>;

bool BoundaryPointFinite(const phase_offset_navigation::TubeRawSample& sample) {
  if (!sample.p.allFinite() || !sample.N.allFinite() ||
      !std::isfinite(sample.filtered_lower) ||
      !std::isfinite(sample.filtered_upper)) {
    return false;
  }
  return (sample.p + sample.N * sample.filtered_lower).allFinite() &&
         (sample.p + sample.N * sample.filtered_upper).allFinite();
}

geometry_msgs::Point ToPoint(const Eigen::Vector3d& value) {
  geometry_msgs::Point point;
  point.x = value.x(); point.y = value.y(); point.z = value.z();
  return point;
}

visualization_msgs::Marker MakeTubeLine(const ros::Time& stamp,
                                        const std::string& frame_id,
                                        const std::string& ns,
                                        const int id,
                                        const float scale,
                                        const float red,
                                        const float green,
                                        const float blue) {
  visualization_msgs::Marker marker;
  marker.header.stamp = stamp; marker.header.frame_id = frame_id;
  marker.ns = ns; marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = scale;
  marker.color.r = red; marker.color.g = green; marker.color.b = blue;
  marker.color.a = 1.0F;
  return marker;
}

visualization_msgs::Marker MakeTubeRibbon(const ros::Time& stamp,
                                          const std::string& frame_id,
                                          const std::string& ns,
                                          const float red,
                                          const float green,
                                          const float blue,
                                          const float alpha) {
  visualization_msgs::Marker marker;
  marker.header.stamp = stamp; marker.header.frame_id = frame_id;
  marker.ns = ns; marker.id = 2;
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  // TRIANGLE_LIST is rendered in the marker's local frame and the marker
  // scale is applied on all three axes.  Leaving y/z at their message
  // defaults (0) collapses the ribbon, so only the two boundary LINE_STRIPs
  // remain visible in RViz.  The ribbon points already contain the complete
  // world-space geometry; use a neutral unit scale on every axis.
  marker.scale.x = marker.scale.y = marker.scale.z = 1.0F;
  marker.color.r = red; marker.color.g = green; marker.color.b = blue;
  marker.color.a = alpha;
  return marker;
}

visualization_msgs::Marker MakeDelete(const ros::Time& stamp,
                                      const std::string& frame_id,
                                      const std::string& ns,
                                      const int id) {
  visualization_msgs::Marker marker;
  marker.header.stamp = stamp; marker.header.frame_id = frame_id;
  marker.ns = ns; marker.id = id;
  marker.action = visualization_msgs::Marker::DELETE;
  return marker;
}

bool AppendBoundaryPoints(const TubeSampleVector& samples,
                          visualization_msgs::Marker& lower,
                          visualization_msgs::Marker& upper,
                          visualization_msgs::Marker& ribbon) {
  if (samples.size() < 2U) return false;
  for (const auto& sample : samples) {
    if (!BoundaryPointFinite(sample)) return false;
    lower.points.push_back(ToPoint(sample.p + sample.N * sample.filtered_lower));
    upper.points.push_back(ToPoint(sample.p + sample.N * sample.filtered_upper));
  }
  for (std::size_t index = 0U; index + 1U < samples.size(); ++index) {
    const auto& sample = samples[index];
    const auto& next = samples[index + 1U];
    const Eigen::Vector3d lower_i = sample.p + sample.N * sample.filtered_lower;
    const Eigen::Vector3d upper_i = sample.p + sample.N * sample.filtered_upper;
    const Eigen::Vector3d lower_next = next.p + next.N * next.filtered_lower;
    const Eigen::Vector3d upper_next = next.p + next.N * next.filtered_upper;
    ribbon.points.push_back(ToPoint(lower_i));
    ribbon.points.push_back(ToPoint(upper_i));
    ribbon.points.push_back(ToPoint(upper_next));
    ribbon.points.push_back(ToPoint(lower_i));
    ribbon.points.push_back(ToPoint(upper_next));
    ribbon.points.push_back(ToPoint(lower_next));
  }
  return ribbon.points.size() == 6U * (samples.size() - 1U);
}

bool AppendBoundaryPoints(const phase_offset_navigation::TubeProfile& profile,
                          visualization_msgs::Marker& lower,
                          visualization_msgs::Marker& upper,
                          visualization_msgs::Marker& ribbon) {
  if (!profile.complete) return false;
  return AppendBoundaryPoints(profile.samples, lower, upper, ribbon);
}

bool ExactPhase(const double first, const double second) {
  std::uint64_t first_bits = 0U;
  std::uint64_t second_bits = 0U;
  static_assert(sizeof(first_bits) == sizeof(first), "double width changed");
  std::memcpy(&first_bits, &first, sizeof(first_bits));
  std::memcpy(&second_bits, &second, sizeof(second_bits));
  return first_bits == second_bits;
}

bool CandidateRawSampleDisplayable(
    const phase_offset_navigation::TubeRawSample& sample) {
  return sample.complete && std::isfinite(sample.w) &&
      BoundaryPointFinite(sample);
}

bool SelectCandidateDisplaySamples(
    const phase_offset_navigation::TubeProfile& profile,
    const double current_w,
    TubeSampleVector& selected) {
  selected.clear();
  if (profile.source == phase_offset_navigation::TubeSource::NONE) {
    return false;
  }
  if (!std::isfinite(current_w)) {
    if (!profile.complete) return false;
    selected = profile.samples;
    return selected.size() >= 2U;
  }
  if (profile.raw_build_samples.empty()) {
    if (!profile.complete) return false;
    selected = profile.samples;
    return selected.size() >= 2U;
  }

  const auto& raw = profile.raw_build_samples;
  std::size_t anchor = raw.size();
  for (std::size_t index = 0U; index < raw.size(); ++index) {
    if (ExactPhase(raw[index].w, current_w)) {
      anchor = index;
      break;
    }
  }
  if (anchor == raw.size() || !CandidateRawSampleDisplayable(raw[anchor])) {
    return false;
  }

  std::size_t first = anchor;
  std::size_t last = anchor;
  while (first > 0U &&
         CandidateRawSampleDisplayable(raw[first - 1U]) &&
         raw[first].w > raw[first - 1U].w) {
    --first;
  }
  while (last + 1U < raw.size() &&
         CandidateRawSampleDisplayable(raw[last + 1U]) &&
         raw[last + 1U].w > raw[last].w) {
    ++last;
  }
  if (last <= first) return false;
  selected.assign(raw.begin() + first, raw.begin() + last + 1U);
  return true;
}

visualization_msgs::MarkerArray MakeDeleteAll(const ros::Time& stamp,
                                              const std::string& frame_id,
                                              const std::string& ns) {
  visualization_msgs::MarkerArray markers;
  for (int id = 0; id < 3; ++id) {
    markers.markers.push_back(MakeDelete(stamp, frame_id, ns, id));
  }
  return markers;
}

}  // namespace

bool TubeProfileGeometryDisplayable(
    const phase_offset_navigation::TubeProfile& profile) {
  if (profile.source == phase_offset_navigation::TubeSource::NONE ||
      !profile.complete || profile.samples.size() < 2U) {
    return false;
  }
  for (const auto& sample : profile.samples) {
    if (!BoundaryPointFinite(sample)) return false;
  }
  return true;
}

visualization_msgs::MarkerArray MakeCertifiedTubeMarkers(
    const ros::Time& stamp,
    const std::string& frame_id,
    const phase_offset_navigation::TubeProfile& profile,
    const bool certified) {
  if (!certified) return MakeDeleteAll(stamp, frame_id, "phase_offset_manual_tube");

  auto lower = MakeTubeLine(stamp, frame_id, "phase_offset_manual_tube",
                            0, 0.06F, 1.0F, 0.6F, 0.0F);
  auto upper = MakeTubeLine(stamp, frame_id, "phase_offset_manual_tube",
                            1, 0.06F, 0.0F, 1.0F, 1.0F);
  auto ribbon = MakeTubeRibbon(stamp, frame_id, "phase_offset_manual_tube",
                               0.0F, 0.7F, 1.0F, 0.20F);
  if (!AppendBoundaryPoints(profile, lower, upper, ribbon)) {
    return MakeDeleteAll(stamp, frame_id, "phase_offset_manual_tube");
  }
  visualization_msgs::MarkerArray markers;
  markers.markers.push_back(lower);
  markers.markers.push_back(upper);
  markers.markers.push_back(ribbon);
  return markers;
}

visualization_msgs::MarkerArray MakeCandidateTubeMarkers(
    const ros::Time& stamp,
    const std::string& frame_id,
    const phase_offset_navigation::TubeProfile& profile,
    const bool manual_mode) {
  return MakeCandidateTubeMarkers(stamp, frame_id, profile, manual_mode,
                                  std::numeric_limits<double>::quiet_NaN());
}

visualization_msgs::MarkerArray MakeCandidateTubeMarkers(
    const ros::Time& stamp,
    const std::string& frame_id,
    const phase_offset_navigation::TubeProfile& profile,
    const bool manual_mode,
    const double current_w) {
  TubeSampleVector selected;
  if (!manual_mode || !SelectCandidateDisplaySamples(profile, current_w,
                                                      selected)) {
    return MakeDeleteAll(stamp, frame_id, "phase_offset_manual_tube_candidate");
  }

  auto lower = MakeTubeLine(stamp, frame_id, "phase_offset_manual_tube_candidate",
                            0, 0.04F, 0.55F, 0.50F, 0.40F);
  auto upper = MakeTubeLine(stamp, frame_id, "phase_offset_manual_tube_candidate",
                            1, 0.04F, 0.55F, 0.50F, 0.40F);
  auto ribbon = MakeTubeRibbon(stamp, frame_id, "phase_offset_manual_tube_candidate",
                               0.55F, 0.50F, 0.40F, 0.10F);
  if (!AppendBoundaryPoints(selected, lower, upper, ribbon)) {
    return MakeDeleteAll(stamp, frame_id, "phase_offset_manual_tube_candidate");
  }
  visualization_msgs::MarkerArray markers;
  markers.markers.push_back(lower);
  markers.markers.push_back(upper);
  markers.markers.push_back(ribbon);
  return markers;
}

}  // namespace FLAG_Race
