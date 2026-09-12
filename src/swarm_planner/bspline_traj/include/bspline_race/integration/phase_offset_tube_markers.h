#pragma once

#include <bspline_race/continuous_phase_normal_frame.h>
#include <phase_offset_navigation/section_tube.h>
#include <phase_offset_navigation/tube_profile_v2.h>
#include <phase_offset_navigation/tube_types.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <memory>

namespace FLAG_Race {

// Geometry-only predicate for the UNCERTIFIED candidate tube display.
bool TubeProfileGeometryDisplayable(
    const phase_offset_navigation::TubeProfile& profile);

// M3C Section visualization.  This entry point reads one immutable planner
// path together with one committed SectionTubeProfile and draws the two
// horizontal boundaries p(w) + N(w) * lower and p(w) + N(w) * upper as
// LINE_STRIP markers.  It is display-only: no map identity, certificate,
// worker, runtime, reserve, or control state is consulted, and nothing here
// can influence execution.  Missing, non-finite, out-of-domain, or
// fewer-than-two-knot input produces one complete DELETE bundle for both
// namespaces, so RViz never keeps showing stale geometry.
visualization_msgs::MarkerArray MakeSectionTubeMarkers(
    const ros::Time& stamp,
    const std::string& frame_id,
    const std::shared_ptr<const ContinuousPhasePath>& path,
    const std::shared_ptr<const phase_offset_navigation::SectionTubeProfile>& profile);

// Certified tube topic markers. `certified` is the adapter's full display
// certificate; all geometry is read from the same TubeProfile.
visualization_msgs::MarkerArray MakeCertifiedTubeMarkers(
    const ros::Time& stamp,
    const std::string& frame_id,
    const phase_offset_navigation::TubeProfile& profile,
    bool certified);

// R3 filtered Candidate geometry markers.  This display is independent of
// execution-qualified `/tube`: when `displayable` is true, geometry is read
// only from the immutable profile's filtered samples.  The caller owns the
// full provenance/current-request predicate; malformed geometry still fails
// closed to one all-DELETE bundle.
visualization_msgs::MarkerArray MakeCertifiedGeometryTubeMarkers(
    const ros::Time& stamp,
    const std::string& frame_id,
    const phase_offset_navigation::TubeProfile& profile,
    bool displayable);

// UNCERTIFIED candidate tube markers. `manual_mode` is the only adapter state
// considered in addition to the profile geometry.  Production callers use the
// exact-current overload below so a failed certification may still expose the
// same epoch's pre-certification raw build geometry without granting control.
visualization_msgs::MarkerArray MakeCandidateTubeMarkers(
    const ros::Time& stamp,
    const std::string& frame_id,
    const phase_offset_navigation::TubeProfile& profile,
    bool manual_mode);

// Production candidate publication supplies the exact current phase from the
// immutable command/epoch snapshot.  The four-argument overload above remains
// for deterministic callers that already provide a complete profile.
visualization_msgs::MarkerArray MakeCandidateTubeMarkers(
    const ros::Time& stamp,
    const std::string& frame_id,
    const phase_offset_navigation::TubeProfile& profile,
    bool manual_mode,
    double current_w);

// V2 visualization reads the certified PWL bounds directly and evaluates the
// immutable path/frame owner at each knot.  These overloads deliberately do
// not translate V2 evidence into a legacy TubeProfile.  Missing, malformed or
// path/frame-mismatched evidence produces one complete DELETE bundle.
visualization_msgs::MarkerArray MakeCertifiedTubeMarkersV2(
    const ros::Time& stamp,
    const std::string& frame_id,
    const std::shared_ptr<const phase_offset_navigation::TubeProfileV2>& profile,
    const std::shared_ptr<const ContinuousPhaseNormalFrame>& frame_owner);

visualization_msgs::MarkerArray MakeCandidateTubeMarkersV2(
    const ros::Time& stamp,
    const std::string& frame_id,
    const std::shared_ptr<const phase_offset_navigation::TubeProfileV2>& profile,
    const std::shared_ptr<const ContinuousPhaseNormalFrame>& frame_owner);

}  // namespace FLAG_Race
