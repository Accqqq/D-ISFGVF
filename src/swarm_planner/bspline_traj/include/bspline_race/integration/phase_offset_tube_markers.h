#pragma once

#include <phase_offset_navigation/tube_types.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>

namespace FLAG_Race {

// Geometry-only predicate for the UNCERTIFIED candidate tube display.
bool TubeProfileGeometryDisplayable(
    const phase_offset_navigation::TubeProfile& profile);

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

}  // namespace FLAG_Race
