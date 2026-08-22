#pragma once

#include <phase_offset_navigation/tube_cross_section.h>

#include <Eigen/Core>

#include <functional>

class SDFMap;

namespace FLAG_Race {

// DEPRECATED TEST-ONLY REFERENCE (A5-G2d): no production tube path links or
// invokes this raw-log-odds bridge.  Cloud occupycloud tubes use
// phase_offset_cloud_occupancy_query instead.

// Local evidence that the current UAV already physically occupies a small
// volume.  It is an integration-layer query overlay only; it never changes
// SDFMap storage or certifies raw UNKNOWN outside this exact sphere.
struct RawOccupancySelfFreeSeed {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double radius = 0.0;
};

// Test-visible origin of a raw bridge classification.  Navigation continues
// to receive only DistanceStatus, so neither its public status enum nor ROS
// diagnostics schemas change.
enum class RawOccupancyProbeEvidence {
  UNAVAILABLE,
  OUT_OF_MAP,
  RAW_OCCUPIED,
  RAW_UNKNOWN,
  RAW_KNOWN_FREE,
  KNOWN_FREE_BY_SELF,
};

struct RawOccupancyProbeResult {
  phase_offset_navigation::DistanceStatus status =
      phase_offset_navigation::DistanceStatus::UNAVAILABLE;
  RawOccupancyProbeEvidence evidence = RawOccupancyProbeEvidence::UNAVAILABLE;
};

using RawOccupancyProbe = std::function<RawOccupancyProbeResult(
    const Eigen::Vector3d&)>;

// True only when SDFMap's public raw occupancy storage is present for every
// configured voxel.  This guards the G2a compatibility fallback for a map
// object that has not initialized raw occupancy storage at all; it is not a
// readiness verdict for observed/unknown cells.
bool rawOccupancyStorageReady(const SDFMap* map);

// Classifies the raw occupancy result and, only for a raw UNKNOWN inside a
// valid self-free seed, projects it to KNOWN_FREE with explicit evidence.
RawOccupancyProbe makeRawOccupancyProbe(
    SDFMap* map, const RawOccupancySelfFreeSeed& seed);

// Preserves the original no-seed status semantics for existing callers.
RawOccupancyProbe makeRawOccupancyProbe(SDFMap* map);

// Adapts only SDFMap's raw occupancy layer to the ROS-free navigation query.
// SDFMap ownership stays with the caller and the query must not be used as an
// ESDF or inflated-occupancy proxy.
phase_offset_navigation::RawOccupancyQuery makeRawOccupancyQuery(
    SDFMap* map, const RawOccupancySelfFreeSeed& seed);

// Preserves the original no-seed status semantics for existing callers.
phase_offset_navigation::RawOccupancyQuery makeRawOccupancyQuery(SDFMap* map);

}  // namespace FLAG_Race
