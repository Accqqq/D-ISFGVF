#pragma once

#include <phase_offset_navigation/environment_evidence.h>
#include <plan_env/sdf_map_environment_evidence.h>

class SDFMap;

namespace FLAG_Race {

// Converts only the SDFMap-local evidence result into the ROS-free navigation
// contract.  This helper is intentionally unused by production code in
// A5-G2c.
phase_offset_navigation::EnvironmentEvidenceResult
translateSDFMapEnvironmentEvidence(
    const plan_env::SDFMapEnvironmentEvidenceResult& result);

phase_offset_navigation::EnvironmentEvidenceQuery makeEnvironmentEvidenceQuery(
    SDFMap* map);

}  // namespace FLAG_Race
