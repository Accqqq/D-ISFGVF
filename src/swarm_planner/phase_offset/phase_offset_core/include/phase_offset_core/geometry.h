#pragma once

#include <Eigen/Core>

#include "phase_offset_core/geometry_types.h"
#include "phase_offset_core/path_state.h"

namespace phase_offset_core {

class GeometryEvaluator {
 public:
  explicit GeometryEvaluator(const GeometryParams& params = GeometryParams());

  bool evaluate(const PathDifferentialState& path,
                const Eigen::Vector3d& position,
                double delta,
                PhaseOffsetGeometryState& output) const;

  void preparePath(const PathDifferentialState& path,
                   PreparedPathGeometry& output) const;

  bool evaluatePreparedReference(const PreparedPathGeometry& prepared,
                                 const Eigen::Vector3d& position,
                                 double delta,
                                 PreparedReferenceResult& output) const;

 private:
  GeometryParams params_;
};

}  // namespace phase_offset_core
