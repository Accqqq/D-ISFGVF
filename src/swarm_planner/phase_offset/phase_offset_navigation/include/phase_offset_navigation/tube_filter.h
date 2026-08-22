#pragma once

#include "phase_offset_navigation/tube_types.h"

namespace phase_offset_navigation {

class TubeFilter {
 public:
  explicit TubeFilter(const TubeFilterConfig& config = TubeFilterConfig());

  bool configurationValid() const;
  // Filters only the continuous raw geometry segment anchored at the live
  // phase.  Retained-delta containment is an installation/runtime question,
  // not Candidate geometry completeness.  A remote preview conflict must
  // truncate this local corridor, never invalidate it wholesale.
  bool filter(TubeProfile& profile, double current_w) const;
  static bool query(const TubeProfile& profile, double w, TubeBounds& bounds);

 private:
  TubeFilterConfig config_;
};

}  // namespace phase_offset_navigation
