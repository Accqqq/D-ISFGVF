#ifndef _REFERENCE_JOIN_H
#define _REFERENCE_JOIN_H

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

namespace FLAG_Race
{

struct ReferenceJoinState
{
    bool active = true;
    int idx = -1;
    int stable_count = 0;
};

struct ReferenceJoinSelection
{
    int best_idx = 0;
    int goal_idx = 0;
    double distance = std::numeric_limits<double>::infinity();
    bool exited = false;
};

inline int wrapReferenceIndex(int idx, int size)
{
    if (size <= 0) return 0;
    return ((idx % size) + size) % size;
}

inline ReferenceJoinSelection selectReferenceJoinGoal(const Eigen::MatrixXd& traj,
                                                      const Eigen::Vector3d& curr_pos,
                                                      int lookahead,
                                                      int search_window,
                                                      double exit_dist,
                                                      int exit_stable_needed,
                                                      bool progress_initialized,
                                                      ReferenceJoinState& state)
{
    ReferenceJoinSelection selection;
    const int n = static_cast<int>(traj.rows());
    if (n <= 0 || traj.cols() < 3) {
        state.active = false;
        selection.exited = true;
        return selection;
    }

    const int window = std::max(0, search_window);
    double best_dist_sq = std::numeric_limits<double>::infinity();

    auto considerIndex = [&](int raw_idx) {
        const int idx = wrapReferenceIndex(raw_idx, n);
        const Eigen::Vector3d pt = traj.row(idx).transpose();
        const double dist_sq = (pt - curr_pos).squaredNorm();
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            selection.best_idx = idx;
        }
    };

    if (state.idx < 0 || state.idx >= n) {
        for (int i = 0; i < n; ++i) {
            considerIndex(i);
        }
    } else {
        for (int dk = -window; dk <= window; ++dk) {
            considerIndex(state.idx + dk);
        }
    }

    selection.distance = std::sqrt(std::max(0.0, best_dist_sq));
    state.idx = selection.best_idx;

    if (selection.distance <= exit_dist) {
        state.stable_count += 1;
    } else {
        state.stable_count = 0;
    }

    if (progress_initialized && state.stable_count >= std::max(1, exit_stable_needed)) {
        state.active = false;
        selection.exited = true;
    }

    selection.goal_idx = wrapReferenceIndex(selection.best_idx + std::max(1, lookahead), n);
    return selection;
}

}  // namespace FLAG_Race

#endif
