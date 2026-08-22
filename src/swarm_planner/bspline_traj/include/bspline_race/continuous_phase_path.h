#ifndef BSPLINE_RACE_CONTINUOUS_PHASE_PATH_H
#define BSPLINE_RACE_CONTINUOUS_PHASE_PATH_H

#include <Eigen/Dense>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <bspline_race/UniformBspline_3d.h>
#include <phase_offset_core/path_state.h>

namespace FLAG_Race
{

struct ContinuousPhasePathState
{
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Vector3d dp_dw = Eigen::Vector3d::Zero();
    Eigen::Vector3d d2p_dw2 = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    bool valid = false;
};

class ContinuousPhasePath
{
public:
    using PointEvaluator = std::function<bool(double, ContinuousPhasePathState&)>;
    using CellBoundEvaluator = std::function<bool(
        double, double, phase_offset_core::PathCellGeometryCertificate&)>;

    // A path evaluator and its optional interval proof travel as one value.
    // The callback is optional so legacy/synthetic evaluators remain
    // explicitly unsupported for certified-cell use rather than being
    // silently approximated from point samples.
    class Evaluator
    {
    public:
        Evaluator() = default;
        Evaluator(const PointEvaluator& point_evaluator,
                  const CellBoundEvaluator& cell_bound_evaluator =
                      CellBoundEvaluator())
            : point_evaluator_(point_evaluator),
              cell_bound_evaluator_(cell_bound_evaluator) {}

        template <typename Callable,
                  typename std::enable_if<!std::is_same<
                      typename std::decay<Callable>::type, Evaluator>::value,
                      int>::type = 0>
        Evaluator(Callable&& point_evaluator)
            : point_evaluator_(std::forward<Callable>(point_evaluator)) {}

        bool operator()(double w, ContinuousPhasePathState& state) const {
            return point_evaluator_ && point_evaluator_(w, state);
        }
        explicit operator bool() const {
            return static_cast<bool>(point_evaluator_);
        }
        bool cellBounds(
            double w0, double w1,
            phase_offset_core::PathCellGeometryCertificate& certificate) const {
            return cell_bound_evaluator_ &&
                cell_bound_evaluator_(w0, w1, certificate);
        }
        bool hasCellBounds() const {
            return static_cast<bool>(cell_bound_evaluator_);
        }

    private:
        PointEvaluator point_evaluator_;
        CellBoundEvaluator cell_bound_evaluator_;
    };

    struct Segment
    {
        double w0 = 0.0;
        double w1 = 0.0;
        std::uint64_t identity = 0U;
        std::string label;
        Evaluator evaluate;
    };

    bool appendSegment(double w0,
                       double w1,
                       const std::string& label,
                       const Evaluator& evaluator);
    bool appendSlice(const ContinuousPhasePath& source,
                     double slice_w0,
                     double slice_w1);

    bool evaluate(double w,
                  ContinuousPhasePathState& state,
                  bool clamp_to_domain = true) const;
    // Returns a certificate only when [w0,w1] belongs to exactly one stored
    // segment.  Crossing or unsupported cells intentionally return false.
    bool cellBounds(double w0, double w1,
                    phase_offset_core::PathCellGeometryCertificate& certificate) const;
    bool sample(double step_w,
                std::vector<double>& w,
                std::vector<ContinuousPhasePathState>& states) const;

    bool empty() const { return segments_.empty(); }
    double startW() const;
    double endW() const;
    const std::vector<Segment>& segments() const { return segments_; }

    static Evaluator makeQuinticHermite(
        double w0,
        double w1,
        const ContinuousPhasePathState& start,
        const ContinuousPhasePathState& end);

    static Evaluator makeMappedBspline(
        const UniformBspline& position,
        double spline_t_anchor,
        double spline_t_end,
        double phase_w_anchor,
        double phase_w_end);

    // This is the same deterministic quintic/Simpson arclength primitive used
    // by makeMappedBspline and endpoint trimming.  It measures the raw spline
    // interval; callers retain ownership of their phase coordinates.
    static bool measureBsplineArcLength(
        const UniformBspline& position,
        double spline_t_start,
        double spline_t_end,
        double& total_length);

    static bool trimBsplineTimeDomainByArcLength(
        const UniformBspline& position,
        double spline_t_start,
        double spline_t_end,
        double trim_start_length,
        double trim_end_length,
        double& executable_t_start,
        double& executable_t_end,
        double& total_length);

    static Evaluator makePeriodicCircle(
        const Eigen::Vector3d& center,
        double radius,
        double period_w,
        double nominal_speed);

    static Evaluator makePeriodicFigureEight(
        const Eigen::Vector3d& center,
        double radius,
        double period_w,
        double nominal_speed);

private:
    std::vector<Segment> segments_;
    std::uint64_t next_segment_identity_ = 1U;
};

}  // namespace FLAG_Race

#endif
