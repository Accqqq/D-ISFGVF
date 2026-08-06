#ifndef BSPLINE_RACE_CONTINUOUS_PHASE_PATH_H
#define BSPLINE_RACE_CONTINUOUS_PHASE_PATH_H

#include <Eigen/Dense>

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <bspline_race/UniformBspline_3d.h>

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
    using Evaluator = std::function<bool(double, ContinuousPhasePathState&)>;

    struct Segment
    {
        double w0 = 0.0;
        double w1 = 0.0;
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
};

}  // namespace FLAG_Race

#endif
