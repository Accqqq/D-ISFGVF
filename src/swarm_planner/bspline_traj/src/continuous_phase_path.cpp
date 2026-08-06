#include <bspline_race/continuous_phase_path.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace FLAG_Race
{
namespace
{
constexpr double kDomainEps = 1e-8;

bool finiteState(const ContinuousPhasePathState& state)
{
    return state.p.allFinite() && state.dp_dw.allFinite() &&
           state.d2p_dw2.allFinite() && state.vel.allFinite();
}

Eigen::Vector3d evaluateSplineAtTime(const UniformBspline& spline, double t)
{
    const double u = t * spline.beta_ + spline.u_(spline.p_);
    return spline.singleDeboor(u);
}
}  // namespace

bool ContinuousPhasePath::appendSegment(
    double w0,
    double w1,
    const std::string& label,
    const Evaluator& evaluator)
{
    if (!std::isfinite(w0) || !std::isfinite(w1) ||
        w1 <= w0 + kDomainEps || !evaluator) {
        return false;
    }
    if (!segments_.empty() && w0 < segments_.back().w1 - kDomainEps) {
        return false;
    }

    Segment segment;
    segment.w0 = w0;
    segment.w1 = w1;
    segment.label = label;
    segment.evaluate = evaluator;
    segments_.push_back(segment);
    return true;
}

bool ContinuousPhasePath::appendSlice(
    const ContinuousPhasePath& source,
    double slice_w0,
    double slice_w1)
{
    if (!std::isfinite(slice_w0) || !std::isfinite(slice_w1) ||
        slice_w1 <= slice_w0 + kDomainEps) {
        return false;
    }

    bool appended = false;
    for (const auto& segment : source.segments_) {
        const double w0 = std::max(slice_w0, segment.w0);
        const double w1 = std::min(slice_w1, segment.w1);
        if (w1 <= w0 + kDomainEps) continue;
        if (!appendSegment(w0, w1, segment.label, segment.evaluate)) {
            return false;
        }
        appended = true;
    }
    return appended;
}

bool ContinuousPhasePath::evaluate(
    double w,
    ContinuousPhasePathState& state,
    bool clamp_to_domain) const
{
    state = ContinuousPhasePathState();
    if (segments_.empty() || !std::isfinite(w)) return false;

    double query_w = w;
    if (query_w < startW()) {
        if (!clamp_to_domain) return false;
        query_w = startW();
    } else if (query_w > endW()) {
        if (!clamp_to_domain) return false;
        query_w = endW();
    }

    const Segment* selected = nullptr;
    for (const auto& segment : segments_) {
        if (query_w >= segment.w0 - kDomainEps &&
            query_w <= segment.w1 + kDomainEps) {
            selected = &segment;
            break;
        }
    }
    if (!selected) return false;

    const double bounded_w = std::max(selected->w0, std::min(query_w, selected->w1));
    if (!selected->evaluate(bounded_w, state)) return false;
    state.valid = finiteState(state);
    return state.valid;
}

bool ContinuousPhasePath::sample(
    double step_w,
    std::vector<double>& w,
    std::vector<ContinuousPhasePathState>& states) const
{
    w.clear();
    states.clear();
    if (empty() || !std::isfinite(step_w) || step_w <= 0.0) return false;

    const double span = endW() - startW();
    const int count = std::max(2, static_cast<int>(std::ceil(span / step_w)) + 1);
    w.reserve(count);
    states.reserve(count);
    for (int i = 0; i < count; ++i) {
        const double ratio = static_cast<double>(i) /
            static_cast<double>(count - 1);
        const double query_w = startW() + ratio * span;
        ContinuousPhasePathState state;
        if (!evaluate(query_w, state, false)) {
            w.clear();
            states.clear();
            return false;
        }
        w.push_back(query_w);
        states.push_back(state);
    }
    return true;
}

double ContinuousPhasePath::startW() const
{
    return segments_.empty() ? 0.0 : segments_.front().w0;
}

double ContinuousPhasePath::endW() const
{
    return segments_.empty() ? 0.0 : segments_.back().w1;
}

ContinuousPhasePath::Evaluator ContinuousPhasePath::makeQuinticHermite(
    double w0,
    double w1,
    const ContinuousPhasePathState& start,
    const ContinuousPhasePathState& end)
{
    if (!std::isfinite(w0) || !std::isfinite(w1) ||
        w1 <= w0 + kDomainEps || !finiteState(start) || !finiteState(end)) {
        return Evaluator();
    }

    const double h = w1 - w0;
    const Eigen::Vector3d a0 = start.p;
    const Eigen::Vector3d a1 = h * start.dp_dw;
    const Eigen::Vector3d a2 = 0.5 * h * h * start.d2p_dw2;
    const Eigen::Vector3d r0 = end.p - a0 - a1 - a2;
    const Eigen::Vector3d r1 = h * end.dp_dw - a1 - 2.0 * a2;
    const Eigen::Vector3d r2 = h * h * end.d2p_dw2 - 2.0 * a2;
    const Eigen::Vector3d a3 = 10.0 * r0 - 4.0 * r1 + 0.5 * r2;
    const Eigen::Vector3d a4 = -15.0 * r0 + 7.0 * r1 - r2;
    const Eigen::Vector3d a5 = 6.0 * r0 - 3.0 * r1 + 0.5 * r2;
    const double speed0 = std::max(0.0, start.vel.norm());
    const double speed1 = std::max(0.0, end.vel.norm());

    return [=](double w, ContinuousPhasePathState& state) {
        const double s = std::max(0.0, std::min(1.0, (w - w0) / h));
        const double s2 = s * s;
        const double s3 = s2 * s;
        const double s4 = s3 * s;
        const double s5 = s4 * s;
        state.p = a0 + a1 * s + a2 * s2 + a3 * s3 + a4 * s4 + a5 * s5;
        state.dp_dw = (a1 + 2.0 * a2 * s + 3.0 * a3 * s2 +
                       4.0 * a4 * s3 + 5.0 * a5 * s4) / h;
        state.d2p_dw2 = (2.0 * a2 + 6.0 * a3 * s + 12.0 * a4 * s2 +
                         20.0 * a5 * s3) / (h * h);
        const double blend = s * s * (3.0 - 2.0 * s);
        const double speed = (1.0 - blend) * speed0 + blend * speed1;
        if (state.dp_dw.norm() > 1e-9) {
            state.vel = state.dp_dw.normalized() * speed;
        } else {
            state.vel.setZero();
        }
        state.valid = finiteState(state);
        return state.valid;
    };
}

ContinuousPhasePath::Evaluator ContinuousPhasePath::makeMappedBspline(
    const UniformBspline& position,
    double spline_t_anchor,
    double spline_t_end,
    double phase_w_anchor,
    double phase_w_end)
{
    if (!std::isfinite(spline_t_anchor) || !std::isfinite(spline_t_end) ||
        !std::isfinite(phase_w_anchor) || !std::isfinite(phase_w_end) ||
        spline_t_end <= spline_t_anchor + kDomainEps ||
        phase_w_end <= phase_w_anchor + kDomainEps) {
        return Evaluator();
    }

    const UniformBspline p = position;
    const UniformBspline dp_dt = p.getDerivative();
    const UniformBspline d2p_dt2 = dp_dt.getDerivative();
    const double dt_dw = (spline_t_end - spline_t_anchor) /
                         (phase_w_end - phase_w_anchor);

    return [=](double w, ContinuousPhasePathState& state) {
        const double bounded_w = std::max(
            phase_w_anchor, std::min(w, phase_w_end));
        const double t = spline_t_anchor + dt_dw * (bounded_w - phase_w_anchor);
        state.p = evaluateSplineAtTime(p, t);
        state.vel = evaluateSplineAtTime(dp_dt, t);
        state.dp_dw = state.vel * dt_dw;
        state.d2p_dw2 = evaluateSplineAtTime(d2p_dt2, t) * dt_dw * dt_dw;
        state.valid = finiteState(state);
        return state.valid;
    };
}

ContinuousPhasePath::Evaluator ContinuousPhasePath::makePeriodicCircle(
    const Eigen::Vector3d& center,
    double radius,
    double period_w,
    double nominal_speed)
{
    if (!center.allFinite() || !std::isfinite(radius) || radius <= 0.0 ||
        !std::isfinite(period_w) || period_w <= kDomainEps) {
        return Evaluator();
    }
    const double omega = 2.0 * M_PI / period_w;
    const double speed = std::max(0.0, nominal_speed);
    return [=](double w, ContinuousPhasePathState& state) {
        const double theta = omega * w;
        const double c = std::cos(theta);
        const double s = std::sin(theta);
        state.p = center + Eigen::Vector3d(radius * c, radius * s, 0.0);
        state.dp_dw = Eigen::Vector3d(-radius * omega * s,
                                      radius * omega * c, 0.0);
        state.d2p_dw2 = Eigen::Vector3d(-radius * omega * omega * c,
                                        -radius * omega * omega * s, 0.0);
        state.vel = state.dp_dw.normalized() * speed;
        state.valid = finiteState(state);
        return state.valid;
    };
}

ContinuousPhasePath::Evaluator ContinuousPhasePath::makePeriodicFigureEight(
    const Eigen::Vector3d& center,
    double radius,
    double period_w,
    double nominal_speed)
{
    if (!center.allFinite() || !std::isfinite(radius) || radius <= 0.0 ||
        !std::isfinite(period_w) || period_w <= kDomainEps) {
        return Evaluator();
    }
    const double omega = 2.0 * M_PI / period_w;
    const double speed = std::max(0.0, nominal_speed);
    return [=](double w, ContinuousPhasePathState& state) {
        const double theta = omega * w;
        const double sin_theta = std::sin(theta);
        const double cos_theta = std::cos(theta);
        const double sin_2theta = std::sin(2.0 * theta);
        const double cos_2theta = std::cos(2.0 * theta);
        state.p = center + Eigen::Vector3d(0.5 * radius * sin_2theta,
                                           radius * sin_theta, 0.0);
        state.dp_dw = Eigen::Vector3d(radius * omega * cos_2theta,
                                      radius * omega * cos_theta, 0.0);
        state.d2p_dw2 = Eigen::Vector3d(-2.0 * radius * omega * omega * sin_2theta,
                                        -radius * omega * omega * sin_theta, 0.0);
        state.vel = state.dp_dw.normalized() * speed;
        state.valid = finiteState(state);
        return state.valid;
    };
}

}  // namespace FLAG_Race
