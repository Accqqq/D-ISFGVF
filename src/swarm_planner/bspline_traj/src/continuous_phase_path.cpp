#include <bspline_race/continuous_phase_path.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace FLAG_Race
{
namespace
{
constexpr double kDomainEps = 1e-8;
constexpr int kArcLengthTableIntervals = 1024;
constexpr double kArcLengthSpeedEps = 1e-8;
constexpr double kCertificateSpeedEps = 1e-9;
constexpr double kCertificateRoundoff = 1e-11;

struct ArcLengthCell
{
    double t0 = 0.0;
    double h = 0.0;
    std::array<double, 6> a = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
};

struct ArcLengthMap
{
    std::vector<double> table_t;
    std::vector<double> table_s;
    std::vector<double> table_speed;
    std::vector<double> table_speed_derivative;
    std::vector<ArcLengthCell> cells;
};

struct VectorBounds
{
    double inf_norm = 0.0;
    double sup_norm = 0.0;
    bool valid = false;
};

struct DifferentialBounds
{
    double inf_speed = 0.0;
    double inf_horizontal_speed = 0.0;
    double sup_speed = 0.0;
    double sup_acceleration = 0.0;
    double sup_horizontal_acceleration = 0.0;
    double sup_jerk = 0.0;
    double sup_horizontal_jerk = 0.0;
    bool valid = false;
};

double UpperBound(const double value)
{
    if (!std::isfinite(value) || value < 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::nextafter(value * (1.0 + kCertificateRoundoff) +
                              kCertificateRoundoff,
                          std::numeric_limits<double>::infinity());
}

double LowerBound(const double value)
{
    if (!std::isfinite(value) || value < 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (value == 0.0) return 0.0;
    const double lowered = value * (1.0 - kCertificateRoundoff) -
        kCertificateRoundoff;
    return std::max(0.0, std::nextafter(
        lowered, -std::numeric_limits<double>::infinity()));
}

double Binomial(const int n, const int k)
{
    if (k < 0 || k > n) return 0.0;
    double result = 1.0;
    for (int index = 1; index <= k; ++index) {
        result *= static_cast<double>(n - k + index) /
            static_cast<double>(index);
    }
    return result;
}

std::vector<Eigen::Vector3d> RestrictPowerVector(
    const std::vector<Eigen::Vector3d>& coefficients,
    const double first, const double last)
{
    std::vector<Eigen::Vector3d> restricted(coefficients.size(),
                                             Eigen::Vector3d::Zero());
    if (!std::isfinite(first) || !std::isfinite(last) ||
        last < first || coefficients.empty()) {
        return std::vector<Eigen::Vector3d>();
    }
    const double scale = last - first;
    for (std::size_t order = 0U; order < coefficients.size(); ++order) {
        if (!coefficients[order].allFinite()) {
            return std::vector<Eigen::Vector3d>();
        }
        for (std::size_t output_order = 0U; output_order <= order;
             ++output_order) {
            restricted[output_order] += coefficients[order] *
                (Binomial(static_cast<int>(order),
                          static_cast<int>(output_order)) *
                 std::pow(first,
                          static_cast<int>(order - output_order)) *
                 std::pow(scale, static_cast<int>(output_order)));
        }
    }
    return restricted;
}

std::vector<Eigen::Vector3d> VectorPowerToBernstein(
    const std::vector<Eigen::Vector3d>& coefficients)
{
    if (coefficients.empty()) return std::vector<Eigen::Vector3d>();
    const int degree = static_cast<int>(coefficients.size()) - 1;
    std::vector<Eigen::Vector3d> controls(
        coefficients.size(), Eigen::Vector3d::Zero());
    for (int control = 0; control <= degree; ++control) {
        for (int order = 0; order <= control; ++order) {
            if (!coefficients[static_cast<std::size_t>(order)].allFinite()) {
                return std::vector<Eigen::Vector3d>();
            }
            controls[static_cast<std::size_t>(control)] +=
                coefficients[static_cast<std::size_t>(order)] *
                (Binomial(control, order) / Binomial(degree, order));
        }
    }
    return controls;
}

std::vector<double> ScalarPowerToBernstein(const std::vector<double>& coefficients)
{
    if (coefficients.empty()) return std::vector<double>();
    const int degree = static_cast<int>(coefficients.size()) - 1;
    std::vector<double> controls(coefficients.size(), 0.0);
    for (int control = 0; control <= degree; ++control) {
        for (int order = 0; order <= control; ++order) {
            if (!std::isfinite(coefficients[static_cast<std::size_t>(order)])) {
                return std::vector<double>();
            }
            controls[static_cast<std::size_t>(control)] +=
                coefficients[static_cast<std::size_t>(order)] *
                (Binomial(control, order) / Binomial(degree, order));
        }
    }
    return controls;
}

std::vector<Eigen::Vector3d> DifferentiateVectorPower(
    const std::vector<Eigen::Vector3d>& coefficients)
{
    if (coefficients.size() <= 1U) {
        return std::vector<Eigen::Vector3d>(1U, Eigen::Vector3d::Zero());
    }
    std::vector<Eigen::Vector3d> derivative(coefficients.size() - 1U,
                                              Eigen::Vector3d::Zero());
    for (std::size_t order = 1U; order < coefficients.size(); ++order) {
        derivative[order - 1U] = coefficients[order] *
            static_cast<double>(order);
    }
    return derivative;
}

std::vector<double> DifferentiateScalarPower(const std::vector<double>& coefficients)
{
    if (coefficients.size() <= 1U) return std::vector<double>(1U, 0.0);
    std::vector<double> derivative(coefficients.size() - 1U, 0.0);
    for (std::size_t order = 1U; order < coefficients.size(); ++order) {
        derivative[order - 1U] = coefficients[order] *
            static_cast<double>(order);
    }
    return derivative;
}

VectorBounds BoundsFromBernsteinControls(
    const std::vector<Eigen::Vector3d>& controls, const bool horizontal)
{
    VectorBounds bounds;
    if (controls.empty()) return bounds;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const Eigen::Vector3d& control : controls) {
        if (!control.allFinite()) return bounds;
        const Eigen::Vector3d projected = horizontal
            ? Eigen::Vector3d(control.x(), control.y(), 0.0) : control;
        center += projected;
    }
    center /= static_cast<double>(controls.size());
    double radius = 0.0;
    double upper = 0.0;
    for (const Eigen::Vector3d& control : controls) {
        const Eigen::Vector3d projected = horizontal
            ? Eigen::Vector3d(control.x(), control.y(), 0.0) : control;
        radius = std::max(radius, (projected - center).norm());
        upper = std::max(upper, projected.norm());
    }
    const double lower = std::max(0.0, center.norm() - radius);
    bounds.inf_norm = LowerBound(lower);
    bounds.sup_norm = UpperBound(upper);
    bounds.valid = std::isfinite(bounds.inf_norm) &&
        std::isfinite(bounds.sup_norm) && bounds.sup_norm >= bounds.inf_norm;
    return bounds;
}

VectorBounds BoundsFromVectorPower(const std::vector<Eigen::Vector3d>& coefficients,
                                   const double first, const double last,
                                   const bool horizontal)
{
    return BoundsFromBernsteinControls(
        VectorPowerToBernstein(RestrictPowerVector(coefficients, first, last)),
        horizontal);
}

bool MakeCertificate(const double w0, const double w1,
                     const DifferentialBounds& differential,
                     phase_offset_core::PathCellGeometryCertificate& certificate)
{
    certificate = phase_offset_core::PathCellGeometryCertificate();
    const double h = w1 - w0;
    if (!differential.valid || !std::isfinite(w0) || !std::isfinite(w1) ||
        h <= kDomainEps ||
        differential.inf_speed <= kCertificateSpeedEps ||
        !std::isfinite(differential.sup_speed) ||
        !std::isfinite(differential.sup_acceleration) ||
        !std::isfinite(differential.sup_jerk)) {
        return false;
    }
    // Full 3D bounds are authoritative for frame-bound production paths.
    // Horizontal projections remain diagnostics only and may be exactly zero
    // on a near-vertical path.  For T=p_w/||p_w||,
    // ||T_w|| <= ||p_ww||/inf||p_w||; Bishop transport gives the same bound
    // for ||N_w||.  These are closed-cell conservative bounds, not endpoint
    // samples.
    const double normal_w = UpperBound(
        differential.sup_acceleration / differential.inf_speed);
    const double curvature = UpperBound(
        differential.sup_acceleration /
            (differential.inf_speed * differential.inf_speed));
    const double curvature_w = UpperBound(
        differential.sup_jerk /
            (differential.inf_speed * differential.inf_speed) +
        3.0 * differential.sup_acceleration * differential.sup_acceleration /
            (differential.inf_speed * differential.inf_speed *
             differential.inf_speed));
    certificate.w0 = w0;
    certificate.w1 = w1;
    certificate.inf_p_w_norm = differential.inf_speed;
    certificate.inf_horizontal_p_w_norm = differential.inf_horizontal_speed;
    certificate.sup_p_w_norm = differential.sup_speed;
    certificate.sup_p_ww_norm = differential.sup_acceleration;
    certificate.sup_p_www_norm = differential.sup_jerk;
    certificate.sup_N_w_norm = normal_w;
    certificate.sup_abs_curvature = curvature;
    certificate.normal_variation_bound = UpperBound(normal_w * h);
    certificate.curvature_variation_bound = UpperBound(curvature_w * h);
    certificate.midpoint_position_variation_bound = UpperBound(
        0.5 * differential.sup_speed * h);
    certificate.chord_deviation_bound = UpperBound(
        differential.sup_acceleration * h * h / 8.0);
    certificate.valid = std::isfinite(normal_w) && std::isfinite(curvature) &&
        std::isfinite(curvature_w);
    certificate.complete = certificate.valid;
    return certificate.valid;
}

bool CollectSplineBernsteinControls(const UniformBspline& spline,
                                    const double t0, const double t1,
                                    std::vector<Eigen::Vector3d>& controls)
{
    controls.clear();
    if (!std::isfinite(t0) || !std::isfinite(t1) || t1 < t0 ||
        !std::isfinite(spline.beta_) || spline.beta_ <= 0.0 ||
        spline.p_ < 0 || spline.m_ <= spline.p_ ||
        spline.u_.size() <= spline.m_ || spline.control_points_.cols() != 3 ||
        spline.control_points_.rows() == 0) {
        return false;
    }
    const double u0 = t0 * spline.beta_ + spline.u_(spline.p_);
    const double u1 = t1 * spline.beta_ + spline.u_(spline.p_);
    const int first_span = spline.p_;
    const int last_span = spline.m_ - spline.p_ - 1;
    if (!std::isfinite(u0) || !std::isfinite(u1) || last_span < first_span ||
        u0 < spline.u_(first_span) - kDomainEps ||
        u1 > spline.u_(last_span + 1) + kDomainEps) {
        return false;
    }
    std::vector<bool> selected(
        static_cast<std::size_t>(spline.control_points_.rows()), false);
    for (int span = first_span; span <= last_span; ++span) {
        if (spline.u_(span + 1) < u0 - kDomainEps ||
            spline.u_(span) > u1 + kDomainEps) {
            continue;
        }
        const int control_first = span - spline.p_;
        const int control_last = span;
        if (control_first < 0 || control_last >= spline.control_points_.rows()) {
            return false;
        }
        for (int index = control_first; index <= control_last; ++index) {
            selected[static_cast<std::size_t>(index)] = true;
        }
    }
    for (std::size_t index = 0U; index < selected.size(); ++index) {
        if (!selected[index]) continue;
        const Eigen::Vector3d control = spline.control_points_.row(
            static_cast<Eigen::Index>(index));
        if (!control.allFinite()) return false;
        controls.push_back(control);
    }
    return !controls.empty();
}

bool BoundsFromSpline(const UniformBspline& spline, const double t0,
                      const double t1, VectorBounds& full,
                      VectorBounds& horizontal)
{
    std::vector<Eigen::Vector3d> controls;
    if (!CollectSplineBernsteinControls(spline, t0, t1, controls)) return false;
    full = BoundsFromBernsteinControls(controls, false);
    horizontal = BoundsFromBernsteinControls(controls, true);
    return full.valid && horizontal.valid;
}

bool MakeQuinticDifferentialBounds(
    const std::array<Eigen::Vector3d, 6>& a,
    const double segment_w0, const double segment_w1,
    const double w0, const double w1, DifferentialBounds& differential)
{
    differential = DifferentialBounds();
    if (!std::isfinite(segment_w0) || !std::isfinite(segment_w1) ||
        !std::isfinite(w0) || !std::isfinite(w1) ||
        w0 < segment_w0 - kDomainEps || w1 > segment_w1 + kDomainEps ||
        w1 <= w0 + kDomainEps) {
        return false;
    }
    const double h_segment = segment_w1 - segment_w0;
    if (h_segment <= kDomainEps) return false;
    const double first = std::max(0.0, std::min(1.0,
        (w0 - segment_w0) / h_segment));
    const double last = std::max(0.0, std::min(1.0,
        (w1 - segment_w0) / h_segment));
    std::vector<Eigen::Vector3d> position(a.begin(), a.end());
    const std::vector<Eigen::Vector3d> p_w = DifferentiateVectorPower(position);
    const std::vector<Eigen::Vector3d> p_ww = DifferentiateVectorPower(p_w);
    const std::vector<Eigen::Vector3d> p_www = DifferentiateVectorPower(p_ww);
    const VectorBounds speed = BoundsFromVectorPower(p_w, first, last, false);
    const VectorBounds horizontal_speed = BoundsFromVectorPower(
        p_w, first, last, true);
    const VectorBounds acceleration = BoundsFromVectorPower(
        p_ww, first, last, false);
    const VectorBounds horizontal_acceleration = BoundsFromVectorPower(
        p_ww, first, last, true);
    const VectorBounds jerk = BoundsFromVectorPower(p_www, first, last, false);
    const VectorBounds horizontal_jerk = BoundsFromVectorPower(
        p_www, first, last, true);
    if (!speed.valid || !horizontal_speed.valid || !acceleration.valid ||
        !horizontal_acceleration.valid || !jerk.valid ||
        !horizontal_jerk.valid) {
        return false;
    }
    differential.inf_speed = LowerBound(speed.inf_norm / h_segment);
    differential.inf_horizontal_speed = LowerBound(
        horizontal_speed.inf_norm / h_segment);
    differential.sup_speed = UpperBound(speed.sup_norm / h_segment);
    differential.sup_acceleration = UpperBound(
        acceleration.sup_norm / (h_segment * h_segment));
    differential.sup_horizontal_acceleration = UpperBound(
        horizontal_acceleration.sup_norm / (h_segment * h_segment));
    differential.sup_jerk = UpperBound(
        jerk.sup_norm / (h_segment * h_segment * h_segment));
    differential.sup_horizontal_jerk = UpperBound(
        horizontal_jerk.sup_norm / (h_segment * h_segment * h_segment));
    differential.valid = std::isfinite(differential.inf_speed) &&
        std::isfinite(differential.inf_horizontal_speed) &&
        std::isfinite(differential.sup_speed) &&
        differential.sup_speed >= differential.inf_speed;
    return differential.valid;
}

bool MakeQuinticCertificate(
    const std::array<Eigen::Vector3d, 6>& a, const double segment_w0,
    const double segment_w1, const double w0, const double w1,
    phase_offset_core::PathCellGeometryCertificate& certificate)
{
    if (!std::isfinite(segment_w0) || !std::isfinite(segment_w1) ||
        !std::isfinite(w0) || !std::isfinite(w1) ||
        w0 < segment_w0 - kDomainEps || w1 > segment_w1 + kDomainEps ||
        w1 <= w0 + kDomainEps) {
        return false;
    }
    DifferentialBounds differential;
    if (MakeQuinticDifferentialBounds(a, segment_w0, segment_w1,
                                      w0, w1, differential) &&
        MakeCertificate(w0, w1, differential, certificate)) {
        return true;
    }
    // Bernstein control polygons can contain the origin even when the
    // quintic's derivative is strictly nonzero on the requested cell.  Split
    // only the certificate calculation into deterministic subcells, aggregate
    // their valid bounds, and keep the same fail-closed semantics if any
    // subcell still has no positive speed proof.
    constexpr int kSubdivisions = 32;
    DifferentialBounds aggregate;
    aggregate.inf_speed = std::numeric_limits<double>::infinity();
    aggregate.inf_horizontal_speed = std::numeric_limits<double>::infinity();
    aggregate.sup_speed = 0.0;
    aggregate.sup_acceleration = 0.0;
    aggregate.sup_horizontal_acceleration = 0.0;
    aggregate.sup_jerk = 0.0;
    aggregate.sup_horizontal_jerk = 0.0;
    aggregate.valid = true;
    const double span = w1 - w0;
    for (int index = 0; index < kSubdivisions; ++index) {
        const double local_w0 = w0 + span * static_cast<double>(index) /
            static_cast<double>(kSubdivisions);
        const double local_w1 = w0 + span * static_cast<double>(index + 1) /
            static_cast<double>(kSubdivisions);
        DifferentialBounds local;
        if (!MakeQuinticDifferentialBounds(a, segment_w0, segment_w1,
                                           local_w0, local_w1, local)) {
            aggregate.valid = false;
            break;
        }
        aggregate.inf_speed = std::min(aggregate.inf_speed, local.inf_speed);
        aggregate.inf_horizontal_speed = std::min(
            aggregate.inf_horizontal_speed, local.inf_horizontal_speed);
        aggregate.sup_speed = std::max(aggregate.sup_speed, local.sup_speed);
        aggregate.sup_acceleration = std::max(
            aggregate.sup_acceleration, local.sup_acceleration);
        aggregate.sup_horizontal_acceleration = std::max(
            aggregate.sup_horizontal_acceleration,
            local.sup_horizontal_acceleration);
        aggregate.sup_jerk = std::max(aggregate.sup_jerk, local.sup_jerk);
        aggregate.sup_horizontal_jerk = std::max(
            aggregate.sup_horizontal_jerk, local.sup_horizontal_jerk);
    }
    if (!aggregate.valid || !std::isfinite(aggregate.inf_speed) ||
        !std::isfinite(aggregate.inf_horizontal_speed)) {
        return false;
    }
    return MakeCertificate(w0, w1, aggregate, certificate);
}

bool MakeMappedBsplineCertificate(
    const UniformBspline& p, const UniformBspline& dp_dt,
    const UniformBspline& d2p_dt2, const UniformBspline& d3p_dt3,
    const ArcLengthMap& map, const double spline_t_anchor,
    const double spline_t_end, const double phase_w_anchor,
    const double phase_w_end, const double arclength_per_phase,
    const double w0, const double w1,
    phase_offset_core::PathCellGeometryCertificate& certificate)
{
    (void)p;
    if (!std::isfinite(w0) || !std::isfinite(w1) ||
        w0 < phase_w_anchor - kDomainEps || w1 > phase_w_end + kDomainEps ||
        w1 <= w0 + kDomainEps || !std::isfinite(arclength_per_phase) ||
        arclength_per_phase <= 0.0 || map.cells.empty() ||
        map.table_s.size() != map.cells.size() + 1U ||
        map.table_t.size() != map.table_s.size()) {
        return false;
    }
    const double total_length = map.table_s.back();
    const double target_s0 = std::max(0.0, std::min(total_length,
        arclength_per_phase * (w0 - phase_w_anchor)));
    const double target_s1 = std::max(0.0, std::min(total_length,
        arclength_per_phase * (w1 - phase_w_anchor)));
    if (!std::isfinite(total_length) || !std::isfinite(target_s0) ||
        !std::isfinite(target_s1) || target_s1 < target_s0) {
        return false;
    }
    double enclosed_t0 = std::numeric_limits<double>::infinity();
    double enclosed_t1 = -std::numeric_limits<double>::infinity();
    double q_min = std::numeric_limits<double>::infinity();
    double q_max = 0.0;
    double q_prime_max = 0.0;
    double q_second_max = 0.0;
    bool has_cell = false;
    // Select exactly the arc-length cells intersected by [target_s0,target_s1].
    // `upper_bound(...)-1` gives the containing cell for an interior query and
    // the cell immediately preceding an exact table knot.  The previous code
    // used a lower_bound for the upper endpoint and then added neighbours on
    // both sides; that admitted unrelated cells (and their conservative
    // derivative controls) into the certificate, so one locally valid cell
    // could fail because a neighbouring map cell had a non-positive Bernstein
    // speed control.  Exact-boundary cells are still included through the
    // preceding-cell convention without weakening the interval proof.
    const auto first_table = std::upper_bound(
        map.table_s.begin(), map.table_s.end(), target_s0);
    const auto last_table = std::upper_bound(
        map.table_s.begin(), map.table_s.end(), target_s1);
    std::size_t begin_index = first_table == map.table_s.begin()
        ? 0U : static_cast<std::size_t>(
            std::distance(map.table_s.begin(), first_table) - 1);
    std::size_t end_index = last_table == map.table_s.begin()
        ? 0U : static_cast<std::size_t>(
            std::distance(map.table_s.begin(), last_table) - 1);
    begin_index = std::min(begin_index, map.cells.size() - 1U);
    end_index = std::min(end_index, map.cells.size() - 1U);
    for (std::size_t index = begin_index; index <= end_index; ++index) {
        const double s0 = map.table_s[index];
        const double s1 = map.table_s[index + 1U];
        if (!std::isfinite(s0) || !std::isfinite(s1) || s1 < target_s0 ||
            s0 > target_s1) {
            continue;
        }
        const ArcLengthCell& cell = map.cells[index];
        if (!std::isfinite(cell.h) || cell.h <= 0.0) return false;
        const std::vector<double> map_power(cell.a.begin(), cell.a.end());
        const std::vector<double> first = DifferentiateScalarPower(map_power);
        const std::vector<double> second = DifferentiateScalarPower(first);
        const std::vector<double> third = DifferentiateScalarPower(second);
        const std::vector<double> q_controls = ScalarPowerToBernstein(first);
        const std::vector<double> q_prime_controls =
            ScalarPowerToBernstein(second);
        const std::vector<double> q_second_controls =
            ScalarPowerToBernstein(third);
        if (q_controls.empty() || q_prime_controls.empty() ||
            q_second_controls.empty()) {
            return false;
        }
        for (const double coefficient : q_controls) {
            if (!std::isfinite(coefficient) || coefficient <= 0.0) return false;
            q_min = std::min(q_min, coefficient / cell.h);
            q_max = std::max(q_max, coefficient / cell.h);
        }
        for (const double coefficient : q_prime_controls) {
            if (!std::isfinite(coefficient)) return false;
            q_prime_max = std::max(q_prime_max,
                std::abs(coefficient) / (cell.h * cell.h));
        }
        for (const double coefficient : q_second_controls) {
            if (!std::isfinite(coefficient)) return false;
            q_second_max = std::max(q_second_max,
                std::abs(coefficient) / (cell.h * cell.h * cell.h));
        }
        enclosed_t0 = std::min(enclosed_t0, map.table_t[index]);
        enclosed_t1 = std::max(enclosed_t1, map.table_t[index + 1U]);
        has_cell = true;
    }
    if (!has_cell || !std::isfinite(enclosed_t0) ||
        !std::isfinite(enclosed_t1) || enclosed_t0 < spline_t_anchor - kDomainEps ||
        enclosed_t1 > spline_t_end + kDomainEps) {
        return false;
    }
    q_min = LowerBound(q_min);
    q_max = UpperBound(q_max);
    q_prime_max = UpperBound(q_prime_max);
    q_second_max = UpperBound(q_second_max);
    if (!std::isfinite(q_min) || !std::isfinite(q_max) ||
        !std::isfinite(q_prime_max) || !std::isfinite(q_second_max) ||
        q_min <= kCertificateSpeedEps || q_max < q_min) {
        return false;
    }
    VectorBounds p_t;
    VectorBounds p_t_h;
    VectorBounds p_tt;
    VectorBounds p_tt_h;
    VectorBounds p_ttt;
    VectorBounds p_ttt_h;
    if (!BoundsFromSpline(dp_dt, enclosed_t0, enclosed_t1, p_t, p_t_h) ||
        !BoundsFromSpline(d2p_dt2, enclosed_t0, enclosed_t1, p_tt, p_tt_h) ||
        !BoundsFromSpline(d3p_dt3, enclosed_t0, enclosed_t1, p_ttt, p_ttt_h)) {
        return false;
    }
    const double dt_dw_max = UpperBound(arclength_per_phase / q_min);
    const double dt_dw_min = LowerBound(arclength_per_phase / q_max);
    const double d2t_dw2_max = UpperBound(
        arclength_per_phase * arclength_per_phase * q_prime_max /
        (q_min * q_min * q_min));
    const double d3t_dw3_max = UpperBound(
        arclength_per_phase * arclength_per_phase * arclength_per_phase *
        (3.0 * q_prime_max * q_prime_max /
             (q_min * q_min * q_min * q_min * q_min) +
         q_second_max / (q_min * q_min * q_min * q_min)));
    if (!std::isfinite(dt_dw_max) || !std::isfinite(dt_dw_min) ||
        !std::isfinite(d2t_dw2_max) || !std::isfinite(d3t_dw3_max)) {
        return false;
    }
    DifferentialBounds differential;
    differential.inf_speed = LowerBound(p_t.inf_norm * dt_dw_min);
    differential.inf_horizontal_speed = LowerBound(p_t_h.inf_norm * dt_dw_min);
    differential.sup_speed = UpperBound(p_t.sup_norm * dt_dw_max);
    differential.sup_acceleration = UpperBound(
        p_tt.sup_norm * dt_dw_max * dt_dw_max +
        p_t.sup_norm * d2t_dw2_max);
    differential.sup_horizontal_acceleration = UpperBound(
        p_tt_h.sup_norm * dt_dw_max * dt_dw_max +
        p_t_h.sup_norm * d2t_dw2_max);
    differential.sup_jerk = UpperBound(
        p_ttt.sup_norm * dt_dw_max * dt_dw_max * dt_dw_max +
        3.0 * p_tt.sup_norm * dt_dw_max * d2t_dw2_max +
        p_t.sup_norm * d3t_dw3_max);
    differential.sup_horizontal_jerk = UpperBound(
        p_ttt_h.sup_norm * dt_dw_max * dt_dw_max * dt_dw_max +
        3.0 * p_tt_h.sup_norm * dt_dw_max * d2t_dw2_max +
        p_t_h.sup_norm * d3t_dw3_max);
    differential.valid = std::isfinite(differential.inf_speed) &&
        std::isfinite(differential.inf_horizontal_speed) &&
        std::isfinite(differential.sup_speed) &&
        std::isfinite(differential.sup_acceleration) &&
        std::isfinite(differential.sup_horizontal_acceleration) &&
        std::isfinite(differential.sup_jerk) &&
        std::isfinite(differential.sup_horizontal_jerk);
    return MakeCertificate(w0, w1, differential, certificate);
}

// The executable arclength map is preferred because it removes the planner's
// time-parameterization from phase speed.  Some valid planner splines still
// cannot produce that map (for example a stationary raw endpoint or a
// numerically non-positive speed at one table sample).  The point evaluator
// deliberately retains the historical linear t(w) fallback in that case.
// Keep the fallback generic and certifiable on cells whose actual mapped
// B-spline derivative has a positive lower bound; otherwise it remains
// fail-closed and the caller keeps the historical inset.
bool MakeLinearMappedBsplineCertificate(
    const UniformBspline& dp_dt, const UniformBspline& d2p_dt2,
    const UniformBspline& d3p_dt3, const double spline_t_anchor,
    const double spline_t_end, const double phase_w_anchor,
    const double phase_w_end, const double w0, const double w1,
    phase_offset_core::PathCellGeometryCertificate& certificate)
{
    certificate = phase_offset_core::PathCellGeometryCertificate();
    if (!std::isfinite(spline_t_anchor) || !std::isfinite(spline_t_end) ||
        !std::isfinite(phase_w_anchor) || !std::isfinite(phase_w_end) ||
        !std::isfinite(w0) || !std::isfinite(w1) ||
        spline_t_end <= spline_t_anchor + kDomainEps ||
        phase_w_end <= phase_w_anchor + kDomainEps ||
        w0 < phase_w_anchor - kDomainEps ||
        w1 > phase_w_end + kDomainEps ||
        w1 <= w0 + kDomainEps) {
        return false;
    }
    const double dt_dw = (spline_t_end - spline_t_anchor) /
        (phase_w_end - phase_w_anchor);
    const double t0 = spline_t_anchor + dt_dw * (w0 - phase_w_anchor);
    const double t1 = spline_t_anchor + dt_dw * (w1 - phase_w_anchor);
    if (!std::isfinite(dt_dw) || dt_dw <= 0.0 ||
        !std::isfinite(t0) || !std::isfinite(t1) ||
        t1 <= t0 + kDomainEps) {
        return false;
    }
    VectorBounds p_t;
    VectorBounds p_t_h;
    VectorBounds p_tt;
    VectorBounds p_tt_h;
    VectorBounds p_ttt;
    VectorBounds p_ttt_h;
    if (!BoundsFromSpline(dp_dt, t0, t1, p_t, p_t_h) ||
        !BoundsFromSpline(d2p_dt2, t0, t1, p_tt, p_tt_h) ||
        !BoundsFromSpline(d3p_dt3, t0, t1, p_ttt, p_ttt_h)) {
        return false;
    }
    DifferentialBounds differential;
    differential.inf_speed = LowerBound(p_t.inf_norm * dt_dw);
    differential.inf_horizontal_speed = LowerBound(p_t_h.inf_norm * dt_dw);
    differential.sup_speed = UpperBound(p_t.sup_norm * dt_dw);
    differential.sup_acceleration = UpperBound(
        p_tt.sup_norm * dt_dw * dt_dw);
    differential.sup_horizontal_acceleration = UpperBound(
        p_tt_h.sup_norm * dt_dw * dt_dw);
    differential.sup_jerk = UpperBound(
        p_ttt.sup_norm * dt_dw * dt_dw * dt_dw);
    differential.sup_horizontal_jerk = UpperBound(
        p_ttt_h.sup_norm * dt_dw * dt_dw * dt_dw);
    differential.valid = std::isfinite(differential.inf_speed) &&
        std::isfinite(differential.inf_horizontal_speed) &&
        std::isfinite(differential.sup_speed) &&
        std::isfinite(differential.sup_acceleration) &&
        std::isfinite(differential.sup_horizontal_acceleration) &&
        std::isfinite(differential.sup_jerk) &&
        std::isfinite(differential.sup_horizontal_jerk);
    return MakeCertificate(w0, w1, differential, certificate);
}

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

bool buildArcLengthTable(const UniformBspline& position,
                         double t_start,
                         double t_end,
                         bool require_positive_speed,
                         std::vector<double>& table_t,
                         std::vector<double>& table_s,
                         std::vector<double>* table_speed)
{
    table_t.assign(kArcLengthTableIntervals + 1, 0.0);
    table_s.assign(kArcLengthTableIntervals + 1, 0.0);
    if (table_speed) table_speed->assign(kArcLengthTableIntervals + 1, 0.0);
    if (!std::isfinite(t_start) || !std::isfinite(t_end) ||
        t_end <= t_start + kDomainEps) {
        return false;
    }
    const UniformBspline velocity_spline = position.getDerivative();
    const double dt = (t_end - t_start) / kArcLengthTableIntervals;
    for (int i = 0; i <= kArcLengthTableIntervals; ++i) {
        const double t = (i == kArcLengthTableIntervals)
            ? t_end
            : t_start + dt * static_cast<double>(i);
        const Eigen::Vector3d velocity = evaluateSplineAtTime(velocity_spline, t);
        const double speed = velocity.norm();
        if (!velocity.allFinite() || !std::isfinite(speed) ||
            (require_positive_speed && speed <= kArcLengthSpeedEps)) {
            return false;
        }
        table_t[i] = t;
        if (table_speed) (*table_speed)[i] = speed;
    }
    for (int i = 1; i <= kArcLengthTableIntervals; ++i) {
        const double midpoint_t = 0.5 * (table_t[i - 1] + table_t[i]);
        const double midpoint_speed =
            evaluateSplineAtTime(velocity_spline, midpoint_t).norm();
        if (!std::isfinite(midpoint_speed)) return false;
        table_s[i] = table_s[i - 1] +
            (dt / 6.0) * ((table_speed ? (*table_speed)[i - 1]
                                       : evaluateSplineAtTime(
                                             velocity_spline, table_t[i - 1]).norm()) +
                          4.0 * midpoint_speed +
                          (table_speed ? (*table_speed)[i]
                                       : evaluateSplineAtTime(
                                             velocity_spline, table_t[i]).norm()));
    }
    return std::isfinite(table_s.back()) && table_s.back() > kDomainEps;
}

bool buildExecutableArcLengthMap(const UniformBspline& position,
                                 double t_start,
                                 double t_end,
                                 ArcLengthMap& map)
{
    map = ArcLengthMap();
    if (!buildArcLengthTable(position, t_start, t_end, true,
                             map.table_t, map.table_s, &map.table_speed)) {
        return false;
    }

    const UniformBspline velocity_spline = position.getDerivative();
    const UniformBspline acceleration_spline = velocity_spline.getDerivative();
    map.table_speed_derivative.assign(kArcLengthTableIntervals + 1, 0.0);
    for (int i = 0; i <= kArcLengthTableIntervals; ++i) {
        const Eigen::Vector3d velocity =
            evaluateSplineAtTime(velocity_spline, map.table_t[i]);
        const Eigen::Vector3d acceleration =
            evaluateSplineAtTime(acceleration_spline, map.table_t[i]);
        const double speed = map.table_speed[i];
        if (!velocity.allFinite() || !acceleration.allFinite() ||
            !std::isfinite(speed) || speed <= kArcLengthSpeedEps) {
            return false;
        }
        map.table_speed_derivative[i] = velocity.dot(acceleration) / speed;
        if (!std::isfinite(map.table_speed_derivative[i])) return false;
    }

    map.cells.resize(kArcLengthTableIntervals);
    for (int i = 0; i < kArcLengthTableIntervals; ++i) {
        ArcLengthCell& cell = map.cells[i];
        cell.t0 = map.table_t[i];
        cell.h = map.table_t[i + 1] - map.table_t[i];
        const double h = cell.h;
        if (!std::isfinite(h) || h <= 0.0) return false;
        const double y0 = map.table_s[i];
        const double y1 = map.table_s[i + 1];
        const double d0 = map.table_speed[i];
        const double d1 = map.table_speed[i + 1];
        const double dd0 = map.table_speed_derivative[i];
        const double dd1 = map.table_speed_derivative[i + 1];
        cell.a[0] = y0;
        cell.a[1] = h * d0;
        cell.a[2] = 0.5 * h * h * dd0;
        const double r0 = y1 - cell.a[0] - cell.a[1] - cell.a[2];
        const double r1 = h * d1 - cell.a[1] - 2.0 * cell.a[2];
        const double r2 = h * h * dd1 - 2.0 * cell.a[2];
        cell.a[3] = 10.0 * r0 - 4.0 * r1 + 0.5 * r2;
        cell.a[4] = -15.0 * r0 + 7.0 * r1 - r2;
        cell.a[5] = 6.0 * r0 - 3.0 * r1 + 0.5 * r2;

        // A positive Bernstein control polygon is a sufficient monotonicity
        // proof for ds/dx across the complete cell.
        const double q0 = cell.a[1];
        const double q1 = 2.0 * cell.a[2];
        const double q2 = 3.0 * cell.a[3];
        const double q3 = 4.0 * cell.a[4];
        const double q4 = 5.0 * cell.a[5];
        const std::array<double, 5> bernstein = {{
            q0,
            q0 + 0.25 * q1,
            q0 + 0.5 * q1 + q2 / 6.0,
            q0 + 0.75 * q1 + 0.5 * q2 + 0.25 * q3,
            q0 + q1 + q2 + q3 + q4}};
        for (const double coefficient : bernstein) {
            if (!std::isfinite(coefficient) ||
                coefficient <= cell.h * kArcLengthSpeedEps) {
                return false;
            }
        }
    }
    return std::isfinite(map.table_s.back()) &&
           map.table_s.back() > kDomainEps;
}

double refinedArcLengthOnInterval(const UniformBspline& velocity_spline,
                                  const double t0,
                                  const double t1)
{
    if (!std::isfinite(t0) || !std::isfinite(t1) || t1 <= t0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double midpoint = 0.5 * (t0 + t1);
    const Eigen::Vector3d v0 = evaluateSplineAtTime(velocity_spline, t0);
    const Eigen::Vector3d vm = evaluateSplineAtTime(velocity_spline, midpoint);
    const Eigen::Vector3d v1 = evaluateSplineAtTime(velocity_spline, t1);
    if (!v0.allFinite() || !vm.allFinite() || !v1.allFinite()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return (t1 - t0) * (v0.norm() + 4.0 * vm.norm() + v1.norm()) / 6.0;
}

bool invertRawArcLengthTable(const UniformBspline& position,
                             const std::vector<double>& table_t,
                             const std::vector<double>& table_s,
                             const double target_s,
                             double& t)
{
    t = 0.0;
    if (table_t.size() != kArcLengthTableIntervals + 1U ||
        table_s.size() != table_t.size() || !std::isfinite(target_s)) {
        return false;
    }
    const double total_length = table_s.back();
    const double bounded_s = std::max(0.0, std::min(target_s, total_length));
    const auto upper = std::lower_bound(
        table_s.begin(), table_s.end(), bounded_s);
    if (upper == table_s.begin()) {
        t = table_t.front();
        return true;
    }
    if (upper == table_s.end()) {
        t = table_t.back();
        return true;
    }
    const size_t hi = static_cast<size_t>(std::distance(table_s.begin(), upper));
    const size_t lo = hi - 1U;
    const double s0 = table_s[lo];
    const double s1 = table_s[hi];
    if (!std::isfinite(s0) || !std::isfinite(s1) || s1 <= s0) return false;

    const UniformBspline velocity_spline = position.getDerivative();
    double left_t = table_t[lo];
    double right_t = table_t[hi];
    double left_s = s0;
    for (int iteration = 0; iteration < 48; ++iteration) {
        const double middle_t = 0.5 * (left_t + right_t);
        const double partial = refinedArcLengthOnInterval(
            velocity_spline, table_t[lo], middle_t);
        if (!std::isfinite(partial)) return false;
        const double middle_s = s0 + partial;
        if (middle_s < bounded_s) {
            left_t = middle_t;
            left_s = middle_s;
        } else {
            right_t = middle_t;
        }
        if (std::abs(middle_s - bounded_s) <=
            1e-13 * std::max(1.0, total_length)) {
            t = middle_t;
            return true;
        }
    }
    (void)left_s;
    t = 0.5 * (left_t + right_t);
    return std::isfinite(t);
}

void evaluateArcLengthCell(const ArcLengthCell& cell,
                           const double x,
                           double& value,
                           double& first,
                           double& second)
{
    const double x2 = x * x;
    const double x3 = x2 * x;
    const double x4 = x3 * x;
    const double x5 = x4 * x;
    value = cell.a[0] + cell.a[1] * x + cell.a[2] * x2 +
            cell.a[3] * x3 + cell.a[4] * x4 + cell.a[5] * x5;
    first = (cell.a[1] + 2.0 * cell.a[2] * x +
             3.0 * cell.a[3] * x2 + 4.0 * cell.a[4] * x3 +
             5.0 * cell.a[5] * x4) / cell.h;
    second = (2.0 * cell.a[2] + 6.0 * cell.a[3] * x +
              12.0 * cell.a[4] * x2 + 20.0 * cell.a[5] * x3) /
             (cell.h * cell.h);
}

bool invertArcLengthMap(const ArcLengthMap& map,
                        double target_s,
                        double& t,
                        double& ds_dt,
                        double& d2s_dt2)
{
    if (map.table_t.size() != kArcLengthTableIntervals + 1U ||
        map.table_s.size() != map.table_t.size() ||
        map.table_speed.size() != map.table_t.size() ||
        map.table_speed_derivative.size() != map.table_t.size() ||
        map.cells.size() != kArcLengthTableIntervals ||
        !std::isfinite(target_s)) {
        return false;
    }
    const double total_length = map.table_s.back();
    const double bounded_s = std::max(0.0, std::min(target_s, total_length));
    const auto upper = std::lower_bound(
        map.table_s.begin(), map.table_s.end(), bounded_s);
    if (upper == map.table_s.begin()) {
        t = map.table_t.front();
        ds_dt = map.table_speed.front();
        d2s_dt2 = map.table_speed_derivative.front();
        return true;
    }
    if (upper == map.table_s.end()) {
        t = map.table_t.back();
        ds_dt = map.table_speed.back();
        d2s_dt2 = map.table_speed_derivative.back();
        return true;
    }
    const size_t hi = static_cast<size_t>(
        std::distance(map.table_s.begin(), upper));
    const size_t lo = hi - 1U;
    const double s0 = map.table_s[lo];
    const double s1 = map.table_s[hi];
    const double ds = s1 - s0;
    const ArcLengthCell& cell = map.cells[lo];
    if (!std::isfinite(ds) || ds <= 0.0 || !std::isfinite(cell.h) ||
        cell.h <= 0.0) {
        return false;
    }
    double lower_x = 0.0;
    double upper_x = 1.0;
    double x = std::max(0.0, std::min(1.0, (bounded_s - s0) / ds));
    for (int iteration = 0; iteration < 24; ++iteration) {
        double value = 0.0;
        evaluateArcLengthCell(cell, x, value, ds_dt, d2s_dt2);
        const double residual = value - bounded_s;
        if (std::abs(residual) <= 1e-13 * std::max(1.0, total_length)) {
            break;
        }
        if (residual < 0.0) lower_x = x;
        else upper_x = x;
        double next_x = 0.5 * (lower_x + upper_x);
        if (std::isfinite(ds_dt) && ds_dt > kArcLengthSpeedEps) {
            const double newton = x - residual / (cell.h * ds_dt);
            if (newton > lower_x && newton < upper_x) next_x = newton;
        }
        x = next_x;
    }
    double value = 0.0;
    evaluateArcLengthCell(cell, x, value, ds_dt, d2s_dt2);
    t = cell.t0 + x * cell.h;
    return std::isfinite(t) && std::isfinite(ds_dt) &&
           ds_dt > kArcLengthSpeedEps && std::isfinite(d2s_dt2);
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
    segment.identity = next_segment_identity_++;
    if (segment.identity == 0U) return false;
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

bool ContinuousPhasePath::cellBounds(
    const double w0, const double w1,
    phase_offset_core::PathCellGeometryCertificate& certificate) const
{
    certificate = phase_offset_core::PathCellGeometryCertificate();
    if (segments_.empty() || !std::isfinite(w0) || !std::isfinite(w1) ||
        w1 <= w0 + kDomainEps) {
        return false;
    }
    const Segment* selected = nullptr;
    for (const Segment& segment : segments_) {
        if (w0 >= segment.w0 - kDomainEps && w1 <= segment.w1 + kDomainEps) {
            selected = &segment;
            break;
        }
    }
    if (selected == nullptr || !selected->evaluate.hasCellBounds() ||
        !selected->evaluate.cellBounds(w0, w1, certificate)) {
        certificate = phase_offset_core::PathCellGeometryCertificate();
        return false;
    }
    certificate.w0 = w0;
    certificate.w1 = w1;
    certificate.path_revision = path_revision_;
    certificate.segment_identity = selected->identity;
    certificate.segment_w0 = selected->w0;
    certificate.segment_w1 = selected->w1;
    if (certificate.path_revision == 0U) {
        certificate.path_revision = path_revision_;
    }
    if (!phase_offset_core::pathCellGeometryCertificateIsComplete(certificate)) {
        certificate = phase_offset_core::PathCellGeometryCertificate();
        return false;
    }
    return true;
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
    state.path_revision = path_revision_;
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

bool ContinuousPhasePath::measureBsplineArcLength(
    const UniformBspline& position,
    double spline_t_start,
    double spline_t_end,
    double& total_length)
{
    total_length = 0.0;
    std::vector<double> table_t;
    std::vector<double> table_s;
    if (!buildArcLengthTable(position, spline_t_start, spline_t_end, false,
                             table_t, table_s, nullptr)) {
        return false;
    }
    total_length = table_s.back();
    return std::isfinite(total_length) && total_length > kDomainEps;
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

    const auto point_evaluator = [=](double w, ContinuousPhasePathState& state) {
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
    const std::array<Eigen::Vector3d, 6> coefficients = {{
        a0, a1, a2, a3, a4, a5}};
    const CellBoundEvaluator cell_bound_evaluator =
        [coefficients, w0, w1](const double cell_w0, const double cell_w1,
                                phase_offset_core::PathCellGeometryCertificate& certificate) {
          return MakeQuinticCertificate(coefficients, w0, w1, cell_w0,
                                        cell_w1, certificate);
        };
    return Evaluator(point_evaluator, cell_bound_evaluator);
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
    const double phase_span = phase_w_end - phase_w_anchor;

    CellBoundEvaluator linear_cell_bound_evaluator;
    if (p.p_ >= 3) {
        const UniformBspline d3p_dt3 = d2p_dt2.getDerivative();
        linear_cell_bound_evaluator =
            [dp_dt, d2p_dt2, d3p_dt3, spline_t_anchor, spline_t_end,
             phase_w_anchor, phase_w_end](
                const double cell_w0, const double cell_w1,
                phase_offset_core::PathCellGeometryCertificate& certificate) {
              return MakeLinearMappedBsplineCertificate(
                  dp_dt, d2p_dt2, d3p_dt3, spline_t_anchor, spline_t_end,
                  phase_w_anchor, phase_w_end, cell_w0, cell_w1,
                  certificate);
            };
    }

    // A certificate/map proof is optional metadata.  Preserve a finite
    // legacy point evaluator even when the strict executable Hermite map
    // cannot be built; navigation will then receive no cell certificate and
    // retain the historical fixed inset.  This fallback intentionally uses
    // the original linear t(w) contract and does not claim arclength bounds.
    const auto linear_point_evaluator = [=](double w,
                                            ContinuousPhasePathState& state) {
        const double bounded_w = std::max(
            phase_w_anchor, std::min(w, phase_w_end));
        const double dt_dw = (spline_t_end - spline_t_anchor) / phase_span;
        const double t = spline_t_anchor +
            dt_dw * (bounded_w - phase_w_anchor);
        state.p = evaluateSplineAtTime(p, t);
        state.vel = evaluateSplineAtTime(dp_dt, t);
        const Eigen::Vector3d acceleration = evaluateSplineAtTime(d2p_dt2, t);
        state.dp_dw = state.vel * dt_dw;
        state.d2p_dw2 = acceleration * dt_dw * dt_dw;
        state.valid = finiteState(state);
        return state.valid;
    };
    // This producer certifies the *implemented* monotone quintic-Hermite
    // map S(t) below and its inverse.  It intentionally does not claim that
    // the Simpson table is a validated exact integral of ||p_t||.

    // The planner B-spline is time-parameterized and may have stationary raw
    // endpoints.  The caller must first select a strictly executable time
    // interval.  Inside that interval, build a deterministic normalized-
    // arclength map so phase speed cannot inherit a small |p_t| from the time
    // parameterization.
    ArcLengthMap executable_map;
    if (!buildExecutableArcLengthMap(
            p, spline_t_anchor, spline_t_end, executable_map)) {
        return Evaluator(linear_point_evaluator, linear_cell_bound_evaluator);
    }
    const double total_length = executable_map.table_s.back();
    if (!std::isfinite(total_length) || total_length <= kDomainEps) {
        return Evaluator(linear_point_evaluator, linear_cell_bound_evaluator);
    }
    const double arclength_per_phase = total_length / phase_span;

    const auto point_evaluator = [=](double w, ContinuousPhasePathState& state) {
        const double bounded_w = std::max(
            phase_w_anchor, std::min(w, phase_w_end));
        const double target_s = std::max(
            0.0, std::min(total_length,
                arclength_per_phase * (bounded_w - phase_w_anchor)));
        double t = spline_t_anchor;
        double ds_dt = 0.0;
        double d2s_dt2 = 0.0;
        if (!invertArcLengthMap(executable_map, target_s, t, ds_dt,
                                d2s_dt2)) {
            return false;
        }
        state.p = evaluateSplineAtTime(p, t);
        state.vel = evaluateSplineAtTime(dp_dt, t);
        const Eigen::Vector3d acceleration = evaluateSplineAtTime(d2p_dt2, t);
        const double speed = state.vel.norm();
        if (!state.vel.allFinite() || !acceleration.allFinite() ||
            !std::isfinite(speed) || speed <= kArcLengthSpeedEps ||
            !std::isfinite(ds_dt) || ds_dt <= kArcLengthSpeedEps ||
            !std::isfinite(d2s_dt2)) {
            return false;
        }
        const double dt_dw = arclength_per_phase / ds_dt;
        const double d2t_dw2 = -d2s_dt2 * arclength_per_phase *
            arclength_per_phase / (ds_dt * ds_dt * ds_dt);
        state.dp_dw = state.vel * dt_dw;
        state.d2p_dw2 = acceleration * dt_dw * dt_dw +
                        state.vel * d2t_dw2;
        state.valid = finiteState(state);
        return state.valid;
    };
    CellBoundEvaluator cell_bound_evaluator;
    if (p.p_ >= 3) {
        const UniformBspline d3p_dt3 = d2p_dt2.getDerivative();
        cell_bound_evaluator =
            [p, dp_dt, d2p_dt2, d3p_dt3, executable_map, spline_t_anchor,
             spline_t_end, phase_w_anchor, phase_w_end, arclength_per_phase](
                const double cell_w0, const double cell_w1,
                phase_offset_core::PathCellGeometryCertificate& certificate) {
              return MakeMappedBsplineCertificate(
                  p, dp_dt, d2p_dt2, d3p_dt3, executable_map,
                  spline_t_anchor, spline_t_end, phase_w_anchor,
                  phase_w_end, arclength_per_phase, cell_w0, cell_w1,
                  certificate);
            };
    }
    return Evaluator(point_evaluator, cell_bound_evaluator);
}

bool ContinuousPhasePath::trimBsplineTimeDomainByArcLength(
    const UniformBspline& position,
    double spline_t_start,
    double spline_t_end,
    double trim_start_length,
    double trim_end_length,
    double& executable_t_start,
    double& executable_t_end,
    double& total_length)
{
    executable_t_start = 0.0;
    executable_t_end = 0.0;
    total_length = 0.0;
    if (!std::isfinite(trim_start_length) || !std::isfinite(trim_end_length) ||
        trim_start_length < 0.0 || trim_end_length < 0.0) {
        return false;
    }
    std::vector<double> table_t;
    std::vector<double> table_s;
    // Raw endpoint trim accepts stationary planner endpoint constraints;
    // makeMappedBspline later builds the strict-positive executable map.
    if (!buildArcLengthTable(position, spline_t_start, spline_t_end, false,
                             table_t, table_s, nullptr)) {
        return false;
    }
    total_length = table_s.back();
    if (trim_start_length + trim_end_length >= total_length - kDomainEps) {
        return false;
    }
    if (!invertRawArcLengthTable(position, table_t, table_s, trim_start_length,
                                 executable_t_start) ||
        !invertRawArcLengthTable(position, table_t, table_s,
                                 total_length - trim_end_length,
                                 executable_t_end)) {
        return false;
    }
    return std::isfinite(executable_t_start) && std::isfinite(executable_t_end) &&
           executable_t_end > executable_t_start + kDomainEps;
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
