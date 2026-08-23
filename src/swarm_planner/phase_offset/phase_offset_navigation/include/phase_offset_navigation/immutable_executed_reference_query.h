#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace phase_offset_navigation {

struct ExecutedReferenceQueryResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double w = 0.0;
  Eigen::Vector3d r = Eigen::Vector3d::Zero();
  Eigen::Vector3d r_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d r_ww = Eigen::Vector3d::Zero();
  bool r_ww_valid = false;
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  std::uint64_t owner_revision = 0U;
  std::uint64_t query_revision = 0U;
  bool valid = false;
  std::string provenance;
  std::string invalid_reason;
};

class ImmutableExecutedReferenceQuery {
 public:
  virtual ~ImmutableExecutedReferenceQuery() = default;
  virtual bool query(double w, ExecutedReferenceQueryResult& result) const = 0;
  virtual bool domain(double& w0, double& w1) const = 0;
  virtual std::uint64_t pathRevision() const = 0;
  virtual std::uint64_t frameRevision() const = 0;
  virtual std::uint64_t ownerRevision() const = 0;
  virtual std::uint64_t queryRevision() const = 0;
};

using ImmutableExecutedReferenceQueryPtr =
    std::shared_ptr<const ImmutableExecutedReferenceQuery>;

// Small immutable callback-backed implementation for adapters and tests.
class CallbackExecutedReferenceQuery final
    : public ImmutableExecutedReferenceQuery {
 public:
  using Query = std::function<bool(double, ExecutedReferenceQueryResult&)>;

  CallbackExecutedReferenceQuery(Query query, double w0, double w1,
                                 std::uint64_t path_revision,
                                 std::uint64_t frame_revision,
                                 std::uint64_t owner_revision,
                                 std::uint64_t query_revision,
                                 std::string provenance = std::string())
      : query_(std::move(query)), w0_(w0), w1_(w1),
        path_revision_(path_revision), frame_revision_(frame_revision),
        owner_revision_(owner_revision), query_revision_(query_revision),
        provenance_(std::move(provenance)) {}

  bool query(double w, ExecutedReferenceQueryResult& result) const override {
    result = ExecutedReferenceQueryResult();
    result.w = w;
    result.path_revision = path_revision_;
    result.frame_revision = frame_revision_;
    result.owner_revision = owner_revision_;
    result.query_revision = query_revision_;
    result.provenance = provenance_;
    if (!query_ || !std::isfinite(w) || w < w0_ || w > w1_) {
      result.invalid_reason = "executed reference query is outside domain";
      return false;
    }
    if (!query_(w, result)) return false;
    result.w = w;
    result.path_revision = path_revision_;
    result.frame_revision = frame_revision_;
    result.owner_revision = owner_revision_;
    result.query_revision = query_revision_;
    if (result.provenance.empty()) result.provenance = provenance_;
    return result.valid;
  }

  bool domain(double& w0, double& w1) const override {
    w0 = w0_;
    w1 = w1_;
    return std::isfinite(w0_) && std::isfinite(w1_) && w1_ >= w0_;
  }
  std::uint64_t pathRevision() const override { return path_revision_; }
  std::uint64_t frameRevision() const override { return frame_revision_; }
  std::uint64_t ownerRevision() const override { return owner_revision_; }
  std::uint64_t queryRevision() const override { return query_revision_; }

 private:
  Query query_;
  double w0_ = 0.0;
  double w1_ = 0.0;
  std::uint64_t path_revision_ = 0U;
  std::uint64_t frame_revision_ = 0U;
  std::uint64_t owner_revision_ = 0U;
  std::uint64_t query_revision_ = 0U;
  std::string provenance_;
};

}  // namespace phase_offset_navigation
