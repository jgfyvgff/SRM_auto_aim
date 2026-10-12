#ifndef AUTO_AIM__TRACKER_ASSOCIATION_POLICY_HPP
#define AUTO_AIM__TRACKER_ASSOCIATION_POLICY_HPP

#include <cmath>

namespace auto_aim
{
// 多个候选都通过安全门限时，优先选择综合几何残差较小者。
// Mahalanobis 距离仍负责统计门控，并在综合分数相同时打破并列。
struct AssociationRank
{
  bool gate_passed;
  double association_score;
  double mahalanobis_distance;
};

inline bool association_rank_precedes(
  const AssociationRank & a, const AssociationRank & b)
{
  if (a.gate_passed != b.gate_passed) return a.gate_passed;

  const bool a_score_valid = std::isfinite(a.association_score);
  const bool b_score_valid = std::isfinite(b.association_score);
  if (a_score_valid != b_score_valid) return a_score_valid;
  if (a_score_valid && a.association_score != b.association_score) {
    return a.association_score < b.association_score;
  }
  return a.mahalanobis_distance < b.mahalanobis_distance;
}
}  // namespace auto_aim

#endif
