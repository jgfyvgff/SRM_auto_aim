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

// 已通过全部关联门限后才偏好上一帧 ID；原始分数和接受门限保持不变。
inline double association_score_for_ranking(
  bool gate_passed, double association_score, int candidate_id,
  int previous_id, double id_switch_score_margin)
{
  if (gate_passed && candidate_id != previous_id) {
    return association_score + id_switch_score_margin;
  }
  return association_score;
}

// 标准四装甲首次捕获时角速度尚未收敛，累计足够多的有效更新后再启用图像选板门。
inline bool association_image_model_gate_enabled(
  bool standard_four, int successful_capture_updates, int bootstrap_updates)
{
  return standard_four && successful_capture_updates >= bootstrap_updates;
}

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
