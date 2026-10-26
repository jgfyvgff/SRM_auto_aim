#ifndef AUTO_AIM__TRACKER_ASSOCIATION_POLICY_HPP
#define AUTO_AIM__TRACKER_ASSOCIATION_POLICY_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <vector>

namespace auto_aim
{
struct AssociationPoint2d
{
  double x;
  double y;
};

struct AssociationQuadGeometryError
{
  bool valid = false;
  double corner_rms_px = -1.0;
  double corner_rms_norm = -1.0;
  double center_error_norm = -1.0;
  double edge_angle_error_norm = -1.0;
  double side_length_log_ratio = -1.0;
  double total_cost = -1.0;
  int point_permutation = -1;
    // 独立最小化角点 RMS；不能沿用旧三项代价选出的角点对应。
    double min_corner_rms_px = -1.0;
    double min_corner_rms_norm = -1.0;
    int min_corner_point_permutation = -1;
};

// 只允许角点循环移位和反向绕序，保持四边形的邻接关系，避免下标不同造成误配。
inline AssociationQuadGeometryError association_quad_geometry_error(
  const std::array<AssociationPoint2d, 4> & observed,
  const std::array<AssociationPoint2d, 4> & predicted)
{
  AssociationQuadGeometryError best;
  double min_x = observed[0].x, max_x = observed[0].x;
  double min_y = observed[0].y, max_y = observed[0].y;
  AssociationPoint2d observed_center{0.0, 0.0}, predicted_center{0.0, 0.0};
  for (std::size_t i = 0; i < observed.size(); ++i) {
    const auto & a = observed[i];
    const auto & b = predicted[i];
    if (!std::isfinite(a.x) || !std::isfinite(a.y) ||
        !std::isfinite(b.x) || !std::isfinite(b.y)) return best;
    min_x = std::min(min_x, a.x); max_x = std::max(max_x, a.x);
    min_y = std::min(min_y, a.y); max_y = std::max(max_y, a.y);
    observed_center.x += a.x * 0.25; observed_center.y += a.y * 0.25;
    predicted_center.x += b.x * 0.25; predicted_center.y += b.y * 0.25;
  }
  const double diagonal = std::hypot(max_x - min_x, max_y - min_y);
  if (!std::isfinite(diagonal) || diagonal <= 1e-6) return best;
  const double center_error =
    std::hypot(observed_center.x - predicted_center.x,
               observed_center.y - predicted_center.y) / diagonal;
  constexpr double half_pi = 1.57079632679489661923;
  for (int reversed = 0; reversed < 2; ++reversed) {
    for (int shift = 0; shift < 4; ++shift) {
      double squared_corner_error = 0.0, angle_error = 0.0, side_error = 0.0;
      bool valid = true;
      for (int i = 0; i < 4; ++i) {
        const int next = (i + 1) % 4;
        const int pi = reversed ? (shift - i + 8) % 4 : (shift + i) % 4;
        const int pn = reversed ? (shift - next + 8) % 4 : (shift + next) % 4;
        const double cdx = observed[i].x - predicted[pi].x;
        const double cdy = observed[i].y - predicted[pi].y;
        squared_corner_error += cdx * cdx + cdy * cdy;
        const double odx = observed[next].x - observed[i].x;
        const double ody = observed[next].y - observed[i].y;
        const double pdx = predicted[pn].x - predicted[pi].x;
        const double pdy = predicted[pn].y - predicted[pi].y;
        const double ol = std::hypot(odx, ody), pl = std::hypot(pdx, pdy);
        if (ol <= 1e-6 || pl <= 1e-6) { valid = false; break; }
        angle_error += std::atan2(std::abs(odx * pdy - ody * pdx),
                                  std::abs(odx * pdx + ody * pdy)) / half_pi;
        side_error += std::abs(std::log(ol / pl));
      }
      if (!valid) continue;
      angle_error /= 4.0;
      side_error /= 4.0;
      const double cost = center_error + angle_error + side_error;
      const double corner_rms = std::sqrt(squared_corner_error / 4.0);
        if (std::isfinite(corner_rms) &&
            (best.min_corner_rms_px < 0.0 || corner_rms < best.min_corner_rms_px)) {
            best.min_corner_rms_px = corner_rms;
            best.min_corner_rms_norm = corner_rms / diagonal;
            best.min_corner_point_permutation = reversed * 4 + shift;
        }
      const bool same_cost = best.valid && std::abs(cost - best.total_cost) <= 1e-12;
      if (!std::isfinite(cost) || (best.valid && cost > best.total_cost + 1e-12) ||
          (same_cost && corner_rms >= best.corner_rms_px)) continue;
      best.valid = true;
      best.corner_rms_px = corner_rms;
      best.corner_rms_norm = best.corner_rms_px / diagonal;
      best.center_error_norm = center_error;
      best.edge_angle_error_norm = angle_error;
      best.side_length_log_ratio = side_error;
      best.total_cost = cost;
      best.point_permutation = reversed * 4 + shift;
    }
  }
  return best;
}

struct AssociationAssignmentEdge
{
  std::size_t candidate_index;
  int detection_index;
  int model_id;
  bool image_cost_valid;
  double image_cost;
  double mahalanobis_distance;
  double association_score;
  double image_corner_error_px = -1.0;
  bool physical_feasible = false;
  bool gate_passed = false;
};

// 离线实验只将指定的观测资格条件作为候选边过滤；0 表示不启用像素误差上限。
inline bool association_assignment_edge_eligible(
  const AssociationAssignmentEdge & edge, double max_corner_rms_px,
  bool require_physical_feasible, bool require_gate_passed = false)
{
  if (require_gate_passed && !edge.gate_passed) return false;
  if (require_physical_feasible && !edge.physical_feasible) return false;
  if (max_corner_rms_px > 0.0 &&
      (!edge.image_cost_valid || !std::isfinite(edge.image_corner_error_px) ||
       edge.image_corner_error_px < 0.0 ||
       edge.image_corner_error_px > max_corner_rms_px)) return false;
  return true;
}

// 实验过滤导致无匹配时必须保留“只预测”结果，不能用旧分数兜底重新接纳被拒观测。
inline bool association_score_fallback_allowed(bool experiment_filter_enabled)
{
  return !experiment_filter_enabled;
}

// 当前最多枚举三个可见板号；物理门只作诊断，先最大化匹配数，再最小化缺失图像量、图像代价、NIS 和复合分数。
inline std::vector<std::size_t> association_min_cost_assignment(
  const std::vector<AssociationAssignmentEdge> & edges, const std::vector<int> & model_ids)
{
  std::vector<std::size_t> best, current;
  std::vector<int> used_detections;
  int best_missing = std::numeric_limits<int>::max();
  double best_image = std::numeric_limits<double>::infinity();
  double best_nis = std::numeric_limits<double>::infinity();
  double best_score = std::numeric_limits<double>::infinity();
  const auto better = [&](int missing, double image, double nis, double score) {
    if (current.size() != best.size()) return current.size() > best.size();
    if (missing != best_missing) return missing < best_missing;
    if (image != best_image) return image < best_image;
    if (nis != best_nis) return nis < best_nis;
    return score < best_score;
  };
  std::function<void(std::size_t, int, double, double, double)> visit;
  visit = [&](std::size_t model_index, int missing, double image, double nis, double score) {
    if (model_index == model_ids.size()) {
      if (better(missing, image, nis, score)) {
        best = current; best_missing = missing; best_image = image;
        best_nis = nis; best_score = score;
      }
      return;
    }
    visit(model_index + 1, missing, image, nis, score);
    for (const auto & edge : edges) {
      if (edge.model_id != model_ids[model_index] ||
          std::find(used_detections.begin(), used_detections.end(), edge.detection_index) !=
            used_detections.end() ||
          !std::isfinite(edge.mahalanobis_distance) ||
          !std::isfinite(edge.association_score) ||
          (edge.image_cost_valid && !std::isfinite(edge.image_cost))) continue;
      current.push_back(edge.candidate_index);
      used_detections.push_back(edge.detection_index);
      visit(model_index + 1, missing + (edge.image_cost_valid ? 0 : 1),
            image + (edge.image_cost_valid ? edge.image_cost : 0.0),
            nis + edge.mahalanobis_distance, score + edge.association_score);
      used_detections.pop_back();
      current.pop_back();
    }
  };
  visit(0, 0, 0.0, 0.0, 0.0);
  return best;
}

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
