#include <stdexcept>

#include "tasks/auto_aim/tracker_association_policy.hpp"

namespace
{
void require(bool condition, const char * message)
{
    if (!condition) throw std::runtime_error(message);
}
}  // namespace

int main()
{
    constexpr double switch_margin = 0.05;
    const auto ranking_score = [switch_margin](
        bool gate_passed, double raw_score, int id, int previous_id) {
        return auto_aim::association_score_for_ranking(
            gate_passed, raw_score, id, previous_id, switch_margin);
    };

    // 旧板仍通过门限时，微小分数优势不能导致来回切换。
    const auto_aim::AssociationRank previous{true, ranking_score(true, 0.12, 0, 0), 0.9};
    const auto_aim::AssociationRank near_switch{true, ranking_score(true, 0.10, 3, 0), 0.01};
    require(auto_aim::association_rank_precedes(previous, near_switch),
            "near-tie candidate replaced the previous armor ID");

    // 新板明显更好时仍可切换；未过门限的旧板不得靠连续性偏好胜出。
    const auto_aim::AssociationRank clear_switch{true, ranking_score(true, 0.06, 3, 0), 0.9};
    const auto_aim::AssociationRank rejected_previous{
        false, ranking_score(false, 0.01, 0, 0), 0.01};
    require(auto_aim::association_rank_precedes(clear_switch, previous),
            "clear score advantage did not switch armor ID");
    require(auto_aim::association_rank_precedes(clear_switch, rejected_previous),
            "rejected previous ID outranked a gated candidate");

    // 尚无旧 ID 时，各候选受到相同代价，仍按原始几何分数排序。
    const auto_aim::AssociationRank initial_a{true, ranking_score(true, 0.10, 0, -1), 0.1};
    const auto_aim::AssociationRank initial_b{true, ranking_score(true, 0.08, 3, -1), 0.1};
    require(auto_aim::association_rank_precedes(initial_b, initial_a),
            "initial association order changed");
    require(ranking_score(false, 0.01, 3, 0) == 0.01,
            "rejected diagnostic score was changed");

    // 记录旧图像模型门的反事实启用状态；当前板号由几何分配决定。
    require(!auto_aim::association_image_model_gate_enabled(true, 0, 4),
            "image model gate enabled before capture updates");
    require(!auto_aim::association_image_model_gate_enabled(true, 3, 4),
            "image model gate enabled before bootstrap completed");
    require(auto_aim::association_image_model_gate_enabled(true, 4, 4),
            "image model gate stayed disabled after bootstrap");
    require(auto_aim::association_image_model_gate_enabled(true, 0, 0),
            "default configuration must preserve immediate image gating");
    require(!auto_aim::association_image_model_gate_enabled(false, 4, 4),
            "non-standard geometry enabled the standard-four image gate");

    const std::array<auto_aim::AssociationPoint2d, 4> quad{{
        {0.0, 0.0}, {100.0, 0.0}, {100.0, 50.0}, {0.0, 50.0}}};
    const std::array<auto_aim::AssociationPoint2d, 4> shifted{{
        quad[2], quad[3], quad[0], quad[1]}};
    const auto shifted_error = auto_aim::association_quad_geometry_error(quad, shifted);
    require(shifted_error.valid && shifted_error.total_cost < 1e-9,
            "cyclic corner shift changed a matching quadrilateral cost");
    require(shifted_error.corner_rms_px < 1e-9,
            "diagnostic corner RMS did not use the best cyclic correspondence");
    require(shifted_error.point_permutation >= 0,
            "best corner permutation was not recorded");
    const std::array<auto_aim::AssociationPoint2d, 4> reversed{{
        quad[0], quad[3], quad[2], quad[1]}};
    const auto reversed_error = auto_aim::association_quad_geometry_error(quad, reversed);
    require(reversed_error.valid && reversed_error.total_cost < 1e-9 &&
            reversed_error.corner_rms_px < 1e-9,
            "reversed corner winding changed the quadrilateral comparison");

    require(shifted_error.min_corner_rms_px < 1e-9 &&
            reversed_error.min_corner_rms_px < 1e-9,
            "independent RMS did not preserve cyclic/reversed corner equivalence");

    // 旋转长方形的最小 RMS 对应可以不同于旧边角/边长代价的对应。
    std::array<auto_aim::AssociationPoint2d, 4> rotated{};
    const double rotation = std::acos(-1.0) / 3.0;
    for (std::size_t i = 0; i < quad.size(); ++i) {
        const double x = quad[i].x - 50.0, y = quad[i].y - 25.0;
        rotated[i] = {50.0 + std::cos(rotation) * x - std::sin(rotation) * y,
                      25.0 + std::sin(rotation) * x + std::cos(rotation) * y};
    }
    const auto rotated_error = auto_aim::association_quad_geometry_error(quad, rotated);
    require(rotated_error.valid &&
            rotated_error.min_corner_rms_px + 10.0 < rotated_error.corner_rms_px &&
            rotated_error.min_corner_point_permutation != rotated_error.point_permutation,
            "RMS ranking reused the legacy corner correspondence");
    require(std::abs(rotated_error.min_corner_rms_norm -
                     rotated_error.min_corner_rms_px / std::hypot(100.0, 50.0)) < 1e-12,
            "RMS normalization did not use the observed bounding-box diagonal");

    // 无效几何不能产生有效排序量；默认旧代价的既有断言仍必须通过。
    auto nonfinite_quad = quad;
    nonfinite_quad[0].x = std::numeric_limits<double>::quiet_NaN();
    require(!auto_aim::association_quad_geometry_error(nonfinite_quad, quad).valid,
            "nonfinite quadrilateral became eligible");
    const std::array<auto_aim::AssociationPoint2d, 4> collapsed{};
    require(!auto_aim::association_quad_geometry_error(collapsed, quad).valid,
            "zero-size quadrilateral became eligible");

    const std::vector<auto_aim::AssociationAssignmentEdge> assignment_edges{
        {0, 0, 0, true, 0.01, 0.5, 0.1},
        {1, 0, 1, true, 0.02, 0.5, 0.1},
        {2, 1, 0, true, 0.03, 0.5, 0.1}};
    const auto assignment = auto_aim::association_min_cost_assignment(
        assignment_edges, {0, 1});
    require(assignment.size() == 2 &&
            std::find(assignment.begin(), assignment.end(), 1) != assignment.end() &&
            std::find(assignment.begin(), assignment.end(), 2) != assignment.end(),
            "global assignment did not maximize feasible one-to-one matches");

    const std::vector<auto_aim::AssociationAssignmentEdge> geometry_edges{
        {3, 0, 0, true, 0.4, 0.5, 0.1},
        {4, 0, 0, true, 0.2, 0.5, 0.1}};
    const auto geometry_assignment = auto_aim::association_min_cost_assignment(
        geometry_edges, {0});
    require(geometry_assignment.size() == 1 && geometry_assignment[0] == 4,
            "assignment did not minimize the image geometry cost");

    const std::vector<auto_aim::AssociationAssignmentEdge> duplicate_edges{
        {5, 0, 0, true, 0.1, 0.5, 0.1},
        {6, 1, 0, true, 0.2, 0.5, 0.1}};
    const auto duplicate_assignment = auto_aim::association_min_cost_assignment(
        duplicate_edges, {0});
    require(duplicate_assignment.size() == 1 && duplicate_assignment[0] == 5,
            "assignment reused one model ID for duplicate detections");

    auto_aim::AssociationAssignmentEdge eligible_edge{
        7, 0, 0, true, 0.1, 0.5, 0.1, 40.0, true};
    require(auto_aim::association_assignment_edge_eligible(
                eligible_edge, 40.0, false),
            "corner RMS equal to the configured limit should remain eligible");
    require(auto_aim::association_assignment_edge_eligible(
                eligible_edge, 20.0, false) == false,
            "corner RMS above the configured limit was not rejected");
    auto invalid_geometry_edge = eligible_edge;
    invalid_geometry_edge.image_cost_valid = false;
    require(!auto_aim::association_assignment_edge_eligible(
                invalid_geometry_edge, 20.0, false),
            "missing image geometry must fail an active corner RMS cap");
    auto missing_corner_rms_edge = eligible_edge;
    missing_corner_rms_edge.image_corner_error_px = -1.0;
    require(!auto_aim::association_assignment_edge_eligible(
                missing_corner_rms_edge, 20.0, false),
            "missing corner RMS must not pass an active pixel limit");
    auto physical_infeasible_edge = eligible_edge;
    physical_infeasible_edge.physical_feasible = false;
    require(!auto_aim::association_assignment_edge_eligible(
                physical_infeasible_edge, 0.0, true),
            "physical-feasible comparison did not reject an infeasible edge");
    require(auto_aim::association_assignment_edge_eligible(
                physical_infeasible_edge, 0.0, false),
            "physical feasibility must stay diagnostic when its comparison is off");
    auto legacy_gate_failed_edge = eligible_edge;
    legacy_gate_failed_edge.gate_passed = false;
    require(!auto_aim::association_assignment_edge_eligible(
                legacy_gate_failed_edge, 0.0, false, true),
            "active combined-gate filter accepted a failed legacy gate");
    legacy_gate_failed_edge.gate_passed = true;
    require(auto_aim::association_assignment_edge_eligible(
                legacy_gate_failed_edge, 0.0, false, true),
            "combined-gate filter rejected a candidate passing all legacy gates");

    // 所有实验边都被过滤时，分配保持为空且不能走旧分数兜底，Tracker 才会只预测。
    const auto no_eligible_assignment = auto_aim::association_min_cost_assignment(
        {}, {0, 1});
    require(no_eligible_assignment.empty(),
            "fully filtered assignment unexpectedly selected an observation");
    require(!auto_aim::association_score_fallback_allowed(true),
            "active experiment filtering must disable score fallback");
    require(auto_aim::association_score_fallback_allowed(false),
            "baseline assignment must preserve legacy score fallback");
    return 0;
}
