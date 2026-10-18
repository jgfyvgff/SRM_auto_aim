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

    // 标准四装甲只在捕获后的有限预热期关闭图像门，其他关联门限仍照常工作。
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
    return 0;
}
