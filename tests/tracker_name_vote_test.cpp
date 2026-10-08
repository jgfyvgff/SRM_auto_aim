#include <deque>
#include <iostream>
#include <optional>
#include <string>

#include "tasks/auto_aim/tracker_name_vote.hpp"

namespace
{
using auto_aim::ArmorName;
using auto_aim::majority_armor_name;

bool check(bool condition, const std::string & label)
{
    if (condition) return true;
    std::cerr << label << ": 断言失败\n";
    return false;
}

bool expect_none(const std::optional<auto_aim::NameVote> & vote, const std::string & label)
{
    if (!vote.has_value()) return true;
    std::cerr << label << ": 期望不改判，实际得到 " << static_cast<int>(vote->name) << '\n';
    return false;
}
}  // namespace

int main()
{
    // 窗口没填满时不能改判：刚捕获就凭两三帧跳类别会造成反复重建目标。
    if (!expect_none(
            majority_armor_name({ArmorName::two, ArmorName::two}, 7, 5), "窗口未满不改判")) {
        return 1;
    }

    // 窗口满了但票数不足（3/7 < 5）同样不改判。
    {
        std::deque<ArmorName> names{
            ArmorName::two, ArmorName::two, ArmorName::two, ArmorName::outpost, ArmorName::sentry,
            ArmorName::one, ArmorName::three};
        if (!expect_none(majority_armor_name(names, 7, 5), "票数不足不改判")) return 1;
    }

    // 5/7 的明确多数：返回胜出类别与票数。
    {
        std::deque<ArmorName> names{
            ArmorName::two, ArmorName::outpost, ArmorName::two, ArmorName::two, ArmorName::two,
            ArmorName::two, ArmorName::sentry};
        const auto vote = majority_armor_name(names, 7, 5);
        if (!check(vote.has_value(), "多数票应有结果")) return 1;
        if (!check(vote->name == ArmorName::two, "胜出类别应为 two")) return 1;
        if (!check(vote->votes == 5, "票数应为 5")) return 1;
    }

    // 最高票并列（各 2 票、其它 1 票）时必须弃权：两个类别之间猜错会直接换掉几何模型。
    {
        std::deque<ArmorName> names{
            ArmorName::two, ArmorName::two, ArmorName::outpost, ArmorName::outpost, ArmorName::three,
            ArmorName::four, ArmorName::five};
        if (!expect_none(majority_armor_name(names, 7, 2), "并列时弃权")) return 1;
    }

    // 无论并列双方票数多高都弃权（5:5 这种更强的并列同样不能猜）。
    {
        std::deque<ArmorName> names{
            ArmorName::two, ArmorName::two, ArmorName::two, ArmorName::two, ArmorName::two,
            ArmorName::outpost, ArmorName::outpost, ArmorName::outpost, ArmorName::outpost,
            ArmorName::outpost};
        if (!expect_none(majority_armor_name(names, 10, 5), "5:5 并列弃权")) return 1;
    }

    // 只按最近 window 帧判断：更早的历史不能影响结论（队列由调用方维持长度，
    // 这里显式传入超出窗口的队列，确认实现会忽略窗口之外的样本）。
    {
        std::deque<ArmorName> names{
            ArmorName::outpost, ArmorName::outpost, ArmorName::outpost, ArmorName::outpost,
            ArmorName::outpost, ArmorName::two, ArmorName::two, ArmorName::two};
        const auto vote = majority_armor_name(names, 3, 3);
        if (!check(vote.has_value() && vote->name == ArmorName::two, "只看最近 window 帧")) return 1;
    }

    // 边界收敛：window / min_votes 小于 1 时视为未配置好，一律不改判。
    if (!expect_none(majority_armor_name({ArmorName::two}, 0, 1), "window=0 不改判")) return 1;
    if (!expect_none(majority_armor_name({ArmorName::two}, 1, 0), "votes=0 不改判")) return 1;

    std::cout << "tracker_name_vote_test passed\n";
    return 0;
}
