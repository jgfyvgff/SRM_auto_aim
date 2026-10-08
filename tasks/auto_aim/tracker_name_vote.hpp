#ifndef AUTO_AIM__TRACKER_NAME_VOTE_HPP
#define AUTO_AIM__TRACKER_NAME_VOTE_HPP

#include <deque>
#include <map>
#include <optional>

#include "tasks/auto_aim/armor.hpp"

namespace auto_aim
{
struct NameVote
{
    ArmorName name;
    int votes;
};

// 窗口内类别的多数票。抽出纯函数是为了脱离 Solver 与 EKF 单测投票规则。
//
// 为什么需要：远距离下模型会在相邻类别之间抖动（实测同一块装甲板在 two / outpost
// 之间逐帧跳）。追踪器在第 0 帧没有先验、走整帧缩放，一旦判错，之后正确的类别会被
// 名字过滤全部拦掉直到 temp_lost 超时（实测 30 帧里 29 帧卡住）。类别不该由单帧决定。
//
// 返回 nullopt 的三种情况都属于"证据不足，先不改判"：
// 窗口没填满、最高票不足 min_votes、最高票并列。
inline std::optional<NameVote> majority_armor_name(
    const std::deque<ArmorName> & recent, int window, int min_votes)
{
    if (window < 1 || min_votes < 1 || recent.size() < static_cast<std::size_t>(window)) {
        return std::nullopt;
    }

    std::map<ArmorName, int> votes;
    // 只统计最近 window 个样本：队列长度目前由调用方维持，但这里不依赖它，
    // 免得将来传入更长的历史时结论被旧样本带偏。
    const auto begin = recent.end() - static_cast<std::ptrdiff_t>(window);
    for (auto it = begin; it != recent.end(); ++it) {
        votes[*it]++;
    }

    auto best = votes.begin();
    int tied = 0;
    for (auto it = votes.begin(); it != votes.end(); ++it) {
        if (it->second > best->second) {
            best = it;
            tied = 1;
        } else if (it->second == best->second) {
            tied++;
        }
    }

    // 并列时视为证据不足：宁可持续等待，也不要在两个类别之间猜。
    if (tied > 1 || best->second < min_votes) {
        return std::nullopt;
    }
    return NameVote{best->first, best->second};
}
}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_NAME_VOTE_HPP
