#ifndef PLAN_MANAGE_REPLAN_RETRY_H
#define PLAN_MANAGE_REPLAN_RETRY_H

#include <algorithm>
#include <cmath>

namespace scan_planner
{
// 连续失败 fails 次后，下一次重试前应等待的间隔（秒）：
//   fails<=0      -> 0（没有失败，不等待）
//   min<=0        -> 0（关闭退避）
//   其它          -> min * 2^(fails-1)，封顶 max（max<=0 表示不封顶）
// 例：min=0.2, max=1.0 -> 0.2, 0.4, 0.8, 1.0, 1.0 ...
inline double replanRetryInterval(int fails, double min_interval, double max_interval)
{
  if (fails <= 0 || min_interval <= 0.0)
    return 0.0;
  const int exp = std::min(fails - 1, 20);  // 防止指数溢出
  double interval = min_interval * std::pow(2.0, exp);
  if (max_interval > 0.0)
    interval = std::min(interval, max_interval);
  return interval;
}

// 本拍是否允许重试：elapsed 是距上次规划尝试的秒数。
inline bool replanRetryAllowed(bool enable, double elapsed, int fails,
                               double min_interval, double max_interval)
{
  if (!enable)
    return true;
  return elapsed >= replanRetryInterval(fails, min_interval, max_interval);
}
}  // namespace scan_planner

#endif  // PLAN_MANAGE_REPLAN_RETRY_H
