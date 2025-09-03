#pragma once
#include <cmath>
#include <optional>
#include <utility>

namespace lm {

// 把世界坐标 (x,y) 映射到栅格索引 (ix,iy)；越界返回 std::nullopt
struct GridInfo {
  double res; int w, h; double ox, oy;
};

inline std::optional<std::pair<int,int>>
worldToCell(double x, double y, const GridInfo& g) {
    int ix = static_cast<int>(std::floor((x - g.ox) / g.res));
    int iy = static_cast<int>(std::floor((y - g.oy) / g.res));

    if (ix < 0 || ix >= g.w) return std::nullopt;
    if (iy < 0 || iy >= g.h) return std::nullopt;

    return std::make_pair(ix, iy);
}
// 把 (ix,iy) 变成 data 向量里的线性索引
inline int cellIndex(int ix, int iy, int w) {
  // TODO: return iy * w + ix;
  return iy * w + ix;
}

} // namespace lm
