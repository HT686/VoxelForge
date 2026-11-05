#pragma once
// Finds nearby points quickly. Keeps a reference to the points.
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>
#include "math.hpp"

class PointGrid {
public:
    PointGrid(const std::vector<V3>& pts, float cell) : pts_(pts), inv_(1/cell) {
        grid_.reserve(pts.size());
        for (uint32_t i = 0; i < pts.size(); i++) grid_[key(cellOf(pts[i].x), cellOf(pts[i].y), cellOf(pts[i].z))].push_back(i);
    }
    template <class F> void within(V3 q, float r, F&& f) const {
        const int rx = (int)std::ceil(r*inv_), x = cellOf(q.x), y = cellOf(q.y), z = cellOf(q.z); const float r2 = r*r;
        for (int k = z-rx; k <= z+rx; k++) for (int j = y-rx; j <= y+rx; j++) for (int i = x-rx; i <= x+rx; i++) {
            auto it = grid_.find(key(i, j, k)); if (it == grid_.end()) continue;
            for (uint32_t id : it->second) { const V3 v = pts_[id]-q; const float d2 = dot(v, v); if (d2 <= r2) f(id, d2); }
        }
    }
    long nearest(V3 q, float r, float* outD2 = nullptr) const {
        long best = -1; float bd = 1e30f; within(q, r, [&](uint32_t id, float d2) { if (d2 < bd) { bd = d2; best = id; } });
        if (outD2) *outD2 = bd;
        return best;
    }
private:
    int cellOf(float v) const { return (int)std::floor(v*inv_); }
    static uint64_t key(int x, int y, int z) { return ((uint64_t)(x+(1<<20)) << 42) | ((uint64_t)(y+(1<<20)) << 21) | (uint64_t)(z+(1<<20)); }
    const std::vector<V3>& pts_; float inv_;
    std::unordered_map<uint64_t, std::vector<uint32_t>> grid_;
};
