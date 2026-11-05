#include "mesh.hpp"
#include "i18n.hpp"
#include <algorithm>
#include <functional>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
#include <numeric>
#include <queue>
#include <unordered_map>
#include <omp.h>

namespace {
inline uint64_t cellKey(int x, int y, int z) { return ((uint64_t)(x + (1<<20)) << 42) | ((uint64_t)(y + (1<<20)) << 21) | (uint64_t)(z + (1<<20)); }
struct Grid {
    const Tsdf& t; int minW; mutable Block* last = nullptr; mutable int lx = 1<<30, ly = 0, lz = 0;
    // Value at a lattice corner; false when unobserved.
    bool get(int x, int y, int z, float& v) const {
        int bx = x >> 3, by = y >> 3, bz = z >> 3;
        if (bx != lx || by != ly || bz != lz) { last = t.find(bx, by, bz); lx = bx; ly = by; lz = bz; }
        if (!last) return false;
        const Voxel& vx = last->v[(((z & 7)*8 + (y & 7))*8) + (x & 7)];
        if (vx.seen < minW) return false;
        v = vx.tsdf; return true;
    }
};
struct CellVert { uint64_t key; V3 pos; };
const int kEdges[12][2] = {{0,1},{2,3},{4,5},{6,7},{0,2},{1,3},{4,6},{5,7},{0,4},{1,5},{2,6},{3,7}};
}

void extractMesh(const Tsdf& t, const MeshOptions& opt, Mesh& out) {
    out = Mesh{};
    std::vector<const Block*> blocks; blocks.reserve(t.blockCount());
    for (const Block& b : t.blocks()) blocks.push_back(&b);
    const float vox = t.voxel();
    int tn = omp_get_max_threads();
    std::vector<std::vector<CellVert>> lv(tn);
    #pragma omp parallel
    {
        Grid g{t, opt.minWeight}; auto& o = lv[omp_get_thread_num()];
        #pragma omp for schedule(dynamic, 32)
        for (size_t bi = 0; bi < blocks.size(); bi++) {
            const Block& b = *blocks[bi];
            for (int k = 0; k < 8; k++) for (int j = 0; j < 8; j++) for (int i = 0; i < 8; i++) {
                int x = b.x*8 + i, y = b.y*8 + j, z = b.z*8 + k;
                float c[8]; bool ok = true; int inside = 0;
                for (int q = 0; q < 8 && ok; q++) { ok = g.get(x + (q&1), y + ((q>>1)&1), z + ((q>>2)&1), c[q]); if (ok && c[q] < 0) inside |= 1 << q; }
                if (!ok || inside == 0 || inside == 255) continue;
                V3 sum{}; int n = 0;
                for (auto& e : kEdges) {
                    float a = c[e[0]], d = c[e[1]];
                    if ((a < 0) == (d < 0)) continue;
                    float f = a/(a-d);
                    V3 pa{(float)(e[0]&1), (float)((e[0]>>1)&1), (float)((e[0]>>2)&1)}, pb{(float)(e[1]&1), (float)((e[1]>>1)&1), (float)((e[1]>>2)&1)};
                    sum = sum + pa + (pb-pa)*f; n++;
                }
                sum = sum*(1.f/n);
                o.push_back({cellKey(x, y, z), V3{(x+sum.x)*vox, (y+sum.y)*vox, (z+sum.z)*vox}});
            }
        }
    }
    size_t total = 0; for (auto& v : lv) total += v.size();
    std::unordered_map<uint64_t, uint32_t> id; id.reserve(total*2);
    out.vertices.reserve(total);
    std::vector<uint64_t> keys; keys.reserve(total);
    for (auto& v : lv) for (auto& c : v) { id.emplace(c.key, (uint32_t)out.vertices.size()); out.vertices.push_back(c.pos); keys.push_back(c.key); }

    // One quad per lattice edge crossing the surface; outside (positive) side faces outward.
    Grid g{t, opt.minWeight};
    auto cell = [&](int x, int y, int z, uint32_t& r) { auto it = id.find(cellKey(x, y, z)); if (it == id.end()) return false; r = it->second; return true; };
    auto quad = [&](uint32_t a, uint32_t b, uint32_t c, uint32_t d, bool flip) {
        if (flip) std::swap(b, d);
        out.indices.insert(out.indices.end(), {a, b, c, a, c, d});
    };
    for (uint64_t key : keys) {
        int x = (int)((key >> 42) & 0x1fffff) - (1<<20), y = (int)((key >> 21) & 0x1fffff) - (1<<20), z = (int)(key & 0x1fffff) - (1<<20);
        float c0, c1; uint32_t a, b, c, d;
        if (!g.get(x, y, z, c0)) continue;
        if (g.get(x+1, y, z, c1) && (c0 < 0) != (c1 < 0) && cell(x, y, z, a) && cell(x, y-1, z, b) && cell(x, y-1, z-1, c) && cell(x, y, z-1, d)) quad(a, b, c, d, c0 > 0);
        if (g.get(x, y+1, z, c1) && (c0 < 0) != (c1 < 0) && cell(x, y, z, a) && cell(x, y, z-1, b) && cell(x-1, y, z-1, c) && cell(x-1, y, z, d)) quad(a, b, c, d, c0 > 0);
        if (g.get(x, y, z+1, c1) && (c0 < 0) != (c1 < 0) && cell(x, y, z, a) && cell(x-1, y, z, b) && cell(x-1, y-1, z, c) && cell(x, y-1, z, d)) quad(a, b, c, d, c0 > 0);
    }

    const uint32_t nv = (uint32_t)out.vertices.size();
    if (opt.removeFragments && nv) {
        std::vector<uint32_t> parent(nv); std::iota(parent.begin(), parent.end(), 0);
        auto root = [&](uint32_t a) { while (parent[a] != a) { parent[a] = parent[parent[a]]; a = parent[a]; } return a; };
        for (size_t i = 0; i < out.indices.size(); i += 3) { uint32_t a = root(out.indices[i]); parent[root(out.indices[i+1])] = a; parent[root(out.indices[i+2])] = a; }
        std::vector<uint32_t> tris(nv, 0);
        for (size_t i = 0; i < out.indices.size(); i += 3) tris[root(out.indices[i])]++;
        const uint32_t minTris = (uint32_t)(*std::max_element(tris.begin(), tris.end())*opt.minFragmentShare);
        std::vector<uint32_t> remap(nv, 0xffffffff); std::vector<V3> kept; std::vector<uint32_t> ki;
        for (size_t i = 0; i < out.indices.size(); i += 3) {
            if (tris[root(out.indices[i])] < minTris) continue;
            for (int k = 0; k < 3; k++) { uint32_t v = out.indices[i+k]; if (remap[v] == 0xffffffff) { remap[v] = (uint32_t)kept.size(); kept.push_back(out.vertices[v]); } ki.push_back(remap[v]); }
        }
        out.vertices.swap(kept); out.indices.swap(ki);
    }

    if (opt.smoothIterations > 0 && !out.vertices.empty()) {
        const uint32_t n = (uint32_t)out.vertices.size();
        std::vector<uint32_t> off(n+1, 0), adj;
        for (size_t i = 0; i < out.indices.size(); i += 3) for (int k = 0; k < 3; k++) off[out.indices[i+k]+1] += 2;
        for (uint32_t i = 0; i < n; i++) off[i+1] += off[i];
        adj.resize(off[n]); std::vector<uint32_t> pos(off.begin(), off.end()-1);
        for (size_t i = 0; i < out.indices.size(); i += 3) for (int k = 0; k < 3; k++) {
            uint32_t a = out.indices[i+k]; adj[pos[a]++] = out.indices[i+(k+1)%3]; adj[pos[a]++] = out.indices[i+(k+2)%3];
        }
        std::vector<V3> tmp(n);
        auto pass = [&](float lambda) {
            #pragma omp parallel for
            for (uint32_t i = 0; i < n; i++) {
                V3 s{}; uint32_t c = off[i+1]-off[i];
                for (uint32_t e = off[i]; e < off[i+1]; e++) s = s + out.vertices[adj[e]];
                tmp[i] = c ? out.vertices[i] + (s*(1.f/c) - out.vertices[i])*lambda : out.vertices[i];
            }
            out.vertices.swap(tmp);
        };
        for (int it = 0; it < opt.smoothIterations; it++) { pass(0.5f); pass(-0.53f); }
    }
    if (opt.fillHolesMm != 0) for (int pass = 0; pass < 4; pass++) if (fillHoles(out, opt.fillHolesMm) == 0) break;
    if (opt.keepTriangles < 0.999f) simplifyMesh(out, (size_t)(out.triangleCount()*std::max(0.001f, opt.keepTriangles)));
    computeNormals(out);
    // Vertex colors from the nearest voxels that have seen color.
    bool anyColor = false;
    for (const Block& b : t.blocks()) { for (const Voxel& v : b.v) if (v.colorWeight) { anyColor = true; break; } if (anyColor) break; }
    if (anyColor) {
        out.colors.assign(out.vertices.size()*3, 128);
        const float iv = 1.f/vox;
        #pragma omp parallel for schedule(dynamic, 2048)
        for (size_t i = 0; i < out.vertices.size(); i++) {
            const V3 p = out.vertices[i]*iv; const int x0 = (int)std::floor(p.x), y0 = (int)std::floor(p.y), z0 = (int)std::floor(p.z);
            float wSum = 0, r = 0, g2 = 0, bl = 0;
            for (int dz = -1; dz <= 2; dz++) for (int dy = -1; dy <= 2; dy++) for (int dx = -1; dx <= 2; dx++) {
                const int x = x0+dx, y = y0+dy, z = z0+dz;
                const Block* bk = t.find(x >> 3, y >> 3, z >> 3); if (!bk) continue;
                const Voxel& v = bk->v[(((z & 7)*8 + (y & 7))*8) + (x & 7)];
                if (!v.colorWeight) continue;
                const V3 d{p.x-x, p.y-y, p.z-z}; const float w = v.colorWeight/(0.3f+dot(d, d));
                r += w*v.r; g2 += w*v.g; bl += w*v.b; wSum += w;
            }
            if (wSum > 0) { out.colors[i*3] = (uint8_t)(r/wSum); out.colors[i*3+1] = (uint8_t)(g2/wSum); out.colors[i*3+2] = (uint8_t)(bl/wSum); }
        }
    }
}

size_t countBoundaryEdges(const Mesh& m) {
    std::unordered_map<uint64_t, int> count; count.reserve(m.indices.size());
    for (size_t i = 0; i < m.indices.size(); i += 3) for (int k = 0; k < 3; k++) {
        const uint32_t a = m.indices[i+k], b = m.indices[i+(k+1)%3];
        count[((uint64_t)std::min(a, b) << 32) | std::max(a, b)]++;
    }
    size_t n = 0; for (auto& e : count) n += e.second == 1;
    return n;
}

std::vector<std::vector<uint32_t>> boundaryLoops(const Mesh& m) {
    // Directed boundary edges: an edge whose reverse does not exist. A real boundary edge belongs to
    // exactly one triangle. Edges used twice in the same direction are knots, not holes.
    std::vector<uint64_t> directed; directed.reserve(m.indices.size());
    auto key = [](uint32_t a, uint32_t b) { return ((uint64_t)a << 32) | b; };
    for (size_t i = 0; i < m.indices.size(); i += 3) for (int k = 0; k < 3; k++) directed.push_back(key(m.indices[i+k], m.indices[i+(k+1)%3]));
    std::sort(directed.begin(), directed.end());
    std::unordered_map<uint32_t, std::vector<uint32_t>> out;
    std::vector<uint32_t> starts;
    for (size_t i = 0; i < directed.size(); i++) {
        if ((i > 0 && directed[i-1] == directed[i]) || (i+1 < directed.size() && directed[i+1] == directed[i])) continue;
        const uint32_t a = (uint32_t)(directed[i] >> 32), b = (uint32_t)directed[i];
        if (!std::binary_search(directed.begin(), directed.end(), key(b, a))) { out[a].push_back(b); starts.push_back(a); }
    }
    // Walk along the boundary. Several holes may touch in one vertex, so walk over edges: whenever
    // the path meets a vertex it already contains, the part since then is one simple loop.
    std::vector<std::vector<uint32_t>> loops;
    for (uint32_t s0 : starts) {
        std::vector<uint32_t> path; std::unordered_map<uint32_t, size_t> at;
        uint32_t cur = s0;
        for (;;) {
            auto it = out.find(cur);
            if (it == out.end() || it->second.empty()) break;   // a dead end: leave what is left
            at[cur] = path.size(); path.push_back(cur);
            const uint32_t nx = it->second.back(); it->second.pop_back();
            auto seen = at.find(nx);
            if (seen == at.end()) { cur = nx; continue; }
            loops.emplace_back(path.begin()+seen->second, path.end());
            for (size_t i = seen->second; i < path.size(); i++) at.erase(path[i]);
            path.resize(seen->second);
            cur = nx;
        }
    }
    return loops;
}

int fillHoles(Mesh& m, float maxSize) {
    std::vector<std::vector<uint32_t>> loops = boundaryLoops(m);
    int closed = 0;
    for (std::vector<uint32_t>& loop : loops) {
        if (loop.size() < 3) continue;
        V3 lo{1e9f, 1e9f, 1e9f}, hi{-1e9f, -1e9f, -1e9f};
        for (uint32_t v : loop) { const V3 p = m.vertices[v]; lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)}; hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)}; }
        if (maxSize >= 0 && std::max({hi.x-lo.x, hi.y-lo.y, hi.z-lo.z}) > maxSize) continue;
        // Clip the corner with the smallest angle until the loop is closed. The boundary runs
        // along the existing triangles, so new triangles take the reverse direction.
        while (loop.size() > 3) {
            const size_t n = loop.size(); size_t best = 0; float bestCos = -2;
            const size_t step = std::max<size_t>(1, n/512);
            for (size_t i = 0; i < n; i += step) {
                const V3 p = m.vertices[loop[i]], a = normalized(m.vertices[loop[(i+n-1)%n]]-p), b = normalized(m.vertices[loop[(i+1)%n]]-p);
                const float c = dot(a, b);
                if (c > bestCos) { bestCos = c; best = i; }
            }
            m.indices.insert(m.indices.end(), {loop[(best+1)%n], loop[best], loop[(best+n-1)%n]});
            loop.erase(loop.begin()+best);
        }
        m.indices.insert(m.indices.end(), {loop[2], loop[1], loop[0]});
        closed++;
    }
    return closed;
}

void simplifyMesh(Mesh& m, size_t target) {
    const size_t nv = m.vertices.size(); size_t alive = m.triangleCount();
    if (alive <= target || !nv) return;
    struct Q { double a[10] = {0};   // symmetric 4x4
        void add(const Q& o, double w = 1) { for (int i = 0; i < 10; i++) a[i] += w*o.a[i]; }
        double eval(double x, double y, double z) const { return a[0]*x*x + 2*a[1]*x*y + 2*a[2]*x*z + 2*a[3]*x + a[4]*y*y + 2*a[5]*y*z + 2*a[6]*y + a[7]*z*z + 2*a[8]*z + a[9]; } };
    auto plane = [](double nx, double ny, double nz, double d, double w) { Q q; const double v[4] = {nx, ny, nz, d}; int k = 0; for (int i = 0; i < 4; i++) for (int j = i; j < 4; j++) q.a[k++] = w*v[i]*v[j]; return q; };
    std::vector<Q> quad(nv); std::vector<std::vector<uint32_t>> vt(nv); std::vector<uint32_t>& idx = m.indices;
    const size_t nt = m.triangleCount(); std::vector<uint8_t> dead(nt, 0);
    std::vector<V3>& P = m.vertices;
    std::unordered_map<uint64_t, int> edgeCount; edgeCount.reserve(nt*3);
    auto ekey = [](uint32_t a, uint32_t b) { return ((uint64_t)std::min(a, b) << 32) | std::max(a, b); };
    for (size_t t = 0; t < nt; t++) {
        const uint32_t a = idx[3*t], b = idx[3*t+1], c = idx[3*t+2];
        const V3 pa = P[a], pb = P[b], pc = P[c]; const V3 n = cross(pb-pa, pc-pa); const float l = norm(n);
        vt[a].push_back((uint32_t)t); vt[b].push_back((uint32_t)t); vt[c].push_back((uint32_t)t);
        edgeCount[ekey(a, b)]++; edgeCount[ekey(b, c)]++; edgeCount[ekey(c, a)]++;
        if (l < 1e-12f) continue;
        const V3 u = n*(1.f/l); const Q q = plane(u.x, u.y, u.z, -(double)dot(u, pa), l*0.5);
        quad[a].add(q); quad[b].add(q); quad[c].add(q);
    }
    // Open boundaries get a strong constraint plane, so that their outline does not shrink.
    for (size_t t = 0; t < nt; t++) for (int k = 0; k < 3; k++) {
        const uint32_t a = idx[3*t+k], b = idx[3*t+(k+1)%3], c = idx[3*t+(k+2)%3];
        if (edgeCount[ekey(a, b)] != 1) continue;
        const V3 e = P[b]-P[a], tn = cross(e, P[c]-P[a]), bn = normalized(cross(e, tn));
        const Q q = plane(bn.x, bn.y, bn.z, -(double)dot(bn, P[a]), 100.0*dot(e, e));
        quad[a].add(q); quad[b].add(q);
    }
    std::vector<uint32_t> parent(nv); std::iota(parent.begin(), parent.end(), 0);
    std::vector<uint32_t> version(nv, 0);
    struct Item { float cost; uint32_t a, b, va, vb; V3 pos; bool operator<(const Item& o) const { return cost > o.cost; } };
    auto makeItem = [&](uint32_t a, uint32_t b) {
        Q q = quad[a]; q.add(quad[b]);
        const V3 pa = P[a], pb = P[b], mid = (pa+pb)*0.5f; const V3 cand[3] = {mid, pa, pb};
        double best = 1e300; V3 pos = mid;
        for (const V3& c : cand) { const double e = q.eval(c.x, c.y, c.z); if (e < best) { best = e; pos = c; } }
        return Item{(float)best, a, b, version[a], version[b], pos};
    };
    std::priority_queue<Item> heap;
    { std::vector<Item> items; items.reserve(edgeCount.size());
      for (auto& e : edgeCount) items.push_back(makeItem((uint32_t)(e.first >> 32), (uint32_t)e.first));
      heap = std::priority_queue<Item>(std::less<Item>(), std::move(items)); }
    std::vector<uint32_t> mark(nv, 0); uint32_t stamp = 0;
    while (alive > target && !heap.empty()) {
        const Item it = heap.top(); heap.pop();
        if (version[it.a] != it.va || version[it.b] != it.vb) continue;
        const uint32_t a = it.a, b = it.b;
        // Link condition: the two vertices may share only the two vertices opposite the edge.
        stamp++; int shared = 0, sharedTris = 0;
        for (uint32_t t : vt[a]) if (!dead[t]) for (int k = 0; k < 3; k++) mark[idx[3*t+k]] = stamp;
        { std::vector<uint32_t> seen;
          for (uint32_t t : vt[b]) if (!dead[t]) {
              bool hasA = false; for (int k = 0; k < 3; k++) hasA |= idx[3*t+k] == a;
              sharedTris += hasA;
              for (int k = 0; k < 3; k++) { const uint32_t v = idx[3*t+k]; if (v != a && v != b && mark[v] == stamp) { bool dup = false; for (uint32_t s2 : seen) dup |= s2 == v; if (!dup) { seen.push_back(v); shared++; } } }
          } }
        if (sharedTris == 0 || shared > 2) continue;
        // Reject collapses that flip a neighbouring triangle or make it degenerate.
        bool bad = false;
        for (int side = 0; side < 2 && !bad; side++) for (uint32_t t : vt[side ? b : a]) {
            if (dead[t]) continue;
            uint32_t v[3] = {idx[3*t], idx[3*t+1], idx[3*t+2]};
            const bool hasA = v[0] == a || v[1] == a || v[2] == a, hasB = v[0] == b || v[1] == b || v[2] == b;
            if (hasA && hasB) continue;
            const V3 oldN = cross(P[v[1]]-P[v[0]], P[v[2]]-P[v[0]]);
            V3 q[3]; for (int k = 0; k < 3; k++) q[k] = (v[k] == a || v[k] == b) ? it.pos : P[v[k]];
            const V3 newN = cross(q[1]-q[0], q[2]-q[0]); const float ln = norm(newN), lo = norm(oldN);
            if (ln < 1e-6f*(lo+1e-6f) || (lo > 1e-9f && dot(newN, oldN) < 0.2f*ln*lo)) { bad = true; break; }
        }
        if (bad) continue;
        // Collapse b into a.
        P[a] = it.pos; quad[a].add(quad[b]); version[a]++; version[b]++;
        for (uint32_t t : vt[b]) {
            if (dead[t]) continue;
            bool hasA = false; for (int k = 0; k < 3; k++) hasA |= idx[3*t+k] == a;
            if (hasA) { dead[t] = 1; alive--; continue; }
            for (int k = 0; k < 3; k++) if (idx[3*t+k] == b) idx[3*t+k] = a;
            vt[a].push_back(t);
        }
        vt[b].clear(); vt[b].shrink_to_fit();
        { auto& l = vt[a]; l.erase(std::remove_if(l.begin(), l.end(), [&](uint32_t t) { return dead[t]; }), l.end()); }
        stamp++;
        for (uint32_t t : vt[a]) for (int k = 0; k < 3; k++) {
            const uint32_t v = idx[3*t+k];
            if (v == a || mark[v] == stamp) continue;
            mark[v] = stamp; heap.push(makeItem(a, v));
        }
    }
    // Compact.
    const bool hasColor = m.colors.size() == nv*3;
    std::vector<uint32_t> remap(nv, 0xffffffff), ni; std::vector<V3> nvs; std::vector<uint8_t> ncs; ni.reserve(alive*3); nvs.reserve(alive);
    for (size_t t = 0; t < nt; t++) {
        if (dead[t]) continue;
        for (int k = 0; k < 3; k++) {
            const uint32_t v = idx[3*t+k];
            if (remap[v] == 0xffffffff) {
                remap[v] = (uint32_t)nvs.size(); nvs.push_back(P[v]);
                if (hasColor) ncs.insert(ncs.end(), {m.colors[v*3], m.colors[v*3+1], m.colors[v*3+2]});
            }
            ni.push_back(remap[v]);
        }
    }
    m.vertices.swap(nvs); m.indices.swap(ni); m.colors.swap(ncs); m.normals.clear();
}

void computeNormals(Mesh& m) {
    m.normals.assign(m.vertices.size(), V3{});
    for (size_t i = 0; i < m.indices.size(); i += 3) {
        uint32_t a = m.indices[i], b = m.indices[i+1], c = m.indices[i+2];
        V3 n = cross(m.vertices[b]-m.vertices[a], m.vertices[c]-m.vertices[a]);
        m.normals[a] = m.normals[a] + n; m.normals[b] = m.normals[b] + n; m.normals[c] = m.normals[c] + n;
    }
    for (auto& n : m.normals) n = normalized(n);
}
static std::vector<V3> sampledPoints(const Mesh& m, size_t count) {
    std::vector<V3> out; const size_t stride = std::max<size_t>(1, m.vertices.size()/count); out.reserve(m.vertices.size()/stride+1);
    for (size_t i = 0; i < m.vertices.size(); i += stride) out.push_back(m.vertices[i]);
    return out;
}
void tightBoxSize(const Mesh& m, float size[3]) {
    size[0] = size[1] = size[2] = 0; if (m.vertices.empty()) return;
    const std::vector<V3> pts = sampledPoints(m, 2000);
    // The outermost 0.2 % on each side do not count: a small stray piece must not make a cube read 2 mm larger.
    const size_t trim = pts.size()/500;
    auto volumeAt = [&](float ax, float ay, float az, float out[3]) {
        const float cx = std::cos(ax), sx = std::sin(ax), cy = std::cos(ay), sy = std::sin(ay), cz = std::cos(az), sz = std::sin(az);
        const V3 axes[3] = {{cy*cz, sx*sy*cz-cx*sz, cx*sy*cz+sx*sz}, {cy*sz, sx*sy*sz+cx*cz, cx*sy*sz-sx*cz}, {-sy, sx*cy, cx*cy}};
        std::vector<float> v(pts.size());
        for (int k = 0; k < 3; k++) {
            for (size_t i = 0; i < pts.size(); i++) v[i] = dot(pts[i], axes[k]);
            std::nth_element(v.begin(), v.begin()+trim, v.end()); const float lo = v[trim];
            std::nth_element(v.begin(), v.end()-1-trim, v.end()); out[k] = v[v.size()-1-trim]-lo;
        }
        return out[0]*out[1]*out[2];
    };
    // From a coarse grid of turns to finer ones around the best.
    float cA[3] = {0, 0, 0}, best = 1e30f; const float rad = 0.0174533f;
    struct Stage { int n; float stepDeg; }; const Stage stages[] = {{8, 6.f}, {6, 1.f}, {4, .25f}};
    for (const Stage& st : stages) {
        float nA[3] = {cA[0], cA[1], cA[2]};
        const int N = st.n*2+1;
        #pragma omp parallel for schedule(dynamic, 8)
        for (int q = 0; q < N*N*N; q++) {
            const float a = cA[0]+(q%N-st.n)*st.stepDeg*rad, b = cA[1]+((q/N)%N-st.n)*st.stepDeg*rad, c = cA[2]+((q/N/N)-st.n)*st.stepDeg*rad; float s[3];
            const float v = volumeAt(a, b, c, s);
            #pragma omp critical
            if (v < best) { best = v; nA[0] = a; nA[1] = b; nA[2] = c; size[0] = s[0]; size[1] = s[1]; size[2] = s[2]; }
        }
        cA[0] = nA[0]; cA[1] = nA[1]; cA[2] = nA[2];
    }
    std::sort(size, size+3, std::greater<float>());
}
void orientUpright(Mesh& m, V3 up) {
    if (m.vertices.empty()) return;
    const V3 z = normalized(up);
    const V3 x = normalized(cross(std::fabs(z.y) < 0.9f ? V3{0, 1, 0} : V3{1, 0, 0}, z)), y = cross(z, x);
    auto turn = [&](V3 p) { return V3{dot(p, x), dot(p, y), dot(p, z)}; };
    for (V3& p : m.vertices) p = turn(p);
    for (V3& n : m.normals) n = turn(n);
    // Turn around the vertical so that the base fits the axes tightest: a straight box then measures
    // its true sides in every program, not the larger box around a turned one.
    { const std::vector<V3> sample = sampledPoints(m, 4000); float best = 1e30f, bestA = 0;
      for (int k = 0; k < 180; k++) { const float a = (k*.5f-45.f)*0.0174533f, c = std::cos(a), s = std::sin(a); float x0 = 1e30f, x1 = -1e30f, y0 = 1e30f, y1 = -1e30f;
          for (const V3& p : sample) { const float X = c*p.x-s*p.y, Y = s*p.x+c*p.y; x0 = std::min(x0, X); x1 = std::max(x1, X); y0 = std::min(y0, Y); y1 = std::max(y1, Y); }
          const float area = (x1-x0)*(y1-y0); if (area < best*.999f) { best = area; bestA = a; } }
      const float c = std::cos(bestA), s = std::sin(bestA);
      for (V3& p : m.vertices) p = V3{c*p.x-s*p.y, s*p.x+c*p.y, p.z};
      for (V3& n : m.normals) n = V3{c*n.x-s*n.y, s*n.x+c*n.y, n.z}; }
    // Robust against stray points: the underside is where the lowest 0.1 % begin.
    std::vector<float> zs; zs.reserve(m.vertices.size()); for (const V3& p : m.vertices) zs.push_back(p.z);
    std::nth_element(zs.begin(), zs.begin()+zs.size()/1000, zs.end());
    V3 lo, hi; meshBounds(m, lo, hi);
    const V3 shift{-(lo.x+hi.x)/2, -(lo.y+hi.y)/2, -zs[zs.size()/1000]};
    for (V3& p : m.vertices) p = p + shift;
}
double meshVolume(const Mesh& m) {
    double v = 0;
    for (size_t i = 0; i < m.indices.size(); i += 3)
        v += dot(m.vertices[m.indices[i]], cross(m.vertices[m.indices[i+1]], m.vertices[m.indices[i+2]]));
    return v/6.0;
}
void meshBounds(const Mesh& m, V3& lo, V3& hi) {
    lo = {1e9f, 1e9f, 1e9f}; hi = {-1e9f, -1e9f, -1e9f};
    for (const V3& p : m.vertices) { lo = {std::min(lo.x,p.x), std::min(lo.y,p.y), std::min(lo.z,p.z)}; hi = {std::max(hi.x,p.x), std::max(hi.y,p.y), std::max(hi.z,p.z)}; }
}
std::string uniquePath(const std::string& base, const std::string& ext) {
    std::string p = base + ext;
    for (int i = 2; std::filesystem::exists(p); i++) p = base + "_" + std::to_string(i) + ext;
    return p;
}
bool saveMesh(const Mesh& m, const std::string& path, std::string& error) {
    std::string ext = std::filesystem::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    if (m.indices.empty()) { error = tr("Das Netz ist leer"); return false; }
    std::string tmp = path + ".tmp";
    FILE* f = fopen(tmp.c_str(), "wb");
    if (!f) { error = tr("Datei kann nicht geschrieben werden: ") + path; return false; }
    bool ok = true;
    if (ext == ".stl") {
        char hdr[80] = "VoxelForge binary STL, units: mm"; fwrite(hdr, 1, 80, f);
        uint32_t n = (uint32_t)m.triangleCount(); fwrite(&n, 4, 1, f);
        for (size_t i = 0; i < m.indices.size() && ok; i += 3) {
            V3 a = m.vertices[m.indices[i]], b = m.vertices[m.indices[i+1]], c = m.vertices[m.indices[i+2]], fn = normalized(cross(b-a, c-a));
            float rec[12] = {fn.x,fn.y,fn.z,a.x,a.y,a.z,b.x,b.y,b.z,c.x,c.y,c.z}; uint16_t attr = 0;
            ok = fwrite(rec, 4, 12, f) == 12 && fwrite(&attr, 2, 1, f) == 1;
        }
    } else if (ext == ".obj") {
        fprintf(f, "# VoxelForge, units: mm\n");
        const bool col = m.colors.size() == m.vertices.size()*3;
        for (size_t i = 0; i < m.vertices.size(); i++) {
            if (col) fprintf(f, "v %.4f %.4f %.4f %.3f %.3f %.3f\n", m.vertices[i].x, m.vertices[i].y, m.vertices[i].z, m.colors[i*3]/255.f, m.colors[i*3+1]/255.f, m.colors[i*3+2]/255.f);
            else fprintf(f, "v %.4f %.4f %.4f\n", m.vertices[i].x, m.vertices[i].y, m.vertices[i].z);
            fprintf(f, "vn %.4f %.4f %.4f\n", m.normals[i].x, m.normals[i].y, m.normals[i].z);
        }
        for (size_t i = 0; i < m.indices.size(); i += 3) ok = fprintf(f, "f %u//%u %u//%u %u//%u\n", m.indices[i]+1, m.indices[i]+1, m.indices[i+1]+1, m.indices[i+1]+1, m.indices[i+2]+1, m.indices[i+2]+1) > 0 && ok;
    } else if (ext == ".ply") {
        fprintf(f, "ply\nformat binary_little_endian 1.0\ncomment VoxelForge units mm\nelement vertex %zu\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\n%selement face %zu\nproperty list uchar uint vertex_indices\nend_header\n", m.vertices.size(), m.colors.size() == m.vertices.size()*3 ? "property uchar red\nproperty uchar green\nproperty uchar blue\n" : "", m.triangleCount());
        const bool col = m.colors.size() == m.vertices.size()*3;
        for (size_t i = 0; i < m.vertices.size(); i++) { float r[6] = {m.vertices[i].x, m.vertices[i].y, m.vertices[i].z, m.normals[i].x, m.normals[i].y, m.normals[i].z}; ok = fwrite(r, 4, 6, f) == 6 && ok; if (col) ok = fwrite(&m.colors[i*3], 1, 3, f) == 3 && ok; }
        for (size_t i = 0; i < m.indices.size(); i += 3) { uint8_t c = 3; ok = fwrite(&c, 1, 1, f) == 1 && fwrite(&m.indices[i], 4, 3, f) == 3 && ok; }
    } else { fclose(f); std::remove(tmp.c_str()); error = tr("Unbekanntes Dateiformat: ") + ext; return false; }
    ok = fclose(f) == 0 && ok;
    if (!ok) { std::remove(tmp.c_str()); error = tr("Schreibfehler (Festplatte voll?)"); return false; }
    std::rename(tmp.c_str(), path.c_str());
    return true;
}
