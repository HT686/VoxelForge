#include "meshedit.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <numeric>
#include <random>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include "measure.hpp"

namespace {
inline uint64_t ukey(uint32_t a, uint32_t b) { return ((uint64_t)std::min(a, b) << 32) | std::max(a, b); }
// Neighbours per vertex. Shared edges appear twice, which does no harm to a lookup.
struct Adjacency {
    std::vector<uint32_t> off, adj;
    explicit Adjacency(const Mesh& m) {
        const size_t n = m.vertices.size(); off.assign(n+1, 0);
        for (uint32_t v : m.indices) off[v+1] += 2;
        for (size_t i = 0; i < n; i++) off[i+1] += off[i];
        adj.resize(off[n]); std::vector<uint32_t> pos(off.begin(), off.end()-1);
        for (size_t i = 0; i < m.indices.size(); i += 3) for (int k = 0; k < 3; k++) {
            const uint32_t a = m.indices[i+k]; adj[pos[a]++] = m.indices[i+(k+1)%3]; adj[pos[a]++] = m.indices[i+(k+2)%3];
        }
    }
    bool has(uint32_t a, uint32_t b) const {
        if (a+1 >= off.size()) return false;
        for (uint32_t e = off[a]; e < off[a+1]; e++) if (adj[e] == b) return true;
        return false;
    }
};
inline float triArea(V3 a, V3 b, V3 c) { return 0.5f*norm(cross(b-a, c-a)); }
inline float angleAt(V3 p, V3 a, V3 b) { return std::acos(std::clamp(dot(normalized(a-p), normalized(b-p)), -1.f, 1.f)); }

// Fills holes one by one. The adjacency of the old mesh is shared, which makes small holes cheap
// even in a mesh with millions of triangles.
class Filler {
public:
    Filler(Mesh& m, FillType t) : m_(m), type_(t), adj_(m) {}
    bool fill(const std::vector<uint32_t>& loop);
private:
    using Tri = std::array<uint32_t, 3>;
    static constexpr size_t kMaxExact = 180;   // minimum-weight triangulation is cubic in the rim length
    static constexpr size_t kMaxFree = 8000;   // new vertices per hole; the direct solve stays below a few seconds
    Mesh& m_; FillType type_; Adjacency adj_;
    std::unordered_set<uint64_t> added_;       // edges that exist since the adjacency was built
    bool edgeExists(uint32_t a, uint32_t b) const { return adj_.has(a, b) || added_.count(ukey(a, b)); }
    // Everything below uses local numbers: rim 0..rim-1 in loop order, then new vertices.
    std::vector<V3> p_; std::vector<Tri> t_; uint32_t rim_ = 0; std::vector<uint32_t> loop_;
    bool triangulate(std::vector<uint32_t> sub);
    bool triangulateExact(const std::vector<uint32_t>& sub);
    void earClip(std::vector<uint32_t> sub);
    void refine(float edge);
    void relax(int iterations);
    int flipSweep();
    int splitLong(float limit);
    bool fair(int order);
};

// Minimum-weight triangulation of a rim piece (Barequet & Sharir, Liepa). Weight is area, with some
// edge length so that a flat straight rim does not end in triangles without area.
bool Filler::triangulateExact(const std::vector<uint32_t>& sub) {
    const int n = (int)sub.size();
    if (n < 3) return true;
    if (n == 3) { t_.push_back({sub[0], sub[2], sub[1]}); return true; }
    std::vector<float> W((size_t)n*n, 0.f); std::vector<int> K((size_t)n*n, -1);
    std::vector<uint8_t> ok((size_t)n*n, 1);
    for (int i = 0; i < n; i++) for (int j = i+2; j < n; j++) if (!(i == 0 && j == n-1)) ok[(size_t)i*n+j] = !edgeExists(loop_[sub[i]], loop_[sub[j]]);
    auto weight = [&](int i, int k, int j) {
        const V3 a = p_[sub[i]], b = p_[sub[k]], c = p_[sub[j]]; const V3 e0 = b-a, e1 = c-b, e2 = a-c;
        return triArea(a, b, c) + 0.05f*(dot(e0, e0)+dot(e1, e1)+dot(e2, e2));
    };
    for (int len = 2; len < n; len++) for (int i = 0; i+len < n; i++) {
        const int j = i+len; float best = 1e30f; int bestK = -1;
        if (!ok[(size_t)i*n+j]) { W[(size_t)i*n+j] = 1e30f; continue; }
        for (int k = i+1; k < j; k++) {
            const float a = W[(size_t)i*n+k], b = W[(size_t)k*n+j]; if (a >= 1e30f || b >= 1e30f) continue;
            const float w = a+b+weight(i, k, j); if (w < best) { best = w; bestK = k; }
        }
        W[(size_t)i*n+j] = best; K[(size_t)i*n+j] = bestK;
    }
    if (K[(size_t)n-1] < 0) return false;
    std::vector<std::pair<int, int>> st{{0, n-1}};
    while (!st.empty()) {
        const auto [i, j] = st.back(); st.pop_back();
        if (j-i < 2) continue;
        const int k = K[(size_t)i*n+j];
        t_.push_back({sub[i], sub[j], sub[k]});   // the rim runs along the old triangles, so new ones run against it
        added_.insert(ukey(loop_[sub[i]], loop_[sub[j]])); added_.insert(ukey(loop_[sub[i]], loop_[sub[k]])); added_.insert(ukey(loop_[sub[k]], loop_[sub[j]]));
        st.push_back({i, k}); st.push_back({k, j});
    }
    return true;
}
// Last resort when the rim touches itself so often that no clean triangulation exists.
void Filler::earClip(std::vector<uint32_t> sub) {
    while (sub.size() > 3) {
        const size_t n = sub.size(); size_t best = 0; float bestCos = -2;
        for (size_t i = 0; i < n; i++) {
            const V3 q = p_[sub[i]], a = normalized(p_[sub[(i+n-1)%n]]-q), b = normalized(p_[sub[(i+1)%n]]-q);
            const float c = dot(a, b); if (c > bestCos) { bestCos = c; best = i; }
        }
        t_.push_back({sub[(best+1)%n], sub[best], sub[(best+n-1)%n]});
        sub.erase(sub.begin()+best);
    }
    if (sub.size() == 3) t_.push_back({sub[2], sub[1], sub[0]});
}
bool Filler::triangulate(std::vector<uint32_t> sub) {
    // Long rims are first cut along their narrowest place, which also suits long thin gaps.
    if (sub.size() > kMaxExact) {
        const int n = (int)sub.size(); int bi = -1, bj = -1; float best = 1e30f;
        const int step = std::max(1, n/96);
        for (int i = 0; i < n; i += step) for (int j = i+n/4; j <= i+3*n/4 && j < n; j += step) {
            const uint32_t a = loop_[sub[i]], b = loop_[sub[j]]; if (edgeExists(a, b)) continue;
            const float d = norm(p_[sub[i]]-p_[sub[j]]); if (d < best) { best = d; bi = i; bj = j; }
        }
        if (bi >= 0) {
            added_.insert(ukey(loop_[sub[bi]], loop_[sub[bj]]));
            std::vector<uint32_t> a(sub.begin()+bi, sub.begin()+bj+1), b(sub.begin()+bj, sub.end()); b.insert(b.end(), sub.begin(), sub.begin()+bi+1);
            return triangulate(std::move(a)) && triangulate(std::move(b));
        }
        earClip(std::move(sub)); return true;
    }
    if (!triangulateExact(sub)) earClip(std::move(sub));
    return true;
}
int Filler::splitLong(float limit) {
    struct E { uint64_t key; uint32_t tri; };
    std::vector<E> es; es.reserve(t_.size()*3);
    for (uint32_t i = 0; i < t_.size(); i++) for (int k = 0; k < 3; k++) es.push_back({ukey(t_[i][k], t_[i][(k+1)%3]), i});
    std::sort(es.begin(), es.end(), [](const E& a, const E& b) { return a.key < b.key; });
    struct S { float len; uint32_t a, b, t0, t1; }; std::vector<S> cand;
    for (size_t i = 0; i+1 < es.size(); i++) if (es[i].key == es[i+1].key) {
        const uint32_t a = (uint32_t)(es[i].key >> 32), b = (uint32_t)es[i].key;
        const float l = norm(p_[a]-p_[b]); if (l > limit) cand.push_back({l, a, b, es[i].tri, es[i+1].tri});
    }
    std::sort(cand.begin(), cand.end(), [](const S& x, const S& y) { return x.len > y.len; });
    std::vector<uint8_t> touched(t_.size(), 0); int n = 0;
    auto replace = [&](uint32_t tri, uint32_t a, uint32_t b, uint32_t mid) {
        Tri t = t_[tri]; int k = 0; while (!((t[k] == a && t[(k+1)%3] == b) || (t[k] == b && t[(k+1)%3] == a))) k++;
        const uint32_t x = t[k], y = t[(k+1)%3], c = t[(k+2)%3];
        t_[tri] = {x, mid, c}; t_.push_back({mid, y, c});
    };
    for (const S& s : cand) {
        if (touched[s.t0] || touched[s.t1] || p_.size() - rim_ >= kMaxFree) continue;
        touched[s.t0] = touched[s.t1] = 1;
        const uint32_t mid = (uint32_t)p_.size(); p_.push_back((p_[s.a]+p_[s.b])*0.5f);
        replace(s.t0, s.a, s.b, mid); replace(s.t1, s.a, s.b, mid); n++;
    }
    return n;
}
int Filler::flipSweep() {
    std::unordered_map<uint64_t, std::array<int, 2>> em; em.reserve(t_.size()*3);
    for (int i = 0; i < (int)t_.size(); i++) for (int k = 0; k < 3; k++) {
        auto it = em.try_emplace(ukey(t_[i][k], t_[i][(k+1)%3]), std::array<int, 2>{-1, -1}).first;
        (it->second[0] < 0 ? it->second[0] : it->second[1]) = i;
    }
    std::vector<uint8_t> touched(t_.size(), 0); int flips = 0;
    std::vector<uint64_t> keys; keys.reserve(em.size());
    for (auto& e : em) if (e.second[1] >= 0) keys.push_back(e.first);
    std::sort(keys.begin(), keys.end());   // the same result on every run
    for (uint64_t key : keys) {
        const auto tr = em[key]; if (touched[tr[0]] || touched[tr[1]]) continue;
        uint32_t a = (uint32_t)(key >> 32), b = (uint32_t)key;
        Tri t0 = t_[tr[0]]; int k = 0; while (t0[k] == a || t0[k] == b) k++;
        const uint32_t c = t0[k];
        if (t0[(k+1)%3] != a) std::swap(a, b);   // now t0 runs a -> b -> c
        Tri t1 = t_[tr[1]]; k = 0; while (t1[k] == a || t1[k] == b) k++;
        const uint32_t d = t1[k]; if (c == d) continue;
        if (angleAt(p_[c], p_[a], p_[b]) + angleAt(p_[d], p_[a], p_[b]) <= 3.1416f + 1e-3f) continue;
        if (em.count(ukey(c, d))) continue;
        if (c < rim_ && d < rim_ && edgeExists(loop_[c], loop_[d])) continue;
        t_[tr[0]] = {c, a, d}; t_[tr[1]] = {d, b, c};
        touched[tr[0]] = touched[tr[1]] = 1; flips++;
        em[ukey(c, d)] = {tr[0], tr[1]};
    }
    return flips;
}
void Filler::relax(int iterations) {
    const uint32_t n = (uint32_t)p_.size(); if (n == rim_) return;
    std::vector<std::vector<uint32_t>> nb(n);
    for (const Tri& t : t_) for (int k = 0; k < 3; k++) nb[t[k]].push_back(t[(k+1)%3]);
    for (int it = 0; it < iterations; it++) for (uint32_t v = rim_; v < n; v++) {
        V3 s{}; for (uint32_t o : nb[v]) s = s + p_[o];
        if (!nb[v].empty()) p_[v] = s*(1.f/nb[v].size());
    }
}
void Filler::refine(float edge) {
    for (int pass = 0; pass < 24; pass++) {
        const int splits = splitLong(edge*1.33f);
        for (int s = 0; s < 6; s++) if (!flipSweep()) break;
        relax(4);
        if (!splits) break;
    }
    for (int s = 0; s < 8; s++) { if (!flipSweep()) break; relax(2); }
}
// Solves L^order x = 0 for the new vertices, with everything else as boundary condition.
bool Filler::fair(int order) {
    const uint32_t nLocal = (uint32_t)p_.size(), nFree = nLocal-rim_;
    if (!nFree) return true;
    std::unordered_map<uint32_t, uint32_t> id;   // old mesh vertex -> node
    for (uint32_t i = 0; i < rim_; i++) id[loop_[i]] = i;
    std::vector<std::vector<uint32_t>> nb(nLocal); std::vector<V3> pos = p_; std::vector<uint32_t> global(loop_.begin(), loop_.end());
    for (const Tri& t : t_) for (int k = 0; k < 3; k++) nb[t[k]].push_back(t[(k+1)%3]), nb[t[(k+1)%3]].push_back(t[k]);
    // Grow outward through the old mesh: the rim needs its old neighbours, order 3 also theirs.
    std::vector<uint32_t> front(rim_); std::iota(front.begin(), front.end(), 0);
    global.resize(nLocal, 0xffffffff);
    for (int layer = 1; layer < order; layer++) {
        std::vector<uint32_t> next;
        for (uint32_t node : front) {
            const uint32_t g = global[node];
            if (g+1 >= adj_.off.size()) continue;
            for (uint32_t e = adj_.off[g]; e < adj_.off[g+1]; e++) {
                const uint32_t o = adj_.adj[e];
                auto [it, fresh] = id.try_emplace(o, (uint32_t)pos.size());
                if (fresh) { pos.push_back(m_.vertices[o]); global.push_back(o); nb.emplace_back(); next.push_back(it->second); }
                nb[node].push_back(it->second);
            }
        }
        front.swap(next);
    }
    const uint32_t N = (uint32_t)pos.size();
    for (auto& l : nb) { std::sort(l.begin(), l.end()); l.erase(std::unique(l.begin(), l.end()), l.end()); }
    // Edge lengths as weights (Fujiwara): a uniform Laplacian only suits an even mesh, and the higher
    // the order, the more an uneven one distorts the result.
    float meanLen = 0; size_t nLen = 0;
    for (uint32_t i = 0; i < N; i++) for (uint32_t o : nb[i]) { meanLen += norm(pos[i]-pos[o]); nLen++; }
    meanLen = nLen ? std::max(1e-5f, meanLen/nLen) : 1.f;
    std::vector<std::vector<float>> wgt(N); std::vector<double> diag(N, 0), massInv(N, 1);
    for (uint32_t i = 0; i < N; i++) {
        wgt[i].resize(nb[i].size()); double sumLen = 0;
        for (size_t e = 0; e < nb[i].size(); e++) { const float l = std::max(norm(pos[i]-pos[nb[i][e]])/meanLen, 0.05f); wgt[i][e] = 1.f/l; diag[i] += wgt[i][e]; sumLen += l; }
        if (sumLen > 0) massInv[i] = 2.0/sumLen;
    }
    // Row of L (M^-1 L)^(order-1) per free vertex. Only rows of vertices closer than the outermost
    // layer are needed, and those are complete, so the matrix is exact and symmetric.
    V3 c0{}; for (uint32_t i = 0; i < rim_; i++) c0 = c0 + p_[i]; c0 = c0*(1.f/rim_);
    std::vector<std::vector<std::pair<uint32_t, double>>> rows(nFree);
    std::vector<std::array<double, 3>> rhs(nFree, std::array<double, 3>{0, 0, 0});
    std::vector<double> y(N, 0), z(N, 0); std::vector<uint32_t> act, nxt; std::vector<uint8_t> inN(N, 0);
    for (uint32_t f = 0; f < nFree; f++) {
        act.assign(1, rim_+f); y[rim_+f] = 1;
        for (int it = 0; it < order; it++) {
            nxt.clear();
            for (uint32_t node : act) {
                const double v = y[node]; y[node] = 0;
                if (!inN[node]) { inN[node] = 1; nxt.push_back(node); }
                z[node] += v*diag[node];
                for (size_t e = 0; e < nb[node].size(); e++) { const uint32_t o = nb[node][e]; z[o] -= v*wgt[node][e]; if (!inN[o]) { inN[o] = 1; nxt.push_back(o); } }
            }
            for (uint32_t node : nxt) { y[node] = it < order-1 ? z[node]*massInv[node] : z[node]; z[node] = 0; inN[node] = 0; }
            act = nxt;
        }
        for (uint32_t node : act) {
            const double v = y[node]; y[node] = 0; if (v == 0) continue;
            if (node >= rim_ && node < nLocal) rows[f].push_back({node-rim_, v});
            else { const V3 q = pos[node]-c0; rhs[f][0] -= v*q.x; rhs[f][1] -= v*q.y; rhs[f][2] -= v*q.z; }
        }
    }
    // Reverse Cuthill-McKee keeps the factor narrow.
    std::vector<uint32_t> order_; order_.reserve(nFree); std::vector<uint8_t> seen(nFree, 0);
    std::vector<uint32_t> starts(nFree); std::iota(starts.begin(), starts.end(), 0);
    std::sort(starts.begin(), starts.end(), [&](uint32_t a, uint32_t b) { return rows[a].size() != rows[b].size() ? rows[a].size() < rows[b].size() : a < b; });
    for (uint32_t s : starts) {
        if (seen[s]) continue;
        size_t head = order_.size(); seen[s] = 1; order_.push_back(s);
        while (head < order_.size()) {
            const uint32_t u = order_[head++]; std::vector<uint32_t> ns;
            for (auto& r : rows[u]) if (!seen[r.first]) { seen[r.first] = 1; ns.push_back(r.first); }
            std::sort(ns.begin(), ns.end(), [&](uint32_t a, uint32_t b) { return rows[a].size() != rows[b].size() ? rows[a].size() < rows[b].size() : a < b; });
            order_.insert(order_.end(), ns.begin(), ns.end());
        }
    }
    std::reverse(order_.begin(), order_.end());
    std::vector<uint32_t> perm(nFree); for (uint32_t i = 0; i < nFree; i++) perm[order_[i]] = i;
    std::vector<uint32_t> first(nFree); std::vector<size_t> start(nFree+1, 0);
    for (uint32_t f = 0; f < nFree; f++) { uint32_t lo = perm[f]; for (auto& r : rows[f]) lo = std::min(lo, perm[r.first]); first[perm[f]] = lo; }
    for (uint32_t i = 0; i < nFree; i++) start[i+1] = start[i]+(i-first[i]+1);
    if (start[nFree] > (size_t)300000000) return false;
    std::vector<double> L(start[nFree], 0.0);
    for (uint32_t f = 0; f < nFree; f++) { const uint32_t i = perm[f]; for (auto& r : rows[f]) { const uint32_t j = perm[r.first]; if (j <= i) L[start[i]+(j-first[i])] = r.second; } }
    for (uint32_t i = 0; i < nFree; i++) {
        double* Li = &L[start[i]]; const uint32_t fi = first[i];
        for (uint32_t j = fi; j <= i; j++) {
            const double* Lj = &L[start[j]]; const uint32_t fj = first[j], k0 = std::max(fi, fj);
            double s = Li[j-fi];
            for (uint32_t k = k0; k < j; k++) s -= Li[k-fi]*Lj[k-fj];
            if (j < i) Li[j-fi] = s/Lj[j-fj];
            else { if (s <= 1e-14) return false; Li[j-fi] = std::sqrt(s); }
        }
    }
    std::vector<double> sol(nFree);
    std::array<std::vector<double>, 3> out;
    for (int c = 0; c < 3; c++) {
        for (uint32_t f = 0; f < nFree; f++) sol[perm[f]] = rhs[f][c];
        for (uint32_t i = 0; i < nFree; i++) { double s = sol[i]; for (uint32_t k = first[i]; k < i; k++) s -= L[start[i]+k-first[i]]*sol[k]; sol[i] = s/L[start[i]+i-first[i]]; }
        for (uint32_t i = nFree; i-- > 0;) { const double v = sol[i] /= L[start[i]+i-first[i]]; for (uint32_t k = first[i]; k < i; k++) sol[k] -= L[start[i]+k-first[i]]*v; }
        out[c].resize(nFree); for (uint32_t f = 0; f < nFree; f++) { out[c][f] = sol[perm[f]]; if (!std::isfinite(out[c][f])) return false; }
    }
    for (uint32_t f = 0; f < nFree; f++) p_[rim_+f] = V3{(float)out[0][f], (float)out[1][f], (float)out[2][f]}+c0;
    return true;
}
bool Filler::fill(const std::vector<uint32_t>& loop) {
    if (loop.size() < 3) return false;
    loop_ = loop; rim_ = (uint32_t)loop.size(); t_.clear(); p_.resize(rim_);
    float rimLen = 0; for (uint32_t i = 0; i < rim_; i++) { p_[i] = m_.vertices[loop[i]]; rimLen += norm(m_.vertices[loop[i]]-m_.vertices[loop[(i+1)%rim_]]); }
    std::vector<uint32_t> all(rim_); std::iota(all.begin(), all.end(), 0);
    if (!triangulate(all)) return false;
    float area = 0; for (const Tri& t : t_) area += triArea(p_[t[0]], p_[t[1]], p_[t[2]]);
    // The filling continues the mesh density around it, unless that would be too many vertices.
    const float edge = std::max(rimLen/rim_, std::sqrt(area/(0.43f*kMaxFree)));
    refine(edge);
    const std::vector<V3> relaxed = p_;
    // Twice for the higher orders: the first result gives the edge lengths for the second.
    const int target = type_ == FillType::Curvature ? 3 : type_ == FillType::Tangent ? 2 : 1;
    bool solved = false;
    for (int order = target; order >= 1 && !solved; order--) {
        solved = target == 1 || fair(std::min(order, 2));
        if (solved) { const std::vector<V3> first = p_; if (!fair(order)) { p_ = first; } }
    }
    if (!solved) p_ = relaxed;
    // Into the mesh.
    const uint32_t base = (uint32_t)m_.vertices.size(), nNew = (uint32_t)p_.size()-rim_;
    auto global = [&](uint32_t v) { return v < rim_ ? loop_[v] : base+(v-rim_); };
    const bool hasN = m_.normals.size() == m_.vertices.size(), hasC = m_.colors.size() == m_.vertices.size()*3;
    m_.vertices.insert(m_.vertices.end(), p_.begin()+rim_, p_.end());
    if (hasN) m_.normals.resize(m_.vertices.size(), V3{});
    if (hasC) {
        // Colors flow in from the rim.
        std::vector<std::array<float, 3>> col(p_.size(), std::array<float, 3>{0, 0, 0}); std::array<float, 3> mean{0, 0, 0};
        for (uint32_t i = 0; i < rim_; i++) for (int c = 0; c < 3; c++) { col[i][c] = m_.colors[loop_[i]*3+c]; mean[c] += col[i][c]/rim_; }
        for (uint32_t i = rim_; i < p_.size(); i++) col[i] = mean;
        std::vector<std::vector<uint32_t>> nb(p_.size());
        for (const Tri& t : t_) for (int k = 0; k < 3; k++) nb[t[k]].push_back(t[(k+1)%3]);
        for (int it = 0; it < 60; it++) for (uint32_t v = rim_; v < p_.size(); v++) if (!nb[v].empty()) {
            std::array<float, 3> s{0, 0, 0}; for (uint32_t o : nb[v]) for (int c = 0; c < 3; c++) s[c] += col[o][c];
            for (int c = 0; c < 3; c++) col[v][c] = s[c]/nb[v].size();
        }
        m_.colors.resize(m_.vertices.size()*3);
        for (uint32_t i = 0; i < nNew; i++) for (int c = 0; c < 3; c++) m_.colors[(base+i)*3+c] = (uint8_t)std::clamp(col[rim_+i][c], 0.f, 255.f);
    }
    for (const Tri& t : t_) {
        const uint32_t a = global(t[0]), b = global(t[1]), c = global(t[2]);
        m_.indices.insert(m_.indices.end(), {a, b, c});
        added_.insert(ukey(a, b)); added_.insert(ukey(b, c)); added_.insert(ukey(c, a));
    }
    return true;
}
}

size_t countKnotEdges(const Mesh& m) {
    std::vector<uint64_t> e; e.reserve(m.indices.size());
    for (size_t i = 0; i < m.indices.size(); i += 3) for (int k = 0; k < 3; k++) e.push_back(ukey(m.indices[i+k], m.indices[i+(k+1)%3]));
    std::sort(e.begin(), e.end()); size_t n = 0;
    for (size_t i = 0; i < e.size();) { size_t j = i; while (j < e.size() && e[j] == e[i]) j++; n += j-i > 2; i = j; }
    return n;
}
static size_t separateSheets(Mesh& m) {
    // Triangles with a repeated vertex connect nothing.
    { std::vector<uint32_t> ni; ni.reserve(m.indices.size());
      for (size_t i = 0; i < m.indices.size(); i += 3) { const uint32_t a = m.indices[i], b = m.indices[i+1], c = m.indices[i+2]; if (a != b && b != c && a != c) ni.insert(ni.end(), {a, b, c}); }
      m.indices.swap(ni); }
    const size_t nt = m.triangleCount(); if (!nt) return 0;
    // Corners (triangle, position) of the same fan get joined across shared edges.
    std::vector<uint32_t> up(nt*3); std::iota(up.begin(), up.end(), 0);
    auto root = [&](uint32_t a) { while (up[a] != a) { up[a] = up[up[a]]; a = up[a]; } return a; };
    struct HE { uint64_t key; uint32_t corner; };   // corner = 3*t+k, the edge runs from there to the next corner
    std::vector<HE> he(nt*3);
    #pragma omp parallel for
    for (size_t t = 0; t < nt; t++) for (int k = 0; k < 3; k++) he[3*t+k] = {ukey(m.indices[3*t+k], m.indices[3*t+(k+1)%3]), (uint32_t)(3*t+k)};
    std::sort(he.begin(), he.end(), [](const HE& a, const HE& b) { return a.key != b.key ? a.key < b.key : a.corner < b.corner; });
    auto next = [](uint32_t c) { return c-c%3+(c+1)%3; };
    auto join = [&](uint32_t c0, uint32_t c1) {   // two half edges in opposite directions
        up[root(c0)] = root(next(c1)); up[root(next(c0))] = root(c1);
    };
    struct Around { float angle; uint32_t corner; bool forward; }; std::vector<Around> around;
    for (size_t i = 0; i < he.size();) {
        size_t j = i; while (j < he.size() && he[j].key == he[i].key) j++;
        const uint32_t a = (uint32_t)(he[i].key >> 32), b = (uint32_t)he[i].key;
        if (j-i == 2) {
            if ((m.indices[he[i].corner] == a) != (m.indices[he[i+1].corner] == a)) join(he[i].corner, he[i+1].corner);
        } else if (j-i > 2) {
            // Sort the triangles around the edge. For one running a -> b the outside lies at larger
            // angles, so the neighbour at the smaller angle closes the solid with it.
            const V3 axis = normalized(m.vertices[b]-m.vertices[a]);
            const V3 ref = normalized(cross(axis, std::fabs(axis.x) < .9f ? V3{1, 0, 0} : V3{0, 1, 0})), ref2 = cross(axis, ref);
            around.clear();
            for (size_t q = i; q < j; q++) {
                const uint32_t c = he[q].corner, tri = c/3, opp = m.indices[3*tri+((c%3)+2)%3];
                const V3 d = m.vertices[opp]-m.vertices[a];
                around.push_back({std::atan2(dot(d, ref2), dot(d, ref)), c, m.indices[c] == a});
            }
            std::sort(around.begin(), around.end(), [](const Around& x, const Around& y) { return x.angle < y.angle; });
            const size_t n = around.size(); std::vector<uint8_t> used(n, 0);
            for (size_t q = 0; q < n; q++) {
                if (!around[q].forward || used[q]) continue;
                const size_t o = (q+n-1)%n;
                if (!used[o] && !around[o].forward) { used[q] = used[o] = 1; join(around[q].corner, around[o].corner); }
            }
        }
        i = j;
    }
    // One vertex per fan. Where a vertex splits, its copies move a little into their own fan, so
    // that they stay apart in formats that merge equal points, such as STL.
    const size_t nv0 = m.vertices.size();
    std::vector<uint32_t> first(nv0, 0xffffffff), firstRoot(nv0, 0xffffffff); std::vector<uint8_t> split(nv0, 0);
    std::unordered_map<uint32_t, uint32_t> copyOf;   // root corner -> new vertex
    const bool hasC = m.colors.size() == nv0*3;
    std::vector<uint32_t> cornerVertex(nt*3);
    for (uint32_t c = 0; c < nt*3; c++) {
        const uint32_t v = m.indices[c], r = root(c);
        if (firstRoot[v] == 0xffffffff) { firstRoot[v] = r; cornerVertex[c] = v; continue; }
        if (firstRoot[v] == r) { cornerVertex[c] = v; continue; }
        auto [it, fresh] = copyOf.try_emplace(r, (uint32_t)m.vertices.size());
        if (fresh) { m.vertices.push_back(m.vertices[v]); if (hasC) m.colors.insert(m.colors.end(), {m.colors[v*3], m.colors[v*3+1], m.colors[v*3+2]}); split[v] = 1; }
        cornerVertex[c] = it->second;
    }
    const size_t added = m.vertices.size()-nv0;
    if (added) {
        std::vector<V3> sum(m.vertices.size(), V3{}); std::vector<uint32_t> cnt(m.vertices.size(), 0); std::vector<uint8_t> touched(m.vertices.size(), 0);
        for (size_t v = 0; v < nv0; v++) touched[v] = split[v];
        for (size_t v = nv0; v < m.vertices.size(); v++) touched[v] = 1;
        for (uint32_t c = 0; c < nt*3; c++) {
            const uint32_t v = cornerVertex[c]; if (!touched[v]) continue;
            sum[v] = sum[v]+m.vertices[cornerVertex[next(c)]]+m.vertices[cornerVertex[next(next(c))]]; cnt[v] += 2;
        }
        const std::vector<V3> old = m.vertices;
        for (size_t v = 0; v < old.size(); v++) if (touched[v] && cnt[v]) m.vertices[v] = old[v]+(sum[v]*(1.f/cnt[v])-old[v])*0.04f;
        m.indices.swap(cornerVertex);
    }
    computeNormals(m);
    return added;
}

size_t repairManifold(Mesh& m) {
    size_t added = 0;
    // Sheets that also hang together elsewhere cannot be separated at a knot. There only removing the
    // triangles helps, which leaves a small hole that the filling closes again. Few passes are enough.
    for (int pass = 0; pass < 12; pass++) {
        added += separateSheets(m);
        struct HE { uint64_t key; uint32_t tri; bool forward; };
        std::vector<HE> he; he.reserve(m.indices.size());
        for (uint32_t t = 0; t < m.triangleCount(); t++) for (int k = 0; k < 3; k++) { const uint32_t a = m.indices[3*t+k], b = m.indices[3*t+(k+1)%3]; he.push_back({ukey(a, b), t, a < b}); }
        std::sort(he.begin(), he.end(), [](const HE& x, const HE& y) { return x.key < y.key; });
        std::vector<uint8_t> bad(m.triangleCount(), 0); bool any = false;
        for (size_t i = 0; i < he.size();) {
            size_t j = i; while (j < he.size() && he[j].key == he[i].key) j++;
            if (j-i > 2 || (j-i == 2 && he[i].forward == he[i+1].forward)) { for (size_t q = i; q < j; q++) bad[he[q].tri] = 1; any = true; }
            i = j;
        }
        if (!any) break;
        deleteTriangles(m, bad);
    }
    computeNormals(m);
    return added;
}

std::vector<MeshHole> findHoles(const Mesh& m) {
    std::vector<MeshHole> out;
    for (auto& loop : boundaryLoops(m)) {
        if (loop.size() < 3) continue;
        MeshHole h; V3 lo{1e9f, 1e9f, 1e9f}, hi{-1e9f, -1e9f, -1e9f}, sum{};
        for (size_t i = 0; i < loop.size(); i++) {
            const V3 p = m.vertices[loop[i]]; sum = sum + p; h.perimeterMm += norm(p-m.vertices[loop[(i+1)%loop.size()]]);
            lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)}; hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
        }
        h.center = sum*(1.f/loop.size()); h.sizeMm = std::max({hi.x-lo.x, hi.y-lo.y, hi.z-lo.z}); h.loop = std::move(loop);
        out.push_back(std::move(h));
    }
    std::sort(out.begin(), out.end(), [](const MeshHole& a, const MeshHole& b) { return a.perimeterMm < b.perimeterMm; });
    return out;
}
int fillHoleList(Mesh& m, const std::vector<MeshHole>& holes, FillType type) {
    if (holes.empty()) return 0;
    Filler f(m, type); int n = 0;
    for (const MeshHole& h : holes) n += f.fill(h.loop);
    computeNormals(m);
    return n;
}
int fillHolesUpTo(Mesh& m, float maxPerimeter, FillType type) {
    std::vector<MeshHole> all = findHoles(m), use;
    // A slit can be very long and still no opening of the object: seams in a fused surface measure hundreds
    // of mm around but under a millimeter across. Its width is judged from the area it encloses.
    constexpr float kSlitWidthMm = 1.5f;
    for (MeshHole& h : all) {
        V3 twice{}; const size_t n = h.loop.size();
        for (size_t i = 0; i < n; i++) twice = twice + cross(m.vertices[h.loop[i]]-h.center, m.vertices[h.loop[(i+1)%n]]-h.center);
        const float width = h.perimeterMm > 0 ? norm(twice)/h.perimeterMm : 0;
        if (maxPerimeter < 0 || h.perimeterMm <= maxPerimeter || width < kSlitWidthMm) use.push_back(std::move(h));
    }
    return fillHoleList(m, use, type);
}
bool bridgeEdges(Mesh& m, uint32_t a0, uint32_t a1, uint32_t b0, uint32_t b1) {
    if (a0 == b0 && a1 == b1) return false;
    if (a0 == a1 || b0 == b1) return false;
    std::vector<uint32_t> tri;
    if (a1 == b0) tri = {b1, a1, a0};
    else if (b1 == a0) tri = {a1, b1, b0};
    else {
        // Quad a1 a0 b1 b0, cut along the shorter diagonal.
        if (a0 == b0 || a1 == b1) return false;
        if (norm(m.vertices[a1]-m.vertices[b1]) <= norm(m.vertices[a0]-m.vertices[b0])) tri = {a1, a0, b1, b1, b0, a1};
        else tri = {a0, b1, b0, b0, a1, a0};
    }
    m.indices.insert(m.indices.end(), tri.begin(), tri.end());
    computeNormals(m);
    return true;
}

void smoothSurface(Mesh& m, const SmoothOptions& o) {
    if (o.iterations <= 0 || m.vertices.empty()) return;
    const uint32_t n = (uint32_t)m.vertices.size(); Adjacency ad(m);
    std::vector<uint8_t> rim(n, 0);
    for (auto& loop : boundaryLoops(m)) for (uint32_t v : loop) rim[v] = 1;
    std::vector<float> weight(n, 1.f);
    if (o.keepEdges) {
        // A vertex on a sharp edge has neighbour faces with different directions: it moves less.
        if (m.normals.size() != m.vertices.size()) computeNormals(m);
        #pragma omp parallel for schedule(dynamic, 4096)
        for (uint32_t i = 0; i < n; i++) {
            float minCos = 1; for (uint32_t e = ad.off[i]; e < ad.off[i+1]; e++) minCos = std::min(minCos, dot(m.normals[i], m.normals[ad.adj[e]]));
            const float t = std::clamp((minCos-0.75f)/(0.95f-0.75f), 0.f, 1.f); weight[i] = t*t*(3-2*t);
        }
    }
    std::vector<V3> tmp(n);
    auto pass = [&](float lambda) {
        #pragma omp parallel for
        for (uint32_t i = 0; i < n; i++) {
            const uint32_t c = ad.off[i+1]-ad.off[i];
            if (!c || rim[i]) { tmp[i] = m.vertices[i]; continue; }
            V3 s{}; for (uint32_t e = ad.off[i]; e < ad.off[i+1]; e++) s = s + m.vertices[ad.adj[e]];
            tmp[i] = m.vertices[i] + (s*(1.f/c) - m.vertices[i])*(lambda*weight[i]);
        }
        m.vertices.swap(tmp);
    };
    for (int it = 0; it < o.iterations; it++) { pass(0.5f); pass(-0.53f); }
    computeNormals(m);
}
void smoothRims(Mesh& m, Level level) {
    const int iterations = smoothIterationsFor(level); if (!iterations) return;
    const auto loops = boundaryLoops(m);
    std::vector<V3> tmp;
    auto pass = [&](const std::vector<uint32_t>& loop, float lambda) {
        const size_t n = loop.size(); tmp.resize(n);
        for (size_t i = 0; i < n; i++) { const V3 p = m.vertices[loop[i]]; tmp[i] = p + ((m.vertices[loop[(i+n-1)%n]]+m.vertices[loop[(i+1)%n]])*0.5f-p)*lambda; }
        for (size_t i = 0; i < n; i++) m.vertices[loop[i]] = tmp[i];
    };
    for (auto& loop : loops) if (loop.size() >= 6) for (int it = 0; it < iterations; it++) { pass(loop, 0.5f); pass(loop, -0.53f); }
    computeNormals(m);
}
size_t removeSpikes(Mesh& m, Level level) {
    if (level == Level::None || m.indices.empty()) return 0;
    // Angle sum around a vertex: flat 360 degrees, cube corner 270. Below these limits only a thin
    // needle or pit remains, in either direction.
    const float maxSum = (level == Level::Low ? 0.6f : level == Level::Medium ? 0.9f : 1.2f)*3.14159f;
    const uint32_t n = (uint32_t)m.vertices.size(); Adjacency ad(m); std::vector<uint8_t> rim(n, 0), moved(n, 0);
    for (auto& loop : boundaryLoops(m)) for (uint32_t v : loop) rim[v] = 1;
    for (int pass = 0; pass < 6; pass++) {
        std::vector<float> corner(m.indices.size());
        #pragma omp parallel for
        for (size_t i = 0; i < m.indices.size(); i += 3) for (int k = 0; k < 3; k++)
            corner[i+k] = angleAt(m.vertices[m.indices[i+k]], m.vertices[m.indices[i+(k+1)%3]], m.vertices[m.indices[i+(k+2)%3]]);
        std::vector<float> sum(n, 0.f);
        for (size_t i = 0; i < corner.size(); i++) sum[m.indices[i]] += corner[i];
        size_t changed = 0;
        for (uint32_t i = 0; i < n; i++) {
            if (rim[i] || ad.off[i+1] == ad.off[i] || sum[i] >= maxSum) continue;
            V3 s{}; for (uint32_t e = ad.off[i]; e < ad.off[i+1]; e++) s = s + m.vertices[ad.adj[e]];
            m.vertices[i] = s*(1.f/(ad.off[i+1]-ad.off[i])); moved[i] = 1; changed++;
        }
        if (!changed) break;
    }
    computeNormals(m);
    size_t total = 0; for (uint8_t f : moved) total += f;
    return total;
}
void triangleParts(const Mesh& m, std::vector<uint32_t>& part) {
    const uint32_t nv = (uint32_t)m.vertices.size(); std::vector<uint32_t> p(nv); std::iota(p.begin(), p.end(), 0);
    auto root = [&](uint32_t a) { while (p[a] != a) { p[a] = p[p[a]]; a = p[a]; } return a; };
    for (size_t i = 0; i < m.indices.size(); i += 3) { const uint32_t a = root(m.indices[i]); p[root(m.indices[i+1])] = a; p[root(m.indices[i+2])] = a; }
    part.resize(m.triangleCount());
    for (size_t t = 0; t < part.size(); t++) part[t] = root(m.indices[3*t]);
}
size_t deleteTriangles(Mesh& m, const std::vector<uint8_t>& marked) {
    const size_t nt = m.triangleCount(); if (marked.size() != nt) return 0;
    std::vector<uint32_t> remap(m.vertices.size(), 0xffffffff), ni; ni.reserve(m.indices.size());
    std::vector<V3> nv, nn; std::vector<uint8_t> nc; const bool hasN = m.normals.size() == m.vertices.size(), hasC = m.colors.size() == m.vertices.size()*3;
    size_t removed = 0;
    for (size_t t = 0; t < nt; t++) {
        if (marked[t]) { removed++; continue; }
        for (int k = 0; k < 3; k++) {
            const uint32_t v = m.indices[3*t+k];
            if (remap[v] == 0xffffffff) {
                remap[v] = (uint32_t)nv.size(); nv.push_back(m.vertices[v]);
                if (hasN) nn.push_back(m.normals[v]);
                if (hasC) nc.insert(nc.end(), {m.colors[v*3], m.colors[v*3+1], m.colors[v*3+2]});
            }
            ni.push_back(remap[v]);
        }
    }
    m.vertices.swap(nv); m.indices.swap(ni); m.normals.swap(nn); m.colors.swap(nc);
    return removed;
}
int removeSmallParts(Mesh& m, float minShare) {
    if (m.indices.empty() || minShare <= 0) return 0;
    std::vector<uint32_t> part; triangleParts(m, part);
    std::unordered_map<uint32_t, uint32_t> count; for (uint32_t p : part) count[p]++;
    uint32_t largest = 0; for (auto& c : count) largest = std::max(largest, c.second);
    const float limit = largest*minShare; int removed = 0; for (auto& c : count) removed += c.second < limit;
    if (!removed) return 0;
    std::vector<uint8_t> mark(part.size()); for (size_t t = 0; t < part.size(); t++) mark[t] = count[part[t]] < limit;
    deleteTriangles(m, mark);
    return removed;
}
void rotateMesh(Mesh& m, float degX, float degY, float degZ) {
    if (m.vertices.empty()) return;
    V3 lo, hi; meshBounds(m, lo, hi); const V3 c = (lo+hi)*0.5f;
    const double r[6] = {degX*M_PI/180, degY*M_PI/180, degZ*M_PI/180, 0, 0, 0};
    const double x[6] = {r[0], 0, 0, 0, 0, 0}, y[6] = {0, r[1], 0, 0, 0, 0}, z[6] = {0, 0, r[2], 0, 0, 0};
    const Pose P = mul(expTwist(z), mul(expTwist(y), expTwist(x)));
    for (V3& p : m.vertices) p = P.rot(p-c)+c;
    for (V3& n : m.normals) n = P.rot(n);
}
void markBelowPlane(const Mesh& m, const CutPlane& plane, std::vector<uint8_t>& marked) {
    marked.resize(m.triangleCount());
    #pragma omp parallel for
    for (size_t t = 0; t < marked.size(); t++) {
        const V3 c = (m.vertices[m.indices[3*t]]+m.vertices[m.indices[3*t+1]]+m.vertices[m.indices[3*t+2]])*(1.f/3);
        marked[t] = plane.side(c) < 0;
    }
}
bool planeFromPoints(V3 a, V3 b, V3 c, CutPlane& out) {
    const V3 n = cross(b-a, c-a); if (norm(n) < 1e-6f) return false;
    out.normal = normalized(n); out.d = dot(out.normal, a); return true;
}
bool largestPlane(const Mesh& m, CutPlane& out, float tol) {
    const size_t nt = m.triangleCount(); if (!nt) return false;
    std::vector<float> area(nt);
    for (size_t t = 0; t < nt; t++) { area[t] = triArea(m.vertices[m.indices[3*t]], m.vertices[m.indices[3*t+1]], m.vertices[m.indices[3*t+2]]); }
    std::mt19937 g(7); std::uniform_int_distribution<size_t> pick(0, nt-1); float best = 0; const size_t stride = std::max<size_t>(1, nt/40000);
    for (int it = 0; it < 300; it++) {
        const size_t t0 = pick(g); CutPlane pl;
        if (!planeFromPoints(m.vertices[m.indices[3*t0]], m.vertices[m.indices[3*t0+1]], m.vertices[m.indices[3*t0+2]], pl)) continue;
        float sum = 0;
        #pragma omp parallel for reduction(+:sum)
        for (size_t t = 0; t < nt; t += stride) if (std::fabs(pl.side(m.vertices[m.indices[3*t]])) < tol) sum += area[t];
        if (sum > best) { best = sum; out = pl; }
    }
    return best > 0;
}

void MeshHistory::push(const Mesh& before) {
    undo_.push_back(before); redo_.clear();
    while (undo_.size() > 1 && bytes() > maxBytes) undo_.erase(undo_.begin());
}
size_t MeshHistory::bytes() const {
    size_t b = 0; for (const Mesh& m : undo_) b += m.vertices.size()*24 + m.indices.size()*4 + m.colors.size();
    return b;
}
bool MeshHistory::undo(Mesh& cur) { if (undo_.empty()) return false; redo_.push_back(std::move(cur)); cur = std::move(undo_.back()); undo_.pop_back(); return true; }
bool MeshHistory::redo(Mesh& cur) { if (redo_.empty()) return false; undo_.push_back(std::move(cur)); cur = std::move(redo_.back()); redo_.pop_back(); return true; }

OneClickReport runOneClick(Mesh& m, const OneClickConfig& c, const std::function<void(const char*, float)>& progress) {
    OneClickReport r; r.trianglesBefore = m.triangleCount();
    auto step = [&](const char* s, float f) { if (progress) progress(s, f); };
    repairManifold(m);
    step("Kleine Teile entfernen", 0.f); r.partsRemoved = removeSmallParts(m, c.minPartShare);
    step("Spitzen entfernen", .12f); r.spikesMoved = removeSpikes(m, c.spikes);
    step("Glätten", .25f); { SmoothOptions so; so.iterations = smoothIterationsFor(c.smooth); so.keepEdges = c.keepEdges; smoothSurface(m, so); }
    step("Ränder glätten", .4f); smoothRims(m, c.rimSmooth);
    step("Löcher füllen", .5f);
    if (c.fillHoles) r.holesClosed = fillHolesUpTo(m, c.maxHolePerimeterMm, c.fillType);
    r.holesLeft = (int)findHoles(m).size();
    step("Vereinfachen", .85f);
    if (c.keepTriangles < 0.999f) simplifyMesh(m, (size_t)(m.triangleCount()*std::max(0.001f, c.keepTriangles)));
    computeNormals(m);
    step("Fertig", 1.f); r.trianglesAfter = m.triangleCount();
    return r;
}
bool saveOneClickConfig(const OneClickConfig& c, const std::string& path) {
    std::error_code ec; std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
    std::ofstream f(path); if (!f) return false;
    f << "minPartShare " << c.minPartShare << "\nspikes " << (int)c.spikes << "\nsmooth " << (int)c.smooth << "\nkeepEdges " << c.keepEdges
      << "\nrimSmooth " << (int)c.rimSmooth << "\nfillHoles " << c.fillHoles << "\nfillType " << (int)c.fillType
      << "\nmaxHolePerimeterMm " << c.maxHolePerimeterMm << "\nkeepTriangles " << c.keepTriangles << "\n";
    return (bool)f;
}
bool loadOneClickConfig(const std::string& path, OneClickConfig& c) {
    std::ifstream f(path); if (!f) return false;
    std::string k; float v;
    auto level = [](float x) { return (Level)std::clamp((int)x, 0, 3); };
    while (f >> k >> v) {
        if (k == "minPartShare") c.minPartShare = std::clamp(v, 0.f, 1.f);
        else if (k == "spikes") c.spikes = level(v);
        else if (k == "smooth") c.smooth = level(v);
        else if (k == "keepEdges") c.keepEdges = v != 0;
        else if (k == "rimSmooth") c.rimSmooth = level(v);
        else if (k == "fillHoles") c.fillHoles = v != 0;
        else if (k == "fillType") c.fillType = (FillType)std::clamp((int)v, 0, 2);
        else if (k == "maxHolePerimeterMm") c.maxHolePerimeterMm = v;
        else if (k == "keepTriangles") c.keepTriangles = std::clamp(v, 0.001f, 1.f);
    }
    return true;
}

void weldVertices(Mesh& m) {
    struct H { size_t operator()(const std::array<uint32_t, 3>& a) const { return (size_t)a[0]*73856093u ^ (size_t)a[1]*19349663u ^ (size_t)a[2]*83492791u; } };
    std::unordered_map<std::array<uint32_t, 3>, uint32_t, H> id; id.reserve(m.vertices.size());
    std::vector<uint32_t> remap(m.vertices.size()); std::vector<V3> nv; std::vector<uint8_t> nc; const bool hasC = m.colors.size() == m.vertices.size()*3;
    for (size_t i = 0; i < m.vertices.size(); i++) {
        std::array<uint32_t, 3> k; std::memcpy(k.data(), &m.vertices[i], 12);
        auto [it, fresh] = id.try_emplace(k, (uint32_t)nv.size());
        if (fresh) { nv.push_back(m.vertices[i]); if (hasC) nc.insert(nc.end(), {m.colors[i*3], m.colors[i*3+1], m.colors[i*3+2]}); }
        remap[i] = it->second;
    }
    std::vector<uint32_t> ni; ni.reserve(m.indices.size());
    for (size_t t = 0; t+2 < m.indices.size(); t += 3) {
        const uint32_t a = remap[m.indices[t]], b = remap[m.indices[t+1]], c = remap[m.indices[t+2]];
        if (a != b && b != c && a != c) ni.insert(ni.end(), {a, b, c});
    }
    m.vertices.swap(nv); m.indices.swap(ni); m.colors.swap(nc); m.normals.clear();
}
static bool loadObj(const std::string& path, Mesh& out, std::string& error) {
    FILE* f = fopen(path.c_str(), "r"); if (!f) { error = "Datei nicht lesbar: " + path; return false; }
    out = Mesh{}; char line[512]; bool col = false;
    while (fgets(line, sizeof line, f)) {
        if (line[0] == 'v' && line[1] == ' ') {
            float v[6]; const int n = sscanf(line+2, "%f %f %f %f %f %f", v, v+1, v+2, v+3, v+4, v+5);
            if (n < 3) continue;
            out.vertices.push_back(V3{v[0], v[1], v[2]});
            if (n == 6) { col = true; for (int c = 0; c < 3; c++) out.colors.push_back((uint8_t)std::clamp(v[3+c]*255.f+.5f, 0.f, 255.f)); }
        } else if (line[0] == 'f' && line[1] == ' ') {
            uint32_t first = 0, prev = 0; int n = 0; char* s = line+2;
            while (*s) {
                while (*s == ' ' || *s == '\t') s++;
                if (*s == '\n' || *s == '\r' || !*s) break;
                char* e; long idx = strtol(s, &e, 10); if (e == s) break;
                const uint32_t v = idx < 0 ? (uint32_t)(out.vertices.size()+idx) : (uint32_t)(idx-1);
                while (*e && *e != ' ' && *e != '\t' && *e != '\n' && *e != '\r') e++;
                s = e;
                if (n == 0) first = v; else if (n >= 2) out.indices.insert(out.indices.end(), {first, prev, v});
                prev = v; n++;
            }
        }
    }
    fclose(f);
    if (!col || out.colors.size() != out.vertices.size()*3) out.colors.clear();
    for (uint32_t v : out.indices) if (v >= out.vertices.size()) { error = "OBJ-Datei ist beschädigt"; return false; }
    return true;
}
static bool loadPly(const std::string& path, Mesh& out, std::string& error) {
    FILE* f = fopen(path.c_str(), "rb"); if (!f) { error = "Datei nicht lesbar: " + path; return false; }
    char line[256]; size_t nv = 0, nf = 0; bool hasN = false, hasC = false, binary = false;
    while (fgets(line, sizeof line, f)) {
        if (!strncmp(line, "format binary_little_endian", 27)) binary = true;
        sscanf(line, "element vertex %zu", &nv); sscanf(line, "element face %zu", &nf);
        if (strstr(line, "property float nx")) hasN = true;
        if (strstr(line, "property uchar red")) hasC = true;
        if (!strncmp(line, "end_header", 10)) break;
    }
    if (!binary || !nv) { fclose(f); error = "Nur binäre PLY-Dateien aus VoxelForge werden unterstützt"; return false; }
    out = Mesh{}; out.vertices.resize(nv); if (hasC) out.colors.resize(nv*3);
    bool ok = true;
    for (size_t i = 0; i < nv && ok; i++) {
        float r[6]; ok = fread(r, 4, hasN ? 6 : 3, f) == (hasN ? 6u : 3u); out.vertices[i] = V3{r[0], r[1], r[2]};
        if (hasC && ok) ok = fread(&out.colors[i*3], 1, 3, f) == 3;
    }
    for (size_t i = 0; i < nf && ok; i++) {
        uint8_t c = 0; uint32_t t[3]; ok = fread(&c, 1, 1, f) == 1 && c == 3 && fread(t, 4, 3, f) == 3;
        if (ok) out.indices.insert(out.indices.end(), t, t+3);
    }
    fclose(f);
    if (!ok) error = "PLY-Datei ist abgeschnitten oder hat ein unbekanntes Format";
    return ok;
}
bool loadMeshFile(const std::string& path, Mesh& out, std::string& error) {
    std::string ext = std::filesystem::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    bool ok;
    if (ext == ".stl") ok = loadStl(path, out, error);
    else if (ext == ".obj") ok = loadObj(path, out, error);
    else if (ext == ".ply") ok = loadPly(path, out, error);
    else { error = "Unbekanntes Dateiformat: " + ext; return false; }
    if (!ok) return false;
    if (out.indices.empty()) { error = "Die Datei enthält kein Netz"; return false; }
    weldVertices(out); computeNormals(out);
    return true;
}
