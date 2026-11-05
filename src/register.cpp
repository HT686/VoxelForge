#include "register.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <unordered_map>
#include <omp.h>
#include "markers.hpp"
#include "pointgrid.hpp"

namespace {
struct Cloud { std::vector<V3> p, n; float spacing = 1; };
// One point per grid cell, averaged, with the surface direction.
Cloud sampleMesh(const Mesh& m, float d) {
    struct Cell { V3 p, n; uint32_t c = 0; }; std::unordered_map<uint64_t, Cell> grid; grid.reserve(m.vertices.size()/4+16);
    const bool hasN = m.normals.size() == m.vertices.size(); Mesh tmp; const Mesh* src = &m;
    if (!hasN) { tmp = m; computeNormals(tmp); src = &tmp; }
    const float inv = 1/d;
    for (size_t i = 0; i < src->vertices.size(); i++) {
        const V3 v = src->vertices[i];
        const uint64_t k = ((uint64_t)((int)std::floor(v.x*inv)+(1<<20)) << 42) | ((uint64_t)((int)std::floor(v.y*inv)+(1<<20)) << 21) | (uint64_t)((int)std::floor(v.z*inv)+(1<<20));
        Cell& c = grid[k]; c.p = c.p+v; c.n = c.n+src->normals[i]; c.c++;
    }
    std::vector<std::pair<uint64_t, Cell>> sorted(grid.begin(), grid.end());
    std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) { return a.first < b.first; });   // the same result on every run
    Cloud out; out.spacing = d; out.p.reserve(sorted.size()); out.n.reserve(sorted.size());
    for (auto& e : sorted) { const V3 n = normalized(e.second.n); if (norm(n) < .5f) continue; out.p.push_back(e.second.p*(1.f/e.second.c)); out.n.push_back(n); }
    return out;
}
constexpr int kBins = 11, kDim = 33;
using Feature = std::array<float, kDim>;
// Fast Point Feature Histograms (Rusu 2009): how the surface turns around each point.
std::vector<Feature> computeFeatures(const Cloud& c, float radius) {
    const size_t n = c.p.size(); PointGrid grid(c.p, radius); std::vector<Feature> spfh(n), out(n);
    std::vector<std::vector<uint32_t>> nb(n);
    #pragma omp parallel for schedule(dynamic, 64)
    for (size_t i = 0; i < n; i++) {
        Feature h; h.fill(0); int cnt = 0;
        grid.within(c.p[i], radius, [&](uint32_t j, float d2) {
            if (j == i || d2 < 1e-10f) return;
            nb[i].push_back(j);
            V3 ps = c.p[i], ns = c.n[i], pt = c.p[j], nt = c.n[j]; V3 dv = pt-ps; const float dist = std::sqrt(d2);
            if (dot(ns, dv) < dot(nt, dv)*-1.f) { std::swap(ps, pt); std::swap(ns, nt); dv = dv*-1.f; }
            const V3 d = dv*(1/dist), v = normalized(cross(d, ns)), w = cross(ns, v);
            const float a = dot(v, nt), phi = dot(ns, d), th = std::atan2(dot(w, nt), dot(ns, nt));
            auto bin = [](float t) { return std::clamp((int)(t*kBins), 0, kBins-1); };
            h[bin((a+1)*.5f)] += 1; h[kBins+bin((phi+1)*.5f)] += 1; h[2*kBins+bin((th+3.14159f)/6.28318f)] += 1; cnt++;
        });
        if (cnt) for (float& x : h) x *= 100.f/cnt;
        spfh[i] = h;
    }
    #pragma omp parallel for schedule(dynamic, 64)
    for (size_t i = 0; i < n; i++) {
        Feature h = spfh[i]; float wSum = 0; Feature acc; acc.fill(0);
        for (uint32_t j : nb[i]) { const float w = 1/std::max(norm(c.p[i]-c.p[j]), 1e-4f); wSum += w; for (int k = 0; k < kDim; k++) acc[k] += w*spfh[j][k]; }
        if (wSum > 0) for (int k = 0; k < kDim; k++) h[k] += acc[k]/wSum;
        out[i] = h;
    }
    return out;
}
struct Match { uint32_t s, d; };
std::vector<Match> matchFeatures(const std::vector<Feature>& a, const std::vector<Feature>& b) {
    std::vector<int> ab(a.size(), -1), ba(b.size(), -1);
    auto nearest = [](const std::vector<Feature>& x, const std::vector<Feature>& y, std::vector<int>& out) {
        #pragma omp parallel for schedule(dynamic, 64)
        for (size_t i = 0; i < x.size(); i++) {
            float best = 1e30f; int bi = -1;
            for (size_t j = 0; j < y.size(); j++) { float s = 0; for (int k = 0; k < kDim; k++) { const float e = x[i][k]-y[j][k]; s += e*e; } if (s < best) { best = s; bi = (int)j; } }
            out[i] = bi;
        }
    };
    nearest(a, b, ab); nearest(b, a, ba);
    std::vector<Match> m; m.reserve(a.size()+b.size());
    for (size_t i = 0; i < a.size(); i++) if (ab[i] >= 0) m.push_back({(uint32_t)i, (uint32_t)ab[i]});
    for (size_t j = 0; j < b.size(); j++) if (ba[j] >= 0 && ab[ba[j]] != (int)j) m.push_back({(uint32_t)ba[j], (uint32_t)j});
    return m;
}
inline V3 applyP(const Pose& P, V3 p) { return P.apply(p); }
// Point-to-plane ICP with shrinking search distance.
Pose icp(const Cloud& src, const Cloud& dst, const PointGrid& grid, Pose T, float dist, float endDist, int iterations) {
    const float shrink = iterations > 1 ? std::pow(endDist/dist, 1.f/(iterations-1)) : 1.f;
    for (int it = 0; it < iterations; it++, dist *= shrink) {
        double A[36] = {0}, b[6] = {0}; int used = 0;
        #pragma omp parallel
        {
            double la[36] = {0}, lb[6] = {0}; int lu = 0;
            #pragma omp for schedule(static) nowait
            for (size_t i = 0; i < src.p.size(); i++) {
                const V3 p = T.apply(src.p[i]), n = T.rot(src.n[i]);
                const long j = grid.nearest(p, dist); if (j < 0) continue;
                const V3 q = dst.p[j], nq = dst.n[j]; if (dot(n, nq) < 0.5f) continue;
                const V3 c = cross(p, nq); const double row[6] = {c.x, c.y, c.z, nq.x, nq.y, nq.z}; const double e = dot(nq, q-p);
                for (int r = 0; r < 6; r++) { lb[r] += row[r]*e; for (int k = 0; k < 6; k++) la[r*6+k] += row[r]*row[k]; }
                lu++;
            }
            #pragma omp critical
            { for (int r = 0; r < 36; r++) A[r] += la[r]; for (int r = 0; r < 6; r++) b[r] += lb[r]; used += lu; }
        }
        if (used < 12) break;
        for (int r = 0; r < 6; r++) A[r*6+r] += 1e-6*used;
        double x[6]; if (!solve6(A, b, x)) break;
        T = mul(expTwist(x), T);
    }
    return T;
}
RegisterResult evaluate(const Cloud& src, const Cloud& dst, const PointGrid& grid, const Pose& T) {
    const float tol = src.spacing*1.2f, near = src.spacing*4; int in = 0, conflict = 0; double sum = 0;
    #pragma omp parallel for reduction(+:in,conflict,sum) schedule(static)
    for (size_t i = 0; i < src.p.size(); i++) {
        const V3 p = T.apply(src.p[i]), n = T.rot(src.n[i]); float d2;
        const long j = grid.nearest(p, near, &d2); if (j < 0) continue;
        const float along = dot(dst.n[j], p-dst.p[j]);
        if (std::sqrt(d2) < tol && dot(n, dst.n[j]) > 0.6f) { in++; sum += along*along; }
        // Well in front of a surface that was seen, or facing the opposite way right on top of it: impossible.
        else if (along > tol*1.5f || (std::sqrt(d2) < tol && dot(n, dst.n[j]) < -0.3f)) conflict++;
    }
    RegisterResult r; r.srcToDst = T; const float smaller = (float)std::min(src.p.size(), dst.p.size());
    r.overlap = in/std::max(1.f, smaller); r.conflict = conflict/std::max(1.f, (float)src.p.size()); r.rmsMm = in ? (float)std::sqrt(sum/in) : 1e9f;
    r.score = r.overlap-2*r.conflict;
    r.ok = r.overlap > 0.10f && r.conflict < 0.25f;
    return r;
}
float sampleSpacing(const Mesh& a, const Mesh& b) {
    V3 l0, h0, l1, h1; meshBounds(a, l0, h0); meshBounds(b, l1, h1);
    return std::max(0.2f, std::max(norm(h0-l0), norm(h1-l1))/70.f);
}
}

std::vector<RegisterResult> registerAuto(const Mesh& srcMesh, const Mesh& dstMesh, const std::function<void(float)>& progress) {
    std::vector<RegisterResult> results;
    if (srcMesh.vertices.size() < 32 || dstMesh.vertices.size() < 32) return results;
    const float d = sampleSpacing(srcMesh, dstMesh);
    const Cloud src = sampleMesh(srcMesh, d), dst = sampleMesh(dstMesh, d); if (src.p.size() < 16 || dst.p.size() < 16) return results;
    const float featureRadius = d*5;
    if (progress) progress(0.05f);
    const auto fa = computeFeatures(src, featureRadius), fb = computeFeatures(dst, featureRadius);
    if (progress) progress(0.2f);
    const std::vector<Match> matches = matchFeatures(fa, fb); if (matches.size() < 8) return results;
    if (progress) progress(0.35f);
    // Random consensus. The two distance checks throw out nearly all wrong triples without solving anything.
    struct Hypo { Pose T; int votes; };
    const int threads = omp_get_max_threads(); std::vector<std::vector<Hypo>> found(threads);
    const float voteTol = d*1.5f; const int tries = 400000;
    #pragma omp parallel
    {
        const int tid = omp_get_thread_num(); std::mt19937 g(1234+tid); std::uniform_int_distribution<size_t> pick(0, matches.size()-1); auto& out = found[tid];
        #pragma omp for schedule(static)
        for (int t = 0; t < tries; t++) {
            const Match a = matches[pick(g)], b = matches[pick(g)], c = matches[pick(g)];
            auto similar = [&](Match x, Match y) { const float s = norm(src.p[x.s]-src.p[y.s]), e = norm(dst.p[x.d]-dst.p[y.d]); return s > d*4 && std::min(s, e) > 0.92f*std::max(s, e); };
            if (!similar(a, b) || !similar(b, c) || !similar(a, c)) continue;
            auto normalsAgree = [&](Match x, Match y) { return std::fabs(dot(src.n[x.s], src.n[y.s])-dot(dst.n[x.d], dst.n[y.d])) < 0.25f; };
            if (!normalsAgree(a, b) || !normalsAgree(b, c) || !normalsAgree(a, c)) continue;
            std::vector<V3> pa{src.p[a.s], src.p[b.s], src.p[c.s]}, pb{dst.p[a.d], dst.p[b.d], dst.p[c.d]};
            Pose T; if (!alignRigid(pa, pb, T)) continue;
            int votes = 0; const size_t stride = std::max<size_t>(1, matches.size()/800);
            for (size_t m = 0; m < matches.size(); m += stride) { const V3 v = T.apply(src.p[matches[m].s])-dst.p[matches[m].d]; votes += dot(v, v) < voteTol*voteTol; }
            if (votes < 6) continue;
            out.push_back({T, votes});
            if (out.size() > 4000) { std::nth_element(out.begin(), out.begin()+1000, out.end(), [](const Hypo& x, const Hypo& y) { return x.votes > y.votes; }); out.resize(1000); }
        }
    }
    std::vector<Hypo> all; for (auto& f : found) all.insert(all.end(), f.begin(), f.end());
    if (all.empty()) return results;
    std::sort(all.begin(), all.end(), [](const Hypo& x, const Hypo& y) { return x.votes != y.votes ? x.votes > y.votes : x.T.t.x < y.T.t.x; });
    if (progress) progress(0.5f);
    // Different placements only, then measure each one properly.
    V3 center{}; for (const V3& p : src.p) center = center+p; center = center*(1.f/src.p.size());
    auto differs = [&](const Pose& x, const Pose& y) {
        const V3 ax{1, 0, 0}, ay{0, 1, 0};
        return norm(x.apply(center)-y.apply(center)) > d*3 || norm(x.rot(ax)-y.rot(ax)) > 0.15f || norm(x.rot(ay)-y.rot(ay)) > 0.15f;
    };
    std::vector<Pose> starts;
    for (const Hypo& h : all) { bool nw = true; for (const Pose& s : starts) if (!differs(h.T, s)) { nw = false; break; } if (nw) starts.push_back(h.T); if (starts.size() >= 96) break; }
    PointGrid gridD(dst.p, d*2);
    std::vector<RegisterResult> tried(starts.size());
    for (size_t i = 0; i < starts.size(); i++) {
        tried[i] = evaluate(src, dst, gridD, icp(src, dst, gridD, starts[i], d*4, d*1.2f, 16));
        if (progress) progress(0.5f+0.45f*(i+1)/starts.size());
    }
    std::sort(tried.begin(), tried.end(), [](const RegisterResult& a, const RegisterResult& b) { return a.score > b.score; });
    for (const RegisterResult& r : tried) {
        if (!r.ok) continue;
        bool nw = true; for (const RegisterResult& k : results) if (!differs(r.srcToDst, k.srcToDst)) { nw = false; break; }
        if (nw) results.push_back(r);
        if (results.size() >= 16) break;
    }
    // The best ones get their final fine alignment, and only then does the remaining distance count.
    // Symmetric shapes fit well in the wrong place too, but never as tightly as in the right one.
    { std::vector<RegisterResult> fine; for (const RegisterResult& r : results) { const RegisterResult f = refineRegistration(srcMesh, dstMesh, r.srcToDst, d*1.5f); if (f.ok) fine.push_back(f); } results.swap(fine); }
    if (results.empty()) return results;
    // The tightest fit sets the scale: it shows what these two surfaces can reach at all.
    float tightest = 1e9f; for (const RegisterResult& r : results) tightest = std::min(tightest, r.rmsMm);
    for (RegisterResult& r : results) { const float t = tightest/std::max(1e-5f, r.rmsMm); r.score = r.overlap*t*t-2*r.conflict; }
    std::stable_sort(results.begin(), results.end(), [](const RegisterResult& a, const RegisterResult& b) { return a.score > b.score; });
    // Different starts often arrive at the same place.
    { std::vector<RegisterResult> distinct; for (const RegisterResult& r : results) { bool nw = true; for (const RegisterResult& k : distinct) if (!differs(r.srcToDst, k.srcToDst)) { nw = false; break; } if (nw) distinct.push_back(r); } results.swap(distinct); }
    if (results.size() > 8) results.resize(8);
    RegisterResult& best = results[0];
    best.confident = best.overlap > 0.15f && best.conflict < 0.025f && best.rmsMm < d*0.35f && (results.size() < 2 || best.score > 1.3f*results[1].score);
    if (progress) progress(1.f);
    return results;
}
RegisterResult refineRegistration(const Mesh& srcMesh, const Mesh& dstMesh, const Pose& start, float startDistance) {
    const float coarse = sampleSpacing(srcMesh, dstMesh), d = std::max(0.2f, coarse*0.4f);
    const Cloud src = sampleMesh(srcMesh, d), dst = sampleMesh(dstMesh, d);
    if (src.p.size() < 16 || dst.p.size() < 16) return {};
    PointGrid grid(dst.p, std::max(d*2, startDistance));
    const Pose T = icp(src, dst, grid, start, std::max(startDistance, d*2), d*1.2f, 24);
    RegisterResult r = evaluate(src, dst, grid, T);
    return r;
}
bool registerPointPairs(const std::vector<V3>& src, const std::vector<V3>& dst, Pose& out) {
    if (src.size() < 3 || src.size() != dst.size()) return false;
    // Not on one line: the triangle spanned by the pairs needs some area.
    float best = 0; for (size_t i = 1; i < src.size(); i++) for (size_t j = i+1; j < src.size(); j++) best = std::max(best, norm(cross(src[i]-src[0], src[j]-src[0])));
    if (best < 1e-3f) return false;
    return alignRigid(src, dst, out);
}
bool registerMarkers(const std::vector<V3>& src, const std::vector<V3>& dst, Pose& out, int& matched, float tol) {
    matched = 0; if (src.size() < 4 || dst.size() < 4) return false;
    // Each pair of source markers proposes pairs in the target with the same distance. A third fixes the pose.
    const int ns = (int)src.size(), nd = (int)dst.size(); int best = 0; Pose bestT;
    auto count = [&](const Pose& T, std::vector<V3>* a, std::vector<V3>* b) {
        int c = 0; for (const V3& p : src) { const V3 q = T.apply(p); float bd = tol*tol; const V3* hit = nullptr; for (const V3& t : dst) { const V3 e = t-q; const float d2 = dot(e, e); if (d2 < bd) { bd = d2; hit = &t; } } if (hit) { c++; if (a) { a->push_back(p); b->push_back(*hit); } } }
        return c;
    };
    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < ns; i++) for (int j = i+1; j < ns; j++) for (int k = j+1; k < ns; k++) {
        const float ab = norm(src[i]-src[j]), bc = norm(src[j]-src[k]), ca = norm(src[k]-src[i]);
        if (std::min({ab, bc, ca}) < 5*tol || norm(cross(src[j]-src[i], src[k]-src[i])) < ab*bc*0.2f) continue;
        for (int x = 0; x < nd; x++) for (int y = 0; y < nd; y++) {
            if (y == x || std::fabs(norm(dst[x]-dst[y])-ab) > tol) continue;
            for (int z = 0; z < nd; z++) {
                if (z == x || z == y || std::fabs(norm(dst[y]-dst[z])-bc) > tol || std::fabs(norm(dst[z]-dst[x])-ca) > tol) continue;
                Pose T; const std::vector<V3> pa{src[i], src[j], src[k]}, pb{dst[x], dst[y], dst[z]}; if (!alignRigid(pa, pb, T)) continue;
                const int c = count(T, nullptr, nullptr);
                #pragma omp critical
                if (c > best) { best = c; bestT = T; }
            }
        }
    }
    if (best < 4) return false;
    std::vector<V3> a, b; count(bestT, &a, &b); Pose f; out = alignRigid(a, b, f) ? f : bestT; matched = best;
    return true;
}
