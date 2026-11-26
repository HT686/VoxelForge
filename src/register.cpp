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
struct Cloud { std::vector<V3> p, n; float spacing = 1; Table table; };
// One point per grid cell and facing, averaged, with the surface direction. The facing keeps the two
// sides of a wall thinner than a cell apart: averaged together their normals cancel, and the wall,
// often the only part two scans share, would vanish.
Cloud sampleMesh(const Mesh& m, float d) {
    struct Cell { V3 p, n; uint32_t c = 0; }; std::unordered_map<uint64_t, Cell> grid; grid.reserve(m.vertices.size()/4+16);
    const bool hasN = m.normals.size() == m.vertices.size(); Mesh tmp; const Mesh* src = &m;
    if (!hasN) { tmp = m; computeNormals(tmp); src = &tmp; }
    const float inv = 1/d;
    for (size_t i = 0; i < src->vertices.size(); i++) {
        const V3 v = src->vertices[i], n = src->normals[i]; const V3 a{std::fabs(n.x), std::fabs(n.y), std::fabs(n.z)};
        const uint64_t facing = a.x >= a.y && a.x >= a.z ? (n.x < 0) : a.y >= a.z ? 2+(n.y < 0) : 4+(n.z < 0);
        const uint64_t k = ((uint64_t)((int)std::floor(v.x*inv)+(1<<19)) << 43) | ((uint64_t)((int)std::floor(v.y*inv)+(1<<19)) << 23) | ((uint64_t)((int)std::floor(v.z*inv)+(1<<19)) << 3) | facing;
        Cell& c = grid[k]; c.p = c.p+v; c.n = c.n+n; c.c++;
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
// Point-to-plane ICP with shrinking search distance. Each thread sums its own share, and the shares are added
// in a fixed order: the same input then always gives the same placement.
Pose icp(const Cloud& src, const Cloud& dst, const PointGrid& grid, Pose T, float dist, float endDist, int iterations) {
    const float shrink = iterations > 1 ? std::pow(endDist/dist, 1.f/(iterations-1)) : 1.f;
    const int threads = omp_get_max_threads(); std::vector<std::array<double, 43>> part(threads);
    for (int it = 0; it < iterations; it++, dist *= shrink) {
        for (auto& q : part) q.fill(0);
        #pragma omp parallel
        {
            std::array<double, 43>& q = part[omp_get_thread_num()];
            #pragma omp for schedule(static)
            for (size_t i = 0; i < src.p.size(); i++) {
                const V3 p = T.apply(src.p[i]), n = T.rot(src.n[i]);
                const long j = grid.nearest(p, dist); if (j < 0) continue;
                const V3 qd = dst.p[j], nq = dst.n[j]; if (dot(n, nq) < 0.5f) continue;
                const V3 c = cross(p, nq); const double row[6] = {c.x, c.y, c.z, nq.x, nq.y, nq.z}; const double e = dot(nq, qd-p);
                for (int r = 0; r < 6; r++) { q[36+r] += row[r]*e; for (int k = 0; k < 6; k++) q[r*6+k] += row[r]*row[k]; }
                q[42] += 1;
            }
        }
        double A[36] = {0}, b[6] = {0}; double used = 0;
        for (const auto& q : part) { for (int r = 0; r < 36; r++) A[r] += q[r]; for (int r = 0; r < 6; r++) b[r] += q[36+r]; used += q[42]; }
        if (used < 12) break;
        for (int r = 0; r < 6; r++) A[r*6+r] += 1e-6*used;
        double x[6]; if (!solve6(A, b, x)) break;
        T = mul(expTwist(x), T);
    }
    return T;
}
// Smallest over largest eigenvalue of a symmetric 6x6 matrix, by Jacobi rotations.
Pose inverse(const Pose& P) { Pose q; for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) q.R[r*3+c] = P.R[c*3+r]; q.t = q.rot(P.t)*-1.f; return q; }
double conditioning(double A[36]) {
    for (int sweep = 0; sweep < 30; sweep++) {
        double off = 0; for (int i = 0; i < 6; i++) for (int j = i+1; j < 6; j++) off += A[i*6+j]*A[i*6+j];
        if (off < 1e-18) break;
        for (int p = 0; p < 6; p++) for (int q = p+1; q < 6; q++) {
            if (std::fabs(A[p*6+q]) < 1e-30) continue;
            const double th = (A[q*6+q]-A[p*6+p])/(2*A[p*6+q]), t = (th >= 0 ? 1 : -1)/(std::fabs(th)+std::sqrt(th*th+1)), c = 1/std::sqrt(t*t+1), sn = t*c;
            for (int k = 0; k < 6; k++) { const double x = A[k*6+p], y = A[k*6+q]; A[k*6+p] = c*x-sn*y; A[k*6+q] = sn*x+c*y; }
            for (int k = 0; k < 6; k++) { const double x = A[p*6+k], y = A[q*6+k]; A[p*6+k] = c*x-sn*y; A[q*6+k] = sn*x+c*y; }
        }
    }
    double lo = 1e300, hi = 0; for (int i = 0; i < 6; i++) { lo = std::min(lo, A[i*6+i]); hi = std::max(hi, A[i*6+i]); }
    return hi > 0 ? std::max(0., lo)/hi : 0.;
}
struct OneWay { int in = 0, conflict = 0, total = 0; double sum = 0; double A[36] = {0}; };
// How the points of a lie on b after T. Every stride-th point. With the normal matrix of the matched points,
// rotation measured at the size of the overlap, to see whether the placement is pinned down in every direction.
OneWay oneWay(const Cloud& a, const Cloud& b, const PointGrid& grid, const Pose& T, float tol, size_t stride, V3 center, float radius, float conflictTol = 0) {
    if (conflictTol <= 0) conflictTol = tol*1.5f;
    const float belowTol = std::max(1.5f, a.spacing*3);
    // Thinner than this no wall is; noisier scans need more room. conflictTol grows with the noise, see evaluate.
    const float sheetTol = std::max(0.4f, conflictTol*0.25f);
    OneWay r; const float near = a.spacing*4; const float invR = 1/std::max(1.f, radius);
    // Per thread, added in a fixed order afterwards: the result does not depend on the scheduling.
    struct Part { int in = 0, conflict = 0, total = 0; double sum = 0; double A[36] = {0}; };
    std::vector<Part> parts(omp_get_max_threads());
    #pragma omp parallel
    {
        Part& q = parts[omp_get_thread_num()];
        #pragma omp for schedule(static)
        for (size_t i = 0; i < a.p.size(); i += stride) {
            q.total++;
            const V3 p = T.apply(a.p[i]), n = T.rot(a.n[i]); float d2;
            // Below the table the other scan stood on: impossible, whatever surface is near.
            if (b.table.known && dot(p, b.table.up) < b.table.height-belowTol) { q.conflict++; continue; }
            const long j = grid.nearest(p, near, &d2);
            if (j < 0) continue;
            const float along = dot(b.n[j], p-b.p[j]);
            if (std::sqrt(d2) < tol && dot(n, b.n[j]) > 0.6f) {
                q.in++; q.sum += along*along;
                const V3 c = cross(p-center, b.n[j])*invR; const double row[6] = {c.x, c.y, c.z, b.n[j].x, b.n[j].y, b.n[j].z};
                for (int r6 = 0; r6 < 6; r6++) for (int k = 0; k < 6; k++) q.A[r6*6+k] += row[r6]*row[k];
            }
            // Well in front of a surface that was seen: impossible. Facing the opposite way is the other side of a
            // wall, which is fine unless the two lie closer than any wall is thin.
            else if (dot(n, b.n[j]) < -0.3f) { if (std::fabs(along) < sheetTol && std::sqrt(d2) < tol) q.conflict++; }
            else if (along > conflictTol) q.conflict++;
        }
    }
    int in = 0, conflict = 0, total = 0; double sum = 0;
    for (const Part& q : parts) { in += q.in; conflict += q.conflict; total += q.total; sum += q.sum; for (int k = 0; k < 36; k++) r.A[k] += q.A[k]; }
    r.in = in; r.conflict = conflict; r.total = total; r.sum = sum;
    return r;
}
// Judged from both sides: a placement where src fits onto dst but dst pokes out in front of src is wrong too.
RegisterResult evaluate(const Cloud& src, const Cloud& dst, const PointGrid& gridD, const PointGrid& gridS, const Pose& T) {
    const float tol = src.spacing*1.2f;
    V3 lo{1e9f, 1e9f, 1e9f}, hi{-1e9f, -1e9f, -1e9f}; for (const V3& p : dst.p) { lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)}; hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)}; }
    const V3 center = (lo+hi)*0.5f; const float radius = norm(hi-lo)*0.5f;
    OneWay a = oneWay(src, dst, gridD, T, tol, 1, center, radius);
    // Real scans of the same spot differ by their noise. Only what lies well beyond that is a contradiction.
    const float spread = a.in ? 3*(float)std::sqrt(a.sum/a.in) : 0;
    if (spread > tol*1.5f) a = oneWay(src, dst, gridD, T, tol, 1, center, radius, spread);
    const Pose inv = inverse(T); const OneWay b = oneWay(dst, src, gridS, inv, tol, 1, inv.apply(center), radius, std::max(spread, tol*1.5f));
    RegisterResult r; r.srcToDst = T;
    r.overlap = std::max(a.in/std::max(1.f, (float)a.total), b.in/std::max(1.f, (float)b.total));
    r.conflict = std::max(a.conflict/std::max(1.f, (float)a.total), b.conflict/std::max(1.f, (float)b.total));
    r.rmsMm = a.in ? (float)std::sqrt(a.sum/a.in) : 1e9f;
    double A[36]; for (int k = 0; k < 36; k++) A[k] = a.A[k]/std::max(1, a.in);
    r.pinned = (float)std::sqrt(conditioning(A));
    r.score = r.overlap-2*r.conflict;
    r.ok = r.overlap > 0.10f && r.conflict < 0.25f;
    return r;
}
// Technical parts have few distinctive spots but clear main faces. The strongest directions of the surface,
// each found as the normal with the most others close to it.
std::vector<V3> mainDirections(const Cloud& c, int maxDirs) {
    std::vector<V3> n; const size_t stride = std::max<size_t>(1, c.n.size()/6000); for (size_t i = 0; i < c.n.size(); i += stride) n.push_back(c.n[i]);
    std::vector<uint8_t> used(n.size(), 0); std::vector<V3> out;
    for (int k = 0; k < maxDirs; k++) {
        const size_t cs = std::max<size_t>(1, n.size()/700); int best = 0; V3 bestDir;
        for (size_t i = 0; i < n.size(); i += cs) {
            if (used[i]) continue;
            int cnt = 0; for (size_t j = 0; j < n.size(); j++) cnt += !used[j] && dot(n[i], n[j]) > 0.97f;
            if (cnt > best) { best = cnt; bestDir = n[i]; }
        }
        if (best < (int)(n.size()*0.04)) break;
        V3 m{}; for (size_t j = 0; j < n.size(); j++) if (!used[j] && dot(bestDir, n[j]) > 0.97f) m = m+n[j];
        const V3 d = normalized(m); out.push_back(d);
        for (size_t j = 0; j < n.size(); j++) if (dot(d, n[j]) > 0.9f) used[j] = 1;
    }
    return out;
}
void frameOf(V3 a, V3 b, V3 e[3]) { e[0] = a; e[1] = normalized(b-a*dot(a, b)); e[2] = cross(e[0], e[1]); }
// Placements that put main faces onto main faces, the ends of the part flush in each direction.
Pose axisAngle(V3 axis, float a);
Pose turnOnto(V3 a, V3 b);
// Turntable scans: both stood on their table, so the object was only turned about the vertical, or turned
// over. Seen from above and from below the outline is the same, and the heights follow from the tables:
// table onto table, or turned over, table onto the top of the other scan.
std::vector<Pose> uprightStarts(const Cloud& src, const Cloud& dst) {
    std::vector<Pose> out; if (!src.table.known || !dst.table.known) return out;
    const V3 us = src.table.up, ud = dst.table.up;
    const V3 hx = normalized(cross(std::fabs(ud.z) < 0.9f ? V3{0, 0, 1} : V3{1, 0, 0}, ud)), hy = cross(ud, hx);
    auto pct = [](std::vector<float>& v, float q) { const size_t k = std::min(v.size()-1, (size_t)(v.size()*q)); std::nth_element(v.begin(), v.begin()+k, v.end()); return v[k]; };
    std::vector<float> h; for (const V3& p : dst.p) h.push_back(dot(p, ud));
    const float top = pct(h, 0.99f);
    auto centerOf = [&](const std::vector<V3>& pts, const Pose& R) { std::vector<float> x, y; for (const V3& p : pts) { const V3 q = R.rot(p); x.push_back(dot(q, hx)); y.push_back(dot(q, hy)); }
        const float x0 = pct(x, 0.02f), x1 = pct(x, 0.98f), y0 = pct(y, 0.02f), y1 = pct(y, 0.98f); return hx*((x0+x1)/2)+hy*((y0+y1)/2); };
    const V3 cd = centerOf(dst.p, Pose{});
    for (int flip = 0; flip < 2; flip++) {
        // Turned over: about a horizontal axis by half a turn.
        const Pose base = flip ? mul(axisAngle(hx, 3.14159265f), turnOnto(us, ud)) : turnOnto(us, ud);
        for (int k = 0; k < 72; k++) {
            Pose R = mul(axisAngle(ud, k*6.2831853f/72), base);
            const V3 cs = centerOf(src.p, R);
            // Height: the src table, now at R.rot(us)*height, lands on the dst table or, turned over, on the top.
            const float srcTableH = src.table.height*dot(R.rot(us), ud);   // signed height of the src table plane along ud after turning
            const float want = flip ? top : dst.table.height;
            R.t = (cd-cs)+ud*(want-srcTableH);
            out.push_back(R);
        }
    }
    return out;
}
std::vector<Pose> frameStarts(const Cloud& src, const Cloud& dst) {
    const std::vector<V3> ds = mainDirections(src, 5), dd = mainDirections(dst, 5); std::vector<Pose> rots;
    for (size_t i = 0; i < ds.size(); i++) for (size_t j = 0; j < ds.size(); j++) {
        const float cij = dot(ds[i], ds[j]); if (i == j || std::fabs(cij) > 0.87f) continue;
        V3 es[3]; frameOf(ds[i], ds[j], es);
        for (size_t k = 0; k < dd.size(); k++) for (size_t l = 0; l < dd.size(); l++) {
            if (k == l || std::fabs(dot(dd[k], dd[l])-cij) > 0.12f) continue;
            V3 ed[3]; frameOf(dd[k], dd[l], ed); Pose P;
            for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) { float v = 0; for (int q = 0; q < 3; q++) { const float a = q == 0 ? (r == 0 ? ed[0].x : r == 1 ? ed[0].y : ed[0].z) : q == 1 ? (r == 0 ? ed[1].x : r == 1 ? ed[1].y : ed[1].z) : (r == 0 ? ed[2].x : r == 1 ? ed[2].y : ed[2].z); const float b = q == 0 ? (c == 0 ? es[0].x : c == 1 ? es[0].y : es[0].z) : q == 1 ? (c == 0 ? es[1].x : c == 1 ? es[1].y : es[1].z) : (c == 0 ? es[2].x : c == 1 ? es[2].y : es[2].z); v += a*b; } P.R[r*3+c] = v; }
            bool dup = false; for (const Pose& q : rots) { float e = 0; for (int x = 0; x < 9; x++) e += std::fabs(q.R[x]-P.R[x]); if (e < 0.3f) { dup = true; break; } }
            if (!dup) rots.push_back(P);
        }
    }
    // Along the axes of the target: the start, the end or the middle of both extents meet. Percentiles, so that a stray point does not count.
    auto extent = [](std::vector<float> v, float& lo, float& hi) { std::sort(v.begin(), v.end()); lo = v[v.size()*2/100]; hi = v[v.size()*98/100]; };
    const V3 axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    std::vector<Pose> out;
    for (const Pose& R : rots) {
        // Axes of the target's own main faces where there are any, so that "flush" means flush to a face.
        V3 ax[3] = {axes[0], axes[1], axes[2]};
        if (dd.size() >= 2) { size_t b = 1; for (size_t q = 1; q < dd.size(); q++) if (std::fabs(dot(dd[0], dd[q])) < std::fabs(dot(dd[0], dd[b]))) b = q; frameOf(dd[0], dd[b], ax); }
        float sh[3][3];
        for (int a = 0; a < 3; a++) {
            std::vector<float> ps, pd; ps.reserve(src.p.size()); pd.reserve(dst.p.size());
            for (const V3& p : src.p) ps.push_back(dot(R.rot(p), ax[a]));
            for (const V3& p : dst.p) pd.push_back(dot(p, ax[a]));
            float s0, s1, d0, d1; extent(ps, s0, s1); extent(pd, d0, d1);
            sh[a][0] = d0-s0; sh[a][1] = d1-s1; sh[a][2] = (d0+d1-s0-s1)*0.5f;
        }
        for (int x = 0; x < 3; x++) for (int y = 0; y < 3; y++) for (int z = 0; z < 3; z++) { Pose P = R; P.t = ax[0]*sh[0][x]+ax[1]*sh[1][y]+ax[2]*sh[2][z]; out.push_back(P); }
    }
    return out;
}
float sampleSpacing(const Mesh& a, const Mesh& b) {
    V3 l0, h0, l1, h1; meshBounds(a, l0, h0); meshBounds(b, l1, h1);
    return std::max(0.2f, std::max(norm(h0-l0), norm(h1-l1))/70.f);
}
}

namespace {
Cloud sampleWithTable(const Mesh& m, float d, Table t) { Cloud c = sampleMesh(m, d); if (t.known) { t.up = normalized(t.up); c.table = t; } return c; }
struct Fine { Cloud src, dst; PointGrid gridD, gridS; float d;
    Fine(const Mesh& s, const Mesh& t, float d_, float reach, Table ts, Table td) : src(sampleWithTable(s, d_, ts)), dst(sampleWithTable(t, d_, td)), gridD(dst.p, std::max(d_*2, reach)), gridS(src.p, std::max(d_*2, reach)), d(d_) {} };
RegisterResult refineOn(const Fine& f, const Pose& start, float startDistance) {
    if (f.src.p.size() < 16 || f.dst.p.size() < 16) return {};
    const Pose T = icp(f.src, f.dst, f.gridD, start, std::max(startDistance, f.d*2), f.d*1.2f, 24);
    return evaluate(f.src, f.dst, f.gridD, f.gridS, T);
}
}

namespace {
// One search: surface features with random consensus, plus main faces onto main faces, each fine-aligned.
// scale: of the sampling distance, seed: of the random draws, so that a second search looks differently.
std::vector<RegisterResult> searchOnce(const Mesh& srcMesh, const Mesh& dstMesh, const std::function<void(float)>& progress, Table srcTable, Table dstTable, float scale, unsigned seed) {
    std::vector<RegisterResult> results;
    if (srcMesh.vertices.size() < 32 || dstMesh.vertices.size() < 32) return results;
    const float d = sampleSpacing(srcMesh, dstMesh)*scale;
    const Cloud src = sampleWithTable(srcMesh, d, srcTable), dst = sampleWithTable(dstMesh, d, dstTable); if (src.p.size() < 16 || dst.p.size() < 16) return results;
    const float featureRadius = d*5;
    if (progress) progress(0.05f);
    const auto fa = computeFeatures(src, featureRadius), fb = computeFeatures(dst, featureRadius);
    if (progress) progress(0.2f);
    const std::vector<Match> matches = matchFeatures(fa, fb);
    if (progress) progress(0.35f);
    // Random consensus. The two distance checks throw out nearly all wrong triples without solving anything.
    struct Hypo { Pose T; int votes; };
    const int threads = omp_get_max_threads(); std::vector<std::vector<Hypo>> found(threads);
    const float voteTol = d*1.5f; const int tries = matches.size() >= 8 ? 400000 : 0;
    #pragma omp parallel
    {
        const int tid = omp_get_thread_num(); std::mt19937 g(seed+tid); std::uniform_int_distribution<size_t> pick(0, matches.empty() ? 0 : matches.size()-1); auto& out = found[tid];
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
    std::sort(all.begin(), all.end(), [](const Hypo& x, const Hypo& y) { return x.votes != y.votes ? x.votes > y.votes : x.T.t.x < y.T.t.x; });
    if (progress) progress(0.45f);
    // Different placements only, then measure each one properly.
    V3 center{}; for (const V3& p : src.p) center = center+p; center = center*(1.f/src.p.size());
    auto differs = [&](const Pose& x, const Pose& y) {
        const V3 ax{1, 0, 0}, ay{0, 1, 0};
        return norm(x.apply(center)-y.apply(center)) > d*3 || norm(x.rot(ax)-y.rot(ax)) > 0.15f || norm(x.rot(ay)-y.rot(ay)) > 0.15f;
    };
    std::vector<Pose> starts;
    for (const Hypo& h : all) { bool nw = true; for (const Pose& s : starts) if (!differs(h.T, s)) { nw = false; break; } if (nw) starts.push_back(h.T); if (starts.size() >= 96) break; }
    PointGrid gridD(dst.p, d*2), gridS(src.p, d*2);
    // Main faces onto main faces, ranked by a quick look before any fine alignment.
    {
        const std::vector<Pose> fs = frameStarts(src, dst); std::vector<std::pair<int, size_t>> rank(fs.size());
        V3 lo{1e9f, 1e9f, 1e9f}, hi{-1e9f, -1e9f, -1e9f}; for (const V3& p : dst.p) { lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)}; hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)}; }
        const size_t stride = std::max<size_t>(1, src.p.size()/1500);
        for (size_t i = 0; i < fs.size(); i++) { const OneWay w = oneWay(src, dst, gridD, fs[i], d*2.5f, stride, (lo+hi)*0.5f, norm(hi-lo)*0.5f); rank[i] = {w.in-2*w.conflict, i}; }
        std::sort(rank.begin(), rank.end(), [](auto& x, auto& y) { return x.first != y.first ? x.first > y.first : x.second < y.second; });
        int added = 0; for (auto& r : rank) { const Pose& P = fs[r.second]; bool nw = true; for (const Pose& s : starts) if (!differs(P, s)) { nw = false; break; } if (nw) { starts.push_back(P); added++; } if (added >= 48) break; }
    }
    // Turned about the vertical or turned over on the table, ranked the same way.
    {
        const std::vector<Pose> us = uprightStarts(src, dst); std::vector<std::pair<int, size_t>> rank(us.size());
        V3 lo{1e9f, 1e9f, 1e9f}, hi{-1e9f, -1e9f, -1e9f}; for (const V3& p : dst.p) { lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)}; hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)}; }
        const size_t stride = std::max<size_t>(1, src.p.size()/1500);
        for (size_t i = 0; i < us.size(); i++) { const OneWay w = oneWay(src, dst, gridD, us[i], d*2.5f, stride, (lo+hi)*0.5f, norm(hi-lo)*0.5f); rank[i] = {w.in-2*w.conflict, i}; }
        std::sort(rank.begin(), rank.end(), [](auto& x, auto& y) { return x.first != y.first ? x.first > y.first : x.second < y.second; });
        int added = 0; for (auto& r : rank) { const Pose& P = us[r.second]; bool nw = true; for (const Pose& s : starts) if (!differs(P, s)) { nw = false; break; } if (nw) { starts.push_back(P); added++; } if (added >= 24) break; }
    }
    if (starts.empty()) return results;
    if (progress) progress(0.5f);
    std::vector<RegisterResult> tried(starts.size());
    for (size_t i = 0; i < starts.size(); i++) {
        tried[i] = evaluate(src, dst, gridD, gridS, icp(src, dst, gridD, starts[i], d*4, d*1.2f, 16));
        if (progress) progress(0.5f+0.35f*(i+1)/starts.size());
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
    {
        const Fine fine(srcMesh, dstMesh, std::max(0.2f, d*0.4f), d*1.5f, srcTable, dstTable); std::vector<RegisterResult> out;
        for (size_t i = 0; i < results.size(); i++) { const RegisterResult f = refineOn(fine, results[i].srcToDst, d*1.5f); if (f.ok) out.push_back(f); if (progress) progress(0.85f+0.15f*(i+1)/results.size()); }
        results.swap(out);
    }
    if (results.empty()) return results;
    // The tightest fit sets the scale: it shows what these two surfaces can reach at all.
    float tightest = 1e9f; for (const RegisterResult& r : results) tightest = std::min(tightest, r.rmsMm);
    for (RegisterResult& r : results) { const float t = tightest/std::max(1e-5f, r.rmsMm); r.score = r.overlap*t*t-2*r.conflict; }
    std::stable_sort(results.begin(), results.end(), [](const RegisterResult& a, const RegisterResult& b) { return a.score > b.score; });
    // Different starts often arrive at the same place.
    { std::vector<RegisterResult> distinct; for (const RegisterResult& r : results) { bool nw = true; for (const RegisterResult& k : distinct) if (!differs(r.srcToDst, k.srcToDst)) { nw = false; break; } if (nw) distinct.push_back(r); } results.swap(distinct); }
    if (results.size() > 8) results.resize(8);
    if (progress) progress(1.f);
    return results;
}
bool samePlace(const Pose& x, const Pose& y, V3 center, float d) {
    const V3 ax{1, 0, 0}, ay{0, 1, 0};
    return norm(x.apply(center)-y.apply(center)) <= d*3 && norm(x.rot(ax)-y.rot(ax)) <= 0.15f && norm(x.rot(ay)-y.rot(ay)) <= 0.15f;
}
// Ranks by overlap and contradictions, weighted by how tight each fit is against the tightest, and says whether the best is certain.
void rank(std::vector<RegisterResult>& results, float d) {
    if (results.empty()) return;
    float tightest = 1e9f; for (const RegisterResult& r : results) tightest = std::min(tightest, r.rmsMm);
    for (RegisterResult& r : results) { const float t = tightest/std::max(1e-5f, r.rmsMm); r.score = r.overlap*t*t-6*r.conflict; r.confident = false; }
    std::stable_sort(results.begin(), results.end(), [](const RegisterResult& a, const RegisterResult& b) { return a.score > b.score; });
    RegisterResult& b = results[0];
    b.confident = b.overlap > 0.15f && b.conflict < 0.025f && b.rmsMm < d*0.35f && b.pinned > kMinPinned && (results.size() < 2 || b.score > 1.3f*results[1].score);
}
}
std::vector<RegisterResult> registerAuto(const Mesh& srcMesh, const Mesh& dstMesh, const std::function<void(float)>& progress, Table srcTable, Table dstTable) {
    const float d = sampleSpacing(srcMesh, dstMesh);
    auto part = [&](float a, float b) { return std::function<void(float)>([&, a, b](float f) { if (progress) progress(a+(b-a)*f); }); };
    // Two searches that look differently: another sampling distance and other random draws. A placement only
    // one of them finds may be the right one, but it is never certain: a missed right placement often leaves
    // a wrong one that looks good on its own.
    std::vector<RegisterResult> first = searchOnce(srcMesh, dstMesh, part(0, 0.5f), srcTable, dstTable, 1.f, 1234);
    std::vector<RegisterResult> second = searchOnce(srcMesh, dstMesh, part(0.5f, 1.f), srcTable, dstTable, 1.3f, 98765);
    V3 lo, hi; meshBounds(srcMesh, lo, hi); const V3 center = (lo+hi)*0.5f;
    std::vector<RegisterResult> results = first; std::vector<uint8_t> both(first.size(), 0);
    for (const RegisterResult& r : second) {
        bool nw = true;
        for (size_t k = 0; k < results.size(); k++) if (samePlace(r.srcToDst, results[k].srcToDst, center, d)) { nw = false; both[k] = 1; if (r.score > results[k].score) results[k] = r; break; }
        if (nw) { results.push_back(r); both.push_back(0); }
    }
    for (size_t k = 0; k < results.size(); k++) results[k].agreed = both[k];
    rank(results, d);
    if (!results.empty() && !results[0].agreed) results[0].confident = false;
    if (results.size() > 8) results.resize(8);
    if (progress) progress(1.f);
    return results;
}
RegisterResult refineRegistration(const Mesh& srcMesh, const Mesh& dstMesh, const Pose& start, float startDistance, Table srcTable, Table dstTable) {
    const float coarse = sampleSpacing(srcMesh, dstMesh);
    return refineOn(Fine(srcMesh, dstMesh, std::max(0.2f, coarse*0.4f), startDistance, srcTable, dstTable), start, startDistance);
}
namespace {
Pose axisAngle(V3 axis, float a) {
    const V3 k = normalized(axis); const float c = std::cos(a), sn = std::sin(a), t = 1-c; Pose P;
    const float R[9] = {t*k.x*k.x+c, t*k.x*k.y-sn*k.z, t*k.x*k.z+sn*k.y, t*k.x*k.y+sn*k.z, t*k.y*k.y+c, t*k.y*k.z-sn*k.x, t*k.x*k.z-sn*k.y, t*k.y*k.z+sn*k.x, t*k.z*k.z+c};
    std::copy(R, R+9, P.R); return P;
}
// The smallest turn that brings direction a onto direction b.
Pose turnOnto(V3 a, V3 b) {
    a = normalized(a); b = normalized(b); const V3 c = cross(a, b); const float s = norm(c), co = dot(a, b);
    if (s < 1e-6f) { if (co > 0) return Pose{}; const V3 any = std::fabs(a.x) < 0.9f ? V3{1, 0, 0} : V3{0, 1, 0}; return axisAngle(cross(a, any), 3.14159265f); }
    return axisAngle(c, std::atan2(s, co));
}
}
std::vector<RegisterResult> registerFromPairs(const Mesh& srcMesh, const Mesh& dstMesh, const std::vector<V3>& sp, const std::vector<V3>& sn, const std::vector<V3>& dp, const std::vector<V3>& dn, Table srcTable, Table dstTable) {
    std::vector<RegisterResult> results; const size_t n = std::min(sp.size(), dp.size());
    if (!n || srcMesh.vertices.size() < 32 || dstMesh.vertices.size() < 32) return results;
    const float d = sampleSpacing(srcMesh, dstMesh);
    const Cloud src = sampleWithTable(srcMesh, d, srcTable), dst = sampleWithTable(dstMesh, d, dstTable); if (src.p.size() < 16 || dst.p.size() < 16) return results;
    std::vector<Pose> starts;
    if (n >= 3) { Pose P; if (registerPointPairs(std::vector<V3>(sp.begin(), sp.begin()+n), std::vector<V3>(dp.begin(), dp.begin()+n), P)) starts.push_back(P); }
    // Too few pairs to fix every direction: the rest is searched, turning about what the pairs fix.
    else for (int k = 0; k < 24; k++) {
        const float a = k*6.2831853f/24;
        if (n == 2 && norm(sp[1]-sp[0]) > 1e-3f && norm(dp[1]-dp[0]) > 1e-3f) {
            const Pose R = mul(axisAngle(dp[1]-dp[0], a), turnOnto(sp[1]-sp[0], dp[1]-dp[0]));
            Pose P = R; P.t = (dp[0]+dp[1])*0.5f-R.rot((sp[0]+sp[1])*0.5f); starts.push_back(P);
        } else if (sn.size() >= 1 && dn.size() >= 1 && norm(sn[0]) > 0.5f && norm(dn[0]) > 0.5f) {
            const Pose R = mul(axisAngle(dn[0], a), turnOnto(sn[0], dn[0]));
            Pose P = R; P.t = dp[0]-R.rot(sp[0]); starts.push_back(P);
        }
    }
    if (starts.empty()) return results;
    V3 lo, hi; meshBounds(dstMesh, lo, hi); const float size = norm(hi-lo);
    // A placement that takes the picked points far from each other is not what the person meant.
    const float keepNear = std::max(4.f, std::max(d*4, size*0.03f));
    auto pairsApart = [&](const Pose& P) { float w = 0; for (size_t i = 0; i < n; i++) w = std::max(w, norm(P.apply(sp[i])-dp[i])); return w; };
    PointGrid gridD(dst.p, d*2), gridS(src.p, d*2); std::vector<RegisterResult> tried;
    for (const Pose& P0 : starts) { RegisterResult r = evaluate(src, dst, gridD, gridS, icp(src, dst, gridD, P0, d*4, d*1.2f, 16)); if (r.ok && pairsApart(r.srcToDst) < keepNear) tried.push_back(r); }
    // With three pairs the person has spoken: show their placement even if the surfaces hardly meet.
    if (tried.empty() && n >= 3) { RegisterResult r = evaluate(src, dst, gridD, gridS, starts[0]); r.ok = true; tried.push_back(r); }
    std::sort(tried.begin(), tried.end(), [](const RegisterResult& a, const RegisterResult& b) { return a.score > b.score; });
    V3 center{}; for (const V3& p : src.p) center = center+p; center = center*(1.f/src.p.size());
    auto differs = [&](const Pose& x, const Pose& y) { const V3 ax{1, 0, 0}, ay{0, 1, 0}; return norm(x.apply(center)-y.apply(center)) > d*3 || norm(x.rot(ax)-y.rot(ax)) > 0.15f || norm(x.rot(ay)-y.rot(ay)) > 0.15f; };
    const Fine fine(srcMesh, dstMesh, std::max(0.2f, d*0.4f), d*1.5f, srcTable, dstTable);
    for (const RegisterResult& r : tried) {
        bool nw = true; for (const RegisterResult& k : results) if (!differs(r.srcToDst, k.srcToDst)) { nw = false; break; }
        if (!nw) continue;
        RegisterResult f = refineOn(fine, r.srcToDst, d*1.5f);
        if (!f.ok || pairsApart(f.srcToDst) > keepNear) { f = r; f.ok = true; }
        results.push_back(f); if (results.size() >= 8) break;
    }
    rank(results, d);
    return results;
}
RegisterResult judgePlacement(const Mesh& srcMesh, const Mesh& dstMesh, const Pose& placement, Table srcTable, Table dstTable) {
    const Fine f(srcMesh, dstMesh, std::max(0.2f, sampleSpacing(srcMesh, dstMesh)*0.4f), 0, srcTable, dstTable);
    if (f.src.p.size() < 16 || f.dst.p.size() < 16) return {};
    return evaluate(f.src, f.dst, f.gridD, f.gridS, placement);
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
