#include "markers.hpp"
#include <algorithm>
#include <cmath>

namespace {
constexpr int kBlobThreshold = 200, kMinBlobArea = 150, kMaxBlobArea = 12000;
constexpr int kDotThreshold = 150;
struct Blob { float u, v, radius; };

void findBlobs(const uint8_t* im, int W, int H, std::vector<Blob>& out) {
    std::vector<uint8_t> seen((size_t)W*H, 0); std::vector<int> stack;
    for (int s = 0; s < W*H; s++) {
        if (seen[s] || im[s] < kBlobThreshold) continue;
        stack.assign(1, s); seen[s] = 1;
        double sx = 0, sy = 0; int area = 0, x0 = W, x1 = 0, y0 = H, y1 = 0;
        while (!stack.empty()) {
            int p = stack.back(); stack.pop_back(); int x = p % W, y = p / W;
            sx += x; sy += y; area++;
            x0 = std::min(x0, x); x1 = std::max(x1, x); y0 = std::min(y0, y); y1 = std::max(y1, y);
            const int nb[4] = {p-1, p+1, p-W, p+W};
            for (int q : nb) {
                if (q < 0 || q >= W*H || seen[q] || im[q] < kBlobThreshold) continue;
                if (std::abs(q-p) == 1 && q/W != p/W) continue;
                seen[q] = 1; stack.push_back(q);
            }
        }
        if (area < kMinBlobArea || area > kMaxBlobArea) continue;
        if (x0 <= 1 || y0 <= 1 || x1 >= W-2 || y1 >= H-2) continue;   // cut off by the image border
        float w = x1-x0+1.f, h = y1-y0+1.f;
        if (w/h > 3 || h/w > 3) continue;                             // not marker shaped
        out.push_back({(float)(sx/area), (float)(sy/area), 0.5f*std::max(w, h)});
    }
}
// Largest eigenvector of a symmetric 4x4 matrix (Jacobi rotations).
void largestEigenvector4(double A[4][4], double v[4]) {
    double V[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    for (int sweep = 0; sweep < 32; sweep++) for (int p = 0; p < 3; p++) for (int q = p+1; q < 4; q++) {
        if (std::fabs(A[p][q]) < 1e-14) continue;
        double th = 0.5*std::atan2(2*A[p][q], A[q][q]-A[p][p]), c = std::cos(th), s = std::sin(th);
        for (int k = 0; k < 4; k++) { double a = A[k][p], b = A[k][q]; A[k][p] = c*a-s*b; A[k][q] = s*a+c*b; }
        for (int k = 0; k < 4; k++) { double a = A[p][k], b = A[q][k]; A[p][k] = c*a-s*b; A[q][k] = s*a+c*b; }
        for (int k = 0; k < 4; k++) { double a = V[k][p], b = V[k][q]; V[k][p] = c*a-s*b; V[k][q] = s*a+c*b; }
    }
    int m = 0; for (int i = 1; i < 4; i++) if (A[i][i] > A[m][m]) m = i;
    for (int i = 0; i < 4; i++) v[i] = V[i][m];
}
}

void detectMarkers(const StereoFrame& f, const StereoCalib& c, int shiftMin, int shiftMax, std::vector<Marker3D>& out) {
    out.clear();
    const int W = f.width, H = f.height;
    std::vector<Blob> blobs; findBlobs(f.other.data(), W, H, blobs);
    if (blobs.empty()) return;
    // Integral image of bright reference pixels, to locate the dot cluster along the row.
    std::vector<int> I((size_t)(W+1)*(H+1), 0);
    for (int y = 0; y < H; y++) { int r = 0; for (int x = 0; x < W; x++) { r += f.ref[y*W+x] >= kDotThreshold; I[(y+1)*(W+1)+x+1] = I[y*(W+1)+x+1] + r; } }
    auto box = [&](int x0, int y0, int x1, int y1) {
        x0 = std::max(0, x0); y0 = std::max(0, y0); x1 = std::min(W, x1); y1 = std::min(H, y1);
        return x1 <= x0 || y1 <= y0 ? 0 : I[y1*(W+1)+x1] - I[y0*(W+1)+x1] - I[y1*(W+1)+x0] + I[y0*(W+1)+x0];
    };
    const float cxOther = c.cxRef() - c.dispOffset;
    for (const Blob& b : blobs) {
        const int r = std::max(6, (int)(b.radius)), v = (int)(b.v+.5f), u = (int)(b.u+.5f);
        int bestS = -1, bestN = 0, bestScore = 0;
        for (int s = shiftMin; s <= shiftMax; s++) {
            int x = u + s; if (x + r >= W) break;
            int n = box(x-r, v-r, x+r+1, v+r+1);
            if (n < 12) continue;
            // A marker is a small group with dark around it. From close up the object's own dots are as
            // bright and denser, and would win by count alone.
            const int around = box(x-2*r, v-2*r, x+2*r+1, v+2*r+1)-n, score = n-around;
            if (score > bestScore) { bestScore = score; bestN = n; bestS = s; }
        }
        if (bestS < 0 || bestN < 12) continue;
        // Center of the bright pixels. Iterated, because the first window may hold the cluster off center.
        float cx = (float)(u + bestS), cy = b.v;
        for (int it = 0; it < 4; it++) {
            double sx = 0, sy = 0; int n = 0;
            for (int y = std::max(0, (int)cy-r); y <= std::min(H-1, (int)cy+r); y++)
                for (int x = std::max(0, (int)cx-r); x <= std::min(W-1, (int)cx+r); x++)
                    if (f.ref[y*W+x] >= kDotThreshold) { sx += x; sy += y; n++; }
            if (!n) break;
            cx = (float)(sx/n); cy = (float)(sy/n);
        }
        float shift = cx - b.u, z = c.depthFromShift(shift);
        if (z <= 0) continue;
        Marker3D m;
        // X and Y from the accurate blob center in the other camera, which sits at x = -baseline.
        // The images are mirrored: x points the other way, and the other camera lies at +baseline.
        m.pos = V3{-(b.u-cxOther)/c.focalPx*z + c.baselineMm, (b.v-c.cy())/c.focalPx*z, z};
        m.u = b.u; m.v = b.v; m.radiusPx = b.radius;
        out.push_back(m);
    }
    // Depth from a dot cluster is coarse (about 1 px), the blob center is far better. Markers normally
    // lie on one flat surface: fit it robustly and place each marker where its accurate ray meets it.
    const size_t n = out.size();
    if (n < 5) return;
    size_t bestCount = 0; V3 bestN; float bestD = 0;
    const float tol = 2.5f;
    for (size_t i = 0; i < n; i++) for (size_t j = i+1; j < n; j++) for (size_t k = j+1; k < n; k++) {
        V3 nrm = cross(out[j].pos-out[i].pos, out[k].pos-out[i].pos);
        if (norm(nrm) < 100.f) continue;
        nrm = normalized(nrm); float d = dot(nrm, out[i].pos); size_t count = 0;
        for (const Marker3D& m : out) count += std::fabs(dot(nrm, m.pos)-d) < tol;
        if (count > bestCount) { bestCount = count; bestN = nrm; bestD = d; }
    }
    if (bestCount < 4) return;
    // Least-squares refinement over the inliers, from the normal equations of z = a x + b y + c along the dominant axis.
    std::vector<V3> in; for (const Marker3D& m : out) if (std::fabs(dot(bestN, m.pos)-bestD) < tol) in.push_back(m.pos);
    V3 ctr{}; for (const V3& p : in) ctr = ctr + p; ctr = ctr*(1.f/in.size());
    double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0;
    for (const V3& p : in) { V3 q = p-ctr; xx += q.x*q.x; xy += q.x*q.y; xz += q.x*q.z; yy += q.y*q.y; yz += q.y*q.z; }
    double det = xx*yy-xy*xy;
    if (std::fabs(det) > 1e-6) {
        double a = (xz*yy-yz*xy)/det, b = (yz*xx-xz*xy)/det;
        V3 fit = normalized(V3{(float)-a, (float)-b, 1.f});
        if (std::fabs(fit.z) > 0.2f) { bestN = dot(fit, bestN) < 0 ? fit*-1.f : fit; bestD = dot(bestN, ctr); }
    }
    const V3 o{c.baselineMm, 0, 0};
    std::vector<Marker3D> flat;
    for (const Marker3D& m : out) {
        if (std::fabs(dot(bestN, m.pos)-bestD) > tol*2) continue;   // a false cluster match
        V3 dir{-(m.u-cxOther)/c.focalPx, (m.v-c.cy())/c.focalPx, 1.f};
        float den = dot(bestN, dir); if (std::fabs(den) < 1e-4f) continue;
        Marker3D q = m; q.pos = o + dir*((bestD-dot(bestN, o))/den);
        flat.push_back(q);
    }
    out.swap(flat);
}

bool alignRigid(const std::vector<V3>& a, const std::vector<V3>& b, Pose& out) {
    const size_t n = a.size();
    if (n < 3 || b.size() != n) return false;
    V3 ca{}, cb{};
    for (size_t i = 0; i < n; i++) { ca = ca + a[i]; cb = cb + b[i]; }
    ca = ca*(1.f/n); cb = cb*(1.f/n);
    double S[3][3] = {{0}};
    for (size_t i = 0; i < n; i++) {
        V3 p = a[i]-ca, q = b[i]-cb; const double pv[3] = {p.x, p.y, p.z}, qv[3] = {q.x, q.y, q.z};
        for (int r = 0; r < 3; r++) for (int k = 0; k < 3; k++) S[r][k] += pv[r]*qv[k];
    }
    double N[4][4] = {
        {S[0][0]+S[1][1]+S[2][2], S[1][2]-S[2][1], S[2][0]-S[0][2], S[0][1]-S[1][0]},
        {S[1][2]-S[2][1], S[0][0]-S[1][1]-S[2][2], S[0][1]+S[1][0], S[2][0]+S[0][2]},
        {S[2][0]-S[0][2], S[0][1]+S[1][0], -S[0][0]+S[1][1]-S[2][2], S[1][2]+S[2][1]},
        {S[0][1]-S[1][0], S[2][0]+S[0][2], S[1][2]+S[2][1], -S[0][0]-S[1][1]+S[2][2]}};
    double q[4]; largestEigenvector4(N, q);
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    out.R[0] = (float)(w*w+x*x-y*y-z*z); out.R[1] = (float)(2*(x*y-w*z)); out.R[2] = (float)(2*(x*z+w*y));
    out.R[3] = (float)(2*(y*x+w*z)); out.R[4] = (float)(w*w-x*x+y*y-z*z); out.R[5] = (float)(2*(y*z-w*x));
    out.R[6] = (float)(2*(z*x-w*y)); out.R[7] = (float)(2*(z*y+w*x)); out.R[8] = (float)(w*w-x*x-y*y+z*z);
    out.t = cb - out.rot(ca);
    return true;
}

BasePlane fitBasePlane(const std::vector<V3>& pts, V3 viewpoint) {
    BasePlane out; const size_t n = pts.size();
    if (n < 5) return out;
    constexpr float kTol = 2.5f;
    size_t best = 0; V3 bestN, bestP;
    for (size_t i = 0; i < n; i++) for (size_t j = i+1; j < n; j++) for (size_t k = j+1; k < n; k++) {
        V3 nr = cross(pts[j]-pts[i], pts[k]-pts[i]);
        if (norm(nr) < 100.f) continue;
        nr = normalized(nr); size_t c = 0;
        for (const V3& p : pts) c += std::fabs(dot(nr, p-pts[i])) < kTol;
        if (c > best) { best = c; bestN = nr; bestP = pts[i]; }
    }
    if (best < 5) return out;
    std::vector<V3> in; V3 c{};
    for (const V3& p : pts) if (std::fabs(dot(bestN, p-bestP)) < kTol) { in.push_back(p); c = c + p; }
    c = c*(1.f/in.size());
    // Least squares: the normal is the direction of least spread, found by power iteration.
    double A[3][3] = {{0}};
    for (const V3& p : in) { V3 q = p-c; const double w[3] = {q.x, q.y, q.z}; for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) A[a][b] += w[a]*w[b]; }
    const double tr = A[0][0]+A[1][1]+A[2][2];
    double x[3] = {bestN.x, bestN.y, bestN.z};
    for (int it = 0; it < 100; it++) {
        double y[3]; for (int a = 0; a < 3; a++) { y[a] = tr*x[a]; for (int b = 0; b < 3; b++) y[a] -= A[a][b]*x[b]; }
        const double l = std::sqrt(y[0]*y[0]+y[1]*y[1]+y[2]*y[2]); for (int a = 0; a < 3; a++) x[a] = y[a]/l;
    }
    V3 nrm{(float)x[0], (float)x[1], (float)x[2]};
    if (dot(nrm, viewpoint-c) < 0) nrm = nrm*-1.f;
    out.normal = nrm; out.center = c; out.valid = true;
    for (const V3& p : in) out.radius = std::max(out.radius, out.radial(p));
    return out;
}
BasePlane MarkerTracker::basePlane(V3 viewpoint) const {
    std::vector<V3> pts; for (const MapMarker& m : map_) if (m.seen >= 3) pts.push_back(m.pos);
    return fitBasePlane(pts, viewpoint);
}

bool MarkerTracker::relocalize(const std::vector<Marker3D>& det, Pose& out) const {
    const int n = (int)det.size(), m = (int)map_.size();
    if (n < 4 || m < 5) return false;
    constexpr float kDistTol = 1.5f, kInlier = 2.f;
    int bestInliers = 0; float bestErr = 1e9f;
    auto dist = [](V3 a, V3 b) { return norm(a-b); };
    for (int a = 0; a < n; a++) for (int b = a+1; b < n; b++) for (int c = b+1; c < n; c++) {
        const float ab = dist(det[a].pos, det[b].pos), ac = dist(det[a].pos, det[c].pos), bc = dist(det[b].pos, det[c].pos);
        if (ab < 15 || ac < 15 || bc < 15) continue;     // short distances say little
        for (int i = 0; i < m; i++) for (int j = 0; j < m; j++) {
            if (j == i || std::fabs(dist(map_[i].pos, map_[j].pos)-ab) > kDistTol) continue;
            for (int k = 0; k < m; k++) {
                if (k == i || k == j) continue;
                if (std::fabs(dist(map_[i].pos, map_[k].pos)-ac) > kDistTol || std::fabs(dist(map_[j].pos, map_[k].pos)-bc) > kDistTol) continue;
                Pose T;
                if (!alignRigid({det[a].pos, det[b].pos, det[c].pos}, {map_[i].pos, map_[j].pos, map_[k].pos}, T)) continue;
                int inliers = 0; float e = 0;
                for (const Marker3D& q : det) {
                    V3 w = T.apply(q.pos); float best = kInlier*kInlier;
                    for (const MapMarker& mm : map_) { V3 d = mm.pos-w; best = std::min(best, dot(d, d)); }
                    if (best < kInlier*kInlier) { inliers++; e += best; }
                }
                if (inliers > bestInliers || (inliers == bestInliers && e < bestErr)) { bestInliers = inliers; bestErr = e; out = T; }
            }
        }
    }
    // Demand most markers to agree: patterns can look alike under some rotations.
    return bestInliers >= 4 && bestInliers*10 >= n*7;
}

// Few markers only fix a pose if they span a surface. Close together or in a row, a small error
// tilts everything far away.
static bool wellSpread(const std::vector<V3>& p, float minWidthMm) {
    float far = 0; size_t a = 0, b = 0;
    for (size_t i = 0; i < p.size(); i++) for (size_t j = i+1; j < p.size(); j++) { const float d = norm(p[i]-p[j]); if (d > far) { far = d; a = i; b = j; } }
    if (far < minWidthMm) return false;
    const V3 dir = (p[b]-p[a])*(1.f/far); float width = 0;
    for (const V3& q : p) { const V3 r = q-p[a]; width = std::max(width, norm(r-dir*dot(r, dir))); }
    return width >= minWidthMm;
}
MarkerResult MarkerTracker::update(const std::vector<Marker3D>& det, const Pose& prior) {
    MarkerResult res; res.detected = (int)det.size(); canLearn_ = false;
    if (map_.empty()) {
        if ((int)det.size() < minMatches) return res;
        for (const Marker3D& m : det) map_.push_back({prior.apply(m.pos), 1});
        res.ok = true; res.camToWorld = prior; res.matched = (int)det.size(); res.usable = det.size() >= 5;
        lastPose_ = prior; hasLast_ = true;
        return res;
    }
    std::vector<int> assign;
    auto refine = [&](Pose& P, float radius) {
        for (int it = 0; it < 3; it++) {
            assign.assign(det.size(), -1);
            std::vector<V3> a, b;
            for (size_t i = 0; i < det.size(); i++) {
                V3 w = P.apply(det[i].pos); float best = radius*radius; int bi = -1;
                for (size_t k = 0; k < map_.size(); k++) { V3 d = map_[k].pos-w; float e = dot(d, d); if (e < best) { best = e; bi = (int)k; } }
                if (bi >= 0) { assign[i] = bi; a.push_back(det[i].pos); b.push_back(map_[bi].pos); }
            }
            if ((int)a.size() < minMatches || !alignRigid(a, b, P)) return false;
            radius = std::min(radius, matchRadiusMm);   // once close, stay strict
        }
        return true;
    };
    // The fit must be tight: with a wide radius a wrong assignment can look complete.
    auto residual = [&](const Pose& P) { double e2 = 0; int n = 0; for (size_t i = 0; i < det.size(); i++) if (assign[i] >= 0) { V3 d = map_[assign[i]].pos-P.apply(det[i].pos); e2 += dot(d, d); n++; } return n ? std::sqrt(e2/n) : 1e9; };
    // Near the last pose, then where the motion so far would lead, then with a wider search.
    Pose T = prior; bool found = refine(T, matchRadiusMm);
    if (!found && hasMotion_) { T = mul(motion_, prior); found = refine(T, matchRadiusMm) && residual(T) < 1.f; }
    if (!found && hasMotion_) { T = mul(motion_, prior); found = refine(T, wideRadiusMm) && residual(T) < 1.f; }
    if (!found) { T = prior; found = refine(T, wideRadiusMm) && residual(T) < 1.f; }
    if (!found) {
        if (!relocalize(det, T) || !refine(T, matchRadiusMm)) { hasMotion_ = false; return res; }
        res.relocated = true;
    }
    // One badly measured or false marker must not spoil the pose of all the others, or none counts and the
    // map stops growing. Set aside the worst while it stands out, at most a quarter of them.
    { constexpr int kRejected = -2; int count = 0; for (int a : assign) count += a >= 0;
      for (int removed = 0; removed*4 < count; removed++) {
          std::vector<float> errors; float worst = 0; int worstAt = -1;
          for (size_t i = 0; i < det.size(); i++) if (assign[i] >= 0) { const float e = norm(map_[assign[i]].pos-T.apply(det[i].pos)); errors.push_back(e); if (e > worst) { worst = e; worstAt = (int)i; } }
          if ((int)errors.size() <= minMatches) break;
          std::nth_element(errors.begin(), errors.begin()+errors.size()/2, errors.end());
          if (worst < std::max(.6f, 3.f*errors[errors.size()/2])) break;
          assign[worstAt] = kRejected;
          std::vector<V3> a, b; for (size_t i = 0; i < det.size(); i++) if (assign[i] >= 0) { a.push_back(det[i].pos); b.push_back(map_[assign[i]].pos); }
          if (!alignRigid(a, b, T)) break;
      } }
    double e2 = 0; int n = 0;
    for (size_t i = 0; i < det.size(); i++) if (assign[i] >= 0) { V3 d = map_[assign[i]].pos - T.apply(det[i].pos); e2 += dot(d, d); n++; }
    res.matched = n; res.rmsMm = (float)std::sqrt(e2/std::max(1, n)); res.camToWorld = T; res.ok = true;
    // Motion in world coordinates per frame, for the next prediction. Not across a search by distances.
    if (!res.relocated && hasLast_) {
        Pose inv; for (int r = 0; r < 3; r++) for (int k = 0; k < 3; k++) inv.R[r*3+k] = lastPose_.R[k*3+r];
        inv.t = inv.rot(lastPose_.t*-1.f); motion_ = mul(T, inv); hasMotion_ = true;
    } else hasMotion_ = false;
    lastPose_ = T; hasLast_ = true;
    std::vector<V3> known; for (size_t i = 0; i < det.size(); i++) if (assign[i] >= 0) known.push_back(det[i].pos);
    lastSpread_ = wellSpread(known, 20.f);
    res.usable = n >= 5 || (n == 4 && lastSpread_ && res.rmsMm <= 0.5f);
    lastDet_ = det; lastAssign_ = assign; lastT_ = T; lastN_ = n; lastRms_ = res.rmsMm; canLearn_ = true;
    return res;
}
void MarkerTracker::learn() {
    if (!canLearn_) return;
    canLearn_ = false;
    // Refine the map, and add markers that have just come into view. A few known ones must be enough
    // for this, or the map starves when an object hides much of the ring.
    const bool mayAdd = lastN_ >= 5 || (lastN_ == 4 && lastSpread_ && lastRms_ < 0.4f);
    for (size_t i = 0; i < lastDet_.size(); i++) {
        const V3 w = lastT_.apply(lastDet_[i].pos);
        if (lastAssign_[i] >= 0) { MapMarker& m = map_[lastAssign_[i]]; float k = 1.f/std::min(m.seen+1, 200); m.pos = m.pos + (w-m.pos)*k; m.seen++; }
        else if (mayAdd) {
            bool near = false; for (const MapMarker& m : map_) { V3 d = m.pos-w; near |= dot(d, d) < 100.f; }
            if (!near) map_.push_back({w, 1});
        }
    }
}
