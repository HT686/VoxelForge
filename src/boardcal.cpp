#include "boardcal.hpp"
#include <algorithm>
#include <cmath>
#include <random>

namespace {
constexpr float kDotDiameterMm = 5.f;   // the Otter's board
// Box blur of radius r, separable, on floats.
void blur(const std::vector<float>& in, std::vector<float>& out, int w, int h, int r) {
    std::vector<float> tmp((size_t)w*h); out.assign((size_t)w*h, 0.f); const float inv = 1.f/(2*r+1);
    #pragma omp parallel for
    for (int y = 0; y < h; y++) { double s = 0; for (int x = -r; x <= r; x++) s += in[(size_t)y*w+std::clamp(x, 0, w-1)];
        for (int x = 0; x < w; x++) { tmp[(size_t)y*w+x] = (float)(s*inv); s += in[(size_t)y*w+std::min(w-1, x+r+1)]-in[(size_t)y*w+std::max(0, x-r)]; } }
    #pragma omp parallel for
    for (int x = 0; x < w; x++) { double s = 0; for (int y = -r; y <= r; y++) s += tmp[(size_t)std::clamp(y, 0, h-1)*w+x];
        for (int y = 0; y < h; y++) { out[(size_t)y*w+x] = (float)(s*inv); s += tmp[(size_t)std::min(h-1, y+r+1)*w+x]-tmp[(size_t)std::max(0, y-r)*w+x]; } }
}
}

namespace {
struct Dot { float x, y; int area; float size; };   // size: the larger side of its box, pixels
// Dark round spots of the board's regular size in one 8 bit image.
std::vector<Dot> findDots(const std::vector<uint8_t>& im, int W, int H) {
    std::vector<float> img((size_t)W*H); for (size_t i = 0; i < img.size(); i++) img[i] = im[i];
    std::vector<float> small, large; blur(img, small, W, H, 1); blur(img, large, W, H, 30);
    std::vector<float> dark((size_t)W*H); std::vector<float> sorted;
    for (size_t i = 0; i < dark.size(); i++) dark[i] = std::max(0.f, large[i]-small[i]);
    sorted = dark; const size_t k = sorted.size()*998/1000; std::nth_element(sorted.begin(), sorted.begin()+k, sorted.end()); const float top = sorted[k];
    if (top < 2) return {};
    const float thr = top*0.4f;
    std::vector<int> label((size_t)W*H, -1), stack; std::vector<Dot> dots;
    for (int s0 = 0; s0 < W*H; s0++) {
        if (label[s0] >= 0 || dark[s0] < thr) continue;
        double sx = 0, sy = 0, sw = 0; int n = 0, x0 = W, x1 = 0, y0 = H, y1 = 0; stack.assign(1, s0); label[s0] = 1;
        while (!stack.empty()) {
            const int p = stack.back(); stack.pop_back(); const int x = p%W, y = p/W; const double w = dark[p];
            sx += w*x; sy += w*y; sw += w; n++; x0 = std::min(x0, x); x1 = std::max(x1, x); y0 = std::min(y0, y); y1 = std::max(y1, y);
            const int nb[4] = {x > 0 ? p-1 : -1, x < W-1 ? p+1 : -1, y > 0 ? p-W : -1, y < H-1 ? p+W : -1};
            for (int q : nb) if (q >= 0 && label[q] < 0 && dark[q] >= thr) { label[q] = 1; stack.push_back(q); }
        }
        // Round: about as wide as high, and filling most of its box.
        const int bw = x1-x0+1, bh = y1-y0+1;
        if (n >= 20 && bw < 2*bh && bh < 2*bw && n > 0.5f*bw*bh && x0 > 2 && y0 > 2 && x1 < W-3 && y1 < H-3) dots.push_back({(float)(sx/sw), (float)(sy/sw), n, (float)std::max(bw, bh)});
    }
    // The regular dots are the most common size; the board's larger marking dots stay out.
    if (dots.size() < 6) return {};
    std::vector<int> a; for (const Dot& d : dots) a.push_back(d.area); std::nth_element(a.begin(), a.begin()+a.size()/2, a.end()); const int typ = a[a.size()/2];
    std::vector<Dot> keep; for (const Dot& d : dots) if (d.area > typ*0.6f && d.area < typ*1.5f) keep.push_back(d);
    return keep;
}
}
BoardMeasurement measureBoardStereo(const StereoFrame& f, const StereoCalib& calib, float pitchMm) {
    BoardMeasurement out; const int W = f.width, H = f.height; if (!W) return out;
    const std::vector<Dot> L = findDots(f.ref, W, H), R = findDots(f.other, W, H);
    if (L.size() < 6 || R.size() < 6) return out;
    // Candidate pairs: same row (the images are rectified), a shift in the working range.
    struct Pair { int l, r; float s; };
    std::vector<Pair> cand;
    for (size_t i = 0; i < L.size(); i++) for (size_t j = 0; j < R.size(); j++) {
        const float s = L[i].x-R[j].x; if (std::fabs(L[i].y-R[j].y) > 3.f || s < 60 || s > 560) continue;
        if (R[j].area > L[i].area*1.6f || L[i].area > R[j].area*1.6f) continue;
        // In a regular grid a dot also pairs with its neighbour, and that fits a plane as well. The dot's size
        // tells the distance roughly, enough to tell them apart: neighbours are a whole pitch of shift away.
        const float zSize = kDotDiameterMm*calib.focalPx/std::max(1.f, 0.5f*(L[i].size+R[j].size)), D = calib.dispOffset-s;
        if (D <= 1 || std::fabs(calib.focalPx*calib.baselineMm/D-zSize) > 0.25f*zSize) continue;
        cand.push_back({(int)i, (int)j, s});
    }
    if (cand.size() < 6) return out;
    // On a plane the shift is affine in the image position: s = a x + b y + c. Three pairs make a guess, the
    // one most pairs agree with wins.
    std::mt19937 rng(5); std::uniform_int_distribution<size_t> pick(0, cand.size()-1); int bestN = 0; double best[3] = {0, 0, 0};
    for (int it = 0; it < 2000; it++) {
        const Pair& p0 = cand[pick(rng)]; const Pair& p1 = cand[pick(rng)]; const Pair& p2 = cand[pick(rng)];
        const double M[9] = {L[p0.l].x, L[p0.l].y, 1, L[p1.l].x, L[p1.l].y, 1, L[p2.l].x, L[p2.l].y, 1}, v[3] = {p0.s, p1.s, p2.s};
        const double det = M[0]*(M[4]*M[8]-M[5]*M[7])-M[1]*(M[3]*M[8]-M[5]*M[6])+M[2]*(M[3]*M[7]-M[4]*M[6]); if (std::fabs(det) < 1e3) continue;
        double c[3]; for (int k = 0; k < 3; k++) { double N[9]; for (int r = 0; r < 3; r++) for (int q = 0; q < 3; q++) N[r*3+q] = q == k ? v[r] : M[r*3+q]; c[k] = (N[0]*(N[4]*N[8]-N[5]*N[7])-N[1]*(N[3]*N[8]-N[5]*N[6])+N[2]*(N[3]*N[7]-N[4]*N[6]))/det; }
        int n = 0; for (const Pair& p : cand) n += std::fabs(c[0]*L[p.l].x+c[1]*L[p.l].y+c[2]-p.s) < 1.5;
        if (n > bestN) { bestN = n; std::copy(c, c+3, best); }
    }
    // Each left dot with its partner that fits the plane best, used once.
    std::vector<Pair> pairs; std::vector<uint8_t> usedR(R.size(), 0);
    for (size_t i = 0; i < L.size(); i++) {
        const Pair* b = nullptr; double be = 1.5;
        for (const Pair& p : cand) if (p.l == (int)i) { const double e = std::fabs(best[0]*L[i].x+best[1]*L[i].y+best[2]-p.s); if (e < be && !usedR[p.r]) { be = e; b = &p; } }
        if (b) { pairs.push_back(*b); usedR[b->r] = 1; }
    }
    if (pairs.size() < 6) return out;
    // 3D from the stereo formula. Lengths do not depend on the sign of x, so the mirroring does not matter here.
    const float fb = calib.focalPx*calib.baselineMm, cx = calib.cxRef(), cy = calib.cy();
    for (const Pair& p : pairs) {
        const float D = calib.dispOffset-p.s; if (D <= 1) continue; const float z = fb/D;
        out.centers.push_back({(L[p.l].x-cx)*z/calib.focalPx, (L[p.l].y-cy)*z/calib.focalPx, z});
    }
    std::vector<float> dist;
    for (size_t i = 0; i < out.centers.size(); i++) for (size_t j = i+1; j < out.centers.size(); j++) {
        const float e = norm(out.centers[i]-out.centers[j]); if (e > pitchMm*0.8f && e < pitchMm*1.2f) dist.push_back(e);
    }
    if (dist.size() < 4) return out;
    std::sort(dist.begin(), dist.end()); out.spacingMm = dist[dist.size()/2];
    double s2 = 0; for (float e : dist) s2 += (e-out.spacingMm)*(e-out.spacingMm); out.spacingSpread = (float)std::sqrt(s2/dist.size());
    // Plane through the centers: distance, tilt, how flat.
    V3 c{}; for (const V3& p : out.centers) c = c+p; c = c*(1.f/out.centers.size());
    double A[3][3] = {{0}}; for (const V3& p : out.centers) { const V3 q = p-c; const double w3[3] = {q.x, q.y, q.z}; for (int r = 0; r < 3; r++) for (int s3 = 0; s3 < 3; s3++) A[r][s3] += w3[r]*w3[s3]; }
    const double tr = A[0][0]+A[1][1]+A[2][2]; double x[3] = {0, 0, 1};
    for (int it = 0; it < 80; it++) { double y[3]; for (int r = 0; r < 3; r++) { y[r] = tr*x[r]; for (int s3 = 0; s3 < 3; s3++) y[r] -= A[r][s3]*x[s3]; } const double l = std::sqrt(y[0]*y[0]+y[1]*y[1]+y[2]*y[2]); for (int r = 0; r < 3; r++) x[r] = y[r]/l; }
    const V3 n = normalized(V3{(float)x[0], (float)x[1], (float)x[2]}); double e = 0; for (const V3& p : out.centers) { const float r = dot(n, p-c); e += r*r; }
    out.flatnessMm = (float)std::sqrt(e/out.centers.size()); out.distanceMm = c.z; out.tiltDeg = std::acos(std::min(1.f, std::fabs(n.z)))*57.2958f;
    out.shift = calib.dispOffset-fb/c.z;
    out.dots = (int)out.centers.size(); out.pairs = (int)dist.size(); out.ok = true;
    return out;
}
BoardMeasurement measureBoard(const StereoFrame& f, const DepthMap& d, const StereoCalib& calib, float pitchMm) {
    BoardMeasurement out; const int W = f.width, H = f.height; if (!W || !H || d.width <= 0) return out;
    const float g = (float)d.width/W;   // depth grid per image pixel
    // How large a dot appears: from the depth in the middle of the picture and the dot size.
    std::vector<float> zs; for (int j = d.height/4; j < d.height*3/4; j += 2) for (int i = d.width/4; i < d.width*3/4; i += 2) { const float z = d.z[(size_t)j*d.width+i]; if (z > 0) zs.push_back(z); }
    if (zs.size() < 100) return out;
    std::nth_element(zs.begin(), zs.begin()+zs.size()/2, zs.end()); const float zMid = zs[zs.size()/2];
    const float rPx = kDotDiameterMm*0.5f*calib.focalPx/zMid, expectArea = 3.14159f*rPx*rPx;
    // Darkness against the surroundings. The printed dots take no projector light, so they stay dark disks; the
    // small blur, a fifth of a dot, fills in the projector's speckle, the large one gives the paper's brightness.
    std::vector<float> img((size_t)W*H); for (size_t i = 0; i < img.size(); i++) img[i] = f.ref[i];
    std::vector<float> small, large; blur(img, small, W, H, std::max(2, (int)(rPx*0.4f))); blur(img, large, W, H, std::max(10, (int)(rPx*3)));
    std::vector<float> dark((size_t)W*H); float maxDark = 0;
    for (size_t i = 0; i < dark.size(); i++) { dark[i] = std::max(0.f, large[i]-small[i]); maxDark = std::max(maxDark, dark[i]); }
    if (maxDark < 4) return out;
    const float thr = maxDark*0.35f;
    struct Blob { double sx = 0, sy = 0, sw = 0; int n = 0, x0 = 1 << 30, x1 = 0, y0 = 1 << 30, y1 = 0; };
    std::vector<int> label((size_t)W*H, -1); std::vector<Blob> blobs; std::vector<int> stack;
    for (int s = 0; s < W*H; s++) {
        if (label[s] >= 0 || dark[s] < thr) continue;
        Blob b; stack.assign(1, s); label[s] = (int)blobs.size();
        while (!stack.empty()) {
            const int p = stack.back(); stack.pop_back(); const int x = p%W, y = p/W; const float w = dark[p]-thr*0.5f;
            b.sx += w*x; b.sy += w*y; b.sw += w; b.n++; b.x0 = std::min(b.x0, x); b.x1 = std::max(b.x1, x); b.y0 = std::min(b.y0, y); b.y1 = std::max(b.y1, y);
            const int nb[4] = {x > 0 ? p-1 : -1, x < W-1 ? p+1 : -1, y > 0 ? p-W : -1, y < H-1 ? p+W : -1};
            for (int q : nb) if (q >= 0 && label[q] < 0 && dark[q] >= thr) { label[q] = (int)blobs.size(); stack.push_back(q); }
        }
        blobs.push_back(b);
    }
    // Only disks of the board's dot size, round, and not cut by the picture's edge.
    std::vector<std::pair<float, float>> px;
    for (const Blob& b : blobs) {
        const int bw = b.x1-b.x0+1, bh = b.y1-b.y0+1;
        if (b.n < expectArea*0.4f || b.n > expectArea*1.8f || bw > 2*bh || bh > 2*bw || b.x0 < 3 || b.y0 < 3 || b.x1 > W-4 || b.y1 > H-4 || b.sw <= 0) continue;
        px.push_back({(float)(b.sx/b.sw), (float)(b.sy/b.sw)});
    }
    if (px.size() < 6) return out;
    // The board's plane, from the depth inside the area of the dots, robustly.
    float x0 = 1e9f, x1 = -1e9f, y0 = 1e9f, y1 = -1e9f; for (auto& p : px) { x0 = std::min(x0, p.first); x1 = std::max(x1, p.first); y0 = std::min(y0, p.second); y1 = std::max(y1, p.second); }
    std::vector<V3> pts;
    for (int j = (int)(y0*g); j <= (int)(y1*g) && j < d.height; j++) for (int i = (int)(x0*g); i <= (int)(x1*g) && i < d.width; i++) {
        if (i < 0 || j < 0) continue;
        const float z = d.z[(size_t)j*d.width+i]; if (z <= 0) continue;
        pts.push_back({(i-d.cx)/d.fx*z, (j-d.cy)/d.fy*z, z});
    }
    if (pts.size() < 200) return out;
    std::mt19937 rng(17); std::uniform_int_distribution<size_t> pick(0, pts.size()-1); size_t best = 0; V3 n; float h = 0;
    for (int it = 0; it < 300; it++) {
        const V3 a = pts[pick(rng)], b = pts[pick(rng)], c = pts[pick(rng)]; V3 m = cross(b-a, c-a); if (norm(m) < 1e-3f) continue; m = normalized(m);
        size_t k = 0; for (size_t i = 0; i < pts.size(); i += 4) k += std::fabs(dot(m, pts[i])-dot(m, a)) < 0.6f;
        if (k > best) { best = k; n = m; h = dot(m, a); }
    }
    // Least squares through the points near it.
    { V3 c{}; size_t k = 0; for (const V3& p : pts) if (std::fabs(dot(n, p)-h) < 0.8f) { c = c+p; k++; } if (k < 100) return out; c = c*(1.f/k);
      double A[3][3] = {{0}}; for (const V3& p : pts) if (std::fabs(dot(n, p)-h) < 0.8f) { const V3 q = p-c; const double w3[3] = {q.x, q.y, q.z}; for (int r = 0; r < 3; r++) for (int s2 = 0; s2 < 3; s2++) A[r][s2] += w3[r]*w3[s2]; }
      const double tr = A[0][0]+A[1][1]+A[2][2]; double x[3] = {n.x, n.y, n.z};
      for (int it = 0; it < 60; it++) { double y[3]; for (int r = 0; r < 3; r++) { y[r] = tr*x[r]; for (int s2 = 0; s2 < 3; s2++) y[r] -= A[r][s2]*x[s2]; } const double l = std::sqrt(y[0]*y[0]+y[1]*y[1]+y[2]*y[2]); for (int r = 0; r < 3; r++) x[r] = y[r]/l; }
      n = normalized(V3{(float)x[0], (float)x[1], (float)x[2]}); h = dot(n, c);
      double e = 0; size_t m2 = 0; for (const V3& p : pts) { const float r = dot(n, p)-h; if (std::fabs(r) < 2.f) { e += r*r; m2++; } } out.flatnessMm = m2 ? (float)std::sqrt(e/m2) : 0;
      out.distanceMm = norm(c); out.tiltDeg = std::acos(std::min(1.f, std::fabs(dot(n, normalized(c)))))*57.2958f;
      out.shift = calib.dispOffset-calib.focalPx*calib.baselineMm/c.z; }
    // Dot centers onto the plane, along their viewing rays.
    for (auto& p : px) {
        const V3 ray{(p.first*g-d.cx)/d.fx, (p.second*g-d.cy)/d.fy, 1.f}; const float den = dot(n, ray); if (std::fabs(den) < 1e-6f) continue;
        out.centers.push_back(ray*(h/den));
    }
    // Neighbours: the nearest few of each dot near the pitch, each pair once.
    std::vector<float> dist;
    for (size_t i = 0; i < out.centers.size(); i++) for (size_t j = i+1; j < out.centers.size(); j++) {
        const float e = norm(out.centers[i]-out.centers[j]); if (e > pitchMm*0.8f && e < pitchMm*1.2f) dist.push_back(e);
    }
    if (dist.size() < 4) return out;
    std::sort(dist.begin(), dist.end()); out.spacingMm = dist[dist.size()/2];
    double s2 = 0; for (float e : dist) s2 += (e-out.spacingMm)*(e-out.spacingMm); out.spacingSpread = (float)std::sqrt(s2/dist.size());
    out.dots = (int)out.centers.size(); out.pairs = (int)dist.size(); out.ok = true;
    return out;
}

bool fitStereoCorrection(const std::vector<BoardMeasurement>& m, const StereoCalib& calib, float pitchMm, float& scale, float& offset, float& residualPct) {
    // Lateral lengths go as baseline / disparity. With D' = D + offset and the scale on the baseline, the true
    // spacing is measured * scale * D / (D + offset). Search offset, solve scale in closed form.
    std::vector<std::pair<float, float>> pts;   // disparity D, measured spacing / pitch
    for (const BoardMeasurement& b : m) if (b.ok) pts.push_back({calib.dispOffset-b.shift, b.spacingMm/pitchMm});
    if (pts.size() < 2) return false;
    float bestErr = 1e30f;
    for (float o = -20; o <= 20; o += 0.01f) {
        // ratio r = measured/pitch should equal (D+o)/(D*scale): scale = mean of (D+o)/(D*r).
        double s = 0; for (auto& p : pts) s += (p.first+o)/(p.first*p.second); s /= pts.size();
        double e = 0; for (auto& p : pts) { const double pred = (p.first+o)/(p.first*s); e += (pred-p.second)*(pred-p.second); }
        if (e < bestErr) { bestErr = (float)e; offset = o; scale = (float)s; }
    }
    residualPct = 100*std::sqrt(bestErr/pts.size());
    return true;
}
