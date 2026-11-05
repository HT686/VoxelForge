#include "stereo.hpp"
#include "gpustereo.hpp"
#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

float DepthMap::validRatio() const {
    if (z.empty()) return 0;
    size_t n = 0; for (float v : z) n += v > 0; return (float)n / z.size();
}
namespace {
void half(const uint8_t* s, int w, int h, std::vector<uint8_t>& d) {
    int W = w/2, H = h/2; d.resize(W*H);
    #pragma omp parallel for
    for (int y = 0; y < H; y++) {
        const uint8_t* a = s + 2*y*w; const uint8_t* b = a + w; uint8_t* o = d.data() + y*W;
        for (int x = 0; x < W; x++) o[x] = (a[2*x] + a[2*x+1] + b[2*x] + b[2*x+1] + 2) >> 2;
    }
}
void census7(const uint8_t* s, int w, int h, std::vector<uint64_t>& c) {
    c.assign(w*h, 0);
    #pragma omp parallel for
    for (int y = 3; y < h-3; y++)
        for (int x = 3; x < w-3; x++) {
            uint64_t v = 0; uint8_t m = s[y*w+x];
            for (int dy = -3; dy <= 3; dy++) {
                const uint8_t* r = s + (y+dy)*w + x;
                for (int dx = -3; dx <= 3; dx++) { if (dx == 0 && dy == 0) continue; v = (v << 1) | (r[dx] > m); }
            }
            c[y*w+x] = v;
        }
}
void integral(const uint8_t* s, int w, int h, std::vector<int64_t>& a, std::vector<int64_t>& a2) {
    a.assign((w+1)*(h+1), 0); a2.assign((w+1)*(h+1), 0);
    for (int y = 0; y < h; y++) {
        int64_t r = 0, r2 = 0; const uint8_t* p = s + y*w;
        int64_t* o = a.data() + (y+1)*(w+1), *u = a.data() + y*(w+1);
        int64_t* o2 = a2.data() + (y+1)*(w+1), *u2 = a2.data() + y*(w+1);
        for (int x = 0; x < w; x++) { r += p[x]; r2 += p[x]*p[x]; o[x+1] = u[x+1] + r; o2[x+1] = u2[x+1] + r2; }
    }
}
inline int64_t boxSum(const std::vector<int64_t>& a, int w1, int x0, int y0, int x1, int y1) {
    return a[y1*w1+x1] - a[y0*w1+x1] - a[y1*w1+x0] + a[y0*w1+x0];
}
}

// A surface at an angle appears sheared between the cameras, so a rectangular window compares
// slightly different surface points. Fit the local shift gradient, then compare windows that follow it.
// detail: a window that small may hold no dot at all. There the cell is dropped, not kept at its old value.
void StereoMatcher::refineSlanted(const uint8_t* ref, const uint8_t* other, int W, int H, int gw, int gh, int step, const StereoCalib& calib, int B, float minNcc, bool detail) {
    if (detail) refined_.assign(shift_.size(), 0.f); else refined_ = shift_;
    const int fitR = std::max(3, 12/step);
    constexpr float kRange = 1.5f, kStep = 0.25f; constexpr int kN = 13;
    #pragma omp parallel for schedule(dynamic, 8)
    for (int j = fitR; j < gh-fitR; j++) for (int i = fitR; i < gw-fitR; i++) {
        const float s0 = shift_[j*gw+i];
        if (s0 == 0) continue;
        if (params.slantLimit && !detail) {
            const float z = calib.depthFromShift(s0), X = -(step*i-calib.cxRef())/calib.focalPx*z-params.slantCx, Y = (step*j-calib.cy())/calib.focalPx*z-params.slantCy, Z = z-params.slantCz;
            if (X*X+Y*Y+Z*Z > params.slantR*params.slantR) continue;
        }
        // Least-squares plane through the neighbouring shifts; outliers are left out by the limit.
        double sxx = 0, sxy = 0, syy = 0, sxs = 0, sys = 0; int n = 0;
        for (int dy = -fitR; dy <= fitR; dy++) for (int dx = -fitR; dx <= fitR; dx++) {
            float q = shift_[(j+dy)*gw+i+dx];
            if (q == 0 || std::fabs(q-s0) > 0.25f*std::max(std::abs(dx), std::abs(dy))*step + 1.f) continue;
            double X = dx*step, Y = dy*step, Z = q-s0;
            sxx += X*X; sxy += X*Y; syy += Y*Y; sxs += X*Z; sys += Y*Z; n++;
        }
        const double det = sxx*syy-sxy*sxy;
        if (n < 30 || det < 1e-6) continue;
        const float gx = (float)((sxs*syy-sys*sxy)/det), gy = (float)((sys*sxx-sxs*sxy)/det);
        const int x = step*i, y = step*j;
        if (y-B < 0 || y+B >= H || x-B < 0 || x+B >= W) continue;
        double sa = 0, saa = 0; constexpr int MaxW = 31*31; float A[MaxW]; int N = 0;
        for (int dy = -B; dy <= B; dy++) for (int dx = -B; dx <= B; dx++) { float v = ref[(y+dy)*W+x+dx]; A[N++] = v; sa += v; saa += v*v; }
        const double varA = N*saa-sa*sa;
        if (varA <= (detail ? 64.0*N*N : 0.0)) continue;
        float scores[kN]; bool ok = true;
        for (int c = 0; c < kN && ok; c++) {
            const float s = s0 + (c-kN/2)*kStep;
            double sb = 0, sbb = 0, sab = 0; int a = 0;
            for (int dy = -B; dy <= B && ok; dy++) for (int dx = -B; dx <= B; dx++) {
                const float xo = x+dx - (s + gx*dx + gy*dy);
                const int xi = (int)std::floor(xo);
                if (xi < 0 || xi+1 >= W) { ok = false; break; }
                const float t = xo-xi; const uint8_t* row = other + (y+dy)*W;
                const float v = row[xi]*(1-t) + row[xi+1]*t;
                sb += v; sbb += v*v; sab += A[a++]*v;
            }
            if (!ok) break;
            const double varB = N*sbb-sb*sb;
            scores[c] = varB > 0 ? (float)((N*sab-sa*sb)/std::sqrt(varA*varB)) : -2.f;
        }
        if (!ok) continue;
        int k = 0; for (int c = 1; c < kN; c++) if (scores[c] > scores[k]) k = c;
        if (k == 0 || k == kN-1 || scores[k] < minNcc) continue;
        const float dn = scores[k-1]-2*scores[k]+scores[k+1];
        if (dn < -1e-6f) refined_[j*gw+i] = s0 + (k-kN/2 + 0.5f*(scores[k-1]-scores[k+1])/dn)*kStep;
    }
    shift_.swap(refined_);
}

void StereoMatcher::matchOnCpu(const uint8_t* ref, const uint8_t* other, int W, int H) {
    using Clock = std::chrono::steady_clock; auto mark = Clock::now();
    const auto lap = [&mark] { const auto now = Clock::now(); const float ms = std::chrono::duration<float, std::milli>(now-mark).count(); mark = now; return ms; };
    half(ref, W, H, hRef_); half(other, W, H, hOther_);
    const int hw = W/2, hh = H/2;
    census7(hRef_.data(), hw, hh, cRef_); census7(hOther_.data(), hw, hh, cOther_);

    times.census = lap();
    // Coarse stage. Evaluated on a quarter-resolution grid, but comparing half-resolution
    // census codes: the dots are about 10 px apart, so a real quarter-resolution image
    // would no longer contain the pattern. One shift step equals 2 full-resolution pixels.
    const int qw = W/4, qh = H/4;
    const int sMin = params.shiftMin/2, nS = (params.shiftMax - params.shiftMin)/2;
    cost_.resize((size_t)qw*qh*nS);
    #pragma omp parallel for schedule(static)
    for (int y = 2; y < qh-2; y++) {
        const uint64_t* cr = cRef_.data() + 2*y*hw; const uint64_t* co = cOther_.data() + 2*y*hw;
        uint8_t* c = cost_.data() + (size_t)y*qw*nS;
        for (int x = 0; x < qw; x++, c += nS) {
            const uint64_t code = cr[2*x];
            const int x0 = 2*x - sMin;   // other position for k = 0; moves left as k grows
            for (int k = 0; k < nS; k++) {
                int xo = x0 - k;
                c[k] = (xo >= 3 && xo < hw-3) ? (uint8_t)std::popcount(code ^ co[xo]) : (uint8_t)48;
            }
        }
    }
    constexpr int16_t kNoShift = INT16_MIN;   // shifts can be negative for very close objects
    coarse_.assign(qw*qh, kNoShift);
    constexpr int R = 3;   // 7x7 grid cells, 28x28 full-resolution pixels
    #pragma omp parallel
    {
        std::vector<uint32_t> colSum((size_t)nS*qw), box(nS);
        #pragma omp for schedule(dynamic, 8)
        for (int y = R+2; y < qh-R-2; y++) {
            std::fill(colSum.begin(), colSum.end(), 0);
            for (int yy = y-R; yy <= y+R; yy++) {
                const uint8_t* c = cost_.data() + (size_t)yy*qw*nS;
                for (size_t i = 0; i < colSum.size(); i++) colSum[i] += c[i];
            }
            std::fill(box.begin(), box.end(), 0);
            for (int x = 0; x < R; x++) { const uint32_t* add = colSum.data() + (size_t)x*nS; for (int k = 0; k < nS; k++) box[k] += add[k]; }
            for (int x = 0; x < qw-R; x++) {
                const uint32_t* add = colSum.data() + (size_t)(x+R)*nS;
                for (int k = 0; k < nS; k++) box[k] += add[k];
                if (x-R-1 >= 0) { const uint32_t* sub = colSum.data() + (size_t)(x-R-1)*nS; for (int k = 0; k < nS; k++) box[k] -= sub[k]; }
                if (x < R+2) continue;
                uint32_t best = 0xffffffff, second = 0xffffffff; int bi = -1;
                uint64_t total = 0;
                for (int k = 0; k < nS; k++) { total += box[k]; if (box[k] < best) { best = box[k]; bi = k; } }
                for (int k = 0; k < nS; k++) if (std::abs(k-bi) > 2 && box[k] < second) second = box[k];
                // Accept only a match that clearly beats both the average shift and the runner-up.
                if (bi >= 0 && 2*x - (sMin + bi) >= 3 && 2*x - (sMin + bi) < hw-3 && best*20 < second*18 && (uint64_t)best*nS*100 < total*55) coarse_[y*qw+x] = (int16_t)(2*(sMin + bi));
            }
        }
    }

    times.coarse = lap();
    // Fine stage: NCC at full resolution on the half-resolution grid.
    integral(ref, W, H, iRef_, iRef2_); integral(other, W, H, iOther_, iOther2_);
    const int step = params.gridStep, gw = W/step, gh = H/step, w1 = W+1;
    shift_.assign(gw*gh, 0.f);
    const int B = params.windowRadius, N = (2*B+1)*(2*B+1); constexpr int Search = 4;
    const int margin = B/step + 1;
    const float minNcc = params.minNcc; const int64_t minVar = (int64_t)params.minVariance * N * N;
    #pragma omp parallel for schedule(dynamic, 4)
    for (int j = margin; j < gh-margin; j++) {
        float scores[2*Search+3];
        for (int i = margin; i < gw-margin; i++) {
            const int x = step*i, y = step*j;
            int qx = x/4, qy = y/4;
            int cand[3], nc = 0;
            for (int r = 0; r <= 2 && nc == 0; r++)
                for (int dy = -r; dy <= r; dy++) for (int dx = -r; dx <= r; dx++) {
                    int yy = qy+dy, xx = qx+dx;
                    if (yy < 0 || yy >= qh || xx < 0 || xx >= qw) continue;
                    int c = coarse_[yy*qw+xx];
                    if (c == kNoShift) continue;
                    bool dup = false; for (int q = 0; q < nc; q++) dup |= std::abs(cand[q]-c) <= Search;
                    if (!dup && nc < 3) cand[nc++] = c;
                }
            if (!nc) continue;
            int64_t sa = boxSum(iRef_, w1, x-B, y-B, x+B+1, y+B+1), saa = boxSum(iRef2_, w1, x-B, y-B, x+B+1, y+B+1);
            int64_t varA = N*saa - sa*sa;
            if (varA < minVar) continue;
            float bestScore = -2, bestShift = 0;
            for (int q = 0; q < nc; q++) {
                int s0 = cand[q] - Search - 1, nCand = 2*Search + 3, first = -1, last = -1;
                for (int k = 0; k < nCand; k++) {
                    int s = s0 + k, xo = x - s; scores[k] = -2;
                    if (xo - B < 0 || xo + B + 1 > W) continue;
                    int64_t sb = boxSum(iOther_, w1, xo-B, y-B, xo+B+1, y+B+1), sbb = boxSum(iOther2_, w1, xo-B, y-B, xo+B+1, y+B+1);
                    int64_t varB = N*sbb - sb*sb;
                    if (varB < minVar) continue;
                    int sab = 0;
                    for (int dy = -B; dy <= B; dy++) {
                        const uint8_t* a = ref + (y+dy)*W + x - B; const uint8_t* b = other + (y+dy)*W + xo - B; int acc = 0;
                        for (int dx = 0; dx < 2*B+1; dx++) acc += a[dx]*b[dx];
                        sab += acc;
                    }
                    scores[k] = (float)((double)(N*(int64_t)sab - sa*sb) / std::sqrt((double)varA*(double)varB));
                    if (first < 0) first = k;
                    last = k;
                }
                for (int k = std::max(1, first); first >= 0 && k <= std::min(last, nCand-2); k++) {
                    if (scores[k] > bestScore && scores[k] >= scores[k-1] && scores[k] >= scores[k+1] && scores[k-1] > -1.5f && scores[k+1] > -1.5f) {
                        float dn = scores[k-1] - 2*scores[k] + scores[k+1];
                        bestScore = scores[k];
                        bestShift = (s0 + k) + (dn < -1e-6f ? 0.5f*(scores[k-1]-scores[k+1])/dn : 0.f);
                    }
                }
            }
            if (bestScore >= minNcc) shift_[j*gw+i] = bestShift;
        }
    }

    times.fine = lap();
}

StereoMatcher::StereoMatcher() {}
StereoMatcher::~StereoMatcher() {}
bool StereoMatcher::gpuReady() {
    if (compute_mode == ComputeMode::Cpu) return false;
    if (!gpuTried_) {
        gpuTried_ = true;
        if (getenv("VOXELFORGE_NO_GPU")) { gpuInfo_ = "Abgeschaltet"; return false; }
        gpu_ = std::make_unique<GpuStereo>(); gpuUsable_ = gpu_->init(); gpuInfo_ = gpu_->info();
    }
    return gpuUsable_;
}
// The card has to keep agreeing with the CPU, the reference. A driver fault would not crash but measure wrongly,
// and that would ruin a scan without anyone noticing. So on the first frames with a surface, and from time to time.
void StereoMatcher::checkGpuAgainstCpu(const uint8_t* ref, const uint8_t* other, int W, int H) {
    constexpr int kEveryFrames = 1000; constexpr size_t kMinCells = 500; constexpr float kSameShift = .05f, kMaxDifferent = .02f;
    if (compute_mode != ComputeMode::Auto || --gpuCheckIn_ > 0) return;
    std::vector<float> fromCard = shift_; matchOnCpu(ref, other, W, H);
    size_t both = 0, one = 0, far = 0;
    for (size_t i = 0; i < shift_.size(); i++) {
        const bool a = fromCard[i] != 0, b = shift_[i] != 0; if (a && b) { both++; far += std::fabs(fromCard[i]-shift_[i]) > kSameShift; } else one += a != b;
    }
    if (both < kMinCells) { gpuCheckIn_ = 1; shift_.swap(fromCard); return; }   // nothing to compare yet
    gpuCheckIn_ = kEveryFrames;
    if (far > kMaxDifferent*both || one > kMaxDifferent*both) {
        char b[160]; snprintf(b, sizeof b, "Weicht von der CPU ab (%.1f %% der Punkte), Grafikkarte abgeschaltet.", 100.0*(far+one)/both);
        gpuUsable_ = false; gpuThisFrame_ = false; gpuInfo_ = b; return;   // keeps the CPU result
    }
    shift_.swap(fromCard);
}
void StereoMatcher::compute(const uint8_t* ref, const uint8_t* other, int W, int H, const StereoCalib& calib, DepthMap& out) {
    using Clock = std::chrono::steady_clock; auto mark = Clock::now();
    const auto lap = [&mark] { const auto now = Clock::now(); const float ms = std::chrono::duration<float, std::milli>(now-mark).count(); mark = now; return ms; };
    if (calib.rowOffset != 0) {   // all matching below assumes that corresponding points share a row
        aligned_.assign((size_t)W*H, 0);
        for (int y = 0; y < H; y++) { int yo = y + calib.rowOffset; if (yo >= 0 && yo < H) memcpy(aligned_.data()+(size_t)y*W, other+(size_t)yo*W, W); }
        other = aligned_.data();
    }
    const int step = params.gridStep, gw = W/step, gh = H/step;
    out.width = gw; out.height = gh;
    out.fx = -calib.focalPx / step; out.fy = calib.focalPx / step; out.cx = calib.cxRef() / step; out.cy = calib.cy() / step;
    times = StereoTimes{};
    // The far pair's row offset is handled only by the CPU code.
    gpuThisFrame_ = calib.rowOffset == 0 && gpuReady();
    bool haveShifts = false;
    if (gpuThisFrame_) {
        haveShifts = gpuThisFrame_ = gpu_->match(ref, other, W, H, params, shift_);   // a failure must not lose the frame
        if (!gpuThisFrame_) { gpuUsable_ = false; gpuInfo_ = gpu_->info(); }
        else {
            if (gpuTestOffset != 0) for (float& s : shift_) if (s != 0) s += gpuTestOffset;
            checkGpuAgainstCpu(ref, other, W, H);
        }
        times.fine = lap();
    }
    if (!haveShifts) { matchOnCpu(ref, other, W, H); lap(); }
    // Remove small isolated patches, then convert to depth.
    const int minRegion = params.minRegion*4/(step*step);
    if (minRegion > 0) {
        std::vector<uint8_t> seen(gw*gh, 0); std::vector<int> stack, region;
        for (int s = 0; s < gw*gh; s++) {
            if (seen[s] || shift_[s] == 0) continue;
            stack.assign(1, s); region.clear(); seen[s] = 1;
            while (!stack.empty()) {
                int p = stack.back(); stack.pop_back(); region.push_back(p);
                const int nb[4] = {p-1, p+1, p-gw, p+gw};
                for (int q : nb) {
                    if (q < 0 || q >= gw*gh || seen[q] || shift_[q] == 0) continue;
                    if (std::abs(q-p) == 1 && q/gw != p/gw) continue;
                    if (std::abs(shift_[q]-shift_[p]) > 1.5f) continue;
                    seen[q] = 1; stack.push_back(q);
                }
            }
            if ((int)region.size() < minRegion) for (int p : region) shift_[p] = 0;
        }
    }
    times.region = lap();
    if (params.slantRefine) {
        if (!(gpuThisFrame_ && gpu_->refine(W, H, gw, gh, step, calib, params, params.windowRadius, params.minNcc, false, shift_))) refineSlanted(ref, other, W, H, gw, gh, step, calib, params.windowRadius, params.minNcc, false);
    }
    times.slant = lap();
    out.z.resize(gw*gh);
    #pragma omp parallel for
    for (int p = 0; p < gw*gh; p++) out.z[p] = shift_[p] != 0 ? calib.depthFromShift(shift_[p]) : 0.f;
}

void StereoMatcher::refineDetail(const uint8_t* ref, const uint8_t* other, int W, int H, const StereoCalib& calib, DepthMap& d) {
    const auto started = std::chrono::steady_clock::now();
    if (calib.rowOffset != 0) other = aligned_.data();   // made by compute() for this frame
    const int step = W/d.width;
    shift_.resize(d.z.size());
    const float fb = calib.focalPx*calib.baselineMm;
    #pragma omp parallel for
    for (size_t p = 0; p < d.z.size(); p++) shift_[p] = d.z[p] > 0 ? calib.dispOffset-fb/d.z[p] : 0.f;
    if (!(gpuThisFrame_ && gpu_->refine(W, H, d.width, d.height, step, calib, params, params.detailRadius, params.detailMinNcc, true, shift_))) refineSlanted(ref, other, W, H, d.width, d.height, step, calib, params.detailRadius, params.detailMinNcc, true);
    #pragma omp parallel for
    for (size_t p = 0; p < d.z.size(); p++) d.z[p] = shift_[p] != 0 ? calib.depthFromShift(shift_[p]) : 0.f;
    times.detail = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now()-started).count();
}
