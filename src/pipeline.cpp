#include "pipeline.hpp"
#include "meshedit.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <omp.h>

using Clock = std::chrono::steady_clock;
static float msSince(Clock::time_point a) { return std::chrono::duration<float, std::milli>(Clock::now() - a).count(); }

Pipeline::Pipeline(const PipelineOptions& o) : options(o) {
    stereo.compute_mode = ComputeMode::Auto;
    if (const char* m = getenv("VOXELFORGE_COMPUTE")) { const std::string v = m; stereo.compute_mode = v == "cpu" ? ComputeMode::Cpu : v == "gpu" ? ComputeMode::Gpu : ComputeMode::Auto; }
    reset();
}
void Pipeline::reset() {
    std::lock_guard<std::mutex> g(mutex_);
    tsdf_ = std::make_unique<Tsdf>(options.voxelMm);
    pose_ = Pose{};
    stats_ = PipelineStats{};
    region_.valid = false;
    markers_.reset(); hasMarkerPose_ = false; markerWeak_ = false; unconfirmed_ = 0; hasLastIntegrated_ = false; base_ = BasePlane{}; useObjectDepth_ = false;
    keyframes_.clear(); relocCursor_ = 0; wasLost_ = false;
    session_.clear(); sessionBytes_ = 0;
}

// Projective point-to-plane ICP of the live depth map against a raycast of the model.
bool Pipeline::track(const DepthMap& d) {
    const int rw = d.width/2, rh = d.height/2;
    const float rfx = d.fx/2, rfy = d.fy/2, rcx = d.cx/2, rcy = d.cy/2;
    // No motion prediction: on flat scenes it feeds sliding errors back and diverges.
    const Pose prev = pose_;
    Pose T = pose_;
    tsdf_->raycast(prev, rw, rh, rfx, rfy, rcx, rcy, options.zMin, options.zMax, ray_);
    // Balance surface orientations, so a large flat background cannot outvote the object's shape.
    constexpr int kBins = 8;
    float binWeight[kBins*kBins];
    {
        int hist[kBins*kBins] = {0}, total = 0;
        for (const V3& n : ray_.normal) {
            if (n.x == 0 && n.y == 0 && n.z == 0) continue;
            int a = std::min(kBins-1, (int)((n.x+1)*0.5f*kBins)), b = std::min(kBins-1, (int)((n.y+1)*0.5f*kBins));
            hist[b*kBins+a]++; total++;
        }
        int used = 0; for (int h : hist) used += h > 0;
        for (int k = 0; k < kBins*kBins; k++)
            binWeight[k] = hist[k] ? std::min(8.f, std::sqrt((float)total/(used*hist[k]))) : 1.f;
    }
    // Coarse-to-fine sampling, chosen by how many points there are: small objects have few.
    size_t validCells = 0; for (float z : d.z) validCells += z > 0;
    const int coarse = std::clamp((int)std::sqrt(validCells/1500.f), 1, 16), middle = std::max(1, coarse/2), fine = std::max(1, coarse/4);
    const int strides[] = {coarse, coarse, coarse, middle, middle, middle, fine, fine};
    float rms = 0, inlier = 0; int used = 0;
    for (int stride : strides) {
        double A[36] = {0}, b[6] = {0}; double e2 = 0; long n = 0, valid = 0;
        #pragma omp parallel
        {
            double la[36] = {0}, lb[6] = {0}, le = 0; long ln = 0, lv = 0;
            #pragma omp for schedule(static) nowait
            for (int j = 0; j < d.height; j += stride) {
                for (int i = 0; i < d.width; i += stride) {
                    float z = d.z[j*d.width+i];
                    if (z <= 0) continue;
                    lv++;
                    V3 p{(i-d.cx)/d.fx*z, (j-d.cy)/d.fy*z, z};
                    V3 w = T.apply(p);
                    V3 c = prev.applyInv(w);
                    if (c.z <= 1) continue;
                    int u = (int)(c.x/c.z*rfx + rcx + .5f), v = (int)(c.y/c.z*rfy + rcy + .5f);
                    if (u < 0 || v < 0 || u >= rw || v >= rh) continue;
                    V3 nrm = ray_.normal[v*rw+u];
                    if (nrm.x == 0 && nrm.y == 0 && nrm.z == 0) continue;
                    V3 q = ray_.vertex[v*rw+u];
                    V3 diff = w - q;
                    if (norm(diff) > options.maxPointDist) continue;
                    float r = dot(diff, nrm);
                    float wgt = std::fabs(r) < 1.f ? 1.f : 1.f/std::fabs(r);  // Huber, 1 mm
                    wgt *= binWeight[std::min(kBins-1, (int)((nrm.y+1)*0.5f*kBins))*kBins + std::min(kBins-1, (int)((nrm.x+1)*0.5f*kBins))];
                    V3 cr = cross(w, nrm);
                    const double J[6] = {cr.x, cr.y, cr.z, nrm.x, nrm.y, nrm.z};
                    for (int a = 0; a < 6; a++) { lb[a] -= wgt*J[a]*r; for (int c2 = 0; c2 <= a; c2++) la[a*6+c2] += wgt*J[a]*J[c2]; }
                    le += r*r; ln++;
                }
            }
            #pragma omp critical
            { for (int a = 0; a < 36; a++) A[a] += la[a]; for (int a = 0; a < 6; a++) b[a] += lb[a]; e2 += le; n += ln; valid += lv; }
        }
        for (int a = 0; a < 6; a++) for (int c = a+1; c < 6; c++) A[a*6+c] = A[c*6+a];
        used = (int)n;
        inlier = valid ? (float)n/valid : 0;
        rms = n ? std::sqrt((float)(e2/n)) : 1e9f;
        if (n < 30) { used = 0; break; }   // coarse levels hold few points; the final count is checked below
        for (int a = 0; a < 6; a++) A[a*7] *= 1.0 + 1e-6;
        double x[6];
        if (!solve6(A, b, x)) break;
        // The update acts in world coordinates, around the world origin.
        Pose dT = expTwist(x);
        T = mul(dT, T);
    }
    stats_.inlierRatio = inlier; stats_.rmsMm = rms; stats_.icpPoints = used;
    V3 dt = T.t - pose_.t;
    bool ok = used >= 200 && inlier >= options.minInlierRatio && rms <= options.maxRmsMm && norm(dt) < 60.f;
    if (ok) pose_ = T;
    return ok;
}

// Takes the nearest sufficiently large depth-continuous surface as the object. Size alone
// would pick a wall behind it. The minimum size keeps stray patches from winning.
void Pipeline::locateObject(const DepthMap& d) {
    const int w = d.width, h = d.height;
    const size_t cellScale = (size_t)(w/640)*(w/640);   // size limits below are for the 640 wide grid
    std::vector<uint8_t> seen((size_t)w*h, 0); std::vector<int> stack;
    std::vector<std::vector<int>> regions;
    for (int s = 0; s < w*h; s++) {
        if (seen[s] || d.z[s] <= 0) continue;
        std::vector<int> region; stack.assign(1, s); seen[s] = 1;
        while (!stack.empty()) {
            int p = stack.back(); stack.pop_back(); region.push_back(p);
            const int nb[4] = {p-1, p+1, p-w, p+w};
            for (int q : nb) {
                if (q < 0 || q >= w*h || seen[q] || d.z[q] <= 0) continue;
                if (std::abs(q-p) == 1 && q/w != p/w) continue;
                if (std::fabs(d.z[q]-d.z[p]) > 4.f) continue;
                seen[q] = 1; stack.push_back(q);
            }
        }
        if (region.size() >= 500*cellScale) regions.push_back(std::move(region));
    }
    // The nearest surface of real size seeds the object (a fixed minimum, not a share of the
    // largest surface: a 30 mm cube must win against a wall). Depth edges split one object into
    // several surfaces, so nearby surfaces join it, and the wall behind stays out.
    struct Info { const std::vector<int>* cells; V3 center; };
    std::vector<Info> infos;
    auto cellPoint = [&](int p) { float z = d.z[p]; return V3{((p%w)-d.cx)/d.fx*z, ((p/w)-d.cy)/d.fy*z, z}; };
    for (auto& r : regions) {
        std::vector<float> xs, ys, zs; xs.reserve(r.size()); ys.reserve(r.size()); zs.reserve(r.size());
        for (int p : r) { V3 q = cellPoint(p); xs.push_back(q.x); ys.push_back(q.y); zs.push_back(q.z); }
        auto mid = [](std::vector<float>& v) { std::nth_element(v.begin(), v.begin()+v.size()/2, v.end()); return v[v.size()/2]; };
        infos.push_back({&r, V3{mid(xs), mid(ys), mid(zs)}});
    }
    const Info* seed = nullptr;
    for (const Info& i : infos) if (i.cells->size() >= 1500*cellScale && (!seed || i.center.z < seed->center.z)) seed = &i;
    std::vector<int> best;
    if (seed) for (const Info& i : infos)
        if (norm(i.center - seed->center) < 60.f) best.insert(best.end(), i.cells->begin(), i.cells->end());
    if (best.size() < 500*cellScale) return;
    std::vector<float> xs, ys, zs; xs.reserve(best.size()); ys.reserve(best.size()); zs.reserve(best.size());
    for (int p : best) { float z = d.z[p]; xs.push_back(((p%w)-d.cx)/d.fx*z); ys.push_back(((p/w)-d.cy)/d.fy*z); zs.push_back(z); }
    auto med = [](std::vector<float>& v) { std::nth_element(v.begin(), v.begin()+v.size()/2, v.end()); return v[v.size()/2]; };
    V3 c{med(xs), med(ys), med(zs)};
    std::vector<float> dist; dist.reserve(best.size());
    for (int p : best) { float z = d.z[p]; V3 q{((p%w)-d.cx)/d.fx*z, ((p/w)-d.cy)/d.fy*z, z}; dist.push_back(norm(q-c)); }
    std::nth_element(dist.begin(), dist.begin()+dist.size()*98/100, dist.end());
    // Tight on purpose: a larger sphere lowered tracking accuracy in the turntable test (0.99 vs 9.8
    // degrees). The minimum keeps a 30 mm cube's corners inside.
    region_.center = c; region_.radius = std::max(30.f, dist[dist.size()*98/100]*1.3f); region_.valid = true;
}
void Pipeline::applyRegion(const DepthMap& d, DepthMap& out) const {
    out = d;
    const float r = region_.radius*region_.radiusScale;
    const bool follow = hasMarkerPose_;
    #pragma omp parallel for
    for (int j = 0; j < d.height; j++) for (int i = 0; i < d.width; i++) {
        float& z = out.z[j*d.width+i];
        if (z <= 0) continue;
        V3 q{(i-d.cx)/d.fx*z, (j-d.cy)/d.fy*z, z};
        if (norm((follow ? markerPose_.apply(q) : q)-region_.center) > r) z = 0;
    }
}

// With markers the object is known directly: it stands on their plane, inside their ring.
void Pipeline::applyBase(const DepthMap& d, const BasePlane& base, const Pose& T, DepthMap& out) const {
    out = d;
    // From close up only a part of the ring is in view, and its center and radius are those of that part.
    // The object would fall out of so small a cylinder. The picture itself is small then, so a wide limit is enough.
    const float maxRadial = std::max(base.radius*region_.radiusScale, basePartlySeen_ ? 120.f : 0.f), maxHeight = std::max(3.f*base.radius*region_.radiusScale, basePartlySeen_ ? 150.f : 0.f);
    const bool turntable = region_.enabled;
    const float minHeight = options.removeBase ? options.baseClearanceMm : -3.f;
    #pragma omp parallel for
    for (int j = 0; j < d.height; j++) for (int i = 0; i < d.width; i++) {
        float& z = out.z[j*d.width+i];
        if (z <= 0) continue;
        const V3 w = T.apply(V3{(i-d.cx)/d.fx*z, (j-d.cy)/d.fy*z, z});
        const float h = base.height(w);
        if (h < minHeight || (turntable && (h > maxHeight || base.radial(w) > maxRadial))) z = 0;
    }
}
// Decides which depth counts as object for this frame, from markers if possible, else from the region.
// Stereo errs most at depth jumps, and what remains of a table or a hand are small islands.
void Pipeline::cleanDepth(DepthMap& d) const {
    const int W = d.width, H = d.height; constexpr float kJump = 3.f; constexpr float kKeepShare = 0.15f;
    std::vector<int> label((size_t)W*H, -1), sizes, stack;
    for (int s = 0; s < W*H; s++) {
        if (d.z[s] <= 0 || label[s] >= 0) continue;
        const int id = (int)sizes.size(); int count = 0; stack.assign(1, s); label[s] = id;
        while (!stack.empty()) {
            const int p = stack.back(); stack.pop_back(); count++;
            const int x = p%W, y = p/W; const int nb[4] = {x > 0 ? p-1 : -1, x < W-1 ? p+1 : -1, y > 0 ? p-W : -1, y < H-1 ? p+W : -1};
            for (int q : nb) if (q >= 0 && d.z[q] > 0 && label[q] < 0 && std::fabs(d.z[q]-d.z[p]) < kJump) { label[q] = id; stack.push_back(q); }
        }
        sizes.push_back(count);
    }
    if (sizes.empty()) return;
    // A hand turning the plate is no part of the object, but moves along with it. While scanning the marker
    // map shows where the center is, and whatever stays far out from it is dropped.
    std::vector<uint8_t> farOut(sizes.size(), 0);
    if (region_.enabled && base_.valid && stats_.integratedFrames > 0) {
        std::vector<double> sum(sizes.size(), 0.0); std::vector<int> count(sizes.size(), 0);
        for (int y = 0; y < H; y += 2) for (int x = 0; x < W; x += 2) {
            const int p = y*W+x; if (d.z[p] <= 0 || label[p] < 0) continue;
            const float z = d.z[p]; const V3 w = baseCam_.apply(V3{(x-d.cx)/d.fx*z, (y-d.cy)/d.fy*z, z});
            sum[label[p]] += base_.radial(w); count[label[p]]++;
        }
        const float limit = .7f*base_.radius*region_.radiusScale;
        for (size_t i = 0; i < sizes.size(); i++) if (count[i] && sum[i]/count[i] > limit) farOut[i] = 1;
    }
    int largest = 0; for (size_t i = 0; i < sizes.size(); i++) if (!farOut[i]) largest = std::max(largest, sizes[i]);
    std::vector<float> out = d.z;
    #pragma omp parallel for
    for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) {
        const int p = y*W+x; if (d.z[p] <= 0) continue;
        if (farOut[label[p]] || sizes[label[p]] < kKeepShare*largest) { out[p] = 0; continue; }
        if (x < 1 || y < 1 || x >= W-1 || y >= H-1) { out[p] = 0; continue; }
        const int nb[4] = {p-1, p+1, p-W, p+W};
        for (int q : nb) if (d.z[q] <= 0 || std::fabs(d.z[q]-d.z[p]) >= kJump) { out[p] = 0; break; }
    }
    d.z.swap(out);
}
// True if this frame, at this pose, looks through what the model already knows to be solid.
bool Pipeline::seesThroughModel(const DepthMap& d, const Pose& pose) const {
    RayImage ray; const int w = d.width/4, h = d.height/4; if (w < 8 || h < 8) return false;
    tsdf_->raycast(pose, w, h, d.fx/4, d.fy/4, d.cx/4, d.cy/4, options.zMin, options.zMax, ray);
    int overlap = 0, through = 0;
    for (int j = 0; j < h; j++) for (int i = 0; i < w; i++) {
        const V3 n = ray.normal[j*w+i]; if (n.x == 0 && n.y == 0 && n.z == 0) continue;
        const float z = d.z[(size_t)(j*4)*d.width+i*4]; if (z <= 0) continue;
        const float zModel = pose.applyInv(ray.vertex[j*w+i]).z;
        overlap++; through += z > zModel+5.f;
    }
    return overlap >= 20 && through*5 > overlap;
}
void Pipeline::updateObjectDepth() {
    useObjectDepth_ = false; stats_.baseFound = false;
    if (base_.valid && options.useMarkers && (region_.enabled || options.removeBase)) {
        applyBase(depth_, base_, baseCam_, filtered_);
        if (options.cleanDepth) cleanDepth(filtered_);
        useObjectDepth_ = true; stats_.baseFound = true;
        return;
    }
    if (region_.enabled) {
        if (!region_.valid && stats_.integratedFrames == 0) locateObject(depth_);
        if (region_.valid) { applyRegion(depth_, filtered_); useObjectDepth_ = true; }
    }
}

// Remembers poses at intervals, so that tracking can resume when the scanner returns to known ground.
void Pipeline::rememberKeyframe() {
    constexpr float kMoveMm = 12.f, kTurnDeg = 8.f; constexpr size_t kMax = 256;
    for (const Pose& k : keyframes_) {
        float tr = 0; for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) tr += k.R[r*3+c]*pose_.R[r*3+c];
        const float ang = std::acos(std::clamp((tr-1)/2, -1.f, 1.f))*57.29578f;
        if (norm(k.t-pose_.t) < kMoveMm && ang < kTurnDeg) return;
    }
    if (keyframes_.size() >= kMax) keyframes_.erase(keyframes_.begin());
    keyframes_.push_back(pose_);
}
// Tries remembered poses in turn. A pose counts only if nearly all of the live depth fits the model.
bool Pipeline::relocalize(const DepthMap& d) {
    const Pose lost = pose_;
    const float keepInl = stats_.inlierRatio, keepRms = stats_.rmsMm; const int keepPts = stats_.icpPoints;
    // Several candidates may fit a smooth surface somewhat, so the best one wins, not the first.
    bool found = false; Pose bestPose; float bestScore = 1e9f, bestInl = 0, bestRms = 0; int bestPts = 0;
    auto consider = [&](const Pose& start) {
        pose_ = start;
        if (track(d) && stats_.inlierRatio >= options.relocMinInliers && stats_.rmsMm <= options.relocMaxRmsMm) {
            const float score = stats_.rmsMm/stats_.inlierRatio;
            if (score < bestScore) { bestScore = score; bestPose = pose_; bestInl = stats_.inlierRatio; bestRms = stats_.rmsMm; bestPts = stats_.icpPoints; found = true; }
        }
    };
    consider(lost);
    for (int t = 0; t < options.relocTriesPerFrame && t < (int)keyframes_.size(); t++) {
        relocCursor_ = (relocCursor_ + 1) % keyframes_.size();
        consider(keyframes_[relocCursor_]);
    }
    if (found) { pose_ = bestPose; stats_.inlierRatio = bestInl; stats_.rmsMm = bestRms; stats_.icpPoints = bestPts; stats_.relocations++; return true; }
    pose_ = lost; stats_.inlierRatio = keepInl; stats_.rmsMm = keepRms; stats_.icpPoints = keepPts;
    return false;
}

void Pipeline::trackAndIntegrate(const DepthMap& dIn) {
    std::lock_guard<std::mutex> g(mutex_);
    if (&dIn != &depth_) {   // called directly with a depth map, as the tests do
        if (region_.enabled && !region_.valid && stats_.integratedFrames == 0) locateObject(dIn);
        useObjectDepth_ = region_.enabled && region_.valid;
        if (useObjectDepth_) applyRegion(dIn, filtered_);
    }
    const DepthMap& d = useObjectDepth_ ? filtered_ : dIn;
    // Do not start the model from an empty or nearly empty frame: nothing could ever
    // be tracked against it. This happens right after start, before exposure has settled.
    if (stats_.integratedFrames == 0) {
        size_t valid = 0; for (float z : d.z) valid += z > 0;
        if (valid < 800*(size_t)(d.width/640)*(d.width/640) || (region_.enabled && !useObjectDepth_)) { stats_.state = TrackState::Waiting; region_.valid = false; return; }
    }
    lastFx_ = d.fx; lastFy_ = d.fy; lastCx_ = d.cx; lastCy_ = d.cy;
    auto t0 = Clock::now();
    bool ok = true;
    if (stats_.integratedFrames > 0) {
        if (hasMarkerPose_) {
            // Markers give the absolute pose. The surface may only fine-tune it, not move it far.
            const Pose before = pose_;
            pose_ = markerPose_;
            stats_.markerPose = true;
            if (markerWeak_) {
                // Only a suggestion: few markers, or found from distances, where a ring can look alike
                // when turned. Without the surface agreeing this frame stays out of the model.
                const Pose suggested = pose_;
                ok = track(d);
                if (ok) { V3 dt = pose_.t-suggested.t; if (norm(dt) > 6.f) ok = false; }
                if (!ok) {
                    // A side never seen before cannot agree. After some patience it counts, unless it contradicts the model.
                    if (++unconfirmed_ >= 15 && !seesThroughModel(d, suggested)) { pose_ = suggested; ok = true; }
                    else pose_ = before;
                }
                stats_.poseUnconfirmed = !ok;
                if (ok) { unconfirmed_ = 0; markers_.learn(); }
            } else if (options.refineWithShape) {
                const Pose m = pose_;
                if (track(d)) {
                    V3 dt = pose_.t - m.t;
                    float tr = 0; for (int r = 0; r < 3; r++) for (int k = 0; k < 3; k++) tr += pose_.R[r*3+k]*m.R[r*3+k];
                    float ang = std::acos(std::clamp((tr-1)/2, -1.f, 1.f));
                    if (norm(dt) > 2.f || ang > 0.6f/57.3f) pose_ = m;
                } else pose_ = m;
            }
            if (!markerWeak_) { unconfirmed_ = 0; stats_.poseUnconfirmed = false; }
            (void)before;
        } else {
            stats_.poseUnconfirmed = false;
            stats_.markerPose = false;
            // After a loss the old pose means nothing: only the strict search may resume tracking.
            ok = wasLost_ ? relocalize(d) : (track(d) || relocalize(d));
        }
        wasLost_ = !ok;
    }
    stats_.trackMs = msSince(t0);
    stats_.state = ok ? TrackState::Good : TrackState::Lost;
    t0 = Clock::now();
    if (ok) {
        if (stats_.integratedFrames == 0 && hasMarkerPose_) pose_ = markerPose_;   // the scan may have waited while the object turned
        const DepthMap& forModel = useDetail_ && &dIn == &depth_ ? detail_ : d;
        tsdf_->integrate(forModel, pose_, 64, colorReady_ ? &colorImage_ : nullptr, &colorCalib_); stats_.integratedFrames++;
        if (hasLastIntegrated_) { float tr = 0; for (int r = 0; r < 3; r++) for (int k = 0; k < 3; k++) tr += lastIntegrated_.R[r*3+k]*pose_.R[r*3+k]; const float a = std::acos(std::clamp((tr-1)/2, -1.f, 1.f))*57.29578f; stats_.turnDegPerFrame += (a-stats_.turnDegPerFrame)*0.2f; }
        lastIntegrated_ = pose_; hasLastIntegrated_ = true;
        rememberKeyframe();
        rememberFrame(forModel, currentFrame_);
    }
    else stats_.lostFrames++;
    stats_.fusionMs = msSince(t0);
    stats_.blocks = tsdf_->blockCount(); stats_.memoryBytes = tsdf_->memoryBytes();
}

void Pipeline::process(const StereoFrame& f, const StereoCalib& calib) {
    auto t0 = Clock::now();
    // Search only the shifts that the working distance allows. Even values, as the coarse stage steps by 2.
    const float fb = calib.focalPx*calib.baselineMm;
    const int shiftMin = (int)std::floor((calib.dispOffset - fb/options.zMin)/2)*2, shiftMax = (int)std::ceil((calib.dispOffset - fb/options.zMax)/2)*2;
    stereo.params.shiftMin = shiftMin; stereo.params.shiftMax = std::max(shiftMin+32, shiftMax);
    {
        // Where the object is known, refine only there. It is 0 for the first frames.
        std::lock_guard<std::mutex> g(mutex_);
        stereo.params.slantLimit = base_.valid || (region_.enabled && region_.valid);
        if (!base_.valid && stereo.params.slantLimit) {
            const V3 c = hasMarkerPose_ ? markerPose_.applyInv(region_.center) : region_.center;
            stereo.params.slantCx = c.x; stereo.params.slantCy = c.y; stereo.params.slantCz = c.z; stereo.params.slantR = region_.radius*region_.radiusScale;
        }
    }
    stereo.compute(f.ref.data(), f.other.data(), f.width, f.height, calib, depth_);
    stats_.stereoMs = msSince(t0);
    stats_.validDepth = depth_.validRatio();
    hasMarkerPose_ = false;
    {
        const bool wantMarkers = options.useMarkers;
        if (wantMarkers) detectMarkers(f, calib, shiftMin, stereo.params.shiftMax, detected_); else detected_.clear();
        std::lock_guard<std::mutex> g(mutex_);
        stats_.markersDetected = (int)detected_.size();
        if (wantMarkers && scanning_) {
            MarkerResult mr = markers_.update(detected_, pose_);
            stats_.markersDetected = (int)detected_.size();
            stats_.markersMatched = mr.matched; stats_.markerRmsMm = mr.rmsMm; stats_.markersInMap = (int)markers_.map().size();
            if (mr.ok && mr.usable && mr.rmsMm <= options.maxMarkerRmsMm) {
                hasMarkerPose_ = true; markerPose_ = mr.camToWorld;
                markerWeak_ = (mr.relocated || mr.matched < 5) && stats_.integratedFrames > 0;
                if (!markerWeak_) markers_.learn();
            }
        }
        // The plane comes from the marker map while scanning, and from this frame alone before that.
        if (wantMarkers && stats_.integratedFrames > 0) { baseCam_ = hasMarkerPose_ ? markerPose_ : pose_; base_ = markers_.basePlane(baseCam_.t); }
        else if (wantMarkers) {
            baseCam_ = Pose{}; std::vector<V3> pts; for (const Marker3D& m : detected_) pts.push_back(m.pos);
            base_ = fitBasePlane(pts, V3{});
        } else base_ = BasePlane{};
        basePartlySeen_ = false;
        for (const Marker3D& m : detected_) {
            if (m.pos.z <= 0) continue;
            const float u = (m.pos.x/m.pos.z*depth_.fx+depth_.cx)/depth_.width, v = (m.pos.y/m.pos.z*depth_.fy+depth_.cy)/depth_.height;
            if (u < .1f || u > .9f || v < .1f || v > .9f) { basePartlySeen_ = true; break; }
        }
        updateObjectDepth();
        if (base_.valid) {   // refine slopes only where the object can be
            const V3 cc = baseCam_.applyInv(base_.center);
            stereo.params.slantLimit = true; stereo.params.slantCx = cc.x; stereo.params.slantCy = cc.y; stereo.params.slantCz = cc.z; stereo.params.slantR = std::max(base_.radius*region_.radiusScale*1.5f, basePartlySeen_ ? 180.f : 0.f);
        }
        // Distance shown to the user: of the object if known, otherwise of everything in view.
        const DepthMap& shown = useObjectDepth_ ? filtered_ : depth_;
        std::vector<float> s; s.reserve(4000);
        for (size_t i = 0; i < shown.z.size(); i += 4) if (shown.z[i] > 0) s.push_back(shown.z[i]);
        if (!s.empty()) { std::nth_element(s.begin(), s.begin()+s.size()/2, s.end()); stats_.medianDepth = s[s.size()/2]; } else stats_.medianDepth = 0;
    }
    // Color adds up quickly, so a share of the frames is enough and saves decoding time.
    colorReady_ = false;
    if (scanning_ && colorCalib_.valid && !f.colorJpeg.empty() && ++colorCounter_ % 3 == 0) colorReady_ = decodeJpeg(f.colorJpeg, colorImage_);
    currentFrame_ = &f;
    useDetail_ = false;
    // Depth noise grows with the square of the distance. From afar a small window shows more noise than
    // surface (measured on real recordings at 29 cm), so it is used only when close.
    constexpr float kDetailMaxDistance = 240.f;
    if (scanning_ && options.detailDepth && stats_.medianDepth > 0 && stats_.medianDepth <= kDetailMaxDistance) {
        // Only the object, as this costs per cell. A whole room in view would take too long per frame.
        const DepthMap& object = useObjectDepth_ ? filtered_ : depth_;
        constexpr size_t kMaxCells = 80000; size_t cells = 0; for (float z : object.z) cells += z > 0;
        if (cells && cells <= kMaxCells) { detail_ = object; stereo.refineDetail(f.ref.data(), f.other.data(), f.width, f.height, calib, detail_); useDetail_ = true; }
    }
    if (scanning_) trackAndIntegrate(depth_);
    else if (stats_.state != TrackState::Idle) stats_.state = TrackState::Idle;
    currentFrame_ = nullptr;
    stats_.totalMs = msSince(t0);
}

int Pipeline::nextExposure(const StereoFrame& f, int current) const {
    const DepthMap& d = objectDepth(); constexpr int kWindow = 4; std::vector<int> peaks;
    if (d.width > 0) {
        const int step = f.width/d.width; size_t index = 0;
        for (int y = 0; y < d.height; y++) for (int x = 0; x < d.width; x++, index++) {
            if (d.z[index] <= 0 || index%5) continue;   // a share of the cells is enough
            const int X = step*x, Y = step*y; if (X < kWindow || Y < kWindow || X+kWindow >= f.width || Y+kWindow >= f.height) continue;
            int m = 0; for (int dy = -kWindow; dy <= kWindow; dy++) { const uint8_t* row = f.ref.data()+(size_t)(Y+dy)*f.width+X; for (int dx = -kWindow; dx <= kWindow; dx++) m = std::max<int>(m, row[dx]); }
            peaks.push_back(m);
        }
    }
    if (peaks.size() < 100) return adjustExposure(f.ref, current);   // no object in view yet
    size_t saturated = 0; for (int v : peaks) saturated += v >= 250;
    const float share = (float)saturated/peaks.size();
    std::nth_element(peaks.begin(), peaks.begin()+peaks.size()*95/100, peaks.end()); const int p95 = peaks[peaks.size()*95/100];
    int next = current;
    if (share > .10f) next = (int)(current/1.5f);
    else if (share > .03f) next = (int)(current/1.2f);
    else if (p95 < 170) next = (int)(current*1.15f)+1;
    return std::clamp(next, 200, 12000);
}

void Pipeline::renderPreview(int w, int h, std::vector<uint8_t>& gray) {
    std::lock_guard<std::mutex> g(mutex_);
    gray.assign(w*h, 0);
    if (!lastFx_ || stats_.integratedFrames == 0) return;
    const float sx = (float)w/(lastCx_*2 > 0 ? depth_.width : 1), sy = (float)h/depth_.height;
    RayImage img;
    tsdf_->raycast(pose_, w, h, lastFx_*sx, lastFy_*sy, lastCx_*sx, lastCy_*sy, options.zMin, options.zMax, img);
    V3 light = normalized(pose_.rot(V3{0.2f, -0.3f, -1.f}));
    for (int i = 0; i < w*h; i++) {
        V3 n = img.normal[i];
        if (n.x == 0 && n.y == 0 && n.z == 0) continue;
        float l = std::max(0.f, dot(n, light));
        gray[i] = (uint8_t)(40 + 215*l);
    }
}
void Pipeline::rememberFrame(const DepthMap& d, const StereoFrame* f) {
    // A very long scan must not fill the memory. Scanning goes on, only the rebuild is lost.
    constexpr size_t kMaxBytes = size_t(6)*1024*1024*1024;
    if (sessionBytes_ > kMaxBytes) { if (!session_.empty()) { session_.clear(); session_.shrink_to_fit(); } return; }
    ScanFrame r; r.pose = pose_;
    for (size_t i = 0; i < d.z.size(); i++) if (d.z[i] > 0) { r.cells.push_back((uint32_t)i); r.z.push_back(d.z[i]); }
    if (f && colorReady_) r.colorJpeg = f->colorJpeg;
    if (session_.empty()) { sessionGrid_ = d; sessionGrid_.z.clear(); }
    sessionBytes_ += r.cells.size()*8 + r.colorJpeg.size() + sizeof(ScanFrame);
    session_.push_back(std::move(r));
}
bool Pipeline::rebuildModel(float voxelMm, std::string& report) {
    std::lock_guard<std::mutex> g(mutex_);
    if (session_.empty()) { report = sessionBytes_ ? "Der Scan war zu lang, um ihn im Speicher zu behalten" : "Kein Scan im Speicher"; return false; }
    options.voxelMm = voxelMm;
    tsdf_ = std::make_unique<Tsdf>(voxelMm);
    DepthMap d = sessionGrid_; d.z.assign((size_t)d.width*d.height, 0.f);
    ColorImage img;
    for (const ScanFrame& fr : session_) {
        std::fill(d.z.begin(), d.z.end(), 0.f);
        for (size_t i = 0; i < fr.cells.size(); i++) d.z[fr.cells[i]] = fr.z[i];
        const bool col = colorCalib_.valid && !fr.colorJpeg.empty() && decodeJpeg(fr.colorJpeg, img);
        tsdf_->integrate(d, fr.pose, 64, col ? &img : nullptr, &colorCalib_);
    }
    pose_ = session_.back().pose;
    stats_.blocks = tsdf_->blockCount(); stats_.memoryBytes = tsdf_->memoryBytes();
    char b[120]; snprintf(b, sizeof b, "Neu berechnet mit %.2f mm aus %zu Bildern", voxelMm, session_.size());
    report = b;
    return true;
}
bool Pipeline::takeScan(ScanData& out) {
    {
        std::lock_guard<std::mutex> g(mutex_);
        if (session_.empty()) return false;
        out.frames = std::move(session_); session_.clear();
        out.grid = sessionGrid_; out.grid.z.clear(); out.colorCalib = colorCalib_;
        out.markers.clear(); for (const MapMarker& m : markers_.map()) if (m.seen >= 3) out.markers.push_back(m.pos);
        const BasePlane b = markers_.basePlane(pose_.t);
        out.up = b.valid ? b.normal : pose_.rot(cameraUp(options.roll));
    }
    reset();
    return true;
}
void Pipeline::buildMesh(const MeshOptions& opt, Mesh& out) {
    std::lock_guard<std::mutex> g(mutex_);
    const BasePlane b = markers_.basePlane(pose_.t);
    const V3 up = b.valid ? b.normal : pose_.rot(cameraUp(options.roll));
    if (opt.cleanUp && !session_.empty()) {
        // The same path as several scans: fine alignment, clean-up and color all need the frames.
        Project one; one.scans.emplace_back(); ScanData& s = one.scans[0];
        s.frames.swap(session_); s.grid = sessionGrid_; s.colorCalib = colorCalib_; s.up = up; s.placed = true;
        std::string error; const bool ok = fuseProject(one, tsdf_->voxel(), opt, out, error);
        s.frames.swap(session_);
        if (ok) return;
        out = Mesh{};
    }
    extractMesh(*tsdf_, opt, out);
    if (opt.upright) orientUpright(out, up);
}
