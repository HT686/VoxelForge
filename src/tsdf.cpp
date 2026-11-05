#include "tsdf.hpp"
#include <omp.h>
#include <algorithm>
#include <cmath>

static inline uint64_t hashKey(int64_t k) { uint64_t x = (uint64_t)k; x ^= x >> 33; x *= 0xff51afd7ed558ccdULL; x ^= x >> 33; return x; }

Tsdf::Tsdf(float voxelMm, float truncMm) : voxel_(voxelMm), trunc_(truncMm > 0 ? truncMm : voxelMm * 6.f) {
    table_.assign(1 << 16, Slot{-1, -1}); mask_ = table_.size() - 1;
}
void Tsdf::clear() { boundsLo_ = V3{1e30f, 1e30f, 1e30f}; boundsHi_ = V3{-1e30f, -1e30f, -1e30f}; blocks_.clear(); blockPtr_.clear(); table_.assign(1 << 16, Slot{-1, -1}); mask_ = table_.size() - 1; }

Block* Tsdf::find(int bx, int by, int bz) const {
    int64_t k = packKey(bx, by, bz);
    for (size_t i = hashKey(k) & mask_;; i = (i + 1) & mask_) {
        if (table_[i].key == k) return blockPtr_[table_[i].index];
        if (table_[i].key == -1) return nullptr;
    }
}
void Tsdf::grow() {
    std::vector<Slot> old = std::move(table_);
    table_.assign(old.size() * 2, Slot{-1, -1}); mask_ = table_.size() - 1;
    for (const Slot& s : old) {
        if (s.key == -1) continue;
        size_t i = hashKey(s.key) & mask_;
        while (table_[i].key != -1) i = (i + 1) & mask_;
        table_[i] = s;
    }
}
Block* Tsdf::obtain(int bx, int by, int bz) {
    int64_t k = packKey(bx, by, bz);
    size_t i = hashKey(k) & mask_;
    for (; table_[i].key != -1; i = (i + 1) & mask_)
        if (table_[i].key == k) return blockPtr_[table_[i].index];
    blocks_.emplace_back();
    Block& b = blocks_.back(); b.x = bx; b.y = by; b.z = bz;
    { const float s = voxel_*kBlockSide; const V3 lo{bx*s, by*s, bz*s}, hi{(bx+1)*s, (by+1)*s, (bz+1)*s};
      boundsLo_ = V3{std::min(boundsLo_.x, lo.x), std::min(boundsLo_.y, lo.y), std::min(boundsLo_.z, lo.z)}; boundsHi_ = V3{std::max(boundsHi_.x, hi.x), std::max(boundsHi_.y, hi.y), std::max(boundsHi_.z, hi.z)}; }
    blockPtr_.push_back(&b);
    table_[i] = Slot{k, (int32_t)blockPtr_.size() - 1};
    if (blockPtr_.size() * 2 > table_.size()) grow();
    return &b;
}

void Tsdf::computePixelWeights(const DepthMap& d, int x0, int y0, int x1, int y1) {
    // All of this happens only inside [x0, x1) x [y0, y1], the rectangle around the depth with some margin. Outside
    // there is no depth, so nothing changes. The object often covers a few percent of the picture.
    const int w = d.width; constexpr float kFar = 1e6f; constexpr int kFeatherCells = 10;
    const int rw = x1-x0, rh = y1-y0, r1 = rw+1;
    dist_.resize((size_t)rw*rh); integral_.resize((size_t)r1*(rh+1)); std::fill(integral_.begin(), integral_.begin()+r1, 0);
    std::vector<int>& I = integral_; std::vector<float>& dist = dist_;
    // Distance to the edge of valid depth, in cells (two chamfer passes).
    // Small gaps between the projected dots are no edge: a cell counts if depth lies within two cells.
    for (int y = 0; y < rh; y++) { int r = 0; I[(size_t)(y+1)*r1] = 0; for (int x = 0; x < rw; x++) { r += d.z[(size_t)(y+y0)*w+x+x0] > 0; I[(size_t)(y+1)*r1+x+1] = I[(size_t)y*r1+x+1]+r; } }
    #pragma omp parallel for schedule(static)
    for (int y = 0; y < rh; y++) for (int x = 0; x < rw; x++) {
        const int ax = std::max(0, x-2), bx = std::min(rw, x+3), ay = std::max(0, y-2), by = std::min(rh, y+3);
        dist[(size_t)y*rw+x] = I[(size_t)by*r1+bx]-I[(size_t)ay*r1+bx]-I[(size_t)by*r1+ax]+I[(size_t)ay*r1+ax] >= 6 ? kFar : 0;
    }
    for (int y = 1; y < rh-1; y++) for (int x = 1; x < rw-1; x++) { const size_t p = (size_t)y*rw+x; float& v = dist[p]; v = std::min({v, dist[p-1]+1, dist[p-rw]+1, dist[p-rw-1]+1.4f, dist[p-rw+1]+1.4f}); }
    for (int y = rh-2; y >= 1; y--) for (int x = rw-2; x >= 1; x--) { const size_t p = (size_t)y*rw+x; float& v = dist[p]; v = std::min({v, dist[p+1]+1, dist[p+rw]+1, dist[p+rw+1]+1.4f, dist[p+rw-1]+1.4f}); }
    pixelWeight_.assign((size_t)w*d.height, 0.f);
    const float ifx = 1.f/std::fabs(d.fx), ify = 1.f/d.fy;
    #pragma omp parallel for schedule(static)
    for (int y = 1; y < rh-1; y++) for (int x = 1; x < rw-1; x++) {
        const int p = (y+y0)*w+x+x0; const float z = d.z[p]; if (z <= 0) continue;
        const float edge = std::min(1.f, dist[(size_t)y*rw+x]/kFeatherCells);
        // Slope from the neighbours gives the angle between surface and view.
        const float zl = d.z[p-1], zr = d.z[p+1], zu = d.z[p-w], zd = d.z[p+w];
        float dzx = 0, dzy = 0;
        if (zl > 0 && zr > 0) dzx = (zr-zl)*0.5f; else if (zr > 0) dzx = zr-z; else if (zl > 0) dzx = z-zl;
        if (zu > 0 && zd > 0) dzy = (zd-zu)*0.5f; else if (zd > 0) dzy = zd-z; else if (zu > 0) dzy = z-zu;
        const float gx = dzx/(z*ifx), gy = dzy/(z*ify);   // depth change per mm sideways
        const float cos2 = 1.f/(1.f+gx*gx+gy*gy);
        pixelWeight_[p] = std::max(0.05f, edge*edge*(3-2*edge)*cos2);
    }
}
void Tsdf::integrate(const DepthMap& d, const Pose& T, float maxWeight, const ColorImage* color, const ColorCalib* cc) {
    const float bs = voxel_ * kBlockSide;
    const int w = d.width, h = d.height;
    // The rectangle holding all depth, with room for the steps below.
    int bx0 = w, by0 = h, bx1 = -1, by1 = -1;
    for (int y = 0; y < h; y++) { const float* row = d.z.data()+(size_t)y*w; for (int x = 0; x < w; x++) if (row[x] > 0) { if (x < bx0) bx0 = x; if (x > bx1) bx1 = x; if (y < by0) by0 = y; by1 = y; } }
    if (bx1 < 0) { pixelWeight_.assign((size_t)w*h, 0.f); return; }   // an empty frame adds nothing
    if (feather) { constexpr int kMargin = 4; computePixelWeights(d, std::max(0, bx0-kMargin), std::max(0, by0-kMargin), std::min(w, bx1+1+kMargin), std::min(h, by1+1+kMargin)); }
    // Blocks near the measured surface. Sampled on a sparse grid: a block spans many pixels.
    int tn = omp_get_max_threads();
    std::vector<std::vector<int64_t>> local(tn);
    const float ifx = 1.f/d.fx, ify = 1.f/d.fy;
    const int stride = std::max(1, (int)(bs / 2 / (300.f * std::fabs(ifx))));
    #pragma omp parallel for schedule(dynamic, 8)
    for (int j = by0-by0%stride; j <= by1; j += stride) {
        auto& out = local[omp_get_thread_num()];
        constexpr size_t kRecent = 1024; int64_t recent[kRecent]; std::fill(recent, recent+kRecent, (int64_t)-1);
        for (int i = bx0-bx0%stride; i <= bx1; i += stride) {
            float z = d.z[j*w+i];
            if (z <= 0) continue;
            V3 ray{(i - d.cx)*ifx, (j - d.cy)*ify, 1.f};
            float n = norm(ray);
            for (float q = z - trunc_; q <= z + trunc_ + bs/2; q += bs/2) {
                V3 p = T.apply(ray * q);
                (void)n;
                int bx = (int)std::floor(p.x/bs), by = (int)std::floor(p.y/bs), bz = (int)std::floor(p.z/bs);
                // Neighbouring cells mostly reach the same blocks. Remembering the last ones here saves collecting them
                // millions of times and sorting them afterwards on one core.
                const int64_t key = packKey(bx, by, bz); int64_t& seen = recent[hashKey(key)&(kRecent-1)];
                if (seen != key) { seen = key; out.push_back(key); }
            }
        }
    }
    touched_.clear();
    for (auto& v : local) { touched_.insert(touched_.end(), v.begin(), v.end()); }
    std::sort(touched_.begin(), touched_.end());
    touched_.erase(std::unique(touched_.begin(), touched_.end()), touched_.end());
    std::vector<Block*> active; active.reserve(touched_.size());
    for (int64_t k : touched_) {
        int bx = (int)((k >> 42) & 0x1fffff) - (1<<20), by = (int)((k >> 21) & 0x1fffff) - (1<<20), bz = (int)(k & 0x1fffff) - (1<<20);
        active.push_back(obtain(bx, by, bz));
    }
    const float itrunc = 1.f/trunc_;
    #pragma omp parallel for schedule(dynamic, 16)
    for (size_t a = 0; a < active.size(); a++) {
        Block& b = *active[a];
        for (int k = 0; k < kBlockSide; k++) for (int jj = 0; jj < kBlockSide; jj++) for (int ii = 0; ii < kBlockSide; ii++) {
            V3 p{(b.x*kBlockSide + ii)*voxel_, (b.y*kBlockSide + jj)*voxel_, (b.z*kBlockSide + k)*voxel_};
            V3 c = T.applyInv(p);
            if (c.z <= 1.f) continue;
            int u = (int)(c.x/c.z*d.fx + d.cx + 0.5f), v = (int)(c.y/c.z*d.fy + d.cy + 0.5f);
            if (u < 0 || v < 0 || u >= w || v >= h) continue;
            float dz = d.z[v*w+u];
            // Between the cells the depth is interpolated, or the grid of cells prints into the surface.
            {
                const float fu = c.x/c.z*d.fx + d.cx, fv = c.y/c.z*d.fy + d.cy; const int u0 = (int)std::floor(fu), v0 = (int)std::floor(fv);
                if (u0 >= 0 && v0 >= 0 && u0+1 < w && v0+1 < h) {
                    const float a = d.z[v0*w+u0], b = d.z[v0*w+u0+1], e = d.z[(v0+1)*w+u0], g = d.z[(v0+1)*w+u0+1];
                    if (a > 0 && b > 0 && e > 0 && g > 0 && std::max({a, b, e, g})-std::min({a, b, e, g}) < 1.5f) {
                        const float tu = fu-u0, tv = fv-v0; dz = (a*(1-tu)+b*tu)*(1-tv)+(e*(1-tu)+g*tu)*tv;
                    }
                }
            }
            // A voxel larger than a depth cell covers several cells, and single ones may be gaps
            // between the projected dots. Use the valid cells it covers, on the same surface.
            const int footprint = std::min(3, (int)(voxel_*std::fabs(d.fx)/c.z*0.5f));
            if (footprint > 0) {
                float sum = 0; int n = 0;
                for (int dv = -footprint; dv <= footprint; dv++) for (int du = -footprint; du <= footprint; du++) {
                    const int uu = u+du, vv = v+dv;
                    if (uu < 0 || vv < 0 || uu >= w || vv >= h) continue;
                    const float q = d.z[vv*w+uu];
                    if (q > 0 && std::fabs(q-c.z) <= trunc_*1.5f) { sum += q; n++; }
                }
                if (n > 0) dz = sum/n;
            }
            if (dz <= 0) continue;
            float sdf = dz - c.z;
            if (sdf < -trunc_) continue;
            float t = std::min(1.f, sdf*itrunc);
            Voxel& vx = b.v[(k*kBlockSide + jj)*kBlockSide + ii];
            // At least one step, so that a voxel never stays unseen because of a small weight.
            const float add = feather ? std::max(1.f, kWeightScale*pixelWeight_[v*w+u]) : (float)kWeightScale;
            const float W = vx.weight;
            vx.tsdf = (vx.tsdf*W + t*add)/(W + add);
            vx.weight = (uint16_t)std::min(maxWeight*kWeightScale, W + add);
            if (vx.seen < 255) vx.seen++;
            // Color only from voxels right at the surface, seen from this frame's color camera.
            if (color && std::fabs(sdf) <= voxel_*2.f) {
                const V3 cp = cc->refToColor.apply(c);
                if (cp.z > 1.f) {
                    const int cu = (int)(cp.x/cp.z*cc->focalPx + cc->cx + .5f), cv = (int)(cp.y/cp.z*cc->focalPx + cc->cy + .5f);
                    if (cu >= 0 && cv >= 0 && cu < color->width && cv < color->height) {
                        const uint8_t* px = color->rgb.data() + 3*((size_t)cv*color->width + cu);
                        const int cw = vx.colorWeight;
                        vx.r = (uint8_t)((vx.r*cw + px[0])/(cw+1)); vx.g = (uint8_t)((vx.g*cw + px[1])/(cw+1)); vx.b = (uint8_t)((vx.b*cw + px[2])/(cw+1));
                        if (cw < 30) vx.colorWeight = (uint8_t)(cw+1);
                    }
                }
            }
        }
    }
}

namespace {
struct Sampler {
    const Tsdf& t; mutable Block* last = nullptr; mutable int lx = 1<<30, ly = 0, lz = 0;
    const Voxel* get(int x, int y, int z) const {
        int bx = x >> 3, by = y >> 3, bz = z >> 3;
        if (bx != lx || by != ly || bz != lz) { last = t.find(bx, by, bz); lx = bx; ly = by; lz = bz; }
        if (!last) return nullptr;
        return &last->v[(((z & 7)*8 + (y & 7))*8) + (x & 7)];
    }
    // Trilinear TSDF at p (voxel units). False if a corner has never been observed.
    bool trilinear(float px, float py, float pz, float& out) const {
        int x = (int)std::floor(px), y = (int)std::floor(py), z = (int)std::floor(pz);
        float fx = px-x, fy = py-y, fz = pz-z, val = 0;
        for (int k = 0; k < 8; k++) {
            const Voxel* v = get(x + (k&1), y + ((k>>1)&1), z + ((k>>2)&1));
            if (!v || v->weight == 0) return false;
            val += v->tsdf * ((k&1) ? fx : 1-fx) * ((k&2) ? fy : 1-fy) * ((k&4) ? fz : 1-fz);
        }
        out = val; return true;
    }
};
}

void Tsdf::raycast(const Pose& T, int w, int h, float fx, float fy, float cx, float cy, float zMin, float zMax, RayImage& out) const {
    out.width = w; out.height = h;
    out.vertex.assign(w*h, V3{}); out.normal.assign(w*h, V3{});
    const float iv = 1.f/voxel_, bs = voxel_*kBlockSide;
    #pragma omp parallel for schedule(dynamic, 4)
    for (int j = 0; j < h; j++) {
        Sampler s{*this};
        for (int i = 0; i < w; i++) {
            V3 dir = T.rot(V3{(i-cx)/fx, (j-cy)/fy, 1.f});  // advancing z by 1 mm moves this much in world
            float z = zMin, prev = 1.f; bool have = false, misses = false; float rayEnd = zMax;
            // Nothing exists outside the blocks' bounds: start where the ray enters them and stop where it leaves.
            // One voxel of margin, as the search reads neighbours.
            const auto slab = [&](float o, float d, float lo, float hi) {
                if (std::fabs(d) < 1e-9f) { misses = misses || o < lo || o > hi; return; }
                float a = (lo-o)/d, b = (hi-o)/d; if (a > b) std::swap(a, b);
                z = std::max(z, a); rayEnd = std::min(rayEnd, b);
            };
            slab(T.t.x, dir.x, boundsLo_.x-voxel_, boundsHi_.x+voxel_); slab(T.t.y, dir.y, boundsLo_.y-voxel_, boundsHi_.y+voxel_); slab(T.t.z, dir.z, boundsLo_.z-voxel_, boundsHi_.z+voxel_);
            if (misses) continue;
            while (z < rayEnd) {
                V3 p = T.t + dir*z;
                int bx = (int)std::floor(p.x/bs), by = (int)std::floor(p.y/bs), bz = (int)std::floor(p.z/bs);
                if (!find(bx, by, bz)) { z += bs*0.7f; have = false; continue; }
                float val;
                if (!s.trilinear(p.x*iv, p.y*iv, p.z*iv, val)) { z += voxel_; have = false; continue; }
                if (have && prev > 0 && val <= 0) {
                    float step = std::max(voxel_, prev*trunc_*0.7f);
                    float zHit = z - step*(val/(val-prev)) ;
                    V3 q = T.t + dir*zHit;
                    float a, b; bool ok = true; V3 g;
                    float qx = q.x*iv, qy = q.y*iv, qz = q.z*iv;
                    ok &= s.trilinear(qx+1, qy, qz, a) && s.trilinear(qx-1, qy, qz, b); g.x = a-b;
                    ok = ok && s.trilinear(qx, qy+1, qz, a) && s.trilinear(qx, qy-1, qz, b); g.y = a-b;
                    ok = ok && s.trilinear(qx, qy, qz+1, a) && s.trilinear(qx, qy, qz-1, b); g.z = a-b;
                    if (ok && dot(g, g) > 1e-10f) { out.vertex[j*w+i] = q; out.normal[j*w+i] = normalized(g); }
                    break;
                }
                if (have && prev < 0 && val > 0) break; // back side
                prev = val; have = true;
                z += std::max(voxel_, val*trunc_*0.7f);
            }
        }
    }
}
