#include "project.hpp"
#include <filesystem>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include "meshedit.hpp"
#include "pointgrid.hpp"
#include "tsdf.hpp"

size_t ScanData::bytes() const {
    size_t b = 0; for (const ScanFrame& f : frames) b += f.cells.size()*8 + f.colorJpeg.size() + sizeof(ScanFrame);
    return b;
}
std::string Project::nextName() const {
    for (int i = (int)scans.size()+1;; i++) {
        std::string n = "Scan_" + std::to_string(i); bool used = false;
        for (const ScanData& s : scans) used |= s.name == n;
        if (!used) return n;
    }
}
namespace {
void fillDepth(const ScanFrame& f, DepthMap& d) {
    std::fill(d.z.begin(), d.z.end(), 0.f);
    for (size_t i = 0; i < f.cells.size(); i++) d.z[f.cells[i]] = f.z[i];
}
}
size_t carveFreeSpace(Mesh& m, const std::vector<ScanView>& scans, float tol) {
    const size_t nv = m.vertices.size(); if (!nv) return 0;
    std::vector<uint32_t> agree(nv, 0), through(nv, 0);
    for (const ScanView& s : scans) {
        const int W = s.grid->width, H = s.grid->height; const float fx = s.grid->fx, fy = s.grid->fy, cx = s.grid->cx, cy = s.grid->cy;
        const size_t step = std::max<size_t>(1, s.frames->size()/300), count = s.frames->size()/step+1;
        #pragma omp parallel
        {
            std::vector<float> d((size_t)W*H, 0.f); std::vector<uint32_t> la(nv, 0), lt(nv, 0);
            #pragma omp for schedule(dynamic, 4)
            for (size_t q = 0; q < count; q++) {
                if (q*step >= s.frames->size()) continue;
                const ScanFrame& f = (*s.frames)[q*step];
                for (size_t i = 0; i < f.cells.size(); i++) d[f.cells[i]] = f.z[i];
                const Pose P = s.frameToProject(q*step);
                for (size_t i = 0; i < nv; i++) {
                    const V3 c = P.applyInv(m.vertices[i]); if (c.z <= 1) continue;
                    const int x = (int)std::lround(fx*c.x/c.z+cx), y = (int)std::lround(fy*c.y/c.z+cy);
                    if (x < 1 || y < 1 || x >= W-1 || y >= H-1) continue;
                    // Looks through only if all valid neighbours lie behind: a silhouette or a small pose error must not count.
                    bool any = false, near = false, behind = true;
                    for (int b = -1; b <= 1; b++) for (int a = -1; a <= 1; a++) {
                        const float z = d[(size_t)(y+b)*W+x+a]; if (z <= 0) continue;
                        any = true; if (std::fabs(z-c.z) < tol) near = true; if (z < c.z+tol) behind = false;
                    }
                    if (!any) continue;
                    if (near) la[i]++; else if (behind) lt[i]++;
                }
                for (uint32_t cell : f.cells) d[cell] = 0;
            }
            #pragma omp critical
            for (size_t i = 0; i < nv; i++) { agree[i] += la[i]; through[i] += lt[i]; }
        }
    }
    std::vector<uint8_t> mark(m.triangleCount(), 0); size_t removed = 0;
    for (size_t t = 0; t < mark.size(); t++) {
        int bad = 0;
        for (int k = 0; k < 3; k++) { const uint32_t v = m.indices[3*t+k]; if (through[v] >= 3 && through[v] > 0.3f*(agree[v]+through[v])) bad++; }
        if (bad >= 2) { mark[t] = 1; removed++; }
    }
    if (removed) { deleteTriangles(m, mark); computeNormals(m); }
    return removed;
}
static constexpr int kDarkestUsable = 70;
float colorFromFrames(Mesh& m, const std::vector<ScanView>& scans, float tol) {
    const size_t nv = m.vertices.size(); if (!nv) return 0;
    if (m.normals.size() != nv) computeNormals(m);
    std::vector<float> sum(nv*4, 0.f); ColorImage img; std::vector<float> d; bool any = false;
    constexpr int kCoarse = 8, coarseW = 1280/kCoarse+1, coarseH = 800/kCoarse+1; std::vector<float> nearest((size_t)coarseW*coarseH);
    for (const ScanView& s : scans) {
        if (!s.colorCalib->valid) continue;
        any = true;
        const int W = s.grid->width, H = s.grid->height; const float fx = s.grid->fx, fy = s.grid->fy, cx = s.grid->cx, cy = s.grid->cy; const ColorCalib& cc = *s.colorCalib;
        d.assign((size_t)W*H, 0.f);
        size_t withColor = 0; for (const ScanFrame& f : *s.frames) withColor += !f.colorJpeg.empty();
        const size_t step = std::max<size_t>(1, withColor/300); size_t seen = 0;
        for (size_t fi = 0; fi < s.frames->size(); fi++) {
            const ScanFrame& f = (*s.frames)[fi];
            if (f.colorJpeg.empty() || (seen++ % step) || !decodeJpeg(f.colorJpeg, img)) continue;
            for (size_t i = 0; i < f.cells.size(); i++) d[f.cells[i]] = f.z[i];
            // The color camera sits beside the depth camera, so at edges it sees past or behind what
            // the depth camera sees. What it sees first, on a coarse grid, decides.
            std::fill(nearest.begin(), nearest.end(), 1e9f);
            for (size_t i = 0; i < f.cells.size(); i++) {
                const int gx = (int)(f.cells[i]%W), gy = (int)(f.cells[i]/W); const float z = f.z[i];
                const V3 cp = cc.refToColor.apply(V3{(gx-cx)/fx*z, (gy-cy)/fy*z, z}); if (cp.z <= 1) continue;
                const int bx = (int)(cp.x/cp.z*cc.focalPx+cc.cx)/kCoarse, by = (int)(cp.y/cp.z*cc.focalPx+cc.cy)/kCoarse;
                for (int b = -1; b <= 1; b++) for (int a = -1; a <= 1; a++) { const int xx = bx+a, yy = by+b; if (xx >= 0 && yy >= 0 && xx < coarseW && yy < coarseH) { float& nz = nearest[(size_t)yy*coarseW+xx]; nz = std::min(nz, cp.z); } }
            }
            const Pose P = s.frameToProject(fi);
            #pragma omp parallel for schedule(static)
            for (size_t i = 0; i < nv; i++) {
                const V3 c = P.applyInv(m.vertices[i]); if (c.z <= 1) continue;
                const V3 n = P.rotInv(m.normals[i]);
                const float facing = -dot(n, normalized(c)); if (facing < 0.35f) continue;
                const int x = (int)std::lround(fx*c.x/c.z+cx), y = (int)std::lround(fy*c.y/c.z+cy);
                if (x < 1 || y < 1 || x >= W-1 || y >= H-1) continue;
                // This frame must really see this surface here.
                bool near = false; for (int b = -1; b <= 1 && !near; b++) for (int a = -1; a <= 1 && !near; a++) { const float z = d[(size_t)(y+b)*W+x+a]; near = z > 0 && std::fabs(z-c.z) < tol; }
                if (!near) continue;
                const V3 cp = cc.refToColor.apply(c); if (cp.z <= 1) continue;
                const float fu = cp.x/cp.z*cc.focalPx+cc.cx, fv = cp.y/cp.z*cc.focalPx+cc.cy;
                const int u0 = (int)std::floor(fu), v0 = (int)std::floor(fv); if (u0 < 0 || v0 < 0 || u0+1 >= img.width || v0+1 >= img.height) continue;
                if (cp.z > nearest[(size_t)(v0/kCoarse)*coarseW+u0/kCoarse]+tol) continue;
                const float au = fu-u0, av = fv-v0; const uint8_t* p = img.rgb.data()+3*((size_t)v0*img.width+u0); const int st = img.width*3;
                const float f2 = facing*facing, w = f2*f2*f2;   // frontal views dominate, which keeps the image sharp
                for (int k = 0; k < 3; k++) sum[i*4+k] += w*((1-av)*((1-au)*p[k]+au*p[k+3])+av*((1-au)*p[st+k]+au*p[st+k+3]));
                sum[i*4+3] += w;
            }
            for (uint32_t cell : f.cells) d[cell] = 0;
        }
    }
    if (!any) return 0;
    size_t done = 0; for (size_t i = 0; i < nv; i++) done += sum[i*4+3] > 1e-4f;
    if (!done) return 0;   // no frame shows this surface: leave the mesh as it is
    if (m.colors.size() != nv*3) m.colors.assign(nv*3, 128);
    for (size_t i = 0; i < nv; i++) if (sum[i*4+3] > 1e-4f) for (int k = 0; k < 3; k++) m.colors[i*3+k] = (uint8_t)std::clamp(sum[i*4+k]/sum[i*4+3], 0.f, 255.f);
    // Rooms are often dim. One brightness factor for everything leaves the hues alone.
    { std::vector<uint32_t> hist(256, 0); for (size_t i = 0; i < nv; i++) if (sum[i*4+3] > 1e-4f) hist[std::max({m.colors[i*3], m.colors[i*3+1], m.colors[i*3+2]})]++;
      size_t acc = 0; int top = 255; for (int v = 255; v >= 0; v--) { acc += hist[v]; if (acc > done/100) { top = v; break; } }
      // Too dark to tell colors apart: the color camera shows mostly noise and the tint of the room.
      if (top < kDarkestUsable) { m.colors.clear(); return 0; }
      const float gain = std::clamp(235.f/std::max(1, top), 1.f, 3.f);
      if (gain > 1.01f) for (uint8_t& c : m.colors) c = (uint8_t)std::min(255.f, c*gain); }
    return (float)done/nv;
}
namespace {
void modelOfHalf(const std::vector<ScanView>& scans, int half, float voxelMm, Mesh& out) {
    Tsdf t(voxelMm);
    for (const ScanView& s : scans) {
        DepthMap d = *s.grid; d.z.assign((size_t)d.width*d.height, 0.f);
        for (size_t i = (size_t)half; i < s.frames->size(); i += 2) { fillDepth((*s.frames)[i], d); t.integrate(d, s.frameToProject(i), 64); }
    }
    MeshOptions o; o.minWeight = 2; o.smoothIterations = 1; o.fillHolesMm = 0; o.keepTriangles = 1; o.removeFragments = false;
    extractMesh(t, o, out); computeNormals(out);
}
}
float polishPoses(std::vector<ScanView>& scans, std::vector<std::vector<Pose>>& poses, float voxelMm, int rounds) {
    poses.resize(scans.size());
    for (size_t s = 0; s < scans.size(); s++) {
        poses[s].resize(scans[s].frames->size());
        for (size_t i = 0; i < poses[s].size(); i++) poses[s][i] = mul(scans[s].placement, (*scans[s].frames)[i].pose);
        scans[s].poses = &poses[s];
    }
    float moved = 0;
    for (int round = 0; round < rounds; round++) {
        Mesh model[2]; modelOfHalf(scans, 1, voxelMm, model[0]); modelOfHalf(scans, 0, voxelMm, model[1]);
        double sum = 0; size_t count = 0; const float reach = 1.5f, inlier = 1.2f;
        for (int half = 0; half < 2; half++) {
            if (model[half].vertices.empty()) continue;
            const Mesh& m = model[half]; const PointGrid grid(m.vertices, 1.f);
            for (size_t si = 0; si < scans.size(); si++) {
                const ScanView& s = scans[si]; const long frames = (long)s.frames->size();
                #pragma omp parallel for schedule(dynamic, 4) reduction(+:sum, count)
                for (long i = half; i < frames; i += 2) {
                    const ScanFrame& f = (*s.frames)[i]; Pose T = poses[si][i]; const Pose start = T;
                    std::vector<V3> pts; pts.reserve(f.cells.size()/11+1);
                    for (size_t k = 0; k < f.cells.size(); k += 11) { const int gx = (int)(f.cells[k]%s.grid->width), gy = (int)(f.cells[k]/s.grid->width); const float z = f.z[k]; pts.push_back(V3{(gx-s.grid->cx)/s.grid->fx*z, (gy-s.grid->cy)/s.grid->fy*z, z}); }
                    bool solved = false; V3 center{}; for (const V3& p : pts) center = center+p; if (!pts.empty()) center = center*(1.f/pts.size());
                    for (int it = 0; it < 6; it++) {
                        double A[36] = {0}, b[6] = {0}; int used = 0;
                        for (const V3& c : pts) {
                            const V3 w = T.apply(c); const long j = grid.nearest(w, reach); if (j < 0) continue;
                            const V3 n = m.normals[j]; const float e = dot(n, m.vertices[j]-w); if (std::fabs(e) > inlier) continue;
                            const V3 x = cross(w, n); const double r[6] = {x.x, x.y, x.z, n.x, n.y, n.z};
                            for (int a = 0; a < 6; a++) { b[a] += r[a]*e; for (int d = 0; d < 6; d++) A[a*6+d] += r[a]*r[d]; }
                            used++;
                        }
                        if (used < 200) break;
                        for (int a = 0; a < 6; a++) A[a*6+a] += 1e-3*used;
                        double x[6]; if (!solve6(A, b, x)) break;
                        T = mul(expTwist(x), T); solved = true;
                    }
                    // A frame may only be nudged. Far from where it started, the fit has gone wrong.
                    const double d = norm(T.apply(center)-start.apply(center));
                    if (solved && d < 3.0) { poses[si][i] = T; sum += d; count++; }
                }
            }
        }
        moved = count ? (float)(sum/count) : 0;
    }
    return moved;
}
void buildPointCloud(ScanData& scan) {
    if (scan.cloud || scan.frames.empty()) return;
    // Fine enough to see detail, and the finer the nearer the scan was made.
    std::vector<float> zs; for (size_t i = 0; i < scan.frames.size(); i += std::max<size_t>(1, scan.frames.size()/32)) { const ScanFrame& f = scan.frames[i]; if (!f.z.empty()) zs.push_back(f.z[f.z.size()/2]); }
    float dist = 300; if (!zs.empty()) { std::nth_element(zs.begin(), zs.begin()+zs.size()/2, zs.end()); dist = zs[zs.size()/2]; }
    const float cube = std::clamp(dist/800.f, .15f, 1.f); scan.cloudSpacingMm = cube;
    struct Sample { int64_t key; V3 p, n; };
    constexpr int kMinSamples = 3; constexpr size_t kMaxFrames = 600;
    const size_t step = std::max<size_t>(1, scan.frames.size()/kMaxFrames); const long nFrames = (long)((scan.frames.size()+step-1)/step);
    const DepthMap& grid = scan.grid; const int W = grid.width, H = grid.height;
    std::vector<std::vector<Sample>> perFrame((size_t)nFrames);
    #pragma omp parallel
    {
        std::vector<float> z((size_t)W*H, 0.f);
        #pragma omp for schedule(dynamic, 4)
        for (long fi = 0; fi < nFrames; fi++) {
            const ScanFrame& f = scan.frames[(size_t)fi*step]; std::vector<Sample>& out = perFrame[(size_t)fi]; out.reserve(f.cells.size());
            for (size_t i = 0; i < f.cells.size(); i++) z[f.cells[i]] = f.z[i];
            auto at = [&](int x, int y) { const float d = z[(size_t)y*W+x]; return V3{(x-grid.cx)/grid.fx*d, (y-grid.cy)/grid.fy*d, d}; };
            for (size_t i = 0; i < f.cells.size(); i++) {
                const int x = (int)(f.cells[i]%W), y = (int)(f.cells[i]/W); if (x < 1 || y < 1 || x >= W-1 || y >= H-1) continue;
                if (z[(size_t)y*W+x-1] <= 0 || z[(size_t)y*W+x+1] <= 0 || z[(size_t)(y-1)*W+x] <= 0 || z[(size_t)(y+1)*W+x] <= 0) continue;
                const V3 c = at(x, y); V3 n = normalized(cross(at(x+1, y)-at(x-1, y), at(x, y+1)-at(x, y-1)));
                if (dot(n, c) > 0) n = n*-1.f;   // toward the camera
                if (-dot(n, normalized(c)) < .25f) continue;   // seen too much from the side to be trusted
                const V3 w = f.pose.apply(c);
                const int64_t kx = (int64_t)std::floor(w.x/cube)+(1<<20), ky = (int64_t)std::floor(w.y/cube)+(1<<20), kz = (int64_t)std::floor(w.z/cube)+(1<<20);
                out.push_back({(kx<<42)|(ky<<21)|kz, w, f.pose.rot(n)});
            }
            for (uint32_t cell : f.cells) z[cell] = 0.f;
        }
    }
    size_t total = 0; for (const auto& v : perFrame) total += v.size();
    std::vector<Sample> all; all.reserve(total); for (auto& v : perFrame) { all.insert(all.end(), v.begin(), v.end()); std::vector<Sample>().swap(v); }
    std::sort(all.begin(), all.end(), [](const Sample& a, const Sample& b) { return a.key < b.key; });
    auto m = std::make_shared<Mesh>();
    for (size_t a = 0; a < all.size();) {
        size_t b = a; V3 p{}, n{}; while (b < all.size() && all[b].key == all[a].key) { p = p+all[b].p; n = n+all[b].n; b++; }
        if (b-a >= kMinSamples) { m->vertices.push_back(p*(1.f/(b-a))); m->normals.push_back(normalized(n)); }
        a = b;
    }
    scan.cloud = m;
}
void buildOverview(ScanData& scan) {
    if (scan.overview || scan.frames.empty()) return;
    // Coarse enough for speed, fine enough for shape: scales with the working distance.
    std::vector<float> zs; for (size_t i = 0; i < scan.frames.size(); i += std::max<size_t>(1, scan.frames.size()/32)) { const ScanFrame& f = scan.frames[i]; if (!f.z.empty()) zs.push_back(f.z[f.z.size()/2]); }
    float dist = 300; if (!zs.empty()) { std::nth_element(zs.begin(), zs.begin()+zs.size()/2, zs.end()); dist = zs[zs.size()/2]; }
    Tsdf t(std::clamp(dist/500.f, 0.5f, 2.5f));
    DepthMap d = scan.grid; d.z.assign((size_t)d.width*d.height, 0.f);
    const size_t step = std::max<size_t>(1, scan.frames.size()/400);
    for (size_t i = 0; i < scan.frames.size(); i += step) { fillDepth(scan.frames[i], d); t.integrate(d, scan.frames[i].pose, 64); }
    auto m = std::make_shared<Mesh>(); MeshOptions o; o.minWeight = 1; o.smoothIterations = 2; o.fillHolesMm = 0; o.keepTriangles = 1;
    extractMesh(t, o, *m);
    // Stray pieces would count as contradictions when this scan is aligned.
    const ScanView self{&scan.frames, &scan.grid, &scan.colorCalib, Pose{}};
    if (carveFreeSpace(*m, {self}, std::max(2.f, t.voxel()*2))) removeSmallParts(*m, 0.02f);
    computeNormals(*m);
    scan.overview = m;
}
// A voxel averages this many frames, far more than while scanning: the dots then cross it at many
// places, and what they get wrong evens out instead of staying in the surface.
static constexpr float kFinalMaxWeight = 500.f;
bool fuseProject(const Project& p, float voxelMm, const MeshOptions& options, Mesh& out, std::string& error, const std::function<void(float)>& progress) {
    size_t total = 0, done = 0; const ScanData* first = nullptr;
    for (const ScanData& s : p.scans) if (s.included) {
        if (!s.placed) { error = s.name + " ist noch nicht ausgerichtet."; return false; }
        total += s.frames.size(); if (!first) first = &s;
    }
    if (!total) { error = "Kein Scan ausgewählt."; return false; }
    std::vector<ScanView> views; for (const ScanData& s : p.scans) if (s.included) views.push_back({&s.frames, &s.grid, &s.colorCalib, s.placement});
    std::vector<std::vector<Pose>> polished;
    if (options.cleanUp) { if (progress) progress(0.02f); polishPoses(views, polished, std::max(voxelMm, 0.5f)); }
    Tsdf t(voxelMm); ColorImage img;
    for (const ScanView& s : views) {
        DepthMap d = *s.grid; d.z.assign((size_t)d.width*d.height, 0.f);
        for (size_t i = 0; i < s.frames->size(); i++) {
            const ScanFrame& f = (*s.frames)[i];
            fillDepth(f, d);
            const bool col = s.colorCalib->valid && !f.colorJpeg.empty() && decodeJpeg(f.colorJpeg, img);
            t.integrate(d, s.frameToProject(i), kFinalMaxWeight, col ? &img : nullptr, s.colorCalib);
            if (progress && ++done % 16 == 0) progress(0.2f+0.7f*done/total);
        }
    }
    MeshOptions o = options; o.upright = false;
    o.removeFragments = false;   // only after the clean-up below
    extractMesh(t, o, out);
    if (out.indices.empty()) { error = "Die Fusion ergibt kein Netz."; return false; }
    const float tol = std::max(2.f, voxelMm*2);
    if (options.cleanUp) carveFreeSpace(out, views, tol);
    if (options.removeFragments) removeSmallParts(out, options.minFragmentShare);
    if (options.cleanUp) { computeNormals(out); colorFromFrames(out, views, tol); }
    if (out.indices.empty()) { error = "Die Fusion ergibt kein Netz."; return false; }
    if (options.upright) { orientUpright(out, first->placement.rot(first->up)); computeNormals(out); }
    return true;
}

bool writeThumbnail(const Mesh& m, V3 up, const std::string& path) {
    if (m.indices.empty()) return false;
    constexpr int W = 480, H = 300;
    const V3 u = normalized(up), side = normalized(cross(std::fabs(u.y) < .9f ? V3{0, 1, 0} : V3{1, 0, 0}, u)), fwd0 = cross(u, side);
    const float az = .6f, el = .35f; const V3 yaw = side*std::cos(az)+fwd0*std::sin(az), back = side*std::sin(az)-fwd0*std::cos(az);
    const V3 ex = yaw, ez = back*std::cos(el)+u*std::sin(el), ey = cross(ez, ex);   // ez points to the viewer
    // The outermost points are left out of the framing: a stray piece must not shrink the object.
    std::vector<float> xs, ys; const size_t stride = std::max<size_t>(1, m.vertices.size()/20000);
    for (size_t i = 0; i < m.vertices.size(); i += stride) { xs.push_back(dot(m.vertices[i], ex)); ys.push_back(dot(m.vertices[i], ey)); }
    std::sort(xs.begin(), xs.end()); std::sort(ys.begin(), ys.end());
    const float x0 = xs[xs.size()/100], x1 = xs[xs.size()*99/100], y0 = ys[ys.size()/100], y1 = ys[ys.size()*99/100];
    const float scale = .78f*std::min(W/std::max(.1f, x1-x0), H/std::max(.1f, y1-y0)), cx = (x0+x1)/2, cy = (y0+y1)/2;
    std::vector<float> z((size_t)W*H, -1e30f); std::vector<V3> n((size_t)W*H);
    for (size_t t = 0; t+2 < m.indices.size(); t += 3) {
        const V3 A = m.vertices[m.indices[t]], B = m.vertices[m.indices[t+1]], C = m.vertices[m.indices[t+2]];
        const float ax = W/2+(dot(A, ex)-cx)*scale, ay = H/2-(dot(A, ey)-cy)*scale, bx = W/2+(dot(B, ex)-cx)*scale, by = H/2-(dot(B, ey)-cy)*scale, qx = W/2+(dot(C, ex)-cx)*scale, qy = H/2-(dot(C, ey)-cy)*scale;
        const float d = (bx-ax)*(qy-ay)-(by-ay)*(qx-ax); if (std::fabs(d) < 1e-6f) continue;
        const int px0 = std::max(0, (int)std::floor(std::min({ax, bx, qx}))), px1 = std::min(W-1, (int)std::ceil(std::max({ax, bx, qx}))), py0 = std::max(0, (int)std::floor(std::min({ay, by, qy}))), py1 = std::min(H-1, (int)std::ceil(std::max({ay, by, qy})));
        const float az2 = dot(A, ez), bz = dot(B, ez), qz = dot(C, ez); const V3 nr = normalized(cross(B-A, C-A));
        for (int y = py0; y <= py1; y++) for (int x = px0; x <= px1; x++) {
            const float a = ((x-ax)*(qy-ay)-(y-ay)*(qx-ax))/d, b = ((bx-ax)*(y-ay)-(by-ay)*(x-ax))/d; if (a < -.02f || b < -.02f || a+b > 1.02f) continue;
            const float zz = az2*(1-a-b)+bz*a+qz*b; size_t p = (size_t)y*W+x; if (zz > z[p]) { z[p] = zz; n[p] = nr; }
        }
    }
    const V3 key = normalized(ez*1.f+ex*.5f+ey*.7f), fill = normalized(ez*1.f-ex*.8f);
    std::vector<uint8_t> rgb((size_t)W*H*3);
    for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) {
        const size_t p = (size_t)y*W+x; uint8_t* o = rgb.data()+3*p;
        if (z[p] < -1e29f) { const float g = 1-(float)y/H; o[0] = (uint8_t)(10+10*g); o[1] = (uint8_t)(13+12*g); o[2] = (uint8_t)(18+15*g); continue; }
        const float l = .18f+.72f*std::fabs(dot(n[p], key))+.14f*std::fabs(dot(n[p], fill));
        o[0] = (uint8_t)std::min(255.f, 150*l); o[1] = (uint8_t)std::min(255.f, 188*l); o[2] = (uint8_t)std::min(255.f, 240*l);
    }
    std::error_code ec; std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
    FILE* f = fopen(path.c_str(), "wb"); if (!f) return false;
    fprintf(f, "P6 %d %d 255\n", W, H); const bool ok = fwrite(rgb.data(), 1, rgb.size(), f) == rgb.size(); return fclose(f) == 0 && ok;
}
bool writeProjectThumbnail(Project& p, const std::string& thumbnail) {
    Mesh all; V3 up{0, 0, 1}; bool first = true;
    for (ScanData& s : p.scans) {
        if (!s.placed || !s.included || s.frames.empty()) continue;
        if (!s.overview) buildOverview(s);
        if (!s.overview) continue;
        if (first) { up = s.placement.rot(s.up); first = false; }
        const uint32_t base = (uint32_t)all.vertices.size();
        for (const V3& v : s.overview->vertices) all.vertices.push_back(s.placement.apply(v));
        for (uint32_t i : s.overview->indices) all.indices.push_back(base+i);
    }
    if (all.indices.empty()) for (ScanData& s : p.scans) {   // nothing placed yet: the first scan alone
        if (s.frames.empty()) continue;
        if (!s.overview) buildOverview(s);
        if (s.overview) { all = *s.overview; up = s.up; break; }
    }
    return writeThumbnail(all, up, thumbnail);
}
bool makeProjectThumbnail(const std::string& projectFile, const std::string& thumbnail) {
    Project p; std::string error; if (!loadProject(p, projectFile, error)) return false;
    return writeProjectThumbnail(p, thumbnail);
}

namespace {
constexpr char kMagic[8] = {'V', 'F', 'P', 'R', 'O', 'J', '0', '1'};
struct Out { FILE* f; bool ok = true;
    void bytes(const void* p, size_t n) { if (n) ok = fwrite(p, 1, n, f) == n && ok; }
    template <class T> void pod(const T& v) { bytes(&v, sizeof v); }
    template <class T> void vec(const std::vector<T>& v) { const uint64_t n = v.size(); pod(n); bytes(v.data(), n*sizeof(T)); }
    void str(const std::string& s) { const uint64_t n = s.size(); pod(n); bytes(s.data(), n); } };
struct In { FILE* f; bool ok = true; uint64_t left;
    void bytes(void* p, size_t n) { if (n > left) { ok = false; return; } if (n) ok = fread(p, 1, n, f) == n && ok; left -= n; }
    template <class T> void pod(T& v) { bytes(&v, sizeof v); }
    template <class T> void vec(std::vector<T>& v) { uint64_t n = 0; pod(n); if (!ok || n > left/sizeof(T)) { ok = false; return; } v.resize(n); bytes(v.data(), n*sizeof(T)); }
    void str(std::string& s) { uint64_t n = 0; pod(n); if (!ok || n > left) { ok = false; return; } s.resize(n); bytes(s.data(), n); } };
}
bool saveProject(const Project& p, const std::string& path, std::string& error) {
    const std::string tmp = path + ".tmp"; FILE* f = fopen(tmp.c_str(), "wb");
    if (!f) { error = "Datei kann nicht geschrieben werden: " + path; return false; }
    Out o{f}; o.bytes(kMagic, 8); o.pod((uint64_t)p.scans.size());
    for (const ScanData& s : p.scans) {
        o.str(s.name); o.pod(s.grid.width); o.pod(s.grid.height); o.pod(s.grid.fx); o.pod(s.grid.fy); o.pod(s.grid.cx); o.pod(s.grid.cy);
        o.pod(s.colorCalib); o.vec(s.markers); o.pod(s.up); o.pod(s.placement); o.pod((uint8_t)s.placed); o.pod((uint8_t)s.included);
        o.pod((uint64_t)s.frames.size());
        for (const ScanFrame& fr : s.frames) { o.vec(fr.cells); o.vec(fr.z); o.pod(fr.pose); o.vec(fr.colorJpeg); }
    }
    const bool ok = fclose(f) == 0 && o.ok;
    if (!ok) { std::remove(tmp.c_str()); error = "Schreibfehler (Festplatte voll?)"; return false; }
    std::rename(tmp.c_str(), path.c_str());
    return true;
}
bool loadProject(Project& p, const std::string& path, std::string& error) {
    FILE* f = fopen(path.c_str(), "rb"); if (!f) { error = "Datei nicht lesbar: " + path; return false; }
    fseek(f, 0, SEEK_END); const uint64_t len = (uint64_t)ftell(f); fseek(f, 0, SEEK_SET);
    In in{f, true, len}; char m[8] = {0}; in.bytes(m, 8);
    if (!in.ok || memcmp(m, kMagic, 8)) { fclose(f); error = "Das ist keine VoxelForge-Projektdatei."; return false; }
    Project q; uint64_t n = 0; in.pod(n);
    for (uint64_t i = 0; i < n && in.ok; i++) {
        ScanData s; uint8_t placed = 0, included = 0; uint64_t frames = 0;
        in.str(s.name); in.pod(s.grid.width); in.pod(s.grid.height); in.pod(s.grid.fx); in.pod(s.grid.fy); in.pod(s.grid.cx); in.pod(s.grid.cy);
        in.pod(s.colorCalib); in.vec(s.markers); in.pod(s.up); in.pod(s.placement); in.pod(placed); in.pod(included); in.pod(frames);
        s.placed = placed; s.included = included;
        if (!in.ok || s.grid.width <= 0 || s.grid.height <= 0 || frames > in.left) { in.ok = false; break; }
        const size_t gridCells = (size_t)s.grid.width*s.grid.height;
        s.frames.resize(frames);
        for (ScanFrame& fr : s.frames) {
            in.vec(fr.cells); in.vec(fr.z); in.pod(fr.pose); in.vec(fr.colorJpeg);
            if (!in.ok || fr.cells.size() != fr.z.size()) { in.ok = false; break; }
            for (uint32_t c : fr.cells) if (c >= gridCells) { in.ok = false; break; }
        }
        q.scans.push_back(std::move(s));
    }
    fclose(f);
    if (!in.ok) { error = "Die Projektdatei ist beschädigt oder abgeschnitten."; return false; }
    p = std::move(q);
    return true;
}
