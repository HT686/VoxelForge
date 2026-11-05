// Self-contained checks with a synthetic scene: a sphere on a plane, with a random dot
// pattern fixed to the surfaces, rendered exactly like the Otter's rectified IR pair.
#include <cstdio>
#include <turbojpeg.h>
#include <unistd.h>
#include <cstring>
#include <map>
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>
#include "measure.hpp"
#include "mesh.hpp"
#include "meshedit.hpp"
#include <fstream>
#include <filesystem>
#include "pipeline.hpp"
#include "i18n.hpp"
#include "settings.hpp"
#include "register.hpp"

// Closed box made of small triangles, optionally sheared and noisy, like a scanned box.
static Mesh makeBox(float a, float b, float c, float shearXperZ, float noise) {
    Mesh m; std::mt19937 g(3); std::normal_distribution<float> nd(0.f, noise > 0 ? noise : 1.f);
    const int N = 80;
    auto face = [&](V3 o, V3 u, V3 v) {
        uint32_t base = (uint32_t)m.vertices.size();
        for (int j = 0; j <= N; j++) for (int i = 0; i <= N; i++) {
            V3 p = o + u*((float)i/N) + v*((float)j/N);
            if (noise > 0) p = p + normalized(cross(u, v))*nd(g);
            p.x += shearXperZ*p.z;
            m.vertices.push_back(p);
        }
        for (int j = 0; j < N; j++) for (int i = 0; i < N; i++) {
            uint32_t q = base + j*(N+1) + i;
            m.indices.insert(m.indices.end(), {q, q+1, q+N+2, q, q+N+2, q+N+1});
        }
    };
    face({0,0,0},{0,b,0},{a,0,0}); face({0,0,c},{a,0,0},{0,b,0});
    face({0,0,0},{a,0,0},{0,0,c}); face({0,b,0},{0,0,c},{a,0,0});
    face({0,0,0},{0,0,c},{0,b,0}); face({a,0,0},{0,b,0},{0,0,c});
    return m;
}
static std::vector<double> sortedLengths(const BoxReport& r) { std::vector<double> v; for (auto& q : r.pairs) if (q.parallel) v.push_back(q.distanceMm); std::sort(v.begin(), v.end()); return v; }
static double maxRightAngleError(const BoxReport& r) { double e = 0; for (auto& q : r.pairs) if (!q.parallel) e = std::max(e, std::fabs(q.angleDeg-90)); return e; }

// Closed sphere made of shared vertices, by subdividing an octahedron.
static Mesh makeSphere(float r, int depth) {
    Mesh m; m.vertices = {{r,0,0},{-r,0,0},{0,r,0},{0,-r,0},{0,0,r},{0,0,-r}};
    m.indices = {0,2,4, 2,1,4, 1,3,4, 3,0,4, 2,0,5, 1,2,5, 3,1,5, 0,3,5};
    for (int d = 0; d < depth; d++) {
        std::map<uint64_t, uint32_t> mid; std::vector<uint32_t> ni;
        auto get = [&](uint32_t a, uint32_t b) { uint64_t k = ((uint64_t)std::min(a, b) << 32) | std::max(a, b); auto it = mid.find(k); if (it != mid.end()) return it->second; V3 p = normalized(m.vertices[a]+m.vertices[b])*r; m.vertices.push_back(p); uint32_t id = (uint32_t)m.vertices.size()-1; mid[k] = id; return id; };
        for (size_t i = 0; i < m.indices.size(); i += 3) { uint32_t a = m.indices[i], b = m.indices[i+1], c = m.indices[i+2], ab = get(a,b), bc = get(b,c), ca = get(c,a); ni.insert(ni.end(), {a,ab,ca, ab,b,bc, ca,bc,c, ab,bc,ca}); }
        m.indices.swap(ni);
    }
    return m;
}

// A plain colored image as JPEG, as the scanner delivers them.
static std::vector<uint8_t> solidJpeg(int w, int h, uint8_t r, uint8_t g, uint8_t b) {
    std::vector<uint8_t> px((size_t)w*h*3); for (size_t i = 0; i < px.size(); i += 3) { px[i] = r; px[i+1] = g; px[i+2] = b; }
    tjhandle t = tjInitCompress(); unsigned char* out = nullptr; unsigned long len = 0;
    tjCompress2(t, px.data(), w, 0, h, TJPF_RGB, &out, &len, TJSAMP_444, 95, 0);
    std::vector<uint8_t> v(out, out+len); tjFree(out); tjDestroy(t); return v;
}
static int failures = 0;
#define CHECK(cond, ...) do { bool ok_ = (cond); printf("%s  ", ok_ ? "PASS" : "FAIL"); printf(__VA_ARGS__); printf("\n"); if (!ok_) failures++; } while (0)

struct Scene {
    V3 c{0, 0, 300}; float r = 40; float planeZ = 340;
    std::vector<uint8_t> tex; int ts = 2048;
    Scene() {
        std::mt19937 g(7); tex.assign(ts*ts, 15);
        for (int n = 0; n < ts*ts/18; n++) {
            int x = g()%ts, y = g()%ts;
            for (int dy = -2; dy <= 2; dy++) for (int dx = -2; dx <= 2; dx++) {
                float v = 240*std::exp(-(dx*dx+dy*dy)/2.5f);
                int X = (x+dx+ts)%ts, Y = (y+dy+ts)%ts; tex[Y*ts+X] = (uint8_t)std::min(255.f, tex[Y*ts+X]+v);
            }
        }
    }
    uint8_t sample(float u, float v) const {
        u -= std::floor(u); v -= std::floor(v);
        float x = u*(ts-1), y = v*(ts-1); int xi = (int)x, yi = (int)y; float a = x-xi, b = y-yi; int x1 = (xi+1)%ts, y1 = (yi+1)%ts;
        return (uint8_t)((1-a)*(1-b)*tex[yi*ts+xi] + a*(1-b)*tex[yi*ts+x1] + (1-a)*b*tex[y1*ts+xi] + a*b*tex[y1*ts+x1]);
    }
    // Ray from o along d (world). Returns texture value and depth along d.
    bool hit(V3 o, V3 d, float& t, uint8_t& val) const {
        V3 oc = o-c; float A = dot(d,d), B = 2*dot(oc,d), C = dot(oc,oc)-r*r, D = B*B-4*A*C;
        t = 1e9f; bool sphere = false;
        if (D > 0) { float s = (-B-std::sqrt(D))/(2*A); if (s > 0) { t = s; sphere = true; } }
        float s = (planeZ-o.z)/d.z;
        if (s > 0 && s < t) { t = s; sphere = false; }
        if (t > 1e8f) return false;
        V3 p = o+d*t;
        (void)sphere;
        val = sample(p.x/170.f + p.z/410.f, p.y/170.f + p.z/530.f);   // no poles, little stretching
        return true;
    }
};
// Renders the pair for a camera at camToWorld. Reference camera is the pose origin; the other camera sits B to the side.
static void render(const Scene& s, const StereoCalib& c, const Pose& T, StereoFrame& f) {
    const int W = 1280, H = 800; f.width = W; f.height = H; f.ref.assign(W*H, 0); f.other.assign(W*H, 0);
    // other pixel x_o = x_ref - shift with true disparity D = dispOffset - shift, so cam other is at +B in x for cxOther = cxRef - dispOffset
    const float cxR = c.cxRef(), cxO = c.cxRef() - c.dispOffset, fp = c.focalPx;
    V3 oOther = T.apply(V3{c.baselineMm, 0, 0});   // images are mirrored, as on the device
    #pragma omp parallel for
    for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) {
        V3 dr = T.rot(V3{-(x-cxR)/fp, (y-c.cy())/fp, 1.f}), dot_ = T.rot(V3{-(x-cxO)/fp, (y-c.cy())/fp, 1.f});
        float t; uint8_t v;
        if (s.hit(T.t, dr, t, v)) f.ref[y*W+x] = v;
        if (s.hit(oOther, dot_, t, v)) f.other[y*W+x] = v;
    }
}
static Pose orbit(float angleDeg) {
    // Camera circles the sphere center at 300 mm distance around the y axis, looking at it.
    double x[6] = {0, angleDeg*M_PI/180.0, 0, 0, 0, 0}; Pose R = expTwist(x);
    Pose T = R; T.t = V3{0, 0, 300} - R.rot(V3{0, 0, 300});
    return T;
}

// Turntable scene as the real scanner sees it: the camera and a wall stand still, an
// asymmetric object turns, and the dots are projected from the scanner, so they do not
// stick to the surface. Shape is the only cue for motion.
struct TurntableScene {
    V3 axisPoint{0, 0, 300}; float angle = 0;
    std::vector<uint8_t> dots; int ds = 2048;
    V3 projector{25.f, 0, 0}; float wallZ = 430;
    TurntableScene() {
        std::mt19937 g(11); dots.assign(ds*ds, 12);
        for (int n = 0; n < ds*ds/20; n++) {
            int x = g()%ds, y = g()%ds;
            for (int dy = -3; dy <= 3; dy++) for (int dx = -3; dx <= 3; dx++) {
                int X = x+dx, Y = y+dy; if (X < 0 || Y < 0 || X >= ds || Y >= ds) continue;
                dots[Y*ds+X] = (uint8_t)std::min(255.f, dots[Y*ds+X] + 230*std::exp(-(dx*dx+dy*dy)/3.f));
            }
        }
    }
    Pose objectMotion() const {   // object coordinates -> camera coordinates
        double x[6] = {0, angle*M_PI/180.0, 0, 0, 0, 0}; Pose R = expTwist(x);
        R.t = axisPoint - R.rot(axisPoint); return R;
    }
    static bool sphere(V3 o, V3 d, V3 c, float r, float& t) {
        V3 oc = o-c; float A = dot(d,d), B = 2*dot(oc,d), C = dot(oc,oc)-r*r, D = B*B-4*A*C;
        if (D <= 0) return false;
        float s = (-B-std::sqrt(D))/(2*A); if (s <= 0 || s >= t) return false; t = s; return true;
    }
    uint8_t shade(V3 p) const {
        V3 q = p-projector; float u = q.x/q.z/0.9f+0.5f, v = q.y/q.z/0.9f+0.5f;
        if (u < 0 || v < 0 || u >= 1 || v >= 1) return 0;
        float x = u*(ds-1), y = v*(ds-1); int xi = (int)x, yi = (int)y; float a = x-xi, b = y-yi;
        int x1 = std::min(ds-1, xi+1), y1 = std::min(ds-1, yi+1);
        return (uint8_t)((1-a)*(1-b)*dots[yi*ds+xi] + a*(1-b)*dots[yi*ds+x1] + (1-a)*b*dots[y1*ds+xi] + a*b*dots[y1*ds+x1]);
    }
    bool hit(V3 o, V3 d, uint8_t& val) const {
        Pose M = objectMotion();
        V3 oo = M.applyInv(o), dd = M.rotInv(d);
        float t = (wallZ-o.z)/d.z;      // wall is fixed in camera coordinates
        bool any = t > 0;
        // Object: body, head and a side lobe, all off-axis so that turning changes its outline.
        sphere(oo, dd, V3{-12, 15, 300}, 34, t); sphere(oo, dd, V3{5, -28, 292}, 24, t); sphere(oo, dd, V3{30, 12, 282}, 13, t);
        if (!any && t > 1e8f) return false;
        val = shade(o + d*t);
        return true;
    }
};
static void renderTurntable(const TurntableScene& s, const StereoCalib& c, StereoFrame& f) {
    const int W = 1280, H = 800; f.width = W; f.height = H; f.ref.assign(W*H, 0); f.other.assign(W*H, 0);
    const float cxR = c.cxRef(), cxO = c.cxRef()-c.dispOffset, fp = c.focalPx; const V3 oOther{c.baselineMm, 0, 0};
    #pragma omp parallel for
    for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) {
        uint8_t v;
        if (s.hit(V3{}, V3{-(x-cxR)/fp, (y-c.cy())/fp, 1.f}, v)) f.ref[y*W+x] = v;
        if (s.hit(oOther, V3{-(x-cxO)/fp, (y-c.cy())/fp, 1.f}, v)) f.other[y*W+x] = v;
    }
}
// Returns the largest tracked rotation angle relative to the true one over a sweep.
struct SweepResult { float trueDeg, trackedDeg, maxAngleErr; int lost; };
static SweepResult sweep(const StereoCalib& c, bool turntable) {
    TurntableScene sc; PipelineOptions po; po.voxelMm = 0.75f; po.zMax = 620.f; Pipeline p(po); p.setTurntable(turntable); p.setScanning(true);
    SweepResult r{0, 0, 0, 0};
    for (int i = 0; i <= 90; i++) {
        if (i == 1 && getenv("VOXELFORGE_TRACE") && turntable) { auto g = p.region(); printf("     Bereich: Mitte (%.0f %.0f %.0f), Radius %.0f\n", g.center.x, g.center.y, g.center.z, g.radius); }
        sc.angle = i*0.5f; StereoFrame f; renderTurntable(sc, c, f); p.process(f, c);
        r.lost += p.stats().state == TrackState::Lost;
        const Pose& T = p.pose(); float tr = T.R[0]+T.R[4]+T.R[8];
        r.trackedDeg = std::acos(std::clamp((tr-1)/2, -1.f, 1.f))*57.29578f; r.trueDeg = sc.angle;
        r.maxAngleErr = std::max(r.maxAngleErr, std::fabs(r.trackedDeg - r.trueDeg));
    }
    return r;
}

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    StereoCalib c; c.focalPx = 1581.7565f; c.baselineMm = 50.0866f; c.dispOffset = 640.f;
    Scene scene;
    // 1. Formula check against the two real wall measurements.
    CHECK(std::fabs(c.depthFromShift(374.7f) - 298.6f) < 0.2f, "Tiefe bei Versatz 374.7 = %.2f mm", c.depthFromShift(374.7f));
    CHECK(std::fabs(c.depthFromShift(248.12f) - 202.2f) < 0.2f, "Tiefe bei Versatz 248.12 = %.2f mm", c.depthFromShift(248.12f));
    // 2. Stereo accuracy.
    StereoFrame f; render(scene, c, Pose{}, f);
    StereoMatcher sm; DepthMap d;
    sm.compute(f.ref.data(), f.other.data(), f.width, f.height, c, d);
    double e = 0, e2 = 0; int n = 0, bad = 0;
    for (int j = 0; j < d.height; j++) for (int i = 0; i < d.width; i++) {
        float z = d.z[j*d.width+i]; if (z <= 0) continue;
        V3 dir{(i-d.cx)/d.fx, (j-d.cy)/d.fy, 1.f}; float t; uint8_t v;
        if (!scene.hit(V3{}, dir, t, v)) continue;
        float err = z - t; if (std::fabs(err) > 3) { bad++; continue; }
        e += err; e2 += err*err; n++;
    }
    float rms = n ? std::sqrt(e2/n) : 99;
    printf("     Stereo: %.1f%% gueltig, RMS %.3f mm, Mittelfehler %.3f mm, Ausreisser %.2f%%\n", 100*d.validRatio(), rms, n ? e/n : 0, 100.0*bad/std::max(1, n+bad));
    CHECK(d.validRatio() > 0.25f, "Stereo liefert genug Punkte");
    CHECK(rms < 0.25f, "Stereo-Genauigkeit unter 0,25 mm");
    CHECK(100.0*bad/std::max(1, n+bad) < 3, "Stereo-Ausreisser unter 3 %%");
    // 2a. The sharp second measurement stays true: small windows must not add a bias or stray depth.
    {
        DepthMap fine = d; sm.refineDetail(f.ref.data(), f.other.data(), f.width, f.height, c, fine);
        double fe = 0, fe2 = 0; int fn = 0, fbad = 0; size_t fv = 0;
        for (int j = 0; j < fine.height; j++) for (int i = 0; i < fine.width; i++) {
            const float z = fine.z[j*fine.width+i]; if (z <= 0) continue; fv++;
            V3 dir{(i-fine.cx)/fine.fx, (j-fine.cy)/fine.fy, 1.f}; float t; uint8_t v; if (!scene.hit(V3{}, dir, t, v)) continue;
            const float err = z-t; if (std::fabs(err) > 3) { fbad++; continue; } fe += err; fe2 += err*err; fn++;
        }
        printf("     Detail: %.1f%% der Punkte, RMS %.3f mm, Mittelfehler %.3f mm, Ausreisser %.2f%%\n", 100.0*fv/std::max<size_t>(1, n+bad), fn ? std::sqrt(fe2/fn) : 99, fn ? fe/fn : 0, 100.0*fbad/std::max(1, fn+fbad));
        CHECK(fv > (size_t)(n+bad)/3, "Detail-Tiefe: genug Punkte bleiben");
        CHECK(fn && std::fabs(fe/fn) < 0.05, "Detail-Tiefe: kein systematischer Versatz (%.3f mm)", fn ? fe/fn : 0);
        CHECK(fn && std::sqrt(fe2/fn) < 0.4, "Detail-Tiefe: Rauschen begrenzt (RMS %.3f mm)", fn ? std::sqrt(fe2/fn) : 0);
        CHECK(100.0*fbad/std::max(1, fn+fbad) < 3, "Detail-Tiefe: Ausreisser unter 3 %%");
    }
    // 2a2. Exposure follows the object: too bright lowers it at once, too dim raises it.
    {
        Pipeline ep; ep.process(f, c);
        StereoFrame bright = f, dim = f; for (auto& v : bright.ref) v = (uint8_t)std::min(255, v*4); for (auto& v : dim.ref) v = (uint8_t)(v/4);
        const int down = ep.nextExposure(bright, 3000), up = ep.nextExposure(dim, 3000);
        CHECK(down < 2600, "Belichtung: ueberstrahltes Objekt senkt sie (%d)", down);
        CHECK(up > 3000, "Belichtung: dunkles Objekt hebt sie (%d)", up);
    }
    // 2a3. From close up the object's dense dots lie in the same row as a marker's dots, and must not win.
    {
        StereoFrame mf; mf.width = 1280; mf.height = 800; mf.ref.assign(1280*800, 0); mf.other.assign(1280*800, 0);
        const int u = 300, v = 400, shift = 240;
        for (int y = -14; y <= 14; y++) for (int x = -14; x <= 14; x++) if (x*x+y*y <= 196) mf.other[(size_t)(v+y)*1280+u+x] = 255;
        auto dot = [&](int cx, int cy) { for (int y = -2; y <= 2; y++) for (int x = -2; x <= 2; x++) mf.ref[(size_t)(cy+y)*1280+cx+x] = 255; };
        for (int k = 0; k < 6; k++) dot(u+shift+(int)(8*std::cos(k*1.05f)), v+(int)(8*std::sin(k*1.05f)));
        for (int y = -60; y <= 60; y += 8) for (int x = -60; x <= 60; x += 8) dot(u+shift+150+x, v+y);   // the object
        std::vector<Marker3D> found; detectMarkers(mf, c, 64, 512, found);
        const float want = c.depthFromShift((float)shift);
        CHECK(found.size() == 1 && std::fabs(found[0].pos.z-want) < 1.f, "Marker: nicht mit den Punkten des Objekts verwechselt (%zu gefunden, z %.1f, soll %.1f)", found.size(), found.empty() ? 0.f : found[0].pos.z, want);
    }
    // 2a4. The graphics card must measure what the CPU measures, and step aside when it does not.
    {
        StereoMatcher card; card.compute_mode = ComputeMode::Gpu; DepthMap dg; card.params.slantRefine = true; StereoMatcher ref2; ref2.params.slantRefine = true; DepthMap dr;
        card.compute(f.ref.data(), f.other.data(), f.width, f.height, c, dg);
        if (!card.usingGpu()) printf("     Grafikkarte nicht geprueft: %s\n", card.gpuInfo().c_str());
        else {
            ref2.compute(f.ref.data(), f.other.data(), f.width, f.height, c, dr);
            size_t both = 0, diff = 0, one = 0; float worst = 0; for (size_t i = 0; i < dg.z.size(); i++) { const bool a = dg.z[i] > 0, b = dr.z[i] > 0; if (a && b) { both++; const float e = std::fabs(dg.z[i]-dr.z[i]); worst = std::max(worst, e); diff += e > .05f; } else one += a != b; }
            CHECK(both > 1000 && diff*1000 < both && one*200 < both, "Grafikkarte: gleiche Tiefe wie die CPU (%zu Punkte, %zu abweichend, %zu nur einseitig)", both, diff, one);
            DepthMap fg = dg, fr = dr; card.refineDetail(f.ref.data(), f.other.data(), f.width, f.height, c, fg); ref2.refineDetail(f.ref.data(), f.other.data(), f.width, f.height, c, fr);
            both = diff = one = 0; for (size_t i = 0; i < fg.z.size(); i++) { const bool a = fg.z[i] > 0, b = fr.z[i] > 0; if (a && b) { both++; diff += std::fabs(fg.z[i]-fr.z[i]) > .05f; } else one += a != b; }
            CHECK(both > 1000 && diff*1000 < both && one*200 < both, "Grafikkarte: gleiche Detail-Tiefe (%zu Punkte, %zu abweichend, %zu nur einseitig)", both, diff, one);
            // A card that measures wrongly: the check notices on the first frame and the CPU result is used.
            StereoMatcher wrong; wrong.compute_mode = ComputeMode::Auto; wrong.gpuTestOffset = .5f; wrong.params.slantRefine = true; DepthMap dw; wrong.compute(f.ref.data(), f.other.data(), f.width, f.height, c, dw);
            size_t off = 0, n2 = 0; for (size_t i = 0; i < dw.z.size(); i++) if (dw.z[i] > 0 && dr.z[i] > 0) { n2++; off += std::fabs(dw.z[i]-dr.z[i]) > .05f; }
            CHECK(!wrong.usingGpu() && n2 > 1000 && off*1000 < n2, "Grafikkarte: Abweichung erkannt, CPU uebernimmt (%s)", wrong.gpuInfo().c_str());
            StereoMatcher good; good.compute_mode = ComputeMode::Auto; DepthMap dk; good.compute(f.ref.data(), f.other.data(), f.width, f.height, c, dk); CHECK(good.usingGpu(), "Grafikkarte: bleibt aktiv, wenn sie stimmt");
        }
        setenv("VOXELFORGE_NO_GPU", "1", 1); StereoMatcher off; off.compute_mode = ComputeMode::Auto; DepthMap dn; off.compute(f.ref.data(), f.other.data(), f.width, f.height, c, dn); unsetenv("VOXELFORGE_NO_GPU");
        CHECK(!off.usingGpu() && dn.validRatio() > .25f, "Ohne Grafikkarte rechnet die CPU");
    }
    // 2b. The far pair's images are one row apart; the matcher must compensate for that.
    {
        StereoCalib c2 = c; c2.rowOffset = 1; StereoFrame sh = f;
        for (int y = 799; y > 0; y--) memcpy(sh.other.data()+(size_t)y*1280, f.other.data()+(size_t)(y-1)*1280, 1280);
        StereoMatcher m2, m3; DepthMap d2, d3;
        m2.compute(sh.ref.data(), sh.other.data(), 1280, 800, c2, d2); m3.compute(sh.ref.data(), sh.other.data(), 1280, 800, c, d3);
        CHECK(d2.validRatio() > d.validRatio()*0.95f, "Zeilenversatz ausgeglichen (%.1f %% gueltig)", 100*d2.validRatio());
        printf("     ohne Ausgleich: %.1f %% gueltig\n", 100*d3.validRatio());
    }
    // 3. Tracking and fusion over an orbit.
    PipelineOptions po; po.voxelMm = 0.75f; po.zMax = 620.f; Pipeline p(po); p.setScanning(true);   // the plane reaches far at steep angles
    // A sphere on a plane looks the same when turned about the axis through the sphere and normal to
    // the plane, so tracking may drift about it freely, and at this distance that moves the camera
    // by millimeters. Only distance from that axis and height along it can be judged.
    float maxErr = 0, maxObservable = 0; int lost = 0;
    for (int f2 = 0; f2 <= 120; f2++) {
        float a = f2*0.5f;   // about 2.6 mm per frame, roughly 8 cm/s at 30 fps
        Pose T0 = orbit(a); StereoFrame g; render(scene, c, T0, g);
        p.process(g, c);
        if (p.stats().state == TrackState::Lost) lost++;
        maxErr = std::max(maxErr, norm(p.pose().t - T0.t));
        { const V3 e = p.pose().t, w = T0.t; const float dr = std::hypot(e.x-scene.c.x, e.y-scene.c.y)-std::hypot(w.x-scene.c.x, w.y-scene.c.y), dh = e.z-w.z;
          maxObservable = std::max(maxObservable, std::hypot(dr, dh)); }
        if (getenv("VOXELFORGE_TRACE") && f2 % 8 == 0) printf("  f=%3d err=%7.3f rms=%6.3f inl=%.2f pts=%d valid=%.2f state=%d\n", f2, norm(p.pose().t - T0.t), p.stats().rmsMm, p.stats().inlierRatio, p.stats().icpPoints, p.stats().validDepth, (int)p.stats().state);
    }
    printf("     Tracking: letzter Fehler %.3f mm, maximal %.3f mm, verloren %d, %.1f ms/Bild, RMS %.3f\n", norm(p.pose().t - orbit(60).t), maxErr, lost, p.stats().totalMs, p.stats().rmsMm);
    CHECK(lost == 0, "Tracking verliert kein Bild");
    // Sphere + plane is rotationally symmetric about the axis through the sphere centre,
    // so sliding along that axis is unobservable. Surface accuracy is checked separately below.
    CHECK(maxObservable < 2.5f, "Tracking: beobachtbarer Positionsfehler ueber 60 Grad unter 2,5 mm (%.2f)", maxObservable);
    CHECK(maxErr < 10.0f, "Tracking: laeuft auch in der freien Richtung nicht weg (%.2f mm gesamt)", maxErr);
    // 4. Mesh geometry: points on the sphere must lie on r=40.
    Mesh m; MeshOptions mo; mo.removeFragments = false; p.buildMesh(mo, m);
    double se = 0; int sn = 0;
    for (const V3& v : m.vertices) { float rr = norm(v - scene.c); if (std::fabs(rr - scene.r) < 3 && v.z < 335) { se += (rr-scene.r)*(rr-scene.r); sn++; } }
    float srms = sn ? std::sqrt(se/sn) : 99;
    printf("     Netz: %zu Dreiecke, Kugelabweichung RMS %.3f mm auf %d Punkten\n", m.triangleCount(), srms, sn);
    CHECK(sn > 5000, "Netz enthaelt die Kugel");
    CHECK(srms < 0.3f, "Kugelabweichung unter 0,3 mm");
    {   // Triangles must face outward: slicers and viewers rely on the winding.
        size_t outward = 0, total = 0;
        for (size_t i = 0; i < m.indices.size(); i += 3) {
            V3 a = m.vertices[m.indices[i]], b = m.vertices[m.indices[i+1]], c2 = m.vertices[m.indices[i+2]], ctr = (a+b+c2)*(1.f/3);
            if (std::fabs(norm(ctr-scene.c)-scene.r) > 3 || ctr.z > 335) continue;
            total++; outward += dot(cross(b-a, c2-a), ctr-scene.c) > 0;
        }
        CHECK(total > 5000 && outward > total*99/100, "Dreiecke der Kugel zeigen nach aussen (%.1f %%)", 100.0*outward/std::max<size_t>(1, total));
    }
    // 5. Turntable: reproduces the reported bug without the mode and must be fixed with it.
    SweepResult off = sweep(c, false), on = sweep(c, true);
    printf("     Drehteller: wahr %.1f Grad | ohne Modus %.1f Grad | mit Modus %.1f Grad, max. Winkelfehler %.2f Grad, verloren %d\n", on.trueDeg, off.trackedDeg, on.trackedDeg, on.maxAngleErr, on.lost);
    CHECK(off.trackedDeg < 0.5f*off.trueDeg, "Ohne Drehteller-Modus tritt der gemeldete Fehler auf (Test ist aussagekraeftig)");
    CHECK(on.lost == 0, "Drehteller: kein Bild verloren");
    CHECK(on.maxAngleErr < 1.5f, "Drehteller: Winkelfehler unter 1,5 Grad");
    // 6. Starting on an empty frame must not poison the model (seen on the real device).
    {
        Pipeline q; q.setTurntable(true); q.setScanning(true);
        TurntableScene ts; StereoFrame good, dark; renderTurntable(ts, c, good);
        dark = good; std::fill(dark.ref.begin(), dark.ref.end(), 0); std::fill(dark.other.begin(), dark.other.end(), 0);
        for (int i = 0; i < 3; i++) q.process(dark, c);
        bool waited = q.stats().state == TrackState::Waiting && q.stats().integratedFrames == 0;
        for (int i = 0; i < 3; i++) q.process(good, c);
        CHECK(waited, "Leere Startbilder werden nicht ins Modell uebernommen");
        CHECK(q.stats().integratedFrames == 3 && q.stats().lostFrames == 0, "Danach startet der Scan normal (%d integriert, %d verloren)", q.stats().integratedFrames, q.stats().lostFrames);
    }
    // 6b. The displayed distance belongs to the object, not to the wall behind it, and the
    // object region is available before scanning starts.
    {
        PipelineOptions qo; qo.zMax = 620.f; Pipeline q(qo); q.setTurntable(true);
        TurntableScene ts; StereoFrame fr; renderTurntable(ts, c, fr); fr.index = 0;
        q.process(fr, c);
        float med = q.stats().medianDepth;
        CHECK(q.region().valid, "Objektbereich steht schon vor dem Scanstart fest");
        CHECK(med > 250 && med < 320, "Angezeigter Abstand gehoert zum Objekt (%.1f mm, Wand 430 mm)", med);
    }
    // 6c. Marker alignment and map, with exactly known motion.
    {
        double tw[6] = {0.2, -0.5, 0.3, 12, -7, 30}; Pose truth = expTwist(tw);
        std::vector<V3> a = {{0,0,300},{50,10,310},{-40,30,280},{20,-60,290},{-10,-20,330}}, b;
        for (const V3& p : a) b.push_back(truth.apply(p));
        Pose got; bool ok = alignRigid(a, b, got);
        float err = 0; for (size_t i = 0; i < a.size(); i++) err = std::max(err, norm(got.apply(a[i])-b[i]));
        CHECK(ok && err < 1e-2f, "Marker: starre Ausrichtung exakt (Fehler %.5f mm)", err);

        // 16 markers on a ring, seen as a turntable carries them through the camera's view.
        std::vector<V3> ring; for (int i = 0; i < 16; i++) { float t = i*6.2831853f/16; ring.push_back(V3{90*std::cos(t), 0, 90*std::sin(t)}); }
        MarkerTracker mt; Pose T; float maxErr = 0; int fails = 0;
        std::mt19937 g(5); std::normal_distribution<float> noise(0.f, 0.15f);
        for (int f = 0; f <= 800; f++) {
            double ang = f*0.5*M_PI/180; double x[6] = {0, ang, 0, 0, 0, 0}; Pose M = expTwist(x); M.t = V3{0, 20, 250};
            std::vector<Marker3D> det; std::vector<V3> worldTruth;
            for (const V3& p : ring) { V3 c2 = M.apply(p); if (c2.x > 75) continue; Marker3D d; d.pos = c2 + V3{noise(g), noise(g), noise(g)}; det.push_back(d); }
            MarkerResult r = mt.update(det, T); mt.learn();
            if (!r.ok) { fails++; continue; }
            T = r.camToWorld;
            if (f > 0) {
                // Compare with the true object motion relative to the first frame.
                double x0[6] = {0, 0, 0, 0, 0, 0}; Pose M0 = expTwist(x0); M0.t = V3{0, 20, 250};
                V3 probe{30, 0, 250}, viaTruth = M0.apply(M.applyInv(probe));
                maxErr = std::max(maxErr, norm(T.apply(probe)-viaTruth));
            }
        }
        CHECK(fails == 0, "Marker: kein Ausfall ueber 400 Grad");
        CHECK(maxErr < 0.5f, "Marker: Lagefehler unter 0,5 mm nach mehr als einer Runde (%.3f)", maxErr);
        CHECK(mt.map().size() == 16, "Marker: Karte ohne Doppelte (%zu von 16)", mt.map().size());

        // A wrong prior (tracking was off by far) must not end marker tracking for good.
        {
            std::vector<V3> pts = {{0,0,0},{60,5,0},{20,70,0},{-50,35,0},{-30,-65,0},{75,-40,0},{40,45,0},{-75,-10,0}};
            MarkerTracker rt; std::vector<Marker3D> d0; for (const V3& q : pts) { Marker3D m2; m2.pos = q + V3{0, 0, 250}; d0.push_back(m2); }
            rt.update(d0, Pose{});
            double tw2[6] = {0.1, 0.9, -0.2, 15, 8, -6}; Pose motion = expTwist(tw2);
            std::vector<Marker3D> d1; for (const Marker3D& m2 : d0) { Marker3D q = m2; q.pos = motion.apply(m2.pos); d1.push_back(q); }
            double farAway[6] = {0, 0, 0, 300, 300, 300}; MarkerResult rr = rt.update(d1, expTwist(farAway));
            float e2 = 0; for (const Marker3D& m2 : d0) e2 = std::max(e2, norm(rr.camToWorld.apply(motion.apply(m2.pos)) - m2.pos));
            CHECK(rr.ok && e2 < 0.1f, "Marker: Wiederfindung ohne brauchbare Startlage (Fehler %.3f mm)", e2);
        }
    }
    // 6d. The base plane found from markers separates object, underlay and things beside it.
    {
        std::vector<V3> pts; for (int i = 0; i < 12; i++) { float t = i*6.2831853f/12; pts.push_back(V3{60*std::cos(t), 30*std::sin(t), 300.f+0.5f*std::sin(t)*30}); }
        pts.push_back(V3{0, 0, 260});   // a false marker on the object
        BasePlane b = fitBasePlane(pts, V3{0, 0, 0});
        CHECK(b.valid && b.normal.z < -0.8f, "Unterlage: Ebene gefunden, Normale zur Kamera (%.2f)", b.normal.z);
        CHECK(std::fabs(b.radius-65) < 8, "Unterlage: Ringradius passt (%.1f mm)", b.radius);
        V3 above = b.center + b.normal*20.f, on = b.center + b.normal*0.1f;
        CHECK(b.height(above) > 19 && std::fabs(b.height(on)) < 0.5f && b.radial(above) < 0.5f, "Unterlage: Hoehe und Abstand stimmen");
    }
    // 6e. Removing fragments keeps sizeable separate parts and drops crumbs.
    {
        Tsdf t(1.f); DepthMap flat; flat.width = 200; flat.height = 200; flat.fx = flat.fy = 200; flat.cx = flat.cy = 100; flat.z.assign(40000, 0.f);
        for (int j = 20; j < 180; j++) for (int i = 20; i < 90; i++) flat.z[j*200+i] = 200;     // large part
        for (int j = 20; j < 110; j++) for (int i = 120; i < 180; i++) flat.z[j*200+i] = 200;   // second sizeable part
        for (int j = 170; j < 176; j++) for (int i = 170; i < 176; i++) flat.z[j*200+i] = 200;  // crumb
        for (int n = 0; n < 3; n++) t.integrate(flat, Pose{});
        MeshOptions keep; keep.removeFragments = false; keep.smoothIterations = 0; MeshOptions rem; rem.smoothIterations = 0;
        Mesh a, b; extractMesh(t, keep, a); extractMesh(t, rem, b);
        V3 lo, hi; meshBounds(b, lo, hi);
        CHECK(b.triangleCount() < a.triangleCount() && hi.x > 50, "Reste: Kruemel weg (%zu von %zu), zweites Teil bleibt (x bis %.0f mm)", b.triangleCount(), a.triangleCount(), hi.x);
    }
    // 6f. After losing the object, tracking resumes when an already scanned view returns.
    {
        PipelineOptions ro; ro.voxelMm = 0.75f; ro.zMax = 620.f; ro.useMarkers = false; Pipeline q(ro); q.setTurntable(true); q.setScanning(true);
        TurntableScene ts; StereoFrame fr; uint64_t idx = 0;
        auto feed = [&](float ang) { ts.angle = ang; renderTurntable(ts, c, fr); fr.index = idx++; q.process(fr, c); };
        for (int i = 0; i <= 80; i++) feed(i*0.5f);                     // scan up to 40 degrees
        float beforeAng = 0; { const Pose& T = q.pose(); beforeAng = std::acos(std::clamp((T.R[0]+T.R[4]+T.R[8]-1)/2, -1.f, 1.f))*57.29578f; }
        StereoFrame dark = fr; std::fill(dark.ref.begin(), dark.ref.end(), 0); std::fill(dark.other.begin(), dark.other.end(), 0);
        for (int i = 0; i < 5; i++) { dark.index = idx++; q.process(dark, c); }
        const int lostDuringDark = q.stats().lostFrames;
        for (int i = 0; i < 25; i++) feed(15.f);                        // the object is back, at a view seen earlier
        const Pose& T = q.pose(); float ang = std::acos(std::clamp((T.R[0]+T.R[4]+T.R[8]-1)/2, -1.f, 1.f))*57.29578f;
        printf("     Wiederaufnahme: vor Verlust %.1f Grad, danach %.1f Grad (wahr 15), verloren %d, wiedergefunden %d\n", beforeAng, ang, q.stats().lostFrames, q.stats().relocations);
        CHECK(lostDuringDark >= 5 && q.stats().relocations >= 1, "Wiederaufnahme ausgeloest");
        CHECK(q.stats().state == TrackState::Good && std::fabs(ang-15.f) < 2.f, "Wiederaufnahme an der richtigen Stelle (%.1f Grad)", ang);
    }
    // 6h. Handedness: an object on the +x side must come out on the +x side, despite mirrored images.
    {
        Scene sd; sd.c = V3{35, 0, 300}; sd.r = 20; sd.planeZ = 2000; StereoFrame sf; render(sd, c, Pose{}, sf);
        StereoMatcher sm2; sm2.params.shiftMin = 30; sm2.params.shiftMax = 444; DepthMap dm2; sm2.compute(sf.ref.data(), sf.other.data(), 1280, 800, c, dm2);
        double sx = 0; int cn = 0;
        for (int j = 0; j < dm2.height; j++) for (int i = 0; i < dm2.width; i++) { float z = dm2.z[j*dm2.width+i]; if (z > 260 && z < 320) { sx += (i-dm2.cx)/dm2.fx*z; cn++; } }
        CHECK(cn > 500 && std::fabs(sx/cn-35) < 6, "Seitenrichtig: Kugel bei x=35 wird bei x=%.1f rekonstruiert", cn ? sx/cn : 0.0);
    }
    // 6i. Color: a surface seen by a calibrated color camera takes on the color at its projection.
    {
        Tsdf t(1.f); DepthMap flat; flat.width = 200; flat.height = 200; flat.fx = -200; flat.fy = 200; flat.cx = flat.cy = 100; flat.z.assign(40000, 200.f);
        ColorCalib cc; cc.valid = true; cc.focalPx = 400; cc.cx = 200; cc.cy = 200; cc.refToColor.t = V3{10, 0, 0};
        ColorImage img; img.width = 400; img.height = 400; img.rgb.resize(400*400*3);
        for (int y = 0; y < 400; y++) for (int x = 0; x < 400; x++) { uint8_t* o = img.rgb.data()+3*(y*400+x); o[0] = x > 220 ? 220 : 20; o[1] = 90; o[2] = 40; }
        for (int n = 0; n < 4; n++) t.integrate(flat, Pose{}, 64, &img, &cc);
        Mesh m2; MeshOptions mo2; mo2.smoothIterations = 0; extractMesh(t, mo2, m2);
        // x_color = 400*(X+10)/200 + 200 > 220 exactly where X > 0 mm... with a 10 mm camera offset the edge lies at X = 0.
        double right = 0, left = 0; int nr = 0, nl = 0;
        if (m2.colors.size() == m2.vertices.size()*3) for (size_t i = 0; i < m2.vertices.size(); i++) {
            if (m2.vertices[i].x > 10) { right += m2.colors[i*3]; nr++; } else if (m2.vertices[i].x < -10) { left += m2.colors[i*3]; nl++; }
        }
        CHECK(nr > 50 && nl > 50 && right/nr > 180 && left/nl < 60, "Farbe liegt an der richtigen Stelle (rechts %.0f, links %.0f)", nr ? right/nr : 0.0, nl ? left/nl : 0.0);
    }
    // 6j. A finished scan can be rebuilt at another resolution from the same poses.
    {
        PipelineOptions ro; ro.voxelMm = 1.f; ro.zMax = 620.f; ro.useMarkers = false; Pipeline q(ro); q.setTurntable(true); q.setScanning(true);
        TurntableScene ts; StereoFrame fr; uint64_t idx = 0;
        for (int i = 0; i <= 30; i++) { ts.angle = i*0.5f; renderTurntable(ts, c, fr); fr.index = idx++; q.process(fr, c); }
        q.setScanning(false);
        Mesh a, b; MeshOptions mo3; mo3.smoothIterations = 0; q.buildMesh(mo3, a);
        std::string rep; const bool ok = q.rebuildModel(0.5f, rep); q.buildMesh(mo3, b);
        V3 la, ha, lb, hb; meshBounds(a, la, ha); meshBounds(b, lb, hb);
        CHECK(ok && b.triangleCount() > a.triangleCount()*2, "Neu berechnen: feineres Netz aus demselben Scan (%zu statt %zu Dreiecke)", b.triangleCount(), a.triangleCount());
        CHECK(std::fabs((hb.x-lb.x)-(ha.x-la.x)) < 3 && std::fabs((hb.z-lb.z)-(ha.z-la.z)) < 3, "Neu berechnen: gleiche Groesse (%.1f / %.1f mm)", hb.x-lb.x, ha.x-la.x);
    }
    // 6k. Upright export: up becomes +Z, the underside is at 0, and nothing gets mirrored.
    {
        Mesh box = makeBox(40, 55, 70, 0, 0); computeNormals(box);
        const double before = meshVolume(box);
        orientUpright(box, V3{0, 0, 1});   // the box's long side along z stays vertical
        V3 lo, hi; meshBounds(box, lo, hi);
        CHECK(std::fabs(lo.z) < 0.2f && std::fabs(hi.z-70) < 0.2f, "Aufrecht: Unterseite bei 0, Hoehe 70 (%.1f bis %.1f)", lo.z, hi.z);
        CHECK(std::fabs(lo.x+hi.x) < 0.1f && std::fabs(lo.y+hi.y) < 0.1f, "Aufrecht: mittig ausgerichtet");
        Mesh tilted = makeBox(40, 55, 70, 0, 0); computeNormals(tilted); orientUpright(tilted, V3{0, -1, 0});
        CHECK(std::fabs(meshVolume(tilted)/before-1) < 1e-4 && std::fabs(meshVolume(box)/before-1) < 1e-4, "Aufrecht: reine Drehung, nichts gespiegelt (Volumen %.5f %%)", 100*meshVolume(tilted)/before);
        meshBounds(tilted, lo, hi);
        CHECK(std::fabs((hi.z-lo.z)-55) < 0.3f, "Aufrecht: gewuenschte Richtung zeigt nach oben (Hoehe %.1f)", hi.z-lo.z);
    }
    // 6k2. A turned box must measure its true sides. A real cube, turned by 9 degrees, read 34.7 instead of 30 mm.
    {
        Mesh cube = makeBox(30, 40, 50, 0, 0); computeNormals(cube); Mesh turned = cube; rotateMesh(turned, 0, 0, 9); const V3 upZ{0, 0, 1};
        float s[3]; tightBoxSize(turned, s);
        CHECK(std::fabs(s[0]-50) < .1f && std::fabs(s[1]-40) < .1f && std::fabs(s[2]-30) < .1f, "Kleinste Box: um Z gedreht (%.2f %.2f %.2f)", s[0], s[1], s[2]);
        Mesh tumbled = cube; rotateMesh(tumbled, 33, 17, 52); tightBoxSize(tumbled, s);
        CHECK(std::fabs(s[0]-50) < .35f && std::fabs(s[1]-40) < .35f && std::fabs(s[2]-30) < .35f, "Kleinste Box: beliebig gedreht (%.2f %.2f %.2f)", s[0], s[1], s[2]);
        { Mesh withStray = turned; for (int k = 0; k < 3; k++) { withStray.vertices.push_back(V3{120.f, 5.f*k, 0}); } tightBoxSize(withStray, s); CHECK(s[0] < 52.f, "Kleinste Box: ein kleiner Fetzen verfaelscht sie nicht (%.2f)", s[0]); }
        orientUpright(turned, upZ); V3 lo, hi; meshBounds(turned, lo, hi);
        CHECK(std::fabs(hi.x-lo.x-30) < .2f && std::fabs(hi.y-lo.y-40) < .2f && std::fabs(hi.z-lo.z-50) < .2f, "Aufrecht: Grundflaeche liegt an den Achsen (%.2f %.2f %.2f)", hi.x-lo.x, hi.y-lo.y, hi.z-lo.z);
    }
    // 6k2b. The point cloud of a scan: the measured surface, without single stray points.
    {
        ScanData s; s.grid.width = 64; s.grid.height = 40; s.grid.fx = -80; s.grid.fy = 80; s.grid.cx = 32; s.grid.cy = 20;
        for (int k = 0; k < 8; k++) {
            ScanFrame f; for (int y = 8; y < 32; y++) for (int x = 16; x < 48; x++) { f.cells.push_back((uint32_t)(y*64+x)); f.z.push_back(200.f); }
            if (k == 0) { f.cells.push_back((uint32_t)(3*64+3)); f.z.push_back(150.f); }   // a stray point, once
            s.frames.push_back(std::move(f));
        }
        buildPointCloud(s); float worst = 0, minZ = 1e9f; bool facing = s.cloud && !s.cloud->vertices.empty();
        if (s.cloud) for (size_t i = 0; i < s.cloud->vertices.size(); i++) { worst = std::max(worst, std::fabs(s.cloud->vertices[i].z-200.f)); minZ = std::min(minZ, s.cloud->vertices[i].z); facing = facing && s.cloud->normals[i].z < -.9f; }
        CHECK(s.cloud && s.cloud->vertices.size() > 100, "Punktwolke: Oberflaeche ist da (%zu Punkte)", s.cloud ? s.cloud->vertices.size() : 0);
        CHECK(worst < .1f && minZ > 199.f, "Punktwolke: nur die Ebene, kein einzelner Ausreisser (Abweichung %.2f)", worst);
        CHECK(facing, "Punktwolke: Normalen zeigen zur Kamera");
        CHECK(s.cloud && s.cloud->indices.empty(), "Punktwolke: reine Punkte");
    }
    // 6k2c. Languages: the system decides, English when it is none of ours, and no line may break a format.
    {
        CHECK(languageFrom("de_DE.UTF-8") == Language::German && languageFrom("de_AT") == Language::German && languageFrom("fr_CA.UTF-8") == Language::French && languageFrom("es_MX.UTF-8") == Language::Spanish && languageFrom("en_GB") == Language::English, "Sprache: aus dem System");
        CHECK(languageFrom("it_IT.UTF-8") == Language::English && languageFrom("C") == Language::English && languageFrom("POSIX") == Language::English && languageFrom("") == Language::English && languageFrom("dev") == Language::English && languageFrom(nullptr) == Language::English, "Sprache: sonst Englisch");
        const auto conversions = [](const char* s) { std::string out; for (const char* p = s; *p; p++) if (*p == '%') { p++; if (*p == '%') continue; while (*p && !strchr("dsfgxucz", *p)) p++; if (*p == 'z') p++; if (*p) out += *p; else break; } return out; };
        int broken = 0, empty = 0, spaces = 0; for (int i = 0; i < translationCount(); i++) {
            const char* de = translationText(i, 0); const bool isDate = !strncmp(de, "%d.%m", 5);
            for (int l = 1; l < 4; l++) { const char* t = translationText(i, l); if (!*t) empty++; if (!isDate && conversions(de) != conversions(t)) { broken++; printf("     Platzhalter: %s\n", t); } const size_t a = strlen(de), b = strlen(t); if (a && b && ((de[a-1] == ' ') != (t[b-1] == ' ') || (de[0] == ' ') != (t[0] == ' '))) { spaces++; printf("     Leerzeichen am Rand: %s\n", t); } }
        }
        CHECK(translationCount() > 300 && !empty, "Sprachen: alle %d Texte vorhanden", translationCount());
        CHECK(!broken, "Sprachen: Platzhalter stimmen ueberall (%d Abweichungen)", broken);
        CHECK(!spaces, "Sprachen: Leerzeichen am Rand stimmen ueberall (%d Abweichungen)", spaces);
        const Language before = language(); setLanguage(Language::French);
        CHECK(!strcmp(tr("Speichern"), "Enregistrer"), "Sprachen: Uebersetzung (%s)", tr("Speichern"));
        CHECK(!strcmp(tr("Speichern##proj"), "Enregistrer##proj"), "Sprachen: die ID bleibt (%s)", tr("Speichern##proj"));
        CHECK(!strcmp(tr("\xEE\x85\x8D  Speichern##x"), "\xEE\x85\x8D  Enregistrer##x"), "Sprachen: das Symbol bleibt");
        CHECK(!strcmp(tr("Scan_17"), "Scan_17"), "Sprachen: Unbekanntes bleibt, wie es ist");
        CHECK(trf("Wird in %s geöffnet", "Bambu Studio") == "Ouverture dans Bambu Studio", "Sprachen: Wert im Satz");
        CHECK(!strcmp(localDecimals("62.8 x 50.35 x 45.6 mm"), "62,8 x 50,35 x 45,6 mm"), "Dezimalkomma: Werte (%s)", localDecimals("62.8 x 50.35 x 45.6 mm"));
        CHECK(!strcmp(localDecimals("1.874.488 triangles"), "1.874.488 triangles"), "Dezimalkomma: Tausender bleiben");
        setLanguage(Language::English); CHECK(!strcmp(localDecimals("62.8 mm"), "62.8 mm"), "Dezimalkomma: Englisch behaelt den Punkt");
        setLanguage(Language::German); CHECK(!strcmp(tr("Speichern"), "Speichern"), "Sprachen: Deutsch ist das Original"); CHECK(!strcmp(localDecimals("0.25 mm"), "0,25 mm"), "Dezimalkomma: Deutsch"); setLanguage(before);
    }
    // 6k3. What the program remembers, and where new projects go.
    {
        CHECK(sanitizeProjectName("  Eule/2\\\n. ") == "Eule2", "Projektname: Pfadzeichen und Raender entfallen (%s)", sanitizeProjectName("  Eule/2\\\n. ").c_str());
        CHECK(sanitizeProjectName(" ... ").empty(), "Projektname: nichts Brauchbares bleibt leer");
        CHECK(newProjectFile("/a/b", "Eule") == "/a/b/Eule/Eule.vfproj", "Neues Projekt: Unterordner mit Projektnamen");
        const auto dir = std::filesystem::temp_directory_path()/"voxelforge_test_settings"; std::filesystem::remove_all(dir); std::filesystem::create_directories(dir/"Eule");
        Settings s; s.projectDir = dir.string(); s.slicerPath[1] = "/opt/prusa"; s.scanRoll = 2; s.computeMode = 1;
        { std::ofstream((dir/"Eule"/"Eule.vfproj").string()) << "x"; std::ofstream((dir/"alt.vfproj").string()) << "yy"; }
        rememberProject(s, "/gibt/es/nicht.vfproj"); rememberProject(s, (dir/"Eule"/"Eule.vfproj").string());
        const std::string f = (dir/"settings.ini").string(); CHECK(saveSettings(s, f), "Einstellungen: speichern");
        const Settings r = loadSettings(f);
        CHECK(r.projectDir == s.projectDir && r.slicerPath[1] == "/opt/prusa" && r.scanRoll == 2 && r.computeMode == 1 && r.recent.size() == 2 && r.recent[0] == s.recent[0], "Einstellungen: alles kommt zurueck");
        auto list = listProjects(r); CHECK(list.size() == 2, "Projektliste: findet Projekte im Ordner, Fehlendes entfaellt (%zu)", list.size());
        Settings h = r; h.hidden.push_back((dir/"alt.vfproj").string()); CHECK(listProjects(h).size() == 1, "Projektliste: aus der Liste genommen");
        CHECK(thumbnailFile("/a/Eule.vfproj") == "/a/Eule.thumb.ppm", "Vorschaubild liegt neben dem Projekt");
        Mesh box = makeBox(30, 40, 50, 0, 0); computeNormals(box); const std::string tf = (dir/"t.ppm").string();
        CHECK(writeThumbnail(box, V3{0, 0, 1}, tf) && std::filesystem::file_size(tf) > 400000, "Vorschaubild wird geschrieben");
        { std::ifstream in(tf, std::ios::binary); std::string magic; int w, h2, mx; in >> magic >> w >> h2 >> mx; in.get(); std::vector<char> px((size_t)w*h2*3); in.read(px.data(), px.size()); int lit = 0; for (size_t i = 2; i < px.size(); i += 3) lit += (unsigned char)px[i] > 60; CHECK(magic == "P6" && lit > w*h2/20, "Vorschaubild zeigt das Modell (%d Punkte)", lit); }
        { Mesh wedge; wedge.vertices = {{-20, -20, 0}, {20, -20, 0}, {20, 20, 0}, {-20, 20, 0}, {0, 0, 60}}; wedge.indices = {0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4}; computeNormals(wedge); writeThumbnail(wedge, V3{0, 0, 1}, tf); std::ifstream in(tf, std::ios::binary); std::string magic; int w, h2, mx; in >> magic >> w >> h2 >> mx; in.get(); std::vector<unsigned char> px((size_t)w*h2*3); in.read((char*)px.data(), px.size()); auto width = [&](int y) { int n = 0; for (int x = 0; x < w; x++) n += px[3*((size_t)y*w+x)+2] > 60; return n; };
          int top = 0; while (top < h2 && !width(top)) top++; int bottom = h2-1; while (bottom > 0 && !width(bottom)) bottom--; CHECK(width(top+(bottom-top)/5) < width(bottom-(bottom-top)/5), "Vorschaubild: oben ist oben"); }
        std::filesystem::remove_all(dir);
        // Held upright: what is up lies along the camera's x axis. Right: its top leans right, so up is to the camera's left.
        CHECK(cameraUp(ScanRoll::Level).y == -1 && cameraUp(ScanRoll::Right).x == -1 && cameraUp(ScanRoll::Left).x == 1, "Scanner-Haltung: Oben folgt der Drehung");
    }
    // 6l. The mesh view must never mirror: this went wrong twice while the matrix was written by hand.
    {
        float worstDet = 0, worstUp = 2;
        for (float yaw : {0.f, 1.f, 2.5f, 4.f}) for (float pitch : {-1.f, 0.f, 0.35f, 1.4f}) {
            float R[9]; viewRotationZUp(yaw, pitch, R);
            const float det = R[0]*(R[4]*R[8]-R[5]*R[7]) - R[1]*(R[3]*R[8]-R[5]*R[6]) + R[2]*(R[3]*R[7]-R[4]*R[6]);
            worstDet = std::max(worstDet, std::fabs(det-1));
            worstUp = std::min(worstUp, R[5]);       // model Z must not point down on screen in this range
        }
        float Rt[9]; viewRotationZUp(0.f, 0.35f, Rt);
        CHECK(worstDet < 1e-4f, "Ansicht: reine Drehung, keine Spiegelung (Abweichung %.6f)", worstDet);
        CHECK(worstUp > 0 && Rt[8] > 0, "Ansicht: Z zeigt nach oben, bei positiver Neigung von oben gesehen");
    }
    // 6m. Hole filling and mesh reduction on an exactly known closed sphere.
    {
        Mesh sph = makeSphere(40, 6); const double volFull = meshVolume(sph); const size_t triFull = sph.triangleCount();
        CHECK(countBoundaryEdges(sph) == 0, "Netz: Testkugel ist geschlossen (%zu Dreiecke)", triFull);
        Mesh holed = sph; { std::vector<uint32_t> keep;   // cut a small cap and a large cap
            for (size_t i = 0; i < holed.indices.size(); i += 3) { V3 c = (holed.vertices[holed.indices[i]]+holed.vertices[holed.indices[i+1]]+holed.vertices[holed.indices[i+2]])*(1.f/3);
                if (norm(c-V3{0,0,40}) < 3 || norm(c-V3{0,0,-40}) < 25) continue; keep.insert(keep.end(), {holed.indices[i], holed.indices[i+1], holed.indices[i+2]}); } holed.indices.swap(keep); }
        Mesh small = holed; int nSmall = fillHoles(small, 8.f); size_t leftSmall = countBoundaryEdges(small);
        Mesh all = holed; int nAll = fillHoles(all, -1.f); const size_t leftAll = countBoundaryEdges(all);
        CHECK(nSmall == 1 && leftSmall > 0, "Loecher: nur das kleine gefuellt (%d, offen %zu Kanten)", nSmall, leftSmall);
        CHECK(nAll == 2 && leftAll == 0, "Loecher: alle gefuellt, wasserdicht (%d, offen %zu)", nAll, leftAll);
        CHECK(meshVolume(all) > 0.97*volFull && meshVolume(all) < 1.001*volFull, "Loecher: Fuellung zeigt nach aussen (Volumen %.1f %% )", 100*meshVolume(all)/volFull);
        {   // Two holes touching in a single vertex, as real scans have them.
            Mesh touch = makeSphere(40, 3); const uint32_t v = touch.indices[0];
            std::vector<uint32_t> tris; for (size_t i = 0; i < touch.indices.size(); i += 3) if (touch.indices[i] == v || touch.indices[i+1] == v || touch.indices[i+2] == v) tris.push_back((uint32_t)(i/3));
            std::vector<uint8_t> drop(touch.triangleCount(), 0); drop[tris[0]] = 1; drop[tris[tris.size()/2]] = 1;
            std::vector<uint32_t> keep; for (size_t t = 0; t < touch.triangleCount(); t++) if (!drop[t]) keep.insert(keep.end(), {touch.indices[3*t], touch.indices[3*t+1], touch.indices[3*t+2]});
            touch.indices.swap(keep); const size_t before = countBoundaryEdges(touch); const int closedTouch = fillHoles(touch, -1.f);
            CHECK(before > 0 && closedTouch >= 1 && countBoundaryEdges(touch) == 0, "Loecher: zwei beruehrende Loecher dicht (%d geschlossen, offen %zu)", closedTouch, countBoundaryEdges(touch));
        }
        Mesh red = sph; simplifyMesh(red, triFull/10);
        double maxErr2 = 0; for (const V3& p : red.vertices) maxErr2 = std::max(maxErr2, (double)std::fabs(norm(p)-40));
        CHECK(red.triangleCount() <= triFull/10+40 && red.triangleCount() > triFull/20, "Reduktion: Ziel erreicht (%zu von %zu)", red.triangleCount(), triFull);
        CHECK(countBoundaryEdges(red) == 0, "Reduktion: bleibt geschlossen");
        CHECK(maxErr2 < 0.5, "Reduktion: Form bleibt (max. %.3f mm Abweichung)", maxErr2);
        CHECK(std::fabs(meshVolume(red)/volFull-1) < 0.01, "Reduktion: Volumen bleibt (%.2f %%)", 100*meshVolume(red)/volFull);
        Mesh openRed = holed; const size_t edgesBefore = countBoundaryEdges(openRed); simplifyMesh(openRed, openRed.triangleCount()/4);
        double rimErr = 0; { std::map<uint64_t,int> ec; for (size_t i = 0; i < openRed.indices.size(); i += 3) for (int k = 0; k < 3; k++) { uint32_t a = openRed.indices[i+k], b = openRed.indices[i+(k+1)%3]; ec[((uint64_t)std::min(a,b)<<32)|std::max(a,b)]++; }
            for (auto& e : ec) if (e.second == 1) rimErr = std::max(rimErr, (double)std::fabs(norm(openRed.vertices[(uint32_t)e.first])-40)); }
        CHECK(countBoundaryEdges(openRed) > 0 && countBoundaryEdges(openRed) <= edgesBefore && rimErr < 0.5, "Reduktion: offene Raender bleiben erhalten (%.3f mm)", rimErr);
    }
    // 6n. Editing tools. Holes in a sphere and in a flat face have a known correct answer.
    {
        const float R = 40; Mesh sph = makeSphere(R, 6); computeNormals(sph); const double volFull = meshVolume(sph);
        auto cut = [](const Mesh& in, V3 c, float r) { Mesh o = in; std::vector<uint8_t> mark(o.triangleCount());
            for (size_t t = 0; t < mark.size(); t++) { V3 g = (o.vertices[o.indices[3*t]]+o.vertices[o.indices[3*t+1]]+o.vertices[o.indices[3*t+2]])*(1.f/3); mark[t] = norm(g-c) < r; }
            deleteTriangles(o, mark); return o; };
        auto closedBox = [](float a, float b, float c) { Mesh x = makeBox(a, b, c, 0, 0); for (V3& p : x.vertices) p = V3{std::round(p.x*1000)/1000, std::round(p.y*1000)/1000, std::round(p.z*1000)/1000}; weldVertices(x); computeNormals(x); return x; };
        CHECK(countBoundaryEdges(closedBox(60, 60, 60)) == 0, "Testwuerfel ist nach dem Verschmelzen geschlossen");
        auto nonManifold = [](const Mesh& m) { std::map<uint64_t,int> ec; for (size_t i = 0; i < m.indices.size(); i += 3) for (int k = 0; k < 3; k++) { uint32_t a = m.indices[i+k], b = m.indices[i+(k+1)%3]; ec[((uint64_t)std::min(a,b)<<32)|std::max(a,b)]++; } int n = 0; for (auto& e : ec) n += e.second > 2; return n; };
        Mesh holed = cut(cut(sph, V3{0,0,R}, 6), V3{0,0,-R}, 25);
        auto holes = findHoles(holed);
        CHECK(holes.size() == 2 && holes[0].perimeterMm < holes[1].perimeterMm, "Loecher: zwei gefunden und nach Groesse sortiert (%zu)", holes.size());
        if (holes.size() == 2) {
            // The rim follows triangle edges, so it zigzags around the circle and is somewhat longer.
            CHECK(holes[0].perimeterMm > 2*3.14159f*5 && holes[0].perimeterMm < 2*3.14159f*6*1.5f && holes[1].perimeterMm > 2*3.14159f*22 && holes[1].perimeterMm < 2*3.14159f*25*1.5f, "Loecher: Umfang plausibel (%.1f und %.1f mm)", holes[0].perimeterMm, holes[1].perimeterMm);
            CHECK(norm(holes[0].center-V3{0,0,R}) < 2, "Loecher: Position stimmt");
        }
        double err[3] = {0, 0, 0}; const char* names[3] = {"Planar", "Tangente", "Kruemmung"};
        for (int ty = 0; ty < 3; ty++) {
            Mesh f = holed; const size_t tri0 = f.triangleCount();
            const int n = fillHolesUpTo(f, -1, (FillType)ty);
            CHECK(n == 2 && countBoundaryEdges(f) == 0 && nonManifold(f) == 0, "Loecher %s: wasserdicht und sauber (%d, offen %zu, Knoten %d)", names[ty], n, countBoundaryEdges(f), nonManifold(f));
            double e = 0; for (size_t t = tri0*3; t < f.indices.size(); t++) e = std::max(e, (double)std::fabs(norm(f.vertices[f.indices[t]])-R)); err[ty] = e;
            CHECK(f.normals.size() == f.vertices.size(), "Loecher %s: Normalen vorhanden", names[ty]);
            const double vol = meshVolume(f); CHECK(vol > 0.9*volFull && vol < 1.05*volFull, "Loecher %s: Fuellung zeigt nach aussen (Volumen %.1f %%), Abweichung von der Kugel %.2f mm", names[ty], 100*vol/volFull, e);
        }
        CHECK(err[1] < err[0]*0.5 && err[2] <= err[1]*1.05, "Loecher: Tangente und Kruemmung treffen die Kugel besser als Planar (%.2f / %.2f / %.2f mm)", err[0], err[1], err[2]);
        {   // One hole only, the other stays.
            Mesh f = holed; auto hs = findHoles(f); hs.resize(1);
            CHECK(fillHoleList(f, hs, FillType::Tangent) == 1 && findHoles(f).size() == 1, "Loecher: einzeln gefuellt, das andere bleibt");
            { // A long narrow seam closes, a wide opening of the same perimeter stays.
                Mesh seam = makeSphere(10, 6); computeNormals(seam); std::vector<uint8_t> cut(seam.triangleCount(), 0);
                for (size_t t = 0; t < seam.triangleCount(); t++) { const V3 p = (seam.vertices[seam.indices[3*t]]+seam.vertices[seam.indices[3*t+1]]+seam.vertices[seam.indices[3*t+2]])*(1.f/3); cut[t] = std::fabs(p.x) < .3f && p.z > 0; }
                deleteTriangles(seam, cut); weldVertices(seam); auto before = findHoles(seam);
                const float seamPerimeter = before.empty() ? 0 : before.back().perimeterMm;
                fillHolesUpTo(seam, 5.f, FillType::Tangent);
                CHECK(!before.empty() && seamPerimeter > 5.f && findHoles(seam).empty(), "Loecher: langer Schlitz schliesst trotz Umfang %.0f mm", seamPerimeter);
                Mesh wide = makeSphere(10, 6); computeNormals(wide); std::vector<uint8_t> cap(wide.triangleCount(), 0);
                for (size_t t = 0; t < wide.triangleCount(); t++) cap[t] = wide.vertices[wide.indices[3*t]].z > 7.f;
                deleteTriangles(wide, cap); weldVertices(wide); fillHolesUpTo(wide, 5.f, FillType::Tangent);
                CHECK(findHoles(wide).size() == 1, "Loecher: breite Oeffnung bleibt offen");
            }
            Mesh g = holed; CHECK(fillHolesUpTo(g, 50.f, FillType::Planar) == 1 && findHoles(g).size() == 1, "Loecher: Lochumfang begrenzt die Auswahl");
        }
        {   // A hole in a flat face must stay exactly flat with every type.
            Mesh box = closedBox(60, 60, 60);
            Mesh open = cut(box, V3{30, 30, 60}, 12); auto hs = findHoles(open);
            CHECK(hs.size() == 1, "Loch in ebener Flaeche gefunden (%zu)", hs.size());
            for (int ty = 0; ty < 3; ty++) {
                Mesh f = open; const size_t tri0 = f.triangleCount(); fillHoleList(f, hs, (FillType)ty);
                double e = 0; for (size_t t = tri0*3; t < f.indices.size(); t++) e = std::max(e, (double)std::fabs(f.vertices[f.indices[t]].z-60));
                CHECK(e < 1e-3 && countBoundaryEdges(f) == 0, "Ebene Flaeche %s bleibt eben (%.5f mm)", names[ty], e);
            }
        }
        {   // A long rim needs the cut in pieces, and the filling must still close and stay close to the shape.
            Mesh f = cut(sph, V3{0,0,0}, 0); std::vector<uint8_t> mark(f.triangleCount());
            for (size_t t = 0; t < mark.size(); t++) mark[t] = f.vertices[f.indices[3*t]].z < -10;
            deleteTriangles(f, mark); auto hs = findHoles(f);
            CHECK(hs.size() == 1 && hs[0].loop.size() > 180, "Langer Rand vorhanden (%zu Punkte)", hs.empty() ? 0 : hs[0].loop.size());
            CHECK(fillHoleList(f, hs, FillType::Tangent) == 1 && countBoundaryEdges(f) == 0 && nonManifold(f) == 0, "Langer Rand: dicht und sauber (offen %zu, Knoten %d)", countBoundaryEdges(f), nonManifold(f));
        }
        {   // Colors flow into the filling, and reduction keeps them.
            Mesh f = holed; f.colors.assign(f.vertices.size()*3, 200); auto hs = findHoles(f); fillHoleList(f, hs, FillType::Tangent);
            bool col = f.colors.size() == f.vertices.size()*3; for (size_t i = 0; col && i < f.colors.size(); i++) col = f.colors[i] >= 198 && f.colors[i] <= 200;
            CHECK(col, "Loecher: Farbe wird uebernommen");
            simplifyMesh(f, f.triangleCount()/4); CHECK(f.colors.size() == f.vertices.size()*3 && f.colors[0] >= 198, "Reduktion: Farbe bleibt erhalten");
        }
        {   // Bridges: one edge pair inside a hole splits it, a pair across two holes joins them.
            Mesh f = cut(sph, V3{0,0,R}, 12); auto hs = findHoles(f); const auto& l = hs[0].loop; const size_t n = l.size();
            CHECK(bridgeEdges(f, l[0], l[1], l[n/2], l[n/2+1]) && findHoles(f).size() == 2, "Bruecke teilt ein Loch in zwei (%zu)", findHoles(f).size());
            CHECK(fillHolesUpTo(f, -1, FillType::Planar) == 2 && countBoundaryEdges(f) == 0 && nonManifold(f) == 0, "Bruecke: beide Teile lassen sich schliessen");
            Mesh g = cut(cut(sph, V3{0,0,R}, 8), V3{14,0,R-3}, 4); auto h2 = findHoles(g);
            if (h2.size() == 2) { CHECK(bridgeEdges(g, h2[0].loop[0], h2[0].loop[1], h2[1].loop[0], h2[1].loop[1]) && findHoles(g).size() == 1, "Bruecke verbindet zwei Loecher zu einem (%zu)", findHoles(g).size()); }
            else CHECK(false, "Bruecke: Testaufbau hat %zu Loecher", h2.size());
        }
        {   // Smoothing: noise goes, volume stays, and open rims do not move.
            Mesh noisy = sph; std::mt19937 g(5); std::normal_distribution<float> nd(0.f, 0.15f); for (V3& p : noisy.vertices) p = p*(1.f+nd(g)/R);
            auto rms = [&](const Mesh& m) { double s = 0; for (const V3& p : m.vertices) { double d = norm(p)-R; s += d*d; } return std::sqrt(s/m.vertices.size()); };
            Mesh sm = noisy; smoothSurface(sm, {smoothIterationsFor(Level::Medium), false});
            CHECK(rms(sm) < rms(noisy)*0.5 && std::fabs(meshVolume(sm)/volFull-1) < 0.01, "Glaetten: Rauschen %.3f -> %.3f mm, Volumen %.2f %%", rms(noisy), rms(sm), 100*meshVolume(sm)/volFull);
            Mesh op = holed; auto before = boundaryLoops(op); std::vector<V3> rim; for (uint32_t v : before[0]) rim.push_back(op.vertices[v]);
            smoothSurface(op, {8, false}); double mv = 0; for (size_t i = 0; i < rim.size(); i++) mv = std::max(mv, (double)norm(rim[i]-op.vertices[before[0][i]]));
            CHECK(mv == 0, "Glaetten: offene Raender bleiben stehen");
            Mesh rm = holed; std::vector<uint32_t> lp = findHoles(rm)[1].loop; std::mt19937 g2(9); for (uint32_t v : lp) rm.vertices[v] = rm.vertices[v]+V3{nd(g2), nd(g2), 0};
            auto jag = [&](const Mesh& m) { double s = 0; for (size_t i = 0; i < lp.size(); i++) s += norm(m.vertices[lp[i]]-m.vertices[lp[(i+1)%lp.size()]]); return s; };
            const double j0 = jag(rm); smoothRims(rm, Level::High); CHECK(jag(rm) < j0, "Randglaettung: Umriss wird ruhiger (%.1f -> %.1f mm)", j0, jag(rm));
            // keepEdges: a cube edge stays sharper.
            Mesh c0 = closedBox(60, 60, 60); Mesh a = c0, b = c0;
            smoothSurface(a, {20, false}); smoothSurface(b, {20, true});
            auto corner = [](const Mesh& m) { double d = 1e9; for (const V3& p : m.vertices) d = std::min(d, (double)norm(p-V3{60, 60, 60})); return d; };
            CHECK(corner(b) < corner(a), "Glaetten: Kanten erhalten wirkt (Ecke %.3f statt %.3f mm entfernt)", corner(b), corner(a));
        }
        {   // Spikes go, but real corners stay.
            Mesh s = sph; std::vector<float> dummy; size_t tip = 100; s.vertices[tip] = s.vertices[tip]*1.3f; computeNormals(s);
            CHECK(removeSpikes(s, Level::Medium) > 0 && std::fabs(norm(s.vertices[tip])-R) < 1.5f, "Spitzen: Nadel entfernt (%.2f mm uebrig)", norm(s.vertices[tip])-R);
            Mesh box = closedBox(60, 60, 60); Mesh c = box;
            CHECK(removeSpikes(c, Level::High) == 0, "Spitzen: Wuerfelecken bleiben auch auf Stufe Hoch");
        }
        {   // Parts, selection, deletion.
            Mesh two = makeSphere(20, 4); const size_t a = two.triangleCount(); Mesh small = makeSphere(3, 1); const uint32_t off = (uint32_t)two.vertices.size();
            for (V3 p : small.vertices) two.vertices.push_back(p+V3{50,0,0}); for (uint32_t v : small.indices) two.indices.push_back(v+off); computeNormals(two);
            std::vector<uint32_t> part; triangleParts(two, part); CHECK(part[0] != part[a], "Auswahl: zwei getrennte Teile erkannt");
            Mesh k = two; CHECK(removeSmallParts(k, 0.05f) == 1 && k.triangleCount() == a, "Kleine Teile: entfernt (%zu Dreiecke)", k.triangleCount());
            Mesh l = two; CHECK(removeSmallParts(l, 0.f) == 0 && l.triangleCount() == two.triangleCount(), "Kleine Teile: Anteil 0 entfernt nichts");
            std::vector<uint8_t> mk(two.triangleCount(), 0); for (size_t t = a; t < mk.size(); t++) mk[t] = 1; Mesh d = two;
            CHECK(deleteTriangles(d, mk) == two.triangleCount()-a && d.vertices.size() == off && d.normals.size() == off, "Loeschen: Dreiecke und Punkte entfernt");
        }
        {   // Cut plane: three points on Z = 10 with the normal up mark everything below.
            Mesh s = sph; CutPlane pl; CHECK(planeFromPoints({0,0,10}, {10,0,10}, {0,10,10}, pl) && pl.normal.z > 0.99f && std::fabs(pl.d-10) < 1e-4f, "Schnittebene aus drei Punkten");
            CutPlane bad; CHECK(!planeFromPoints({0,0,0}, {1,0,0}, {2,0,0}, bad), "Schnittebene: Punkte auf einer Linie abgelehnt");
            std::vector<uint8_t> mk; markBelowPlane(s, pl, mk); deleteTriangles(s, mk); V3 lo, hi; meshBounds(s, lo, hi);
            CHECK(lo.z > 9 && hi.z > 39, "Schnittebene: unterer Teil entfernt (z ab %.2f)", lo.z);
            Mesh box = closedBox(80, 80, 20); CutPlane big; CHECK(largestPlane(box, big) && std::fabs(big.normal.z) > 0.99f, "Groesste Ebene gefunden (Normale z %.2f)", big.normal.z);
            Mesh r = makeBox(60, 40, 20, 0, 0); rotateMesh(r, 0, 0, 90); V3 l2, h2; meshBounds(r, l2, h2); CHECK(std::fabs(h2.x-l2.x-40) < 0.01f && std::fabs(h2.y-l2.y-60) < 0.01f, "Ausrichten: Drehung um Z vertauscht X und Y");
            Mesh rd = makeBox(60, 40, 20, 0, 0); const double v0 = meshVolume(rd); rotateMesh(rd, 33, 12, 77); CHECK(std::fabs(meshVolume(rd)/v0-1) < 1e-3, "Ausrichten: reine Drehung, nicht gespiegelt");
        }
        {   // Knots: the real scans have edges with 3 or 4 triangles, where rims used to break off.
            Mesh two = closedBox(20, 20, 20); const uint32_t off = (uint32_t)two.vertices.size(); Mesh b2 = closedBox(20, 20, 20);
            for (V3 p : b2.vertices) two.vertices.push_back(p+V3{20, 20, 0}); for (uint32_t v : b2.indices) two.indices.push_back(v+off); weldVertices(two); computeNormals(two);
            CHECK(countKnotEdges(two) > 0, "Knoten: zwei Wuerfel beruehren sich an einer Kante (%zu Knotenkanten)", countKnotEdges(two));
            const double vb = meshVolume(two); size_t added = repairManifold(two);
            std::vector<uint32_t> pt; triangleParts(two, pt); std::map<uint32_t,int> parts; for (uint32_t x : pt) parts[x]++;
            CHECK(added > 0 && countKnotEdges(two) == 0 && countBoundaryEdges(two) == 0, "Knoten: getrennt und weiter geschlossen (+%zu Punkte, Knoten %zu, offen %zu)", added, countKnotEdges(two), countBoundaryEdges(two));
            CHECK(parts.size() == 2, "Knoten: um das Innere verbunden, also zwei Koerper (%zu)", parts.size());
            CHECK(std::fabs(meshVolume(two)/vb-1) < 1e-3, "Knoten: Form bleibt (Volumen %.3f %%)", 100*meshVolume(two)/vb);
            Mesh stl = two; std::string er; const bool sv = saveMesh(stl, "vf_knot.stl", er); Mesh back; const bool ld = sv && loadMeshFile("vf_knot.stl", back, er); remove("vf_knot.stl");
            CHECK(ld && countKnotEdges(back) == 0 && countBoundaryEdges(back) == 0, "Knoten: bleiben auch nach STL getrennt (%zu)", countKnotEdges(back));
            Mesh three = makeSphere(20, 3); { std::vector<uint32_t> keep; const uint32_t a0 = three.indices[0], a1 = three.indices[1]; const uint32_t extra = (uint32_t)three.vertices.size(); three.vertices.push_back(V3{50, 0, 0}); keep = three.indices; keep.insert(keep.end(), {a0, a1, extra}); three.indices.swap(keep); }
            computeNormals(three); CHECK(countKnotEdges(three) == 1, "Knoten: Kante mit drei Dreiecken als Testfall");
            repairManifold(three); auto loops = boundaryLoops(three); size_t inLoops = 0; for (auto& l : loops) inLoops += l.size();
            CHECK(countKnotEdges(three) == 0 && inLoops == countBoundaryEdges(three), "Knoten: alle offenen Kanten liegen auf geschlossenen Raendern (%zu von %zu)", inLoops, countBoundaryEdges(three));
            {   // A knot that cannot be separated, because the sheets also hang together elsewhere: a sphere
                // with a fin glued to it on both sides of one edge. Afterwards it must still fill up cleanly.
                Mesh f = makeSphere(20, 3); const uint32_t a = f.indices[0], b = f.indices[1], e = (uint32_t)f.vertices.size(); f.vertices.push_back(f.vertices[a]*1.5f);
                f.indices.insert(f.indices.end(), {a, b, e, b, a, e}); computeNormals(f); CHECK(countKnotEdges(f) >= 1, "Knoten: untrennbare Kante als Testfall");
                repairManifold(f); auto lp = boundaryLoops(f); size_t on = 0; for (auto& l : lp) on += l.size();
                CHECK(countKnotEdges(f) == 0 && on == countBoundaryEdges(f), "Knoten: entfernt, Raender geschlossen (%zu von %zu)", on, countBoundaryEdges(f));
                fillHolesUpTo(f, -1, FillType::Tangent); CHECK(countBoundaryEdges(f) == 0 && countKnotEdges(f) == 0, "Knoten: danach wasserdicht");
            }
            Mesh bow = makeSphere(10, 2); const uint32_t o2 = (uint32_t)bow.vertices.size(), v0 = 0; Mesh b3 = makeSphere(10, 2); std::vector<uint32_t> map(b3.vertices.size()); for (size_t i = 0; i < map.size(); i++) { if (i == 1) map[i] = v0; else { map[i] = (uint32_t)bow.vertices.size(); bow.vertices.push_back(b3.vertices[i]+V3{20, 0, 0}); } } for (uint32_t v : b3.indices) bow.indices.push_back(map[v]); (void)o2;
            CHECK(repairManifold(bow) >= 1 && countBoundaryEdges(bow) == 0, "Knoten: Koerper, die sich nur in einem Punkt beruehren, werden getrennt");
        }
        {   // Undo and redo.
            MeshHistory h; Mesh m = makeSphere(10, 2); const size_t n0 = m.triangleCount(); h.push(m); simplifyMesh(m, n0/2); const size_t n1 = m.triangleCount();
            CHECK(h.undo(m) && m.triangleCount() == n0, "Rueckgaengig stellt den alten Stand her"); CHECK(h.redo(m) && m.triangleCount() == n1, "Wiederholen stellt den neuen Stand her");
            CHECK(!h.redo(m) && h.undo(m) && !h.undo(m), "Rueckgaengig: Grenzen eingehalten");
            h.maxBytes = 1; h.push(m); h.push(m); h.push(m); int cnt = 0; Mesh tmp = m; while (h.undo(tmp)) cnt++; CHECK(cnt == 1, "Rueckgaengig: Speichergrenze wirkt, ein Schritt bleibt (%d)", cnt);
        }
        {   // One click runs all steps and leaves a clean, closed result.
            Mesh in = holed; const uint32_t off = (uint32_t)in.vertices.size(); Mesh crumb = makeSphere(1, 1); for (V3 p : crumb.vertices) in.vertices.push_back(p+V3{70,0,0}); for (uint32_t v : crumb.indices) in.indices.push_back(v+off); computeNormals(in);
            OneClickConfig cfg; cfg.keepTriangles = 0.5f; cfg.maxHolePerimeterMm = -1; int calls = 0; float last = -1; bool mono = true;
            OneClickReport rp = runOneClick(in, cfg, [&](const char*, float f) { calls++; mono = mono && f >= last; last = f; });
            CHECK(rp.partsRemoved == 1 && rp.holesClosed == 2 && rp.holesLeft == 0, "Ein-Klick: Reste weg, Loecher zu (%d, %d, %d)", rp.partsRemoved, rp.holesClosed, rp.holesLeft);
            CHECK(countBoundaryEdges(in) == 0 && in.normals.size() == in.vertices.size(), "Ein-Klick: Ergebnis ist wasserdicht");
            CHECK(rp.trianglesAfter < rp.trianglesBefore*0.7 && calls >= 6 && mono && last == 1.f, "Ein-Klick: Reduktion und Fortschritt (%zu -> %zu)", rp.trianglesBefore, rp.trianglesAfter);
            V3 lo, hi; meshBounds(in, lo, hi); CHECK(hi.x < 45, "Ein-Klick: Form bleibt erhalten");
            Mesh keep = holed; OneClickConfig lim; lim.maxHolePerimeterMm = 50; OneClickReport r2 = runOneClick(keep, lim); CHECK(r2.holesClosed == 1 && r2.holesLeft == 1, "Ein-Klick: grosse Oeffnung bleibt unter dem Grenzwert offen");
            OneClickConfig w; w.fillType = FillType::Curvature; w.smooth = Level::High; w.maxHolePerimeterMm = 321; w.keepEdges = false; CHECK(saveOneClickConfig(w, "vf_cfg.tmp"), "Ein-Klick: Einstellungen speichern");
            OneClickConfig rd; CHECK(loadOneClickConfig("vf_cfg.tmp", rd) && rd.fillType == FillType::Curvature && rd.smooth == Level::High && rd.maxHolePerimeterMm == 321 && !rd.keepEdges, "Ein-Klick: Einstellungen laden"); remove("vf_cfg.tmp");
        }
        {   // Files: all formats come back with connections, so that holes can be found.
            std::string err; Mesh a = holed; a.colors.assign(a.vertices.size()*3, 90); computeNormals(a);
            for (const char* name : {"vf_io.stl", "vf_io.obj", "vf_io.ply"}) {
                Mesh b; const bool ok = saveMesh(a, name, err) && loadMeshFile(name, b, err); remove(name);
                CHECK(ok && b.triangleCount() == a.triangleCount() && findHoles(b).size() == 2, "Datei %s: Netz und Loecher bleiben (%zu Dreiecke)", name, b.triangleCount());
                if (strstr(name, "stl") == nullptr) CHECK(b.colors.size() == b.vertices.size()*3 && b.colors[0] >= 89 && b.colors[0] <= 91, "Datei %s: Farbe bleibt", name);
            }
            Mesh c; CHECK(!loadMeshFile("gibt_es_nicht.stl", c, err) && !loadMeshFile("x.xyz", c, err), "Datei: Fehler werden gemeldet");
        }
    }
    // 6o. Several scans. An uneven shape cut into two overlapping views, one of them turned far away.
    {
        // A sphere with bumps of different size: no symmetry, as real objects.
        Mesh shape = makeSphere(30, 6);
        const V3 bumps[6] = {{1,0,0},{-.2f,1,.3f},{.3f,-.6f,.8f},{-.8f,-.3f,-.4f},{.2f,.5f,-.9f},{-.5f,.7f,.6f}}; const float h[6] = {12, 7, 9, 5, 14, 4}, w[6] = {.35f, .2f, .25f, .15f, .4f, .12f};
        for (V3& p : shape.vertices) { const V3 d = normalized(p); float r = 30; for (int i = 0; i < 6; i++) { const float a = norm(d-normalized(bumps[i])); r += h[i]*std::exp(-a*a/(2*w[i]*w[i])); } p = d*r; }
        computeNormals(shape);
        auto view = [](const Mesh& in, V3 dir, float keep) { Mesh o = in; std::vector<uint8_t> mk(o.triangleCount()); for (size_t t = 0; t < mk.size(); t++) mk[t] = dot(o.vertices[o.indices[3*t]], dir) < -keep; deleteTriangles(o, mk); computeNormals(o); return o; };
        const Mesh dst = view(shape, V3{0,0,1}, 8);        // upper part
        Mesh src = view(shape, V3{0,0,-1}, 8);             // lower part: the overlap is only a band
        const double tw[6] = {1.1, -0.7, 2.2, 40, -25, 60}; const Pose truth = expTwist(tw);
        Pose inv; for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) inv.R[r*3+c] = truth.R[c*3+r]; inv.t = inv.rot(truth.t*-1.f);
        for (V3& p : src.vertices) p = inv.apply(p);
        computeNormals(src);
        auto res = registerAuto(src, dst);
        CHECK(!res.empty(), "Ausrichten: Vorschlag gefunden (%zu)", res.size());
        if (!res.empty()) {
            auto err = [&](const Pose& P) { double e = 0; for (const V3& p : src.vertices) e = std::max(e, (double)norm(P.apply(p)-truth.apply(p))); return e; };
            CHECK(err(res[0].srcToDst) < 1.0, "Ausrichten: trifft die bekannte Lage (max. %.3f mm, Ueberlappung %.0f %%, Rest %.3f mm)", err(res[0].srcToDst), 100*res[0].overlap, res[0].rmsMm);
            CHECK(res[0].confident, "Ausrichten: eindeutige Lage gilt als sicher (Konflikt %.2f %%, %zu Vorschlaege, naechster %.3f gegen %.3f)", 100*res[0].conflict, res.size(), res.size() > 1 ? res[1].score : 0.f, res[0].score);
            const float a = res[0].srcToDst.R[0]*(res[0].srcToDst.R[4]*res[0].srcToDst.R[8]-res[0].srcToDst.R[5]*res[0].srcToDst.R[7])-res[0].srcToDst.R[1]*(res[0].srcToDst.R[3]*res[0].srcToDst.R[8]-res[0].srcToDst.R[5]*res[0].srcToDst.R[6])+res[0].srcToDst.R[2]*(res[0].srcToDst.R[3]*res[0].srcToDst.R[7]-res[0].srcToDst.R[4]*res[0].srcToDst.R[6]);
            CHECK(std::fabs(a-1) < 1e-3, "Ausrichten: reine Drehung, nicht gespiegelt");
        }
        // Parts without any overlap must not be reported as certain, whatever fits by chance.
        { const Mesh top = view(shape, V3{0,0,1}, -10), bottom = view(shape, V3{0,0,-1}, -10); auto none = registerAuto(bottom, top); CHECK(none.empty() || !none[0].confident, "Ausrichten: ohne Ueberlappung nie sicher (%zu Vorschlaege)", none.size()); }
        // Rough start, then fine alignment.
        const double nudge[6] = {0.04, -0.03, 0.02, 1.2, -0.8, 0.9}; const RegisterResult fine = refineRegistration(src, dst, mul(expTwist(nudge), truth), 4);
        { double e = 0; for (const V3& p : src.vertices) e = std::max(e, (double)norm(fine.srcToDst.apply(p)-truth.apply(p))); CHECK(fine.ok && e < 0.3, "Feinausrichtung: aus grober Lage (%.3f mm)", e); }
        // Point pairs.
        { std::vector<V3> a{{0,0,0},{10,0,0},{0,20,0},{5,5,9}}, b; for (const V3& p : a) b.push_back(truth.apply(p)); Pose P; CHECK(registerPointPairs(a, b, P) && norm(P.apply(V3{7,7,7})-truth.apply(V3{7,7,7})) < 0.01f, "Punktpaare: Lage stimmt");
          std::vector<V3> line{{0,0,0},{1,0,0},{2,0,0}}; Pose Q; CHECK(!registerPointPairs(line, line, Q), "Punktpaare: Punkte auf einer Linie abgelehnt"); CHECK(!registerPointPairs({{0,0,0},{1,0,0}}, {{0,0,0},{1,0,0}}, Q), "Punktpaare: zu wenige abgelehnt"); }
        // Marker maps with some markers missing and some extra ones.
        { std::mt19937 g(4); std::uniform_real_distribution<float> u(-80, 80); std::vector<V3> full; for (int i = 0; i < 14; i++) full.push_back(V3{u(g), u(g), 0.f});
          std::vector<V3> a(full.begin(), full.begin()+10), b; for (size_t i = 3; i < full.size(); i++) b.push_back(truth.apply(full[i])); b.push_back(V3{500, 0, 0});
          Pose P; int mt = 0; CHECK(registerMarkers(a, b, P, mt) && mt >= 6 && norm(P.apply(V3{1,2,3})-truth.apply(V3{1,2,3})) < 0.05f, "Marker: Lage aus gemeinsamen Markern (%d)", mt);
          std::vector<V3> few{{0,0,0},{50,0,0}}; CHECK(!registerMarkers(few, b, P, mt), "Marker: zu wenige abgelehnt"); }
        // A project: two scans of the synthetic scene, the second seen from a moved rig, fused and saved.
        Project pr; auto makeScan = [&](const char* name, const Pose& worldMove) {
            ScanData s; s.name = name; DepthMap d; d.width = 320; d.height = 200; d.fx = 400; d.fy = 400; d.cx = 160; d.cy = 100; s.grid = d; s.grid.z.clear();
            for (int f = 0; f < 6; f++) { ScanFrame fr; fr.pose = Pose{}; fr.pose.t = V3{(float)f, 0, 0}; for (int y = 40; y < 160; y++) for (int x = 60; x < 260; x++) { fr.cells.push_back((uint32_t)(y*320+x)); fr.z.push_back(300.f); } s.frames.push_back(fr); }
            s.placement = worldMove; return s; };
        Pose shift; shift.t = V3{0, 0, 0}; pr.scans.push_back(makeScan("Scan_1", Pose{})); pr.scans[0].placed = true; pr.scans.push_back(makeScan("Scan_2", shift));
        CHECK(pr.nextName() == "Scan_3", "Projekt: naechster Name (%s)", pr.nextName().c_str());
        Mesh merged; std::string er; MeshOptions mop; mop.minWeight = 1; mop.smoothIterations = 0; mop.fillHolesMm = 0;
        CHECK(!fuseProject(pr, 1.f, mop, merged, er) && er.find("Scan_2") != std::string::npos, "Projekt: unausgerichteter Scan wird gemeldet (%s)", er.c_str());
        pr.scans[1].placed = true; Mesh one; { Project single; single.scans.push_back(makeScan("A", Pose{})); single.scans[0].placed = true; std::string e2; fuseProject(single, 1.f, mop, one, e2); }
        CHECK(fuseProject(pr, 1.f, mop, merged, er) && std::llabs((long long)merged.triangleCount()-(long long)one.triangleCount()) < (long long)one.triangleCount()/10, "Projekt: gleiche Flaeche zweimal ergibt keine doppelte Schicht (%zu statt %zu)", merged.triangleCount(), one.triangleCount());
        Project moved; moved.scans.push_back(makeScan("A", Pose{})); moved.scans[0].placed = true; Pose far; far.t = V3{0, 0, 500}; moved.scans.push_back(makeScan("B", far)); moved.scans[1].placed = true; Mesh two; fuseProject(moved, 1.f, mop, two, er);
        { V3 lo, hi; meshBounds(two, lo, hi); CHECK(hi.z-lo.z > 450 && two.triangleCount() > one.triangleCount()*1.8, "Projekt: Platzierung wird angewendet (Tiefe %.0f mm)", hi.z-lo.z); }
        moved.scans[1].included = false; Mesh only; fuseProject(moved, 1.f, mop, only, er); { V3 lo, hi; meshBounds(only, lo, hi); CHECK(hi.z-lo.z < 50, "Projekt: abgewaehlter Scan bleibt draussen"); }
        moved.scans[0].included = false; CHECK(!fuseProject(moved, 1.f, mop, only, er), "Projekt: ohne Scan eine Meldung (%s)", er.c_str()); moved.scans[0].included = true; moved.scans[1].included = true;
        moved.scans[1].markers = {{1,2,3},{4,5,6}}; moved.scans[1].up = V3{0,1,0}; moved.scans[1].frames[0].colorJpeg = {1,2,3};
        CHECK(saveProject(moved, "vf_proj.tmp", er), "Projekt speichern");
        Project back; const bool lp = loadProject(back, "vf_proj.tmp", er);
        CHECK(lp && back.scans.size() == 2 && back.scans[1].frames.size() == 6 && back.scans[1].frames[3].z.size() == moved.scans[1].frames[3].z.size() && back.scans[1].placement.t.z == 500 && back.scans[1].markers.size() == 2 && back.scans[1].frames[0].colorJpeg.size() == 3 && back.scans[1].name == "B", "Projekt laden: alles wieder da");
        { FILE* f = fopen("vf_proj.tmp", "r+b"); fseek(f, 0, SEEK_END); long len = ftell(f); fclose(f); if (truncate("vf_proj.tmp", len/2) == 0) { Project bad; bad.scans.push_back(ScanData{}); CHECK(!loadProject(bad, "vf_proj.tmp", er) && bad.scans.size() == 1, "Projekt: abgeschnittene Datei wird abgelehnt, altes Projekt bleibt (%s)", er.c_str()); } remove("vf_proj.tmp"); }
        Project none; CHECK(!loadProject(none, "gibt_es_nicht.vfp", er), "Projekt: fehlende Datei wird gemeldet");
    }
    // 6p. Clean-up by lines of sight, and color from the best views.
    {
        DepthMap grid; grid.width = 320; grid.height = 200; grid.fx = 400; grid.fy = 400; grid.cx = 160; grid.cy = 100;
        auto wallFrame = [&](const Pose& pose, float z, int x0, int x1) { ScanFrame f; f.pose = pose; for (int y = 20; y < 180; y++) for (int x = x0; x < x1; x++) { f.cells.push_back((uint32_t)(y*320+x)); f.z.push_back(z); } return f; };
        // A flat patch at depth d in front of the camera, facing it.
        auto patch = [](Mesh& m, float x0, float x1, float y0, float y1, float d) { const int n = 40; const uint32_t b = (uint32_t)m.vertices.size();
            for (int j = 0; j <= n; j++) for (int i = 0; i <= n; i++) m.vertices.push_back(V3{x0+(x1-x0)*i/n, y0+(y1-y0)*j/n, d});
            for (int j = 0; j < n; j++) for (int i = 0; i < n; i++) { const uint32_t q = b+j*(n+1)+i; m.indices.insert(m.indices.end(), {q, q+n+1, q+1, q+1, q+n+1, q+n+2}); } };
        std::vector<ScanFrame> frames; for (int i = 0; i < 12; i++) { Pose P; P.t.x = (float)i; frames.push_back(wallFrame(P, 300, 20, 300)); }
        ColorCalib noColor; const std::vector<ScanView> view{{&frames, &grid, &noColor, Pose{}}};
        Mesh wall; patch(wall, -40, 40, -40, 40, 300); const size_t wallTris = wall.triangleCount();
        Mesh both = wall; patch(both, -10, 10, -10, 10, 250); computeNormals(both);   // floats in front of the wall, the camera sees through it
        const size_t removed = carveFreeSpace(both, view, 2.f); V3 lo, hi; meshBounds(both, lo, hi);
        CHECK(removed == both.triangleCount()+removed-wallTris && lo.z > 299, "Sichtlinien: durchschautes Stueck entfernt (%zu), Wand bleibt (%zu)", removed, both.triangleCount());
        Mesh keep = wall; computeNormals(keep); CHECK(carveFreeSpace(keep, view, 2.f) == 0, "Sichtlinien: bestaetigte Flaeche bleibt ganz");
        Mesh behind = wall; patch(behind, -10, 10, -10, 10, 340); computeNormals(behind); CHECK(carveFreeSpace(behind, view, 2.f) == 0, "Sichtlinien: verdeckte Flaeche hinter der Wand bleibt, dort sieht niemand hin");
        // Another scan knows that the space is empty.
        std::vector<ScanFrame> other; { Pose P; P.t.z = -100; other.push_back(wallFrame(P, 400, 20, 300)); other.push_back(wallFrame(P, 400, 20, 300)); other.push_back(wallFrame(P, 400, 20, 300)); }
        Mesh stray = wall; patch(stray, -10, 10, -10, 10, 250); computeNormals(stray);
        std::vector<ScanFrame> onlyStray; for (int i = 0; i < 12; i++) onlyStray.push_back(wallFrame(Pose{}, 250, 140, 180));
        const std::vector<ScanView> two{{&onlyStray, &grid, &noColor, Pose{}}, {&frames, &grid, &noColor, Pose{}}};
        const size_t r2 = carveFreeSpace(stray, two, 2.f); CHECK(r2 > 0, "Sichtlinien: entfernt auch, was nur ein Scan gesehen hat (%zu), wenn ein anderer hindurchsieht", r2);
        // Color: a frontal red view beats a grazing blue one, and a view that does not show the point is left out.
        ColorCalib cc; cc.valid = true; cc.focalPx = 800; cc.cx = 640; cc.cy = 400;
        const std::vector<uint8_t> red = solidJpeg(1280, 800, 220, 20, 20), blue = solidJpeg(1280, 800, 20, 20, 220), green = solidJpeg(1280, 800, 20, 220, 20);
        std::vector<ScanFrame> cf; { ScanFrame a = wallFrame(Pose{}, 300, 20, 300); a.colorJpeg = red; cf.push_back(a);
            double tw[6] = {0, 1.1, 0, 0, 0, 0}; Pose turned = expTwist(tw); turned.t = V3{-268, 0, 164}; ScanFrame b; b.pose = turned; b.colorJpeg = blue; for (uint32_t c = 0; c < 64000; c++) { b.cells.push_back(c); b.z.push_back(300); } cf.push_back(b); 
            ScanFrame g = wallFrame(Pose{}, 200, 20, 300); g.colorJpeg = green; cf.push_back(g); }
        const std::vector<ScanView> cview{{&cf, &grid, &cc, Pose{}}};
        Mesh painted; patch(painted, -5, 5, -5, 5, 300); computeNormals(painted);
        { float nz = 0; for (const V3& n : painted.normals) nz += n.z; if (nz > 0) { for (size_t i = 0; i < painted.indices.size(); i += 3) std::swap(painted.indices[i+1], painted.indices[i+2]); computeNormals(painted); } }   // facing the camera, as a scanned surface does
        const float share = colorFromFrames(painted, cview, 2.f);
        CHECK(share > 0.99f && painted.colors.size() == painted.vertices.size()*3, "Farbe: Flaeche eingefaerbt (%.0f %%)", 100*share);
        if (!painted.colors.empty()) CHECK(painted.colors[0] > 3*painted.colors[1] && painted.colors[0] > 3*painted.colors[2], "Farbe: frontale Ansicht zaehlt, verdeckte faellt heraus (%d %d %d)", painted.colors[0], painted.colors[1], painted.colors[2]);
        Mesh dim; patch(dim, -5, 5, -5, 5, 300); computeNormals(dim); { float nz = 0; for (const V3& n : dim.normals) nz += n.z; if (nz > 0) { for (size_t i = 0; i < dim.indices.size(); i += 3) std::swap(dim.indices[i+1], dim.indices[i+2]); computeNormals(dim); } }
        std::vector<ScanFrame> df; { ScanFrame a = wallFrame(Pose{}, 300, 20, 300); a.colorJpeg = solidJpeg(1280, 800, 100, 50, 25); df.push_back(a); } const std::vector<ScanView> dview{{&df, &grid, &cc, Pose{}}};
        colorFromFrames(dim, dview, 2.f); CHECK(dim.colors.size() >= 3 && dim.colors[0] > 200 && std::fabs((float)dim.colors[0]/dim.colors[1]-2.f) < 0.2f, "Farbe: dunkler Raum wird aufgehellt, Farbton bleibt (%d %d %d)", dim.colors.empty() ? 0 : dim.colors[0], dim.colors.empty() ? 0 : dim.colors[1], dim.colors.empty() ? 0 : dim.colors[2]);
        { Mesh dark; patch(dark, -5, 5, -5, 5, 300); computeNormals(dark); { float nz = 0; for (const V3& n : dark.normals) nz += n.z; if (nz > 0) { for (size_t i = 0; i < dark.indices.size(); i += 3) std::swap(dark.indices[i+1], dark.indices[i+2]); computeNormals(dark); } }
          std::vector<ScanFrame> nf; ScanFrame a = wallFrame(Pose{}, 300, 20, 300); a.colorJpeg = solidJpeg(1280, 800, 40, 30, 60); nf.push_back(a); const std::vector<ScanView> nv2{{&nf, &grid, &cc, Pose{}}};
          CHECK(colorFromFrames(dark, nv2, 2.f) == 0 && dark.colors.empty(), "Farbe: im Dunkeln keine erfundene Farbe"); }
        { Mesh unseen; patch(unseen, -5, 5, -5, 5, 900); computeNormals(unseen); CHECK(colorFromFrames(unseen, dview, 2.f) == 0 && unseen.colors.empty(), "Farbe: zeigt kein Bild die Flaeche, bleibt das Netz unberuehrt"); }
        Mesh none; patch(none, -5, 5, -5, 5, 300); computeNormals(none); const std::vector<ScanView> nv{{&frames, &grid, &noColor, Pose{}}}; CHECK(colorFromFrames(none, nv, 2.f) == 0 && none.colors.empty(), "Farbe: ohne Kalibrierung bleibt das Netz ohne Farbe");
    }
    // 6q. Marker tracking with few visible markers and fast turning, as with a large object on a turntable.
    {
        const V3 c{0, 60, 320}, axis = normalized(V3{0, -.82f, -.57f}); const V3 u = normalized(cross(axis, V3{1, 0, 0})), v = cross(axis, u);
        std::vector<V3> ring; for (int i = 0; i < 16; i++) { const float a = i*6.28318f/16+0.09f*std::sin(i*2.4f), r = 95+((i*7)%5)*6.f; ring.push_back(u*(r*std::cos(a))+v*(r*std::sin(a))); }
        struct Run { int ok, lost; float err; size_t map; };
        auto run = [&](float speed, float window) {
            std::mt19937 g(5); std::normal_distribution<float> nd(0, .15f); MarkerTracker t; Pose prior; Run res{0, 0, 0, 0};
            for (int f = 0; f < (int)(720/speed)+40; f++) {
                const float rot = f*speed*.01745f; const double tw[6] = {axis.x*(double)rot, axis.y*(double)rot, axis.z*(double)rot, 0, 0, 0}; const Pose Tw = expTwist(tw);
                std::vector<Marker3D> det;
                for (const V3& m : ring) { const V3 w = Tw.rot(m); const float az = std::atan2(dot(w, v), dot(w, u)); if (std::fabs(az-1.57f) > window || std::fabs(az-1.57f) < .25f) continue; Marker3D d; d.pos = c+w+V3{nd(g), nd(g), nd(g)}; det.push_back(d); }
                const MarkerResult r = t.update(det, prior);
                if (r.ok && r.usable && r.rmsMm <= .8f) {
                    t.learn(); res.ok++; prior = r.camToWorld;
                    const double ti[6] = {-axis.x*(double)rot, -axis.y*(double)rot, -axis.z*(double)rot, 0, 0, 0}; const Pose inv = expTwist(ti); const V3 cam = V3{50, 30, 0}+c;
                    res.err = std::max(res.err, norm(r.camToWorld.apply(cam)-(inv.rot(cam-c)+c)));
                } else res.lost++;
            }
            res.map = t.map().size(); return res;
        };
        const Run normal = run(1, 1.6f); CHECK(normal.lost == 0 && normal.map == 16 && normal.err < 2, "Marker: normaler Drehteller ohne Ausfall (%d Bilder, Karte %zu, Fehler %.2f mm)", normal.ok, normal.map, normal.err);
        const Run fast = run(8, 1.6f); CHECK(fast.lost == 0 && fast.err < 2, "Marker: schnelles Drehen 8 Grad je Bild ohne Ausfall (verloren %d, Fehler %.2f mm)", fast.lost, fast.err);
        const Run faster = run(20, 1.6f); CHECK(faster.lost == 0 && faster.err < 2, "Marker: sehr schnelles Drehen 20 Grad je Bild ohne Ausfall (verloren %d)", faster.lost);
        const Run few = run(1, 1.2f); CHECK(few.ok > 250 && few.err < 5, "Marker: wenige sichtbare Marker, die Karte verhungert nicht (%d von %d Bildern, Fehler %.2f mm)", few.ok, few.ok+few.lost, few.err);
        const Run hidden = run(1, .9f); CHECK(hidden.err < 3, "Marker: bei zu wenigen Markern keine falsche Lage, nur Pause (Fehler %.2f mm)", hidden.err);
        // A row of markers fixes nothing, a wide layout does.
        std::vector<Marker3D> line; for (int i = 0; i < 4; i++) { Marker3D d; d.pos = V3{i*10.f, 0, 300}; line.push_back(d); }
        std::vector<Marker3D> wide = line; wide[1].pos = V3{100, 0, 300}; wide[2].pos = V3{0, 90, 300}; wide[3].pos = V3{100, 90, 300};
        MarkerTracker t0; t0.update(wide, Pose{}); const MarkerResult wr = t0.update(wide, Pose{}); CHECK(wr.ok && wr.usable && wr.matched == 4, "Marker: vier weit verteilte reichen");
        MarkerTracker t1; std::vector<Marker3D> five = line; Marker3D e; e.pos = V3{40, 0, 300}; five.push_back(e); t1.update(five, Pose{}); const MarkerResult lr = t1.update(line, Pose{}); CHECK(lr.ok && !lr.usable, "Marker: vier in einer Reihe reichen nicht");
    }
    // 6r. Fine alignment of frames on an object with a known true shape: it must get closer to the
    // truth and must not shrink or slide. Depth comes straight from the known surface.
    {
        const float R = 40; const V3 C{0, 0, 300};
        // An uneven surface, as radius over direction, so that nothing can turn freely.
        auto radius = [&](V3 dir) { return R+5*std::sin(3*dir.x+1)*std::cos(4*dir.y)+3*std::sin(5*dir.z+2*dir.x); };
        ScanData sd; sd.name = "s"; sd.placed = true; sd.grid.width = 320; sd.grid.height = 200; sd.grid.fx = 400; sd.grid.fy = 400; sd.grid.cx = 160; sd.grid.cy = 100;
        std::mt19937 g(11); std::normal_distribution<float> dn(0, .15f), tn(0, .6f), an(0, .0025f);
        std::vector<Pose> truth;
        for (int f = 0; f < 240; f++) {
            const double ang = f*3*M_PI/180, tilt = 0.3*std::sin(f*0.05);
            const double tw[6] = {tilt, ang, 0, 0, 0, 0}; Pose T = expTwist(tw); T.t = C-T.rot(C);   // camera turns around the object
            truth.push_back(T);
            ScanFrame fr;
            for (int y = 0; y < 200; y += 2) for (int x = 0; x < 320; x += 2) {
                // Ray march to the surface.
                const V3 dir = normalized(V3{(x-160)/400.f, (y-100)/400.f, 1}); const V3 o = T.t, d = T.rot(dir);
                float s0 = 200; bool hit = false; for (int it = 0; it < 60; it++) { const V3 q = o+d*s0-C; const float n = norm(q); const float e = n-radius(q*(1/std::max(n, 1e-3f))); if (s0 > 400) break; if (e < 0.02f) { hit = e > -1.f; break; } s0 += e*0.7f; }
                if (!hit) continue;
                fr.cells.push_back((uint32_t)(y*320+x)); fr.z.push_back(s0*dir.z+dn(g));
            }
            // What the tracker delivers: the true pose with small errors.
            const double err[6] = {an(g), an(g), an(g), tn(g), tn(g), tn(g)}; fr.pose = mul(expTwist(err), T);
            sd.frames.push_back(std::move(fr));
        }
        // A shift shared by all frames cannot be seen from the frames themselves and does not change the shape.
        auto poseErr = [&](const std::vector<Pose>& poses) { std::vector<V3> a, b; const V3 p = C+V3{0, 0, -20}; for (size_t i = 0; i < truth.size(); i++) { a.push_back(poses[i].apply(truth[i].applyInv(p))); b.push_back(p); } Pose common; alignRigid(a, b, common); double e = 0; for (size_t i = 0; i < a.size(); i++) { const V3 d = common.apply(a[i])-b[i]; e += dot(d, d); } return std::sqrt(e/a.size()); };
        std::vector<Pose> before; for (const ScanFrame& f : sd.frames) before.push_back(f.pose);
        std::vector<ScanView> views{{&sd.frames, &sd.grid, &sd.colorCalib, Pose{}}}; std::vector<std::vector<Pose>> polished;
        const float mv = polishPoses(views, polished, 0.5f);
        // Sideways errors on a smooth shape are hard to see and do not blur it, so the shape checks below count most.
        CHECK(poseErr(polished[0]) < 0.95*poseErr(before), "Feinausrichtung: naeher an den wahren Lagen (%.3f statt %.3f mm, zuletzt %.3f mm bewegt)", poseErr(polished[0]), poseErr(before), mv);
        // Shape: distance of the model from the true surface, also signed: does it shrink?
        auto shapeErr = [&](bool polishOn, double& mean) { Project pr; pr.scans.push_back(sd); MeshOptions o; o.upright = false; o.cleanUp = polishOn; o.fillHolesMm = 0; o.removeFragments = false; o.keepTriangles = 1; o.smoothIterations = 0; Mesh m; std::string er; fuseProject(pr, 0.5f, o, m, er); double s = 0, sum = 0; for (const V3& p : m.vertices) { const V3 q = p-C; const float n = norm(q); const float e = n-radius(q*(1/n)); s += e*e; sum += e; } mean = sum/m.vertices.size(); return std::sqrt(s/m.vertices.size()); };
        double m0 = 0, m1 = 0; const double e0 = shapeErr(false, m0), e1 = shapeErr(true, m1);
        CHECK(e1 < 0.9*e0, "Feinausrichtung: Modell naeher an der wahren Form (%.3f statt %.3f mm)", e1, e0);
        CHECK(std::fabs(m1) < 0.1, "Feinausrichtung: Modell schrumpft und waechst nicht (im Mittel %.3f mm)", m1);
    }
    // 6g. Coarse voxels must work on depth with gaps between the projected dots.
    {
        PipelineOptions co; co.voxelMm = 1.f; co.zMax = 620.f; co.useMarkers = false; Pipeline q(co); q.setScanning(true);
        std::mt19937 g(9); DepthMap dd; dd.width = 640; dd.height = 400; dd.fx = dd.fy = 790.9f; dd.cx = 480; dd.cy = 200; dd.z.assign(640*400, 0.f);
        for (int j = 0; j < 400; j++) for (int i = 0; i < 640; i++) {
            const float x = (i-dd.cx)/dd.fx*300, y = (j-dd.cy)/dd.fy*300;
            if (g()%100 < 45 && x*x+y*y < 60*60) dd.z[j*640+i] = 300 - 0.003f*(x*x+y*y);   // a dome, with many gaps
        }
        for (int n = 0; n < 10; n++) q.trackAndIntegrate(dd);
        Mesh cm; MeshOptions cmo; q.buildMesh(cmo, cm);
        CHECK(q.stats().lostFrames == 0 && cm.triangleCount() > 1000, "Grobe Voxel: stabil auf Tiefe mit Luecken (%d verloren, %zu Dreiecke)", q.stats().lostFrames, cm.triangleCount());
    }
    // 7. Measuring tool on boxes with known dimensions.
    {
        BoxReport r = measureBox(makeBox(40, 55, 70, 0, 0.1f));
        auto l = sortedLengths(r);
        CHECK(r.planes.size() == 6 && l.size() == 3 && std::fabs(l[0]-40) < 0.1 && std::fabs(l[1]-55) < 0.1 && std::fabs(l[2]-70) < 0.1, "Messwerkzeug: Quader 40/55/70 erkannt (%zu Flaechen)", r.planes.size());
        CHECK(maxRightAngleError(r) < 0.3, "Messwerkzeug: rechte Winkel erkannt (Fehler %.2f Grad)", maxRightAngleError(r));
        BoxReport s = measureBox(makeBox(40, 55, 70, 0.0524f, 0.1f));   // 3 degree shear
        CHECK(std::fabs(maxRightAngleError(s)-3) < 0.5, "Messwerkzeug: Scherung von 3 Grad wird sichtbar (%.2f Grad)", maxRightAngleError(s));
    }
    // 8. Export round trip.
    std::string err;
    const char* stlPath = "voxelforge_test.stl";
    bool saved = saveMesh(m, stlPath, err);
    CHECK(saved, "STL schreiben %s", err.c_str());
    if (!saved) return 1;
    FILE* fs = fopen(stlPath, "rb"); uint32_t tris = 0; fseek(fs, 80, SEEK_SET); if (fread(&tris, 4, 1, fs) != 1) tris = 0; fseek(fs, 0, SEEK_END); long len = ftell(fs); fclose(fs);
    std::string again = uniquePath("voxelforge_test", ".stl");
    CHECK(again == "voxelforge_test_2.stl", "Vorhandene Datei wird nicht ueberschrieben (%s)", again.c_str());
    remove(stlPath);
    CHECK(tris == m.triangleCount() && len == 84 + 50L*tris, "STL-Struktur korrekt (%u Dreiecke)", tris);
    printf(failures ? "\n%d FEHLGESCHLAGEN\n" : "\nALLE TESTS BESTANDEN\n", failures);
    return failures ? 1 : 0;
}
