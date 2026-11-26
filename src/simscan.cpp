#include "simscan.hpp"
#include <algorithm>
#include <cmath>
#include <omp.h>

// What a turntable scan sees of a model standing on the table with "up" pointing up: views all around at
// several heights, with occlusion, and nothing within clearMm of the table. Rasterizes along each view.
Mesh scannedPart(const Mesh& m, V3 up, float clearMm) {
    up = normalized(up); const V3 ax = normalized(cross(std::fabs(up.z) < 0.9f ? V3{0, 0, 1} : V3{1, 0, 0}, up)), ay = cross(up, ax);
    float lowest = 1e9f; for (const V3& v : m.vertices) lowest = std::min(lowest, dot(v, up));
    const size_t nt = m.indices.size()/3; std::vector<uint8_t> seen(nt, 0);
    V3 lo, hi; meshBounds(m, lo, hi); const float pix = std::max(0.2f, norm(hi-lo)/700.f);
    std::vector<V3> dirs; for (float el : {15.f, 35.f, 55.f, 75.f}) for (int a = 0; a < 24; a++) { const float e = el*3.14159f/180, az = a*6.28318f/24; dirs.push_back(normalized(ax*(std::cos(e)*std::cos(az))+ay*(std::cos(e)*std::sin(az))+up*std::sin(e))); }
    #pragma omp parallel for schedule(dynamic)
    for (size_t k = 0; k < dirs.size(); k++) {
        const V3 d = dirs[k], u = normalized(cross(std::fabs(d.z) < 0.9f ? V3{0, 0, 1} : V3{1, 0, 0}, d)), w = cross(d, u);
        float u0 = 1e9f, u1 = -1e9f, w0 = 1e9f, w1 = -1e9f; for (const V3& v : m.vertices) { u0 = std::min(u0, dot(v, u)); u1 = std::max(u1, dot(v, u)); w0 = std::min(w0, dot(v, w)); w1 = std::max(w1, dot(v, w)); }
        const int W = (int)((u1-u0)/pix)+2, H = (int)((w1-w0)/pix)+2; std::vector<float> depth((size_t)W*H, -1e30f); std::vector<int32_t> id((size_t)W*H, -1);
        for (size_t t = 0; t < nt; t++) {
            const uint32_t* f = &m.indices[3*t]; V3 p[3]; for (int i = 0; i < 3; i++) p[i] = m.vertices[f[i]];
            if (dot(cross(p[1]-p[0], p[2]-p[0]), d) <= 0) continue;   // faces away
            float x[3], y[3], z[3]; for (int i = 0; i < 3; i++) { x[i] = (dot(p[i], u)-u0)/pix; y[i] = (dot(p[i], w)-w0)/pix; z[i] = dot(p[i], d); }
            const int xa = std::max(0, (int)std::floor(std::min({x[0], x[1], x[2]}))), xb = std::min(W-1, (int)std::ceil(std::max({x[0], x[1], x[2]})));
            const int ya = std::max(0, (int)std::floor(std::min({y[0], y[1], y[2]}))), yb = std::min(H-1, (int)std::ceil(std::max({y[0], y[1], y[2]})));
            const float den = (y[1]-y[2])*(x[0]-x[2])+(x[2]-x[1])*(y[0]-y[2]); if (std::fabs(den) < 1e-12f) continue;
            for (int yy = ya; yy <= yb; yy++) for (int xx = xa; xx <= xb; xx++) {
                const float px = xx+0.5f, py = yy+0.5f;
                const float l0 = ((y[1]-y[2])*(px-x[2])+(x[2]-x[1])*(py-y[2]))/den, l1 = ((y[2]-y[0])*(px-x[2])+(x[0]-x[2])*(py-y[2]))/den, l2 = 1-l0-l1;
                if (l0 < -0.01f || l1 < -0.01f || l2 < -0.01f) continue;
                const float zz = l0*z[0]+l1*z[1]+l2*z[2]; const size_t q = (size_t)yy*W+xx;
                if (zz > depth[q]) { depth[q] = zz; id[q] = (int32_t)t; }
            }
        }
        for (int32_t t : id) if (t >= 0) seen[t] = 1;   // different threads only ever write 1
    }
    Mesh out; std::vector<int64_t> map(m.vertices.size(), -1);
    for (size_t t = 0; t < nt; t++) {
        if (!seen[t]) continue;
        const uint32_t* f = &m.indices[3*t];
        bool low = false; for (int k = 0; k < 3; k++) low |= dot(m.vertices[f[k]], up)-lowest < clearMm; if (low) continue;
        for (int k = 0; k < 3; k++) { if (map[f[k]] < 0) { map[f[k]] = (int64_t)out.vertices.size(); out.vertices.push_back(m.vertices[f[k]]); } out.indices.push_back((uint32_t)map[f[k]]); }
    }
    computeNormals(out); return out;
}

// A box from lo to hi, its faces split into squares of about cell mm so that what is seen can be told apart finely.
void addBox(Mesh& m, V3 lo, V3 hi, float cell) {
    const V3 c[8] = {{lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, hi.y, lo.z}, {lo.x, hi.y, lo.z}, {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}};
    const int faces[6][4] = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {2, 3, 7, 6}, {1, 2, 6, 5}, {3, 0, 4, 7}};   // outward, counter-clockwise
    for (const auto& f : faces) {
        const V3 a = c[f[0]], b = c[f[1]], d = c[f[3]]; const int nu = std::max(1, (int)(norm(b-a)/cell)), nv = std::max(1, (int)(norm(d-a)/cell));
        const uint32_t base = (uint32_t)m.vertices.size();
        for (int j = 0; j <= nv; j++) for (int i = 0; i <= nu; i++) m.vertices.push_back(a+(b-a)*((float)i/nu)+(d-a)*((float)j/nv));
        for (int j = 0; j < nv; j++) for (int i = 0; i < nu; i++) { const uint32_t q = base+j*(nu+1)+i; m.indices.insert(m.indices.end(), {q, q+1, q+(uint32_t)nu+2, q, q+(uint32_t)nu+2, q+(uint32_t)nu+1}); }
    }
}
// Like a toy car track: a thin floor with two thin side walls, a tab at one end and a few ribs below.
Mesh trackModel() {
    Mesh m; const float L = 136, W = 40, T = 1.6f, Hw = 18, c = 1.f;
    addBox(m, {0, 0, 0}, {L, W, T}, c); addBox(m, {0, 0, T}, {L, T, Hw}, c); addBox(m, {0, W-T, T}, {L, W, Hw}, c);
    addBox(m, {L, 10, 0}, {L+14, 30, T}, c);                 // connector tab at one end
    for (float x : {20.f, 60.f, 100.f}) addBox(m, {x, 6, -3}, {x+2, W-6, 0}, c);   // ribs underneath
    addBox(m, {8, 14, T}, {16, 26, T+1.2f}, c);              // a small mark on the floor
    for (V3& v : m.vertices) v.z += 3;                        // the ribs stand on the table
    computeNormals(m); return m;
}
Pose randomPose(std::mt19937& g, float shiftMm) {
    std::normal_distribution<float> n(0, 1); std::uniform_real_distribution<float> u(-shiftMm, shiftMm);
    // A uniformly random rotation from a random unit quaternion.
    float q[4]; float l = 0; for (float& x : q) { x = n(g); l += x*x; } l = std::sqrt(l); for (float& x : q) x /= l;
    const float w = q[0], x = q[1], y = q[2], z = q[3]; Pose P;
    const float R[9] = {1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), 2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), 2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)};
    std::copy(R, R+9, P.R); P.t = {u(g), u(g), u(g)}; return P;
}
Pose inversePose(const Pose& P) { Pose q; for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) q.R[r*3+c] = P.R[c*3+r]; q.t = q.rot(P.t)*-1.f; return q; }
