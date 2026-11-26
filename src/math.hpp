#pragma once
// Small float math helpers: 3-vectors, rigid poses, a 6x6 solver for ICP.
#include <cmath>
#include <cstring>
struct V3 { float x = 0, y = 0, z = 0; };
inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float norm(V3 a) { return std::sqrt(dot(a, a)); }
inline V3 normalized(V3 a) { float n = norm(a); return n > 1e-12f ? a * (1.f / n) : V3{0, 0, 0}; }

// Rigid transform p' = R p + t. R is row-major.
struct Pose {
    float R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    V3 t;
    V3 rot(V3 p) const { return {R[0]*p.x + R[1]*p.y + R[2]*p.z, R[3]*p.x + R[4]*p.y + R[5]*p.z, R[6]*p.x + R[7]*p.y + R[8]*p.z}; }
    V3 rotInv(V3 p) const { return {R[0]*p.x + R[3]*p.y + R[6]*p.z, R[1]*p.x + R[4]*p.y + R[7]*p.z, R[2]*p.x + R[5]*p.y + R[8]*p.z}; }
    V3 apply(V3 p) const { return rot(p) + t; }
    V3 applyInv(V3 p) const { return rotInv(p - t); }
};
inline Pose mul(const Pose& a, const Pose& b) {
    Pose c;
    for (int r = 0; r < 3; r++)
        for (int k = 0; k < 3; k++)
            c.R[r*3+k] = a.R[r*3]*b.R[k] + a.R[r*3+1]*b.R[3+k] + a.R[r*3+2]*b.R[6+k];
    c.t = a.apply(b.t);
    return c;
}
// Pose from a small twist (wx, wy, wz, tx, ty, tz); rotation by Rodrigues.
inline Pose expTwist(const double x[6]) {
    Pose p; double a = std::sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]);
    double c = std::cos(a), s = std::sin(a), k = 1 - c;
    double ux = 0, uy = 0, uz = 0;
    if (a > 1e-12) { ux = x[0]/a; uy = x[1]/a; uz = x[2]/a; }
    p.R[0] = c + ux*ux*k;    p.R[1] = ux*uy*k - uz*s; p.R[2] = ux*uz*k + uy*s;
    p.R[3] = uy*ux*k + uz*s; p.R[4] = c + uy*uy*k;    p.R[5] = uy*uz*k - ux*s;
    p.R[6] = uz*ux*k - uy*s; p.R[7] = uz*uy*k + ux*s; p.R[8] = c + uz*uz*k;
    p.t = {(float)x[3], (float)x[4], (float)x[5]};
    return p;
}
// View rotation for a Z-up model: turn about Z (yaw), then tilt toward the viewer (pitch; 0 looks
// straight along the ground, positive looks from above). Result is row-major and x right, y up,
// z toward the viewer. It must stay a pure rotation, otherwise the model appears mirrored.
inline void viewRotationZUp(float yaw, float pitch, float R[9]) {
    const float cy = std::cos(yaw), sy = std::sin(yaw), cp = std::cos(pitch), sp = std::sin(pitch);
    const float out[9] = {cy, -sy, 0, sp*sy, sp*cy, cp, -cp*sy, -cp*cy, sp};
    std::memcpy(R, out, sizeof out);
}
// Solves A x = b for symmetric positive definite A (Cholesky). False if singular.
inline bool solve6(const double A[36], const double b[6], double x[6]) {
    double L[36] = {0};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j <= i; j++) {
            double s = A[i*6+j];
            for (int k = 0; k < j; k++) s -= L[i*6+k]*L[j*6+k];
            if (i == j) { if (s <= 1e-12) return false; L[i*6+i] = std::sqrt(s); }
            else L[i*6+j] = s / L[j*6+j];
        }
    double y[6];
    for (int i = 0; i < 6; i++) { double s = b[i]; for (int k = 0; k < i; k++) s -= L[i*6+k]*y[k]; y[i] = s / L[i*6+i]; }
    for (int i = 5; i >= 0; i--) { double s = y[i]; for (int k = i+1; k < 6; k++) s -= L[k*6+i]*x[k]; x[i] = s / L[i*6+i]; }
    return true;
}
// Eigenvalues and eigenvectors of a symmetric 6x6 matrix by Jacobi rotations. vecs: column k belongs to vals[k].
inline void eigenSym6(const double Ain[36], double vals[6], double vecs[36]) {
    double A[36]; for (int i = 0; i < 36; i++) A[i] = Ain[i];
    for (int i = 0; i < 36; i++) vecs[i] = (i%7 == 0) ? 1 : 0;
    for (int sweep = 0; sweep < 40; sweep++) {
        double off = 0; for (int i = 0; i < 6; i++) for (int j = i+1; j < 6; j++) off += A[i*6+j]*A[i*6+j];
        if (off < 1e-24) break;
        for (int p = 0; p < 6; p++) for (int q = p+1; q < 6; q++) {
            if (std::fabs(A[p*6+q]) < 1e-30) continue;
            const double th = (A[q*6+q]-A[p*6+p])/(2*A[p*6+q]), t = (th >= 0 ? 1 : -1)/(std::fabs(th)+std::sqrt(th*th+1)), c = 1/std::sqrt(t*t+1), sn = t*c;
            for (int k = 0; k < 6; k++) { const double x = A[k*6+p], y = A[k*6+q]; A[k*6+p] = c*x-sn*y; A[k*6+q] = sn*x+c*y; }
            for (int k = 0; k < 6; k++) { const double x = A[p*6+k], y = A[q*6+k]; A[p*6+k] = c*x-sn*y; A[q*6+k] = sn*x+c*y; }
            for (int k = 0; k < 6; k++) { const double x = vecs[k*6+p], y = vecs[k*6+q]; vecs[k*6+p] = c*x-sn*y; vecs[k*6+q] = sn*x+c*y; }
        }
    }
    for (int i = 0; i < 6; i++) vals[i] = A[i*6+i];
}
