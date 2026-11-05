#include "measure.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

namespace {
// Eigenvector of the smallest eigenvalue of a symmetric 3x3 matrix (Jacobi rotations).
V3 smallestEigenvector(double A[3][3]) {
    double V[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    for (int sweep = 0; sweep < 32; sweep++) {
        for (int p = 0; p < 2; p++) for (int q = p+1; q < 3; q++) {
            if (std::fabs(A[p][q]) < 1e-14) continue;
            double th = 0.5*std::atan2(2*A[p][q], A[q][q]-A[p][p]), c = std::cos(th), s = std::sin(th);
            for (int k = 0; k < 3; k++) { double a = A[k][p], b = A[k][q]; A[k][p] = c*a - s*b; A[k][q] = s*a + c*b; }
            for (int k = 0; k < 3; k++) { double a = A[p][k], b = A[q][k]; A[p][k] = c*a - s*b; A[q][k] = s*a + c*b; }
            for (int k = 0; k < 3; k++) { double a = V[k][p], b = V[k][q]; V[k][p] = c*a - s*b; V[k][q] = s*a + c*b; }
        }
    }
    int m = 0; for (int i = 1; i < 3; i++) if (A[i][i] < A[m][m]) m = i;
    return normalized(V3{(float)V[0][m], (float)V[1][m], (float)V[2][m]});
}
}

BoxReport measureBox(const Mesh& m, double minArea, double tol, int maxPlanes) {
    BoxReport rep;
    const size_t nt = m.triangleCount();
    if (!nt) return rep;
    std::vector<V3> tc(nt), tn(nt); std::vector<float> ta(nt); std::vector<uint8_t> used(nt, 0);
    for (size_t t = 0; t < nt; t++) {
        V3 a = m.vertices[m.indices[3*t]], b = m.vertices[m.indices[3*t+1]], c = m.vertices[m.indices[3*t+2]];
        V3 cr = cross(b-a, c-a); float l = norm(cr);
        tc[t] = (a+b+c)*(1.f/3); tn[t] = l > 1e-12f ? cr*(1.f/l) : V3{}; ta[t] = l*0.5f;
    }
    std::mt19937 rng(42);
    const float cosLimit = std::cos(25.f*3.14159265f/180.f);
    for (int pl = 0; pl < maxPlanes; pl++) {
        std::vector<uint32_t> free; free.reserve(nt);
        for (size_t t = 0; t < nt; t++) if (!used[t] && ta[t] > 0) free.push_back((uint32_t)t);
        if (free.size() < 50) break;
        double bestArea = 0; V3 bestN, bestP;
        for (int it = 0; it < 400; it++) {
            uint32_t s = free[rng()%free.size()];
            V3 n = tn[s], p = tc[s]; double area = 0;
            const size_t step = std::max<size_t>(1, free.size()/20000);
            for (size_t i = 0; i < free.size(); i += step) {
                uint32_t t = free[i];
                if (std::fabs(dot(tc[t]-p, n)) < tol && std::fabs(dot(tn[t], n)) > cosLimit) area += ta[t]*step;
            }
            if (area > bestArea) { bestArea = area; bestN = n; bestP = p; }
        }
        if (bestArea < minArea) break;
        // Refine by least squares, then collect the final members.
        V3 n = bestN, p = bestP; std::vector<uint32_t> members;
        for (int ref = 0; ref < 5; ref++) {
            members.clear();
            double w = 0, cx = 0, cy = 0, cz = 0;
            for (uint32_t t : free) if (std::fabs(dot(tc[t]-p, n)) < tol && std::fabs(dot(tn[t], n)) > cosLimit) {
                members.push_back(t); w += ta[t]; cx += ta[t]*tc[t].x; cy += ta[t]*tc[t].y; cz += ta[t]*tc[t].z;
            }
            if (w <= 0) break;
            V3 c{(float)(cx/w), (float)(cy/w), (float)(cz/w)};
            double A[3][3] = {{0}};
            for (uint32_t t : members) {
                V3 d = tc[t]-c; const double v[3] = {d.x, d.y, d.z};
                for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) A[a][b] += ta[t]*v[a]*v[b];
            }
            n = smallestEigenvector(A); p = c;
        }
        double area = 0, e2 = 0;
        for (uint32_t t : members) { double d = dot(tc[t]-p, n); area += ta[t]; e2 += ta[t]*d*d; used[t] = 1; }
        if (area < minArea) continue;
        FoundPlane f; f.normal = n; f.centroid = p; f.areaMm2 = area; f.flatnessRmsMm = std::sqrt(e2/area); f.triangles = members.size();
        rep.planes.push_back(f);
    }
    std::sort(rep.planes.begin(), rep.planes.end(), [](const FoundPlane& a, const FoundPlane& b) { return a.areaMm2 > b.areaMm2; });
    for (int a = 0; a < (int)rep.planes.size(); a++) for (int b = a+1; b < (int)rep.planes.size(); b++) {
        PlanePair q; q.a = a; q.b = b;
        float cosv = std::min(1.f, std::fabs(dot(rep.planes[a].normal, rep.planes[b].normal)));
        q.angleDeg = std::acos(cosv)*180.0/3.14159265358979;   // 0 = parallel, 90 = perpendicular
        q.parallel = q.angleDeg < 20;
        if (q.parallel) {
            // Symmetric distance using both normals, so a small tilt does not bias the length.
            V3 d = rep.planes[b].centroid - rep.planes[a].centroid;
            q.distanceMm = 0.5*(std::fabs(dot(d, rep.planes[a].normal)) + std::fabs(dot(d, rep.planes[b].normal)));
        }
        rep.pairs.push_back(q);
    }
    return rep;
}

bool loadStl(const std::string& path, Mesh& out, std::string& error) {
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) { error = "Datei nicht lesbar: " + path; return false; }
    char hdr[80]; uint32_t n = 0;
    if (fread(hdr, 1, 80, f) != 80 || fread(&n, 4, 1, f) != 1) { fclose(f); error = "Keine gueltige STL-Datei"; return false; }
    fseek(f, 0, SEEK_END); long len = ftell(f); fseek(f, 84, SEEK_SET);
    if (len != 84 + 50L*(long)n) { fclose(f); error = "Nur binaere STL-Dateien werden unterstuetzt"; return false; }
    out = Mesh{}; out.vertices.reserve((size_t)n*3); out.indices.reserve((size_t)n*3);
    for (uint32_t t = 0; t < n; t++) {
        float rec[12]; uint16_t attr;
        if (fread(rec, 4, 12, f) != 12 || fread(&attr, 2, 1, f) != 1) { fclose(f); error = "STL-Datei ist abgeschnitten"; return false; }
        for (int k = 0; k < 3; k++) { out.indices.push_back((uint32_t)out.vertices.size()); out.vertices.push_back(V3{rec[3+3*k], rec[4+3*k], rec[5+3*k]}); }
    }
    fclose(f);
    return true;
}
