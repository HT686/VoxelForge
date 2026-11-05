#pragma once
#include <string>
#include <vector>
#include "math.hpp"
#include "tsdf.hpp"

struct Mesh {
    std::vector<V3> vertices;
    std::vector<uint32_t> indices;  // triangles
    std::vector<V3> normals;
    std::vector<uint8_t> colors;    // 3 bytes per vertex, or empty when the scan has no color
    size_t triangleCount() const { return indices.size() / 3; }
};
struct MeshOptions {
    int minWeight = 2;          // ignore voxels seen fewer times
    bool removeFragments = true;      // drop parts much smaller than the largest one
    float minFragmentShare = 0.05f;   // of the largest part, by triangle count
    int smoothIterations = 4;   // Taubin passes (volume preserving)
    float fillHolesMm = 8.f;    // close holes up to this size; 0 = none, < 0 = all (watertight if possible)
    float keepTriangles = 1.f;  // share of triangles to keep after reduction, 1 = no reduction
    bool cleanUp = true;        // remove what cameras looked through, then color from the best views. Only with stored frames.
    bool upright = false;       // stand the result on its base with Z up, see orientUpright
};
// Surface nets over the observed part of the volume.
void extractMesh(const Tsdf& t, const MeshOptions& opt, Mesh& out);
void computeNormals(Mesh& m);
// Open rims as closed vertex loops, in the direction of the existing triangles.
std::vector<std::vector<uint32_t>> boundaryLoops(const Mesh& m);
// Closes boundary loops whose largest extent is at most maxSizeMm (< 0: all). Returns holes closed.
int fillHoles(Mesh& m, float maxSizeMm);
// Reduces to about targetTriangles with the least change of shape (quadric error metrics).
// Open boundaries keep their outline.
void simplifyMesh(Mesh& m, size_t targetTriangles);
size_t countBoundaryEdges(const Mesh& m);
// Turns the mesh for export: up becomes +Z, which slicers and 3D programs expect, and it is
// centered with its underside at Z = 0. A pure rotation, so left and right stay correct.
void orientUpright(Mesh& m, V3 up);
// Signed volume in mm^3, only meaningful for closed meshes. Bounds via min/max.
double meshVolume(const Mesh& m);
// Sides of the smallest box around the model, however it is turned, largest first.
void tightBoxSize(const Mesh& m, float size[3]);
void meshBounds(const Mesh& m, V3& lo, V3& hi);
bool saveMesh(const Mesh& m, const std::string& path, std::string& error); // .stl .obj .ply
// base + ext if that file does not exist, otherwise base_2 + ext, base_3 + ext, ...
std::string uniquePath(const std::string& base, const std::string& ext);
