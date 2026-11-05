#pragma once
// Editing a finished mesh: holes, smoothing, clean-up, selection and undo. No user interface here,
// so that every operation can be tested on shapes with a known answer.
#include <array>
#include <functional>
#include <string>
#include <vector>
#include "mesh.hpp"

// ---- Clean connections -------------------------------------------------------------------------
// Afterwards every edge has at most two triangles and every vertex one fan, so all open rims form
// closed loops and a filled mesh is truly watertight. Where sheets of the surface only touch, they
// are separated: at an edge with more than two triangles those facing each other across the inside
// of the object stay connected. Returns the number of vertices added.
size_t repairManifold(Mesh& m);
// Edges with more than two triangles.
size_t countKnotEdges(const Mesh& m);

// ---- Holes -----------------------------------------------------------------------------------
// How the new surface meets the old one. These are fairing orders:
// the surface minimizes the k-th power of the Laplacian, which fixes its position (k=1), also its
// tangent plane (k=2) and also its curvature (k=3) along the rim.
enum class FillType { Planar, Tangent, Curvature };
struct MeshHole {
    std::vector<uint32_t> loop;  // walks in the direction of the existing triangles
    float perimeterMm = 0;
    float sizeMm = 0;            // largest extent
    V3 center;
};
// Sorted by perimeter, smallest first.
std::vector<MeshHole> findHoles(const Mesh& m);
// Closes the given holes. New triangles are appended, so all triangles from the old count on are
// the filling. Normals and colors stay valid. Returns the number closed.
int fillHoleList(Mesh& m, const std::vector<MeshHole>& holes, FillType type);
// Closes every hole with a perimeter up to maxPerimeterMm (< 0: all).
// Closes holes up to this perimeter, < 0: all. Narrow slits close whatever their length.
int fillHolesUpTo(Mesh& m, float maxPerimeterMm, FillType type);
// Connects two rim edges with a strip. Both must be boundary edges given in loop direction
// (a0 -> a1, b0 -> b1). On one hole this splits it in two, on two holes it joins them.
bool bridgeEdges(Mesh& m, uint32_t a0, uint32_t a1, uint32_t b0, uint32_t b1);

// ---- Surface ---------------------------------------------------------------------------------
enum class Level { None, Low, Medium, High };
struct SmoothOptions {
    int iterations = 4;          // Taubin passes, keeps the volume
    bool keepEdges = false;      // smooth less across sharp edges
};
inline int smoothIterationsFor(Level l) { return l == Level::Low ? 3 : l == Level::Medium ? 8 : l == Level::High ? 20 : 0; }
// Open rims stay in place.
void smoothSurface(Mesh& m, const SmoothOptions& o);
// Smooths only the outline of open rims.
void smoothRims(Mesh& m, Level level);
// Pulls in vertices that stick out far more than any real corner. Returns vertices moved.
size_t removeSpikes(Mesh& m, Level level);
// Removes parts smaller than this share of the largest part. Returns parts removed.
int removeSmallParts(Mesh& m, float minShare);
// Turns about the center of the bounding box; degrees about the model axes.
void rotateMesh(Mesh& m, float degX, float degY, float degZ);

// ---- Selection (one byte per triangle) ---------------------------------------------------------
// Connected part per triangle.
void triangleParts(const Mesh& m, std::vector<uint32_t>& partOfTriangle);
// Removes marked triangles and vertices no longer used. Returns triangles removed.
size_t deleteTriangles(Mesh& m, const std::vector<uint8_t>& marked);
struct CutPlane { V3 normal{0, 0, 1}; float d = 0; float side(V3 p) const { return dot(normal, p) - d; } };
// Marks triangles whose center lies below the plane.
void markBelowPlane(const Mesh& m, const CutPlane& plane, std::vector<uint8_t>& marked);
bool planeFromPoints(V3 a, V3 b, V3 c, CutPlane& out);
// The flat surface with the largest area, found by random sampling.
bool largestPlane(const Mesh& m, CutPlane& out, float tolMm = 0.5f);

// ---- History ---------------------------------------------------------------------------------
class MeshHistory {
public:
    void clear() { undo_.clear(); redo_.clear(); }
    // Call with the state before a change.
    void push(const Mesh& before);
    bool canUndo() const { return !undo_.empty(); }
    bool canRedo() const { return !redo_.empty(); }
    bool undo(Mesh& current);
    bool redo(Mesh& current);
    size_t maxBytes = size_t(3) << 30;
private:
    std::vector<Mesh> undo_, redo_;
    size_t bytes() const;
};

// ---- One click ---------------------------------------------------------------------------------
struct OneClickConfig {
    float minPartShare = 0.05f;
    Level spikes = Level::Medium;
    Level smooth = Level::Low;
    bool keepEdges = true;
    Level rimSmooth = Level::Low;
    bool fillHoles = true;
    FillType fillType = FillType::Tangent;
    float maxHolePerimeterMm = 100.f;   // the opening of a scan without an underside stays, unless raised
    float keepTriangles = 1.f;          // share kept
};
struct OneClickReport { int partsRemoved = 0; size_t spikesMoved = 0; int holesClosed = 0; int holesLeft = 0; size_t trianglesBefore = 0, trianglesAfter = 0; };
// The same steps the tools offer by hand, in a fixed order. progress gets a step name and 0..1.
OneClickReport runOneClick(Mesh& m, const OneClickConfig& c, const std::function<void(const char*, float)>& progress = {});
bool saveOneClickConfig(const OneClickConfig& c, const std::string& path);
bool loadOneClickConfig(const std::string& path, OneClickConfig& c);

// ---- Files -------------------------------------------------------------------------------------
// Binary STL, OBJ and the PLY files this program writes. Merges equal points, so holes can be found.
bool loadMeshFile(const std::string& path, Mesh& out, std::string& error);
void weldVertices(Mesh& m);
