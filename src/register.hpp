#pragma once
// Finds how one scan lies relative to another, without a starting guess: surface features (FPFH),
// random consensus, then fine alignment (ICP). The surfaces only need to overlap in part.
#include <functional>
#include <vector>
#include "mesh.hpp"

struct RegisterResult {
    bool ok = false;
    Pose srcToDst;
    float overlap = 0;     // share of the smaller surface that lies on the other one
    float rmsMm = 0;       // distance in the overlap
    float conflict = 0;    // share of points floating in front of the other surface; high means wrong
    float score = 0;
    float pinned = 0;      // how firmly the shared surface holds the placement in its weakest direction, 0..1. Flat walls alone let it slide.
    bool agreed = false;      // two independent searches both found this placement
    bool confident = false;   // tight, free of conflict and clearly ahead of the others. Otherwise a person should look.
};
// The table a scan stood on, in that scan's coordinates: nothing of the object can lie below it.
struct Table { bool known = false; V3 up; float height = 0; };
// Below this the placement can slide or turn along the shared surface and is never called certain.
constexpr float kMinPinned = 0.05f;
// All plausible placements, best first. Symmetric objects have several, so the user can step through.
std::vector<RegisterResult> registerAuto(const Mesh& src, const Mesh& dst, const std::function<void(float)>& progress = {}, Table srcTable = {}, Table dstTable = {});
// Fine alignment from a rough placement.
RegisterResult refineRegistration(const Mesh& src, const Mesh& dst, const Pose& start, float startDistanceMm, Table srcTable = {}, Table dstTable = {});
// From points a person picked on both scans, src onto dst. Three or more pairs fix the placement, which is then
// fine-aligned. One pair (with the surface directions) or two leave a turn open, which is searched. Best first.
std::vector<RegisterResult> registerFromPairs(const Mesh& src, const Mesh& dst, const std::vector<V3>& srcPoints, const std::vector<V3>& srcNormals,
                                              const std::vector<V3>& dstPoints, const std::vector<V3>& dstNormals, Table srcTable = {}, Table dstTable = {});
// Measures a placement as it is, without moving it.
RegisterResult judgePlacement(const Mesh& src, const Mesh& dst, const Pose& placement, Table srcTable = {}, Table dstTable = {});
// From corresponding points picked by hand (at least 3, not on one line).
bool registerPointPairs(const std::vector<V3>& src, const std::vector<V3>& dst, Pose& out);
// From two marker maps of the same layout. matched: markers that agree. Needs at least 4.
bool registerMarkers(const std::vector<V3>& src, const std::vector<V3>& dst, Pose& out, int& matched, float tolMm = 1.5f);
