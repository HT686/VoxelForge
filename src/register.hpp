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
    bool confident = false;   // tight, free of conflict and clearly ahead of the others. Otherwise a person should look.
};
// All plausible placements, best first. Symmetric objects have several, so the user can step through.
std::vector<RegisterResult> registerAuto(const Mesh& src, const Mesh& dst, const std::function<void(float)>& progress = {});
// Fine alignment from a rough placement.
RegisterResult refineRegistration(const Mesh& src, const Mesh& dst, const Pose& start, float startDistanceMm);
// From corresponding points picked by hand (at least 3, not on one line).
bool registerPointPairs(const std::vector<V3>& src, const std::vector<V3>& dst, Pose& out);
// From two marker maps of the same layout. matched: markers that agree. Needs at least 4.
bool registerMarkers(const std::vector<V3>& src, const std::vector<V3>& dst, Pose& out, int& matched, float tolMm = 1.5f);
