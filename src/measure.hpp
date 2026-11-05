#pragma once
// Finds flat faces in a mesh and reports their sizes and angles, for checking scan
// accuracy against a box with known dimensions.
#include <string>
#include <vector>
#include "mesh.hpp"

struct FoundPlane { V3 normal; V3 centroid; double areaMm2 = 0; double flatnessRmsMm = 0; size_t triangles = 0; };
struct PlanePair { int a = 0, b = 0; double angleDeg = 0; double distanceMm = -1; bool parallel = false; };
struct BoxReport { std::vector<FoundPlane> planes; std::vector<PlanePair> pairs; };

// minAreaMm2: ignore smaller faces. tolMm: how far a point may be from a face to count.
BoxReport measureBox(const Mesh& m, double minAreaMm2 = 150.0, double tolMm = 0.8, int maxPlanes = 8);
bool loadStl(const std::string& path, Mesh& out, std::string& error);
