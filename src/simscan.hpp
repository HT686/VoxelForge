#pragma once
// Simulated scans for checking alignment without a scanner: simple models and what a turntable scan sees of them.
#include <random>
#include "mesh.hpp"

// What a turntable scan sees of a model standing on the table with "up" pointing up: views all around at
// several heights, with occlusion, and nothing within clearMm of the table.
Mesh scannedPart(const Mesh& m, V3 up, float clearMm);
// A box from lo to hi, its faces split into squares of about cell mm.
void addBox(Mesh& m, V3 lo, V3 hi, float cell);
// Like a toy car track: a thin floor with two thin side walls, a tab at one end and a few ribs below. Z up, on the table at 0.
Mesh trackModel();
// A uniformly random rotation and a shift of up to shiftMm along each axis.
Pose randomPose(std::mt19937& g, float shiftMm);
Pose inversePose(const Pose& P);
