#pragma once
// The calibration board: a grid of dark dots at a known pitch. In the IR image the dots show as dark spots,
// also under the projector's pattern once blurred. Their centers, carried onto the board's plane from the depth
// image, give lengths that depend on the stereo scale alone: X = (u - cx) * baseline / disparity, so a spacing
// measured too long means the depth is too far, at that distance.
#include <vector>
#include "device.hpp"
#include "math.hpp"
#include "stereo.hpp"

struct BoardMeasurement {
    bool ok = false;
    int dots = 0, pairs = 0;     // dots found, neighbour pairs measured
    float distanceMm = 0;        // of the board's middle from the camera
    float tiltDeg = 0;           // of the board against the view
    float spacingMm = 0;         // median distance of neighbouring dots
    float spacingSpread = 0;     // their spread, mm
    float flatnessMm = 0;        // depth points around the plane, rms
    float shift = 0;             // stereo shift x_ref - x_other at the board's middle, pixels
    std::vector<V3> centers;     // dot centers in camera coordinates
};
// Without the projector: the dots found in both images, paired by row, and their 3D centers from the stereo
// formula alone. Tests the factory calibration itself, independent of the dense depth.
BoardMeasurement measureBoardStereo(const StereoFrame& f, const StereoCalib& calib, float pitchMm);
// d: the depth of this frame, computed from f. pitchMm: what neighbouring dots should be apart.
BoardMeasurement measureBoard(const StereoFrame& f, const DepthMap& d, const StereoCalib& calib, float pitchMm);
// From boards at several distances: the correction of the stereo scale. True depth = f * b * scale / (D + offset),
// with D the disparity as measured. Fitted so that the dot spacing becomes the pitch everywhere. False with too few.
bool fitStereoCorrection(const std::vector<BoardMeasurement>& m, const StereoCalib& calib, float pitchMm, float& scale, float& offset, float& residualPct);
