#pragma once
// The stereo matcher on the graphics card, as compute shaders. It does exactly what the CPU code in stereo.cpp
// does, and the CPU code stays: it is the reference, and what runs wherever there is no suitable card.
#include <memory>
#include <string>
#include <vector>
#include "stereo.hpp"

class GpuStereo {
public:
    GpuStereo();
    ~GpuStereo();
    // Creates a context of its own on the calling thread. False: no suitable card, see info().
    bool init();
    const std::string& info() const;    // name of the card, or why it cannot be used
    // Coarse and fine stage: shift gets one value per grid cell, 0 where nothing was found. False: the card failed.
    bool match(const uint8_t* ref, const uint8_t* other, int W, int H, const StereoParams& params, std::vector<float>& shift);
    // Slope-following windows around the shifts in shift, on the images of the last match.
    bool refine(int W, int H, int gw, int gh, int step, const StereoCalib& calib, const StereoParams& params, int radius, float minNcc, bool detail, std::vector<float>& shift);
private:
    struct Impl; std::unique_ptr<Impl> impl_;
};
