#pragma once
// Two-stage stereo for the Otter's rectified IR pair:
//  1. Census matching on half-resolution codes, sampled on a quarter grid, finds a coarse shift.
//  2. Zero-mean NCC around that estimate at full resolution gives the subpixel shift.
// Depth is sampled every params.gridStep pixels (2 = 640x400 for live use, 1 = 1280x800).
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include "device.hpp"

struct DepthMap {
    int width = 0, height = 0;   // grid size; grid (i,j) is image pixel (gridStep*i, gridStep*j)
    float fx = 0, fy = 0, cx = 0, cy = 0; // reference camera on this grid; fx < 0: the IR images are mirrored
    std::vector<float> z;        // depth in mm, 0 = invalid
    float validRatio() const;
};

// Where the time of one frame went, in ms.
struct StereoTimes { float census = 0, coarse = 0, fine = 0, region = 0, slant = 0, detail = 0; };
class GpuStereo;
enum class ComputeMode { Auto, Cpu, Gpu };   // Auto: the graphics card if it is suitable and agrees with the CPU
struct StereoParams {
    int shiftMin = 64, shiftMax = 512; // full-resolution search range; the scan profile sets it from its depth range
    int gridStep = 2;                  // 1 or 2
    int windowRadius = 5;              // NCC window is 2r+1 pixels wide
    bool slantRefine = false;          // second pass with windows warped to the local surface slope
    bool slantLimit = false;           // refine only inside this camera-space sphere (saves time on small objects)
    float slantCx = 0, slantCy = 0, slantCz = 0, slantR = 0;
    float minNcc = 0.6f;
    int minVariance = 16;              // reject windows without texture
    int minRegion = 500;               // reject smaller connected depth patches (cells at gridStep 2)
    int detailRadius = 3;              // window of refineDetail
    float detailMinNcc = 0.8f;
};

class StereoMatcher {
public:
    StereoMatcher();
    ~StereoMatcher();
    StereoParams params;
    StereoTimes times;
    ComputeMode compute_mode = ComputeMode::Cpu;
    bool usingGpu() const { return gpuThisFrame_; }   // the last frame ran on the card
    float gpuTestOffset = 0;                          // for tests: makes the card seem to measure wrongly
    const std::string& gpuInfo() const { return gpuInfo_; }
    // ref and other are width x height, 8 bit. Returns via out.
    void compute(const uint8_t* ref, const uint8_t* other, int width, int height, const StereoCalib& calib, DepthMap& out);
    // Sharper depth for the model: measures again with a small window around the known shift, right
    // after compute() on the same images. Noisier per frame, which fusing many frames evens out, but a
    // large window would blur the surface in every frame alike. Cells without a clear result become 0.
    void refineDetail(const uint8_t* ref, const uint8_t* other, int width, int height, const StereoCalib& calib, DepthMap& depth);
private:
    std::unique_ptr<GpuStereo> gpu_; bool gpuTried_ = false, gpuUsable_ = false, gpuThisFrame_ = false; int gpuCheckIn_ = 0; std::string gpuInfo_ = "Nicht verwendet";
    bool gpuReady();
    void checkGpuAgainstCpu(const uint8_t* ref, const uint8_t* other, int W, int H);
    void matchOnCpu(const uint8_t* ref, const uint8_t* other, int W, int H);
    std::vector<uint8_t> hRef_, hOther_, cost_, aligned_;
    std::vector<uint64_t> cRef_, cOther_;
        std::vector<int16_t> coarse_;
    std::vector<int64_t> iRef_, iRef2_, iOther_, iOther2_;
    std::vector<float> shift_, refined_;
    void refineSlanted(const uint8_t* ref, const uint8_t* other, int W, int H, int gw, int gh, int step, const StereoCalib& calib, int radius, float minNcc, bool detail);
};
