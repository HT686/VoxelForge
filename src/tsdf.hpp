#pragma once
// Sparse truncated signed distance volume, stored in 8x8x8 voxel blocks.
// Not thread safe across calls: the owner serializes integrate/raycast/extract.
#include <cstdint>
#include <deque>
#include <vector>
#include "color.hpp"
#include "math.hpp"
#include "stereo.hpp"

// weight counts in steps of 1/kWeightScale frames, so that a frame can count partly.
constexpr int kWeightScale = 8;
struct Voxel { float tsdf = 1.f; uint16_t weight = 0; uint8_t r = 0, g = 0, b = 0, colorWeight = 0, seen = 0; };   // seen: frames, up to 255
constexpr int kBlockSide = 8;
struct Block { Voxel v[kBlockSide*kBlockSide*kBlockSide]; int x, y, z; };

struct RayImage {
    int width = 0, height = 0;
    std::vector<V3> vertex, normal;  // world coordinates; normal.z==0 && x==0 && y==0 means no hit
};

class Tsdf {
public:
    explicit Tsdf(float voxelMm = 0.5f, float truncMm = 0.f);
    float voxel() const { return voxel_; }
    float trunc() const { return trunc_; }
    size_t blockCount() const { return blocks_.size(); }
    size_t memoryBytes() const { return blocks_.size() * sizeof(Block); }
    void clear();
    // A frame counts less at its edges and where it sees the surface at a slant: stereo errs most there,
    // and overlapping frames blend instead of leaving a step where one ends.
    bool feather = true;
    void integrate(const DepthMap& d, const Pose& camToWorld, float maxWeight = 64,
                   const ColorImage* color = nullptr, const ColorCalib* colorCalib = nullptr);
    void raycast(const Pose& camToWorld, int w, int h, float fx, float fy, float cx, float cy, float zMin, float zMax, RayImage& out) const;

    Block* find(int bx, int by, int bz) const;
    const std::deque<Block>& blocks() const { return blocks_; }
private:
    struct Slot { int64_t key; int32_t index; };
    static int64_t packKey(int x, int y, int z) { return ((int64_t)(x + (1<<20)) << 42) | ((int64_t)(y + (1<<20)) << 21) | (int64_t)(z + (1<<20)); }
    Block* obtain(int bx, int by, int bz);
    void grow();
    float voxel_, trunc_;
    V3 boundsLo_{1e30f, 1e30f, 1e30f}, boundsHi_{-1e30f, -1e30f, -1e30f};   // around all blocks
    std::deque<Block> blocks_;
    std::vector<Block*> blockPtr_;
    std::vector<Slot> table_;
    size_t mask_ = 0;
    std::vector<int64_t> touched_;
    std::vector<float> pixelWeight_;
    std::vector<float> dist_; std::vector<int> integral_;
    void computePixelWeights(const DepthMap& d, int x0, int y0, int x1, int y1);
};
