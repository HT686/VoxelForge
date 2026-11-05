#pragma once
// Depth -> tracking -> fusion. Call process() once per stereo pair from one thread.
#include <memory>
#include <mutex>
#include <string>
#include "markers.hpp"
#include "mesh.hpp"
#include "project.hpp"
#include "stereo.hpp"
#include "tsdf.hpp"

// Working distance per object size for the CR-Scan Otter, in mm.
enum class ScanSize { Small, Medium, Big };
// Depth error grows with the square of the distance. best: where the most detail comes out, and a
// small object still fits the picture; outside of it scanning works, only less sharply.
struct ScanProfile { const char* label; float zMin, zMax, bestMin, bestMax; };
inline ScanProfile profileFor(ScanSize s) {
    switch (s) {
    case ScanSize::Small: return {"Klein", 110.f, 300.f, 130.f, 200.f};
    case ScanSize::Big: return {"Groß", 180.f, 1200.f, 200.f, 450.f};
    default: return {"Mittel", 120.f, 400.f, 140.f, 220.f};
    }
}
// How the scanner is held. Upright by hand is the usual grip, turned by 90 degrees: Right means its top leans
// to the right as seen from behind. What is up in the world then lies along the camera's x axis.
enum class ScanRoll { Level = 0, Right = 1, Left = 2 };
inline V3 cameraUp(ScanRoll r) { return r == ScanRoll::Right ? V3{-1, 0, 0} : r == ScanRoll::Left ? V3{1, 0, 0} : V3{0, -1, 0}; }
enum class TrackState { Idle, Waiting, Good, Lost };
struct PipelineStats {
    TrackState state = TrackState::Idle;
    float stereoMs = 0, trackMs = 0, fusionMs = 0, totalMs = 0;
    float validDepth = 0, inlierRatio = 0, rmsMm = 0, medianDepth = 0;
    int integratedFrames = 0, lostFrames = 0, icpPoints = 0, relocations = 0;
    bool baseFound = false;
    int markersMatched = 0, markersInMap = 0; float markerRmsMm = 0; bool markerPose = false;
    size_t blocks = 0, memoryBytes = 0;
    int markersDetected = 0;      // in this frame, known or not
    float turnDegPerFrame = 0;    // smoothed; fast turning loses frames
    bool poseUnconfirmed = false; // the markers suggest a pose, but the surface does not agree yet
};
struct PipelineOptions {
    float voxelMm = 0.5f;
    float zMin = 120.f, zMax = 400.f;   // working distance, see ScanProfile
    float maxPointDist = 8.f;     // ICP association limit in mm
    float minInlierRatio = 0.35f;
    float maxRmsMm = 2.0f;
    float relocMinInliers = 0.7f; // stricter than normal tracking: a wrong match would damage the model
    float relocMaxRmsMm = 1.0f;
    int relocTriesPerFrame = 8;   // poses tried per frame while lost; each costs one tracking run
    bool useMarkers = true;       // align to reflective markers when enough are visible
    bool refineWithShape = true;  // fine-align a marker pose with the surface
    float maxMarkerRmsMm = 0.8f;  // above this the markers are too noisy, use the surface alone
    bool removeBase = true;       // leave the marked surface out of the model
    bool cleanDepth = true;       // drop small islands and depth jumps from each frame, the source of most stray points
    float baseClearanceMm = 0.8f; // how far above the marked surface the object begins
    ScanRoll roll = ScanRoll::Level;
    bool detailDepth = false;     // the model gets depth from a small stereo window, tracking keeps the steadier one
};

// Turntable mode: the scanner stands still and only the object moves. Static surroundings
// would make tracking report "no motion", so only points inside this sphere count. It follows the
// object only with a marker pose. Following a shape-tracked pose would make its errors self-confirming.
struct ObjectRegion { bool enabled = false; bool valid = false; V3 center; float radius = 0; float radiusScale = 1.f; };

class Pipeline {
public:
    explicit Pipeline(const PipelineOptions& o = {});
    PipelineOptions options;
    StereoMatcher stereo;
    void reset();                 // forget the model and start at identity
    void setProfile(const ScanProfile& p) { std::lock_guard<std::mutex> g(mutex_); options.zMin = p.zMin; options.zMax = p.zMax; }
    // Fine voxels only pay off with slope-following stereo windows and sharp depth for the model, which cost time.
    void setVoxelQuality(float voxelMm) { stereo.params.slantRefine = voxelMm <= 0.25f; options.detailDepth = voxelMm <= 0.25f; }
    void setColorCalib(const ColorCalib& c) { std::lock_guard<std::mutex> g(mutex_); colorCalib_ = c; }
    bool hasColorCalib() const { return colorCalib_.valid; }
    void setScanning(bool on) { scanning_ = on; }
    void setTurntable(bool on) { std::lock_guard<std::mutex> g(mutex_); region_.enabled = on; if (!on) region_.valid = false; }
    void setRegionScale(float s) { std::lock_guard<std::mutex> g(mutex_); region_.radiusScale = s; }
    ObjectRegion region() { std::lock_guard<std::mutex> g(mutex_); ObjectRegion r = region_; if (hasMarkerPose_) r.center = markerPose_.applyInv(r.center); return r; }
    bool scanning() const { return scanning_; }
    // Computes depth; when scanning also tracks and integrates.
    void process(const StereoFrame& f, const StereoCalib& calib);
    const DepthMap& depth() const { return depth_; }
    // Depth actually used for the model: outside the object region it is 0.
    const DepthMap& objectDepth() const { return useObjectDepth_ ? filtered_ : depth_; }
    // Next exposure, judged by the dots on the object of the frame just processed. The rest of the picture
    // is mostly black and says little: a near object was far overexposed before it had any effect.
    int nextExposure(const StereoFrame& f, int current) const;
    const Pose& pose() const { return pose_; }
    const std::vector<Marker3D>& detectedMarkers() const { return detected_; }
    const PipelineStats& stats() const { return stats_; }
    // Shaded model view from the current camera, grayscale.
    void renderPreview(int w, int h, std::vector<uint8_t>& gray);
    void buildMesh(const MeshOptions& opt, Mesh& out);
    // Rebuilds the model from the stored frames and their poses, for example at another
    // resolution. Needs no new scan. The poses stay as they were during the scan.
    bool rebuildModel(float voxelMm, std::string& report);
    // Hands the finished scan over and starts empty, ready for the next one. False if there is no
    // usable scan in memory.
    bool takeScan(ScanData& out);
    size_t sessionFrames() const { return session_.size(); }
    size_t sessionBytes() const { return sessionBytes_; }
    bool hasModel() const { return stats_.integratedFrames > 0; }
    // Run tracking+fusion on an existing depth map (used by tests).
    void trackAndIntegrate(const DepthMap& d);
private:
    bool track(const DepthMap& d);
    bool relocalize(const DepthMap& d);
    void rememberKeyframe();
    std::vector<Pose> keyframes_;
    size_t relocCursor_ = 0;
    bool wasLost_ = false;
    std::vector<ScanFrame> session_;
    size_t sessionBytes_ = 0;
    DepthMap sessionGrid_;
    void rememberFrame(const DepthMap& d, const StereoFrame* f);
    const StereoFrame* currentFrame_ = nullptr;
    ColorCalib colorCalib_;
    ColorImage colorImage_;
    bool colorReady_ = false;
    int colorCounter_ = 0;
    MarkerTracker markers_;
    bool markerWeak_ = false;     // few markers or found by distances: only a suggestion until the surface agrees
    int unconfirmed_ = 0;
    bool seesThroughModel(const DepthMap& d, const Pose& pose) const;
    void cleanDepth(DepthMap& d) const;
    bool basePartlySeen_ = false;   // markers reach the image edge: the ring may go on outside of the picture
    DepthMap detail_;
    bool useDetail_ = false;
    Pose lastIntegrated_; bool hasLastIntegrated_ = false;
    std::vector<Marker3D> detected_;
    bool hasMarkerPose_ = false;
    Pose markerPose_;
    void locateObject(const DepthMap& d);
    void applyRegion(const DepthMap& d, DepthMap& out) const;
    void applyBase(const DepthMap& d, const BasePlane& base, const Pose& camToWorld, DepthMap& out) const;
    void updateObjectDepth();
    ObjectRegion region_;
    DepthMap filtered_;
    bool useObjectDepth_ = false;
    BasePlane base_; Pose baseCam_;
    std::unique_ptr<Tsdf> tsdf_;
    DepthMap depth_;
    Pose pose_;
    PipelineStats stats_;
    bool scanning_ = false;
    RayImage ray_;
    float lastFx_ = 0, lastFy_ = 0, lastCx_ = 0, lastCy_ = 0;
    std::mutex mutex_;
};
