#pragma once
// Reflective marker tracking. Markers show up as large bright blobs in the "other" image and as
// clusters of bright dots at the same height in the reference image. Their 3D positions form a
// map; each frame is aligned to that map. This gives an absolute pose that does not depend on
// the object's shape and does not accumulate drift.
#include <cstdint>
#include <vector>
#include "device.hpp"
#include "math.hpp"

struct Marker3D { V3 pos; float u = 0, v = 0, radiusPx = 0; };
struct MapMarker { V3 pos; int seen = 0; };
struct MarkerResult {
    bool ok = false; int matched = 0, detected = 0; float rmsMm = 0; Pose camToWorld;
    bool relocated = false;   // found from marker distances alone. Rings look alike under rotation, so this needs checking.
    bool usable = false;      // enough well spread markers fit tightly enough to trust this pose
};

// The flat surface the markers lie on, usually a turntable. normal points toward the camera side.
struct BasePlane {
    bool valid = false;
    V3 normal, center;     // center: marker centroid on the plane
    float radius = 0;      // distance of the farthest marker from the center
    float height(V3 p) const { return dot(normal, p-center); }
    float radial(V3 p) const { V3 d = p-center; return norm(d - normal*dot(normal, d)); }
};
// Fits the plane robustly. Needs at least 5 points; viewpoint decides the direction of the normal.
BasePlane fitBasePlane(const std::vector<V3>& points, V3 viewpoint);

// Finds markers in one stereo pair, in reference-camera coordinates (mm).
void detectMarkers(const StereoFrame& f, const StereoCalib& c, int shiftMin, int shiftMax, std::vector<Marker3D>& out);
// Best rigid transform taking a[i] to b[i] (Horn's closed form). Needs at least 3 points.
bool alignRigid(const std::vector<V3>& a, const std::vector<V3>& b, Pose& out);

class MarkerTracker {
public:
    void reset() { map_.clear(); hasMotion_ = false; }
    // prior: last known camera pose. Returns ok when enough markers match the map.
    MarkerResult update(const std::vector<Marker3D>& detected, const Pose& prior);
    // Learns from the last update: refines the map and adds new markers. Call once the pose is
    // accepted. A wrong pose would damage the map for the rest of the scan.
    void learn();
    const std::vector<MapMarker>& map() const { return map_; }
    BasePlane basePlane(V3 viewpoint) const;   // in world coordinates
    int minMatches = 4;
    float matchRadiusMm = 6.f;
    float wideRadiusMm = 16.f;      // second try for fast turning; well below the usual marker spacing
private:
    // Finds the pose from marker-to-marker distances alone, when the prior pose is no help.
    bool relocalize(const std::vector<Marker3D>& detected, Pose& out) const;
    std::vector<MapMarker> map_;
    Pose lastPose_, motion_; bool hasLast_ = false, hasMotion_ = false;   // for predicting fast turns
    std::vector<Marker3D> lastDet_; std::vector<int> lastAssign_; Pose lastT_; int lastN_ = 0; float lastRms_ = 0; bool lastSpread_ = false, canLearn_ = false;
};
