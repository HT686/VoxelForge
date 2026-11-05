#pragma once
// A project holds several scans of one object, each with its own frames. Scans are placed relative
// to each other and then fused into one model. Everything is kept, so this can be repeated with
// other values, also after saving and loading.
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "color.hpp"
#include "math.hpp"
#include "mesh.hpp"
#include "stereo.hpp"

// What a second pass needs from one frame. Depth is sparse: only the object.
struct ScanFrame {
    std::vector<uint32_t> cells; std::vector<float> z;
    Pose pose;
    std::vector<uint8_t> colorJpeg;
};
struct ScanData {
    std::string name;
    std::vector<ScanFrame> frames;
    DepthMap grid;                 // size and camera of the depth grid, z stays empty
    ColorCalib colorCalib;
    std::vector<V3> markers;       // marker map in the coordinates of this scan
    V3 up{0, 0, 1};                // what was up while scanning, in the coordinates of this scan
    Pose placement;                // from this scan into the project
    bool placed = false;           // placement is known. The first scan is placed by definition.
    bool included = true;          // part of the fusion
    std::shared_ptr<Mesh> overview;   // quick coarse model for alignment and display
    std::shared_ptr<Mesh> cloud;      // the measured points thinned out, for looking at this scan
    float cloudSpacingMm = 0.25f;
    size_t bytes() const;
};
struct Project {
    std::vector<ScanData> scans;
    size_t bytes() const { size_t b = 0; for (const ScanData& s : scans) b += s.bytes(); return b; }
    std::string nextName() const;
};

// One scan as seen by the clean-up steps below.
// poses: from the frame into the project, one per frame. Empty: placement and frame pose.
struct ScanView {
    const std::vector<ScanFrame>* frames; const DepthMap* grid; const ColorCalib* colorCalib; Pose placement;
    const std::vector<Pose>* poses = nullptr;
    Pose frameToProject(size_t i) const { return poses ? (*poses)[i] : mul(placement, (*frames)[i].pose); }
};
// Fine-aligns every frame to the model, also across scans. Each half of the frames aligns to a model
// made from the other half, so that nothing confirms itself. Fills poses with one list per scan.
// Returns how far frames moved in the last round, in mm on average.
float polishPoses(std::vector<ScanView>& scans, std::vector<std::vector<Pose>>& poses, float voxelMm, int rounds = 3);
// Removes surface that cameras looked through: if a frame sees something behind a point, nothing can
// be there. This catches stray pieces, which seldom repeat in the same place, and things that were
// only there during one scan. Returns triangles removed; the mesh may then hold small leftovers.
size_t carveFreeSpace(Mesh& m, const std::vector<ScanView>& scans, float toleranceMm);
// Colors the finished surface from the frames. Frontal views count most, hidden points are left out.
// Vertices that no frame shows keep their color. Returns the share of vertices colored.
float colorFromFrames(Mesh& m, const std::vector<ScanView>& scans, float toleranceMm);
// The points of a scan in its own coordinates, one per small cube of space, with normals. A cube needs several
// measurements, so single stray points stay out. Keeps a result it already has.
void buildPointCloud(ScanData& scan);
// Quick model of one scan in its own coordinates.
void buildOverview(ScanData& scan);
// Fuses the included scans at their placements. minWeight: how often a voxel must be seen.
// The result stands upright as the first included scan stood. False if there is nothing to fuse.
bool fuseProject(const Project& p, float voxelMm, const MeshOptions& options, Mesh& out, std::string& error, const std::function<void(float)>& progress = {});
// A small picture of a model for the start page, a plain PPM file. up: what is up in the picture.
bool writeThumbnail(const Mesh& m, V3 up, const std::string& path);
// The same from the scans of a project file. Loads the whole project, so this belongs in the background.
bool makeProjectThumbnail(const std::string& projectFile, const std::string& thumbnail);
// From a project in memory, when no finished model is at hand.
bool writeProjectThumbnail(Project& p, const std::string& thumbnail);
bool saveProject(const Project& p, const std::string& path, std::string& error);
bool loadProject(Project& p, const std::string& path, std::string& error);
