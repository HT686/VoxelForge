#pragma once
// The "Mesh-Verarbeitung" and "Schnellmessung" part of the window:
// tools with a preview that is confirmed or cancelled, a selection bar under the view, and one click.
#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "meshedit.hpp"
#include "meshview.hpp"

enum class EditTool { None, Simplify, Smooth, Clean, Holes, Align, Cut };
enum class SelectMode { None, Lasso, Box, Object };
class EditUi {
public:
    // Builds a mesh from the current scan. voxelMm <= 0 keeps the resolution.
    struct ScanSource { std::function<void()> prepare; std::function<bool()> available; std::function<float()> voxelMm; std::function<bool(float voxelMm, int minWeight, bool keepAllParts, Mesh& out, std::string& error)> build; };
    EditUi(MeshView& view, ScanSource source);
    ~EditUi();
    void frame();                    // once per frame: finished jobs, keyboard
    void newMesh(Mesh&& m, float voxelMm);   // from a scan or a file
    bool loadFile(const std::string& path);
    bool hasMesh() const { return !base_.indices.empty(); }
    const Mesh& mesh() const { return base_; }
    void clear();                    // back to no model
    // Exports start here, named after the project. Empty: the default folder.
    void setOutputBase(const std::string& pathWithoutExtension, const std::string& folder) { outputBase_ = pathWithoutExtension; outputFolder_ = folder; }
    std::function<void()> printCard; // draws the "3D-Druck" card below the export
    // Writes the model for a slicer next to the project and then calls open with the file.
    void exportForPrint(std::function<void(const std::string& file)> open);
    bool busy() const { return jobRunning_; }
    bool analysing() const { return analysisRunning_; }
    std::string busyLabel() { std::lock_guard<std::mutex> g(progressMutex_); return jobLabel_; }
    float busyProgress() const { return jobProgress_; }
    void panel();                    // left side
    void viewport(bool measure);     // the 3D view with everything on it
    void openOneClick();
    void oneClickDialog();
    std::function<void()> onMeshArrived;   // switch to this tab
    std::function<bool()> colorsExpected;  // a scan with a color camera was made
    std::function<void(const std::string&)> onError;   // the scan tab shows no edit messages
    std::string describe() const;    // state in one line, for scripted checks
private:
    MeshView& view_; ScanSource source_;
    Mesh base_, preview_; bool hasPreview_ = false; size_t firstFilling_ = 0; MeshHistory history_;
    float voxelMm_ = 0.5f; char outPath_[1024] = ""; std::string outputBase_, outputFolder_; void updateOutPath(float voxelMm);
    EditTool tool_ = EditTool::None; SelectMode select_ = SelectMode::None; bool penetrate_ = false;
    // Background work. Tools only read the mesh while it runs.
    std::thread job_; std::atomic<bool> jobRunning_{false}, jobDone_{false}; std::function<void()> jobFinish_;
    std::mutex progressMutex_; std::string jobLabel_; std::atomic<float> jobProgress_{-1};
    void startJob(const std::string& label, std::function<void()> work, std::function<void()> finish);
    void setProgress(const std::string& label, float f);
    // Analysis of the confirmed mesh.
    bool analysisDirty_ = false, analysisRunning_ = false; std::vector<MeshHole> holes_, jobHoles_; size_t openEdges_ = 0;
    double volume_ = 0; V3 lo_, hi_; float tight_[3] = {0, 0, 0}; bool haveSize_ = false;
    void meshChanged(bool resetCamera);
    void showCurrent(bool resetCamera);
    void refreshFlags();
    const Mesh& shown() const { return hasPreview_ ? preview_ : base_; }
    void dropPreview();
    void confirmPreview();
    void runPreview(const std::string& label, std::function<void(Mesh&)> work, size_t firstFilling = 0);
    void exportPanel();
    void toolHeader(const char* title, const char* text = nullptr);
    bool footer(const std::function<void()>& preview, const std::function<void()>& reset);
    // Tools.
    void chooseTool(EditTool t);
    int simplifyPercent_ = 50;
    int smoothLevel_ = 1, smoothIterations_ = 3; bool keepEdges_ = true;
    float cleanShare_ = 5; int cleanSpikes_ = 2, cleanRims_ = 1;
    // Holes.
    bool holeManual_ = false; int fillType_ = 1; float holeLimit_ = 100; std::vector<uint8_t> holeChosen_, holeFilled_; long holeHover_ = -1; bool linesDirty_ = true;
    bool bridging_ = false; bool bridgeHasFirst_ = false; uint32_t bridgeA0_ = 0, bridgeA1_ = 0; V3 bridgeP_;
    void holesPanel(); void holesInput(float mx, float my, bool clicked); void rebuildHoleLines(); void previewFill();
    long nearestRim(float mx, float my, size_t* segment, float limitPx);
    // Selection.
    std::vector<uint8_t> selected_; std::vector<uint32_t> parts_; bool partsValid_ = false, flagsDirty_ = true;
    std::vector<ImVec2> lasso_; bool dragging_ = false; ImVec2 dragStart_{};
    void selectionBar(ImVec2 imageMin, ImVec2 imageSize);
    void finishDragSelect(ImVec2 imageMin, bool remove);
    void selectObject(int x, int y, bool remove);
    void deleteSelected();
    void undo(); void redo();
    size_t selectedCount_ = 0;
    // Cut plane.
    int cutMode_ = 0; bool cutHave_ = false; CutPlane cutBase_; float cutOffset_ = 0, cutTiltA_ = 0, cutTiltB_ = 0; bool cutFlip_ = false; std::vector<V3> cutPoints_; std::vector<uint8_t> cutMarks_; bool cutMarksDirty_ = false;
    CutPlane cutPlane() const; void cutPanel(); void cutInput(float mx, float my); void drawCutPlane(ImVec2 imageMin); void startCut(const CutPlane& p);
    // Alignment.
    float rotX_ = 0, rotY_ = 0, rotZ_ = 0; bool pickFloor_ = false; void alignPanel();
    // Measuring.
    std::vector<V3> measurePoints_; void measurePanel(); void measureInput(float mx, float my); void drawMeasure(ImVec2 imageMin);
    bool pickPoint(float mx, float my, V3& out, long* triangle = nullptr);
    // One click.
    bool oneClickRequest_ = false; OneClickConfig oneClick_; int oneClickVoxel_ = 2, oneClickNoise_ = 15; std::string oneClickReport_; bool useScanForOneClick_ = false;
    void startOneClick();
    std::string configPath() const;
    void gizmo(ImVec2 imageMin, ImVec2 imageSize);
public:
    void measureSidePanel() { measurePanel(); }
};
