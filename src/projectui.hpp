#pragma once
// The project in the scan tab: list of scans, alignment with preview, batch fusion, saving.
#include <atomic>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "editui.hpp"
#include "meshview.hpp"
#include "pipeline.hpp"
#include "project.hpp"
#include "register.hpp"

class ProjectUi {
public:
    ProjectUi(Pipeline& pipe, EditUi& edit);
    ~ProjectUi();
    void init();                     // needs the graphics context
    void frame();                    // once per frame: finished jobs and dialogs
    void panel(bool canAct, int liveFrames, bool canStartTools, bool hasLive);   // list, actions and file, in the scan tab
    bool aligning() const { return aligning_; }
    // Looking at a stored scan instead of the live image. "Neuer Scan" leads back to the live image.
    bool reviewing() const { return reviewScan_ >= 0; }
    void reviewView();
    void alignPanel();               // replaces the scan panel while aligning
    void alignView();                // replaces the live view while aligning
    bool busy() const { return jobRunning_; }
    std::string busyLabel() { std::lock_guard<std::mutex> g(labelMutex_); return jobLabel_; }
    float busyProgress() const { return jobProgress_; }
    bool hasScans() const { return !project_.scans.empty(); }
    // Moves the running scan into the project. Returns false if there is none.
    bool commitLive();
    void openFusion() { fusionRequest_ = true; }
    bool loadFile(const std::string& path);
    // Starts an empty project that lives in this file. Creates the folder and writes the file right away.
    bool createProject(const std::string& file, std::string& error);
    void closeProject();             // forgets everything, nothing is saved
    bool hasProject() const { return *path_ != 0; }
    std::string projectName() const;
    std::string projectFile() const { return path_; }
    bool hasUnsaved() const { return dirty_; }
    void markDirty() { dirty_ = true; }
    void requestSave();              // asks where, then saves
    void requestOpen();              // asks which, then opens
    std::function<void(const std::string& file)> onFileChanged;   // a project was opened, created or saved under this name
    std::string defaultFolder;       // where the dialogs start without a project
    // For the one-click process: places what is not placed yet and fuses. Runs in a worker thread.
    bool buildMerged(float voxelMm, int minWeight, bool keepAllParts, Mesh& out, std::string& error);
    std::string describe() const;
private:
    Pipeline& pipe_; EditUi& edit_; Project project_; MeshView view_, viewB_;
    std::thread job_; std::atomic<bool> jobRunning_{false}, jobDone_{false}; std::function<void()> jobFinish_;
    std::mutex labelMutex_; std::string jobLabel_; std::atomic<float> jobProgress_{-1};
    void startJob(const std::string& label, std::function<void()> work, std::function<void()> finish);
    std::string message;   // last message, for scripted checks
    void say(const std::string& text, bool ok);
    // Alignment.
    bool aligning_ = false, byMarkers_ = false, manual_ = false, hasResult_ = false;
    std::vector<uint8_t> chosen_;
    struct Proposal { std::vector<RegisterResult> options; size_t current = 0; Pose fromMarkers; int markers = 0; };
    std::map<size_t, Proposal> proposals_;   // per scan
    size_t reference_ = 0;
    void openAlign();
    void closeAlign();
    void runAuto();
    void runManual();
    void acceptAlign();
    void showPreview(bool resetCamera);
    void showManualViews();
    Pose proposedPlacement(size_t scan) const;
    std::shared_ptr<Mesh> combined_;
    int fixedScan_ = 0, movingScan_ = 1; std::vector<V3> fixedPoints_, movingPoints_;
    void pickInto(MeshView& view, const Mesh& mesh, float x, float y, std::vector<V3>& points);
    void drawPoints(MeshView& view, const std::vector<V3>& points, ImVec2 origin, ImU32 color);
    void oneView(MeshView& view, ImVec2 size, const char* title, const Mesh* pickMesh, std::vector<V3>* points);
    // Fusion dialog.
    bool fusionRequest_ = false; int fusionVoxel_ = 2, fusionNoise_ = 15; bool fusionKeepParts_ = false, fusionClean_ = true;
    void fusionDialog();
    void startFusion();
    // Files and removing.
    char path_[1024] = ""; bool dirty_ = false; int removeIndex_ = -1; bool removeRequest_ = false;
    void removeDialog();
    void removeScan(size_t i);
    void saveTo(const std::string& file);
    int reviewScan_ = -1; bool reviewDirty_ = false; MeshView reviewView_;
    void reviewLast();
    void showReview(int scan);
    void updateReview();
    void newScan(bool hasLive);
};
