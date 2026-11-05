#pragma once
// The start page with the last projects, the dialog for a new project and the settings.
#include <imgui.h>
#include <atomic>
#include <functional>
#include <map>
#include <string>
#include <thread>
#include <vector>
#include "settings.hpp"
#include "slicers.hpp"

class HomeUi {
public:
    explicit HomeUi(Settings& settings);
    ~HomeUi();
    void init();                  // needs the graphics context
    void refresh();               // reads the list of projects again
    void page();                  // fills the current window
    void dialogs();               // once per frame
    void requestNew();            // asks for name and place
    void requestSettings() { settingsRequest_ = true; }
    void guard(std::function<void()> action);   // does it, after asking if work would be lost
    const Slicer* slicers() const { return slicers_; }
    void save();
    std::function<void(const std::string&)> onNewProject, onOpenProject, onOpenModel;
    std::function<void()> onResume;
    std::function<std::string()> currentName;   // the open project, empty if none
    std::function<bool()> hasUnsavedWork;
    std::function<void(int)> onComputeMode;
    std::function<std::string()> computeInfo;   // what the matcher reports right now
private:
    Settings& settings_; std::vector<ProjectEntry> entries_; Slicer slicers_[kSlicerCount];
    struct Thumb { unsigned texture = 0; int64_t stamp = 0; };
    std::map<std::string, Thumb> thumbs_;
    unsigned thumbFor(const std::string& project);
    // Pictures of older projects are made in the background, one at a time.
    std::thread worker_; std::atomic<bool> workerBusy_{false}; std::vector<std::string> failed_;
    void makeMissingThumbs();
    bool newRequest_ = false, settingsRequest_ = false, confirmRequest_ = false;
    char newName_[128] = ""; std::string newFolder_; std::function<void()> pending_;
    void newDialog(); void settingsDialog(); void confirmDialog(); void projectCard(const ProjectEntry& e, ImVec2 size);
};
