#pragma once
// What the program remembers between runs: where projects go, where the slicers are, which projects were
// used last. A plain text file, so that it can be read and repaired by hand.
#include <cstdint>
#include <string>
#include <vector>

struct Settings {
    std::string projectDir;                  // new projects get a folder of their own in here
    std::string slicerPath[3];               // Bambu Studio, PrusaSlicer, OrcaSlicer; empty: find it
    int computeMode = 0;                     // 0 automatic, 1 CPU, 2 graphics card, see ComputeMode
    int scanRoll = 0;                        // how the scanner is held, see ScanRoll in pipeline.hpp
    std::vector<std::string> recent, hidden; // newest first; hidden: taken off the start page
};
std::string settingsFile();                  // in the user's config folder
std::string defaultProjectDir();             // Documents/VoxelForge
Settings loadSettings(const std::string& file);
bool saveSettings(const Settings& s, const std::string& file);

// A name usable as a folder and file: no path separators or control characters, no dots or spaces at the
// ends. Empty if nothing usable is left.
std::string sanitizeProjectName(const std::string& name);
// Where a new project goes: <folder>/<name>/<name>.vfproj
std::string newProjectFile(const std::string& folder, const std::string& name);
void rememberProject(Settings& s, const std::string& file);

struct ProjectEntry { std::string file, name; int64_t modified = 0; uint64_t bytes = 0; };
// Recent projects and those found in the project folder and its subfolders, newest first. Files gone are left out.
std::vector<ProjectEntry> listProjects(const Settings& s);
std::string thumbnailFile(const std::string& projectFile);   // next to the project
