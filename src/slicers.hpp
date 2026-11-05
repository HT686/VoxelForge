#pragma once
// The 3D printing slicers: where they are installed and how a model is opened in them. Their icons come from
// the installed program, the repository holds no files of other makers.
#include <string>
#include "settings.hpp"

struct Slicer {
    const char* name = ""; const char* letter = ""; float color[3] = {0, 0, 0};
    bool found = false;
    std::string executable, flatpakId;   // one of the two
    std::string where;                   // for the settings page
    unsigned texture = 0;                // the program's icon, 0 if there is none we can read
};
constexpr int kSlicerCount = 3;
// Looks everything up again. An own path from the settings wins. Needs the graphics context for the icons.
void findSlicers(const Settings& s, Slicer out[kSlicerCount]);
// Starts the slicer with the file and returns at once. False if that was impossible.
bool openInSlicer(const Slicer& s, const std::string& file);
