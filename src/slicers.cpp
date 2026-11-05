#include "slicers.hpp"
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <png.h>
#include <unistd.h>
#include <sys/wait.h>
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <vector>

namespace fs = std::filesystem;
namespace {
struct Known { const char* name; const char* letter; float color[3]; std::vector<const char*> binaries, flatpaks, icons, appImageKeys; };
const Known kKnown[kSlicerCount] = {
    {"Bambu Studio", "B", {.0f, .68f, .26f}, {"bambu-studio", "BambuStudio", "bambustudio"}, {"com.bambulab.BambuStudio"}, {"BambuStudio", "bambu-studio", "com.bambulab.BambuStudio"}, {"bambu"}},
    {"PrusaSlicer", "P", {.93f, .42f, .13f}, {"prusa-slicer", "PrusaSlicer", "prusaslicer"}, {"com.prusa3d.PrusaSlicer"}, {"PrusaSlicer", "prusa-slicer", "com.prusa3d.PrusaSlicer"}, {"prusa"}},
    {"OrcaSlicer", "O", {.0f, .59f, .53f}, {"orca-slicer", "OrcaSlicer", "orcaslicer"}, {"io.github.softfever.OrcaSlicer", "com.orcaslicer.OrcaSlicer"}, {"OrcaSlicer", "orca-slicer", "io.github.softfever.OrcaSlicer", "com.orcaslicer.OrcaSlicer"}, {"orca"}},
};
std::string home() { const char* h = getenv("HOME"); return h ? h : ""; }
// Inside Flatpak the system lies under /run/host, and programs on it start through flatpak-spawn.
bool sandboxed() { static const bool inside = access("/.flatpak-info", F_OK) == 0; return inside; }
std::string hostPath(const std::string& p) { return sandboxed() ? "/run/host"+p : p; }
std::string onPath(const char* tool) {
    std::string all; if (sandboxed()) all = "/usr/bin:/usr/local/bin:/opt/bin:"+home()+"/.local/bin"; else { const char* p = getenv("PATH"); if (!p) return ""; all = p; }
    size_t a = 0;
    while (a <= all.size()) {
        const size_t b = std::min(all.find(':', a), all.size()); const std::string f = all.substr(a, b-a)+"/"+tool; a = b+1;
        const bool inHome = !home().empty() && f.rfind(home(), 0) == 0;
        if (access((inHome ? f : hostPath(f)).c_str(), X_OK) == 0) return f;
    }
    return "";
}
std::string flatpakRoot(const char* id) {
    for (const std::string& root : {std::string("/var/lib/flatpak/app/"), home()+"/.local/share/flatpak/app/"}) { std::error_code ec; const std::string d = root+id+"/current/active"; if (fs::exists(d, ec)) return d; }
    return "";
}
std::string appImageFor(const char* key) {
    for (const std::string& dir : {home()+"/Applications", home()+"/AppImages", home()+"/.local/bin", home()+"/Downloads"}) {
        std::error_code ec;
        for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
            std::string n = it->path().filename().string(); std::string low = n; std::transform(low.begin(), low.end(), low.begin(), ::tolower);
            if (low.find(key) != std::string::npos && low.find(".appimage") != std::string::npos && access(it->path().c_str(), X_OK) == 0) return it->path().string();
        }
    }
    return "";
}
// The largest picture up to 256 pixels in an icon theme folder. Vector icons cannot be read here.
std::string iconIn(const std::string& hicolor, const char* name) {
    int best = 0; std::string file; std::error_code ec;
    for (fs::directory_iterator it(hicolor, ec), end; !ec && it != end; it.increment(ec)) {
        const int size = atoi(it->path().filename().c_str()); const std::string f = it->path().string()+"/apps/"+name+".png"; std::error_code e;
        if (size > best && size <= 256 && fs::exists(f, e)) { best = size; file = f; }
    }
    return file;
}
unsigned loadPng(const std::string& file) {
    png_image img{}; img.version = PNG_IMAGE_VERSION;
    if (!png_image_begin_read_from_file(&img, file.c_str())) return 0;
    img.format = PNG_FORMAT_RGBA; std::vector<unsigned char> px(PNG_IMAGE_SIZE(img));
    if (!png_image_finish_read(&img, nullptr, px.data(), 0, nullptr)) { png_image_free(&img); return 0; }
    GLuint t = 0; glGenTextures(1, &t); glBindTexture(GL_TEXTURE_2D, t);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, img.width, img.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
    return t;
}
}
void findSlicers(const Settings& s, Slicer out[kSlicerCount]) {
    for (int i = 0; i < kSlicerCount; i++) {
        const Known& k = kKnown[i]; Slicer r; r.name = k.name; r.letter = k.letter; std::copy(k.color, k.color+3, r.color);
        std::string flatRoot;
        if (!s.slicerPath[i].empty()) { if (access(s.slicerPath[i].c_str(), X_OK) == 0 || access(hostPath(s.slicerPath[i]).c_str(), X_OK) == 0) { r.found = true; r.executable = s.slicerPath[i]; r.where = s.slicerPath[i]; } }
        if (!r.found && s.slicerPath[i].empty()) for (const char* b : k.binaries) { const std::string f = onPath(b); if (!f.empty()) { r.found = true; r.executable = f; r.where = f; break; } }
        if (!r.found && s.slicerPath[i].empty()) for (const char* id : k.flatpaks) { flatRoot = flatpakRoot(id); if (!flatRoot.empty()) { r.found = true; r.flatpakId = id; r.where = std::string("Flatpak ")+id; break; } }
        if (!r.found && s.slicerPath[i].empty()) for (const char* key : k.appImageKeys) { const std::string f = appImageFor(key); if (!f.empty()) { r.found = true; r.executable = f; r.where = f; break; } }
        std::vector<std::string> themes; if (!flatRoot.empty()) themes.push_back(flatRoot+"/export/share/icons/hicolor");
        themes.push_back(home()+"/.local/share/icons/hicolor"); themes.push_back(hostPath("/usr/share/icons/hicolor")); themes.push_back("/var/lib/flatpak/exports/share/icons/hicolor");
        if (r.found) for (const std::string& t : themes) { for (const char* n : k.icons) { const std::string f = iconIn(t, n); if (!f.empty()) { r.texture = loadPng(f); if (r.texture) break; } } if (r.texture) break; }
        if (out[i].texture && out[i].texture != r.texture) { GLuint old = out[i].texture; glDeleteTextures(1, &old); }
        out[i] = std::move(r);
    }
}
bool openInSlicer(const Slicer& s, const std::string& file) {
    if (!s.found) return false;
    std::vector<std::string> args;
    // File forwarding lets a sandboxed program read the file, wherever it lies.
    if (!s.flatpakId.empty()) args = {"flatpak", "run", "--file-forwarding", s.flatpakId, "@@", file, "@@"};
    else args = {s.executable, file};
    if (sandboxed()) args.insert(args.begin(), {"flatpak-spawn", "--host"});
    std::vector<char*> argv; for (std::string& a : args) argv.push_back(a.data()); argv.push_back(nullptr);
    const pid_t first = fork(); if (first < 0) return false;
    if (first == 0) {
        // A second fork, so that the slicer belongs to nobody and stays when VoxelForge is closed.
        setsid(); const pid_t second = fork(); if (second != 0) _exit(second < 0 ? 1 : 0);
        execvp(argv[0], argv.data()); _exit(127);
    }
    int status = 0; waitpid(first, &status, 0);
    return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}
