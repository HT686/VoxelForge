#include "settings.hpp"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <set>

namespace fs = std::filesystem;
static std::string homeDir() { const char* h = getenv("HOME"); return h ? h : "."; }
std::string settingsFile() {
    const char* x = getenv("XDG_CONFIG_HOME");
    return (x && *x ? std::string(x) : homeDir()+"/.config")+"/voxelforge/settings.ini";
}
std::string defaultProjectDir() {
    std::string base = homeDir();
    for (const char* d : {"/Dokumente", "/Documents"}) if (fs::is_directory(base+d)) { base += d; break; }
    return base+"/VoxelForge";
}
Settings loadSettings(const std::string& file) {
    Settings s; s.projectDir = defaultProjectDir();
    std::ifstream in(file); std::string line;
    while (std::getline(in, line)) {
        const size_t eq = line.find('='); if (eq == std::string::npos) continue;
        const std::string k = line.substr(0, eq), v = line.substr(eq+1);
        if (k == "project_dir") { if (!v.empty()) s.projectDir = v; }
        else if (k == "slicer_bambu") s.slicerPath[0] = v;
        else if (k == "slicer_prusa") s.slicerPath[1] = v;
        else if (k == "slicer_orca") s.slicerPath[2] = v;
        else if (k == "scan_roll") { const int r = atoi(v.c_str()); s.scanRoll = r >= 0 && r <= 2 ? r : 0; }
        else if (k == "compute") { const int r = atoi(v.c_str()); s.computeMode = r >= 0 && r <= 2 ? r : 0; }
        else if (k == "recent") s.recent.push_back(v);
        else if (k == "hidden") s.hidden.push_back(v);
    }
    return s;
}
bool saveSettings(const Settings& s, const std::string& file) {
    std::error_code ec; fs::create_directories(fs::path(file).parent_path(), ec);
    const std::string tmp = file+".tmp";
    { std::ofstream o(tmp); if (!o) return false;
      o << "project_dir=" << s.projectDir << "\nslicer_bambu=" << s.slicerPath[0] << "\nslicer_prusa=" << s.slicerPath[1] << "\nslicer_orca=" << s.slicerPath[2] << "\nscan_roll=" << s.scanRoll << "\ncompute=" << s.computeMode << "\n";
      for (const std::string& r : s.recent) o << "recent=" << r << "\n";
      for (const std::string& h : s.hidden) o << "hidden=" << h << "\n";
      if (!o.flush()) return false; }
    fs::rename(tmp, file, ec);
    return !ec;
}
std::string sanitizeProjectName(const std::string& name) {
    std::string out;
    for (const char c : name) { if ((unsigned char)c < 32 || c == '/' || c == '\\' || c == 127) continue; out += c; }
    const auto edge = [](char c) { return c == ' ' || c == '.'; };
    while (!out.empty() && edge(out.front())) out.erase(out.begin());
    while (!out.empty() && edge(out.back())) out.pop_back();
    return out;
}
std::string newProjectFile(const std::string& folder, const std::string& name) { return (fs::path(folder)/name/(name+".vfproj")).string(); }
void rememberProject(Settings& s, const std::string& file) {
    s.recent.erase(std::remove(s.recent.begin(), s.recent.end(), file), s.recent.end());
    s.hidden.erase(std::remove(s.hidden.begin(), s.hidden.end(), file), s.hidden.end());
    s.recent.insert(s.recent.begin(), file);
    if (s.recent.size() > 50) s.recent.resize(50);
}
std::vector<ProjectEntry> listProjects(const Settings& s) {
    std::set<std::string> files(s.recent.begin(), s.recent.end()); std::error_code ec;
    for (fs::recursive_directory_iterator it(s.projectDir, fs::directory_options::skip_permission_denied, ec), end; !ec && it != end; it.increment(ec)) {
        if (it.depth() > 1) { it.disable_recursion_pending(); continue; }
        if (it->path().extension() == ".vfproj") files.insert(it->path().string());
    }
    std::vector<ProjectEntry> out;
    for (const std::string& f : files) {
        if (std::find(s.hidden.begin(), s.hidden.end(), f) != s.hidden.end()) continue;
        std::error_code e; const auto when = fs::last_write_time(f, e); if (e) continue;
        const uint64_t size = fs::file_size(f, e); if (e) continue;
        ProjectEntry p; p.file = f; p.name = fs::path(f).stem().string(); p.bytes = size;
        p.modified = std::chrono::system_clock::to_time_t(std::chrono::time_point_cast<std::chrono::system_clock::duration>(std::chrono::file_clock::to_sys(when)));
        out.push_back(std::move(p));
    }
    std::sort(out.begin(), out.end(), [](const ProjectEntry& a, const ProjectEntry& b) { return a.modified > b.modified; });
    return out;
}
std::string thumbnailFile(const std::string& projectFile) { fs::path p(projectFile); p.replace_extension(".thumb.ppm"); return p.string(); }
