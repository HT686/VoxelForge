#include "filedialog.hpp"
#include <sys/wait.h>
#include <imgui.h>
#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <thread>
#include <unistd.h>
#include "theme.hpp"

namespace fs = std::filesystem;
namespace filedialog {
namespace {
struct Pending { std::mutex m; bool finished = false, failed = false; std::string answer; };
std::shared_ptr<Pending> gPending; std::function<void(const std::string&)> gDone; std::string gTitle;
Kind gKind; std::vector<Filter> gFilters;
// The drawn dialog, when the desktop has no helper.
bool gBuiltin = false, gBuiltinOpen = false; std::string gDir; char gName[256] = ""; std::vector<fs::directory_entry> gEntries;

std::string quote(const std::string& s) { std::string o = "'"; for (const char c : s) { if (c == '\'') o += "'\\''"; else o += c; } return o+"'"; }
bool sandboxed() { static const bool inside = access("/.flatpak-info", F_OK) == 0; return inside; }
bool onPath(const char* tool) {
    if (sandboxed()) { std::error_code ec; return fs::exists(std::string("/run/host/usr/bin/")+tool, ec); }
    const char* p = getenv("PATH"); if (!p) return false; std::string all = p; size_t a = 0;
    while (a <= all.size()) { const size_t b = std::min(all.find(':', a), all.size()); std::error_code ec; if (fs::exists(all.substr(a, b-a)+"/"+tool, ec)) return true; a = b+1; }
    return false;
}
std::string commandFor(Kind kind, const std::string& title, const std::string& start, const std::vector<Filter>& filters) {
    const char* desktop = getenv("XDG_CURRENT_DESKTOP"); const bool kde = desktop && std::string(desktop).find("KDE") != std::string::npos;
    const bool haveK = onPath("kdialog"), haveZ = onPath("zenity");
    if (haveK && (kde || !haveZ)) {
        std::string c = std::string(sandboxed() ? "flatpak-spawn --host " : "")+"kdialog --title "+quote(title)+(kind == Kind::Open ? " --getopenfilename " : kind == Kind::Save ? " --getsavefilename " : " --getexistingdirectory ")+quote(start);
        if (kind != Kind::Folder && !filters.empty()) {
            std::string f; for (const Filter& fl : filters) { if (!f.empty()) f += "\n"; std::string pat; for (const std::string& e : fl.extensions) pat += (pat.empty() ? "*." : " *.")+e; f += pat+"|"+fl.name; }
            c += " "+quote(f);
        }
        return c+" 2>/dev/null";
    }
    if (haveZ) {
        std::string c = std::string(sandboxed() ? "flatpak-spawn --host " : "")+"zenity --file-selection --title="+quote(title)+" --filename="+quote(start);
        if (kind == Kind::Save) c += " --save --confirm-overwrite";
        if (kind == Kind::Folder) c += " --directory";
        for (const Filter& fl : filters) { std::string pat; for (const std::string& e : fl.extensions) pat += " *."+e; c += " "+quote("--file-filter="+fl.name+" |"+pat); }
        return c+" 2>/dev/null";
    }
    return "";
}
// For scripted checks: the next line of this file is the answer, an empty line means cancelled.
bool scriptedAnswer(std::string& out) {
    const char* file = getenv("VOXELFORGE_FILE_ANSWERS"); if (!file) return false;
    static std::vector<std::string> lines; static size_t next = 0; static bool loaded = false;
    if (!loaded) { std::ifstream in(file); std::string l; while (std::getline(in, l)) lines.push_back(l); loaded = true; }
    out = next < lines.size() ? lines[next++] : ""; return true;
}
void listDir() {
    gEntries.clear(); std::error_code ec;
    for (fs::directory_iterator it(gDir, fs::directory_options::skip_permission_denied, ec), end; !ec && it != end; it.increment(ec)) {
        const std::string n = it->path().filename().string(); if (n.empty() || n[0] == '.') continue;
        std::error_code e; const bool dir = it->is_directory(e);
        if (!dir) {
            if (gKind == Kind::Folder) continue;
            if (!gFilters.empty()) { std::string ext = it->path().extension().string(); if (!ext.empty()) ext.erase(0, 1); std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower); bool ok = false; for (const Filter& f : gFilters) for (const std::string& x : f.extensions) ok |= x == ext; if (!ok) continue; }
        }
        gEntries.push_back(*it);
    }
    std::sort(gEntries.begin(), gEntries.end(), [](const fs::directory_entry& a, const fs::directory_entry& b) { std::error_code e; const bool da = a.is_directory(e), db = b.is_directory(e); if (da != db) return da; return a.path().filename() < b.path().filename(); });
}
std::string withExtension(std::string path) {
    if (gKind == Kind::Save && !gFilters.empty() && !gFilters[0].extensions.empty() && fs::path(path).extension().empty()) path += "."+gFilters[0].extensions[0];
    return path;
}
void deliver(const std::string& path) { auto done = std::move(gDone); gDone = nullptr; gPending.reset(); if (done && !path.empty()) done(withExtension(path)); }
void builtinDialog() {
    if (!gBuiltinOpen) { ImGui::OpenPopup("Datei wählen##fd"); gBuiltinOpen = true; }
    const ImVec2 c = ImGui::GetMainViewport()->GetCenter(); ImGui::SetNextWindowPos(c, ImGuiCond_Appearing, {.5f, .5f}); ImGui::SetNextWindowSize({forge::px(620), forge::px(520)});
    if (!ImGui::BeginPopupModal("Datei wählen##fd", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar)) return;
    ImGui::PushFont(forge::fontTitle); ImGui::TextUnformatted(gTitle.c_str()); ImGui::PopFont();
    if (forge::button(IC_CHEVRON_UP "  Übergeordneter Ordner")) { gDir = fs::path(gDir).parent_path().string(); listDir(); }
    ImGui::SameLine(); ImGui::TextDisabled("%s", gDir.c_str());
    ImGui::BeginChild("##list", {0, -forge::px(92)}, ImGuiChildFlags_Borders);
    for (const fs::directory_entry& e : gEntries) {
        std::error_code ec; const bool dir = e.is_directory(ec); const std::string n = e.path().filename().string(), label = std::string(dir ? IC_FOLDER "  " : "      ")+n;
        if (ImGui::Selectable(label.c_str(), n == gName)) { if (dir) { gDir = e.path().string(); gName[0] = 0; listDir(); break; } snprintf(gName, sizeof gName, "%s", n.c_str()); }
    }
    ImGui::EndChild();
    if (gKind != Kind::Folder) { ImGui::SetNextItemWidth(-1); ImGui::InputTextWithHint("##name", "Dateiname", gName, sizeof gName); }
    const float w = (ImGui::GetContentRegionAvail().x-ImGui::GetStyle().ItemSpacing.x)/2;
    if (forge::button("Abbrechen##fd", {w, forge::px(36)})) { ImGui::CloseCurrentPopup(); gBuiltin = gBuiltinOpen = false; deliver(""); ImGui::EndPopup(); return; }
    ImGui::SameLine(); ImGui::BeginDisabled(gKind != Kind::Folder && !*gName);
    if (forge::primaryButton(gKind == Kind::Save ? "Speichern##fd" : "Auswählen##fd", {w, forge::px(36)})) { const std::string p = gKind == Kind::Folder ? gDir : (fs::path(gDir)/gName).string(); ImGui::CloseCurrentPopup(); gBuiltin = gBuiltinOpen = false; deliver(p); ImGui::EndDisabled(); ImGui::EndPopup(); return; }
    ImGui::EndDisabled();
    ImGui::EndPopup();
}
}
bool active() { return gPending != nullptr || gBuiltin; }
void request(Kind kind, const std::string& title, const std::string& start, const std::vector<Filter>& filters, std::function<void(const std::string&)> done) {
    if (active()) return;
    gDone = std::move(done); gTitle = title; gKind = kind; gFilters = filters;
    std::string scripted;
    if (scriptedAnswer(scripted)) { gPending = std::make_shared<Pending>(); gPending->finished = true; gPending->answer = scripted; return; }
    const std::string command = commandFor(kind, title, start, filters);
    auto openBuiltin = [&] {
        std::error_code ec; fs::path p(start); gDir = fs::is_directory(p, ec) ? p.string() : p.parent_path().string(); if (gDir.empty() || !fs::is_directory(gDir, ec)) { const char* h = getenv("HOME"); gDir = h ? h : "/"; }
        snprintf(gName, sizeof gName, "%s", kind == Kind::Save ? p.filename().c_str() : ""); listDir(); gBuiltin = true; gBuiltinOpen = false;
    };
    if (command.empty()) { openBuiltin(); return; }
    auto pending = std::make_shared<Pending>(); gPending = pending;
    std::thread([pending, command] {
        std::string out; int status = -1;
        if (FILE* p = popen(command.c_str(), "r")) { char b[4096]; while (fgets(b, sizeof b, p)) out += b; status = pclose(p); }
        while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) out.pop_back();
        std::lock_guard<std::mutex> g(pending->m); pending->answer = out; pending->failed = status == -1 || (WIFEXITED(status) && WEXITSTATUS(status) >= 126); pending->finished = true;
    }).detach();
}
void frame() {
    if (gBuiltin) { builtinDialog(); return; }
    if (!gPending) return;
    bool finished, failed; std::string answer;
    { std::lock_guard<std::mutex> g(gPending->m); finished = gPending->finished; failed = gPending->failed; answer = gPending->answer; }
    if (finished) {
        if (failed) { gPending.reset(); forge::toast("Der Dateidialog des Systems ließ sich nicht öffnen.", forge::Toast::Error); gDone = nullptr; return; }
        deliver(answer); return;
    }
    // The dialog is another window: keep this one from being used meanwhile.
    ImGuiViewport* vp = ImGui::GetMainViewport(); ImGui::SetNextWindowPos(vp->Pos); ImGui::SetNextWindowSize(vp->Size); ImGui::SetNextWindowFocus();
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0, 0, 0, .45f));
    ImGui::Begin("##filewait", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);
    const char* t = "Dateidialog ist geöffnet"; const ImVec2 ts = ImGui::CalcTextSize(t); ImGui::SetCursorPos({(vp->Size.x-ts.x)/2, (vp->Size.y-ts.y)/2}); ImGui::TextUnformatted(t);
    ImGui::End(); ImGui::PopStyleColor();
}
}
