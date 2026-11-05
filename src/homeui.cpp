#include "homeui.hpp"
#include "i18n.hpp"
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <algorithm>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include "filedialog.hpp"
#include "project.hpp"
#include "theme.hpp"

namespace fs = std::filesystem;
using forge::px;
static const std::vector<filedialog::Filter> kProjectFilter = {{"VoxelForge-Projekt", {"vfproj"}}}, kModelFilter = {{"3D-Modell", {"stl", "obj", "ply"}}};

HomeUi::HomeUi(Settings& settings) : settings_(settings) {}
HomeUi::~HomeUi() { if (worker_.joinable()) worker_.join(); }
void HomeUi::init() { findSlicers(settings_, slicers_); refresh(); }
void HomeUi::save() { if (!saveSettings(settings_, settingsFile())) forge::toast(tr("Die Einstellungen ließen sich nicht speichern."), forge::Toast::Error); }
void HomeUi::refresh() { entries_ = listProjects(settings_); }
void HomeUi::guard(std::function<void()> action) {
    if (hasUnsavedWork && hasUnsavedWork()) { pending_ = std::move(action); confirmRequest_ = true; }
    else action();
}
void HomeUi::requestNew() { guard([this] { newName_[0] = 0; newFolder_ = settings_.projectDir; newRequest_ = true; }); }
void HomeUi::makeMissingThumbs() {
    if (workerBusy_) return;
    if (worker_.joinable()) worker_.join();
    for (const ProjectEntry& e : entries_) {
        std::error_code ec; const std::string t = thumbnailFile(e.file);
        if (fs::exists(t, ec) || std::find(failed_.begin(), failed_.end(), e.file) != failed_.end()) continue;
        failed_.push_back(e.file);   // one attempt per run, a broken file must not be read again and again
        workerBusy_ = true; const std::string file = e.file;
        worker_ = std::thread([this, file, t] { makeProjectThumbnail(file, t); workerBusy_ = false; });
        return;
    }
}
unsigned HomeUi::thumbFor(const std::string& project) {
    const std::string file = thumbnailFile(project); std::error_code ec; const auto when = fs::last_write_time(file, ec); if (ec) return 0;
    const int64_t stamp = when.time_since_epoch().count(); Thumb& t = thumbs_[project];
    if (t.texture && t.stamp == stamp) return t.texture;
    FILE* f = fopen(file.c_str(), "rb"); if (!f) return t.texture;
    int w = 0, h = 0, maxV = 0; std::vector<unsigned char> px2;
    if (fscanf(f, "P6 %d %d %d", &w, &h, &maxV) == 3 && w > 0 && h > 0 && w < 4096 && h < 4096) { fgetc(f); px2.resize((size_t)w*h*3); if (fread(px2.data(), 1, px2.size(), f) != px2.size()) px2.clear(); }
    fclose(f); if (px2.empty()) return t.texture;
    if (!t.texture) { GLuint id = 0; glGenTextures(1, &id); t.texture = id; }
    glBindTexture(GL_TEXTURE_2D, t.texture); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, px2.data());
    t.stamp = stamp;
    return t.texture;
}
void HomeUi::projectCard(const ProjectEntry& e, ImVec2 size) {
    ImGui::PushID(e.file.c_str()); const ImVec2 p = ImGui::GetCursorScreenPos(), q{p.x+size.x, p.y+size.y};
    const float imgH = size.x*.625f, r = px(14); ImDrawList* dl = ImGui::GetWindowDrawList();
    ImGui::SetNextItemAllowOverlap();   // the small button on top must get its clicks
    const bool open = ImGui::InvisibleButton("##card", size); const bool hovered = ImGui::IsItemHovered(); forge::noteItem(("Projekt "+e.name).c_str());
    dl->AddRectFilled(p, q, forge::col(hovered ? forge::raised : forge::surface), r);
    const unsigned tex = thumbFor(e.file);
    if (tex) dl->AddImageRounded((ImTextureID)(intptr_t)tex, {p.x+1, p.y+1}, {q.x-1, p.y+imgH}, {0, 0}, {1, 1}, IM_COL32_WHITE, r, ImDrawFlags_RoundCornersTop);
    else {
        dl->AddRectFilled({p.x+1, p.y+1}, {q.x-1, p.y+imgH}, forge::col(forge::bg), r, ImDrawFlags_RoundCornersTop);
        const bool gaveUp = !workerBusy_ && std::find(failed_.begin(), failed_.end(), e.file) != failed_.end(); const char* t = gaveUp ? tr("Noch keine Vorschau") : tr("Vorschau wird erstellt"); const ImVec2 ts = ImGui::CalcTextSize(t); dl->AddText({p.x+(size.x-ts.x)/2, p.y+imgH/2-ts.y/2}, forge::col(forge::textFaint), t);
    }
    dl->AddRect(p, q, forge::col(hovered ? forge::sky : forge::line), r);
    ImGui::PushFont(forge::fontMedium); { const std::string n = e.name; dl->PushClipRect({p.x+px(14), p.y}, {q.x-px(14), q.y}, true); dl->AddText({p.x+px(14), p.y+imgH+px(10)}, forge::col(forge::text), n.c_str()); dl->PopClipRect(); } ImGui::PopFont();
    char when[40], sub[80]; const time_t tt = (time_t)e.modified;
    strftime(when, sizeof when, tr("%d.%m.%Y, %H:%M"), localtime(&tt));
    if (e.bytes >= 1000000000ull) snprintf(sub, sizeof sub, "%s  ·  %.1f GB", when, e.bytes/1e9); else snprintf(sub, sizeof sub, "%s  ·  %.0f MB", when, e.bytes/1e6);
    dl->AddText(forge::fontBody, ImGui::GetFontSize()*.88f, {p.x+px(14), p.y+imgH+px(32)}, forge::col(forge::textDim), sub);
    bool removed = false;
    if (hovered || ImGui::IsMouseHoveringRect({q.x-px(40), p.y}, {q.x, p.y+px(40)})) {
        const ImVec2 back = ImGui::GetCursorScreenPos(); ImGui::SetCursorScreenPos({q.x-px(38), p.y+px(8)});
        char key[300]; snprintf(key, sizeof key, "Entfernen %s", e.name.c_str());
        if (forge::iconButton(IC_X, key, tr("Aus der Liste nehmen. Die Dateien bleiben erhalten."), false, px(30))) removed = true;
        ImGui::SetCursorScreenPos(back);
    }
    if (hovered) ImGui::SetTooltip("%s", e.file.c_str());
    if (removed) { settings_.hidden.push_back(e.file); settings_.recent.erase(std::remove(settings_.recent.begin(), settings_.recent.end(), e.file), settings_.recent.end()); save(); ImGui::PopID(); refresh(); return; }
    if (open) { const std::string f = e.file; guard([this, f] { if (onOpenProject) onOpenProject(f); }); }
    ImGui::PopID();
}
void HomeUi::page() {
    makeMissingThumbs();
    const float pad = px(28); ImGui::SetCursorPos({pad, px(20)});
    ImGui::BeginChild("##home", {ImGui::GetContentRegionAvail().x-pad, ImGui::GetContentRegionAvail().y-px(8)}, 0, 0);
    ImGui::PushFont(forge::fontTitle); ImGui::TextUnformatted(tr("Willkommen bei VoxelForge")); ImGui::PopFont();
    ImGui::TextColored(forge::textDim, tr("Neues Projekt beginnen oder dort weitermachen, wo du aufgehört hast.")); ImGui::Dummy({0, px(10)});
    const ImVec2 tile{px(190), px(84)};
    if (forge::toolTile(IC_FOLDER_PLUS, tr("Neues Projekt"), tile)) requestNew();
    ImGui::SameLine(0, px(12));
    if (forge::toolTile(IC_FOLDER_OPEN, tr("Projekt öffnen"), tile)) guard([this] { filedialog::request(filedialog::Kind::Open, tr("Projekt öffnen"), settings_.projectDir, kProjectFilter, [this](const std::string& f) { if (onOpenProject) onOpenProject(f); }); });
    ImGui::SameLine(0, px(12));
    if (forge::toolTile(IC_FILE_BOX, tr("Modell öffnen"), tile)) filedialog::request(filedialog::Kind::Open, tr("Modell öffnen"), settings_.projectDir, kModelFilter, [this](const std::string& f) { if (onOpenModel) onOpenModel(f); });
    const std::string current = currentName ? currentName() : "";
    if (!current.empty()) { ImGui::SameLine(0, px(12)); const std::string l = trf("Weiter: %s", current); if (forge::toolTile(IC_ARROW_RIGHT, l.c_str(), tile, true) && onResume) onResume(); }
    ImGui::Dummy({0, px(18)});
    ImGui::PushFont(forge::fontMedium); ImGui::TextUnformatted(tr(IC_CLOCK "  Letzte Projekte")); ImGui::PopFont(); ImGui::Dummy({0, px(4)});
    if (entries_.empty()) forge::hint(tr("Noch keine Projekte. Sobald du eines anlegst, erscheint es hier mit einem Vorschaubild."));
    const float gap = px(14), avail = ImGui::GetContentRegionAvail().x-px(8); const int cols = std::max(1, (int)((avail+gap)/(px(250)+gap))); const float w = (avail-gap*(cols-1))/cols; const ImVec2 card{w, w*.625f+px(60)};
    const std::vector<ProjectEntry> shown = entries_;   // a card may change the list
    // Fixed places: the button that appears on a hovered card must not move the others.
    const ImVec2 origin = ImGui::GetCursorScreenPos(); const size_t rows = (shown.size()+cols-1)/cols;
    for (size_t i = 0; i < shown.size(); i++) { ImGui::SetCursorScreenPos({origin.x+(i%cols)*(w+gap), origin.y+(i/cols)*(card.y+gap)}); projectCard(shown[i], card); }
    ImGui::SetCursorScreenPos(origin); ImGui::Dummy({avail, rows*(card.y+gap)+px(20)});
    ImGui::EndChild();
}
void HomeUi::dialogs() {
    if (newRequest_) { newRequest_ = false; ImGui::OpenPopup("Neues Projekt##dlg"); }
    if (settingsRequest_) { settingsRequest_ = false; findSlicers(settings_, slicers_); ImGui::OpenPopup("Einstellungen##dlg"); }
    if (confirmRequest_) { confirmRequest_ = false; ImGui::OpenPopup("Verwerfen##dlg"); }
    const ImVec2 c = ImGui::GetMainViewport()->GetCenter();
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {px(20), px(18)}); ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, px(16)); ImGui::PushStyleColor(ImGuiCol_PopupBg, forge::surface);
    ImGui::SetNextWindowPos(c, ImGuiCond_Appearing, {.5f, .5f}); ImGui::SetNextWindowSize({px(520), 0}); newDialog();
    ImGui::SetNextWindowPos(c, ImGuiCond_Always, {.5f, .5f}); ImGui::SetNextWindowSize({px(640), 0}); ImGui::SetNextWindowSizeConstraints({px(640), 0}, {px(640), ImGui::GetMainViewport()->Size.y*.94f}); settingsDialog();
    ImGui::SetNextWindowPos(c, ImGuiCond_Appearing, {.5f, .5f}); ImGui::SetNextWindowSize({px(440), 0}); confirmDialog();
    ImGui::PopStyleVar(2); ImGui::PopStyleColor();
}
void HomeUi::newDialog() {
    if (!ImGui::BeginPopupModal("Neues Projekt##dlg", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar)) return;
    ImGui::PushFont(forge::fontTitle); ImGui::TextUnformatted(tr("Neues Projekt")); ImGui::PopFont(); ImGui::Dummy({0, px(4)});
    ImGui::TextColored(forge::textDim, tr("Name"));
    if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
    ImGui::SetNextItemWidth(-1); ImGui::InputTextWithHint("##pname", tr("zum Beispiel Eule"), newName_, sizeof newName_); forge::noteItem("Projektname");
    ImGui::TextColored(forge::textDim, tr("Speicherort"));
    { const float bw = px(120); ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x-bw-ImGui::GetStyle().ItemSpacing.x); char b[1024]; snprintf(b, sizeof b, "%s", newFolder_.c_str()); if (ImGui::InputText("##pfolder", b, sizeof b)) newFolder_ = b; ImGui::SameLine();
      if (forge::button(IC_FOLDER_OPEN "  Wählen##pf", {bw, 0})) filedialog::request(filedialog::Kind::Folder, tr("Speicherort für das Projekt"), newFolder_, {}, [this](const std::string& f) { newFolder_ = f; }); forge::noteItem("Speicherort wählen"); }
    const std::string name = sanitizeProjectName(newName_), file = name.empty() ? "" : newProjectFile(newFolder_, name); std::error_code ec; const bool exists = !file.empty() && fs::exists(file, ec);
    if (!name.empty()) { ImGui::Dummy({0, px(2)}); char b[1200]; snprintf(b, sizeof b, tr("Wird angelegt in: %s"), fs::path(file).parent_path().c_str()); forge::hint(b); }
    if (exists) ImGui::TextColored(forge::amber, tr("Ein Projekt mit diesem Namen gibt es dort schon."));
    ImGui::Dummy({0, px(6)}); const float w = (ImGui::GetContentRegionAvail().x-ImGui::GetStyle().ItemSpacing.x)/2;
    if (forge::button("Abbrechen##np", {w, px(38)})) ImGui::CloseCurrentPopup();
    forge::noteItem("Abbrechen Neues Projekt");
    ImGui::SameLine(); ImGui::BeginDisabled(name.empty() || newFolder_.empty() || exists);
    if (forge::primaryButton("Projekt anlegen##np", {w, px(38)})) { ImGui::CloseCurrentPopup(); if (onNewProject) onNewProject(file); }
    forge::noteItem("Projekt anlegen");
    ImGui::EndDisabled();
    filedialog::frame();
    ImGui::EndPopup();
}
void HomeUi::settingsDialog() {
    if (!ImGui::BeginPopupModal("Einstellungen##dlg", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar)) return;
    ImGui::PushFont(forge::fontTitle); ImGui::TextUnformatted(tr("Einstellungen")); ImGui::PopFont();
    forge::beginCard("##folder", tr("Projekte"), IC_FOLDER);
    forge::hint(tr("Standardspeicherpfad. Jedes neue Projekt bekommt darin einen Unterordner mit seinem Namen."));
    { const float bw = px(110); ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x-bw-ImGui::GetStyle().ItemSpacing.x); char b[1024]; snprintf(b, sizeof b, "%s", settings_.projectDir.c_str()); if (ImGui::InputText("##dir", b, sizeof b, ImGuiInputTextFlags_EnterReturnsTrue) || ImGui::IsItemDeactivatedAfterEdit()) { settings_.projectDir = b; save(); refresh(); } ImGui::SameLine();
      if (forge::button("Wählen##dir", {bw, 0})) filedialog::request(filedialog::Kind::Folder, tr("Standardspeicherpfad"), settings_.projectDir, {}, [this](const std::string& f) { settings_.projectDir = f; save(); refresh(); }); forge::noteItem("Standardpfad wählen"); }
    forge::endCard();
    forge::beginCard("##slicer", tr("Slicer"), IC_PRINTER);
    forge::hint(tr("Werden automatisch gesucht. Liegt ein Programm woanders, hier die ausführbare Datei angeben."));
    for (int i = 0; i < kSlicerCount; i++) {
        ImGui::PushID(i); Slicer& s = slicers_[i];
        ImGui::PushFont(forge::fontMedium); ImGui::TextUnformatted(s.name); ImGui::PopFont();
        ImGui::SameLine(); ImGui::TextColored(s.found ? forge::green : forge::textFaint, "%s", s.found ? (settings_.slicerPath[i].empty() ? tr("automatisch gefunden") : tr("eigener Pfad")) : (settings_.slicerPath[i].empty() ? tr("nicht gefunden") : tr("Pfad ungültig")));
        const float bw = px(84); ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x-2*bw-2*ImGui::GetStyle().ItemSpacing.x);
        char b[1024]; snprintf(b, sizeof b, "%s", settings_.slicerPath[i].c_str());
        if (ImGui::InputTextWithHint("##p", s.where.empty() ? tr("Pfad zum Programm") : s.where.c_str(), b, sizeof b) || false) settings_.slicerPath[i] = b;
        if (ImGui::IsItemDeactivatedAfterEdit()) { save(); findSlicers(settings_, slicers_); }
        ImGui::SameLine(); char key[32]; snprintf(key, sizeof key, "Wählen##s%d", i);
        if (forge::button(key, {bw, 0})) filedialog::request(filedialog::Kind::Open, trf("Programm für %s", s.name), settings_.slicerPath[i].empty() ? "/usr/bin" : settings_.slicerPath[i], {}, [this, i](const std::string& f) { settings_.slicerPath[i] = f; save(); findSlicers(settings_, slicers_); });
        forge::noteItem(key);
        ImGui::SameLine(); ImGui::BeginDisabled(settings_.slicerPath[i].empty()); snprintf(key, sizeof key, "Auto##s%d", i);
        if (forge::button(key, {bw, 0})) { settings_.slicerPath[i].clear(); save(); findSlicers(settings_, slicers_); }
        ImGui::EndDisabled(); ImGui::PopID();
    }
    forge::endCard();
    forge::beginCard("##compute", tr("Berechnung"), IC_GAUGE);
    forge::hint(tr("Automatisch nutzt die Grafikkarte und prüft sie laufend gegen die CPU. Weicht sie ab oder fällt sie aus, rechnet die CPU weiter."));
    { const char* const modes[] = {tr("Automatisch"), "CPU", tr("Grafikkarte")}; if (forge::segmented("compute", settings_.computeMode, modes, 3)) { save(); if (onComputeMode) onComputeMode(settings_.computeMode); } }
    if (computeInfo) { const std::string i = computeInfo(); if (!i.empty()) forge::hint(i.c_str()); }
    forge::endCard();
    if (forge::button("Schließen##set", {-FLT_MIN, px(38)})) { save(); ImGui::CloseCurrentPopup(); }
    forge::noteItem("Einstellungen schließen");
    filedialog::frame();
    ImGui::EndPopup();
}
void HomeUi::confirmDialog() {
    if (!ImGui::BeginPopupModal("Verwerfen##dlg", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar)) return;
    ImGui::PushFont(forge::fontTitle); ImGui::TextUnformatted(tr("Ungespeicherte Scans")); ImGui::PopFont();
    ImGui::PushTextWrapPos(0); ImGui::TextColored(forge::textDim, tr("Das offene Projekt hat Scans, die noch nicht gespeichert sind. Sie gehen verloren, wenn du jetzt weitermachst.")); ImGui::PopTextWrapPos(); ImGui::Dummy({0, px(6)});
    const float w = (ImGui::GetContentRegionAvail().x-ImGui::GetStyle().ItemSpacing.x)/2;
    if (forge::button("Zurück##cf", {w, px(38)})) { pending_ = nullptr; ImGui::CloseCurrentPopup(); }
    forge::noteItem("Zurück");
    ImGui::SameLine();
    if (forge::dangerButton("Verwerfen##cf", {w, px(38)})) { auto a = std::move(pending_); pending_ = nullptr; ImGui::CloseCurrentPopup(); if (a) a(); }
    forge::noteItem("Verwerfen");
    ImGui::EndPopup();
}
