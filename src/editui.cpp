#include <imgui_internal.h>
#include <imgui.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include "editui.hpp"
#include "filedialog.hpp"
#include "theme.hpp"

using forge::px;

namespace {
const char* const kLevels[] = {"Keine", "Niedrig", "Mittel", "Hoch"};
const char* const kFillTypes[] = {"Planar", "Tangente", "Krümmung"};
const float kVoxels[] = {0.15f, 0.25f, 0.5f, 1.f};
const char* const kVoxelNames[] = {"0,15 mm", "0,25 mm", "0,5 mm", "1 mm"};
std::string grouped(size_t n) {
    std::string s = std::to_string(n);
    for (int i = (int)s.size()-3; i > 0; i -= 3) s.insert((size_t)i, ".");
    return s;
}
std::string holesText(size_t n) { return n == 1 ? "1 Loch" : std::to_string(n) + " Löcher"; }
std::string outputBase(float voxelMm) {
    const char* home = getenv("HOME"); std::string base = home ? home : ".";
    for (const char* d : {"/Dokumente", "/Documents"}) if (std::filesystem::is_directory(base + d)) { base += d; break; }
    base += "/VoxelForge";
    char ts[32]; time_t t = time(nullptr); strftime(ts, sizeof ts, "scan_%Y%m%d_%H%M%S", localtime(&t));
    char res[16]; snprintf(res, sizeof res, "_%gmm", voxelMm);
    return base + "/" + ts + res;
}
bool insidePolygon(float x, float y, const std::vector<ImVec2>& poly) {
    bool in = false;
    for (size_t i = 0, j = poly.size()-1; i < poly.size(); j = i++)
        if ((poly[i].y > y) != (poly[j].y > y) && x < (poly[j].x-poly[i].x)*(y-poly[i].y)/(poly[j].y-poly[i].y)+poly[i].x) in = !in;
    return in;
}
// Green for small, red for large holes.
void holeColor(float perimeter, float& r, float& g, float& b) {
    const float t = std::clamp(std::log(std::max(perimeter, 1.f)/5.f)/std::log(200.f/5.f), 0.f, 1.f);
    r = std::min(1.f, 2*t); g = std::min(1.f, 2*(1-t)); b = 0.1f;
}
}

EditUi::EditUi(MeshView& view, ScanSource source) : view_(view), source_(std::move(source)) { loadOneClickConfig(configPath(), oneClick_); }
EditUi::~EditUi() { if (job_.joinable()) job_.join(); }
std::string EditUi::configPath() const {
    const char* xdg = getenv("XDG_CONFIG_HOME"); const char* home = getenv("HOME");
    return (xdg && *xdg ? std::string(xdg) : std::string(home ? home : ".") + "/.config") + "/voxelforge/oneclick.txt";
}
void EditUi::setProgress(const std::string& label, float f) { std::lock_guard<std::mutex> g(progressMutex_); jobLabel_ = label; jobProgress_ = f; }
void EditUi::startJob(const std::string& label, std::function<void()> work, std::function<void()> finish) {
    if (jobRunning_) return;
    if (job_.joinable()) job_.join();
    setProgress(label, -1); jobFinish_ = std::move(finish); jobRunning_ = true; jobDone_ = false;
    job_ = std::thread([this, work = std::move(work)] { work(); jobDone_ = true; });
}
void EditUi::frame() {
    if (jobDone_.exchange(false)) {
        if (job_.joinable()) job_.join();
        jobRunning_ = false;
        auto f = std::move(jobFinish_); jobFinish_ = nullptr; if (f) f();
    }
    if (!jobRunning_ && analysisDirty_ && hasMesh()) {
        analysisDirty_ = false; analysisRunning_ = true;
        startJob("Netz wird geprüft", [this] { jobHoles_ = findHoles(base_); }, [this] {
            holes_ = std::move(jobHoles_); analysisRunning_ = false; openEdges_ = 0; for (const MeshHole& h : holes_) openEdges_ += h.loop.size();
            holeChosen_.assign(holes_.size(), 0); holeFilled_.assign(holes_.size(), 0); holeHover_ = -1; linesDirty_ = true;
            holeLimit_ = std::max(holeLimit_, 1.f);
        });
    }
    if (oneClickRequest_) { oneClickRequest_ = false; ImGui::OpenPopup("Ein-Klick-Prozess"); }
    oneClickDialog();
    if (flagsDirty_) { refreshFlags(); flagsDirty_ = false; }
    if (linesDirty_) { rebuildHoleLines(); linesDirty_ = false; }
    const ImGuiIO& io = ImGui::GetIO();
    if (!io.WantTextInput && hasMesh() && !jobRunning_) {
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z, false)) undo();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Y, false)) redo();
        if (ImGui::IsKeyPressed(ImGuiKey_Delete, false) && selectedCount_) deleteSelected();
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) { lasso_.clear(); dragging_ = false; bridgeHasFirst_ = false; select_ = SelectMode::None; }
    }
}
void EditUi::meshChanged(bool resetCamera) {
    partsValid_ = false; selected_.assign(base_.triangleCount(), 0); selectedCount_ = 0; lasso_.clear();
    meshBounds(base_, lo_, hi_); tightBoxSize(base_, tight_); haveSize_ = hasMesh(); volume_ = meshVolume(base_);
    analysisDirty_ = true; holes_.clear(); holeChosen_.clear(); holeFilled_.clear(); holeHover_ = -1; bridgeHasFirst_ = false;
    showCurrent(resetCamera);
}
void EditUi::showCurrent(bool resetCamera) { view_.setMesh(shown(), resetCamera); if (!resetCamera) view_.keepInSight(shown()); flagsDirty_ = true; linesDirty_ = true; }
void EditUi::refreshFlags() {
    if (!hasMesh()) { view_.setFlags({}); return; }
    const size_t n = shown().triangleCount();
    if (hasPreview_ && firstFilling_ && firstFilling_ < n) { std::vector<uint8_t> f(n, MeshView::Plain); std::fill(f.begin()+firstFilling_, f.end(), (uint8_t)MeshView::Filling); view_.setFlags(f); }
    else if (tool_ == EditTool::Cut && cutHave_ && cutMarks_.size() == n) { std::vector<uint8_t> f(n); for (size_t i = 0; i < n; i++) f[i] = cutMarks_[i] ? MeshView::Marked : MeshView::Plain; view_.setFlags(f); }
    else if (selectedCount_ && selected_.size() == n) { std::vector<uint8_t> f(n); for (size_t i = 0; i < n; i++) f[i] = selected_[i] ? MeshView::Marked : MeshView::Plain; view_.setFlags(f); }
    else view_.setFlags({});
}
void EditUi::updateOutPath(float voxelMm) {
    if (outputBase_.empty()) { snprintf(outPath_, sizeof outPath_, "%s", outputBase(voxelMm).c_str()); return; }
    char v[32]; snprintf(v, sizeof v, "_%.2fmm", voxelMm); snprintf(outPath_, sizeof outPath_, "%s%s", outputBase_.c_str(), v);
}
void EditUi::clear() { Mesh none; base_ = std::move(none); preview_ = Mesh{}; hasPreview_ = false; firstFilling_ = 0; history_.clear(); tool_ = EditTool::None; select_ = SelectMode::None; measurePoints_.clear(); holes_.clear(); openEdges_ = 0; haveSize_ = false; view_.setMesh(base_, true); }
void EditUi::exportForPrint(std::function<void(const std::string&)> open) {
    if (!hasMesh() || jobRunning_) return;
    if (!*outPath_) updateOutPath(voxelMm_);
    const std::string file = (outputBase_.empty() ? std::string(outPath_) : outputBase_)+"_3d-druck.stl";
    auto ok = std::make_shared<bool>(false); auto err = std::make_shared<std::string>();
    startJob("Modell für den 3D-Druck wird geschrieben", [this, file, ok, err] { std::error_code ec; std::filesystem::create_directories(std::filesystem::path(file).parent_path(), ec); *ok = saveMesh(base_, file, *err); }, [file, ok, err, open] { if (*ok) open(file); else forge::toast(*err, forge::Toast::Error); });
}
void EditUi::newMesh(Mesh&& m, float voxelMm) {
    base_ = std::move(m); voxelMm_ = voxelMm; history_.clear(); hasPreview_ = false; firstFilling_ = 0; tool_ = EditTool::None; select_ = SelectMode::None; measurePoints_.clear();
    updateOutPath(voxelMm);
    meshChanged(true);
    if (colorsExpected && colorsExpected() && base_.colors.empty()) forge::toast("Zu wenig Licht für Farbe. Das Modell ist ohne Farbe. Für Farbe den Raum heller machen und neu scannen.", forge::Toast::Warning);
    if (onMeshArrived) onMeshArrived();
}
bool EditUi::loadFile(const std::string& path) {
    Mesh m; std::string err;
    if (!loadMeshFile(path, m, err)) { forge::toast(err, forge::Toast::Error); return false; }
    repairManifold(m); newMesh(std::move(m), 0.5f);
    return true;
}
std::string EditUi::describe() const {
    size_t chosen = 0; for (uint8_t c : holeChosen_) chosen += c;
    char b[260]; snprintf(b, sizeof b, "Dreiecke=%zu Vorschau=%d Löcher=%zu gewählt=%zu offen=%zu Auswahl=%zu Füllung=%zu Rückgängig=%d Messpunkte=%zu Größe=%.1fx%.1fx%.1f", shown().triangleCount(), hasPreview_, holes_.size(), chosen, openEdges_, selectedCount_, hasPreview_ && firstFilling_ ? shown().triangleCount()-firstFilling_ : 0, history_.canUndo(), measurePoints_.size(), tight_[0], tight_[1], tight_[2]);
    return b;
}
void EditUi::dropPreview() { if (!hasPreview_) return; hasPreview_ = false; firstFilling_ = 0; preview_ = Mesh{}; holeFilled_.assign(holes_.size(), 0); showCurrent(false); }
void EditUi::confirmPreview() {
    if (!hasPreview_) return;
    history_.push(base_); base_ = std::move(preview_); preview_ = Mesh{}; hasPreview_ = false; firstFilling_ = 0;
    meshChanged(false);
}
void EditUi::runPreview(const std::string& label, std::function<void(Mesh&)> work, size_t firstFilling) {
    auto result = std::make_shared<Mesh>(base_);
    startJob(label, [result, work = std::move(work)] { work(*result); }, [this, result, firstFilling] {
        preview_ = std::move(*result); hasPreview_ = true; firstFilling_ = firstFilling; showCurrent(false);
    });
}
void EditUi::undo() { if (hasPreview_) { dropPreview(); return; } if (history_.undo(base_)) meshChanged(false); }
void EditUi::redo() { if (!hasPreview_ && history_.redo(base_)) meshChanged(false); }
void EditUi::chooseTool(EditTool t) {
    dropPreview(); tool_ = tool_ == t ? EditTool::None : t; select_ = SelectMode::None; bridging_ = false; bridgeHasFirst_ = false; pickFloor_ = false;
    cutHave_ = false; cutPoints_.clear(); cutMarks_.clear(); rotX_ = rotY_ = rotZ_ = 0; flagsDirty_ = true; linesDirty_ = true;
    if (t == EditTool::Holes && tool_ == t && holes_.size() != holeChosen_.size()) holeChosen_.assign(holes_.size(), 0);
}
bool EditUi::footer(const std::function<void()>& preview, const std::function<void()>& reset) {
    const float gap = ImGui::GetStyle().ItemSpacing.x, w = ImGui::GetContentRegionAvail().x; const float h = px(38);
    bool left = false;
    ImGui::BeginDisabled(jobRunning_);
    if (!hasPreview_) {
        if (forge::primaryButton(IC_EYE "  Vorschau##f", {-FLT_MIN, h})) preview();
        forge::noteItem("Vorschau");
        if (forge::button("Zurücksetzen", {(w-gap)/2, 0})) reset();
        ImGui::SameLine(); if (forge::button("Abbrechen", {(w-gap)/2, 0})) { chooseTool(tool_); left = true; }
    } else {
        if (forge::primaryButton(IC_CHECK "  Bestätigen##f", {-FLT_MIN, h})) confirmPreview();
        forge::noteItem("Bestätigen");
        if (forge::button("Abbrechen", {-FLT_MIN, 0})) dropPreview();
    }
    ImGui::EndDisabled();
    return left;
}
void EditUi::toolHeader(const char* title, const char* text) {
    if (forge::iconButton(IC_CHEVRON_LEFT, "Zurück zu Werkzeugen", "Zurück", false, px(30))) { const EditTool t = tool_; chooseTool(t); }
    ImGui::SameLine(0, px(6)); ImGui::PushFont(forge::fontTitle); ImGui::SetCursorPosY(ImGui::GetCursorPosY()+px(6)); ImGui::TextUnformatted(title); ImGui::PopFont();
    if (text) forge::hint(text);
    ImGui::Dummy({0, px(2)});
}
void EditUi::panel() {
    if (tool_ == EditTool::None) {
        forge::beginCard("##tools", "Werkzeuge", IC_BOXES);
        ImGui::BeginDisabled(!hasMesh() || jobRunning_);
        const float gap = px(8), w = (ImGui::GetContentRegionAvail().x-gap)/2, h = px(70);
        auto tile = [&](const char* icon, const char* label, EditTool t, bool sameLine) { if (forge::toolTile(icon, label, {w, h}, tool_ == t)) chooseTool(t); if (sameLine) ImGui::SameLine(0, gap); };
        tile(IC_TRIANGLE, "Vereinfachen", EditTool::Simplify, true); tile(IC_WAVES, "Glätten", EditTool::Smooth, false);
        tile(IC_BRUSH, "Bereinigen", EditTool::Clean, true); tile(IC_CIRCLE_DASHED, "Lochfüllung", EditTool::Holes, false);
        tile(IC_ROTATE_3D, "Modell ausrichten", EditTool::Align, true); tile(IC_SCISSORS, "Schnittebene", EditTool::Cut, false);
        ImGui::EndDisabled();
        ImGui::BeginDisabled(jobRunning_ || (!hasMesh() && !source_.available()));
        if (forge::primaryButton(IC_SPARKLES "  Ein-Klick-Prozess##t", {-FLT_MIN, px(38)})) openOneClick();
        forge::noteItem("Ein-Klick-Prozess");
        ImGui::EndDisabled();
        if (hasMesh() && view_.hasColor()) forge::toggle("Farbe anzeigen", &view_.showColor);
        if (hasMesh()) forge::toggle("Boden anzeigen", &view_.showFloor);
        forge::endCard();
        exportPanel();
        return;
    }
    forge::beginCard("##tool");
    switch (tool_) {
    case EditTool::Simplify: {
        toolHeader("Vereinfachen", "Verringert die Zahl der Dreiecke und erhält dabei die Form so gut wie möglich.");
        if (forge::sliderInt("Dreiecke behalten", &simplifyPercent_, 1, 100, "%d %%")) dropPreview();
        forge::labelValue("Ergebnis", ("etwa "+grouped(base_.triangleCount()*simplifyPercent_/100)).c_str());
        footer([this] { const size_t target = base_.triangleCount()*simplifyPercent_/100; runPreview("Vereinfachen", [target](Mesh& m) { simplifyMesh(m, target); computeNormals(m); }); }, [this] { simplifyPercent_ = 50; dropPreview(); });
        break; }
    case EditTool::Smooth: {
        toolHeader("Glätten", "Beruhigt die Oberfläche, ohne dass das Modell schrumpft. Offene Ränder bleiben stehen.");
        if (forge::segmented("smooth", smoothLevel_, kLevels, 4)) { smoothIterations_ = smoothIterationsFor((Level)smoothLevel_); dropPreview(); }
        if (forge::sliderInt("Durchgänge", &smoothIterations_, 0, 40)) { smoothLevel_ = smoothIterations_ == 3 ? 1 : smoothIterations_ == 8 ? 2 : smoothIterations_ == 20 ? 3 : smoothIterations_ == 0 ? 0 : -1; dropPreview(); }
        if (forge::toggle("Scharfe Kanten erhalten", &keepEdges_)) dropPreview();
        footer([this] { SmoothOptions o; o.iterations = smoothIterations_; o.keepEdges = keepEdges_; runPreview("Glätten", [o](Mesh& m) { smoothSurface(m, o); }); }, [this] { smoothLevel_ = 1; smoothIterations_ = 3; keepEdges_ = true; dropPreview(); });
        break; }
    case EditTool::Clean: {
        toolHeader("Bereinigen", "Entfernt, was nicht zum Objekt gehört.");
        char fmt[48]; snprintf(fmt, sizeof fmt, cleanShare_ <= 0 ? "aus" : "unter %.1f %%%%", cleanShare_);
        if (forge::slider("Kleine Teile entfernen", &cleanShare_, 0, 50, fmt)) dropPreview();
        ImGui::PushStyleColor(ImGuiCol_Text, forge::textDim); ImGui::TextUnformatted("Spitzen entfernen"); ImGui::PopStyleColor();
        if (forge::segmented("spikes", cleanSpikes_, kLevels, 4)) dropPreview();
        ImGui::PushStyleColor(ImGuiCol_Text, forge::textDim); ImGui::TextUnformatted("Ränder glätten"); ImGui::PopStyleColor();
        if (forge::segmented("rims", cleanRims_, kLevels, 4)) dropPreview();
        footer([this] { const float share = cleanShare_/100; const Level sp = (Level)cleanSpikes_, ri = (Level)cleanRims_; runPreview("Bereinigen", [=](Mesh& m) { removeSmallParts(m, share); removeSpikes(m, sp); smoothRims(m, ri); computeNormals(m); }); }, [this] { cleanShare_ = 5; cleanSpikes_ = 2; cleanRims_ = 1; dropPreview(); });
        break; }
    case EditTool::Holes: holesPanel(); break;
    case EditTool::Align: alignPanel(); break;
    case EditTool::Cut: cutPanel(); break;
    case EditTool::None: break;
    }
    forge::endCard();
}
void EditUi::exportPanel() {
    forge::beginCard("##export", "Modell exportieren", IC_DOWNLOAD);
    ImGui::BeginDisabled(!hasMesh() || jobRunning_);
    const char* exts[] = {".stl", ".obj", ".ply"}; const char* labels[] = {"STL", "OBJ", "PLY"};
    const float bw = (ImGui::GetContentRegionAvail().x-2*ImGui::GetStyle().ItemSpacing.x)/3;
    for (int i = 0; i < 3; i++) {
        if (i) ImGui::SameLine();
        if (forge::button(labels[i], {bw, 0})) {
            const std::string ext = exts[i]; const std::vector<filedialog::Filter> filter = {{std::string(labels[i])+"-Datei", {ext.substr(1)}}};
            filedialog::request(filedialog::Kind::Save, std::string("Modell als ")+labels[i]+" speichern", uniquePath(outPath_, exts[i]), filter, [this](const std::string& p) {
                std::string err; std::error_code ec; std::filesystem::create_directories(std::filesystem::path(p).parent_path(), ec);
                if (saveMesh(base_, p, err)) forge::toast("Gespeichert: "+std::filesystem::path(p).filename().string(), forge::Toast::Success); else forge::toast(err, forge::Toast::Error);
            });
        }
        forge::noteItem(labels[i]);
    }
    ImGui::EndDisabled();
    forge::endCard();
    if (printCard) printCard();
    forge::beginCard("##open", "Modell öffnen", IC_FOLDER_OPEN);
    ImGui::BeginDisabled(jobRunning_);
    if (forge::button(IC_FOLDER_OPEN "  Datei wählen ...##model", {-FLT_MIN, 0})) {
        const std::string start = !outputFolder_.empty() ? outputFolder_ : std::filesystem::path(outPath_).parent_path().string();
        filedialog::request(filedialog::Kind::Open, "Modell öffnen", start, {{"3D-Modell", {"stl", "obj", "ply"}}}, [this](const std::string& p) { if (loadFile(p)) forge::toast("Geöffnet: "+std::filesystem::path(p).filename().string(), forge::Toast::Success); });
    }
    forge::noteItem("Modell öffnen");
    ImGui::EndDisabled();
    forge::endCard();
    if (!oneClickReport_.empty()) forge::hint(oneClickReport_.c_str());
}

// ---- Holes ---------------------------------------------------------------------------------------
void EditUi::previewFill() {
    std::vector<MeshHole> use; holeFilled_.assign(holes_.size(), 0);
    for (size_t i = 0; i < holes_.size(); i++) {
        const bool take = holeManual_ ? (i < holeChosen_.size() && holeChosen_[i]) : holes_[i].perimeterMm <= holeLimit_;
        if (take) { use.push_back(holes_[i]); holeFilled_[i] = 1; }
    }
    if (use.empty()) { forge::toast(holeManual_ ? "Erst Löcher anklicken." : "Kein Loch liegt unter diesem Umfang.", forge::Toast::Warning); hasPreview_ = false; linesDirty_ = true; return; }
    const FillType ty = (FillType)fillType_; const size_t first = base_.triangleCount();
    runPreview("Löcher füllen", [use = std::move(use), ty](Mesh& m) { fillHoleList(m, use, ty); }, first);
}
void EditUi::holesPanel() {
    toolHeader("Lochfüllung");
    if (analysisRunning_ || analysisDirty_) { forge::hint("Löcher werden gesucht ..."); return; }
    if (holes_.empty()) { forge::chip("Keine Löcher: das Modell ist wasserdicht", forge::green, IC_CIRCLE_CHECK); return; }
    int mode = holeManual_ ? 1 : 0; const char* const modes[] = {"Auto", "Manuell"};
    if (forge::segmented("mode", mode, modes, 2)) { holeManual_ = mode == 1; dropPreview(); bridging_ = false; bridgeHasFirst_ = false; linesDirty_ = true; }
    ImGui::PushStyleColor(ImGuiCol_Text, forge::textDim); ImGui::TextUnformatted("Typ"); ImGui::PopStyleColor();
    if (forge::segmented("type", fillType_, kFillTypes, 3) && hasPreview_) { dropPreview(); previewFill(); }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Planar: straff gespannt.\nTangente: läuft in Richtung der Umgebung aus.\nKrümmung: setzt auch die Wölbung fort, empfindlich bei unruhigen Rändern.");
    const float maxP = holes_.back().perimeterMm;
    if (!holeManual_) {
        if (forge::slider("Lochumfang (mm)", &holeLimit_, 1, std::max(2.f, maxP*1.05f), "%.0f", true)) { dropPreview(); linesDirty_ = true; }
        size_t n = 0; for (const MeshHole& h : holes_) n += h.perimeterMm <= holeLimit_;
        char b[48]; snprintf(b, sizeof b, "%zu von %zu", n, holes_.size()); forge::labelValue("Werden gefüllt", b);
        snprintf(b, sizeof b, "%.0f mm Umfang", maxP); forge::labelValue("Größtes Loch", b);
    } else {
        size_t chosen = 0; for (uint8_t c : holeChosen_) chosen += c;
        char b[48]; snprintf(b, sizeof b, "%zu von %zu", chosen, holes_.size()); forge::labelValue("Ausgewählt", b);
        forge::hint("Rand im Bild anklicken. Grün ist klein, rot ist groß.");
        const float w = (ImGui::GetContentRegionAvail().x-ImGui::GetStyle().ItemSpacing.x)/2;
        if (forge::button("Alle", {w, 0})) { std::fill(holeChosen_.begin(), holeChosen_.end(), 1); dropPreview(); linesDirty_ = true; }
        ImGui::SameLine(); if (forge::button("Keine", {w, 0})) { std::fill(holeChosen_.begin(), holeChosen_.end(), 0); dropPreview(); linesDirty_ = true; }
        const float lineH = ImGui::GetTextLineHeightWithSpacing(); const float listH = std::clamp(ImGui::GetMainViewport()->Size.y*.3f, 5*lineH, 16*lineH);
        ImGui::PushStyleColor(ImGuiCol_ChildBg, forge::bg);
        ImGui::BeginChild("##holes", {-1, listH}, ImGuiChildFlags_AlwaysUseWindowPadding);
        ImGuiListClipper clip; clip.Begin((int)holes_.size());
        long hoverFromList = -1;
        while (clip.Step()) for (int row = clip.DisplayStart; row < clip.DisplayEnd; row++) {
            const size_t i = holes_.size()-1-(size_t)row;   // largest first
            float r, g, bl; holeColor(holes_[i].perimeterMm, r, g, bl); char label[96];
            snprintf(label, sizeof label, "      %.1f mm, %.1f mm groß##%zu", holes_[i].perimeterMm, holes_[i].sizeMm, i);
            const ImVec2 at = ImGui::GetCursorScreenPos();
            if (ImGui::Selectable(label, holeChosen_[i] != 0, ImGuiSelectableFlags_AllowDoubleClick)) {
                holeChosen_[i] = !holeChosen_[i]; dropPreview(); linesDirty_ = true;
                if (ImGui::IsMouseDoubleClicked(0)) { holeChosen_[i] = 1; view_.focus(holes_[i].center, std::max(holes_[i].sizeMm, 2.f)); }
            }
            ImGui::GetWindowDrawList()->AddCircleFilled({at.x+px(7), at.y+ImGui::GetTextLineHeight()/2}, px(4), forge::col(ImVec4(r, g, bl, 1)));
            if (row == 0) forge::noteItem("Lochliste1");
            if (ImGui::IsItemHovered()) hoverFromList = (long)i;
        }
        ImGui::EndChild(); ImGui::PopStyleColor();
        if (hoverFromList >= 0 && hoverFromList != holeHover_) { holeHover_ = hoverFromList; linesDirty_ = true; }
        forge::hint("Doppelklick springt zum Loch.");
        ImGui::BeginDisabled(hasPreview_);
        if (forge::toggleButton(IC_SPLINE "  Brücke bauen##b", bridging_, {-FLT_MIN, 0})) { bridging_ = !bridging_; bridgeHasFirst_ = false; }
        forge::noteItem("Brücke bauen");
        ImGui::EndDisabled();
        if (bridging_) forge::hint(bridgeHasFirst_ ? "Jetzt die zweite Randkante anklicken." : "Zwei Randkanten anklicken. Die Brücke teilt ein großes Loch oder verbindet zwei Löcher.");
    }
    footer([this] { previewFill(); }, [this] { holeLimit_ = 100; fillType_ = 1; std::fill(holeChosen_.begin(), holeChosen_.end(), 0); bridging_ = false; dropPreview(); linesDirty_ = true; });
}
void EditUi::rebuildHoleLines() {
    std::vector<LineVertex> lines;
    if (tool_ == EditTool::Holes && !analysisRunning_) {
        for (size_t i = 0; i < holes_.size(); i++) {
            if (i < holeFilled_.size() && holeFilled_[i] && hasPreview_) continue;
            const MeshHole& h = holes_[i]; float r, g, b; holeColor(h.perimeterMm, r, g, b);
            if (!holeManual_ && h.perimeterMm > holeLimit_) { r = g = b = 0.45f; }
            if (holeManual_ && i < holeChosen_.size() && holeChosen_[i]) { r = .2f; g = .92f; b = .92f; }
            if ((long)i == holeHover_) { r = g = b = 1; }
            for (size_t k = 0; k < h.loop.size(); k++) {
                lines.push_back({base_.vertices[h.loop[k]], r, g, b}); lines.push_back({base_.vertices[h.loop[(k+1)%h.loop.size()]], r, g, b});
            }
        }
    }
    view_.setLines(lines);
}
long EditUi::nearestRim(float mx, float my, size_t* segment, float limitPx) {
    view_.prepareDepth();
    long best = -1; float bestD = limitPx*limitPx; const float tol = view_.radius()*0.01f;
    for (size_t i = 0; i < holes_.size(); i++) {
        if (i < holeFilled_.size() && holeFilled_[i] && hasPreview_) continue;
        const auto& l = holes_[i].loop;
        float px = 0, py = 0, pd = 0; bool pv = false;
        for (size_t k = 0; k <= l.size(); k++) {
            float x, y, d; const bool ok = view_.project(base_.vertices[l[k%l.size()]], x, y, d) && view_.isVisible(x, y, d, tol);
            if (ok && pv) {
                const float vx = x-px, vy = y-py, wx = mx-px, wy = my-py, ln = vx*vx+vy*vy; const float t = ln > 0 ? std::clamp((vx*wx+vy*wy)/ln, 0.f, 1.f) : 0;
                const float dx = wx-t*vx, dy = wy-t*vy, dist = dx*dx+dy*dy;
                if (dist < bestD) { bestD = dist; best = (long)i; if (segment) *segment = k-1; }
            }
            px = x; py = y; pd = d; pv = ok;
        }
        (void)pd;
    }
    return best;
}
void EditUi::holesInput(float mx, float my, bool clicked) {
    if (analysisRunning_ || holes_.empty()) return;
    size_t seg = 0; const long h = nearestRim(mx, my, &seg, 9);
    if (h != holeHover_) { holeHover_ = h; linesDirty_ = true; }
    if (h >= 0) ImGui::SetTooltip("Umfang %.1f mm\nGröße %.1f mm\n%zu Randpunkte", holes_[h].perimeterMm, holes_[h].sizeMm, holes_[h].loop.size());
    if (!clicked || h < 0 || !holeManual_) return;
    if (bridging_) {
        const auto& l = holes_[h].loop; const uint32_t a = l[seg%l.size()], b = l[(seg+1)%l.size()];
        if (!bridgeHasFirst_) { bridgeHasFirst_ = true; bridgeA0_ = a; bridgeA1_ = b; bridgeP_ = (base_.vertices[a]+base_.vertices[b])*0.5f; return; }
        Mesh next = base_;
        if (bridgeEdges(next, bridgeA0_, bridgeA1_, a, b)) { history_.push(base_); base_ = std::move(next); meshChanged(false); }
        bridgeHasFirst_ = false; bridging_ = false;
        return;
    }
    holeChosen_[h] = !holeChosen_[h]; dropPreview(); linesDirty_ = true;
}

// ---- Selection -----------------------------------------------------------------------------------
void EditUi::finishDragSelect(ImVec2 imageMin, bool remove) {
    std::vector<ImVec2> poly;
    if (select_ == SelectMode::Box) {
        const ImVec2 a = dragStart_, b = ImGui::GetMousePos();
        poly = {{a.x-imageMin.x, a.y-imageMin.y}, {b.x-imageMin.x, a.y-imageMin.y}, {b.x-imageMin.x, b.y-imageMin.y}, {a.x-imageMin.x, b.y-imageMin.y}};
    } else for (const ImVec2& p : lasso_) poly.push_back({p.x-imageMin.x, p.y-imageMin.y});
    if (poly.size() < 3) return;
    float x0 = 1e9f, x1 = -1e9f, y0 = 1e9f, y1 = -1e9f; for (const ImVec2& p : poly) { x0 = std::min(x0, p.x); x1 = std::max(x1, p.x); y0 = std::min(y0, p.y); y1 = std::max(y1, p.y); }
    const bool see = !penetrate_; if (see) view_.prepareDepth();
    const float tol = view_.radius()*0.008f; const size_t nt = base_.triangleCount(); selected_.resize(nt, 0);
    #pragma omp parallel for schedule(dynamic, 8192)
    for (size_t t = 0; t < nt; t++) {
        const V3 c = (base_.vertices[base_.indices[3*t]]+base_.vertices[base_.indices[3*t+1]]+base_.vertices[base_.indices[3*t+2]])*(1.f/3);
        float x, y, d; if (!view_.project(c, x, y, d)) continue;
        if (x < x0 || x > x1 || y < y0 || y > y1 || !insidePolygon(x, y, poly)) continue;
        if (see && !view_.isVisible(x, y, d, tol)) continue;
        selected_[t] = !remove;
    }
    selectedCount_ = 0; for (uint8_t s : selected_) selectedCount_ += s;
    flagsDirty_ = true;
}
void EditUi::selectObject(int x, int y, bool remove) {
    const long t = view_.pickTriangle(x, y); if (t < 0 || (size_t)t >= base_.triangleCount()) return;
    if (!partsValid_) { triangleParts(base_, parts_); partsValid_ = true; }
    const uint32_t part = parts_[t]; selected_.resize(base_.triangleCount(), 0);
    for (size_t i = 0; i < parts_.size(); i++) if (parts_[i] == part) selected_[i] = !remove;
    selectedCount_ = 0; for (uint8_t s : selected_) selectedCount_ += s;
    flagsDirty_ = true;
}
void EditUi::deleteSelected() {
    if (!selectedCount_) return;
    history_.push(base_); deleteTriangles(base_, selected_); computeNormals(base_); meshChanged(false);
}
void EditUi::selectionBar(ImVec2 imageMin, ImVec2 imageSize) {
    const float b = px(36), gap = px(3), sep = px(13), barH = b+px(12);
    const float barW = 9*b+6*gap+2*sep+px(12)+px(2);
    forge::beginFloating("##bar", {imageMin.x+(imageSize.x-barW)/2, imageMin.y+imageSize.y-barH-px(16)}, {barW, barH});
    const bool idle = !jobRunning_ && !hasPreview_ && tool_ == EditTool::None;
    auto separator = [&] { ImGui::SameLine(0, sep/2); const ImVec2 c = ImGui::GetCursorScreenPos(); ImGui::GetWindowDrawList()->AddLine({c.x, c.y+px(8)}, {c.x, c.y+b-px(8)}, forge::col(forge::lineStrong)); ImGui::Dummy({0, 0}); ImGui::SameLine(0, sep/2); };
    ImGui::BeginDisabled(!idle);
    auto mode = [&](const char* icon, const char* key, SelectMode m, const char* tip) { if (forge::iconButton(icon, key, tip, select_ == m)) select_ = select_ == m ? SelectMode::None : m; ImGui::SameLine(); };
    mode(IC_LASSO, "Lasso", SelectMode::Lasso, "Lasso: Fläche frei umfahren, Strg wählt ab");
    mode(IC_SQUARE_DASHED, "Rechteck", SelectMode::Box, "Rechteck aufziehen, Strg wählt ab");
    mode(IC_MOUSE_POINTER_CLICK, "Objekt", SelectMode::Object, "Zusammenhängendes Teil anklicken, Strg wählt ab");
    if (forge::iconButton(IC_LAYERS, "Durchdringen", "Durchdringen: wählt auch verdeckte Rückseiten", penetrate_)) penetrate_ = !penetrate_;
    separator();
    if (forge::iconButton(IC_FLIP_HORIZONTAL_2, "Umkehren", "Auswahl umkehren")) { for (uint8_t& s : selected_) s = !s; selectedCount_ = selected_.size()-selectedCount_; flagsDirty_ = true; }
    ImGui::SameLine();
    ImGui::BeginDisabled(!selectedCount_);
    if (forge::iconButton(IC_CIRCLE_X, "Aufheben", "Auswahl aufheben")) { std::fill(selected_.begin(), selected_.end(), 0); selectedCount_ = 0; flagsDirty_ = true; }
    ImGui::SameLine();
    if (forge::iconButton(IC_TRASH_2, "Löschen", "Auswahl löschen (Entf)")) deleteSelected();
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    separator();
    ImGui::BeginDisabled(jobRunning_ || (!history_.canUndo() && !hasPreview_));
    if (forge::iconButton(IC_UNDO_2, "Rückgängig", "Rückgängig (Strg+Z)")) undo();
    ImGui::EndDisabled(); ImGui::SameLine();
    ImGui::BeginDisabled(jobRunning_ || !history_.canRedo() || hasPreview_);
    if (forge::iconButton(IC_REDO_2, "Wiederholen", "Wiederholen (Strg+Y)")) redo();
    ImGui::EndDisabled();
    forge::endFloating();
}

// ---- Cut plane -----------------------------------------------------------------------------------
CutPlane EditUi::cutPlane() const {
    CutPlane p = cutBase_;
    V3 u = normalized(cross(p.normal, std::fabs(p.normal.z) < .9f ? V3{0, 0, 1} : V3{1, 0, 0})), v = cross(p.normal, u);
    const V3 center = view_.center(); const V3 onPlane = center-p.normal*p.side(center);
    const float a = cutTiltA_*(float)M_PI/180, b = cutTiltB_*(float)M_PI/180;
    const V3 n = normalized(p.normal+u*std::tan(a)+v*std::tan(b));
    CutPlane out; out.normal = cutFlip_ ? n*-1.f : n; out.d = dot(out.normal, onPlane+p.normal*cutOffset_);
    return out;
}
void EditUi::startCut(const CutPlane& plane) {
    cutBase_ = plane; cutOffset_ = cutTiltA_ = cutTiltB_ = 0; cutFlip_ = false;
    // Keep the larger part: the marked side is the one with less of the model.
    size_t above = 0; const size_t stride = std::max<size_t>(1, base_.vertices.size()/50000), total = base_.vertices.size()/stride;
    for (size_t i = 0; i < base_.vertices.size(); i += stride) above += cutBase_.side(base_.vertices[i]) > 0;
    if (above < total/2) { cutBase_.normal = cutBase_.normal*-1.f; cutBase_.d = -cutBase_.d; }
    cutHave_ = true; cutMarksDirty_ = true;
}
void EditUi::cutPanel() {
    toolHeader("Schnittebene", "Rot wird entfernt. So lassen sich Unterlage und Reste abschneiden.");
    int mode = cutMode_; const char* const modes[] = {"Automatisch", "Punkte", "Waagerecht"};
    ImGui::BeginDisabled(jobRunning_);
    if (forge::segmented("cutmode", mode, modes, 3) || (!cutHave_ && cutPoints_.empty() && false)) {
        cutMode_ = mode; cutPoints_.clear(); cutHave_ = false; cutMarks_.clear(); flagsDirty_ = true;
        if (mode == 0) { CutPlane p; if (largestPlane(base_, p, std::max(0.3f, view_.radius()*0.004f))) startCut(p); }
        if (mode == 2) { CutPlane p; p.normal = {0, 0, 1}; p.d = lo_.z+(hi_.z-lo_.z)*0.05f; startCut(p); }
    }
    ImGui::EndDisabled();
    if (cutMode_ == 1 && !cutHave_) { char b[64]; snprintf(b, sizeof b, "Punkt %zu von 3 auf dem Modell anklicken", cutPoints_.size()+1); forge::hint(b); }
    if (cutMode_ != 1 && !cutHave_) forge::hint("Ebene oben wählen.");
    if (cutHave_) {
        bool ch = false;
        ch |= forge::slider("Ebene verschieben", &cutOffset_, -view_.radius(), view_.radius(), "%.1f mm");
        ch |= forge::slider("Drehen, Achse 1", &cutTiltA_, -45, 45, "%.1f °"); ch |= forge::slider("Drehen, Achse 2", &cutTiltB_, -45, 45, "%.1f °");
        if (forge::button(IC_REPEAT_2 "  Seite wechseln##c", {-FLT_MIN, 0})) { cutFlip_ = !cutFlip_; ch = true; }
        forge::noteItem("Seite wechseln");
        if (ch) cutMarksDirty_ = true;
        if (cutMarksDirty_) { markBelowPlane(base_, cutPlane(), cutMarks_); cutMarksDirty_ = false; flagsDirty_ = true; }
        size_t n = 0; for (uint8_t m : cutMarks_) n += m;
        forge::labelValue("Wird entfernt", (grouped(n)+" Dreiecke").c_str(), forge::danger);
        if (forge::primaryButton(IC_SCISSORS "  Anwenden##c", {-FLT_MIN, px(38)})) { history_.push(base_); deleteTriangles(base_, cutMarks_); computeNormals(base_); cutHave_ = false; cutPoints_.clear(); cutMarks_.clear(); meshChanged(false); }
        forge::noteItem("Anwenden");
    }
    if (forge::button("Abbrechen", {-FLT_MIN, 0})) { chooseTool(EditTool::Cut); }
}
bool EditUi::pickPoint(float mx, float my, V3& out, long* triOut) {
    const long t = view_.pickTriangle((int)mx, (int)my); const Mesh& m = shown();
    if (t < 0 || (size_t)t >= m.triangleCount()) return false;
    const V3 a = m.vertices[m.indices[3*t]], b = m.vertices[m.indices[3*t+1]], c = m.vertices[m.indices[3*t+2]];
    V3 o, d; view_.ray(mx, my, o, d);
    const V3 n = cross(b-a, c-a); const float den = dot(n, d);
    out = std::fabs(den) > 1e-9f ? o+d*(dot(n, a-o)/den) : (a+b+c)*(1.f/3);
    if (triOut) *triOut = t;
    return true;
}
void EditUi::cutInput(float mx, float my) {
    if (cutMode_ != 1 || cutHave_) return;
    V3 p; if (!pickPoint(mx, my, p)) return;
    cutPoints_.push_back(p);
    if (cutPoints_.size() == 3) {
        CutPlane pl;
        if (planeFromPoints(cutPoints_[0], cutPoints_[1], cutPoints_[2], pl)) startCut(pl);
        else { cutPoints_.clear(); forge::toast("Die drei Punkte liegen auf einer Linie.", forge::Toast::Warning); }
    }
}
void EditUi::drawCutPlane(ImVec2 o) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    for (const V3& p : cutPoints_) { float x, y, d; if (view_.project(p, x, y, d)) dl->AddCircleFilled({o.x+x, o.y+y}, 5, IM_COL32(50, 235, 235, 255)); }
    if (!cutHave_) return;
    const CutPlane p = cutPlane(); const V3 c = view_.center(), mid = c-p.normal*p.side(c);
    const V3 u = normalized(cross(p.normal, std::fabs(p.normal.z) < .9f ? V3{0, 0, 1} : V3{1, 0, 0})), v = cross(p.normal, u);
    const float s = view_.radius()*1.05f; ImVec2 q[4]; const float a[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    for (int i = 0; i < 4; i++) { float x, y, d; if (!view_.project(mid+u*(a[i][0]*s)+v*(a[i][1]*s), x, y, d)) return; q[i] = {o.x+x, o.y+y}; }
    dl->AddConvexPolyFilled(q, 4, IM_COL32(72, 190, 220, 46)); dl->AddPolyline(q, 4, IM_COL32(72, 190, 220, 200), ImDrawFlags_Closed, 1.5f);
}

// ---- Align ---------------------------------------------------------------------------------------
void EditUi::alignPanel() {
    toolHeader("Modell ausrichten");
    ImGui::BeginDisabled(jobRunning_);
    if (forge::button(IC_LAYERS "  Größte Fläche nach unten##a", {-FLT_MIN, 0})) {
        runPreview("Ausrichten", [](Mesh& m) {
            CutPlane p; if (!largestPlane(m, p, 0.5f)) return;
            // The object lies on the side with most of its points.
            V3 c{}; for (const V3& v : m.vertices) c = c+v; c = c*(1.f/std::max<size_t>(1, m.vertices.size()));
            orientUpright(m, p.side(c) >= 0 ? p.normal : p.normal*-1.f); computeNormals(m);
        });
    }
    forge::noteItem("Automatisch: größte Fläche nach unten");
    if (forge::toggleButton(pickFloor_ ? IC_CROSSHAIR "  Jetzt eine Fläche anklicken##f" : IC_CROSSHAIR "  Fläche im Bild als Boden##f", pickFloor_, {-FLT_MIN, 0})) pickFloor_ = !pickFloor_;
    forge::noteItem("Fläche im Bild als Boden wählen");
    ImGui::EndDisabled();
    auto rotate = [this] { const float x = rotX_, y = rotY_, z = rotZ_; runPreview("Drehen", [x, y, z](Mesh& m) { rotateMesh(m, x, y, z); orientUpright(m, V3{0, 0, 1}); computeNormals(m); }); };
    bool released = false;
    forge::slider("Drehen um X", &rotX_, -180, 180, "%.1f °"); released |= ImGui::IsItemDeactivated();
    forge::slider("Drehen um Y", &rotY_, -180, 180, "%.1f °"); released |= ImGui::IsItemDeactivated();
    forge::slider("Drehen um Z", &rotZ_, -180, 180, "%.1f °"); released |= ImGui::IsItemDeactivated();
    if (released && !jobRunning_) rotate();
    const float gap = ImGui::GetStyle().ItemSpacing.x, bw = (ImGui::GetContentRegionAvail().x-2*gap)/3;
    auto quarter = [&](const char* label, float& axis) { if (forge::button(label, {bw, 0})) { axis = axis+90 > 180 ? axis-270 : axis+90; rotate(); } };
    ImGui::BeginDisabled(jobRunning_);
    quarter("X +90", rotX_); ImGui::SameLine(); quarter("Y +90", rotY_); ImGui::SameLine(); quarter("Z +90", rotZ_);
    ImGui::EndDisabled();
    footer(rotate, [this] { rotX_ = rotY_ = rotZ_ = 0; pickFloor_ = false; dropPreview(); });
}

// ---- Measure -------------------------------------------------------------------------------------
void EditUi::measurePanel() {
    forge::beginCard("##measure", "Schnellmessung", IC_RULER);
    forge::hint("Zwei Punkte auf dem Modell anklicken, um ihren Abstand zu messen.");
    if (measurePoints_.size() == 2) {
        const V3 d = measurePoints_[1]-measurePoints_[0]; char b[48];
        ImGui::PushFont(forge::fontTitle); snprintf(b, sizeof b, "%.2f mm", norm(d)); ImGui::TextColored(forge::orange, "%s", b); ImGui::PopFont();
        snprintf(b, sizeof b, "%.2f mm", std::fabs(d.x)); forge::labelValue("X", b); snprintf(b, sizeof b, "%.2f mm", std::fabs(d.y)); forge::labelValue("Y", b); snprintf(b, sizeof b, "%.2f mm", std::fabs(d.z)); forge::labelValue("Z", b);
    }
    ImGui::BeginDisabled(measurePoints_.empty());
    if (forge::button("Messpunkte löschen", {-FLT_MIN, 0})) measurePoints_.clear();
    ImGui::EndDisabled();
    forge::endCard();
    if (haveSize_) {
        forge::beginCard("##size", "Modell", IC_CUBOID); char b[64];
        snprintf(b, sizeof b, "%.1f x %.1f x %.1f mm", tight_[0], tight_[1], tight_[2]); forge::labelValue("Größe", b);
        forge::labelValue("Dreiecke", grouped(base_.triangleCount()).c_str());
        if (holes_.empty() && !analysisRunning_ && !analysisDirty_) { snprintf(b, sizeof b, "%.2f cm³", std::fabs(volume_)/1000); forge::labelValue("Volumen", b); }
        forge::endCard();
    }
}
void EditUi::measureInput(float mx, float my) {
    V3 p; if (!pickPoint(mx, my, p)) return;
    if (measurePoints_.size() == 2) measurePoints_.clear();
    measurePoints_.push_back(p);
}
void EditUi::drawMeasure(ImVec2 o) {
    ImDrawList* dl = ImGui::GetWindowDrawList(); ImVec2 s[2]; int n = 0;
    for (const V3& p : measurePoints_) { float x, y, d; if (view_.project(p, x, y, d)) { s[n] = {o.x+x, o.y+y}; dl->AddCircleFilled(s[n], 5, IM_COL32(245, 158, 64, 255)); n++; } }
    if (n == 2) {
        dl->AddLine(s[0], s[1], IM_COL32(245, 158, 64, 255), 2); char b[32]; snprintf(b, sizeof b, "%.2f mm", norm(measurePoints_[1]-measurePoints_[0]));
        const ImVec2 m{(s[0].x+s[1].x)/2+8, (s[0].y+s[1].y)/2-8}, ts = ImGui::CalcTextSize(b);
        dl->AddRectFilled({m.x-3, m.y-2}, {m.x+ts.x+3, m.y+ts.y+2}, IM_COL32(3, 12, 24, 220), 3); dl->AddText(m, IM_COL32(245, 158, 64, 255), b);
    }
}

// ---- View ----------------------------------------------------------------------------------------
void EditUi::gizmo(ImVec2 o, ImVec2 size) {
    ImDrawList* dl = ImGui::GetWindowDrawList(); const float* R = view_.rotation(); const float len = px(24); const ImVec2 c{o.x+size.x-px(50), o.y+px(50)};
    dl->AddCircleFilled(c, px(38), IM_COL32(12, 16, 22, 170), 48); dl->AddCircle(c, px(38), forge::col(forge::line), 48);
    const struct { const char* name; V3 a; ImU32 col; } axes[3] = {{"X", {1, 0, 0}, IM_COL32(244, 96, 96, 255)}, {"Y", {0, 1, 0}, IM_COL32(120, 214, 130, 255)}, {"Z", {0, 0, 1}, IM_COL32(96, 165, 250, 255)}};
    // Axes pointing away are drawn first.
    int order[3] = {0, 1, 2}; float depth[3]; for (int i = 0; i < 3; i++) depth[i] = R[6]*axes[i].a.x+R[7]*axes[i].a.y+R[8]*axes[i].a.z;
    std::sort(order, order+3, [&](int a, int b) { return depth[a] < depth[b]; });
    ImGui::PushFont(forge::fontMedium);
    for (int k : order) {
        const auto& ax = axes[k]; const float x = R[0]*ax.a.x+R[1]*ax.a.y+R[2]*ax.a.z, y = R[3]*ax.a.x+R[4]*ax.a.y+R[5]*ax.a.z; const ImVec2 e{c.x+x*len, c.y-y*len};
        dl->AddLine(c, e, ax.col, px(2)); dl->AddCircleFilled(e, px(8), ax.col); const ImVec2 ts = ImGui::CalcTextSize(ax.name); dl->AddText(forge::fontMedium, ImGui::GetFontSize()*.78f, {e.x-ts.x*.39f, e.y-ts.y*.39f}, IM_COL32(10, 14, 20, 255), ax.name);
    }
    ImGui::PopFont();
}
void EditUi::viewport(bool measure) {
    const ImVec2 avail = ImGui::GetContentRegionAvail(); const int w = std::max(16, (int)avail.x-(int)px(10)), h = std::max(16, (int)avail.y-(int)px(10));
    const ImVec2 o = ImGui::GetCursorScreenPos(); const ImGuiIO& io = ImGui::GetIO(); ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 e{o.x+w, o.y+h};
    if (!hasMesh()) {
        const int v0 = dl->VtxBuffer.Size; dl->AddRectFilled(o, e, IM_COL32_WHITE, px(14)); ImGui::ShadeVertsLinearColorGradientKeepAlpha(dl, v0, dl->VtxBuffer.Size, o, {o.x, e.y}, IM_COL32(19, 24, 32, 255), IM_COL32(9, 12, 17, 255));
        ImGui::PushFont(forge::fontTitle); const char* t = "Noch kein Modell"; const ImVec2 ts = ImGui::CalcTextSize(t); dl->AddText({o.x+(w-ts.x)/2, o.y+h/2.f-px(14)}, forge::col(forge::text), t); ImGui::PopFont();
        const char* t2 = "Erstelle eines im Reiter Scan oder öffne links eine Datei."; const ImVec2 t2s = ImGui::CalcTextSize(t2); dl->AddText({o.x+(w-t2s.x)/2, o.y+h/2.f+px(12)}, forge::col(forge::textDim), t2);
        return;
    }
    const GLuint tex = view_.render(w, h);
    ImGui::SetCursorScreenPos(o); ImGui::InvisibleButton("##view3d", {(float)w, (float)h}, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
    dl->AddImageRounded((ImTextureID)(intptr_t)tex, o, e, {0, 1}, {1, 0}, IM_COL32_WHITE, px(14));
    const bool hoveredOnly = ImGui::IsItemHovered(); const bool hovered = hoveredOnly || ImGui::IsItemActive(); const float mx = io.MousePos.x-o.x, my = io.MousePos.y-o.y;
    const bool toolBusy = jobRunning_ && !analysisRunning_;
    const bool picksWithLeft = measure || select_ != SelectMode::None || (tool_ == EditTool::Holes && holeManual_) || (tool_ == EditTool::Cut && cutMode_ == 1 && !cutHave_) || (tool_ == EditTool::Align && pickFloor_);
    const bool leftTurns = !picksWithLeft || io.KeyShift;
    if (hovered) {
        if (ImGui::IsMouseDown(ImGuiMouseButton_Middle) || (leftTurns && ImGui::IsMouseDown(ImGuiMouseButton_Left) && !dragging_)) view_.rotate(io.MouseDelta.x, io.MouseDelta.y);
        if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) view_.pan(io.MouseDelta.x, io.MouseDelta.y);
    }
    if (hoveredOnly) {
        if (io.MouseWheel != 0) view_.zoom(io.MouseWheel);
        const bool clicked = !io.KeyShift && ImGui::IsMouseClicked(ImGuiMouseButton_Left);
        if (!toolBusy) {
            if (measure) { if (clicked) measureInput(mx, my); }
            else if (tool_ == EditTool::Holes) holesInput(mx, my, clicked);
            else if (tool_ == EditTool::Cut) { if (clicked) cutInput(mx, my); }
            else if (tool_ == EditTool::Align && pickFloor_ && clicked) {
                long t = -1; V3 p;
                if (pickPoint(mx, my, p, &t)) {
                    pickFloor_ = false; const V3 a = base_.vertices[base_.indices[3*t]], b = base_.vertices[base_.indices[3*t+1]], c = base_.vertices[base_.indices[3*t+2]]; V3 n = normalized(cross(b-a, c-a));
                    // Averaged over the neighbourhood, a single small triangle is too noisy.
                    V3 sum{}; const float r2 = std::pow(view_.radius()*0.04f, 2.f);
                    for (size_t i = 0; i < base_.indices.size(); i += 3) { const V3 g = base_.vertices[base_.indices[i]]-p; if (dot(g, g) < r2) { const V3 fn = cross(base_.vertices[base_.indices[i+1]]-base_.vertices[base_.indices[i]], base_.vertices[base_.indices[i+2]]-base_.vertices[base_.indices[i]]); if (dot(fn, n) > 0) sum = sum+fn; } }
                    if (norm(sum) > 0) n = normalized(sum);
                    runPreview("Ausrichten", [n](Mesh& m) { orientUpright(m, n*-1.f); computeNormals(m); });
                }
            } else if (select_ == SelectMode::Object && clicked) selectObject((int)mx, (int)my, io.KeyCtrl);
            else if ((select_ == SelectMode::Lasso || select_ == SelectMode::Box) && clicked) { dragging_ = true; dragStart_ = io.MousePos; lasso_.assign(1, io.MousePos); }
        }
    }
    if (dragging_) {
        if (select_ == SelectMode::Lasso && !lasso_.empty()) { const ImVec2 l = lasso_.back(), m = io.MousePos; if ((l.x-m.x)*(l.x-m.x)+(l.y-m.y)*(l.y-m.y) > 9) lasso_.push_back(m); }
        if (select_ == SelectMode::Box) { dl->AddRectFilled(dragStart_, io.MousePos, forge::col(forge::sky, .15f)); dl->AddRect(dragStart_, io.MousePos, forge::col(forge::sky)); }
        else if (lasso_.size() > 1) dl->AddPolyline(lasso_.data(), (int)lasso_.size(), forge::col(forge::sky), ImDrawFlags_Closed, 1.5f);
        if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) { dragging_ = false; finishDragSelect(o, io.KeyCtrl); lasso_.clear(); }
    }
    if (tool_ == EditTool::Cut) drawCutPlane(o);
    if (measure) drawMeasure(o);
    if (bridgeHasFirst_) { float x, y, d; if (view_.project(bridgeP_, x, y, d)) dl->AddCircleFilled({o.x+x, o.y+y}, 6, forge::col(forge::sky)); }
    gizmo(o, {(float)w, (float)h});
    // What the model is: chips, top left.
    ImVec2 hud{o.x+px(12), o.y+px(12)}; const float line = px(34); char b[96];
    snprintf(b, sizeof b, "%s Dreiecke", grouped(shown().triangleCount()).c_str()); forge::chipAt(dl, hud, b, forge::textDim, IC_TRIANGLE); hud.y += line;
    if (analysisRunning_ || analysisDirty_) forge::chipAt(dl, hud, "Netz wird geprüft", forge::textDim, IC_ACTIVITY);
    else if (holes_.empty()) forge::chipAt(dl, hud, "Wasserdicht", forge::green, IC_CIRCLE_CHECK);
    else { snprintf(b, sizeof b, "%s, %s offene Kanten", holesText(holes_.size()).c_str(), grouped(openEdges_).c_str()); forge::chipAt(dl, hud, b, forge::amber, IC_CIRCLE_DASHED); }
    hud.y += line;
    snprintf(b, sizeof b, "%.1f x %.1f x %.1f mm", tight_[0], tight_[1], tight_[2]); forge::chipAt(dl, hud, b, forge::textDim, IC_CUBOID); hud.y += line;
    if (hasPreview_) { forge::chipAt(dl, hud, "Vorschau, noch nicht übernommen", forge::orange, IC_EYE); hud.y += line; }
    if (selectedCount_) { snprintf(b, sizeof b, "%s ausgewählt", grouped(selectedCount_).c_str()); forge::chipAt(dl, hud, b, forge::danger, IC_LASSO); hud.y += line; }
    if (!measure) selectionBar(o, {(float)w, (float)h});
    { const char* tip = picksWithLeft ? "Links wählen  ·  Umschalt drehen  ·  Rechts bewegen" : "Links drehen  ·  Rechts bewegen  ·  Rad zoomen"; const float fs = ImGui::GetFontSize()*.88f; const float tw = forge::fontBody->CalcTextSizeA(fs, FLT_MAX, 0, tip).x; if (tw+px(28) < (w-px(380))/2) dl->AddText(forge::fontBody, fs, {o.x+px(14), e.y-px(26)}, forge::col(forge::textFaint), tip); }
    if (toolBusy) { std::string label = busyLabel(); forge::busyOverlay(o, {(float)w, (float)h}, label.c_str(), jobProgress_); }
}

// ---- One click -----------------------------------------------------------------------------------
void EditUi::openOneClick() {
    if (source_.prepare) source_.prepare();
    useScanForOneClick_ = source_.available();
    if (useScanForOneClick_) { const float v = source_.voxelMm(); for (int i = 0; i < 4; i++) if (std::fabs(kVoxels[i]-v) < 1e-3f) oneClickVoxel_ = i; }
    oneClickRequest_ = true;
}
void EditUi::oneClickDialog() {
    const ImVec2 c = ImGui::GetMainViewport()->GetCenter(); ImGui::SetNextWindowPos(c, ImGuiCond_Appearing, {.5f, .5f}); ImGui::SetNextWindowSize({px(560), 0});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {px(20), px(18)}); ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, px(16)); ImGui::PushStyleColor(ImGuiCol_PopupBg, forge::surface); ImGui::PushStyleColor(ImGuiCol_TitleBgActive, forge::surface);
    const bool open = ImGui::BeginPopupModal("Ein-Klick-Prozess", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);
    if (!open) { ImGui::PopStyleVar(2); ImGui::PopStyleColor(2); return; }
    ImGui::PushFont(forge::fontTitle); ImGui::PushStyleColor(ImGuiCol_Text, forge::orange); ImGui::TextUnformatted(IC_SPARKLES); ImGui::PopStyleColor(); ImGui::SameLine(0, px(8)); ImGui::TextUnformatted("Ein-Klick-Prozess"); ImGui::PopFont();
    forge::hint("Führt alle Schritte nacheinander aus. Danach lässt sich jedes Ergebnis mit den Werkzeugen weiter bearbeiten oder rückgängig machen.");
    OneClickConfig& k = oneClick_;
    const float colW = (ImGui::GetContentRegionAvail().x-px(12))/2;
    ImGui::BeginGroup(); ImGui::PushItemWidth(colW); ImGui::BeginChild("##oc1", {colW, 0}, ImGuiChildFlags_AutoResizeY);
    if (useScanForOneClick_) {
        forge::beginCard("##ocf", "Fusion");
        { const float prev = oneClickVoxel_; (void)prev; ImGui::PushStyleColor(ImGuiCol_Text, forge::textDim); ImGui::TextUnformatted("Auflösung"); ImGui::PopStyleColor(); ImGui::SetNextItemWidth(-1);
          const bool cb = ImGui::BeginCombo("##vox", kVoxelNames[oneClickVoxel_]); forge::noteItem("Fusion-Auflösung");
          if (cb) { for (int i = 0; i < 4; i++) { if (ImGui::Selectable(kVoxelNames[i], oneClickVoxel_ == i)) oneClickVoxel_ = i; forge::noteItem(kVoxelNames[i]); } ImGui::EndCombo(); } }
        forge::sliderInt("Rauschunterdrückung", &oneClickNoise_, 0, 100, "%d %%");
        forge::endCard();
    }
    forge::beginCard("##ocn", "Netz");
    { float pct = k.minPartShare*100; char fmt[32]; snprintf(fmt, sizeof fmt, pct <= 0 ? "behalten" : "unter %.1f %%%%", pct); if (forge::slider("Kleine Teile", &pct, 0, 50, fmt)) k.minPartShare = pct/100; }
    auto levels = [&](const char* label, const char* id, Level& l) { int v = (int)l; ImGui::PushStyleColor(ImGuiCol_Text, forge::textDim); ImGui::TextUnformatted(label); ImGui::PopStyleColor(); if (forge::segmented(id, v, kLevels, 4)) l = (Level)v; };
    levels("Spitzen", "osp", k.spikes); levels("Glätten", "osm", k.smooth); forge::toggle("Scharfe Kanten erhalten", &k.keepEdges); levels("Ränder glätten", "ori", k.rimSmooth);
    forge::endCard();
    ImGui::EndChild(); ImGui::PopItemWidth(); ImGui::EndGroup();
    ImGui::SameLine(0, px(12));
    ImGui::BeginChild("##oc2", {colW, 0}, ImGuiChildFlags_AutoResizeY);
    forge::beginCard("##och", "Löcher");
    forge::toggle("Löcher füllen", &k.fillHoles);
    ImGui::BeginDisabled(!k.fillHoles);
    { int v = (int)k.fillType; ImGui::PushStyleColor(ImGuiCol_Text, forge::textDim); ImGui::TextUnformatted("Typ"); ImGui::PopStyleColor(); if (forge::segmented("oft", v, kFillTypes, 3)) k.fillType = (FillType)v; }
    bool all = k.maxHolePerimeterMm < 0; if (forge::toggle("Alle, wasserdicht", &all)) k.maxHolePerimeterMm = all ? -1 : 100;
    if (!all) forge::slider("Lochumfang bis", &k.maxHolePerimeterMm, 1, 2000, "%.0f mm", true);
    ImGui::EndDisabled();
    forge::endCard();
    forge::beginCard("##ocs", "Vereinfachen");
    { int keep = (int)std::round(k.keepTriangles*100); if (forge::sliderInt("Dreiecke behalten", &keep, 1, 100, "%d %%")) k.keepTriangles = keep/100.f; }
    forge::endCard();
    ImGui::EndChild();
    ImGui::Dummy({0, px(4)});
    const float gap = ImGui::GetStyle().ItemSpacing.x, w = (ImGui::GetContentRegionAvail().x-2*gap)/3, h = px(38);
    if (forge::button("Standard", {w, h})) { oneClick_ = OneClickConfig{}; oneClickNoise_ = 15; }
    ImGui::SameLine(); if (forge::button("Abbrechen##oc", {w, h})) ImGui::CloseCurrentPopup();
    ImGui::SameLine(); if (forge::primaryButton(IC_PLAY "  Start##oc", {w, h})) { saveOneClickConfig(oneClick_, configPath()); ImGui::CloseCurrentPopup(); startOneClick(); }
    forge::noteItem("Start");
    ImGui::EndPopup(); ImGui::PopStyleVar(2); ImGui::PopStyleColor(2);
}
void EditUi::startOneClick() {
    dropPreview(); tool_ = EditTool::None; select_ = SelectMode::None;
    const bool fromScan = useScanForOneClick_; const float voxel = kVoxels[oneClickVoxel_]; const int minWeight = 1+(oneClickNoise_*7+50)/100; const OneClickConfig cfg = oneClick_;
    auto result = std::make_shared<Mesh>(); auto report = std::make_shared<OneClickReport>(); auto error = std::make_shared<std::string>();
    if (!fromScan) *result = base_;
    const bool hadMesh = hasMesh();
    startJob("Ein-Klick-Prozess", [=, this] {
        if (fromScan) { setProgress("Netz wird berechnet", 0); if (!source_.build(voxel, minWeight, true, *result, *error)) return; }
        *report = runOneClick(*result, cfg, [this](const char* step, float f) { setProgress(step, f); });
    }, [=, this] {
        if (!error->empty() || result->indices.empty()) { const std::string text = error->empty() ? "Das Netz ist leer." : *error; if (onError) onError(text); return; }
        if (hadMesh) {
            history_.push(base_); base_ = std::move(*result);
            if (fromScan) { voxelMm_ = voxel; updateOutPath(voxel); }
            meshChanged(false);
        } else newMesh(std::move(*result), voxel);
        char b[240]; snprintf(b, sizeof b, "Ein-Klick-Prozess fertig: %d Teile entfernt, %s gefüllt, %d offen, %s Dreiecke.", report->partsRemoved, holesText(report->holesClosed).c_str(), report->holesLeft, grouped(report->trianglesAfter).c_str());
        oneClickReport_ = b; forge::toast(b, forge::Toast::Success);
    });
}
