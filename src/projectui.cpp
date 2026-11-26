#include <imgui.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include "measure.hpp"
#include "i18n.hpp"
#include "projectui.hpp"
#include "filedialog.hpp"
#include "settings.hpp"
#include "theme.hpp"

using forge::px;

namespace {
const float kVoxels[] = {0.15f, 0.25f, 0.5f, 1.f};
const char* const kVoxelNames[] = {"0,15 mm", "0,25 mm", "0,5 mm", "1 mm"};
// Each scan gets its own color while aligning, so that a wrong placement is easy to see.
const float kScanColors[6][3] = {{.25f, .62f, .95f}, {.98f, .58f, .16f}, {.3f, .85f, .45f}, {.9f, .35f, .75f}, {.95f, .85f, .25f}, {.55f, .45f, .95f}};
void appendMesh(Mesh& into, const Mesh& m, const Pose& P, const float* col) {
    const uint32_t off = (uint32_t)into.vertices.size();
    for (const V3& v : m.vertices) { into.vertices.push_back(P.apply(v)); for (int c = 0; c < 3; c++) into.colors.push_back((uint8_t)(col[c]*170)); }
    for (uint32_t i : m.indices) into.indices.push_back(i+off);
}
bool insidePolygon(float x, float y, const std::vector<ImVec2>& poly) {
    bool in = false;
    for (size_t i = 0, j = poly.size()-1; i < poly.size(); j = i++)
        if ((poly[i].y > y) != (poly[j].y > y) && x < (poly[j].x-poly[i].x)*(y-poly[i].y)/(poly[j].y-poly[i].y)+poly[i].x) in = !in;
    return in;
}
}

ProjectUi::ProjectUi(Pipeline& pipe, EditUi& edit) : pipe_(pipe), edit_(edit) {}
ProjectUi::~ProjectUi() { if (job_.joinable()) job_.join(); }
void ProjectUi::say(const std::string& text, bool ok) { message = text; forge::toast(text, ok ? forge::Toast::Success : forge::Toast::Error); }
void ProjectUi::init() { view_.init(); viewB_.init(); viewC_.init(); reviewView_.init(); }
void ProjectUi::startJob(const std::string& label, std::function<void()> work, std::function<void()> finish) {
    if (jobRunning_) return;
    if (job_.joinable()) job_.join();
    { std::lock_guard<std::mutex> g(labelMutex_); jobLabel_ = label; } jobProgress_ = -1;
    jobFinish_ = std::move(finish); jobRunning_ = true; jobDone_ = false;
    job_ = std::thread([this, work = std::move(work)] { work(); jobDone_ = true; });
}
void ProjectUi::frame() {
    if (jobDone_.exchange(false)) {
        if (job_.joinable()) job_.join();
        jobRunning_ = false;
        auto f = std::move(jobFinish_); jobFinish_ = nullptr; if (f) f();
    }
    if (fusionRequest_) {
        fusionRequest_ = false;
        const float v = pipe_.options.voxelMm; for (int i = 0; i < 4; i++) if (std::fabs(kVoxels[i]-v) < 1e-3f) fusionVoxel_ = i;
        ImGui::OpenPopup("Fusion-Batch-Prozess");
    }
    if (!jobRunning_) reportSupports();   // also after the one-click process, which runs outside these jobs
    updateReview();
    fusionDialog();
    if (removeRequest_) { removeRequest_ = false; ImGui::OpenPopup("Scan entfernen"); }
    removeDialog();
    renameDialog();
}
std::string ProjectUi::describe() const {
    std::string s = "Scans=" + std::to_string(project_.scans.size());
    for (const ScanData& d : project_.scans) s += " " + d.name + ":" + std::to_string(d.frames.size()) + (d.placed ? ":platziert" : ":offen");
    s += " Ansicht=" + (reviewing() ? project_.scans[reviewScan_].name : std::string("live")) + " Ausrichten=" + std::to_string(aligning_) + " Ergebnis=" + std::to_string(hasResult_);
    if (reviewing() && project_.scans[reviewScan_].cloud) s += " Wolke=" + std::to_string(project_.scans[reviewScan_].cloud->vertices.size()) + " markiert=" + std::to_string(cloudMarkedCount_) + " Schnitte=" + std::to_string(cuts_.size());
    for (auto& e : proposals_) if (!e.second.options.empty()) {
        const RegisterResult& r = e.second.options[e.second.current]; char b[120];
        snprintf(b, sizeof b, " [%s: %zu Vorschläge, %.0f%%, Rest %.3f, %s]", project_.scans[e.first].name.c_str(), e.second.options.size(), 100*r.overlap, r.rmsMm, r.confident ? "sicher" : "unsicher"); s += b;
    }
    if (!message.empty()) s += " Meldung: " + message;
    return s;
}
bool ProjectUi::commitLive() {
    ScanData s; s.name = project_.nextName();
    if (!pipe_.takeScan(s)) return false;
    s.placed = project_.scans.empty();   // the first scan defines the coordinates
    project_.scans.push_back(std::move(s)); dirty_ = true; reviewLast();
    return true;
}
void ProjectUi::panel(bool canAct, int liveFrames, bool canStartTools, bool hasLive) {
    { const std::string title = hasProject() ? projectName() : std::string(tr("Projekt")); forge::beginCard("##proj", title.c_str(), IC_LAYERS); }
    const bool free = canStartTools && !jobRunning_;
    if (!project_.scans.empty()) {
        for (size_t i = 0; i < project_.scans.size(); i++) {
            ScanData& s = project_.scans[i]; ImGui::PushID((int)i); const ImVec2 at = ImGui::GetCursorScreenPos(); const float w = ImGui::GetContentRegionAvail().x, h = px(44);
            ImGui::SetNextItemAllowOverlap(); ImGui::BeginDisabled(!free);
            if (ImGui::InvisibleButton("##row", {w-px(70), h})) showReview((int)i);
            const bool rowHover = ImGui::IsItemHovered(); { char note[160]; snprintf(note, sizeof note, "Zeile %s", s.name.c_str()); forge::noteItem(note); } ImGui::EndDisabled();
            if (rowHover) ImGui::SetTooltip("%s", tr("Punktwolke dieses Scans ansehen und bearbeiten. Rechte Maustaste: weitere Befehle"));
            if (free && ImGui::IsItemClicked(ImGuiMouseButton_Right)) { menuIndex_ = (int)i; ImGui::OpenPopup("##scanmenu"); }
            ImDrawList* dl = ImGui::GetWindowDrawList(); dl->AddRectFilled(at, {at.x+w, at.y+h}, forge::col(rowHover ? forge::raised : forge::bg), px(9));
            if (reviewScan_ == (int)i) dl->AddRect(at, {at.x+w, at.y+h}, forge::col(forge::sky), px(9), 0, px(1.5f));
            const float* colr = kScanColors[i%6]; dl->AddCircleFilled({at.x+px(14), at.y+h/2}, px(4.5f), forge::col(ImVec4(colr[0], colr[1], colr[2], s.included ? 1.f : .3f)));
            ImGui::PushFont(forge::fontMedium); dl->AddText({at.x+px(28), at.y+px(6)}, forge::col(s.included ? forge::text : forge::textDim), s.name.c_str()); ImGui::PopFont();
            char sub[160]; snprintf(sub, sizeof sub, tr("%zu Bilder  ·  %s"), s.frames.size(), s.placed ? tr("ausgerichtet") : tr("offen")); dl->AddText(forge::fontBody, ImGui::GetFontSize()*.88f, {at.x+px(28), at.y+px(24)}, forge::col(s.placed ? forge::textDim : forge::amber), sub);
            ImGui::SetCursorScreenPos({at.x+w-px(66), at.y+px(7)}); ImGui::BeginDisabled(!free);
            char key[32]; snprintf(key, sizeof key, "Sichtbar##%zu", i); if (forge::iconButton(s.included ? IC_EYE : IC_EYE_OFF, key, s.included ? tr("Gehört zum Modell") : tr("Gehört nicht zum Modell"), false, px(30))) s.included = !s.included;
            ImGui::SameLine(0, px(2)); snprintf(key, sizeof key, "X##%zu", i); if (forge::iconButton(IC_TRASH_2, key, tr("Scan entfernen"), false, px(30))) { removeIndex_ = (int)i; removeRequest_ = true; }
            ImGui::EndDisabled();
            ImGui::SetCursorScreenPos({at.x, at.y+h+px(5)}); ImGui::Dummy({0, 0});
            scanMenu(i);
            ImGui::PopID();
        }
    } else if (!liveFrames) forge::hint(tr("Mehrere Scans aus verschiedenen Lagen werden hier gesammelt und zu einem Modell zusammengeführt."));
    if (liveFrames) {
        char b[160]; snprintf(b, sizeof b, tr(IC_SCAN_LINE "  Aktueller Scan: %d Bilder"), liveFrames);
        if (forge::toggleButton(b, !reviewing(), {-FLT_MIN, 0})) { reviewScan_ = -1; reviewDirty_ = false; }
        forge::noteItem("Aktueller Scan");
    }
    ImGui::BeginDisabled(!free);
    const float gap = ImGui::GetStyle().ItemSpacing.x, w3 = (ImGui::GetContentRegionAvail().x-2*gap)/3, hb = px(60);
    const size_t total = project_.scans.size()+(hasLive ? 1 : 0);
    ImGui::BeginDisabled(!hasLive && !reviewing());   // already at the live image, with nothing to keep
    if (forge::toolTile(IC_PLUS, tr("Neuer Scan"), {w3, hb})) newScan(hasLive);
    forge::noteItem("Neuer Scan Karte");
    ImGui::EndDisabled(); ImGui::SameLine(0, gap);
    ImGui::BeginDisabled(total < 2);
    if (forge::toolTile(IC_MOVE_3D, tr("Ausrichten"), {w3, hb})) { if (!hasLive || commitLive()) openAlign(); }
    ImGui::EndDisabled(); ImGui::SameLine(0, gap);
    ImGui::BeginDisabled(total < 1 || (project_.scans.empty() && !hasLive));
    if (forge::toolTile(IC_COMBINE, tr("Fusion"), {w3, hb})) { if (hasLive) commitLive(); openFusion(); }
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    ImGui::BeginDisabled(!canAct || jobRunning_);
    { const float w = (ImGui::GetContentRegionAvail().x-ImGui::GetStyle().ItemSpacing.x)/2;
      if (forge::button(IC_SAVE "  Speichern##proj", {w, 0})) { if (liveFrames) commitLive(); if (project_.scans.empty()) say(tr("Noch kein Scan im Projekt."), false); else requestSave(); }
      forge::noteItem("Speichern##proj"); ImGui::SameLine();
      if (forge::button(IC_FOLDER_OPEN "  Öffnen##proj", {w, 0})) requestOpen();
      forge::noteItem("Öffnen##proj"); }
    ImGui::EndDisabled();
    forge::endCard();
}
void ProjectUi::newScan(bool hasLive) {
    if (hasLive) {
        if (commitLive()) say(trf("Gespeichert als %s. Objekt umlegen und den nächsten Scan starten.", project_.scans.back().name), true);
        else say(tr("Dieser Scan war zu lang, um ihn im Speicher zu behalten."), false);
    }
    reviewScan_ = -1; reviewDirty_ = false;
}
void ProjectUi::reviewLast() { reviewScan_ = (int)project_.scans.size()-1; reviewDirty_ = reviewScan_ >= 0; }
void ProjectUi::showReview(int scan) { reviewScan_ = scan; reviewDirty_ = true; }
// The picture is prepared in the background, unless another job has to finish first.
void ProjectUi::updateReview() {
    if (!reviewDirty_ || jobRunning_ || aligning_) return;
    reviewDirty_ = false; clearCloudMarks(); cloudLasso_.clear(); cloudDragging_ = false;
    if (reviewScan_ < 0 || reviewScan_ >= (int)project_.scans.size()) { reviewScan_ = -1; return; }
    ScanData* s = &project_.scans[reviewScan_]; const int shown = reviewScan_;
    if (s->cloud) uploadReview(true);
    else startJob(tr("Punktwolke wird vorbereitet"), [this, s] { if (removeSupport(*s)) { std::lock_guard<std::mutex> g(labelMutex_); supportCut_ = s->name; } buildPointCloud(*s); }, [this, shown] { reportSupports(); if (shown == reviewScan_) uploadReview(true); });
}
void ProjectUi::uploadReview(bool resetCamera) {
    if (reviewScan_ < 0 || reviewScan_ >= (int)project_.scans.size()) return;
    const ScanData& d = project_.scans[reviewScan_]; if (!d.cloud) return;
    reviewView_.setUpDirection(d.up); reviewView_.setPointCloud(*d.cloud, resetCamera, d.cloudSpacingMm);
    clearCloudMarks();
}
void ProjectUi::reviewView() {
    const ImVec2 o = ImGui::GetCursorScreenPos(), r{ImGui::GetContentRegionAvail().x-px(10), ImGui::GetContentRegionAvail().y-px(10)};
    char title[120] = ""; if (reviewScan_ < (int)project_.scans.size() && reviewScan_ >= 0) { const ScanData& s = project_.scans[reviewScan_]; snprintf(title, sizeof title, tr("%s  ·  %zu Bilder"), s.name.c_str(), s.frames.size()); }
    const bool selecting = cloudSelect_ != CloudSelect::None && !ImGui::GetIO().KeyShift;
    oneView(reviewView_, r, title, nullptr, nullptr, !selecting && !cloudDragging_);
    cloudInput(o, ImGui::IsItemHovered());
    if (cloudMarkedCount_) { char b[96]; snprintf(b, sizeof b, tr("%zu Punkte ausgewählt"), cloudMarkedCount_); forge::chipAt(ImGui::GetWindowDrawList(), {o.x+px(12), o.y+px(46)}, b, forge::danger); }
    else if (cloudSelect_ != CloudSelect::None) forge::chipAt(ImGui::GetWindowDrawList(), {o.x+px(12), o.y+px(46)}, tr("Ziehen wählt aus, Strg wählt ab, Umschalt dreht"), forge::textDim);
    cloudBar(o, r);
    if (jobRunning_) forge::busyOverlay(o, r, busyLabel().c_str(), busyProgress());
}
void ProjectUi::clearCloudMarks() { cloudMarked_.clear(); cloudMarkedCount_ = 0; reviewView_.setFlags({}); }
void ProjectUi::cloudInput(ImVec2 o, bool hovered) {
    const ImGuiIO& io = ImGui::GetIO(); ImDrawList* dl = ImGui::GetWindowDrawList();
    if (!jobRunning_ && !io.WantTextInput && reviewing()) {
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) { if (cloudDragging_) { cloudDragging_ = false; cloudLasso_.clear(); } else if (cloudMarkedCount_) clearCloudMarks(); else cloudSelect_ = CloudSelect::None; }
        if (ImGui::IsKeyPressed(ImGuiKey_Delete, false)) deleteCloudMarked();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z, false)) undoCloudCut();
    }
    if (hovered && !jobRunning_ && cloudSelect_ != CloudSelect::None && !io.KeyShift && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) { cloudDragging_ = true; cloudDragStart_ = io.MousePos; cloudLasso_.assign(1, io.MousePos); }
    if (!cloudDragging_) return;
    if (cloudSelect_ == CloudSelect::Lasso) { const ImVec2 l = cloudLasso_.back(), m = io.MousePos; if ((l.x-m.x)*(l.x-m.x)+(l.y-m.y)*(l.y-m.y) > 9) cloudLasso_.push_back(m); }
    if (cloudSelect_ == CloudSelect::Box) { dl->AddRectFilled(cloudDragStart_, io.MousePos, forge::col(forge::sky, .15f)); dl->AddRect(cloudDragStart_, io.MousePos, forge::col(forge::sky)); }
    else if (cloudLasso_.size() > 1) dl->AddPolyline(cloudLasso_.data(), (int)cloudLasso_.size(), forge::col(forge::sky), ImDrawFlags_Closed, 1.5f);
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) { cloudDragging_ = false; finishCloudSelect(o, io.KeyCtrl); cloudLasso_.clear(); }
}
void ProjectUi::finishCloudSelect(ImVec2 o, bool remove) {
    if (reviewScan_ < 0 || reviewScan_ >= (int)project_.scans.size() || !project_.scans[reviewScan_].cloud) return;
    std::vector<ImVec2> poly;
    if (cloudSelect_ == CloudSelect::Box) { const ImVec2 a = cloudDragStart_, b = ImGui::GetMousePos(); poly = {{a.x-o.x, a.y-o.y}, {b.x-o.x, a.y-o.y}, {b.x-o.x, b.y-o.y}, {a.x-o.x, b.y-o.y}}; }
    else for (const ImVec2& p : cloudLasso_) poly.push_back({p.x-o.x, p.y-o.y});
    if (poly.size() < 3) return;
    float x0 = 1e9f, x1 = -1e9f, y0 = 1e9f, y1 = -1e9f; for (const ImVec2& p : poly) { x0 = std::min(x0, p.x); x1 = std::max(x1, p.x); y0 = std::min(y0, p.y); y1 = std::max(y1, p.y); }
    const Mesh& c = *project_.scans[reviewScan_].cloud; const bool see = !cloudPenetrate_; if (see) reviewView_.prepareDepth();
    // Points are drawn as small squares: what lies within a few of them behind the front counts as seen. A rough or
    // furry surface gives a layer of points several mm thick: deeper points count as well while they face the viewer,
    // so that the back of a thin wall stays.
    const float spacing = project_.scans[reviewScan_].cloudSpacingMm, tol = std::max(reviewView_.radius()*0.008f, 3*spacing), deep = std::max(reviewView_.radius()*0.05f, 12*spacing);
    const bool normals = c.normals.size() == c.vertices.size(); V3 eye, dir; reviewView_.ray((x0+x1)/2, (y0+y1)/2, eye, dir);
    cloudMarked_.resize(c.vertices.size(), 0);
    #pragma omp parallel for schedule(dynamic, 16384)
    for (size_t i = 0; i < c.vertices.size(); i++) {
        float x, y, d; if (!reviewView_.project(c.vertices[i], x, y, d)) continue;
        if (x < x0 || x > x1 || y < y0 || y > y1 || !insidePolygon(x, y, poly)) continue;
        if (see && !reviewView_.isVisible(x, y, d, tol) && !(normals && dot(c.normals[i], c.vertices[i]-eye) < 0 && reviewView_.isVisible(x, y, d, deep))) continue;
        cloudMarked_[i] = !remove;
    }
    cloudMarkedCount_ = 0; for (uint8_t m : cloudMarked_) cloudMarkedCount_ += m;
    if (cloudMarkedCount_) reviewView_.setFlags(cloudMarked_); else reviewView_.setFlags({});
}
void ProjectUi::deleteCloudMarked() {
    if (!cloudMarkedCount_ || jobRunning_ || reviewScan_ < 0 || reviewScan_ >= (int)project_.scans.size()) return;
    const int scan = reviewScan_; auto marked = std::make_shared<std::vector<uint8_t>>(std::move(cloudMarked_)); clearCloudMarks();
    auto step = std::make_shared<CutStep>(); step->scan = scan;
    startJob(tr("Punkte werden aus dem Scan entfernt"), [this, scan, marked, step] { step->cut = cutCloudPoints(project_.scans[scan], *marked); }, [this, scan, step] {
        if (!step->cut.measurements && !step->cut.cloud) return;
        cuts_.push_back(std::move(*step)); if (cuts_.size() > 20) cuts_.erase(cuts_.begin());
        dirty_ = true; if (scan == reviewScan_) uploadReview(false);
        say(trf("Aus %s entfernt. Ausrichtung und Fusion verwenden diese Stellen nicht mehr.", project_.scans[scan].name), true);
    });
}
void ProjectUi::undoCloudCut() {
    if (cuts_.empty() || jobRunning_) return;
    CutStep& s = cuts_.back(); if (s.scan < 0 || s.scan >= (int)project_.scans.size()) { cuts_.pop_back(); return; }
    restoreCut(project_.scans[s.scan], s.cut); const int scan = s.scan; cuts_.pop_back(); dirty_ = true;
    if (scan != reviewScan_) showReview(scan); else uploadReview(false);
}
void ProjectUi::cloudBar(ImVec2 o, ImVec2 r) {
    const float b = px(36), gap = px(3), sep = px(13), newW = px(150), barH = b+px(12);
    const float barW = 6*b+4*gap+3*sep+newW+px(12)+px(2);
    forge::beginFloating("##reviewbar", {o.x+(r.x-barW)/2, o.y+r.y-barH-px(16)}, {barW, barH});
    ImGui::SetCursorPos({px(6), px(6)});
    auto separator = [&] { ImGui::SameLine(0, sep/2); const ImVec2 c = ImGui::GetCursorScreenPos(); ImGui::GetWindowDrawList()->AddLine({c.x, c.y+px(8)}, {c.x, c.y+b-px(8)}, forge::col(forge::lineStrong)); ImGui::Dummy({0, 0}); ImGui::SameLine(0, sep/2); };
    const bool hasCloud = reviewScan_ >= 0 && reviewScan_ < (int)project_.scans.size() && project_.scans[reviewScan_].cloud;
    ImGui::BeginDisabled(jobRunning_ || !hasCloud);
    auto mode = [&](const char* icon, const char* key, CloudSelect m, const char* tip) { if (forge::iconButton(icon, key, tip, cloudSelect_ == m)) cloudSelect_ = cloudSelect_ == m ? CloudSelect::None : m; ImGui::SameLine(); };
    mode(IC_LASSO, tr("Lasso"), CloudSelect::Lasso, tr("Lasso: Punkte frei umfahren, Strg wählt ab"));
    mode(IC_SQUARE_DASHED, tr("Rechteck"), CloudSelect::Box, tr("Rechteck aufziehen, Strg wählt ab"));
    if (forge::iconButton(IC_LAYERS, tr("Durchdringen"), tr("Durchdringen: wählt auch verdeckte Punkte"), cloudPenetrate_)) cloudPenetrate_ = !cloudPenetrate_;
    separator();
    ImGui::BeginDisabled(!cloudMarkedCount_);
    if (forge::iconButton(IC_CIRCLE_X, tr("Aufheben"), tr("Auswahl aufheben"))) clearCloudMarks();
    ImGui::SameLine();
    if (forge::iconButton(IC_TRASH_2, tr("Löschen"), tr("Ausgewählte Punkte aus dem Scan entfernen (Entf)"))) deleteCloudMarked();
    ImGui::EndDisabled();
    separator();
    ImGui::BeginDisabled(cuts_.empty());
    if (forge::iconButton(IC_UNDO_2, tr("Rückgängig"), tr("Rückgängig (Strg+Z)"))) undoCloudCut();
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    separator();
    ImGui::BeginDisabled(jobRunning_);
    if (forge::primaryButton(IC_PLUS "  Neuer Scan##r", {newW, b})) newScan(false);
    forge::noteItem("Neuer Scan");
    ImGui::EndDisabled(); forge::endFloating();
}
std::string ProjectUi::projectName() const { return std::filesystem::path(path_).stem().string(); }
void ProjectUi::closeProject() { project_ = Project{}; path_[0] = 0; dirty_ = false; reviewScan_ = -1; reviewDirty_ = false; cuts_.clear(); closeAlign(); }
bool ProjectUi::createProject(const std::string& file, std::string& error) {
    std::error_code ec; std::filesystem::create_directories(std::filesystem::path(file).parent_path(), ec);
    if (ec) { error = tr("Der Ordner lässt sich nicht anlegen: ")+ec.message(); return false; }
    Project empty; if (!saveProject(empty, file, error)) return false;
    project_ = Project{}; dirty_ = false; reviewScan_ = -1; reviewDirty_ = false; cuts_.clear(); closeAlign(); snprintf(path_, sizeof path_, "%s", file.c_str());
    if (onFileChanged) onFileChanged(file);
    return true;
}
void ProjectUi::requestSave() {
    const std::string start = hasProject() ? std::string(path_) : defaultFolder+"/projekt.vfproj";
    filedialog::request(filedialog::Kind::Save, tr("Projekt speichern"), start, {{tr("VoxelForge-Projekt"), {"vfproj"}}}, [this](const std::string& f) { saveTo(f); });
}
void ProjectUi::requestOpen() {
    filedialog::request(filedialog::Kind::Open, tr("Projekt öffnen"), hasProject() ? std::string(path_) : defaultFolder, {{tr("VoxelForge-Projekt"), {"vfproj"}}}, [this](const std::string& f) { loadFile(f); });
}
void ProjectUi::saveTo(const std::string& file) {
    auto error = std::make_shared<std::string>(); auto ok = std::make_shared<bool>(false);
    // The finished model makes the best picture. Copied now, as it may change while this runs.
    std::shared_ptr<Mesh> model; if (edit_.hasMesh()) model = std::make_shared<Mesh>(edit_.mesh());
    std::error_code ec; std::filesystem::create_directories(std::filesystem::path(file).parent_path(), ec);
    startJob(tr("Projekt wird gespeichert"), [this, file, error, ok, model] {
        *ok = saveProject(project_, file, *error);
        if (*ok) { const std::string thumb = thumbnailFile(file); if (!model || !writeThumbnail(*model, V3{0, 0, 1}, thumb)) writeProjectThumbnail(project_, thumb); }
    }, [this, file, error, ok] {
        if (*ok) { snprintf(path_, sizeof path_, "%s", file.c_str()); dirty_ = false; say(tr("Gespeichert: ")+std::filesystem::path(file).filename().string(), true); if (onFileChanged) onFileChanged(file); }
        else say(*error, false);
    });
}
bool ProjectUi::loadFile(const std::string& path) {
    auto loaded = std::make_shared<Project>(); auto error = std::make_shared<std::string>(); auto ok = std::make_shared<bool>(false);
    startJob(tr("Projekt wird geladen"), [path, loaded, error, ok] { *ok = loadProject(*loaded, path, *error); }, [this, path, loaded, error, ok] {
        if (*ok) { project_ = std::move(*loaded); dirty_ = false; cuts_.clear(); closeAlign(); reviewLast(); snprintf(path_, sizeof path_, "%s", path.c_str()); say(tr("Geöffnet: ")+std::filesystem::path(path).stem().string(), true); if (onFileChanged) onFileChanged(path); }
        else say(*error, false);
    });
    return true;
}
// Right click on a scan: rename it, release its placement, or remove it, as in the original's project panel.
void ProjectUi::scanMenu(size_t i) {
    if (!ImGui::BeginPopup("##scanmenu")) return;
    ScanData& s = project_.scans[i];
    if (ImGui::MenuItem(tr("Umbenennen"))) { renameIndex_ = (int)i; snprintf(renameText_, sizeof renameText_, "%s", s.name.c_str()); renameRequest_ = true; }
    // The first placed scan holds the coordinates; releasing it would leave the others without reference.
    int placed = 0; for (const ScanData& d : project_.scans) placed += d.placed;
    if (ImGui::MenuItem(tr("Ausrichtung aufheben"), nullptr, false, s.placed && placed > 1)) { s.placed = false; s.placement = Pose{}; dirty_ = true; say(trf("%s ist wieder offen und kann neu ausgerichtet werden.", s.name), true); }
    if (ImGui::MenuItem(tr("Entfernen"))) { removeIndex_ = (int)i; removeRequest_ = true; }
    ImGui::EndPopup();
}
void ProjectUi::renameDialog() {
    if (renameRequest_) { renameRequest_ = false; ImGui::OpenPopup("Scan umbenennen"); }
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {px(20), px(18)}); ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, px(16)); ImGui::PushStyleColor(ImGuiCol_PopupBg, forge::surface);
    if (ImGui::BeginPopupModal("Scan umbenennen", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar)) {
        ImGui::PushFont(forge::fontTitle); ImGui::TextUnformatted(tr("Scan umbenennen")); ImGui::PopFont();
        if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
        ImGui::SetNextItemWidth(px(346)); const bool enter = ImGui::InputText("##name", renameText_, sizeof renameText_, ImGuiInputTextFlags_EnterReturnsTrue);
        ImGui::Dummy({0, px(4)});
        if (forge::button("Abbrechen##rn", {px(170), px(36)})) ImGui::CloseCurrentPopup();
        ImGui::SameLine();
        const bool valid = renameText_[0] != 0 && renameIndex_ >= 0 && (size_t)renameIndex_ < project_.scans.size();
        ImGui::BeginDisabled(!valid);
        if ((forge::primaryButton(IC_CHECK "  Übernehmen##rn", {px(170), px(36)}) || enter) && valid) { project_.scans[renameIndex_].name = renameText_; dirty_ = true; ImGui::CloseCurrentPopup(); }
        ImGui::EndDisabled();
        ImGui::EndPopup();
    }
    ImGui::PopStyleVar(2); ImGui::PopStyleColor();
}
void ProjectUi::removeDialog() {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {px(20), px(18)}); ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, px(16)); ImGui::PushStyleColor(ImGuiCol_PopupBg, forge::surface);
    const bool open = ImGui::BeginPopupModal("Scan entfernen", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar);
    if (open) {
        ImGui::PushFont(forge::fontTitle); ImGui::TextUnformatted(tr("Scan entfernen?")); ImGui::PopFont();
        if (removeIndex_ >= 0 && (size_t)removeIndex_ < project_.scans.size()) { ImGui::PushStyleColor(ImGuiCol_Text, forge::textDim); ImGui::Text(tr("%s wird aus dem Projekt entfernt. Das lässt sich nicht rückgängig machen."), project_.scans[removeIndex_].name.c_str()); ImGui::PopStyleColor(); }
        ImGui::Dummy({0, px(4)});
        if (forge::button("Abbrechen##rm", {px(170), px(36)})) ImGui::CloseCurrentPopup();
        ImGui::SameLine();
        if (forge::dangerButton(IC_TRASH_2 "  Entfernen##rm", {px(170), px(36)})) { if (removeIndex_ >= 0) removeScan((size_t)removeIndex_); removeIndex_ = -1; ImGui::CloseCurrentPopup(); }
        forge::noteItem("Entfernen");
        ImGui::EndPopup();
    }
    ImGui::PopStyleVar(2); ImGui::PopStyleColor();
}
void ProjectUi::removeScan(size_t i) {
    if (i >= project_.scans.size()) return;
    project_.scans.erase(project_.scans.begin()+i); dirty_ = true; cuts_.clear(); if (reviewScan_ >= 0) reviewLast();
    bool anyPlaced = false; for (const ScanData& s : project_.scans) anyPlaced |= s.placed;
    if (!anyPlaced && !project_.scans.empty()) { project_.scans[0].placed = true; project_.scans[0].placement = Pose{}; }
}

// ---- Alignment -----------------------------------------------------------------------------------
// In a worker: scans without markers lose the surface they stood on, before anything is built from them.
void ProjectUi::cutSupports() {
    for (ScanData& s : project_.scans) if (removeSupport(s)) { std::lock_guard<std::mutex> g(labelMutex_); if (!supportCut_.empty()) supportCut_ += ", "; supportCut_ += s.name; }
}
// Back on the main thread: says which scans lost their support surface.
void ProjectUi::reportSupports() {
    std::string names; { std::lock_guard<std::mutex> g(labelMutex_); names.swap(supportCut_); }
    if (names.empty()) return;
    dirty_ = true; forge::toast(trf("Unterlage entfernt: %s", names), forge::Toast::Info);
}
void ProjectUi::openAlign() {
    aligning_ = true; hasResult_ = false; proposals_.clear(); chosen_.assign(project_.scans.size(), 1); message.clear();
    fixedPoints_.clear(); movingPoints_.clear(); press_ = {}; liveValid_ = false; mergedReset_ = true;
    // Manual: a placed scan stays, the first one not placed yet moves.
    fixedScan_ = 0; movingScan_ = project_.scans.size() > 1 ? 1 : 0;
    for (size_t i = 0; i < project_.scans.size(); i++) if (project_.scans[i].placed) { fixedScan_ = (int)i; break; }
    for (size_t i = 0; i < project_.scans.size(); i++) if (!project_.scans[i].placed && (int)i != fixedScan_) { movingScan_ = (int)i; break; }
    if (movingScan_ == fixedScan_) movingScan_ = fixedScan_ == 0 && project_.scans.size() > 1 ? 1 : 0;
    startJob(tr("Scans werden vorbereitet"), [this] {
        cutSupports();
        const size_t n = project_.scans.size();
        #pragma omp parallel for schedule(dynamic)
        for (size_t i = 0; i < n; i++) buildOverview(project_.scans[i]);
        // The fine points with their colors, for picking the same spot on two scans.
        for (size_t i = 0; i < n; i++) { buildPointCloud(project_.scans[i]); jobProgress_ = (i+1.f)/n; }
    }, [this] { reportSupports(); if (manual_) showManualViews(); else showPreview(true); });
}
void ProjectUi::closeAlign() { aligning_ = false; hasResult_ = false; proposals_.clear(); combined_.reset(); }
Pose ProjectUi::proposedPlacement(size_t i) const {
    auto it = proposals_.find(i);
    if (it == proposals_.end()) return project_.scans[i].placed ? project_.scans[i].placement : Pose{};
    if (!it->second.options.empty()) return it->second.options[it->second.current].srcToDst;
    return it->second.fromMarkers;
}
void ProjectUi::showPreview(bool resetCamera) {
    auto m = std::make_shared<Mesh>();
    // Up is what was up in the first scan that is shown.
    V3 up{0, 0, 1}; bool haveUp = false; size_t ref = 0;
    for (size_t i = 0; i < project_.scans.size() && !haveUp; i++) if (i < chosen_.size() && chosen_[i]) { const ScanData& s = project_.scans[i]; up = (s.placed ? s.placement : Pose{}).rot(s.up); haveUp = true; ref = i; }
    (void)ref;
    const V3 z = normalized(up), right = normalized(cross(std::fabs(z.y) < 0.9f ? V3{0, 1, 0} : V3{1, 0, 0}, z));
    float offset = 0;
    for (size_t i = 0; i < project_.scans.size(); i++) {
        const ScanData& s = project_.scans[i]; if (!s.overview || i >= chosen_.size() || !chosen_[i]) continue;
        Pose P = proposedPlacement(i);
        if (!s.placed && !proposals_.count(i)) {
            // Not placed yet: upright like the others and side by side, so that all can be seen.
            const V3 az = normalized(s.up), ax = normalized(cross(std::fabs(az.y) < 0.9f ? V3{0, 1, 0} : V3{1, 0, 0}, az)), ay = cross(az, ax);
            const V3 bz = z, bx = right, by = cross(bz, bx);
            for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) { const float tgt[3] = {r == 0 ? bx.x : r == 1 ? bx.y : bx.z, r == 0 ? by.x : r == 1 ? by.y : by.z, r == 0 ? bz.x : r == 1 ? bz.y : bz.z}; const float a[3] = {c == 0 ? ax.x : c == 1 ? ax.y : ax.z, c == 0 ? ay.x : c == 1 ? ay.y : ay.z, c == 0 ? az.x : c == 1 ? az.y : az.z}; P.R[r*3+c] = tgt[0]*a[0]+tgt[1]*a[1]+tgt[2]*a[2]; }
            V3 lo, hi; meshBounds(*s.overview, lo, hi); const V3 c0 = P.rot((lo+hi)*0.5f); const float width = norm(hi-lo);
            V3 anchor{}; for (size_t k = 0; k < m->vertices.size(); k += std::max<size_t>(1, m->vertices.size()/512)) anchor = anchor+m->vertices[k];
            if (!m->vertices.empty()) anchor = anchor*(1.f/((m->vertices.size()-1)/std::max<size_t>(1, m->vertices.size()/512)+1));
            offset += width*0.65f; P.t = anchor+right*offset-c0; offset += width*0.65f;
        }
        appendMesh(*m, *s.overview, P, kScanColors[i%6]);
    }
    computeNormals(*m); combined_ = m;
    view_.setUpDirection(up);
    if (!m->indices.empty()) view_.setMesh(*m, resetCamera);
}
void ProjectUi::showManualViews() {
    auto show = [this](MeshView& v, int index) {
        if ((size_t)index >= project_.scans.size()) return;
        const ScanData& s = project_.scans[index]; v.setUpDirection(s.up);
        if (s.cloud && !s.cloud->vertices.empty()) v.setPointCloud(*s.cloud, true, s.cloudSpacingMm);
        else if (s.overview) { Mesh m = *s.overview; m.colors.clear(); computeNormals(m); v.setMesh(m, true); }
    };
    show(view_, fixedScan_); show(viewB_, movingScan_);
    press_ = {}; mergedReset_ = true; pointsChanged();
}
// Three pairs fix the placement at once, before any fine alignment: the merged view follows every click.
void ProjectUi::pointsChanged() {
    hasResult_ = false; proposals_.clear(); message.clear(); liveValid_ = false; pairApart_.clear();
    const size_t n = std::min(fixedPoints_.size(), movingPoints_.size());
    if (n >= 3 && (size_t)fixedScan_ < project_.scans.size()) {
        const ScanData& fix = project_.scans[fixedScan_]; const Pose fp = fix.placed ? fix.placement : Pose{};
        std::vector<V3> src, dst; for (size_t i = 0; i < n; i++) { src.push_back(movingPoints_[i].p); dst.push_back(fp.apply(fixedPoints_[i].p)); }
        if (registerPointPairs(src, dst, livePose_)) { liveValid_ = true; for (size_t i = 0; i < n; i++) pairApart_.push_back(norm(livePose_.apply(src[i])-dst[i])); }
    }
    showMerged();
}
void ProjectUi::showMerged() {
    if ((size_t)fixedScan_ >= project_.scans.size() || (size_t)movingScan_ >= project_.scans.size()) return;
    const ScanData& fix = project_.scans[fixedScan_]; const ScanData& mov = project_.scans[movingScan_];
    if (!fix.overview || !mov.overview) return;
    const Pose fp = fix.placed ? fix.placement : Pose{};
    auto m = std::make_shared<Mesh>(); appendMesh(*m, *fix.overview, fp, kScanColors[0]);
    const bool placed = hasResult_ || liveValid_;
    if (placed) appendMesh(*m, *mov.overview, hasResult_ ? proposedPlacement((size_t)movingScan_) : livePose_, kScanColors[1]);
    computeNormals(*m); combined_ = m;
    viewC_.setUpDirection(fp.rot(fix.up)); viewC_.setMesh(*m, mergedReset_);
    if (placed) mergedReset_ = false;
}
void ProjectUi::removePair(size_t i) {
    if (i < fixedPoints_.size()) fixedPoints_.erase(fixedPoints_.begin()+i);
    if (i < movingPoints_.size()) movingPoints_.erase(movingPoints_.begin()+i);
    pointsChanged();
}
void ProjectUi::runAuto() {
    std::vector<size_t> use; for (size_t i = 0; i < chosen_.size(); i++) if (chosen_[i]) use.push_back(i);
    if (use.size() < 2) { say(tr("Mindestens zwei Scans auswählen."), false); return; }
    // A placed scan stays where it is and serves as the reference. Otherwise the first one does.
    size_t ref = use[0]; for (size_t i : use) if (project_.scans[i].placed) { ref = i; break; }
    reference_ = ref; message.clear(); const bool markers = byMarkers_;
    auto result = std::make_shared<std::map<size_t, Proposal>>(); auto failed = std::make_shared<std::string>();
    startJob(markers ? tr("Ausrichten über Marker") : tr("Ausrichten über Merkmale"), [this, use, ref, markers, result, failed] {
        Mesh merged; std::vector<V3> mapMerged;
        const Pose refPose = project_.scans[ref].placed ? project_.scans[ref].placement : Pose{};
        { const float none[3] = {0, 0, 0}; appendMesh(merged, *project_.scans[ref].overview, refPose, none); merged.colors.clear(); computeNormals(merged); }
        for (const V3& v : project_.scans[ref].markers) mapMerged.push_back(refPose.apply(v));
        // Nothing of the object lies below the table the reference stood on, nor below that of the scan being placed.
        const Table refTable = placedTable(scanTable(project_.scans[ref]), refPose);
        size_t done = 0;
        for (size_t i : use) {
            if (i == ref) continue;
            const ScanData& s = project_.scans[i]; Proposal pr; Mesh src = *s.overview; src.colors.clear(); computeNormals(src);
            if (markers) {
                Pose P; int matched = 0;
                if (!registerMarkers(s.markers, mapMerged, P, matched)) { *failed = trf("%s: zu wenige gemeinsame Marker. Merkmale oder Manuell versuchen.", s.name); return; }
                pr.fromMarkers = P; pr.markers = matched;
                const RegisterResult fine = refineRegistration(src, merged, P, 3.f, scanTable(s), refTable); if (fine.ok && fine.overlap > 0.1f) { RegisterResult f = fine; f.confident = matched >= 5; pr.options.push_back(f); }
            } else {
                pr.options = registerAuto(src, merged, [this, done, &use](float f) { jobProgress_ = (done+f)/(use.size()-1); }, scanTable(s), refTable);
                if (pr.options.empty()) { *failed = trf("%s: keine passende Lage gefunden. Überlappen sich die Scans genug? Sonst Manuell versuchen.", s.name); return; }
            }
            const Pose P = pr.options.empty() ? pr.fromMarkers : pr.options[0].srcToDst;
            // Later scans can also align to this one.
            { const float none[3] = {0, 0, 0}; appendMesh(merged, *s.overview, P, none); merged.colors.clear(); computeNormals(merged); }
            for (const V3& v : s.markers) mapMerged.push_back(P.apply(v));
            (*result)[i] = std::move(pr); done++;
        }
    }, [this, result, failed] {
        if (!failed->empty()) { say(*failed, false); return; }
        proposals_ = std::move(*result); hasResult_ = true; showPreview(true);
    });
}
void ProjectUi::runManual() {
    if (fixedScan_ == movingScan_) { say(tr("Zwei verschiedene Scans wählen."), false); return; }
    const size_t n = std::min(fixedPoints_.size(), movingPoints_.size());
    if (n < 1) { say(tr("Mindestens ein Punktpaar setzen: dieselbe Stelle links und rechts anklicken."), false); return; }
    const ScanData& fix = project_.scans[fixedScan_];
    bool anyPlaced = false; for (const ScanData& s : project_.scans) anyPlaced |= s.placed;
    if (!fix.placed && anyPlaced) { say(tr("Als festen Scan einen bereits ausgerichteten wählen."), false); return; }
    const Pose fp = fix.placed ? fix.placement : Pose{};
    std::vector<V3> sp, sn, dp, dn;
    for (size_t i = 0; i < n; i++) { sp.push_back(movingPoints_[i].p); sn.push_back(movingPoints_[i].n); dp.push_back(fp.apply(fixedPoints_[i].p)); dn.push_back(fp.rot(fixedPoints_[i].n)); }
    message.clear(); const size_t mov = (size_t)movingScan_, fx = (size_t)fixedScan_; reference_ = fx;
    auto pr = std::make_shared<Proposal>();
    startJob(n >= 3 ? tr("Feinausrichtung") : tr("Lage wird gesucht"), [this, pr, mov, fx, fp, sp, sn, dp, dn] {
        Mesh src = *project_.scans[mov].overview, dstM; src.colors.clear(); computeNormals(src);
        const float none[3] = {0, 0, 0}; appendMesh(dstM, *project_.scans[fx].overview, fp, none); dstM.colors.clear(); computeNormals(dstM);
        pr->options = registerFromPairs(src, dstM, sp, sn, dp, dn, scanTable(project_.scans[mov]), placedTable(scanTable(project_.scans[fx]), fp));
    }, [this, pr, mov] {
        if (pr->options.empty()) { say(tr("Zu diesen Punkten passt keine Lage. Punkte prüfen oder ein weiteres Paar setzen."), false); return; }
        proposals_.clear(); proposals_[mov] = std::move(*pr); hasResult_ = true; showMerged();
    });
}
void ProjectUi::acceptAlign() {
    dirty_ = true;
    for (auto& e : proposals_) { ScanData& s = project_.scans[e.first]; s.placement = proposedPlacement(e.first); s.placed = true; }
    if (reference_ < project_.scans.size()) project_.scans[reference_].placed = true;
    // Manual work goes on with the next scan that is not placed yet, against the one that stayed.
    if (manual_) {
        int next = -1; for (size_t i = 0; i < project_.scans.size(); i++) if (!project_.scans[i].placed) { next = (int)i; break; }
        if (next >= 0) {
            movingScan_ = next; fixedPoints_.clear(); movingPoints_.clear(); hasResult_ = false; proposals_.clear();
            showManualViews(); say(trf("Übernommen. Weiter mit %s.", project_.scans[next].name), true); return;
        }
    }
    say(tr("Ausrichtung übernommen. Jetzt Fusion starten."), true);
    closeAlign();
}
namespace {
// One color per pair of points, the same on both scans.
const ImVec4 kPairColors[10] = {{.98f, .45f, .35f, 1}, {.35f, .85f, .45f, 1}, {.36f, .66f, .98f, 1}, {.98f, .82f, .25f, 1}, {.82f, .45f, .95f, 1},
                                {.25f, .88f, .85f, 1}, {.98f, .58f, .16f, 1}, {.65f, .9f, .3f, 1}, {.95f, .45f, .7f, 1}, {.6f, .62f, .98f, 1}};
ImVec4 pairColor(size_t i) { return kPairColors[i%10]; }
}
void ProjectUi::alignPanel() {
    forge::beginCard("##align");
    if (forge::iconButton(IC_CHEVRON_LEFT, tr("Zurück zum Scan"), tr("Zurück"), false, px(30))) { closeAlign(); forge::endCard(); return; }
    ImGui::SameLine(0, px(6)); ImGui::PushFont(forge::fontTitle); ImGui::SetCursorPosY(ImGui::GetCursorPosY()+px(6)); ImGui::TextUnformatted(tr("Ausrichten")); ImGui::PopFont();
    ImGui::BeginDisabled(jobRunning_);
    int kind = byMarkers_ ? 1 : 0; const char* const kinds[] = {tr("Merkmale"), tr("Marker")};
    if (!manual_ && forge::segmented("kind", kind, kinds, 2)) { byMarkers_ = kind == 1; hasResult_ = false; proposals_.clear(); showPreview(false); }
    int how = manual_ ? 1 : 0; const char* const hows[] = {tr("Auto"), tr("Manuell")};
    if (forge::segmented("how", how, hows, 2)) { manual_ = how == 1; hasResult_ = false; proposals_.clear(); fixedPoints_.clear(); movingPoints_.clear(); chosen_.assign(project_.scans.size(), 1); message.clear(); if (manual_) showManualViews(); else showPreview(true); }
    ImGui::EndDisabled();
    if (byMarkers_ && !manual_) forge::hint(tr("Nur wenn das Objekt zu den Markern gleich liegt. Wurde es auf dem Drehteller umgelegt, Merkmale nehmen."));
    char b[200];
    auto resultLines = [&](const RegisterResult& r) {
        snprintf(b, sizeof b, "%.0f %%", 100*r.overlap); forge::labelValue(tr("Überlappung"), localDecimals(b)); snprintf(b, sizeof b, "%.2f mm", r.rmsMm); forge::labelValue(tr("Abstand"), localDecimals(b));
        if (r.confident) forge::chip(tr("Eindeutig"), forge::green, IC_CIRCLE_CHECK); else forge::chip(tr("Nicht sicher, bitte prüfen"), forge::amber, IC_TRIANGLE_ALERT);
        if (r.pinned < kMinPinned) forge::hint(tr("Die gemeinsame Fläche hält die Lage nicht in jeder Richtung fest, etwa wenn sich nur flache Wände überlappen. Die Lage kann verrutscht sein. Bei Punktpaaren an Enden und Kanten setzen."));
    };
    if (!manual_) {
        forge::hint(tr("Liegen die Scans richtig, mischen sich ihre Farben fein. Große einfarbige Flächen bedeuten eine falsche Lage."));
        ImGui::BeginDisabled(jobRunning_ || hasResult_);
        for (size_t i = 0; i < project_.scans.size() && i < chosen_.size(); i++) {
            bool c = chosen_[i] != 0; const float* col = kScanColors[i%6]; const ImVec2 at = ImGui::GetCursorScreenPos();
            ImGui::GetWindowDrawList()->AddCircleFilled({at.x+px(5), at.y+ImGui::GetTextLineHeight()/2+px(2)}, px(4.5f), forge::col(ImVec4(col[0], col[1], col[2], 1)));
            ImGui::Indent(px(16)); if (forge::toggle(project_.scans[i].name.c_str(), &c)) { chosen_[i] = c; showPreview(false); } ImGui::Unindent(px(16));
        }
        ImGui::EndDisabled();
        if (hasResult_) {
            for (auto& e : proposals_) {
                Proposal& p = e.second; ImGui::PushID((int)e.first);
                ImGui::PushFont(forge::fontMedium); ImGui::TextUnformatted(project_.scans[e.first].name.c_str()); ImGui::PopFont();
                if (!p.options.empty()) {
                    resultLines(p.options[p.current]);
                    ImGui::BeginDisabled(jobRunning_);
                    if (p.options.size() > 1) {
                        char label[64]; snprintf(label, sizeof label, tr(IC_ARROW_RIGHT "  Nächster Vorschlag (%zu von %zu)"), p.current+1, p.options.size());
                        if (forge::button(label, {-FLT_MIN, 0})) { p.current = (p.current+1)%p.options.size(); showPreview(false); }
                        forge::noteItem("Nächster Vorschlag");
                    }
                    if (!p.options[p.current].confident) {
                        // Straight to picking points for exactly this scan against the reference.
                        if (forge::button(tr(IC_CROSSHAIR "  Manuell ausrichten##m"), {-FLT_MIN, 0})) {
                            const int mov = (int)e.first; ImGui::PopID(); ImGui::EndDisabled();
                            manual_ = true; hasResult_ = false; proposals_.clear(); fixedPoints_.clear(); movingPoints_.clear(); fixedScan_ = (int)reference_; movingScan_ = mov; showManualViews();
                            forge::endCard(); return;
                        }
                        forge::noteItem("Manuell ausrichten##m");
                    }
                    ImGui::EndDisabled();
                } else if (p.markers) { snprintf(b, sizeof b, "%d", p.markers); forge::labelValue(tr("Gemeinsame Marker"), b); }
                ImGui::PopID();
            }
        }
    } else {
        std::vector<const char*> names; for (const ScanData& s : project_.scans) names.push_back(s.name.c_str());
        ImGui::BeginDisabled(jobRunning_);
        bool ch = false;
        ch |= forge::combo(tr("Bleibt liegen"), &fixedScan_, names.data(), (int)names.size());
        ch |= forge::combo(tr("Wird bewegt"), &movingScan_, names.data(), (int)names.size());
        if (forge::button(tr(IC_REPEAT_2 "  Seiten tauschen##sw"), {-FLT_MIN, 0})) { std::swap(fixedScan_, movingScan_); std::swap(fixedPoints_, movingPoints_); showManualViews(); }
        forge::noteItem("Seiten tauschen##sw");
        if (ch) { fixedPoints_.clear(); movingPoints_.clear(); showManualViews(); }
        ImGui::EndDisabled();
        forge::hint(tr("Dieselbe Stelle links und rechts anklicken. Ein Paar genügt oft, die Vorschau sucht den Rest. Drei weit verteilte Paare legen die Lage fest, die Ansicht unten folgt sofort."));
        forge::hint(tr("Ziehen dreht, rechte Taste verschiebt, Rad zoomt. Punkte lassen sich mit der Maus verschieben, Strg+Klick löscht ein Paar."));
        const size_t pairs = std::max(fixedPoints_.size(), movingPoints_.size());
        // Pairs far apart after placing by the points alone were probably picked on different spots.
        float typical = 0; if (pairApart_.size() >= 3) { std::vector<float> v = pairApart_; std::nth_element(v.begin(), v.begin()+v.size()/2, v.end()); typical = v[v.size()/2]; }
        ImGui::BeginDisabled(jobRunning_);
        for (size_t i = 0; i < pairs; i++) {
            ImGui::PushID((int)i); const ImVec2 at = ImGui::GetCursorScreenPos(); const float lh = ImGui::GetFrameHeight();
            ImGui::GetWindowDrawList()->AddCircleFilled({at.x+px(6), at.y+lh/2}, px(5), forge::col(pairColor(i)));
            ImGui::SetCursorScreenPos({at.x+px(18), at.y+(lh-ImGui::GetTextLineHeight())/2});
            snprintf(b, sizeof b, tr("Paar %d"), (int)i+1); ImGui::TextUnformatted(b); ImGui::SameLine(px(90));
            const bool hasL = i < fixedPoints_.size(), hasR = i < movingPoints_.size();
            if (!hasL) ImGui::TextColored(forge::amber, "%s", tr("links fehlt"));
            else if (!hasR) ImGui::TextColored(forge::amber, "%s", tr("rechts fehlt"));
            else if (i < pairApart_.size()) { const bool off = pairApart_[i] > std::max(2.f, 3*typical); snprintf(b, sizeof b, "%.1f mm", pairApart_[i]); ImGui::TextColored(off ? forge::danger : forge::textDim, "%s", localDecimals(b)); if (off && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tr("Passt nicht zu den anderen Paaren. Liegen beide Punkte auf derselben Stelle?")); }
            else ImGui::TextColored(forge::textDim, "%s", IC_CHECK);
            ImGui::SetCursorScreenPos({at.x+ImGui::GetContentRegionAvail().x-px(26), at.y});
            char key[32]; snprintf(key, sizeof key, "Paar weg##%zu", i);
            if (forge::iconButton(IC_TRASH_2, key, tr("Paar löschen"), false, px(26))) { removePair(i); ImGui::PopID(); break; }
            ImGui::SetCursorScreenPos({at.x, at.y+lh+px(3)}); ImGui::Dummy({0, 0}); ImGui::PopID();
        }
        if (pairs) { if (forge::button(tr(IC_TRASH_2 "  Alle Punkte löschen##p"), {-FLT_MIN, 0})) { fixedPoints_.clear(); movingPoints_.clear(); pointsChanged(); } forge::noteItem("Alle Punkte löschen##p"); }
        ImGui::EndDisabled();
        if (hasResult_) for (auto& e : proposals_) if (!e.second.options.empty()) {
            Proposal& p = e.second; resultLines(p.options[p.current]);
            if (p.options.size() > 1) {
                char label[64]; snprintf(label, sizeof label, tr(IC_ARROW_RIGHT "  Nächster Vorschlag (%zu von %zu)"), p.current+1, p.options.size());
                ImGui::BeginDisabled(jobRunning_); if (forge::button(label, {-FLT_MIN, 0})) { p.current = (p.current+1)%p.options.size(); showMerged(); } ImGui::EndDisabled();
                forge::noteItem("Nächster Vorschlag");
            }
        }
    }
    const float h = px(38);
    ImGui::BeginDisabled(jobRunning_);
    if (!hasResult_) {
        const bool ready = !manual_ || std::min(fixedPoints_.size(), movingPoints_.size()) >= 1;
        ImGui::BeginDisabled(!ready);
        if (forge::primaryButton(IC_EYE "  Vorschau##al", {-FLT_MIN, h})) { if (manual_) runManual(); else runAuto(); }
        forge::noteItem("Vorschau##al");
        ImGui::EndDisabled();
        if (forge::button("Abbrechen##al", {-FLT_MIN, 0})) closeAlign();
    } else {
        if (forge::primaryButton(IC_CHECK "  Bestätigen##al", {-FLT_MIN, h})) acceptAlign();
        forge::noteItem("Bestätigen##al");
        if (forge::button("Zurück##al", {-FLT_MIN, 0})) { hasResult_ = false; proposals_.clear(); message.clear(); if (manual_) pointsChanged(); else { chosen_.assign(project_.scans.size(), 1); showPreview(true); } }
    }
    ImGui::EndDisabled();
    forge::endCard();
}
bool ProjectUi::pickSurface(MeshView& view, const ScanData& s, float x, float y, PickPoint& out) {
    const Mesh* m = pickMesh(s); if (!m) return false;
    const long id = view.pickNear((int)x, (int)y, (int)px(5)); if (id < 0) return false;
    if (view.pointMode()) {
        if ((size_t)id >= m->vertices.size()) return false;
        out.p = m->vertices[id]; out.n = m->normals.size() == m->vertices.size() ? m->normals[id] : V3{}; return true;
    }
    if ((size_t)id >= m->triangleCount()) return false;
    const V3 a = m->vertices[m->indices[3*id]], b = m->vertices[m->indices[3*id+1]], c = m->vertices[m->indices[3*id+2]];
    V3 o, d; view.ray(x, y, o, d); const V3 n = cross(b-a, c-a); const float den = dot(n, d);
    out.p = std::fabs(den) > 1e-9f ? o+d*(dot(n, a-o)/den) : (a+b+c)*(1.f/3); out.n = normalized(n);
    return true;
}
// One of the two scans to pick points on. A click sets the next point, dragging turns the view, dragging a
// point moves it over the surface, Ctrl+click removes its pair.
void ProjectUi::pickView(MeshView& view, int side, ImVec2 size) {
    const ImGuiIO& io = ImGui::GetIO(); const ImVec2 o = ImGui::GetCursorScreenPos(); ImDrawList* dl = ImGui::GetWindowDrawList();
    const int w = std::max(16, (int)size.x), h = std::max(16, (int)size.y);
    const GLuint tex = view.render(w, h);
    ImGui::PushID(side); ImGui::InvisibleButton("##pick", {(float)w, (float)h}, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle); ImGui::PopID();
    forge::noteItem(side == 0 ? "Ansicht fest" : "Ansicht bewegt");
    dl->AddImageRounded((ImTextureID)(intptr_t)tex, o, {o.x+w, o.y+h}, {0, 1}, {1, 0}, IM_COL32_WHITE, px(14));
    const size_t si = (size_t)(side == 0 ? fixedScan_ : movingScan_); if (si >= project_.scans.size()) return;
    const ScanData& scan = project_.scans[si];
    std::vector<PickPoint>& pts = side == 0 ? fixedPoints_ : movingPoints_; const std::vector<PickPoint>& other = side == 0 ? movingPoints_ : fixedPoints_;
    const bool hovered = ImGui::IsItemHovered(); const ImVec2 mouse = io.MousePos;
    auto onScreen = [&](size_t i, ImVec2& q) { float x, y, d; if (!view.project(pts[i].p, x, y, d)) return false; q = {o.x+x, o.y+y}; return true; };
    auto pointAt = [&](ImVec2 at, float radius) { int best = -1; float bd = radius*radius; for (size_t i = 0; i < pts.size(); i++) { ImVec2 q; if (!onScreen(i, q)) continue; const float dx = q.x-at.x, dy = q.y-at.y; if (dx*dx+dy*dy < bd) { bd = dx*dx+dy*dy; best = (int)i; } } return best; };
    if (hovered && !jobRunning_) {
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) press_ = {side, mouse, false, io.KeyCtrl ? -1 : pointAt(mouse, px(10))};
        if (io.MouseWheel != 0) view.zoom(io.MouseWheel);
    }
    if ((hovered || ImGui::IsItemActive()) && !jobRunning_) {
        if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) view.pan(io.MouseDelta.x, io.MouseDelta.y);
        if (ImGui::IsMouseDown(ImGuiMouseButton_Middle)) view.rotate(io.MouseDelta.x, io.MouseDelta.y);
    }
    if (press_.view == side && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        if (!press_.moved) { const float dx = mouse.x-press_.at.x, dy = mouse.y-press_.at.y; press_.moved = dx*dx+dy*dy > px(4)*px(4); }
        if (press_.moved) {
            if (press_.point >= 0 && (size_t)press_.point < pts.size()) { PickPoint q; if (pickSurface(view, scan, mouse.x-o.x, mouse.y-o.y, q)) pts[press_.point] = q; }
            else view.rotate(io.MouseDelta.x, io.MouseDelta.y);
        }
    }
    if (press_.view == side && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        const Press pr = press_; press_ = {};
        if (!pr.moved) {
            if (io.KeyCtrl) { const int i = pointAt(mouse, px(14)); if (i >= 0) removePair((size_t)i); }
            else if (pr.point < 0) { PickPoint q; if (pickSurface(view, scan, mouse.x-o.x, mouse.y-o.y, q)) { pts.push_back(q); pointsChanged(); } }
        } else if (pr.point >= 0) pointsChanged();
    }
    // Points: filled once the pair is complete, a ring while the other side still lacks it.
    const int hot = hovered && press_.view < 0 ? pointAt(mouse, px(10)) : -1;
    for (size_t i = 0; i < pts.size(); i++) {
        ImVec2 q; if (!onScreen(i, q)) continue; const ImU32 c = forge::col(pairColor(i)); const float r = px(i == (size_t)hot || (press_.view == side && press_.point == (int)i) ? 8.f : 6.f);
        dl->AddCircleFilled(q, r+px(2), IM_COL32(3, 12, 24, 230));
        if (i < other.size()) dl->AddCircleFilled(q, r, c); else dl->AddCircle(q, r-px(1), c, 0, px(2.5f));
        char nb[24]; snprintf(nb, sizeof nb, "%zu", i+1); const ImVec2 at{q.x+r+px(3), q.y-px(9)}, ts = ImGui::CalcTextSize(nb);
        dl->AddRectFilled({at.x-px(2), at.y-px(1)}, {at.x+ts.x+px(2), at.y+ts.y+px(1)}, IM_COL32(3, 12, 24, 220), px(3)); dl->AddText(at, c, nb);
    }
    // The next point this view takes, at the mouse; the frame lights up where a pair waits for its second point.
    if (hovered && press_.view < 0 && hot < 0 && !jobRunning_ && !io.KeyCtrl) dl->AddCircle(mouse, px(7), forge::col(pairColor(pts.size()), .9f), 0, px(2));
    if (pts.size() < other.size()) dl->AddRect(o, {o.x+w, o.y+h}, forge::col(pairColor(pts.size())), px(14), 0, px(2.5f));
    const std::string title = trf(side == 0 ? "%s bleibt liegen" : "%s wird bewegt", scan.name);
    forge::chipAt(dl, {o.x+px(12), o.y+px(12)}, title.c_str(), side == 0 ? ImVec4(kScanColors[0][0], kScanColors[0][1], kScanColors[0][2], 1) : ImVec4(kScanColors[1][0], kScanColors[1][1], kScanColors[1][2], 1));
}
void ProjectUi::oneView(MeshView& view, ImVec2 size, const char* title, const Mesh* pickMesh, std::vector<V3>* points, bool leftTurns) {
    const ImGuiIO& io = ImGui::GetIO(); const ImVec2 o = ImGui::GetCursorScreenPos(); ImDrawList* dl = ImGui::GetWindowDrawList();
    const int w = std::max(16, (int)size.x), h = std::max(16, (int)size.y);
    const GLuint tex = view.render(w, h);
    ImGui::PushID(title ? title : "view"); ImGui::InvisibleButton("##v", {(float)w, (float)h}, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle); ImGui::PopID();
    dl->AddImageRounded((ImTextureID)(intptr_t)tex, o, {o.x+w, o.y+h}, {0, 1}, {1, 0}, IM_COL32_WHITE, px(14));
    const bool hoveredOnly = ImGui::IsItemHovered(), hovered = hoveredOnly || ImGui::IsItemActive();
    if (hovered) {
        if (ImGui::IsMouseDown(ImGuiMouseButton_Middle) || (leftTurns && ImGui::IsMouseDown(ImGuiMouseButton_Left))) view.rotate(io.MouseDelta.x, io.MouseDelta.y);
        if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) view.pan(io.MouseDelta.x, io.MouseDelta.y);
        if (hoveredOnly && io.MouseWheel != 0) view.zoom(io.MouseWheel);
    }
    (void)pickMesh; (void)points;
    if (title) forge::chipAt(dl, {o.x+px(12), o.y+px(12)}, title, forge::text);
}
void ProjectUi::alignView() {
    const ImVec2 full = ImGui::GetContentRegionAvail(); const ImVec2 avail{full.x-px(10), full.y-px(10)}; const ImVec2 o = ImGui::GetCursorScreenPos();
    if (manual_) {
        // As in the original: fixed left, moving right, and below both together as they would be placed.
        const float gap = px(8), topH = std::floor((avail.y-gap)*0.56f), half = (avail.x-gap)/2;
        if ((size_t)fixedScan_ < project_.scans.size() && (size_t)movingScan_ < project_.scans.size()) {
            pickView(view_, 0, {half, topH}); ImGui::SameLine(0, gap); pickView(viewB_, 1, {half, topH});
            ImGui::SetCursorScreenPos({o.x, o.y+topH+gap});
            const ImVec2 at = ImGui::GetCursorScreenPos(), sz{avail.x, avail.y-topH-gap};
            oneView(viewC_, sz, hasResult_ ? tr("Vorschau") : tr("Zusammengeführt"), nullptr, nullptr);
            if (!hasResult_ && !liveValid_) {
                const char* t = std::min(fixedPoints_.size(), movingPoints_.size()) ? tr("Mit „Vorschau“ wird die Lage aus den Punkten gesucht.") : tr("Hier erscheinen beide Scans zusammen, sobald drei Punktpaare gesetzt sind.");
                const ImVec2 ts = ImGui::CalcTextSize(t); ImGui::GetWindowDrawList()->AddText({at.x+(sz.x-ts.x)/2, at.y+sz.y-ts.y-px(18)}, forge::col(forge::textDim), t);
            }
        }
    } else if (combined_ && !combined_->indices.empty()) oneView(view_, avail, nullptr, nullptr, nullptr);
    if (jobRunning_) { std::string label = busyLabel(); forge::busyOverlay(o, avail, label.c_str(), jobProgress_); }
}

// ---- Fusion --------------------------------------------------------------------------------------
void ProjectUi::fusionDialog() {
    const ImVec2 c = ImGui::GetMainViewport()->GetCenter(); ImGui::SetNextWindowPos(c, ImGuiCond_Appearing, {.5f, .5f}); ImGui::SetNextWindowSize({px(520), 0});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {px(20), px(18)}); ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, px(16)); ImGui::PushStyleColor(ImGuiCol_PopupBg, forge::surface);
    if (!ImGui::BeginPopupModal("Fusion-Batch-Prozess", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar)) { ImGui::PopStyleVar(2); ImGui::PopStyleColor(); return; }
    ImGui::PushFont(forge::fontTitle); ImGui::PushStyleColor(ImGuiCol_Text, forge::sky); ImGui::TextUnformatted(IC_COMBINE); ImGui::PopStyleColor(); ImGui::SameLine(0, px(8)); ImGui::TextUnformatted(tr("Fusion")); ImGui::PopFont();
    forge::hint(tr("Verschmilzt die gewählten Scans zu einer einzigen Oberfläche."));
    bool blocked = false, any = false;
    forge::beginCard("##fuslist");
    for (size_t i = 0; i < project_.scans.size(); i++) {
        ScanData& s = project_.scans[i]; ImGui::PushID((int)i); char label[96]; snprintf(label, sizeof label, tr("%s  ·  %zu Bilder  ·  %s"), s.name.c_str(), s.frames.size(), s.placed ? tr("ausgerichtet") : tr("offen"));
        forge::toggle(label, &s.included);
        if (s.included) { any = true; blocked |= !s.placed; }
        ImGui::PopID();
    }
    forge::endCard();
    forge::beginCard("##fusset");
    forge::combo(tr("Auflösung"), &fusionVoxel_, kVoxelNames, 4);
    forge::sliderInt(tr("Rauschunterdrückung"), &fusionNoise_, 0, 100, "%d %%");
    forge::toggle(tr("Kleine Teile behalten"), &fusionKeepParts_);
    forge::toggle(tr("Bereinigen, fein ausrichten, beste Farbe"), &fusionClean_);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip(tr("Entfernt Fläche, durch die eine Kamera hindurchgesehen hat, etwa Stützen oder Fetzen.\nAusschalten, falls echte Teile des Objekts fehlen."));
    forge::endCard();
    if (blocked) {
        forge::chip(tr("Offene Scans erst ausrichten oder abwählen"), forge::amber, IC_TRIANGLE_ALERT);
        if (forge::button(IC_MOVE_3D "  Jetzt ausrichten##fus", {-FLT_MIN, 0})) { ImGui::CloseCurrentPopup(); openAlign(); }
        forge::noteItem("Jetzt ausrichten");
    }
    const float gap = ImGui::GetStyle().ItemSpacing.x, w = (ImGui::GetContentRegionAvail().x-gap)/2, h = px(38);
    if (forge::button("Abbrechen##fus", {w, h})) ImGui::CloseCurrentPopup();
    ImGui::SameLine();
    ImGui::BeginDisabled(blocked || !any);
    if (forge::primaryButton(IC_PLAY "  Fusion starten##fus", {w, h})) { ImGui::CloseCurrentPopup(); startFusion(); }
    forge::noteItem("OK##fus");
    ImGui::EndDisabled();
    ImGui::EndPopup(); ImGui::PopStyleVar(2); ImGui::PopStyleColor();
}
void ProjectUi::startFusion() {
    const float voxel = kVoxels[fusionVoxel_]; const int minWeight = 1+(fusionNoise_*7+50)/100; const bool keep = fusionKeepParts_, clean = fusionClean_;
    auto out = std::make_shared<Mesh>(); auto error = std::make_shared<std::string>(); auto ok = std::make_shared<bool>(false);
    message.clear();
    startJob(tr("Fusion"), [this, voxel, minWeight, keep, clean, out, error, ok] {
        cutSupports();
        MeshOptions o; o.minWeight = minWeight; o.removeFragments = !keep; o.cleanUp = clean; o.fillHolesMm = 0; o.keepTriangles = 1; o.upright = true;
        *ok = fuseProject(project_, voxel, o, *out, *error, [this](float f) { jobProgress_ = f; });
        if (*ok) repairManifold(*out);
    }, [this, voxel, out, error, ok] { reportSupports(); if (*ok) edit_.newMesh(std::move(*out), voxel); else say(*error, false); });
}
bool ProjectUi::buildMerged(float voxelMm, int minWeight, bool keepAllParts, Mesh& out, std::string& error) {
    // Runs in a worker thread, with the scan tab locked by the caller.
    cutSupports();
    std::vector<size_t> open; size_t ref = project_.scans.size();
    for (size_t i = 0; i < project_.scans.size(); i++) { ScanData& s = project_.scans[i]; if (!s.included) continue; buildOverview(s); if (s.placed && ref == project_.scans.size()) ref = i; else if (!s.placed) open.push_back(i); }
    if (ref == project_.scans.size()) { error = tr("Kein ausgerichteter Scan ausgewählt."); return false; }
    Mesh merged; { const float none[3] = {0, 0, 0}; appendMesh(merged, *project_.scans[ref].overview, project_.scans[ref].placement, none); merged.colors.clear(); computeNormals(merged); }
    const Table refTable = placedTable(scanTable(project_.scans[ref]), project_.scans[ref].placement);
    for (size_t i : open) {
        ScanData& s = project_.scans[i]; Mesh src = *s.overview; src.colors.clear(); computeNormals(src);
        const auto res = registerAuto(src, merged, {}, scanTable(s), refTable);
        // One click must not fuse a guess: a wrong placement would ruin the whole model without anyone noticing.
        if (res.empty() || !res[0].confident) { error = trf("%s lässt sich nicht sicher ausrichten. Bitte das Werkzeug Ausrichten benutzen und das Ergebnis prüfen.", s.name); return false; }
        s.placement = res[0].srcToDst; s.placed = true;
        const float none[3] = {0, 0, 0}; appendMesh(merged, *s.overview, s.placement, none); merged.colors.clear(); computeNormals(merged);
    }
    MeshOptions o; o.minWeight = minWeight; o.removeFragments = !keepAllParts; o.fillHolesMm = 0; o.keepTriangles = 1; o.upright = true;
    return fuseProject(project_, voxelMm, o, out, error);
}
