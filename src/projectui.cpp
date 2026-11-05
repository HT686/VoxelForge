#include <imgui.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include "measure.hpp"
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
}

ProjectUi::ProjectUi(Pipeline& pipe, EditUi& edit) : pipe_(pipe), edit_(edit) {}
ProjectUi::~ProjectUi() { if (job_.joinable()) job_.join(); }
void ProjectUi::say(const std::string& text, bool ok) { message = text; forge::toast(text, ok ? forge::Toast::Success : forge::Toast::Error); }
void ProjectUi::init() { view_.init(); viewB_.init(); reviewView_.init(); }
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
    updateReview();
    fusionDialog();
    if (removeRequest_) { removeRequest_ = false; ImGui::OpenPopup("Scan entfernen"); }
    removeDialog();
}
std::string ProjectUi::describe() const {
    std::string s = "Scans=" + std::to_string(project_.scans.size());
    for (const ScanData& d : project_.scans) s += " " + d.name + ":" + std::to_string(d.frames.size()) + (d.placed ? ":platziert" : ":offen");
    s += " Ansicht=" + (reviewing() ? project_.scans[reviewScan_].name : std::string("live")) + " Ausrichten=" + std::to_string(aligning_) + " Ergebnis=" + std::to_string(hasResult_);
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
    { const std::string title = hasProject() ? projectName() : std::string("Projekt"); forge::beginCard("##proj", title.c_str(), IC_LAYERS); }
    const bool free = canStartTools && !jobRunning_;
    if (!project_.scans.empty()) {
        for (size_t i = 0; i < project_.scans.size(); i++) {
            ScanData& s = project_.scans[i]; ImGui::PushID((int)i); const ImVec2 at = ImGui::GetCursorScreenPos(); const float w = ImGui::GetContentRegionAvail().x, h = px(44);
            ImGui::SetNextItemAllowOverlap(); ImGui::BeginDisabled(!free);
            if (ImGui::InvisibleButton("##row", {w-px(70), h})) showReview((int)i);
            const bool rowHover = ImGui::IsItemHovered(); { char note[80]; snprintf(note, sizeof note, "Zeile %s", s.name.c_str()); forge::noteItem(note); } ImGui::EndDisabled();
            if (rowHover) ImGui::SetTooltip("Punktwolke dieses Scans ansehen");
            ImDrawList* dl = ImGui::GetWindowDrawList(); dl->AddRectFilled(at, {at.x+w, at.y+h}, forge::col(rowHover ? forge::raised : forge::bg), px(9));
            if (reviewScan_ == (int)i) dl->AddRect(at, {at.x+w, at.y+h}, forge::col(forge::sky), px(9), 0, px(1.5f));
            const float* colr = kScanColors[i%6]; dl->AddCircleFilled({at.x+px(14), at.y+h/2}, px(4.5f), forge::col(ImVec4(colr[0], colr[1], colr[2], s.included ? 1.f : .3f)));
            ImGui::PushFont(forge::fontMedium); dl->AddText({at.x+px(28), at.y+px(6)}, forge::col(s.included ? forge::text : forge::textDim), s.name.c_str()); ImGui::PopFont();
            char sub[64]; snprintf(sub, sizeof sub, "%zu Bilder  ·  %s", s.frames.size(), s.placed ? "ausgerichtet" : "offen"); dl->AddText(forge::fontBody, ImGui::GetFontSize()*.88f, {at.x+px(28), at.y+px(24)}, forge::col(s.placed ? forge::textDim : forge::amber), sub);
            ImGui::SetCursorScreenPos({at.x+w-px(66), at.y+px(7)}); ImGui::BeginDisabled(!free);
            char key[32]; snprintf(key, sizeof key, "Sichtbar##%zu", i); if (forge::iconButton(s.included ? IC_EYE : IC_EYE_OFF, key, s.included ? "Gehört zum Modell" : "Gehört nicht zum Modell", false, px(30))) s.included = !s.included;
            ImGui::SameLine(0, px(2)); snprintf(key, sizeof key, "X##%zu", i); if (forge::iconButton(IC_TRASH_2, key, "Scan entfernen", false, px(30))) { removeIndex_ = (int)i; removeRequest_ = true; }
            ImGui::EndDisabled();
            ImGui::SetCursorScreenPos({at.x, at.y+h+px(5)}); ImGui::Dummy({0, 0}); ImGui::PopID();
        }
    } else if (!liveFrames) forge::hint("Mehrere Scans aus verschiedenen Lagen werden hier gesammelt und zu einem Modell zusammengeführt.");
    if (liveFrames) {
        char b[64]; snprintf(b, sizeof b, IC_SCAN_LINE "  Aktueller Scan: %d Bilder", liveFrames);
        if (forge::toggleButton(b, !reviewing(), {-FLT_MIN, 0})) { reviewScan_ = -1; reviewDirty_ = false; }
        forge::noteItem("Aktueller Scan");
    }
    ImGui::BeginDisabled(!free);
    const float gap = ImGui::GetStyle().ItemSpacing.x, w3 = (ImGui::GetContentRegionAvail().x-2*gap)/3, hb = px(60);
    const size_t total = project_.scans.size()+(hasLive ? 1 : 0);
    ImGui::BeginDisabled(!hasLive && !reviewing());   // already at the live image, with nothing to keep
    if (forge::toolTile(IC_PLUS, "Neuer Scan", {w3, hb})) newScan(hasLive);
    forge::noteItem("Neuer Scan Karte");
    ImGui::EndDisabled(); ImGui::SameLine(0, gap);
    ImGui::BeginDisabled(total < 2);
    if (forge::toolTile(IC_MOVE_3D, "Ausrichten", {w3, hb})) { if (!hasLive || commitLive()) openAlign(); }
    ImGui::EndDisabled(); ImGui::SameLine(0, gap);
    ImGui::BeginDisabled(total < 1 || (project_.scans.empty() && !hasLive));
    if (forge::toolTile(IC_COMBINE, "Fusion", {w3, hb})) { if (hasLive) commitLive(); openFusion(); }
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    ImGui::BeginDisabled(!canAct || jobRunning_);
    { const float w = (ImGui::GetContentRegionAvail().x-ImGui::GetStyle().ItemSpacing.x)/2;
      if (forge::button(IC_SAVE "  Speichern##proj", {w, 0})) { if (liveFrames) commitLive(); if (project_.scans.empty()) say("Noch kein Scan im Projekt.", false); else requestSave(); }
      forge::noteItem("Speichern##proj"); ImGui::SameLine();
      if (forge::button(IC_FOLDER_OPEN "  Öffnen##proj", {w, 0})) requestOpen();
      forge::noteItem("Öffnen##proj"); }
    ImGui::EndDisabled();
    forge::endCard();
}
void ProjectUi::newScan(bool hasLive) {
    if (hasLive) {
        if (commitLive()) say("Gespeichert als "+project_.scans.back().name+". Objekt umlegen und den nächsten Scan starten.", true);
        else say("Dieser Scan war zu lang, um ihn im Speicher zu behalten.", false);
    }
    reviewScan_ = -1; reviewDirty_ = false;
}
void ProjectUi::reviewLast() { reviewScan_ = (int)project_.scans.size()-1; reviewDirty_ = reviewScan_ >= 0; }
void ProjectUi::showReview(int scan) { reviewScan_ = scan; reviewDirty_ = true; }
// The picture is prepared in the background, unless another job has to finish first.
void ProjectUi::updateReview() {
    if (!reviewDirty_ || jobRunning_ || aligning_) return;
    reviewDirty_ = false;
    if (reviewScan_ < 0 || reviewScan_ >= (int)project_.scans.size()) { reviewScan_ = -1; return; }
    ScanData* s = &project_.scans[reviewScan_]; const int shown = reviewScan_;
    auto upload = [this, shown] {
        if (shown != reviewScan_ || shown >= (int)project_.scans.size()) return;
        const ScanData& d = project_.scans[shown]; if (!d.cloud) return;
        reviewView_.setUpDirection(d.up); reviewView_.setPointCloud(*d.cloud, true, d.cloudSpacingMm);
    };
    if (s->cloud) upload();
    else startJob("Punktwolke wird vorbereitet", [s] { buildPointCloud(*s); }, upload);
}
void ProjectUi::reviewView() {
    const ImVec2 o = ImGui::GetCursorScreenPos(), r{ImGui::GetContentRegionAvail().x-px(10), ImGui::GetContentRegionAvail().y-px(10)};
    char title[120] = ""; if (reviewScan_ < (int)project_.scans.size() && reviewScan_ >= 0) { const ScanData& s = project_.scans[reviewScan_]; snprintf(title, sizeof title, "%s  ·  %zu Bilder", s.name.c_str(), s.frames.size()); }
    oneView(reviewView_, r, title, nullptr, nullptr);
    const float barW = px(220), barH = px(60); forge::beginFloating("##reviewbar", {o.x+(r.x-barW)/2, o.y+r.y-barH-px(16)}, {barW, barH});
    ImGui::SetCursorPos({px(7), px(7)}); ImGui::BeginDisabled(jobRunning_);
    if (forge::primaryButton(IC_PLUS "  Neuer Scan##r", {barW-px(14), barH-px(14)})) newScan(false);
    forge::noteItem("Neuer Scan");
    ImGui::EndDisabled(); forge::endFloating();
    if (jobRunning_) forge::busyOverlay(o, r, busyLabel().c_str(), busyProgress());
}
std::string ProjectUi::projectName() const { return std::filesystem::path(path_).stem().string(); }
void ProjectUi::closeProject() { project_ = Project{}; path_[0] = 0; dirty_ = false; reviewScan_ = -1; reviewDirty_ = false; closeAlign(); }
bool ProjectUi::createProject(const std::string& file, std::string& error) {
    std::error_code ec; std::filesystem::create_directories(std::filesystem::path(file).parent_path(), ec);
    if (ec) { error = "Der Ordner lässt sich nicht anlegen: "+ec.message(); return false; }
    Project empty; if (!saveProject(empty, file, error)) return false;
    project_ = Project{}; dirty_ = false; reviewScan_ = -1; reviewDirty_ = false; closeAlign(); snprintf(path_, sizeof path_, "%s", file.c_str());
    if (onFileChanged) onFileChanged(file);
    return true;
}
void ProjectUi::requestSave() {
    const std::string start = hasProject() ? std::string(path_) : defaultFolder+"/projekt.vfproj";
    filedialog::request(filedialog::Kind::Save, "Projekt speichern", start, {{"VoxelForge-Projekt", {"vfproj"}}}, [this](const std::string& f) { saveTo(f); });
}
void ProjectUi::requestOpen() {
    filedialog::request(filedialog::Kind::Open, "Projekt öffnen", hasProject() ? std::string(path_) : defaultFolder, {{"VoxelForge-Projekt", {"vfproj"}}}, [this](const std::string& f) { loadFile(f); });
}
void ProjectUi::saveTo(const std::string& file) {
    auto error = std::make_shared<std::string>(); auto ok = std::make_shared<bool>(false);
    // The finished model makes the best picture. Copied now, as it may change while this runs.
    std::shared_ptr<Mesh> model; if (edit_.hasMesh()) model = std::make_shared<Mesh>(edit_.mesh());
    std::error_code ec; std::filesystem::create_directories(std::filesystem::path(file).parent_path(), ec);
    startJob("Projekt wird gespeichert", [this, file, error, ok, model] {
        *ok = saveProject(project_, file, *error);
        if (*ok) { const std::string thumb = thumbnailFile(file); if (!model || !writeThumbnail(*model, V3{0, 0, 1}, thumb)) writeProjectThumbnail(project_, thumb); }
    }, [this, file, error, ok] {
        if (*ok) { snprintf(path_, sizeof path_, "%s", file.c_str()); dirty_ = false; say("Gespeichert: "+std::filesystem::path(file).filename().string(), true); if (onFileChanged) onFileChanged(file); }
        else say(*error, false);
    });
}
bool ProjectUi::loadFile(const std::string& path) {
    auto loaded = std::make_shared<Project>(); auto error = std::make_shared<std::string>(); auto ok = std::make_shared<bool>(false);
    startJob("Projekt wird geladen", [path, loaded, error, ok] { *ok = loadProject(*loaded, path, *error); }, [this, path, loaded, error, ok] {
        if (*ok) { project_ = std::move(*loaded); dirty_ = false; closeAlign(); reviewLast(); snprintf(path_, sizeof path_, "%s", path.c_str()); say("Geöffnet: "+std::filesystem::path(path).stem().string(), true); if (onFileChanged) onFileChanged(path); }
        else say(*error, false);
    });
    return true;
}
void ProjectUi::removeDialog() {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {px(20), px(18)}); ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, px(16)); ImGui::PushStyleColor(ImGuiCol_PopupBg, forge::surface);
    const bool open = ImGui::BeginPopupModal("Scan entfernen", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar);
    if (open) {
        ImGui::PushFont(forge::fontTitle); ImGui::TextUnformatted("Scan entfernen?"); ImGui::PopFont();
        if (removeIndex_ >= 0 && (size_t)removeIndex_ < project_.scans.size()) { ImGui::PushStyleColor(ImGuiCol_Text, forge::textDim); ImGui::Text("%s wird aus dem Projekt entfernt. Das lässt sich nicht rückgängig machen.", project_.scans[removeIndex_].name.c_str()); ImGui::PopStyleColor(); }
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
    project_.scans.erase(project_.scans.begin()+i); dirty_ = true; if (reviewScan_ >= 0) reviewLast();
    bool anyPlaced = false; for (const ScanData& s : project_.scans) anyPlaced |= s.placed;
    if (!anyPlaced && !project_.scans.empty()) { project_.scans[0].placed = true; project_.scans[0].placement = Pose{}; }
}

// ---- Alignment -----------------------------------------------------------------------------------
void ProjectUi::openAlign() {
    aligning_ = true; hasResult_ = false; proposals_.clear(); chosen_.assign(project_.scans.size(), 1); message.clear();
    fixedPoints_.clear(); movingPoints_.clear(); fixedScan_ = 0; movingScan_ = project_.scans.size() > 1 ? 1 : 0;
    startJob("Scans werden vorbereitet", [this] {
        const size_t n = project_.scans.size();
        #pragma omp parallel for schedule(dynamic)
        for (size_t i = 0; i < n; i++) buildOverview(project_.scans[i]);
    }, [this] { if (manual_) showManualViews(); else showPreview(true); });
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
        if ((size_t)index >= project_.scans.size() || !project_.scans[index].overview) return;
        v.setUpDirection(project_.scans[index].up);
        Mesh m = *project_.scans[index].overview; m.colors.clear(); computeNormals(m); v.setMesh(m, true);
    };
    show(view_, fixedScan_); show(viewB_, movingScan_);
}
void ProjectUi::runAuto() {
    std::vector<size_t> use; for (size_t i = 0; i < chosen_.size(); i++) if (chosen_[i]) use.push_back(i);
    if (use.size() < 2) { say("Mindestens zwei Scans auswählen.", false); return; }
    // A placed scan stays where it is and serves as the reference. Otherwise the first one does.
    size_t ref = use[0]; for (size_t i : use) if (project_.scans[i].placed) { ref = i; break; }
    reference_ = ref; message.clear(); const bool markers = byMarkers_;
    auto result = std::make_shared<std::map<size_t, Proposal>>(); auto failed = std::make_shared<std::string>();
    startJob(markers ? "Ausrichten über Marker" : "Ausrichten über Merkmale", [this, use, ref, markers, result, failed] {
        Mesh merged; std::vector<V3> mapMerged;
        const Pose refPose = project_.scans[ref].placed ? project_.scans[ref].placement : Pose{};
        { const float none[3] = {0, 0, 0}; appendMesh(merged, *project_.scans[ref].overview, refPose, none); merged.colors.clear(); computeNormals(merged); }
        for (const V3& v : project_.scans[ref].markers) mapMerged.push_back(refPose.apply(v));
        size_t done = 0;
        for (size_t i : use) {
            if (i == ref) continue;
            const ScanData& s = project_.scans[i]; Proposal pr; Mesh src = *s.overview; src.colors.clear(); computeNormals(src);
            if (markers) {
                Pose P; int matched = 0;
                if (!registerMarkers(s.markers, mapMerged, P, matched)) { *failed = s.name + ": zu wenige gemeinsame Marker. Merkmale oder Manuell versuchen."; return; }
                pr.fromMarkers = P; pr.markers = matched;
                const RegisterResult fine = refineRegistration(src, merged, P, 3.f); if (fine.ok && fine.overlap > 0.1f) { RegisterResult f = fine; f.confident = matched >= 5; pr.options.push_back(f); }
            } else {
                pr.options = registerAuto(src, merged, [this, done, &use](float f) { jobProgress_ = (done+f)/(use.size()-1); });
                if (pr.options.empty()) { *failed = s.name + ": keine passende Lage gefunden. Überlappen sich die Scans genug? Sonst Manuell versuchen."; return; }
            }
            const Pose P = pr.options.empty() ? pr.fromMarkers : pr.options[0].srcToDst;
            // Later scans can also align to this one.
            { const float none[3] = {0, 0, 0}; appendMesh(merged, *s.overview, P, none); merged.colors.clear(); computeNormals(merged); }
            for (const V3& v : s.markers) mapMerged.push_back(P.apply(v));
            (*result)[i] = std::move(pr); done++;
        }
    }, [this, result, failed] {
        if (!failed->empty()) { say(*failed, false); return; }
        proposals_ = std::move(*result); hasResult_ = true; showPreview(false);
    });
}
void ProjectUi::runManual() {
    if (fixedScan_ == movingScan_) { say("Zwei verschiedene Scans wählen.", false); return; }
    if (fixedPoints_.size() != movingPoints_.size() || fixedPoints_.size() < 3) { say("Mindestens drei Punktpaare setzen.", false); return; }
    std::vector<V3> dst; const ScanData& fix = project_.scans[fixedScan_]; const Pose fp = fix.placed ? fix.placement : Pose{};
    for (const V3& p : fixedPoints_) dst.push_back(fp.apply(p));
    Pose P; if (!registerPointPairs(movingPoints_, dst, P)) { say("Die Punkte liegen auf einer Linie. Weiter verteilt setzen.", false); return; }
    message.clear(); const size_t mov = (size_t)movingScan_, fx = (size_t)fixedScan_; reference_ = fx;
    auto pr = std::make_shared<Proposal>();
    startJob("Feinausrichtung", [this, P, pr, mov, fx, fp] {
        Mesh src = *project_.scans[mov].overview, dstM; src.colors.clear(); computeNormals(src);
        const float none[3] = {0, 0, 0}; appendMesh(dstM, *project_.scans[fx].overview, fp, none); dstM.colors.clear(); computeNormals(dstM);
        pr->fromMarkers = P;
        const RegisterResult fine = refineRegistration(src, dstM, P, 6.f); if (fine.ok) pr->options.push_back(fine);
    }, [this, pr, mov, fx] {
        proposals_.clear(); proposals_[mov] = std::move(*pr); hasResult_ = true;
        chosen_.assign(project_.scans.size(), 0); chosen_[mov] = chosen_[fx] = 1; manual_ = false; showPreview(true); manual_ = true;
    });
}
void ProjectUi::acceptAlign() {
    dirty_ = true;
    for (auto& e : proposals_) { ScanData& s = project_.scans[e.first]; s.placement = proposedPlacement(e.first); s.placed = true; }
    if (reference_ < project_.scans.size()) project_.scans[reference_].placed = true;
    say("Ausrichtung übernommen. Jetzt Fusion starten.", true);
    closeAlign();
}
void ProjectUi::alignPanel() {
    forge::beginCard("##align");
    if (forge::iconButton(IC_CHEVRON_LEFT, "Zurück zum Scan", "Zurück", false, px(30))) { closeAlign(); forge::endCard(); return; }
    ImGui::SameLine(0, px(6)); ImGui::PushFont(forge::fontTitle); ImGui::SetCursorPosY(ImGui::GetCursorPosY()+px(6)); ImGui::TextUnformatted("Ausrichten"); ImGui::PopFont();
    ImGui::BeginDisabled(jobRunning_);
    int kind = byMarkers_ ? 1 : 0; const char* const kinds[] = {"Merkmale", "Marker"};
    if (forge::segmented("kind", kind, kinds, 2)) { byMarkers_ = kind == 1; hasResult_ = false; proposals_.clear(); showPreview(false); }
    int how = manual_ ? 1 : 0; const char* const hows[] = {"Auto", "Manuell"};
    if (forge::segmented("how", how, hows, 2)) { manual_ = how == 1; hasResult_ = false; proposals_.clear(); fixedPoints_.clear(); movingPoints_.clear(); chosen_.assign(project_.scans.size(), 1); message.clear(); if (manual_) showManualViews(); else showPreview(true); }
    ImGui::EndDisabled();
    if (byMarkers_ && !manual_) forge::hint("Nur wenn das Objekt zu den Markern gleich liegt. Wurde es auf dem Drehteller umgelegt, Merkmale nehmen.");
    if (!manual_) {
        forge::hint("Liegen die Scans richtig, mischen sich ihre Farben fein. Große einfarbige Flächen bedeuten eine falsche Lage.");
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
                    const RegisterResult& r = p.options[p.current]; char b[48];
                    snprintf(b, sizeof b, "%.0f %%", 100*r.overlap); forge::labelValue("Überlappung", b); snprintf(b, sizeof b, "%.2f mm", r.rmsMm); forge::labelValue("Abstand", b);
                    if (r.confident) forge::chip("Eindeutig", forge::green, IC_CIRCLE_CHECK); else forge::chip("Nicht sicher, bitte prüfen", forge::amber, IC_TRIANGLE_ALERT);
                    if (p.options.size() > 1) {
                        char label[64]; snprintf(label, sizeof label, IC_ARROW_RIGHT "  Nächster Vorschlag (%zu von %zu)", p.current+1, p.options.size());
                        ImGui::BeginDisabled(jobRunning_);
                        if (forge::button(label, {-FLT_MIN, 0})) { p.current = (p.current+1)%p.options.size(); showPreview(false); }
                        forge::noteItem("Nächster Vorschlag");
                        ImGui::EndDisabled();
                    }
                } else if (p.markers) { char b[48]; snprintf(b, sizeof b, "%d", p.markers); forge::labelValue("Gemeinsame Marker", b); }
                ImGui::PopID();
            }
        }
    } else {
        std::vector<const char*> names; for (const ScanData& s : project_.scans) names.push_back(s.name.c_str());
        ImGui::BeginDisabled(jobRunning_);
        bool ch = false;
        ch |= forge::combo("Bleibt liegen", &fixedScan_, names.data(), (int)names.size());
        ch |= forge::combo("Wird bewegt", &movingScan_, names.data(), (int)names.size());
        if (ch) { fixedPoints_.clear(); movingPoints_.clear(); hasResult_ = false; proposals_.clear(); showManualViews(); }
        ImGui::EndDisabled();
        forge::hint("Abwechselnd dieselbe Stelle in beiden Bildern anklicken, mindestens drei weit verteilte Paare.");
        char b[80]; snprintf(b, sizeof b, "%zu links, %zu rechts", fixedPoints_.size(), movingPoints_.size()); forge::labelValue("Punkte", b);
        ImGui::BeginDisabled(jobRunning_ || (fixedPoints_.empty() && movingPoints_.empty()));
        if (forge::button(IC_UNDO_2 "  Letzten Punkt entfernen##p", {-FLT_MIN, 0})) { if (fixedPoints_.size() > movingPoints_.size()) fixedPoints_.pop_back(); else if (!movingPoints_.empty()) movingPoints_.pop_back(); else if (!fixedPoints_.empty()) fixedPoints_.pop_back(); message.clear(); }
        forge::noteItem("Letzten Punkt entfernen");
        ImGui::EndDisabled();
        if (hasResult_) for (auto& e : proposals_) if (!e.second.options.empty()) { const RegisterResult& r = e.second.options[0]; snprintf(b, sizeof b, "%.0f %%", 100*r.overlap); forge::labelValue("Überlappung", b); snprintf(b, sizeof b, "%.2f mm", r.rmsMm); forge::labelValue("Abstand", b); }
    }
    const float gap = ImGui::GetStyle().ItemSpacing.x, w = ImGui::GetContentRegionAvail().x, h = px(38);
    ImGui::BeginDisabled(jobRunning_);
    if (!hasResult_) {
        if (forge::primaryButton(IC_EYE "  Vorschau##al", {-FLT_MIN, h})) { if (manual_) runManual(); else runAuto(); }
        forge::noteItem("Vorschau##al");
        if (forge::button("Abbrechen##al", {-FLT_MIN, 0})) closeAlign();
    } else {
        if (forge::primaryButton(IC_CHECK "  Bestätigen##al", {-FLT_MIN, h})) acceptAlign();
        forge::noteItem("Bestätigen##al");
        if (forge::button("Zurück##al", {-FLT_MIN, 0})) { hasResult_ = false; proposals_.clear(); message.clear(); if (manual_) showManualViews(); else { chosen_.assign(project_.scans.size(), 1); showPreview(true); } }
    }
    ImGui::EndDisabled(); (void)gap; (void)w;
    forge::endCard();
}
void ProjectUi::pickInto(MeshView& view, const Mesh& mesh, float x, float y, std::vector<V3>& points) {
    const long t = view.pickTriangle((int)x, (int)y); if (t < 0 || (size_t)t >= mesh.triangleCount()) return;
    const V3 a = mesh.vertices[mesh.indices[3*t]], b = mesh.vertices[mesh.indices[3*t+1]], c = mesh.vertices[mesh.indices[3*t+2]];
    V3 o, d; view.ray(x, y, o, d); const V3 n = cross(b-a, c-a); const float den = dot(n, d);
    points.push_back(std::fabs(den) > 1e-9f ? o+d*(dot(n, a-o)/den) : (a+b+c)*(1.f/3));
    message.clear();
}
void ProjectUi::drawPoints(MeshView& view, const std::vector<V3>& pts, ImVec2 o, ImU32 color) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    for (size_t i = 0; i < pts.size(); i++) {
        float x, y, d; if (!view.project(pts[i], x, y, d)) continue;
        const ImVec2 p{o.x+x, o.y+y}; dl->AddCircleFilled(p, 6, color); char b[24]; snprintf(b, sizeof b, "%zu", i+1);
        const ImVec2 at{p.x+8, p.y-9}, ts = ImGui::CalcTextSize(b); dl->AddRectFilled({at.x-2, at.y-1}, {at.x+ts.x+2, at.y+ts.y+1}, IM_COL32(3, 12, 24, 220), 3); dl->AddText(at, color, b);
    }
}
void ProjectUi::oneView(MeshView& view, ImVec2 size, const char* title, const Mesh* pickMesh, std::vector<V3>* points) {
    const ImGuiIO& io = ImGui::GetIO(); const ImVec2 o = ImGui::GetCursorScreenPos(); ImDrawList* dl = ImGui::GetWindowDrawList();
    const int w = std::max(16, (int)size.x), h = std::max(16, (int)size.y);
    const GLuint tex = view.render(w, h);
    ImGui::PushID(title ? title : "view"); ImGui::InvisibleButton("##v", {(float)w, (float)h}, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle); ImGui::PopID();
    dl->AddImageRounded((ImTextureID)(intptr_t)tex, o, {o.x+w, o.y+h}, {0, 1}, {1, 0}, IM_COL32_WHITE, px(14));
    const bool hoveredOnly = ImGui::IsItemHovered(), hovered = hoveredOnly || ImGui::IsItemActive();
    if (hovered) {
        const bool picks = pickMesh && points;
        if (ImGui::IsMouseDown(ImGuiMouseButton_Middle) || ((!picks || io.KeyShift) && ImGui::IsMouseDown(ImGuiMouseButton_Left))) view.rotate(io.MouseDelta.x, io.MouseDelta.y);
        if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) view.pan(io.MouseDelta.x, io.MouseDelta.y);
        if (hoveredOnly && io.MouseWheel != 0) view.zoom(io.MouseWheel);
        if (hoveredOnly && picks && !io.KeyShift && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !jobRunning_) pickInto(view, *pickMesh, io.MousePos.x-o.x, io.MousePos.y-o.y, *points);
    }
    if (points) drawPoints(view, *points, o, forge::col(forge::orange));
    if (title) forge::chipAt(dl, {o.x+px(12), o.y+px(12)}, title, forge::text);
}
void ProjectUi::alignView() {
    const ImVec2 full = ImGui::GetContentRegionAvail(); const ImVec2 avail{full.x-px(10), full.y-px(10)}; const ImVec2 o = ImGui::GetCursorScreenPos();
    if (manual_ && !hasResult_) {
        const ScanData* a = (size_t)fixedScan_ < project_.scans.size() ? &project_.scans[fixedScan_] : nullptr; const ScanData* b = (size_t)movingScan_ < project_.scans.size() ? &project_.scans[movingScan_] : nullptr;
        if (a && b && a->overview && b->overview) {
            const float w = (avail.x-px(8))/2; const std::string ta = a->name+" bleibt liegen", tb = b->name+" wird bewegt";
            oneView(view_, {w, avail.y}, ta.c_str(), a->overview.get(), &fixedPoints_);
            ImGui::SameLine(0, px(8));
            oneView(viewB_, {w, avail.y}, tb.c_str(), b->overview.get(), &movingPoints_);
        }
    } else if (combined_ && !combined_->indices.empty()) oneView(view_, avail, nullptr, nullptr, nullptr);
    if (jobRunning_) { std::string label = busyLabel(); forge::busyOverlay(o, avail, label.c_str(), jobProgress_); }
}

// ---- Fusion --------------------------------------------------------------------------------------
void ProjectUi::fusionDialog() {
    const ImVec2 c = ImGui::GetMainViewport()->GetCenter(); ImGui::SetNextWindowPos(c, ImGuiCond_Appearing, {.5f, .5f}); ImGui::SetNextWindowSize({px(520), 0});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {px(20), px(18)}); ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, px(16)); ImGui::PushStyleColor(ImGuiCol_PopupBg, forge::surface);
    if (!ImGui::BeginPopupModal("Fusion-Batch-Prozess", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar)) { ImGui::PopStyleVar(2); ImGui::PopStyleColor(); return; }
    ImGui::PushFont(forge::fontTitle); ImGui::PushStyleColor(ImGuiCol_Text, forge::sky); ImGui::TextUnformatted(IC_COMBINE); ImGui::PopStyleColor(); ImGui::SameLine(0, px(8)); ImGui::TextUnformatted("Fusion"); ImGui::PopFont();
    forge::hint("Verschmilzt die gewählten Scans zu einer einzigen Oberfläche.");
    bool blocked = false, any = false;
    forge::beginCard("##fuslist");
    for (size_t i = 0; i < project_.scans.size(); i++) {
        ScanData& s = project_.scans[i]; ImGui::PushID((int)i); char label[96]; snprintf(label, sizeof label, "%s  ·  %zu Bilder  ·  %s", s.name.c_str(), s.frames.size(), s.placed ? "ausgerichtet" : "offen");
        forge::toggle(label, &s.included);
        if (s.included) { any = true; blocked |= !s.placed; }
        ImGui::PopID();
    }
    forge::endCard();
    forge::beginCard("##fusset");
    forge::combo("Auflösung", &fusionVoxel_, kVoxelNames, 4);
    forge::sliderInt("Rauschunterdrückung", &fusionNoise_, 0, 100, "%d %%");
    forge::toggle("Kleine Teile behalten", &fusionKeepParts_);
    forge::toggle("Bereinigen, fein ausrichten, beste Farbe", &fusionClean_);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Entfernt Fläche, durch die eine Kamera hindurchgesehen hat, etwa Stützen oder Fetzen.\nAusschalten, falls echte Teile des Objekts fehlen.");
    forge::endCard();
    if (blocked) {
        forge::chip("Offene Scans erst ausrichten oder abwählen", forge::amber, IC_TRIANGLE_ALERT);
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
    startJob("Fusion", [this, voxel, minWeight, keep, clean, out, error, ok] {
        MeshOptions o; o.minWeight = minWeight; o.removeFragments = !keep; o.cleanUp = clean; o.fillHolesMm = 0; o.keepTriangles = 1; o.upright = true;
        *ok = fuseProject(project_, voxel, o, *out, *error, [this](float f) { jobProgress_ = f; });
        if (*ok) repairManifold(*out);
    }, [this, voxel, out, error, ok] { if (*ok) edit_.newMesh(std::move(*out), voxel); else say(*error, false); });
}
bool ProjectUi::buildMerged(float voxelMm, int minWeight, bool keepAllParts, Mesh& out, std::string& error) {
    // Runs in a worker thread, with the scan tab locked by the caller.
    std::vector<size_t> open; size_t ref = project_.scans.size();
    for (size_t i = 0; i < project_.scans.size(); i++) { ScanData& s = project_.scans[i]; if (!s.included) continue; buildOverview(s); if (s.placed && ref == project_.scans.size()) ref = i; else if (!s.placed) open.push_back(i); }
    if (ref == project_.scans.size()) { error = "Kein ausgerichteter Scan ausgewählt."; return false; }
    Mesh merged; { const float none[3] = {0, 0, 0}; appendMesh(merged, *project_.scans[ref].overview, project_.scans[ref].placement, none); merged.colors.clear(); computeNormals(merged); }
    for (size_t i : open) {
        ScanData& s = project_.scans[i]; Mesh src = *s.overview; src.colors.clear(); computeNormals(src);
        const auto res = registerAuto(src, merged);
        // One click must not fuse a guess: a wrong placement would ruin the whole model without anyone noticing.
        if (res.empty() || !res[0].confident) { error = s.name + " lässt sich nicht sicher ausrichten. Bitte das Werkzeug Ausrichten benutzen und das Ergebnis prüfen."; return false; }
        s.placement = res[0].srcToDst; s.placed = true;
        const float none[3] = {0, 0, 0}; appendMesh(merged, *s.overview, s.placement, none); merged.colors.clear(); computeNormals(merged);
    }
    MeshOptions o; o.minWeight = minWeight; o.removeFragments = !keepAllParts; o.fillHolesMm = 0; o.keepTriangles = 1; o.upright = true;
    return fuseProject(project_, voxelMm, o, out, error);
}
