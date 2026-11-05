// VoxelForge main window: live view, scan control, mesh preview and export.
#define GLFW_INCLUDE_NONE
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <fstream>
#include <sstream>
#include "device.hpp"
#include "i18n.hpp"
#include "editui.hpp"
#include "filedialog.hpp"
#include "homeui.hpp"
#include "settings.hpp"
#include "logo_data.hpp"
#include "pipeline.hpp"
#include "projectui.hpp"
#include "theme.hpp"

namespace {
struct Shared {
    std::mutex m;
    std::vector<uint8_t> ir, depthRgb, preview;
    int irW = 0, irH = 0, depthW = 0, depthH = 0, prevW = 0, prevH = 0;
    bool newIr = false, newDepth = false, newPreview = false;
    PipelineStats stats;
    std::string status = tr("Suche Scanner ..."), deviceInfo;
    bool connected = false, colorAvailable = false;
    float fps = 0;
    int exposure = 3000;
    bool usingGpu = false; std::string gpuInfo;
    std::vector<ImVec2> markerSpots;   // in the image, 0..1
};
struct Commands {
    std::atomic<bool> quit{false}, scanning{false}, resetModel{false}, autoExposure{true}, turntable{true};
    std::atomic<float> regionScale{1.f};
    std::atomic<int> size{1};
    std::atomic<bool> removeBase{true};
    std::atomic<int> manualExposure{3000};
    std::atomic<float> voxel{0.5f};
    std::atomic<int> roll{0};
    std::atomic<int> compute{0};
};
void depthToRgb(const DepthMap& d, const DepthMap& used, std::vector<uint8_t>& rgb) {
    rgb.resize((size_t)d.width*d.height*3);
    #pragma omp parallel for
    for (int i = 0; i < d.width*d.height; i++) {
        float z = d.z[i]; uint8_t* o = rgb.data()+3*(size_t)i;
        if (z <= 0) { o[0] = 8; o[1] = 18; o[2] = 32; continue; }
        if (used.z[i] <= 0) { o[0] = 42; o[1] = 56; o[2] = 72; continue; }   // seen, but not part of the object
        float t = std::clamp((z-120.f)/(500.f-120.f), 0.f, 1.f); // near warm, far cool
        o[0] = (uint8_t)(255*std::clamp(1.5f-std::fabs(4*t-0.5f), 0.f, 1.f));
        o[1] = (uint8_t)(255*std::clamp(1.5f-std::fabs(4*t-1.8f), 0.f, 1.f));
        o[2] = (uint8_t)(255*std::clamp(1.5f-std::fabs(4*t-3.2f), 0.f, 1.f));
    }
}
// Keeps the projected dots bright but below saturation.
// Large objects should use the wide outer pair, but its depth coverage is about 1 % so far.
// All profiles stay on the close pair until that is solved.
CameraPair pairFor(int /*size*/) { return CameraPair::Close; }
void worker(Shared& sh, Commands& cmd, Pipeline& pipe) {
    OtterDevice dev; CameraPair activePair = CameraPair::Close; StereoFrame f; int misses = 0, frameCount = 0, currentExp = 3000;
    auto fpsT = std::chrono::steady_clock::now(); int fpsN = 0; float curVoxel = cmd.voxel;
    auto setStatus = [&](const std::string& s, bool conn) { std::lock_guard<std::mutex> g(sh.m); sh.status = s; sh.connected = conn; };
    while (!cmd.quit) {
        if (!dev.isOpen()) {
            std::string err;
            activePair = pairFor(cmd.size.load());
            dev.setCameraPair(activePair);
            dev.setColorEnabled(true);
            if (!dev.open(err)) { setStatus(err, false); std::this_thread::sleep_for(std::chrono::seconds(1)); continue; }
            if (!dev.startStreams(err)) { setStatus(err, false); dev.close(); std::this_thread::sleep_for(std::chrono::seconds(1)); continue; }
            dev.setIrExposure(currentExp);
            {
                ColorCalib cc; const bool haveColor = dev.colorRunning() && loadColorCalib(colorCalibPath(dev.serial()), cc);
                pipe.setColorCalib(haveColor ? cc : ColorCalib{});
                std::lock_guard<std::mutex> g(sh.m); sh.deviceInfo = dev.description(); sh.colorAvailable = haveColor;
            }
            setStatus(tr("Verbunden"), true); misses = 0;
        }
        if (pairFor(cmd.size.load()) != activePair) { dev.close(); continue; }   // changing pairs needs a stream restart
        if (!dev.nextPair(f, 700)) {
            if (++misses >= 3) { dev.close(); setStatus(tr("Verbindung zum Scanner verloren, versuche erneut ..."), false); }
            continue;
        }
        misses = 0;
        if (cmd.resetModel.exchange(false)) { float v = cmd.voxel; if (v != curVoxel) { pipe.options.voxelMm = v; curVoxel = v; pipe.setVoxelQuality(v); } pipe.reset(); }
        pipe.setTurntable(cmd.turntable); pipe.setRegionScale(cmd.regionScale); pipe.setProfile(profileFor((ScanSize)cmd.size.load())); pipe.options.removeBase = cmd.removeBase; pipe.options.roll = (ScanRoll)cmd.roll.load();
        pipe.stereo.compute_mode = (ComputeMode)cmd.compute.load();
        pipe.setScanning(cmd.scanning);
        pipe.process(f, dev.calib());
        frameCount++; fpsN++;
        if (cmd.autoExposure) {
            if (frameCount % 10 == 0) { int n = pipe.nextExposure(f, currentExp); if (n != currentExp) { currentExp = n; dev.setIrExposure(n); } }
        } else if (cmd.manualExposure != currentExp) { currentExp = cmd.manualExposure; dev.setIrExposure(currentExp); }
        std::vector<uint8_t> rgb; depthToRgb(pipe.depth(), pipe.objectDepth(), rgb);
        std::vector<uint8_t> prev; bool hasPrev = false;
        if (cmd.scanning && frameCount % 2 == 0) {
            std::vector<uint8_t> gray; pipe.renderPreview(640, 400, gray); prev.resize(gray.size()*4);
            for (size_t i = 0; i < gray.size(); i++) {   // empty stays transparent, the surface is a cool steel
                const int v = gray[i]; uint8_t* o = prev.data()+4*i;
                o[0] = (uint8_t)(v*.80f); o[1] = (uint8_t)(v*.88f); o[2] = (uint8_t)v; o[3] = v ? 255 : 0;
            }
            hasPrev = true;
        }
        auto now = std::chrono::steady_clock::now();
        float el = std::chrono::duration<float>(now-fpsT).count();
        std::lock_guard<std::mutex> g(sh.m);
        sh.ir.swap(f.ref); sh.irW = f.width; sh.irH = f.height; sh.newIr = true;
        sh.depthRgb.swap(rgb); sh.depthW = pipe.depth().width; sh.depthH = pipe.depth().height; sh.newDepth = true;
        if (hasPrev) { sh.preview.swap(prev); sh.prevW = 640; sh.prevH = 400; sh.newPreview = true; }
        sh.stats = pipe.stats(); sh.exposure = currentExp; sh.usingGpu = pipe.stereo.usingGpu(); sh.gpuInfo = pipe.stereo.gpuInfo();
        sh.markerSpots.clear();
        { const DepthMap& dm = pipe.depth(); if (dm.width > 0) for (const Marker3D& m : pipe.detectedMarkers()) if (m.pos.z > 0) sh.markerSpots.push_back({(m.pos.x/m.pos.z*dm.fx+dm.cx)/dm.width, (m.pos.y/m.pos.z*dm.fy+dm.cy)/dm.height}); }
        if (el >= 0.5f) { sh.fps = fpsN/el; fpsN = 0; fpsT = now; }
    }
    dev.close();
}

using forge::primaryButton;

struct Tex { GLuint id = 0; int w = 0, h = 0; };
void upload(Tex& t, const uint8_t* data, int w, int h, GLenum fmt) {
    if (!t.id) { glGenTextures(1, &t.id); }
    glBindTexture(GL_TEXTURE_2D, t.id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    if (fmt == GL_RED) { GLint sw[4] = {GL_RED, GL_RED, GL_RED, GL_ONE}; glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, sw); }
    glTexImage2D(GL_TEXTURE_2D, 0, fmt == GL_RED ? GL_R8 : fmt == GL_RGBA ? GL_RGBA8 : GL_RGB8, w, h, 0, fmt, GL_UNSIGNED_BYTE, data);
    t.w = w; t.h = h;
}

void writeScreenshot(const char* path, int w, int h) {
    std::vector<uint8_t> px((size_t)w*h*3); glBindFramebuffer(GL_FRAMEBUFFER, 0); glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, px.data());
    FILE* f = fopen(path, "wb"); if (!f) return; fprintf(f, "P6 %d %d 255\n", w, h);
    for (int y = h-1; y >= 0; y--) fwrite(px.data()+(size_t)y*w*3, 1, w*3, f);
    fclose(f);
}

// Runs a list of user actions through the normal input path, so the whole window can be checked
// without a person: VOXELFORGE_SCRIPT=file. Lines: wait N, idle, press LABEL, move X Y, down, up,
// rdown, rup, wheel N, key NAME, ctrl 0|1, shift 0|1, say TEXT, state, shot PATH, quit.
class Script {
public:
    explicit Script(const char* path) { std::ifstream f(path); std::string l; while (std::getline(f, l)) if (!l.empty() && l[0] != '#') lines_.push_back(l); }
    void step(ImGuiIO& io, EditUi& edit, ProjectUi& project, GLFWwindow* win, int dw, int dh) {
        if (hasPos_) io.AddMousePosEvent(x_, y_);
        if (wait_ > 0) { wait_--; return; }
        if (release_) { io.AddMouseButtonEvent(0, false); release_ = false; wait_ = 3; return; }
        if (pos_ >= lines_.size()) { glfwSetWindowShouldClose(win, 1); return; }
        std::istringstream in(lines_[pos_]); std::string c; in >> c; std::string rest; std::getline(in, rest); if (!rest.empty() && rest[0] == ' ') rest.erase(0, 1);
        if (c == "idle") { if (edit.busy() || project.busy()) return; }
        else if (c == "wait") wait_ = std::atoi(rest.c_str());
        else if (c == "press") {
            auto& known = forge::buttonPositions(); auto it = known.find(rest); if (it == known.end()) it = known.find(tr(rest.c_str()));   // some carry the translated name
            if (it == known.end()) { printf("SKRIPT: unbekannte Schaltfläche %s\n", rest.c_str()); fflush(stdout); }
            else if (!armed_) { setPos(io, it->second.x, it->second.y); armed_ = true; wait_ = 2; return; }
            else { io.AddMouseButtonEvent(0, true); armed_ = false; release_ = true; wait_ = 1; pos_++; return; }
        }
        else if (c == "move") { float x = 0, y = 0; sscanf(rest.c_str(), "%f %f", &x, &y); setPos(io, x, y); wait_ = 2; }
        else if (c == "down") { io.AddMouseButtonEvent(0, true); wait_ = 2; }
        else if (c == "up") { io.AddMouseButtonEvent(0, false); wait_ = 2; }
        else if (c == "rdown") { io.AddMouseButtonEvent(1, true); wait_ = 2; }
        else if (c == "rup") { io.AddMouseButtonEvent(1, false); wait_ = 2; }
        else if (c == "wheel") { io.AddMouseWheelEvent(0, (float)std::atof(rest.c_str())); wait_ = 2; }
        else if (c == "ctrl") { io.AddKeyEvent(ImGuiMod_Ctrl, rest == "1"); wait_ = 1; }
        else if (c == "shift") { io.AddKeyEvent(ImGuiMod_Shift, rest == "1"); wait_ = 1; }
        else if (c == "key") { const ImGuiKey k = rest == "del" ? ImGuiKey_Delete : rest == "z" ? ImGuiKey_Z : rest == "y" ? ImGuiKey_Y : rest == "space" ? ImGuiKey_Space : rest == "f1" ? ImGuiKey_F1 : rest == "1" ? ImGuiKey_1 : rest == "2" ? ImGuiKey_2 : ImGuiKey_Escape; io.AddKeyEvent(k, true); io.AddKeyEvent(k, false); wait_ = 2; }
        else if (c == "type") { io.AddInputCharactersUTF8(rest.c_str()); wait_ = 3; }
        else if (c == "say") { printf("%s\n", rest.c_str()); fflush(stdout); }
        else if (c == "state") { printf("[%s] %s\n", rest.c_str(), edit.describe().c_str()); fflush(stdout); }
        else if (c == "project") { printf("[%s] %s\n", rest.c_str(), project.describe().c_str()); fflush(stdout); }
        else if (c == "shot") { pendingShot_ = rest; }
        else if (c == "quit") { glfwSetWindowShouldClose(win, 1); }
        else if (c == "close") { glfwSetWindowShouldClose(win, 1); wait_ = 10; }
        pos_++;
    }
    // After the frame is drawn.
    void afterDraw(int dw, int dh) { if (!pendingShot_.empty()) { writeScreenshot(pendingShot_.c_str(), dw, dh); pendingShot_.clear(); } }
private:
    void setPos(ImGuiIO& io, float x, float y) { x_ = x; y_ = y; hasPos_ = true; io.AddMousePosEvent(x, y); }
    float x_ = 0, y_ = 0; bool hasPos_ = false;
    std::vector<std::string> lines_; size_t pos_ = 0; int wait_ = 5; bool armed_ = false, release_ = false; std::string pendingShot_;
};
}

// An image that the camera delivers mirrored, turned as the scanner is held so that up is up on screen.
static void addScanImage(ImDrawList* dl, GLuint tex, ImVec2 p, ImVec2 q, int roll) {
    const ImVec2 tl = p, tr{q.x, p.y}, br = q, bl{p.x, q.y};
    if (roll == (int)ScanRoll::Right) dl->AddImageQuad((ImTextureID)(intptr_t)tex, tl, tr, br, bl, {1, 1}, {1, 0}, {0, 0}, {0, 1});
    else if (roll == (int)ScanRoll::Left) dl->AddImageQuad((ImTextureID)(intptr_t)tex, tl, tr, br, bl, {0, 0}, {0, 1}, {1, 1}, {1, 0});
    else dl->AddImage((ImTextureID)(intptr_t)tex, p, q, {1, 0}, {0, 1});
}
// The live image, with what the program sees marked on it.
static void pictureInPicture(ImDrawList* dl, GLuint tex, ImVec2 p, ImVec2 size, const char* title, int roll, const std::vector<ImVec2>* spots) {
    const float r = forge::px(10); const ImVec2 q{p.x+size.x, p.y+size.y};
    dl->AddRectFilled({p.x-1, p.y-1}, {q.x+1, q.y+1}, forge::col(forge::lineStrong), r+1);
    if (tex && roll == 0) dl->AddImageRounded((ImTextureID)(intptr_t)tex, p, q, ImVec2(1, 0), ImVec2(0, 1), IM_COL32_WHITE, r);
    else if (tex) addScanImage(dl, tex, p, q, roll);
    else dl->AddRectFilled(p, q, forge::col(forge::bg), r);
    if (spots) for (const ImVec2& s : *spots) {
        const float x = 1-s.x, y = s.y;   // mirrored as the image
        const ImVec2 at = roll == (int)ScanRoll::Right ? ImVec2(1-y, x) : roll == (int)ScanRoll::Left ? ImVec2(y, 1-x) : ImVec2(x, y);
        dl->AddCircle({p.x+at.x*size.x, p.y+at.y*size.y}, forge::px(6), forge::col(forge::green), 0, forge::px(1.6f));
    }
    ImGui::PushFont(forge::fontMedium); const ImVec2 ts = ImGui::CalcTextSize(title);
    dl->AddRectFilled({p.x+forge::px(6), p.y+forge::px(6)}, {p.x+forge::px(16)+ts.x, p.y+forge::px(12)+ts.y}, IM_COL32(8, 11, 16, 200), forge::px(6));
    dl->AddText({p.x+forge::px(11), p.y+forge::px(9)}, forge::col(forge::text), title); ImGui::PopFont();
}

int main() {
    if (!glfwInit()) { fprintf(stderr, tr("Grafik konnte nicht gestartet werden.\n")); return 1; }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    int winW = 1500, winH = 920; if (const char* size = getenv("VOXELFORGE_WINDOW")) sscanf(size, "%dx%d", &winW, &winH);
    // On Wayland the desktop finds name and icon through this id: it must match voxelforge.desktop.
    { const char* flatpakId = getenv("FLATPAK_ID"); glfwWindowHintString(GLFW_WAYLAND_APP_ID, flatpakId ? flatpakId : "voxelforge"); } glfwWindowHintString(GLFW_X11_CLASS_NAME, "voxelforge"); glfwWindowHintString(GLFW_X11_INSTANCE_NAME, "voxelforge");
    GLFWwindow* win = glfwCreateWindow(winW, winH, "VoxelForge", nullptr, nullptr);
    if (!win) { fprintf(stderr, tr("OpenGL 3.3 wird benötigt, Fenster konnte nicht erstellt werden.\n")); return 1; }
    glfwMakeContextCurrent(win); glfwSwapInterval(1);
    IMGUI_CHECKVERSION(); ImGui::CreateContext(); ImGuiIO& io = ImGui::GetIO(); io.IniFilename = nullptr;
    float xs = 1, ys = 1; glfwGetWindowContentScale(win, &xs, &ys);
    forge::loadFonts(std::max(1.f, xs)); forge::applyTheme();
    GLFWimage icon{kIcon64W, kIcon64H, const_cast<uint8_t*>(kIcon64)}; glfwSetWindowIcon(win, 1, &icon);   // Wayland ignores this and uses the desktop file
    ImGui_ImplGlfw_InitForOpenGL(win, true); ImGui_ImplOpenGL3_Init("#version 330 core");

    Shared sh; Commands cmd; Pipeline pipe;
    std::thread th;
    if (!getenv("VOXELFORGE_NO_DEVICE")) th = std::thread(worker, std::ref(sh), std::ref(cmd), std::ref(pipe));
    Tex texIr, texDepth, texPrev, texWord; MeshView view; view.init();
    glGenTextures(1, &texWord.id); glBindTexture(GL_TEXTURE_2D, texWord.id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, kWordmarkW, kWordmarkH, 0, GL_RGBA, GL_UNSIGNED_BYTE, kWordmark);
    enum class Tab { Home, Scan, Mesh, Measure } tab = Tab::Home;
    std::atomic<bool> meshBusy{false}, meshReady{false}; Mesh scanMesh; std::thread meshThread; float scanMeshVoxel = 0.5f;
    // What a new mesh starts with. Everything else happens in the tools or in the one-click process.
    MeshOptions meshOpt; meshOpt.upright = true; meshOpt.fillHolesMm = 0; meshOpt.keepTriangles = 1;
    std::atomic<int> integratedFrames{0};
    EditUi::ScanSource source;
    ProjectUi* projectPtr = nullptr;
    source.prepare = [&] { if (projectPtr->hasScans() && integratedFrames.load() > 0) projectPtr->commitLive(); };
    source.available = [&] { return integratedFrames.load() > 0 || projectPtr->hasScans(); };
    source.voxelMm = [&] { return cmd.voxel.load(); };
    source.build = [&](float voxel, int minWeight, bool keepAllParts, Mesh& out, std::string& err) {
        if (projectPtr->hasScans()) { if (!projectPtr->buildMerged(voxel, minWeight, keepAllParts, out, err)) return false; cmd.voxel = voxel; return true; }
        if (std::fabs(voxel-cmd.voxel.load()) > 1e-4f || pipe.options.voxelMm != voxel) { if (!pipe.rebuildModel(voxel, err)) return false; err.clear(); cmd.voxel = voxel; }
        MeshOptions o = meshOpt; o.minWeight = minWeight; if (keepAllParts) o.removeFragments = false;
        pipe.buildMesh(o, out);
        return true;
    };
    EditUi edit(view, source);
    edit.onMeshArrived = [&] { tab = Tab::Mesh; };
    edit.colorsExpected = [&] { std::lock_guard<std::mutex> g(sh.m); return sh.colorAvailable; };
    ProjectUi project(pipe, edit); project.init(); projectPtr = &project;
    edit.onError = [&](const std::string& e) { forge::toast(e, forge::Toast::Error); };
    Settings settings = loadSettings(settingsFile()); cmd.roll = settings.scanRoll; cmd.compute = settings.computeMode; if (getenv("VOXELFORGE_COMPUTE")) cmd.compute = (int)pipe.stereo.compute_mode; project.defaultFolder = settings.projectDir;
    HomeUi home(settings); home.init();
    bool scanning = false;
    // A new or opened project starts without anything left of the one before.
    auto startClean = [&] { scanning = false; cmd.scanning = false; cmd.resetModel = true; bool conn; { std::lock_guard<std::mutex> g(sh.m); conn = sh.connected; } if (!conn) pipe.reset(); edit.clear(); };
    project.onFileChanged = [&](const std::string& file) {
        rememberProject(settings, file); home.save(); home.refresh();
        const std::filesystem::path p(file); edit.setOutputBase((p.parent_path()/p.stem()).string(), p.parent_path().string());
        char title[300]; snprintf(title, sizeof title, "%s – VoxelForge", p.stem().c_str()); glfwSetWindowTitle(win, title);
    };
    home.onNewProject = [&](const std::string& file) { startClean(); std::string err; if (project.createProject(file, err)) tab = Tab::Scan; else forge::toast(err, forge::Toast::Error); };
    home.onOpenProject = [&](const std::string& file) { startClean(); project.closeProject(); project.loadFile(file); tab = Tab::Scan; };
    home.onComputeMode = [&](int m) { cmd.compute = m; };
    home.computeInfo = [&] { std::lock_guard<std::mutex> g(sh.m); if (!sh.connected) return std::string(tr("Wird beim Verbinden des Scanners geprüft.")); return std::string(sh.usingGpu ? tr("Stereo rechnet gerade auf: ") : tr("Stereo rechnet gerade auf der CPU. "))+(sh.gpuInfo == "Nicht verwendet" ? "" : tr(sh.gpuInfo.c_str())); };
    home.onOpenModel = [&](const std::string& file) { edit.loadFile(file); };
    home.onResume = [&] { tab = edit.hasMesh() && !project.hasScans() ? Tab::Mesh : Tab::Scan; };
    home.currentName = [&] { return project.hasProject() ? project.projectName() : std::string(); };
    home.hasUnsavedWork = [&] { return project.hasUnsaved() || scanning || integratedFrames.load() > 0; };
    edit.printCard = [&] {
        forge::beginCard("##print", tr("3D-Druck"), IC_PRINTER);
        const float gap = ImGui::GetStyle().ItemSpacing.x, w = (ImGui::GetContentRegionAvail().x-2*gap)/3; const ImVec2 tile{w, forge::px(82)}; const Slicer* sl = home.slicers();
        ImGui::BeginDisabled(!edit.hasMesh() || edit.busy());
        for (int i = 0; i < kSlicerCount; i++) {
            if (i) ImGui::SameLine(0, gap); const Slicer& s = sl[i];
            if (forge::imageTile(s.texture, s.letter, ImVec4(s.color[0], s.color[1], s.color[2], 1), s.name, tile, !s.found)) {
                if (!s.found) { forge::toast(trf("%s wurde nicht gefunden. Pfad in den Einstellungen angeben.", s.name), forge::Toast::Warning); home.requestSettings(); }
                else { const Slicer* one = &sl[i]; edit.exportForPrint([one](const std::string& file) { if (openInSlicer(*one, file)) forge::toast(trf("Wird in %s geöffnet", one->name), forge::Toast::Success); else forge::toast(trf("%s ließ sich nicht starten.", one->name), forge::Toast::Error); }); }
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", s.found ? trf("In %s öffnen", s.name).c_str() : tr("Nicht gefunden. Pfad in den Einstellungen angeben."));
        }
        ImGui::EndDisabled();
        forge::endCard();
    };
    if (const char* pf = getenv("VOXELFORGE_OPEN_PROJECT")) { project.loadFile(pf); tab = Tab::Scan; }
    int sizeChoice = 1, voxelChoice = 2, exposureUi = 3000, noiseUi = 15, rollUi = settings.scanRoll; bool autoExp = true, turntableUi = true, removeBaseUi = true; float regionUi = 1.f;
    if (getenv("VOXELFORGE_AUTOSCAN")) { scanning = true; cmd.scanning = true; }
    if (const char* preload = getenv("VOXELFORGE_OPEN")) edit.loadFile(preload);   // for checking without a scan
    std::unique_ptr<Script> script; if (const char* sp = getenv("VOXELFORGE_SCRIPT")) script = std::make_unique<Script>(sp);
    const char* shot = getenv("VOXELFORGE_SCREENSHOT"); const auto startT = std::chrono::steady_clock::now();
    const float shotAfter = getenv("VOXELFORGE_SCREENSHOT_AFTER") ? atof(getenv("VOXELFORGE_SCREENSHOT_AFTER")) : 5.f;
    std::vector<ImVec2> markerSpots;
    bool helpRequest = false;

    bool mayClose = false;
    while (true) {
        // Closing with scans that are not saved asks first, as starting another project does.
        if (glfwWindowShouldClose(win)) {
            if (mayClose || shot || !home.hasUnsavedWork || !home.hasUnsavedWork()) break;
            glfwSetWindowShouldClose(win, 0);
            scanning = false; cmd.scanning = false;
            home.guard([&] { mayClose = true; glfwSetWindowShouldClose(win, 1); });
        }
        glfwPollEvents();
        PipelineStats st; std::string status, info; bool connected, colorAvailable; float fps; int exposure;
        {
            std::lock_guard<std::mutex> g(sh.m);
            if (sh.newIr) { upload(texIr, sh.ir.data(), sh.irW, sh.irH, GL_RED); sh.newIr = false; }
            if (sh.newDepth) { upload(texDepth, sh.depthRgb.data(), sh.depthW, sh.depthH, GL_RGB); sh.newDepth = false; }
            if (sh.newPreview) { upload(texPrev, sh.preview.data(), sh.prevW, sh.prevH, GL_RGBA); sh.newPreview = false; }
            st = sh.stats; status = sh.status; info = sh.deviceInfo; connected = sh.connected; fps = sh.fps; exposure = sh.exposure; colorAvailable = sh.colorAvailable; markerSpots = sh.markerSpots;
        }
        integratedFrames = scanning ? 0 : st.integratedFrames;
        { const float vox[] = {0.15f, 0.25f, 0.5f, 1.f}; for (int i = 0; i < 4; i++) if (std::fabs(vox[i]-cmd.voxel.load()) < 1e-4f) voxelChoice = i; }   // the one-click process may have changed it
        if (meshReady.exchange(false)) {
            if (meshThread.joinable()) meshThread.join();
            if (!scanMesh.indices.empty()) { repairManifold(scanMesh); edit.newMesh(std::move(scanMesh), scanMeshVoxel); scanMesh = Mesh{}; }
            else forge::toast(tr("Der Scan ergibt noch kein Netz. Länger oder näher scannen."), forge::Toast::Warning);
        }
        ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame();
        if (script) { int dw0, dh0; glfwGetFramebufferSize(win, &dw0, &dh0); script->step(io, edit, project, win, dw0, dh0); }   // after the backend, which would overwrite the mouse
        ImGui::NewFrame();
        edit.frame(); project.frame(); home.dialogs();
        if (!ImGui::IsPopupOpen("Neues Projekt##dlg") && !ImGui::IsPopupOpen("Einstellungen##dlg")) filedialog::frame();
        ImGuiViewport* vp = ImGui::GetMainViewport();
        const float panelW = forge::px(340);
        const ImGuiWindowFlags fixed = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoBringToFrontOnFocus;
        const float topH = forge::px(56);

        // ---- Top bar: logo, navigation, state of the scanner.
        ImGui::SetNextWindowPos(vp->Pos); ImGui::SetNextWindowSize({vp->Size.x, topH});
        ImGui::PushStyleColor(ImGuiCol_WindowBg, forge::bg); ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {forge::px(16), forge::px(8)});
        ImGui::Begin("Leiste", nullptr, fixed | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        {
            const float rowH = topH-forge::px(16); const float logoH = forge::px(26);
            ImGui::SetCursorPosY((topH-logoH)/2); ImGui::Image((ImTextureID)(intptr_t)texWord.id, {logoH*kWordmarkW/kWordmarkH, logoH});
            ImGui::SameLine(panelW+forge::px(8)); ImGui::SetCursorPosY((topH-rowH)/2);
            ImGui::BeginDisabled(scanning);
            if (forge::navItem(IC_HOUSE, tr("Start"), tab == Tab::Home, rowH)) { home.refresh(); tab = Tab::Home; }
            ImGui::EndDisabled(); ImGui::SameLine(0, forge::px(4)); ImGui::SetCursorPosY((topH-rowH)/2);
            if (forge::navItem(IC_SCAN_LINE, tr("Scan"), tab == Tab::Scan, rowH)) { if (project.hasProject()) tab = Tab::Scan; else home.requestNew(); }
            ImGui::SameLine(0, forge::px(4)); ImGui::SetCursorPosY((topH-rowH)/2); ImGui::BeginDisabled(scanning);
            if (forge::navItem(IC_BOX, tr("Mesh-Verarbeitung"), tab == Tab::Mesh, rowH)) tab = Tab::Mesh;
            ImGui::SameLine(0, forge::px(4)); ImGui::SetCursorPosY((topH-rowH)/2); ImGui::BeginDisabled(!edit.hasMesh());
            if (forge::navItem(IC_RULER, tr("Schnellmessung"), tab == Tab::Measure, rowH)) tab = Tab::Measure;
            ImGui::EndDisabled(); ImGui::EndDisabled();
            // Right side: is the scanner there, and how fast does it run?
            bool usingGpu; { std::lock_guard<std::mutex> g(sh.m); usingGpu = sh.usingGpu; }
            char rate[160]; snprintf(rate, sizeof rate, "%.0f fps  ·  %.0f ms  ·  %s", fps, st.totalMs, usingGpu ? "GPU" : "CPU");
            ImDrawList* dl = ImGui::GetWindowDrawList(); ImGui::PushFont(forge::fontMedium);
            const char* devText = connected ? tr("Scanner verbunden") : tr("Kein Scanner");
            const float w1 = ImGui::CalcTextSize(devText).x+forge::px(46), w2 = ImGui::CalcTextSize(rate).x+forge::px(46); ImGui::PopFont();
            const float helpW = forge::px(34);
            ImGui::SetCursorScreenPos({vp->Pos.x+vp->Size.x-forge::px(12)-helpW, vp->Pos.y+(topH-helpW)/2});
            if (forge::iconButton(IC_INFO, tr("Hilfe"), tr("Tastenkürzel und Hinweise (F1)"), false, helpW)) helpRequest = true;
            ImGui::SetCursorScreenPos({vp->Pos.x+vp->Size.x-forge::px(16)-2*helpW, vp->Pos.y+(topH-helpW)/2});
            if (forge::iconButton(IC_SETTINGS, tr("Einstellungen"), tr("Speicherort und Slicer"), false, helpW)) home.requestSettings();
            float x = vp->Pos.x+vp->Size.x-forge::px(24)-2*helpW-w1; const float y = vp->Pos.y+(topH-forge::px(28))/2;
            forge::chipAt(dl, {x, y}, devText, connected ? forge::green : forge::textDim, connected ? IC_CIRCLE_CHECK : IC_CIRCLE_X);
            if (connected) forge::chipAt(dl, {x-w2-forge::px(4), y}, rate, forge::textDim, IC_ACTIVITY);
            if (!connected && !status.empty() && status != "Suche Scanner ...") { const ImVec2 mp = {x, y}; const ImVec2 mq{x+w1, y+forge::px(28)}; if (ImGui::IsMouseHoveringRect(mp, mq)) ImGui::SetTooltip("%s", tr(status.c_str())); }
            else if (connected && ImGui::IsMouseHoveringRect({x, y}, {x+w1, y+forge::px(28)})) ImGui::SetTooltip("%s%s", info.c_str(), colorAvailable ? tr("\nMit Farbe") : tr("\nOhne Farbkalibrierung"));
        }
        ImGui::End(); ImGui::PopStyleVar(); ImGui::PopStyleColor();

        // Keyboard, when no text is being typed.
        if (!io.WantTextInput && !ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId)) {
            if (ImGui::IsKeyPressed(ImGuiKey_F1, false)) helpRequest = true;
            if (ImGui::IsKeyPressed(ImGuiKey_1, false)) { if (project.hasProject()) tab = Tab::Scan; else home.requestNew(); }
            if (ImGui::IsKeyPressed(ImGuiKey_2, false) && !scanning) tab = Tab::Mesh;
            if (ImGui::IsKeyPressed(ImGuiKey_3, false) && !scanning && edit.hasMesh()) tab = Tab::Measure;
            if (ImGui::IsKeyPressed(ImGuiKey_Space, false) && tab == Tab::Scan && connected && !project.aligning() && !project.reviewing()) { scanning = !scanning; cmd.scanning = scanning; }
        }
        if (helpRequest) { helpRequest = false; ImGui::OpenPopup("Hilfe"); }
        { const ImVec2 c = vp->GetCenter(); ImGui::SetNextWindowPos(c, ImGuiCond_Appearing, {.5f, .5f}); ImGui::SetNextWindowSize({forge::px(500), 0});
          ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {forge::px(20), forge::px(18)}); ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, forge::px(16)); ImGui::PushStyleColor(ImGuiCol_PopupBg, forge::surface);
          if (ImGui::BeginPopupModal("Hilfe", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar)) {
              ImGui::PushFont(forge::fontTitle); ImGui::TextUnformatted(tr("Tastenkürzel")); ImGui::PopFont();
              forge::beginCard("##keys");
              const char* keys[][2] = {{tr("Leertaste"), tr("Scan starten oder anhalten")}, {"1, 2, 3", tr("Scan, Mesh-Verarbeitung, Schnellmessung")}, {tr("Strg+Z, Strg+Y"), tr("Rückgängig, Wiederholen")}, {tr("Entf"), tr("Auswahl löschen")}, {"Esc", tr("Auswahlwerkzeug verlassen")}, {tr("Umschalt+Links"), tr("Drehen, während ein Werkzeug wählt")}, {tr("Rechte Maustaste"), tr("Ansicht bewegen")}, {tr("Mausrad"), tr("Zoomen")}, {"F1", tr("Diese Hilfe")}};
              for (auto& k : keys) forge::labelValue(k[1], k[0]);
              forge::endCard();
              ImGui::PushFont(forge::fontTitle); ImGui::TextUnformatted(tr("Gute Scans")); ImGui::PopFont();
              forge::hint(tr("Den Drehteller ganz ins Bild bringen, damit mindestens 5 Marker sichtbar bleiben. Langsam drehen, eine Umdrehung in etwa 15 Sekunden. Für Farbe den Raum gut beleuchten. Für Details nah heran: Der Tiefenfehler wächst mit dem Quadrat des Abstands, 15 cm sind fast viermal genauer als 30 cm. Die Leiste unten zeigt den besten Bereich kräftig grün."));
              ImGui::PushFont(forge::fontTitle); ImGui::TextUnformatted(tr("Über VoxelForge")); ImGui::PopFont();
              forge::hint(trf("Version %s. Apache-2.0. Schrift Inter (SIL OFL 1.1), Symbole Lucide (ISC), Dear ImGui (MIT). „Creality“ und „CR-Scan“ sind Marken ihrer Inhaber, VoxelForge steht in keiner Verbindung zu ihnen.", VOXELFORGE_VERSION).c_str());
              if (forge::button("Schließen##hilfe", {-FLT_MIN, forge::px(36)})) ImGui::CloseCurrentPopup();
              forge::noteItem("Schließen");
              ImGui::EndPopup();
          }
          ImGui::PopStyleVar(2); ImGui::PopStyleColor(); }

        if (tab == Tab::Home) {
            ImGui::SetNextWindowPos({vp->Pos.x, vp->Pos.y+topH}); ImGui::SetNextWindowSize({vp->Size.x, vp->Size.y-topH});
            ImGui::PushStyleColor(ImGuiCol_WindowBg, forge::bg); ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0, 0});
            ImGui::Begin("Start", nullptr, fixed | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            home.page();
            ImGui::End(); ImGui::PopStyleVar(); ImGui::PopStyleColor();
        } else {
        // ---- Left panel.
        ImGui::SetNextWindowPos({vp->Pos.x, vp->Pos.y+topH}); ImGui::SetNextWindowSize({panelW, vp->Size.y-topH});
        ImGui::PushStyleColor(ImGuiCol_WindowBg, forge::bg); ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {forge::px(12), forge::px(6)});
        ImGui::Begin("Steuerung", nullptr, fixed);
        if (tab == Tab::Mesh) edit.panel();
        else if (tab == Tab::Measure) edit.measureSidePanel();
        else if (project.aligning()) project.alignPanel();
        else {
            const bool canSetUp = !scanning && connected;
            forge::beginCard("##setup", tr("Aufnahme"), IC_SETTINGS_2);
            ImGui::BeginDisabled(!canSetUp);
            int moves = turntableUi ? 0 : 1; const char* const movesN[] = {tr("Drehteller"), tr("Handbetrieb")};
            if (forge::segmented("moves", moves, movesN, 2)) { turntableUi = moves == 0; cmd.turntable = turntableUi; cmd.resetModel = true; }
            const char* sizes[] = {tr("Klein (11 bis 30 cm)"), tr("Mittel (12 bis 40 cm)"), tr("Groß (18 bis 120 cm)")};
            if (forge::combo(tr("Objektgröße"), &sizeChoice, sizes, 3)) { cmd.size = sizeChoice; cmd.resetModel = true; }
            const char* res[] = {tr("Sehr fein (0,15 mm)"), tr("Fein (0,25 mm)"), tr("Normal (0,5 mm)"), tr("Grob (1 mm)")};
            if (forge::combo(tr("Auflösung"), &voxelChoice, res, 4)) {
                const float vox[] = {0.15f, 0.25f, 0.5f, 1.f}; cmd.voxel = vox[voxelChoice];
                if (st.integratedFrames == 0) cmd.resetModel = true;   // an existing scan is only rebuilt on request
            }
            ImGui::TextColored(forge::textDim, tr("Scanner-Haltung"));
            if (ImGui::IsItemHovered()) ImGui::SetTooltip(tr("Wie der Scanner gehalten wird. Hochkant: Bild und Oben des Modells drehen mit.\nRechts oder links: wohin die Oberseite des Scanners von hinten gesehen zeigt."));
            const char* const rolls[] = {tr("Quer"), tr("Hochkant rechts"), tr("Hochkant links")};
            if (forge::segmented("roll", rollUi, rolls, 3)) { cmd.roll = rollUi; settings.scanRoll = rollUi; home.save(); }
            ImGui::EndDisabled();
            if (forge::collapsible(tr("Erweitert"))) {
                if (turntableUi && forge::slider(tr("Objektbereich"), &regionUi, 0.5f, 2.5f, "%.1f x")) cmd.regionScale = regionUi;
                if (forge::toggle(tr("Unterlage entfernen"), &removeBaseUi)) { cmd.removeBase = removeBaseUi; cmd.resetModel = true; }
                if (forge::toggle(tr("Belichtung automatisch"), &autoExp)) cmd.autoExposure = autoExp;
                if (autoExp) exposureUi = exposure;
                ImGui::BeginDisabled(autoExp);
                if (forge::sliderInt(tr("Belichtung"), &exposureUi, 100, 14000)) cmd.manualExposure = exposureUi;
                ImGui::EndDisabled();
            }
            forge::endCard();
            if (scanning || st.integratedFrames > 0) {
                forge::beginCard("##live", tr("Aktueller Scan"), IC_ACTIVITY);
                char b[160];
                snprintf(b, sizeof b, "%d", st.integratedFrames); forge::labelValue(tr("Bilder"), b);
                snprintf(b, sizeof b, "%d", st.lostFrames); forge::labelValue(tr("Verloren"), b, st.lostFrames > 0 ? forge::amber : forge::text);
                snprintf(b, sizeof b, tr("%d im Bild, %d bekannt"), st.markersMatched, st.markersInMap); forge::labelValue(tr("Marker"), b, st.markerPose ? forge::text : forge::amber);
                if (st.state == TrackState::Good) { snprintf(b, sizeof b, "%.2f mm", st.rmsMm); forge::labelValue(tr("Abweichung"), b); }
                snprintf(b, sizeof b, "%.0f MB", st.memoryBytes/1e6); forge::labelValue(tr("Speicher"), b);
                if (forge::button(IC_TRASH_2 "  Verwerfen und neu beginnen", {-FLT_MIN, 0})) { scanning = false; cmd.scanning = false; cmd.resetModel = true; }
                forge::endCard();
            }
            project.panel(!scanning && !meshBusy && !edit.busy(), scanning ? 0 : st.integratedFrames, !scanning && !meshBusy && !edit.busy(), st.integratedFrames > 0);
            forge::beginCard("##mesh", tr("Netz"), IC_BOX);
            auto makeMesh = [&](bool rebuild) {
                if (meshThread.joinable()) meshThread.join();
                meshBusy = true; const float v = cmd.voxel; MeshOptions mo = meshOpt; mo.minWeight = 1+(noiseUi*7+50)/100; scanMeshVoxel = v;
                meshThread = std::thread([&, v, mo, rebuild] { std::string r; if (!rebuild || pipe.rebuildModel(v, r)) pipe.buildMesh(mo, scanMesh); else forge::toast(r, forge::Toast::Error); meshBusy = false; meshReady = true; });
            };
            const bool inProject = project.hasScans(), idle = !scanning && !meshBusy && !edit.busy() && !project.busy();
            ImGui::BeginDisabled(!idle || (st.integratedFrames == 0 && !inProject));
            const float big = forge::px(40);
            if (forge::primaryButton(meshBusy ? "Berechne ...##m" : tr("Netz erstellen"), {-FLT_MIN, big})) {
                if (inProject) { if (st.integratedFrames > 0) project.commitLive(); project.openFusion(); }   // several scans: choose in the dialog
                else makeMesh(false);
            }
            if (forge::button(IC_SPARKLES "  Ein-Klick-Prozess", {-FLT_MIN, 0})) edit.openOneClick();
            forge::noteItem("Ein-Klick-Prozess");
            if (!inProject) { const char* resLabel[] = {tr("0,15 mm"), tr("0,25 mm"), tr("0,5 mm"), "1 mm"}; char rebuildLabel[160]; snprintf(rebuildLabel, sizeof rebuildLabel, tr(IC_ROTATE_CW "  Neu berechnen mit %s"), resLabel[voxelChoice]); if (forge::button(rebuildLabel, {-FLT_MIN, 0})) makeMesh(true); }
            ImGui::EndDisabled();
            if (forge::collapsible(tr("Netz-Einstellungen"))) {
                forge::toggle(tr("Kleine Reste entfernen"), &meshOpt.removeFragments);
                forge::toggle(tr("Bereinigen, beste Farbe"), &meshOpt.cleanUp);
                if (ImGui::IsItemHovered()) ImGui::SetTooltip(tr("Entfernt, wodurch eine Kamera hindurchgesehen hat, richtet die Bilder fein aus\nund nimmt die Farbe aus den besten Ansichten."));
                forge::sliderInt(tr("Rauschunterdrückung"), &noiseUi, 0, 100, "%d %%");
                forge::sliderInt(tr("Vorglättung"), &meshOpt.smoothIterations, 0, 20);
            }
            forge::endCard();
        }
        ImGui::End(); ImGui::PopStyleVar(); ImGui::PopStyleColor();

        // ---- The view.
        ImGui::SetNextWindowPos({vp->Pos.x+panelW, vp->Pos.y+topH}); ImGui::SetNextWindowSize({vp->Size.x-panelW, vp->Size.y-topH});
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {forge::px(2), 0}); ImGui::PushStyleColor(ImGuiCol_WindowBg, forge::bg);
        ImGui::Begin("Ansicht", nullptr, fixed | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {forge::px(14), forge::px(14)});
        if (tab != Tab::Scan) edit.viewport(tab == Tab::Measure);
        else if (project.aligning()) project.alignView();
        else if (project.reviewing()) project.reviewView();
        else {
            const ImVec2 o = ImGui::GetCursorScreenPos(), r{ImGui::GetContentRegionAvail().x-forge::px(10), ImGui::GetContentRegionAvail().y-forge::px(10)}; const ImVec2 e{o.x+r.x, o.y+r.y};
            ImDrawList* dl = ImGui::GetWindowDrawList(); const float rnd = forge::px(14);
            // The model as it grows, on a dark stage.
            const int v0 = dl->VtxBuffer.Size; dl->AddRectFilled(o, e, IM_COL32_WHITE, rnd); ImGui::ShadeVertsLinearColorGradientKeepAlpha(dl, v0, dl->VtxBuffer.Size, o, {o.x, e.y}, IM_COL32(19, 24, 32, 255), IM_COL32(9, 12, 17, 255));
            if (texPrev.id && (scanning || st.integratedFrames > 0)) {
                const float aspect = rollUi ? 1/1.6f : 1.6f; float ph = r.y, pw = std::min(r.x, ph*aspect); ph = pw/aspect; const ImVec2 a{o.x+(r.x-pw)/2, o.y+(r.y-ph)/2};
                addScanImage(dl, texPrev.id, a, {a.x+pw, a.y+ph}, rollUi);
            } else {
                ImGui::PushFont(forge::fontIcon); const char* ic = connected ? IC_SCAN_LINE : IC_CAMERA; const ImVec2 is = ImGui::CalcTextSize(ic); dl->AddText(forge::fontIcon, forge::px(44), {o.x+(r.x-is.x*2.3f)/2, o.y+r.y/2-forge::px(66)}, forge::col(forge::textFaint), ic); ImGui::PopFont();
                ImGui::PushFont(forge::fontTitle); const char* t1 = connected ? tr("Bereit zum Scannen") : tr("Scanner anschließen"); const ImVec2 ts = ImGui::CalcTextSize(t1); dl->AddText({o.x+(r.x-ts.x)/2, o.y+r.y/2}, forge::col(forge::text), t1); ImGui::PopFont();
                const char* t2 = connected ? tr("Objekt mittig auf den Teller stellen und Scan starten.") : tr("Der CR-Scan Otter wird automatisch erkannt."); const ImVec2 t2s = ImGui::CalcTextSize(t2); dl->AddText({o.x+(r.x-t2s.x)/2, o.y+r.y/2+forge::px(26)}, forge::col(forge::textDim), t2);
            }
            // Live images, floating top right.
            if (connected) {
                const float m = forge::px(12);
                float w = std::min(forge::px(250), r.x*.24f), h = w*.625f;
                if (rollUi) { h = std::min(forge::px(250), (r.y-forge::px(110))/2-m); w = h*.625f; }   // upright pictures: narrower, and both must still fit
                pictureInPicture(dl, texIr.id, {e.x-w-m, o.y+m}, {w, h}, tr("Infrarot"), rollUi, &markerSpots);
                pictureInPicture(dl, texDepth.id, {e.x-w-m, o.y+m*2+h}, {w, h}, tr("Tiefe"), rollUi, nullptr);
            }
            // State, top left.
            ImVec2 hud{o.x+forge::px(12), o.y+forge::px(12)}; const float line = forge::px(34);
            if (connected) {
                char b[160];
                if (scanning) {
                    if (st.state == TrackState::Lost) forge::chipAt(dl, hud, tr("Position verloren: zu einer bekannten Stelle zurück"), forge::danger, IC_CIRCLE_ALERT);
                    else if (st.poseUnconfirmed) forge::chipAt(dl, hud, tr("Position wird geprüft ..."), forge::amber, IC_CROSSHAIR);
                    else if (st.state == TrackState::Waiting) forge::chipAt(dl, hud, tr("Warte auf Oberfläche im Bild"), forge::amber, IC_FOCUS);
                    else forge::chipAt(dl, hud, tr("Tracking gut"), forge::green, IC_CIRCLE_CHECK);
                    hud.y += line;
                }
                snprintf(b, sizeof b, tr("%d Marker im Bild"), st.markersDetected);
                const bool fewMarkers = st.markersDetected < 5;
                forge::chipAt(dl, hud, b, fewMarkers ? forge::amber : forge::textDim, IC_TARGET); hud.y += line;
                if (st.baseFound) { forge::chipAt(dl, hud, tr("Unterlage erkannt"), forge::textDim, IC_LAYERS); hud.y += line; }
                if (scanning && st.turnDegPerFrame > 6.f) { forge::chipAt(dl, hud, tr("Langsamer drehen"), forge::amber, IC_TRIANGLE_ALERT); hud.y += line; }
                if (fewMarkers && turntableUi && st.medianDepth > 0) { forge::chipAt(dl, hud, tr("Teller ganz ins Bild rücken, mindestens 5 Marker"), forge::amber, IC_INFO); hud.y += line; }
            }
            // Main action and distance, floating at the bottom.
            const float barW = std::min(forge::px(460), r.x-forge::px(40)), barH = forge::px(60); const ImVec2 bp{o.x+(r.x-barW)/2, e.y-barH-forge::px(16)};
            forge::beginFloating("##scanbar", bp, {barW, barH});
            {
                const float btnW = forge::px(170), btnH = barH-forge::px(14);
                ImGui::SetCursorPos({forge::px(7), forge::px(7)});
                ImGui::BeginDisabled(!connected);
                if (!scanning) { if (forge::primaryButton(st.integratedFrames > 0 ? IC_PLAY "  Scan fortsetzen##s" : IC_PLAY "  Scan starten##s", {btnW, btnH})) { scanning = true; cmd.scanning = true; } forge::noteItem(st.integratedFrames > 0 ? tr("Scan fortsetzen") : tr("Scan starten")); forge::noteItem("Scan starten"); }
                else { if (forge::dangerButton(IC_SQUARE "  Scan anhalten##s", {btnW, btnH})) { scanning = false; cmd.scanning = false; } forge::noteItem("Scan anhalten"); }
                ImGui::EndDisabled();
                // Distance: where the object is within the working range.
                const ScanProfile prof = profileFor((ScanSize)sizeChoice); ImDrawList* bl = ImGui::GetWindowDrawList(); const ImVec2 wp = ImGui::GetWindowPos();
                const float gx0 = wp.x+btnW+forge::px(22), gx1 = wp.x+barW-forge::px(18), gy = wp.y+barH*.66f; char b[160];
                const float zLo = prof.zMin*.7f, zHi = prof.zMax*1.2f; auto toX = [&](float z) { return gx0+(gx1-gx0)*std::clamp((z-zLo)/(zHi-zLo), 0.f, 1.f); };
                bl->AddRectFilled({gx0, gy-forge::px(2)}, {gx1, gy+forge::px(2)}, forge::col(forge::hover), forge::px(2));
                bl->AddRectFilled({toX(prof.zMin), gy-forge::px(2)}, {toX(prof.zMax), gy+forge::px(2)}, forge::col(forge::green, .22f), forge::px(2));
                bl->AddRectFilled({toX(prof.bestMin), gy-forge::px(3)}, {toX(prof.bestMax), gy+forge::px(3)}, forge::col(forge::green, .9f), forge::px(3));
                const bool ok = st.medianDepth >= prof.zMin && st.medianDepth <= prof.zMax;
                if (st.medianDepth > 0) { bl->AddCircleFilled({toX(st.medianDepth), gy}, forge::px(6), forge::col(ok ? forge::text : forge::amber)); snprintf(b, sizeof b, tr("Abstand %.0f mm%s"), st.medianDepth, !ok ? (st.medianDepth < prof.zMin ? tr("  (zu nah)") : tr("  (zu weit)")) : st.medianDepth > prof.bestMax ? tr("  ·  näher ran für mehr Detail") : st.medianDepth >= prof.bestMin ? tr("  ·  beste Schärfe") : ""); }
                else snprintf(b, sizeof b, tr("Abstand: keine Oberfläche"));
                bl->AddText({gx0, wp.y+forge::px(11)}, forge::col(st.medianDepth > 0 && !ok ? forge::amber : forge::textDim), b);
            }
            forge::endFloating();
            if (meshBusy) forge::busyOverlay(o, r, tr("Netz wird berechnet"), -1);
            else if (project.busy()) forge::busyOverlay(o, r, project.busyLabel().c_str(), project.busyProgress());
            else if (edit.busy() && !edit.analysing()) forge::busyOverlay(o, r, edit.busyLabel().c_str(), edit.busyProgress());
        }
        ImGui::PopStyleVar();
        ImGui::End(); ImGui::PopStyleColor(); ImGui::PopStyleVar();
        }
        forge::drawToasts();

        ImGui::Render();
        int dw, dh; glfwGetFramebufferSize(win, &dw, &dh);
        glViewport(0, 0, dw, dh); glClearColor(forge::bg.x, forge::bg.y, forge::bg.z, 1); glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        if (shot && std::chrono::duration<float>(std::chrono::steady_clock::now()-startT).count() > shotAfter) { writeScreenshot(shot, dw, dh); glfwSetWindowShouldClose(win, 1); }
        if (script) script->afterDraw(dw, dh);
        glfwSwapBuffers(win);
    }
    cmd.quit = true; if (th.joinable()) th.join();
    if (meshThread.joinable()) meshThread.join();
    ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown(); ImGui::DestroyContext(); glfwDestroyWindow(win); glfwTerminate();
    return 0;
}
