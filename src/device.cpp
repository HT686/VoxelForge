#include <algorithm>
#include "device.hpp"
#include "i18n.hpp"
#include <fcntl.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>

namespace {
constexpr uint16_t kVendor = 0x2bc5, kProduct = 0x06da;
constexpr uint8_t kXuUnit = 4;
constexpr uint16_t kReqMagic = 0x4d47, kRespMagic = 0x4252;
enum { OP_GET_PROP = 1, OP_SET_PROP = 2, OP_GET_STRUCT = 3, OP_SET_STRUCT = 4, OP_RAW_INIT = 17, OP_RAW_READ = 18, OP_RAW_DONE = 19 };
enum { PROP_IR_MIRROR = 18, PROP_LASER = 3, PROP_IR_EXPOSURE = 2026, PROP_IR_SOURCE = 2028 };
enum { STRUCT_VERSION = 1000, STRUCT_DEPTH_MODE = 1043, RAW_DEPTH_CALIB = 4026, RAW_DEPTH_MODES = 4034 };
constexpr size_t kModeSize = 52, kModeNameOffset = 16, kModeNameLen = 32;

std::string readText(const std::filesystem::path& p) {
    std::ifstream f(p); std::string s; std::getline(f, s); return s;
}
std::string modeName(const uint8_t* rec) {
    return std::string((const char*)rec + kModeNameOffset, strnlen((const char*)rec + kModeNameOffset, kModeNameLen));
}
void appendU32(std::vector<uint8_t>& v, uint32_t x) { for (int i = 0; i < 4; i++) v.push_back((x >> (8*i)) & 0xff); }
}

OtterDevice::~OtterDevice() { close(); }

bool OtterDevice::xferXu(uint16_t opcode, const std::vector<uint8_t>& payload, std::vector<uint8_t>& resp, int respSelector) {
    uint16_t id = ++requestId_;
    std::vector<uint8_t> req(8 + payload.size());
    uint16_t hdr[4] = {kReqMagic, (uint16_t)(payload.size() / 2), opcode, id};
    memcpy(req.data(), hdr, 8);
    if (!payload.empty()) memcpy(req.data() + 8, payload.data(), payload.size());
    int sendSel = req.size() <= xuLen_[2] ? 2 : 1;
    if (req.size() > xuLen_[sendSel]) return false;
    req.resize(xuLen_[sendSel], 0);
    uvc_xu_control_query q{};
    q.unit = kXuUnit; q.selector = sendSel; q.query = UVC_SET_CUR; q.size = xuLen_[sendSel]; q.data = req.data();
    if (ioctl(fdDepth_, UVCIOC_CTRL_QUERY, &q) < 0) return false;
    std::vector<uint8_t> r(xuLen_[respSelector]);
    q.selector = respSelector; q.query = UVC_GET_CUR; q.size = xuLen_[respSelector]; q.data = r.data();
    if (ioctl(fdDepth_, UVCIOC_CTRL_QUERY, &q) < 0) return false;
    uint16_t rh[5]; memcpy(rh, r.data(), 10);
    if (rh[0] != kRespMagic || rh[2] != opcode || rh[3] != id || rh[4] != 0) return false;
    size_t n = rh[1] * 2u;              // size counts everything after the 8-byte header, incl. error word
    n = n >= 2 ? n - 2 : 0;
    if (n + 10 > r.size()) n = r.size() - 10;
    resp.assign(r.begin() + 10, r.begin() + 10 + n);
    return true;
}
bool OtterDevice::getProperty(uint32_t id, int32_t v[5]) {
    std::vector<uint8_t> p, r; appendU32(p, id);
    if (!xferXu(OP_GET_PROP, p, r, 1) || r.size() < 20) return false;
    memcpy(v, r.data(), 20); return true;
}
bool OtterDevice::setProperty(uint32_t id, int32_t value) {
    std::vector<uint8_t> p, r; appendU32(p, id); appendU32(p, (uint32_t)value);
    return xferXu(OP_SET_PROP, p, r, 1);
}
bool OtterDevice::readStruct(uint32_t id, std::vector<uint8_t>& out) {
    std::vector<uint8_t> p; appendU32(p, id);
    return xferXu(OP_GET_STRUCT, p, out, 1);
}
bool OtterDevice::readRaw(uint32_t id, std::vector<uint8_t>& out) {
    std::vector<uint8_t> p, r; appendU32(p, id);
    if (!xferXu(OP_RAW_INIT, p, r, 1) || r.size() < 4) return false;
    uint32_t size; memcpy(&size, r.data(), 4);
    out.clear();
    bool ok = true;
    while (out.size() < size) {
        uint32_t chunk = std::min<uint32_t>(512, size - (uint32_t)out.size());
        std::vector<uint8_t> q; appendU32(q, id); appendU32(q, (uint32_t)out.size()); appendU32(q, chunk);
        if (!xferXu(OP_RAW_READ, q, r, 3) || r.size() < chunk) { ok = false; break; }
        out.insert(out.end(), r.begin(), r.begin() + chunk);
    }
    xferXu(OP_RAW_DONE, p, r, 1);
    return ok;
}

bool OtterDevice::ensureBinocularMode(std::string& error) {
    const char* kWantedMode = pair_ == CameraPair::Far ? "Long_range_mode Binocular" : "Close_range_mode Binocular";
    std::vector<uint8_t> cur, list;
    if (!readStruct(STRUCT_DEPTH_MODE, cur) || cur.size() < kModeSize) { error = tr("Tiefenmodus nicht lesbar"); return false; }
    if (modeName(cur.data()) == kWantedMode) return true;
    if (!readRaw(RAW_DEPTH_MODES, list)) { error = tr("Modusliste nicht lesbar"); return false; }
    for (size_t i = 0; i + kModeSize <= list.size(); i += kModeSize) {
        if (modeName(list.data() + i) != kWantedMode) continue;
        std::vector<uint8_t> p, r; appendU32(p, STRUCT_DEPTH_MODE);
        p.insert(p.end(), list.begin() + i, list.begin() + i + kModeSize);
        if (!xferXu(OP_SET_STRUCT, p, r, 1)) { error = tr("Modus konnte nicht gesetzt werden"); return false; }
        usleep(300000);
        return true;
    }
    error = tr("Gerät kennt den Binocular-Modus nicht");
    return false;
}

bool OtterDevice::open(std::string& error) {
    namespace fs = std::filesystem;
    std::string nodeDepth, nodeIr, nodeColor;
    std::error_code ec;
    for (auto& e : fs::directory_iterator("/sys/class/video4linux", ec)) {
        fs::path dev = fs::canonical(e.path() / "device", ec);
        if (ec) continue;
        unsigned vid = std::stoul("0" + readText(dev.parent_path() / "idVendor"), nullptr, 16);
        unsigned pid = std::stoul("0" + readText(dev.parent_path() / "idProduct"), nullptr, 16);
        if (vid != kVendor || pid != kProduct || readText(e.path() / "index") != "0") continue;
        std::string itf = readText(dev / "bInterfaceNumber");
        if (itf == "00") nodeDepth = "/dev/" + e.path().filename().string();
        if (itf == "02") nodeIr = "/dev/" + e.path().filename().string();
        if (itf == "04") nodeColor = "/dev/" + e.path().filename().string();
    }
    if (nodeDepth.empty() || nodeIr.empty()) { error = tr("Kein CR-Scan Otter gefunden. Ist der Scanner per USB 3 angeschlossen?"); return false; }
    fdDepth_ = ::open(nodeDepth.c_str(), O_RDWR | O_NONBLOCK);
    fdIr_ = ::open(nodeIr.c_str(), O_RDWR | O_NONBLOCK);
    if (!nodeColor.empty()) fdColor_ = ::open(nodeColor.c_str(), O_RDWR | O_NONBLOCK);
    if (fdDepth_ < 0 || fdIr_ < 0) {
        error = trf("Kamerazugriff verweigert (%s). Gehört dein Benutzer zur Gruppe 'video'?", strerror(errno));
        close(); return false;
    }
    for (int sel = 1; sel <= 3; sel++) {
        uint8_t len[2] = {0, 0};
        uvc_xu_control_query q{};
        q.unit = kXuUnit; q.selector = (uint8_t)sel; q.query = UVC_GET_LEN; q.size = 2; q.data = len;
        if (ioctl(fdDepth_, UVCIOC_CTRL_QUERY, &q) < 0) { error = tr("Gerätesteuerung antwortet nicht. Bitte Scanner neu einstecken."); close(); return false; }
        xuLen_[sel] = len[0] | (len[1] << 8);
    }
    std::vector<uint8_t> ver;
    if (!readStruct(STRUCT_VERSION, ver) || ver.size() < 164) { error = tr("Geräteinformationen nicht lesbar"); close(); return false; }
    std::string fw((const char*)ver.data(), strnlen((const char*)ver.data(), 16));
    std::string serial((const char*)ver.data() + 80, strnlen((const char*)ver.data() + 80, 16));
    description_ = "CR-Scan Otter, Firmware " + fw + ", S/N " + serial;
    serial_ = serial;
    if (!readCalibration(error)) { close(); return false; }
    return true;
}
// The calibration belongs to the active camera pair, so read it again after every mode change.
bool OtterDevice::readCalibration(std::string& error) {
    std::vector<uint8_t> cal;
    if (!readRaw(RAW_DEPTH_CALIB, cal) || cal.size() < 28) { error = tr("Kalibrierung nicht lesbar"); return false; }
    float f[7]; memcpy(f, cal.data(), 28);
    calib_.baselineMm = f[1]; calib_.focalPx = f[3]; calib_.dispOffset = f[5];
    calib_.rowOffset = pair_ == CameraPair::Far ? 1 : 0;   // measured on a flat wall: constant over the whole image
    if (!(calib_.focalPx > 100 && calib_.baselineMm > 1 && calib_.dispOffset >= 0)) { error = tr("Kalibrierung unplausibel"); return false; }
    // All geometry assumes mirrored IR images. Refuse to produce silently reversed models otherwise.
    int32_t mirror[5];
    if (!getProperty(PROP_IR_MIRROR, mirror) || mirror[0] != 1) { error = tr("Unerwartete Spiegelung der IR-Bilder, bitte Scanner neu einstecken"); return false; }
    return true;
}

void OtterDevice::close() {
    // Only undo what this process did: another program may be streaming with the laser on.
    const bool wasStreaming = streaming_;
    stopStreams();
    if (fdDepth_ >= 0) { if (wasStreaming) setLaser(false); ::close(fdDepth_); }
    if (fdIr_ >= 0) ::close(fdIr_);
    if (fdColor_ >= 0) ::close(fdColor_);
    fdDepth_ = fdIr_ = fdColor_ = -1;
}

bool OtterDevice::setLaser(bool on) { return setProperty(PROP_LASER, on ? 1 : 0); }
bool OtterDevice::setIrExposure(int v) { return setProperty(PROP_IR_EXPOSURE, std::min(14000, std::max(100, v))); }
int OtterDevice::irExposure() { int32_t v[5]; return getProperty(PROP_IR_EXPOSURE, v) ? v[0] : -1; }

bool OtterDevice::setupStream(Stream& s, unsigned pixelFormat, std::string& error) {
    v4l2_format f{}; f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    f.fmt.pix.width = 1280; f.fmt.pix.height = 800; f.fmt.pix.pixelformat = pixelFormat;
    if (ioctl(s.fd, VIDIOC_S_FMT, &f) < 0 || f.fmt.pix.width != 1280 || f.fmt.pix.height != 800 || f.fmt.pix.pixelformat != pixelFormat) { error = tr("Videoformat abgelehnt"); return false; }
    v4l2_streamparm parm{}; parm.type = f.type; parm.parm.capture.timeperframe = {1, 30};
    ioctl(s.fd, VIDIOC_S_PARM, &parm);
    v4l2_requestbuffers r{}; r.count = 6; r.type = f.type; r.memory = V4L2_MEMORY_MMAP;
    if (ioctl(s.fd, VIDIOC_REQBUFS, &r) < 0 || r.count < 2) { error = tr("Puffer nicht verfügbar (Kamera von anderem Programm belegt?)"); return false; }
    s.buffers.clear();
    for (unsigned i = 0; i < r.count; i++) {
        v4l2_buffer b{}; b.type = f.type; b.memory = V4L2_MEMORY_MMAP; b.index = i;
        if (ioctl(s.fd, VIDIOC_QUERYBUF, &b) < 0) { error = tr("Puffer-Abfrage fehlgeschlagen"); return false; }
        void* p = mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, s.fd, b.m.offset);
        if (p == MAP_FAILED) { error = tr("Puffer-Mapping fehlgeschlagen"); return false; }
        s.buffers.push_back({(uint8_t*)p, b.length});
        if (ioctl(s.fd, VIDIOC_QBUF, &b) < 0) { error = tr("Puffer-Einreihung fehlgeschlagen"); return false; }
    }
    int t = f.type;
    if (ioctl(s.fd, VIDIOC_STREAMON, &t) < 0) { error = std::string(tr("Stream-Start fehlgeschlagen: ")) + strerror(errno); return false; }
    s.pending = -1;
    return true;
}
void OtterDevice::teardownStream(Stream& s) {
    if (s.fd < 0) return;
    int t = V4L2_BUF_TYPE_VIDEO_CAPTURE; ioctl(s.fd, VIDIOC_STREAMOFF, &t);
    for (auto& b : s.buffers) munmap(b.first, b.second);
    s.buffers.clear();
    v4l2_requestbuffers r{}; r.count = 0; r.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; r.memory = V4L2_MEMORY_MMAP;
    ioctl(s.fd, VIDIOC_REQBUFS, &r);
    s.pending = -1;
}
bool OtterDevice::startStreams(std::string& error) {
    if (streaming_) return true;
    // Find out first whether the cameras are free: another program's mode must stay untouched.
    for (int fd : {fdDepth_, fdIr_}) {
        v4l2_format f{}; f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        f.fmt.pix.width = 1280; f.fmt.pix.height = 800; f.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
        if (ioctl(fd, VIDIOC_S_FMT, &f) < 0) {
            error = errno == EBUSY ? tr("Kamera ist belegt. Läuft VoxelForge oder ein anderes Kameraprogramm bereits?") : tr("Videoformat abgelehnt");
            return false;
        }
    }
    if (!ensureBinocularMode(error) || !readCalibration(error)) return false;
    // Claim the cameras first. If another program uses them, its settings stay untouched.
    depthStream_.fd = fdDepth_; irStream_.fd = fdIr_;
    if (!setupStream(depthStream_, V4L2_PIX_FMT_GREY, error) || !setupStream(irStream_, V4L2_PIX_FMT_GREY, error)) {
        teardownStream(depthStream_); teardownStream(irStream_);
        if (errno == EBUSY) error = tr("Kamera ist belegt. Läuft VoxelForge oder ein anderes Kameraprogramm bereits?");
        return false;
    }
    if (!setProperty(PROP_IR_SOURCE, 0)) { teardownStream(depthStream_); teardownStream(irStream_); error = tr("IR-Quelle nicht setzbar"); return false; }
    setLaser(true);
    colorRunning_ = false; latestColor_.clear();
    if (colorEnabled_ && fdColor_ >= 0) {
        colorStream_.fd = fdColor_; std::string colorError;
        colorRunning_ = setupStream(colorStream_, V4L2_PIX_FMT_MJPEG, colorError);   // scanning works without color
        if (!colorRunning_) teardownStream(colorStream_);
        else {   // the camera starts with a fixed exposure that is far too dark indoors
            v4l2_control c{}; c.id = V4L2_CID_EXPOSURE_AUTO; c.value = V4L2_EXPOSURE_AUTO; ioctl(fdColor_, VIDIOC_S_CTRL, &c);
            c.id = V4L2_CID_AUTO_WHITE_BALANCE; c.value = 1; ioctl(fdColor_, VIDIOC_S_CTRL, &c);
        }
    }
    streaming_ = true;
    return true;
}
void OtterDevice::stopStreams() {
    if (!streaming_) return;
    teardownStream(depthStream_); teardownStream(irStream_);
    if (colorRunning_) teardownStream(colorStream_);
    colorRunning_ = false;
    streaming_ = false;
}
bool OtterDevice::nextPair(StereoFrame& out, int timeoutMs) {
    if (!streaming_) return false;
    Stream* st[2] = {&depthStream_, &irStream_};
    auto requeue = [](Stream& s) {
        if (s.pending < 0) return;
        v4l2_buffer b{}; b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP; b.index = s.pending;
        ioctl(s.fd, VIDIOC_QBUF, &b); s.pending = -1;
    };
    pollfd p[3] = {{fdDepth_, POLLIN, 0}, {fdIr_, POLLIN, 0}, {colorRunning_ ? fdColor_ : -1, POLLIN, 0}};
    for (;;) {
        int rc = poll(p, 3, timeoutMs);
        if (rc == 0) return false;
        if (rc < 0) { if (errno == EINTR) continue; return false; }
        if (p[2].revents & POLLIN) {   // keep the newest color image; it is not paired, its time is reported
            v4l2_buffer b{}; b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP;
            if (ioctl(fdColor_, VIDIOC_DQBUF, &b) == 0) {
                if (!(b.flags & V4L2_BUF_FLAG_ERROR) && b.bytesused > 0) {
                    const uint8_t* d = colorStream_.buffers[b.index].first;
                    latestColor_.assign(d, d + b.bytesused);
                    latestColorTs_ = b.timestamp.tv_sec + b.timestamp.tv_usec*1e-6;
                }
                ioctl(fdColor_, VIDIOC_QBUF, &b);
            }
        }
        for (int i = 0; i < 2; i++) {
            if (!(p[i].revents & POLLIN)) continue;
            v4l2_buffer b{}; b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP;
            if (ioctl(st[i]->fd, VIDIOC_DQBUF, &b) < 0) continue;
            if (b.flags & V4L2_BUF_FLAG_ERROR) { ioctl(st[i]->fd, VIDIOC_QBUF, &b); continue; }
            requeue(*st[i]);
            st[i]->pending = b.index;
            st[i]->pendingTs = b.timestamp.tv_sec + b.timestamp.tv_usec * 1e-6;
        }
        if (depthStream_.pending < 0 || irStream_.pending < 0) continue;
        double dt = depthStream_.pendingTs - irStream_.pendingTs;
        if (dt > 0.005) { requeue(irStream_); continue; }
        if (dt < -0.005) { requeue(depthStream_); continue; }
        out.width = 1280; out.height = 800;
        out.other.assign(depthStream_.buffers[depthStream_.pending].first, depthStream_.buffers[depthStream_.pending].first + 1280 * 800);
        out.ref.assign(irStream_.buffers[irStream_.pending].first, irStream_.buffers[irStream_.pending].first + 1280 * 800);
        out.timestamp = irStream_.pendingTs;
        out.index = frameIndex_++;
        out.colorJpeg = latestColor_; out.colorTimestamp = latestColorTs_;
        requeue(depthStream_); requeue(irStream_);
        return true;
    }
}

int adjustExposure(const std::vector<uint8_t>& ir, int current) {
    int hist[256] = {0}; for (size_t i = 0; i < ir.size(); i += 7) hist[ir[i]]++;
    int total = (int)(ir.size()/7 + 1), acc = 0, p99 = 255;
    for (int v = 255; v >= 0; v--) { acc += hist[v]; if (acc > total/100) { p99 = v; break; } }
    int next = current;
    if (p99 >= 250) next = (int)(current/1.2f);
    else if (p99 < 170) next = (int)(current*1.15f)+1;
    return std::clamp(next, 200, 12000);
}
