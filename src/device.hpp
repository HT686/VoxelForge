#pragma once
// CR-Scan Otter access through the regular kernel driver (V4L2 + UVC extension unit).
// Only commands verified on real hardware are exposed. There are deliberately no
// firmware, flash or calibration write commands in this class.
#include <cstdint>
#include <string>
#include <vector>

struct StereoCalib {      // from device raw data 4026, valid for 1280x800 rectified IR
    float focalPx = 0;    // focal length in pixels
    float baselineMm = 0;
    float dispOffset = 0; // true disparity = dispOffset - (x_ref - x_other)
    // The Otter delivers its IR images mirrored left to right (property IR_MIRROR is 1), so the
    // focal length used for x is negative in DepthMap. Otherwise every model would be a mirror image.
    int rowOffset = 0;    // a point in reference row y appears in row y + rowOffset of the other image
    float cxRef() const { return 640.f + dispOffset / 2.f; }
    float cy() const { return 400.f; }
    float depthFromShift(float s) const { float d = dispOffset - s; return d > 1.f ? focalPx * baselineMm / d : 0.f; }
};

struct StereoFrame {
    int width = 0, height = 0;
    std::vector<uint8_t> ref, other;   // ref = "IR" interface, other = "Depth" interface
    double timestamp = 0;
    uint64_t index = 0;
    std::vector<uint8_t> colorJpeg;    // newest color image, empty if color is off or none arrived yet
    double colorTimestamp = 0;
};

// The Otter has two stereo pairs: an inner long-focus pair for detail at close range and an outer
// short-focus pair with a wider view for large objects. Each has its own factory calibration.
enum class CameraPair { Close, Far };

class OtterDevice {
public:
    ~OtterDevice();
    bool open(std::string& error);
    void close();
    bool isOpen() const { return fdDepth_ >= 0; }
    const StereoCalib& calib() const { return calib_; }
    const std::string& description() const { return description_; }
    const std::string& serial() const { return serial_; }
    bool colorRunning() const { return colorRunning_; }

    bool setLaser(bool on);
    bool setIrExposure(int value);   // device range 100..14000
    int irExposure();

    void setCameraPair(CameraPair p) { pair_ = p; }   // takes effect at the next startStreams
    void setColorEnabled(bool on) { colorEnabled_ = on; }   // takes effect at the next startStreams
    bool startStreams(std::string& error);
    void stopStreams();
    // Blocks up to timeoutMs for the next synchronized pair. False on timeout.
    bool nextPair(StereoFrame& out, int timeoutMs);

private:
    struct Stream { int fd = -1; std::vector<std::pair<uint8_t*, size_t>> buffers; int pending = -1; double pendingTs = 0; };
    bool xferXu(uint16_t opcode, const std::vector<uint8_t>& payload, std::vector<uint8_t>& resp, int respSelector);
    bool getProperty(uint32_t id, int32_t values[5]);
    bool setProperty(uint32_t id, int32_t value);
    bool readRaw(uint32_t id, std::vector<uint8_t>& out);
    bool readStruct(uint32_t id, std::vector<uint8_t>& out);
    bool ensureBinocularMode(std::string& error);
    bool readCalibration(std::string& error);
    bool setupStream(Stream& s, unsigned pixelFormat, std::string& error);
    void teardownStream(Stream& s);

    int fdDepth_ = -1, fdIr_ = -1, fdColor_ = -1;
    bool colorEnabled_ = false, colorRunning_ = false;
    Stream colorStream_;
    std::vector<uint8_t> latestColor_; double latestColorTs_ = 0;
    uint16_t xuLen_[4] = {0, 0, 0, 0};
    uint16_t requestId_ = 1;
    StereoCalib calib_;
    std::string description_, serial_;
    Stream depthStream_, irStream_;
    CameraPair pair_ = CameraPair::Close;
    bool streaming_ = false;
    uint64_t frameIndex_ = 0;
};

// Next exposure, from how bright the brightest share of the image is.
int adjustExposure(const std::vector<uint8_t>& ir, int current);
