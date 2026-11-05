#pragma once
// Color camera: decoding and its position relative to the reference IR camera.
#include <cstdint>
#include <string>
#include <vector>
#include "math.hpp"

struct ColorCalib {
    bool valid = false;
    float focalPx = 0, cx = 640, cy = 400;
    Pose refToColor;    // reference IR camera coordinates -> color camera coordinates
};
struct ColorImage { int width = 0, height = 0; std::vector<uint8_t> rgb; };

bool decodeJpeg(const std::vector<uint8_t>& jpeg, ColorImage& out);
// Text file: focal cx cy, then 9 rotation values, then 3 translation values.
bool loadColorCalib(const std::string& path, ColorCalib& out);
bool saveColorCalib(const std::string& path, const ColorCalib& c);
std::string colorCalibPath(const std::string& serial);   // per scanner, in the user's config directory
