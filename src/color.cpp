#include "color.hpp"
#include <turbojpeg.h>
#include <cstdio>
#include <cstdlib>
#include <filesystem>

bool decodeJpeg(const std::vector<uint8_t>& jpeg, ColorImage& out) {
    tjhandle h = tjInitDecompress();
    if (!h) return false;
    int w = 0, hgt = 0, sub = 0, cs = 0;
    bool ok = tjDecompressHeader3(h, jpeg.data(), (unsigned long)jpeg.size(), &w, &hgt, &sub, &cs) == 0;
    if (ok) {
        out.width = w; out.height = hgt; out.rgb.resize((size_t)w*hgt*3);
        ok = tjDecompress2(h, jpeg.data(), (unsigned long)jpeg.size(), out.rgb.data(), w, 0, hgt, TJPF_RGB, TJFLAG_FASTDCT) == 0;
    }
    tjDestroy(h);
    return ok;
}
bool loadColorCalib(const std::string& path, ColorCalib& c) {
    FILE* f = fopen(path.c_str(), "r");
    if (!f) return false;
    float v[15]; int n = 0;
    while (n < 15 && fscanf(f, "%f", &v[n]) == 1) n++;
    fclose(f);
    if (n != 15 || v[0] < 100) return false;
    c.focalPx = v[0]; c.cx = v[1]; c.cy = v[2];
    for (int i = 0; i < 9; i++) c.refToColor.R[i] = v[3+i];
    c.refToColor.t = V3{v[12], v[13], v[14]};
    c.valid = true;
    return true;
}
bool saveColorCalib(const std::string& path, const ColorCalib& c) {
    std::error_code ec; std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
    FILE* f = fopen(path.c_str(), "w");
    if (!f) return false;
    fprintf(f, "%.4f %.4f %.4f\n", c.focalPx, c.cx, c.cy);
    for (int i = 0; i < 9; i++) fprintf(f, "%.7f%c", c.refToColor.R[i], i%3 == 2 ? '\n' : ' ');
    fprintf(f, "%.4f %.4f %.4f\n", c.refToColor.t.x, c.refToColor.t.y, c.refToColor.t.z);
    return fclose(f) == 0;
}
std::string colorCalibPath(const std::string& serial) {
    const char* xdg = getenv("XDG_CONFIG_HOME"); const char* home = getenv("HOME");
    std::string base = xdg && *xdg ? xdg : std::string(home ? home : ".") + "/.config";
    return base + "/voxelforge/rgb_" + serial + ".txt";
}
