#include "source.hpp"
#include <cstdio>
#include <filesystem>
static std::string pairPath(const std::string& d, uint64_t i) { char b[32]; snprintf(b, sizeof b, "/%06llu.pair", (unsigned long long)i); return d + b; }
bool writePair(const std::string& dir, uint64_t index, const StereoFrame& f, std::string& error) {
    std::filesystem::create_directories(dir);
    FILE* o = fopen(pairPath(dir, index).c_str(), "wb");
    if (!o) { error = "Aufnahmedatei nicht schreibbar"; return false; }
    bool ok = fwrite(f.ref.data(), 1, f.ref.size(), o) == f.ref.size() && fwrite(f.other.data(), 1, f.other.size(), o) == f.other.size();
    ok = fclose(o) == 0 && ok;
    if (ok && !f.colorJpeg.empty()) {
        std::string jp = pairPath(dir, index); jp.replace(jp.size()-5, 5, ".jpg");
        FILE* c = fopen(jp.c_str(), "wb");
        ok = c && fwrite(f.colorJpeg.data(), 1, f.colorJpeg.size(), c) == f.colorJpeg.size();
        if (c) ok = fclose(c) == 0 && ok;
        FILE* t = fopen((dir + "/times.txt").c_str(), "a");
        if (t) { fprintf(t, "%llu %.6f %.6f\n", (unsigned long long)index, f.timestamp, f.colorTimestamp); fclose(t); }
    }
    if (!ok) error = "Schreibfehler";
    return ok;
}
bool readPair(const std::string& dir, uint64_t index, StereoFrame& f) {
    FILE* in = fopen(pairPath(dir, index).c_str(), "rb");
    if (!in) return false;
    f.width = 1280; f.height = 800; f.ref.resize(1280*800); f.other.resize(1280*800); f.index = index;
    bool ok = fread(f.ref.data(), 1, f.ref.size(), in) == f.ref.size() && fread(f.other.data(), 1, f.other.size(), in) == f.other.size();
    fclose(in);
    f.colorJpeg.clear();
    std::string jp = pairPath(dir, index); jp.replace(jp.size()-5, 5, ".jpg");
    if (FILE* c = fopen(jp.c_str(), "rb")) {
        fseek(c, 0, SEEK_END); long n = ftell(c); fseek(c, 0, SEEK_SET);
        if (n > 0) { f.colorJpeg.resize(n); if (fread(f.colorJpeg.data(), 1, n, c) != (size_t)n) f.colorJpeg.clear(); }
        fclose(c);
    }
    return ok;
}
bool writeCalib(const std::string& dir, const StereoCalib& c) {
    std::filesystem::create_directories(dir);
    FILE* o = fopen((dir + "/calib.txt").c_str(), "w"); if (!o) return false;
    fprintf(o, "%.6f %.6f %.6f %d\n", c.focalPx, c.baselineMm, c.dispOffset, c.rowOffset);
    return fclose(o) == 0;
}
bool readCalib(const std::string& dir, StereoCalib& c) {
    FILE* in = fopen((dir + "/calib.txt").c_str(), "r"); if (!in) return false;
    int n = fscanf(in, "%f %f %f %d", &c.focalPx, &c.baselineMm, &c.dispOffset, &c.rowOffset);
    fclose(in);
    if (n == 3) c.rowOffset = c.dispOffset < 300.f ? 1 : 0;   // older recordings: the far pair has offset 240
    return n >= 3;
}
