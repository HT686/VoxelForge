#pragma once
// Reads back stereo pairs written by `voxelforge-cli record` for offline runs.
#include <string>
#include "device.hpp"
bool writePair(const std::string& dir, uint64_t index, const StereoFrame& f, std::string& error);
bool readPair(const std::string& dir, uint64_t index, StereoFrame& f);
bool writeCalib(const std::string& dir, const StereoCalib& c);
bool readCalib(const std::string& dir, StereoCalib& c);
