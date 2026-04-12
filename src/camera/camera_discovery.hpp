#pragma once

#include "camera_info.hpp"
#include <vector>

// Runs `libcamera-hello --list-cameras` via popen() and parses the output.
// Returns one CameraInfo per discovered camera, each with its sensor modes.
// Returns an empty vector if no cameras found or if libcamera is unavailable.
std::vector<CameraInfo> discover_cameras();
