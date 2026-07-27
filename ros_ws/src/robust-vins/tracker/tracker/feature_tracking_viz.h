#pragma once

#include <common/types.h>

namespace FAR {

// Forward declarations
class FeatureTracker;

void visualizeTracks(
    const FeatureTracker& tracker, size_t frame_index, int sleep);

} // namespace FAR
