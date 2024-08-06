#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

namespace utils {
    namespace calcs {
        float map_float(float x, float in_min, float in_max, float out_min, float out_max);
        float milli_to_single(float x);
    }  // namespace calc
}
#endif  // UTILS_H