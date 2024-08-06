#include "utils.h"

namespace utils {
    namespace calcs {
        float map_float(float x, float in_min, float in_max, float out_min, float out_max){
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        } 
        float milli_to_single(float x){
            return x / 1e3;
        }
    }  // namespace calc
}
