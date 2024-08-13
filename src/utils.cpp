#include "utils.h"

namespace utils {
    namespace calcs {
        float map_float(float x, float in_min, float in_max, float out_min, float out_max){
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        } 
        float calc_dead_band(float x, float max_output, float dead_band){
            if (abs(x) < dead_band) {
                return 0;
            }
            int sign = x > 0 ? 1 : -1;
            float abs_temp = utils::calcs::map_float(abs(x), dead_band, max_output, 0, max_output);
            float scaled_temp = sign * abs_temp;
            return scaled_temp;
        }
        float milli_to_single(float x){
            return x / 1e3;
        }
    }  // namespace calc
}
