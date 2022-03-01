#include <algorithm>
#include "utils.h"

double normalize_angle(double angle) {
    while (angle <= -(M_PI)) {
        angle += 2 * M_PI;
    }
    while (angle > (M_PI)) {
        angle -= 2 * M_PI;
    }
    return angle;
}

size_t argmax(const std::vector<double> &v) {
    return std::distance(v.begin(), max_element(v.begin(), v.end()));
}
