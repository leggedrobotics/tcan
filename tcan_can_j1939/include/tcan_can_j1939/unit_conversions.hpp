#pragma once

namespace tcan_can_j1939 {

inline double radiansFromDegrees(double degrees) {
    return degrees / 180. * M_PI;
}

}  // namespace tcan_can_j1939