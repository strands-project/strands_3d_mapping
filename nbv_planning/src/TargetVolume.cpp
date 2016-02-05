//
// Created by chris on 24/11/15.
//

#include "nbv_planning/TargetVolume.h"

namespace nbv_planning {
    std::ostream& operator<<(std::ostream &os, const TargetVolume &volume) {
        os << "Origin " << volume.m_origin << std::endl;
        os << "Extents: " << volume.m_extents << std::endl;
        os << "Scale: " << volume.m_scale;
        return os;
    }
}