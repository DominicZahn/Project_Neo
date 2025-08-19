#include "neo_utils/stability.hpp"

bool neo_utils::CoM_in_PoS(const Point &p, const Polygon &poly) {
    return bg::covered_by(p, poly);
}

float neo_utils::minPtToEdgeDist(const Point &p, const Polygon &poly) {
    std::vector<Point> polyPts = poly.outer();
    const size_t polyPtsCount = polyPts.size();
    float dist = MAXFLOAT;
    for (uint i = 1; i < polyPtsCount; i++) {
        const Segment seg = Segment(polyPts[i - 1], polyPts[i]);
        dist = std::min((float)bg::distance(seg, p), dist);
    }
    return dist;
}

float neo_utils::Stability::minEdgeDist(const Point &CoM, const Polygon &PoS) {
    const float d = minPtToEdgeDist(CoM, PoS);
    return CoM_in_PoS(CoM, PoS) ? d : -d;
}

float neo_utils::Stability::sumPtDist(const Point &CoM, const Polygon &PoS) {
    float sum = 0.0;
    for (const auto &c : PoS.outer()) {
        sum += bg::distance(CoM, c);        
    }
    return CoM_in_PoS(CoM, PoS) ? sum : -sum;
}