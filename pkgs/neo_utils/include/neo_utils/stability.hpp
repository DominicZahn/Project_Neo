#pragma once

#include <boost/geometry.hpp>
#include <boost/lockfree/spsc_queue.hpp>
namespace bg = boost::geometry;
typedef bg::model::point<double, 2, bg::cs::cartesian> Point;
typedef bg::model::polygon<Point> Polygon;
typedef bg::model::segment<Point> Segment;

namespace neo_utils {
// ----------------------- stability criterias -----------------------
// helper functions

float minPtToEdgeDist(const Point &p, const Polygon &poly);
static bool CoM_in_PoS(const Point &p, const Polygon &poly);
/**
 * This namespace encapsulates everything about the used stability criterias.
 */
namespace Stability {
typedef std::function<const float(const Point &, const Polygon &)> CriteriaFunc;

float minEdgeDist(const Point &CoM, const Polygon &PoS);
float sumPtDist(const Point &CoM, const Polygon &PoS);
}  // namespace Stability
// -------------------------------------------------------------------
}  // namespace neo_utils