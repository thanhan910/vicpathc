#include <iostream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/formulas/spherical.hpp>

namespace bg = boost::geometry;

int main() {
    // Define the geographic point type
    using BPoint = bg::model::point<double, 2, bg::cs::geographic<bg::degree>>;

    // Define the input point and segment endpoints
    BPoint point(144.9631, -37.8136);  // The point in Melbourne, Australia
    BPoint seg_start(144.9000, -37.8000);  // Start of the segment
    BPoint seg_end(145.0000, -37.8500);    // End of the segment

    // Calculate the perpendicular projection of the point onto the great circle
    BPoint projected_point;
    bg::strategy::project::geographic<> strategy;  // Projection strategy for geographic coordinates
    bg::formula::project_point_on_segment<BPoint>(seg_start, seg_end, point, strategy, projected_point);

    // Compute the geodesic distance to the projected point
    double perpendicular_distance = bg::distance(point, projected_point);

    // Output the result
    std::cout << "Perpendicular distance (in meters): "
              << perpendicular_distance << " m" << std::endl;

    return 0;
}
