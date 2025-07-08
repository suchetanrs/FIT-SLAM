#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <iostream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> DT;
typedef CGAL::Voronoi_diagram_2<DT> VD;
typedef K::Point_2 Point;

int main() {
    std::vector<Point> points = { Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1) };
    DT dt;
    dt.insert(points.begin(), points.end());

    VD vd(dt);

    for (auto face = vd.faces_begin(); face != vd.faces_end(); ++face) {
        if (face->is_unbounded()) continue;
        std::cout << "Voronoi face: ";
        auto v = face->dual()->vertex();
        std::cout << "(" << v->point() << ")";
        std::cout << std::endl;
    }

    return 0;
}