#include <gtest/gtest.h>

#include "random_utils.h"

#include <cg/triangulation/delaunay.h>

using cg::point_2;
using cg::triangulation;
using cg::triangle_2;

bool check_triangulation(triangulation<double> const &tr)
{
    std::vector<triangle_2> triangles = tr.get_triangles();
    for (auto t1 : triangles)
        for (auto t2 : triangles)
            for (int i = 0; i < 3; ++i)
                if (!delaunay_criterion(t1[0], t1[1], t1[2], t2[i]))
                    return false;
    return true;
}

TEST(delaunay_triangulation, uniform0)
{

    for (size_t cnt_points = 3; cnt_points < 50; cnt_points++)
    {
        std::vector<point_2> pts = uniform_points(cnt_points);
        triangulation<double> tr;
        for (size_t i = 0; i < cnt_points; ++i)
            tr.add_point(pts[i]);
        EXPECT_TRUE(check_triangulation(tr));
    }
}

TEST(delaunay_triangulation, uniform1)
{
    const int cnt_points = 500;
    for (size_t cnt_tests = 0; cnt_tests < 10; cnt_tests++)
    {
        std::vector<point_2> pts = uniform_points(cnt_points);
        triangulation<double> tr;
        for (size_t i = 0; i < cnt_points; ++i)
            tr.add_point(pts[i]);
        EXPECT_TRUE(check_triangulation(tr));
    }
}
