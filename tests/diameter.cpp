// by Pendryak Arseniy

#include <gtest/gtest.h>

#include <boost/assign/list_of.hpp>

#include <vector>
#include <algorithm>
#include <utility>

#include <cg/primitives/point.h>
#include <cg/operations/diameter.h>

#include "random_utils.h"

using cg::point_2;

bool is_diameter(const std::vector<point_2> &pts, std::pair<point_2, point_2> result)
{
   auto res = std::make_pair(pts[0], pts[0]);
   if (std::find(pts.begin(), pts.end(), result.first) == pts.end() ||
       std::find(pts.begin(), pts.end(), result.second) == pts.end())
   {
      return false;
   }

   for (size_t i = 0; i < pts.size(); ++i) {
      for (size_t j = 0; j < pts.size(); ++j) {
         if (cg::cmp_dist(res.first, res.second, pts[i], pts[j]))
         {
            res = std::make_pair(pts[i], pts[j]);
         }
      }
   }
   return !cg::cmp_dist(res.first, res.second, result.first, result.second) && !cg::cmp_dist(result.first, result.second, res.first, res.second);
}

TEST(diameter, uniform)
{
   std::vector<point_2> pts = uniform_points(10000);
   auto res = cg::diameter(pts.begin(), pts.end());
   EXPECT_TRUE(is_diameter(pts, std::make_pair(*res.first, *res.second)));
}

TEST(diameter, one_point)
{
    std::vector<point_2> pts = boost::assign::list_of(point_2(1, 1));
    auto res = cg::diameter(pts.begin(), pts.end());
    EXPECT_TRUE(is_diameter(pts, std::make_pair(*res.first, *res.second)));
}

TEST(diameter, same_point)
{
    std::vector<point_2> pts = boost::assign::list_of(point_2(1, 1))
                                                     (point_2(1, 1))
                                                     (point_2(1, 1))
                                                     (point_2(1, 1))
                                                     (point_2(1, 1))
                                                     (point_2(1, 1))
                                                     (point_2(1, 1))
                                                     (point_2(1, 1));
    auto res = cg::diameter(pts.begin(), pts.end());
    EXPECT_TRUE(is_diameter(pts, std::make_pair(*res.first, *res.second)));
}

TEST(diameter, one_line)
{
    std::vector<point_2> pts = boost::assign::list_of(point_2(1, 1))
                                                     (point_2(2, 2))
                                                     (point_2(3, 3))
                                                     (point_2(4, 4))
                                                     (point_2(5, 5));
    auto res = cg::diameter(pts.begin(), pts.end());
    EXPECT_TRUE(is_diameter(pts, std::make_pair(*res.first, *res.second)));
}
