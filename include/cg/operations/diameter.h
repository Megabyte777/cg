#pragma once

#include <cg/convex_hull/quick_hull.h>
#include <cg/operations/distance.h>
#include <algorithm>

namespace cg
{
    template <typename ForwardIter>
    ForwardIter next_iter(ForwardIter i, ForwardIter begin, ForwardIter end)
    {
        i++;
        if (i == end)
            return begin;
        return i;
    }

    template <typename ForwardIter>
    std::pair<ForwardIter, ForwardIter> diameter(ForwardIter begin, ForwardIter end)
    {
        typedef typename std::iterator_traits<ForwardIter>::value_type point;
        std::vector<point> points(begin, end);
        auto convex_begin = points.begin();
        auto convex_end = quick_hull(points.begin(), points.end());
        auto minmax = std::minmax_element(convex_begin, convex_end, [] (point const &a, point const &b)
                                                    {
                                                        return a.x < b.x;
                                                    }
        );
        auto p = minmax.first;
        auto p_n = next_iter(p, convex_begin, convex_end);
        auto q = minmax.second;
        auto q_n = next_iter(q, convex_begin, convex_end);
        auto ans = std::make_pair(*p, *q);
        auto less = [] (point const &a, point const &b,
                              point const &c, point const &d)
        {
            return pred(a, b, c, d) == CG_RIGHT;
        };
        do
        {
            if (cmp_dist(ans.first, ans.second, *p, *q))
                ans = std::make_pair(*p, *q);
            if (less(*p, *p_n, *q_n, *q))
            {
                p = p_n;
                p_n = next_iter(p, convex_begin, convex_end);
            }
            else
            {
                q = q_n;
                q_n = next_iter(q, convex_begin, convex_end);
            }
        } while (p != minmax.first || q != minmax.second);
        return std::make_pair(std::find(begin, end, ans.first), std::find(begin, end, ans.second));
    }
}
