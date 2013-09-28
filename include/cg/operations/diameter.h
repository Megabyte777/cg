#pragma once

#include <cg/convex_hull/quick_hull.h>
#include <algorithm>

namespace cg
{

    struct cmp_dist_d
    {
        boost::optional<bool> operator() (point_2 const & a, point_2 const & b, point_2 const & c, point_2 const & d) const
        {
            double dist1 = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
            double dist2 = (d.x - c.x) * (d.x - c.x) + (d.y - c.y) * (d.y - c.y);
            double res = dist2 - dist1;
            double eps = (fabs(dist1) + fabs(dist2)) * 32 * std::numeric_limits<double>::epsilon();
            if (res > eps)
                return true;

            if (res < -eps)
                return false;

            return boost::none;
        }
    };

    struct cmp_dist_i
    {
        boost::optional<bool> operator() (point_2 const & a, point_2 const & b, point_2 const & c, point_2 const & d) const
        {
            typedef boost::numeric::interval_lib::unprotect<boost::numeric::interval<double> >::type interval;

            boost::numeric::interval<double>::traits_type::rounding _;

            interval res =   (interval(d.x) - c.x) * (interval(d.x) - c.x) // res = dist2 - dist1
                           + (interval(d.y) - c.y) * (interval(d.y) - c.y)
                           - (interval(b.x) - a.x) * (interval(b.x) - a.x)
                           - (interval(b.y) - a.y) * (interval(b.y) - a.y);

            if (res.lower() > 0)
                return true;

            if (res.upper() < 0)
                return false;

            if (res.upper() == res.lower())
                return false;

            return boost::none;
        }
    };

    struct cmp_dist_r
    {
        boost::optional<bool> operator() (point_2 const & a, point_2 const & b, point_2 const & c, point_2 const & d) const
        {
            mpq_class res =   (mpq_class(d.x) - c.x) * (mpq_class(d.x) - c.x) // res = dist2 - dist1
                            + (mpq_class(d.y) - c.y) * (mpq_class(d.y) - c.y)
                            - (mpq_class(b.x) - a.x) * (mpq_class(b.x) - a.x)
                            - (mpq_class(b.y) - a.y) * (mpq_class(b.y) - a.y);

            int cres = cmp(res, 0);

            return cres > 0;
        }
    };

    inline bool cmp_dist(point_2 const & a, point_2 const & b, point_2 const & c, point_2 const & d)
    {
        if (boost::optional<bool> v = cmp_dist_d()(a, b, c, d))
            return *v;

        if (boost::optional<bool> v = cmp_dist_i()(a, b, c, d))
            return *v;

       return *pred_r()(a, b, c, d);
    }

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
