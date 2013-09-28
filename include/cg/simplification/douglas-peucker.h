#pragma once

#include <cg/primitives/segment.h>
#include <algorithm>
#include <boost/next_prior.hpp>
#include <cg/convex_hull/quick_hull.h>

namespace cg
{
    template <typename BidIter, typename OutIter>
    OutIter douglas_peucker(BidIter begin, BidIter end, double eps, OutIter out)
    {
        if (end - begin <= 2)
            return std::copy(begin, end, out);
        typedef typename std::iterator_traits<BidIter>::value_type point;
        BidIter second = boost::next(begin);
        BidIter last = boost::prior(end);
        point a = *begin;
        point b = *last;
        BidIter max = std::max_element(second, last,
                                        [&a, &b] (point const &x, point const &y)
                                        {
                                            return pred(a, b, x, y) == CG_RIGHT;
                                        }
                                      );
        point c = *max;
        /*computation of distance*/
        double projection = ((b.x - a.x) * (c.x - a.x) + (b.y - a.y) * (c.y - a.y)) /
                    segment_2t<double>(a, b).length();
        double distance;
        if (projection >= 0 && projection <= segment_2t<double>(a, b).length())
            distance = fabs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)) /
                segment_2t<double>(a, b).length() / 2;
        else if (projection > 0 )
            distance = segment_2t<double>(b, c).length();
        else
            distance = segment_2t<double>(a, c).length();
        /*end of computation of distance*/
        if (distance < eps)
        {
            *out++ = a;
            *out++ = b;
            return out;
        }
        std::vector<point> ans;
        douglas_peucker(begin, max + 1, eps, std::back_inserter(ans));
        ans.pop_back();
        douglas_peucker(max, end, eps, std::back_inserter(ans));
        return std::copy(ans.begin(), ans.end(), out);
    }
}
