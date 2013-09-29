#pragma once

#include <cg/primitives/segment.h>
#include <algorithm>
#include <boost/next_prior.hpp>

namespace cg
{
    template <typename Scalar>
    double distance(segment_2t<Scalar> s, point_2t<Scalar> p)
    {
        double projection =   ((s[1].x - s[0].x) * (p.x - s[0].x)
                            +  (s[1].y - s[0].y) * (p.y - s[0].y))
                            / s.length();
        double distance;
        if (projection >= 0 && projection <= s.length())
            distance = fabs((s[1].x - s[0].x) * (p.y - s[0].y)
                            - (p.x - s[0].x) * (s[1].y - s[0].y))
                        / s.length() / 2;
        else if (projection > 0 )
            distance = segment_2t<Scalar>(s[1], p).length();
        else
            distance = segment_2t<Scalar>(s[0], p).length();
        return distance;
    }

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
                                            return distance(segment_2t<double>(a, b), x) <
                                                   distance(segment_2t<double>(a, b), y);
                                        }
                                      );
        point c = *max;
        double dist = distance(segment_2t<double>(a, b), c);
        if (dist < eps)
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
