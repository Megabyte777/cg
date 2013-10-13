#pragma once

#include <algorithm>
#include <vector>
#include <cg/primitives/contour.h>
#include <cg/operations/orientation.h>
#include <boost/utility.hpp>

namespace cg
{
    template <typename Scalar>
    contour_2t<Scalar> minkowski_convex_sum(contour_2t<Scalar> const& c1, contour_2t<Scalar> const& c2, point_2t<Scalar> origin = point_2t<Scalar>(0, 0))
    {
        auto i = c1.circulator(std::min_element(c1.begin(), c1.end()));
        auto j = c2.circulator(std::min_element(c2.begin(), c2.end()));
        int n = c1.size() + c2.size();
        contour_2t<Scalar> result = contour_2t<Scalar>(std::vector<point_2t<Scalar> >());
        for (int k = 0; k < n; ++k)
        {
            vector_2t<Scalar> delta = *i - origin;
            point_2t<Scalar> p = *j + delta;
            result.add_point(p);
            if (orientation(*i, *boost::next(i), *j, *boost::next(j)) == CG_LEFT)
                ++i;
            else
                ++j;
        }
        return result;
    }
}
