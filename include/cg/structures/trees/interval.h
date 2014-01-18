#pragma once

#include <cg/primitives/range.h>
#include <vector>
#include <algorithm>

namespace cg
{
    template <typename Scalar>
    struct interval_tree
    {
        ~interval_tree()
        {
            delete left_child;
            delete right_child;
        }

        interval_tree(std::vector<range_t<Scalar> > const &segments)
            : left_child(0)
            , right_child(0)
        {
            if (segments.size() == 0)
                return;
            std::vector<Scalar> coordinates;
            for (range_t<Scalar> const &r : segments)
            {
                coordinates.push_back(r.inf);
                coordinates.push_back(r.sup);
            }
            std::nth_element(coordinates.begin(), coordinates.begin() + coordinates.size() / 2, coordinates.end());
            mid = coordinates[coordinates.size() / 2];
            std::vector<range_t<Scalar> > left, right;
            for (range_t<Scalar> const &r : segments)
            {
                if (r.sup < mid)
                    left.push_back(r);
                if (r.inf > mid)
                    right.push_back(r);
                if (r.inf <= mid && r.sup >= mid)
                {
                    left_segments.push_back(r);
                    right_segments.push_back(r);
                }
            }
            std::sort(left_segments.begin(), left_segments.end(),
                    [] (range_t<Scalar> const &a, range_t<Scalar> const &b)
                    {
                        return a.inf < b.inf;
                    }
            );
            std::sort(right_segments.begin(), right_segments.end(),
                    [] (range_t<Scalar> const &a, range_t<Scalar> const &b)
                    {
                        return a.sup > b.sup;
                    }
            );
            left_child = new interval_tree(left);
            right_child = new interval_tree(right);
        }

        std::vector<range_t<Scalar> > get(Scalar q)
        {
            if (left_segments.empty())
                return std::vector<range_t<Scalar> >();
            std::vector<range_t<Scalar> > result;
            if (q < mid)
                result = left_child->get(q);
            if (q > mid)
                result = right_child->get(q);
            if (q < mid)
            {
                for (int i = 0; i < left_segments.size() && left_segments[i].inf <= q; ++i)
                    result.push_back(left_segments[i]);
            }
            else
            {
                for (int i = 0; i < right_segments.size() && right_segments[i].sup >= q; ++i)
                    result.push_back(right_segments[i]);
            }
            return result;
        }

    private:
        interval_tree *left_child, *right_child;
        Scalar mid;
        std::vector<range_t<Scalar> > left_segments, right_segments;

        interval_tree()
            : left_child(0)
            , right_child(0)
        {}
    };
}
