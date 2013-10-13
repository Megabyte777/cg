#pragma once

#include <cg/operations/has_intersection/segment_segment.h>
#include <cg/common/structures/graph.h>
#include <boost/utility.hpp>
#include <boost/concept_check.hpp>
#include <utility>

namespace cg
{
    template <typename Scalar>
    bool points_are_visible(point_2t<Scalar> const& a, point_2t<Scalar> const& b, segment_2t<Scalar> const& s)
    {
        return a == s[0] || a == s[1] || b == s[0] || b == s[1] ||
               !has_intersection(segment_2t<Scalar>(a, b), s);
    }

    template <typename Scalar, typename BidIter>
    bool points_are_visible(point_2t<Scalar> const& a, point_2t<Scalar> const& b, BidIter begin, BidIter end)
    {
        for (BidIter i = begin; i != end; ++i)
        {
            contour_2t<Scalar> c = *i;
            point_2t<Scalar> prev = c[0];
            for (auto j = boost::next(c.begin()); j != c.end(); ++j)
            {
                segment_2t<Scalar> s(prev, *j);
                if (!points_are_visible(a, b, s))
                    return false;
                prev = *j;
            }
            segment_2t<Scalar> s(prev, c[0]);
            if (!points_are_visible(a, b, s))
                return false;
        }
        return true;
    }

    template <typename Point, typename Circulator>
    bool edge_is_necessary(Point const& p, Circulator i)
    {
        --i;
        auto a = *i;
        ++i;
        auto b = *i;
        ++i;
        auto c = *i;
        return orientation(a, b, p) != CG_RIGHT || orientation(b, c, p) != CG_RIGHT;
    }

    template <typename Point, typename Circulator>
    void add_edge_if_necessary(graph &g, int first_node, int second_node, Point const& p1, Point const& p2, Circulator const& c1, Circulator const& c2, bool sparse)
    {
        if (sparse)
        {
            if (edge_is_necessary(p1, c2))
                g.add_edge(first_node, second_node);
            if (edge_is_necessary(p2, c1))
                g.add_edge(second_node, first_node);
        }
        else
            g.add_bidirected_edge(first_node, second_node);
    }

    template <typename BidIter>
    graph visibility_graph(BidIter begin, BidIter end, bool sparse = false)
    {
        typedef decltype((*begin)[0].x) Scalar;
        int graph_size = 0;
        for (auto i = begin; i != end; ++i)
            graph_size += i->size();
        graph result(graph_size);
        int first_node = 0;
        int processed_points = 0;
        for (BidIter i = begin; i != end; ++i)
        {
            contour_2t<Scalar> c1 = *i;
            for (auto j = c1.begin(); j != c1.end(); ++j)
            {
                point_2t<Scalar> p1 = *j;
                // checking visibility within one polygon
                int second_node = first_node + 1;
                for (auto k = boost::next(j); k != c1.end(); ++k)
                {
                    point_2t<Scalar> p2 = *k;
                    if (c1.circulator(k) == ++c1.circulator(j) ||
                        c1.circulator(k) == --c1.circulator(j))
                        add_edge_if_necessary(result, first_node, second_node, p1, p2, c1.circulator(j), c1.circulator(k), sparse);
                    else
                    {
                        if (points_are_visible(p1, p2, begin, end))
                        {
                            point_2t<Scalar> x = *(--c1.circulator(k));
                            point_2t<Scalar> y = p2;
                            point_2t<Scalar> z = *(++c1.circulator(k));
                            if ((orientation(x, y, z) == CG_LEFT && (orientation(x, y, p1) == CG_RIGHT || orientation(y, z, p1) == CG_RIGHT)) ||
                                (orientation(x, y, z) == CG_RIGHT && orientation(x, y, p1) == CG_RIGHT && orientation(y, z, p1) == CG_RIGHT))
                                add_edge_if_necessary(result, first_node, second_node, p1, p2, c1.circulator(j), c1.circulator(k), sparse);
                        }
                    }
                    ++second_node;
                }
                // checking other polygons
                second_node = processed_points + c1.size();
                for (auto k = boost::next(i); k != end; ++k)
                {
                    contour_2t<Scalar> c2 = *k;
                    for (auto l = c2.begin(); l != c2.end(); ++l)
                    {
                        point_2t<Scalar> p2 = *l;
                        if (points_are_visible(p1, p2, begin, end))
                            add_edge_if_necessary(result, first_node, second_node, p1, p2, c1.circulator(j), c2.circulator(l), sparse);
                        ++second_node;
                    }
                }
                ++first_node;
            }
            processed_points += c1.size();
        }
        return result;
    }
}
