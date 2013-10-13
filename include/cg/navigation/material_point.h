#pragma once

#include <cg/visibility/visibility.h>
#include <set>

namespace cg
{
    template <typename Scalar, typename BidIter, typename OutIter>
    OutIter find_shortest_path(point_2t<Scalar> start, point_2t<Scalar> finish, BidIter begin, BidIter end, OutIter out)
    {
        std::vector<contour_2t<Scalar> > contours(begin, end);
        contours.push_back(contour_2t<Scalar>(std::vector<point_2t<Scalar> >(1, start)));
        contours.push_back(contour_2t<Scalar>(std::vector<point_2t<Scalar> >(1, finish)));
        std::vector<point_2t<Scalar> > points;
        for (contour_2t<Scalar> c : contours)
            for (point_2t<Scalar> p : c)
                points.push_back(p);
        int sn = points.size() - 2;
        int fn = points.size() - 1;
        int n = points.size();
        graph vg = visibility_graph(contours.begin(), contours.end(), true);
        std::vector<double> d(n, -1);
        d[sn] = 0;
        std::vector<int> from(n, -1);
        std::set<std::pair<double, int> > nodes;
        nodes.insert(std::make_pair(d[sn], sn));
        while (!nodes.empty())
        {
            auto t = *nodes.begin();
            nodes.erase(nodes.begin());
            double w = t.first;
            int x = t.second;
            auto const &edges = vg.get_edges(x);
            for (int y : edges)
            {
                double dist = sqrt((points[y].x - points[x].x) * (points[y].x - points[x].x) +
                                      (points[y].y - points[x].y) * (points[y].y - points[x].y)) + w;
                if (d[y] == -1 || d[y] > dist)
                {
                    nodes.erase(std::make_pair(d[y], y));
                    d[y] = dist;
                    nodes.insert(std::make_pair(d[y], y));
                    from[y] = x;
                }
            }
        }
        if (d[fn] == -1)
            return out;
        std::vector<point_2t<Scalar> > ans;
        while (fn != sn)
        {
            ans.push_back(points[fn]);
            fn = from[fn];
        }
        ans.push_back(points[sn]);
        return std::copy(ans.rbegin(), ans.rend(), out);
    }
}
