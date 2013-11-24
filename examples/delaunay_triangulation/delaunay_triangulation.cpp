#include <QColor>
#include <QApplication>

#include <cg/visualization/viewer_adapter.h>
#include <cg/visualization/draw_util.h>
#include <cg/triangulation/delaunay.h>
#include <cg/operations/contains/triangle_point.h>

#include <algorithm>

using cg::point_2;
using cg::point_2f;
using cg::segment_2;
using cg::segment_2f;
using cg::triangle_2;
using cg::circle_2;
using cg::vector_2;
using cg::triangulation;

struct delaunay_triangulation_viewer : cg::visualization::viewer_adapter
{
    delaunay_triangulation_viewer()
    {}

    circle_2 get_circumcircle(triangle_2 const &t) const
    {
        if (t[0] == t[1] && t[1] == t[2])
            return circle_2(t[0], 0);
        if (orientation(t[0], t[1], t[2]) == cg::CG_COLLINEAR)
        {
            auto minmax = std::minmax({t[0], t[1], t[2]});
            point_2 p1 = minmax.first;
            point_2 p2 = minmax.second;
            if (t[0] != t[1])
                p2 = t[1];
            else
                p2 = t[2];
            point_2 center = p1 + vector_2((p2.x - p1.x) / 2, (p2.y - p1.y) / 2);
            double radius = segment_2(p1, p2).length() / 2;
            return circle_2(center, radius);
        }
        double A1 = t[1].x - t[0].x;
        double B1 = t[1].y - t[0].y;
        double C1 = (t[1].x - t[0].x) * (t[1].x + t[0].x) / 2 + (t[1].y - t[0].y) * (t[1].y + t[0].y) / 2;
        double A2 = t[2].x - t[0].x;
        double B2 = t[2].y - t[0].y;
        double C2 = (t[2].x - t[0].x) * (t[2].x + t[0].x) / 2 + (t[2].y - t[0].y) * (t[2].y + t[0].y) / 2;
        point_2 center((C1 * B2 - C2 * B1) / (A1 * B2 - A2 * B1),
                       (A1 * C2 - A2 * C1) / (A1 * B2 - A2 * B1));
        double radius = segment_2(center, t[0]).length();
        return circle_2(center, radius);
    }

    boost::optional<triangle_2> find_triangle(point_2 const &p) const
    {
        for (triangle_2 const &t : triangles)
            if (contains(t, p))
                return t;
        return boost::none;
    }

    void draw(cg::visualization::drawer_type & drawer) const
    {
        drawer.set_color(Qt::white);
        for (point_2 p : points)
            drawer.draw_point(p, 5);
        for (triangle_2 t : triangles)
        {
            drawer.set_color(Qt::green);
            for (int j = 0; j < 3; ++j)
                drawer.draw_line(t[j], t[(j + 1) % 3]);
            drawer.set_color(Qt::gray);
            if (auto st = find_triangle(cur_point))
                drawer.draw_circle(get_circumcircle(*st));
        }
    }

    void print(cg::visualization::printer_type & p) const
    {
        p.corner_stream() << "press mouse rbutton to add vertex" << cg::visualization::endl
                            << "double-click to clear" << cg::visualization::endl;
    }

    bool on_double_click(const point_2f & p)
    {
        points.clear();
        triangles.clear();
        triang.clear();
        return true;
    }

    bool on_press(const point_2f & p)
    {
        points.push_back(p);
        triang.add_point(p);
        triangles = triang.get_triangles();
        return true;
    }

    bool on_move(const point_2f & p)
    {
        cur_point = p;
        return true;
    }

private:
    std::vector<point_2> points;
    std::vector<triangle_2> triangles;
    triangulation<double> triang;
    point_2f cur_point;
};

int main(int argc, char ** argv)
{
    QApplication app(argc, argv);
    delaunay_triangulation_viewer viewer;
    cg::visualization::run_viewer(&viewer, "Delaunay triangulation viewer");
}
