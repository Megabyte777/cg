#include <QColor>
#include <QApplication>

#include <cg/visualization/viewer_adapter.h>
#include <cg/visualization/draw_util.h>

#include <cg/navigation/material_point.h>

using cg::point_2;
using cg::point_2f;
using cg::segment_2;
using cg::segment_2f;
using cg::contour_2;
using cg::contour_2f;

struct visibility_viewer : cg::visualization::viewer_adapter
{
    visibility_viewer()
    {
        new_contour();
    }

    void new_contour()
    {
        contours.push_back(contour_2(std::vector<point_2>()));
    }

    void calc_route()
    {
        for (contour_2 &c : contours)
            if (!counterclockwise(c))
                c.reverse();
        route.clear();
        auto begin = contours.begin();
        auto end = contours.end();
        if (contours.back().size() == 0)
            end = end - 1;
        find_shortest_path(points[0], points[1], begin, end, std::back_inserter(route));
    }

    void draw(cg::visualization::drawer_type & drawer) const
    {
        drawer.set_color(Qt::red);
        for (point_2 p : points)
            drawer.draw_point(p, 5);
        drawer.set_color(Qt::white);
        for (contour_2 c : contours)
            for (auto j = c.begin(); j < c.end(); ++j)
            {
                auto p = *j;
                drawer.draw_point(p, 5);
                drawer.draw_line(p, *(++c.circulator(j)));
            }
        drawer.set_color(Qt::green);
        for (int i = 0; i < (int) route.size() - 1; i++)
            drawer.draw_line(route[i], route[i + 1]);
    }

    void print(cg::visualization::printer_type & p) const
    {
        p.corner_stream() << "press mouse rbutton to add vertex" << cg::visualization::endl
                            << "first two points - start and finish" << cg::visualization::endl
                            << "double-click to clear" << cg::visualization::endl
                            << "press space to start a new contour" << cg::visualization::endl;
    }

    bool on_double_click(const point_2f & p)
    {
        points.clear();
        contours.clear();
        new_contour();
        contour_mode = false;
        route.clear();
        return true;
    }

    bool on_press(const point_2f & p)
    {
        if (contour_mode)
            contours.back().add_point(p);
        else
        {
            points.push_back(p);
            if (points.size() >= 2)
                contour_mode = true;
        }
        if (contour_mode)
            calc_route();
        return true;
    }

    bool on_key(int key)
    {
        switch (key)
        {
        case Qt::Key_Space:
            if (contour_mode && contours.back().size() != 0)
                new_contour();
            break;
        default:
            return false;
        }
        return true;
    }

private:
    bool contour_mode;
    std::vector<contour_2> contours;
    std::vector<point_2> points;
    std::vector<point_2> route;
};

int main(int argc, char ** argv)
{
    QApplication app(argc, argv);
    visibility_viewer viewer;
    cg::visualization::run_viewer(&viewer, "Navigation viewer");
}
