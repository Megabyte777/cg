#include <QColor>
#include <QApplication>

#include <cg/visualization/viewer_adapter.h>
#include <cg/visualization/draw_util.h>

#include <cg/visibility/visibility.h>

using cg::point_2;
using cg::point_2f;
using cg::segment_2;
using cg::segment_2f;
using cg::contour_2;
using cg::contour_2f;
using cg::graph;

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

    void calc_visibility_graph()
    {
        int n = 0;
        for (contour_2 &c : contours)
        {
            if (!counterclockwise(c))
            {
                c.reverse();
                std::reverse(points.begin() + n, points.begin() + n + c.size());
            }
            n += c.size();
        }
        graph g = visibility_graph(contours.begin(), contours.end(), true);
        visibility_segments.clear();
        for (int i = 0; i < (int) points.size(); ++i)
        {
            std::vector<int> const &edges = g.get_edges(i);
            for (int j : edges)
                visibility_segments.push_back(segment_2(points[i], points[j]));
        }
    }

    void draw(cg::visualization::drawer_type & drawer) const
    {
        drawer.set_color(Qt::green);
        for (segment_2 s : visibility_segments)
            drawer.draw_line(s[0], s[1]);
        drawer.set_color(Qt::white);
        for (contour_2 c : contours)
        {
            for (auto p = c.begin(); p < c.end(); ++p)
            {
                drawer.draw_point(*p, 5);
                drawer.draw_line(*p, *(++c.circulator(p)));
            }
        }
    }

    void print(cg::visualization::printer_type & p) const
    {
        p.corner_stream() << "press mouse rbutton to add vertex" << cg::visualization::endl
                            << "double-click to clear" << cg::visualization::endl
                            << "press space to start a new contour" << cg::visualization::endl;
    }

    bool on_double_click(const point_2f & p)
    {
        points.clear();
        contours.clear();
        new_contour();
        visibility_segments.clear();
        return true;
    }

    bool on_press(const point_2f & p)
    {
        points.push_back(p);
        contours.back().add_point(p);
        calc_visibility_graph();
        return true;
    }

    bool on_key(int key)
    {
        switch (key)
        {
        case Qt::Key_Space:
            if (contours.back().size() != 0)
                new_contour();
            break;
        default:
            return false;
        }

        return true;
    }

    bool on_move(const point_2f & p)
    {
        cur_pos = p;
        return true;
    }

private:
    point_2 cur_pos;
    std::vector<contour_2> contours;
    std::vector<point_2> points;
    std::vector<segment_2> visibility_segments;
};

int main(int argc, char ** argv)
{
    QApplication app(argc, argv);
    visibility_viewer viewer;
    cg::visualization::run_viewer(&viewer, "Visibility viewer");
}
