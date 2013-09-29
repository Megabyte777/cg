#include <vector>

#include <QColor>
#include <QApplication>

#include "cg/visualization/viewer_adapter.h"
#include "cg/visualization/draw_util.h"

#include <cg/operations/diameter.h>

using cg::point_2;
using cg::point_2f;
using cg::vector_2f;

struct diameter_viewer : cg::visualization::viewer_adapter
{
    void find_diameter()
    {
        auto diam = diameter(points_.begin(), points_.end());
        a = *diam.first;
        b = *diam.second;
    }

    void draw(cg::visualization::drawer_type & drawer) const
    {
        drawer.set_color(Qt::green);
        for (point_2 p : points_)
            drawer.draw_point(p, 5);
        if (points_.size() > 0)
            drawer.draw_line(a, b);
        return;
    }


    void print(cg::visualization::printer_type & p) const
    {
        p.corner_stream() << "click mouse rbutton to add vertex" << cg::visualization::endl
                            << "double-click to clear" << cg::visualization::endl;

        for (size_t i = 0; i < points_.size(); ++i)
            p.global_stream((point_2f)points_[i] + vector_2f(5, 0)) << i;
   }

    bool on_double_click(const point_2f & p)
    {
        points_.clear();
        return true;
    }

    bool on_press(const point_2f & p)
    {
        points_.push_back(p);
        find_diameter();

        return true;
    }

private:
    point_2 a, b;
    std::vector<point_2> points_;
};

int main(int argc, char ** argv)
{
   QApplication app(argc, argv);
   diameter_viewer viewer;
   cg::visualization::run_viewer(&viewer, "diameter viewer");
}
