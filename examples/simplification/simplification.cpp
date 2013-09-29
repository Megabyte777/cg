#include <vector>

#include <QColor>
#include <QApplication>

#include "cg/visualization/viewer_adapter.h"
#include "cg/visualization/draw_util.h"

#include "cg/simplification/douglas-peucker.h"

using cg::point_2;
using cg::point_2f;
using cg::vector_2f;

struct simplification_viewer : cg::visualization::viewer_adapter
{
    simplification_viewer()
    {
        in_building = true;
    }

    void simplify()
    {
        simplified_points_.clear();
        douglas_peucker(points_.begin(), points_.end(), eps, std::back_inserter(simplified_points_));
    }

    void draw(cg::visualization::drawer_type & drawer) const
    {
        if (in_building)
            drawer.set_color(Qt::white);
        else
            drawer.set_color(Qt::green);
        for (point_2 p : points_)
            drawer.draw_point(p, 5);
        std::vector<point_2> const &points = in_building ? points_ : simplified_points_;
        for (size_t i = 1; i < points.size(); ++i)
            drawer.draw_line(points[i - 1], points[i]);
        return;
    }

    void print(cg::visualization::printer_type & p) const
    {
        p.corner_stream() << "double-click to stop drawing" << cg::visualization::endl
                            << "double-click again to clear" << cg::visualization::endl
                            << "press mouse rbutton for add vertex" << cg::visualization::endl
                            << "press Up/Down to increase/decrease eps" << cg::visualization::endl
                            << "eps = " << eps << cg::visualization::endl;

        for (size_t i = 0; i < points_.size(); ++i)
            p.global_stream((point_2f)points_[i] + vector_2f(5, 0)) << i;
   }

    bool on_double_click(const point_2f & p)
    {
        if (in_building)
        {
            in_building = false;
            simplify();
        }
        else
        {
            in_building = true;
            points_.clear();
        }
        return true;
    }

    bool on_key(int key)
    {
        switch (key)
        {
        case Qt::Key_Up:
            eps += 1;
            break;
        case Qt::Key_Down:
            eps = std::max(0.0, eps - 1);
            break;
        default:
            return false;
        }

        if (!in_building)
            simplify();

        return true;
    }

    bool on_press(const point_2f & p)
    {
        if (in_building)
            points_.push_back(p);

        return true;
    }

private:
    bool in_building;
    double eps;
    std::vector<point_2> points_;
    std::vector<point_2> simplified_points_;
};

int main(int argc, char ** argv)
{
   QApplication app(argc, argv);
   simplification_viewer viewer;
   cg::visualization::run_viewer(&viewer, "simplification viewer");
}
