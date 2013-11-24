#pragma once

#include <QtOpenGL>

#include <unordered_map>
#include <cmath>

#include "cg/visualization/viewer.h"

#include "cg/primitives/vector.h"

namespace cg {
namespace visualization
{
   struct drawer_impl : drawer_type
   {
      void set_color(QColor const & c);
      void draw_line(segment_2f const &, float width);
      void draw_line(point_2f const &, point_2f const &, float width);
      void draw_point(point_2f const & pt, float radius);
      void draw_circle(point_2f const &center, float radius, float width);
      void draw_circle(circle_2f const &c, float width);

      drawer_impl()
         : current_color_ (Qt::black)
      {}

      void clear();

      struct point_buffer_t
      {
         std::vector<GLfloat>    points;
         std::vector<GLfloat>    colors;
      };

      struct segment_buffer_t
      {
         std::vector<GLfloat>    segments;
         std::vector<GLfloat>    colors;
      };

      std::unordered_map<float, point_buffer_t>     point_buffers;
      std::unordered_map<float, segment_buffer_t>   segment_buffers;

   private:
      QColor current_color_;
   };
}}
