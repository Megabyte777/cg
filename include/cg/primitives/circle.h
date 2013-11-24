#pragma once

#include "point.h"

namespace cg
{
   template <class Scalar> struct circle_2t;

   typedef circle_2t<double>   circle_2;
   typedef circle_2t<float>    circle_2f;
   typedef circle_2t<int>      circle_2i;

   template <class Scalar>
   struct circle_2t
   {
      point_2t<Scalar> center;
      Scalar radius;

      circle_2t(point_2t<Scalar> const &center, Scalar radius)
         : center(center)
         , radius(radius)
      {}

      template <class UScalar>
      circle_2t(circle_2t<UScalar> const & o)
         : center(o.center)
         , radius(o.radius)
      {}

      circle_2t()
         : center(0, 0)
         , radius(0)
      {}
   };
}
