//  This file is part of Empirical, https://github.com/devosoft/Empirical
//  Copyright (C) Michigan State University, 2016-2017.
//  Released under the MIT Software license; see doc/LICENSE
//
//  A class to manage circles in a 2D plane.

// TODO(discuss): @amlalejini: Had to nuke constexpr to make inheritance from Shape2D work.
//    -- But, it *is* nice to have Circle2D class that can be constructed and operated on at compile time.
//    -- What do?

#ifndef EMP_CIRCLE_2D_H
#define EMP_CIRCLE_2D_H

#include "Shape2D.h"
#include "Point2D.h"

#include "../base/assert.h"

namespace emp {

  template <typename T>
  class Rect2D;

  template <typename TYPE=double> class Circle2D : public Shape2D {
  private:
    Point2D<TYPE> center;
    TYPE radius;

  public:
    Circle2D(const Point2D<TYPE> & _c, TYPE _r=0) : center(_c), radius(_r) { ; }
    Circle2D(TYPE _x, TYPE _y, TYPE _r=0) : center(_x,_y), radius(_r) { ; }
    Circle2D(TYPE _r=0) : center(0.0, 0.0), radius(_r) { ; }

    const Point2D<TYPE> & GetCenter() const override { return center; }
    TYPE GetCenterX() const override { return center.GetX(); }
    TYPE GetCenterY() const override { return center.GetY(); }
    TYPE GetRadius() const override { return radius; }
    TYPE GetSquareRadius() const override {  return radius * radius; }

    Circle2D<TYPE> & SetCenter(const Point2D<TYPE> & c) override { center = c; return *this; }
    Circle2D<TYPE> & SetCenter(TYPE x, TYPE y) { center.Set(x,y); return *this; }
    Circle2D<TYPE> & SetCenterX(TYPE x) override { center.SetX(x); return *this; }
    Circle2D<TYPE> & SetCenterY(TYPE y) override { center.SetY(y); return *this; }
    Circle2D<TYPE> & SetRadius(TYPE new_radius) override { radius = new_radius; return *this; }
    Circle2D<TYPE> & Set(const Point2D<TYPE> & c, TYPE r) { center = c; radius = r; return *this; }
    Circle2D<TYPE> & Set(TYPE x, TYPE y, TYPE r) { center.Set(x,y); radius = r; return *this; }

    Circle2D<TYPE> & Translate(Point2D<TYPE> shift) override { center += shift; return *this; }

    bool Contains(const Point2D<TYPE> & point) const {
      return center.SquareDistance(point) < GetSquareRadius();
    }
    bool Contains(TYPE x, TYPE y) const {
      return center.SquareDistance(Point2D<TYPE>(x,y)) < GetSquareRadius();
    }
    bool Contains(const Circle2D<TYPE> & other) const {
      return (other.center.Distance(center) + other.GetRadius()) < GetRadius();
    }
    bool HasOverlap(const Circle2D<TYPE> & other) const {
      const TYPE min_dist = radius + other.radius;
      return center.SquareDistance(other.center) < (min_dist * min_dist);
    }
    bool HasOverlap(const Rect2D<TYPE> & other) const {
        if (other.GetULX() + other.GetWidth() == other.GetLRX()) {
            // Rect is not rotated
            // I'm worried that there's a literal corner case this is missing
            // std::cout << GetCenterX() + GetRadius() << " " << other.GetULX()
            // << " | " << GetCenterX() - GetRadius() << " " << other.GetLRX()
            // << " | " << GetCenterY() + GetRadius() << " " << other.GetULY() << std::endl;
            return (GetCenterX() + GetRadius() > other.GetULX() &&
                   GetCenterX() - GetRadius() < other.GetLRX()) &&
                   (GetCenterY() + GetRadius() > other.GetULY() &&
                   GetCenterY() - GetRadius() < other.GetLRY());
        }
        emp_assert(false, "NOT IMPLEMENTED");
        return false;
    }


  };

  using Circle = Circle2D<double>;
}

#endif
