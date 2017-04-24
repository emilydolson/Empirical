//  This file is part of Empirical, https://github.com/devosoft/Empirical
//  Copyright (C) Michigan State University, 2016.
//  Released under the MIT Software license; see doc/LICENSE
//
//  Class to manage shapes in a 2D plane. Shapes may have owners (useful for say,
//  physics) or may not have owners (when one simply wants to draw a circle).
//
//  All shapes have a bounding circle that is guaranteed to contain the entirety
//  of the shape.

#ifndef EMP_SHAPE_2D_H
#define EMP_SHAPE_2D_H

#include "Point2D.h"
#include "Angle2D.h"

namespace emp {
  // Every shape should define a center and a radius from that center that
  // encompasses the entirety of the shape.
  class Shape2D {
  protected:
    Point center;
    double radius;
    Angle orientation;
    uint32_t color_id;

    Shape2D(const Point & _c, double _r=0) : center(_c), radius(_r) { ; }
    Shape2D(double _x, double _y, double _r=0) : center(_x,_y), radius(_r) { ; }
    Shape2D(double _r=0) : center(0.0, 0.0), radius(_r) { ; }

  public:
    virtual ~Shape2D() { ; }

    // virtual const Point & GetCenter() const { return center; }
    // virtual double GetCenterX() const { return center.GetX(); }
    // virtual double GetCenterY() const { return center.GetY(); }
    // virtual double GetRadius() const { return radius; }
    // virtual double GetSquareRadius() const { return radius * radius; }
    // virtual const Angle & GetOrientation() const { return orientation; }
    //
    // virtual Shape & SetCenter(const Point & new_center) { center = new_center; return *this; }
    // virtual Shape & SetCenterX(double new_x) { center.SetX(new_x); return *this; }
    // virtual Shape & SetCenterY(double new_y) { center.SetY(new_y); return *this; }
    // virtual Shape & SetRadius(double new_radius) { radius = new_radius; return *this; }
    // virtual Shape & SetOrientation(Angle new_orientation) { orientation = new_orientation; return *this; }
    // virtual Shape & Translate(Point shift) { center += shift; return *this; }
    //
    // uint32_t GetColorID() const { return color_id; }
    //
    // void SetColorID(uint32_t in_id) { color_id = in_id; }

  };
  using Shape = Shape2D;
}

#endif
