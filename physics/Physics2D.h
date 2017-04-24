//  This file is part of Empirical, https://github.com/devosoft/Empirical
//  Copyright (C) Michigan State University, 2016.
//  Released under the MIT Software license; see doc/LICENSE
//
//  This is a modified version of Physics2D.
//
//  Physics2D - handles movement and collissions in a simple 2D world.
//  This describes environment physics.

#ifndef EMP_PHYSICS_2D_H
#define EMP_PHYSICS_2D_H

#include "../tools/Random.h"
#include "../tools/TypeTracker.h"

#include "../geometry/Circle2D.h"
#include "../geometry/Point2D.h"
#include "../control/SignalControl.h"

namespace emp {

  template <typename... BODY_OWNERS>
  class CirclePhysics2D {
  protected:
    using Shape_t = Circle;
    using Tracker_t = TypeTracker<BODY_OWNERS*...>;

    Tracker_t body_owner_tt;
    Point *max_pos;           // Max position in physics.
    bool configured;          // Has physics been configured yet?
    Random *random_ptr;

    Signal<void()> on_update_signal;

    // Internal functions


  public:
    CirclePhysics2D(): configured(false) { emp_assert(sizeof...(BODY_OWNERS) > 0); }

    CirclePhysics2D(double width, double height, Random *r, double surface_friction) {
      emp_assert(sizeof...(BODY_OWNERS) > 0);
      ConfigPhysics(width, height, r, surface_friction);
    }

    ~CirclePhysics2D() {
      emp_assert(configured);
      delete max_pos;
    }

    CirclePhysics2D & Clear() {
      // TODO:
      // clear bodies.
      return *this;
    }

    double GetWidth() const { emp_assert(configured); return max_pos->GetX(); }
    double GetHeight() const { emp_assert(configured); return max_pos->GetY(); }

    // Configure physics. This must be called if default constructor was used when
    // creating the physics.
    void ConfigPhysics(double width, double height, Random *r, double surface_friction) {
      if (configured) {
        delete max_pos;
      }
      max_pos = new Point(width, height);
      random_ptr = r;
      configured = true;
    }

    // Progress physics by a single time step.
    void Update() {
      emp_assert(configured);
      on_update_signal.Trigger();
      // Update bodies.
      // Test for collisions.
      // Cleanup.
    }

  };

}
#endif
