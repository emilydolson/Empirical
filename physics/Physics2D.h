//  This file is part of Empirical, https://github.com/devosoft/Empirical
//  Copyright (C) Michigan State University, 2016.
//  Released under the MIT Software license; see doc/LICENSE
//
//  This is a modified version of Physics2D.
//
//  Physics2D - handles movement and collissions in a simple 2D world.
//  This describes environment physics.
//  TODO:
//    * Can we setup to handle both owned and ownerless bodies? -- currently no.

#ifndef EMP_PHYSICS_2D_H
#define EMP_PHYSICS_2D_H

#include "../tools/Random.h"
#include "../tools/TypeTracker.h"

#include "../geometry/Circle2D.h"
#include "../geometry/Point2D.h"
#include "../control/SignalControl.h"

#include "PhysicsBody2D.h"

#include <algorithm>

namespace emp {

  template <typename... BODY_OWNERS>
  class CirclePhysics2D {
  protected:
    using Shape_t = Circle;
    using Body_t = PhysicsBody2D<Shape_t>;
    using Tracker_t = TypeTracker<BODY_OWNERS*...>;

    Tracker_t body_owner_tt;
    emp::vector<Body_t*> body_set;
    Point *max_pos;           // Max position in physics.
    int max_radius;           // Max radius we know about. Gets updated on physics update.
    double friction;          // 'Friction' on 2D surface?
    bool configured;          // Has physics been configured yet?

    Random *random_ptr;

    Signal<void()> on_update_signal;

    // Internal functions
    // Update all bodies.
    void UpdateBodies() {
      int cur_size = (int) body_set.size();
      int cur_id = 0;
      while (cur_id < cur_size) {
        Body_t *body = body_set[cur_id];
        // Has this body been flagged for removal?
        // TODO: check to make sure physics is still managing this body?
        if (body->GetDestroyFlag()) {
          delete body;
          --cur_size;
          body_set[cur_id] = body_set[cur_size];
          continue;
        // No body to be removed,
        } else {
          // Run a body update.
          body->Update(friction);
          // Update max radius.
          if (body->GetShape().GetRadius() > max_radius)
            max_radius = body->GetShape().GetRadius();
          ++cur_id;
        }
      }
    }

    void TestCollisions() {
      // Test for collisions among bodies by dividing world into sectors.
      //  * Compare only organisms that share a sector.
      //  * Will divide world up into a maximum of 1024 sectors.
      const int num_cols = std::min<int>(GetWidth() / (max_radius * 2.0), 32);
      const int num_rows = std::min<int>(GetHeight() / (max_radius * 2.0), 32);
      const int max_col = num_cols - 1;
      const int max_row = num_rows - 1;
      const int num_sectors = num_cols * num_rows;
      // Calculate sector size.
      const double sector_width = GetWidth() / (double) num_cols;
      const double sector_height = GetHeight() / (double) num_rows;
      vector<vector<Body_t*>> sector_set(num_sectors);
      for (auto *body : body_set) {
        emp_assert(body != nullptr);
        // Determine which sector the current body is in.
        const int cur_col = MakeRange(0, max_col).Valid(body->GetAnchor().GetX()/sector_width);
        const int cur_row = MakeRange(0, max_row).Valid(body->GetAnchor().GetY()/sector_height);
        // See if this body may collide with any of the bodies in this sector and in adjacent sectors.
        for (int k = std::max(0, cur_col-1); k <= std::min(cur_col+1, max_col); k++) {
          for (int j = std::max(0, cur_row-1); j <= std::min(cur_row+1, max_row); j++) {
            const int sector_id = k + num_cols * j;
            if (sector_set[sector_id].size() == 0) continue; // Nothing in current sector.
            for (auto *body2 : sector_set[sector_id]) CollideBodies(body, body2);
          }
        }
        // Add current body to the appropriate sector.
        const int cur_sector = cur_col + cur_row * num_cols;
        emp_assert(cur_sector < (int) sector_set.size());
        sector_set[cur_sector].push_back(body);
      }
    }

    bool CollideBodies(Body_t *body1, Body_t *body2) {
      // TODO
      return true;
    }

  public:
    CirclePhysics2D(): max_radius(0), friction(0), configured(false) {
      //emp_assert(sizeof...(BODY_OWNERS) > 0); (TODO:? @amlalejini: should we allow empty body owners -- probably.)
    }

    CirclePhysics2D(double width, double height, Random *r, double _friction) {
      //emp_assert(sizeof...(BODY_OWNERS) > 0); (TODO:?)
      ConfigPhysics(width, height, r, _friction);
    }

    ~CirclePhysics2D() {
      emp_assert(configured);
      delete max_pos;
      Clear();
    }

    CirclePhysics2D & Clear() {
      std::cout << "Clear physics!" << std::endl;
      // @amlalejini: For now, physics is responsible for deleting bodies managed by physics.
      for (auto * body : body_set) delete body;
      body_set.resize(0);
      return *this;
    }

    double GetWidth() const { emp_assert(configured); return max_pos->GetX(); }
    double GetHeight() const { emp_assert(configured); return max_pos->GetY(); }
    emp::vector<Body_t*> & GetBodySet() { return body_set; }
    const emp::vector<Body_t*> & GetConstBodySet() const { return body_set; }

    // Configure physics. This must be called if default constructor was used when
    // creating the physics.
    void ConfigPhysics(double width, double height, Random *r, double _friction) {
      if (configured) {
        delete max_pos;
      }
      max_pos = new Point(width, height);
      random_ptr = r;
      configured = true;
      max_radius = 0;
      friction = _friction;
    }

    template <typename OWNER_TYPE>
    CirclePhysics2D & AddBody(OWNER_TYPE * body_owner) {
      emp_assert(configured);
      // TODO: check that owner type is in BODY_OWNERS... && check that is a PhysicsBodyOwner type.
      Body_t * body_ptr = body_owner->GetBodyPtr();
      body_owner->SetBodyCleanup(false); // Physics will take cleanup responsibility.
      // Make sure we attach body to (tracked) owner.
      body_ptr->AttachTrackedOwner(body_owner_tt.template New<OWNER_TYPE*>(body_owner));
      body_set.push_back(body_ptr);
      return *this;
    }

    // If we're just working with (ownerless) bodies, use this function.
    CirclePhysics2D & AddBody(Body_t * body) {
      emp_assert(configured);
      body_set.push_back(body);
      return *this;
    }

    template <typename OWNER_TYPE>
    CirclePhysics2D & RemoveBody(OWNER_TYPE * body_owner) {
      emp_assert(configured);
      // TODO: check that owner type is in in BODY_OWNERS... && check that is a PhysicsBodyOwner type.
      Body_t * body_ptr = body_owner->GetBodyPtr();
      // Make body owner responsible for cleaning up memory now.
      body_owner->SetBodyCleanup(true);
      // Make sure we body gets detached from owner.
      body_ptr->DetachTrackedOwner();
      // Delete from body list. (this doesn't call body's destructor)
      body_set.erase(std::remove_if(body_set.begin(), body_set.end(), [body_ptr](Body_t *body_i) { return body_i == body_ptr; }));
      return *this;
    }

    CirclePhysics2D & RemoveBody(Body_t * body) {
      emp_assert(configured);
      // Delete from body list.
      body_set.erase(std::remove_if(body_set.begin(), body_set.end(), [body](Body_t *body_i) { return body_i == body; }));
      return *this;
    }

    // Progress physics by a single time step.
    // TODO: @amlalejini: For on update signals, do we want multiple types of signals
    //        -- pre update bodies, post update bodies/pre collision detection, post collision detection?
    void Update() {
      emp_assert(configured);
      on_update_signal.Trigger();
      // Update bodies.
      //  -- Advance physics body motions by 1 timestep, remove bodies flagged for destruction, update max radius.
      max_radius = 0;
      UpdateBodies();
      // TODO: Test for collisions.
      TestCollisions();
      // TODO: Cleanup.
    }

  };

}
#endif
