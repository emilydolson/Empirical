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
//    * How do we want to hangle immobile objects?

#ifndef EMP_PHYSICS_2D_H
#define EMP_PHYSICS_2D_H

#include "../tools/Random.h"
#include "../tools/TypeTracker.h"

#include "../base/macros.h"

#include "../meta/meta.h"

#include "../geometry/Circle2D.h"
#include "../geometry/Rectangle2D.h"
#include "../geometry/Point2D.h"

#include "../control/SignalControl.h"

#include "PhysicsBody2D.h"
#include "PhysicsBodyOwner.h"

#include <algorithm>

namespace emp {

  template <typename... BODY_OWNERS>
  class CirclePhysics2D {
  protected:
    using Shape_t = Circle;
    using Body_t = PhysicsBody2D<Shape_t>;
    using Tracker_t = TypeTracker<BODY_OWNERS*...>;
    using CollisionResolutionFunction_t = std::function<void(Body_t *, Body_t *)>;

    Tracker_t body_owner_tt;
    emp::vector<Body_t*> body_set;
    Point *max_pos;           // Max position in physics.
    int max_radius;           // Max radius we know about. Gets updated on physics update.
    double friction;          // 'Friction' on 2D surface?
    bool configured;          // Has physics been configured yet?

    Random *random_ptr;

    Signal<void()> on_update_signal;

    CollisionResolutionFunction_t CollisionResolutionFunction;

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
      body_set.resize(cur_size);
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
      // This is the narrow phase of collision detection/resolution between body1 & body2.
      //  (In circle physics, all bodies are circles.)
      // @amlalejini: Do we want links to imply no collision going forward? Maximal flexibility
      //   on collision resolution seems best. Some types of links might want to collide?
      // If bodies are linked, no collision.
      if (body1->IsLinked(*body2)) return false;
      // First: Body-touching math.
      Point dist = body1->GetShape().GetCenter() - body2->GetShape().GetCenter();
      const double sq_pair_dist = dist.SquareMagnitude();
      const double radius_sum = body1->GetShape().GetRadius() + body2->GetShape().GetRadius();
      const double sq_min_dist = radius_sum * radius_sum;
      // No touching, no collision.
      if (sq_pair_dist >= sq_min_dist) return false;
      // Must be touching, collision!
      body1->TriggerCollision(body2);   // Give bodies opportunity to respond to the collision.
      body2->TriggerCollision(body1);
      // Give owners opportunity to respond to collision. TODO: what if no owners?
      if (body1->IsColliding() || body2->IsColliding())
        body_owner_tt.RunFunction(body1->GetTrackedOwnerPtr(), body2->GetTrackedOwnerPtr());
      // If collision is not resolved yet, fall back to default behavior.
      if (body1->IsColliding() || body2->IsColliding())
        this->CollisionResolutionFunction(body1, body2);
      return true;
    }

    // Default collision resolution function
    //  - Simple impulse resolution.
    //  - Currently no support for rotational velocity.
    void DefaultCollisionResolution(Body_t *body1, Body_t *body2) {
      // TODO: @amlalejini: This math is redundant. Could have a collision info struct that gets passed around.
      Point dist = body1->GetShape().GetCenter() - body2->GetShape().GetCenter();
      double sq_pair_dist = dist.SquareMagnitude();
      const double radius_sum = body1->GetShape().GetRadius() + body2->GetShape().GetRadius();
      const double sq_min_dist = radius_sum * radius_sum;
      // If bodies are directly (and centered) on top of one another, shift one such that they are no longer on top of each other.
      if (sq_pair_dist == 0.0) {
        body2->GetShape().Translate(Point(0.01, 0.01));
        dist = body1->GetShape().GetCenter() - body2->GetShape().GetCenter();
        sq_pair_dist = dist.SquareMagnitude();
      }
      // Re-adjust position to remove overlap.
      const double true_dist = sqrt(sq_pair_dist);
      const double overlap_dist = ((double)radius_sum) - true_dist;
      const double overlap_frac = overlap_dist / true_dist;
      const Point cur_shift = dist * (overlap_frac / 2.0);
      body1->AddShift(cur_shift);
      body2->AddShift(-cur_shift);
      // Resolve collision using impulse resolution.
      const double coeff_of_restitution = 1.0;
      const Point collision_normal(dist / true_dist);
      const Point rel_velocity(body1->GetVelocity() - body2->GetVelocity());
      const double velocity_along_normal = (rel_velocity.GetX() * collision_normal.GetX())
                                         + (rel_velocity.GetY() * collision_normal.GetY());
      // If velocities are separating, we can go ahead and resolve the collision.
      if (velocity_along_normal > 0) {
        body1->ResolveCollision(); body2->ResolveCollision();
        return;
      }
      double j = -(1 + coeff_of_restitution) * velocity_along_normal; // Calculate j, the impulse scalar.
      j /= body1->GetInvMass() + body2->GetInvMass();
      const Point impulse(collision_normal * j);
      // Apply the impulse.
      body1->SetVelocity(body1->GetVelocity() + (impulse * body1->GetInvMass()));
      body2->SetVelocity(body2->GetVelocity() - (impulse * body2->GetInvMass()));
      // Mark collision as resolved.
      body1->ResolveCollision(); body2->ResolveCollision();
    }

    void FinalizeBodies() {
      for (auto *body : body_set) body->FinalizePosition(*max_pos);
    }

  public:
    CirclePhysics2D(): max_radius(0), friction(0), configured(false) {
      emp_assert(sizeof...(BODY_OWNERS) > 0); // (TODO:? @amlalejini: should we allow for bodies with no owners?)
    }

    CirclePhysics2D(double width, double height, Random *r, double _friction) {
      emp_assert(sizeof...(BODY_OWNERS) > 0); // (TODO: @amlalejini: see todo from default constructor)
      ConfigPhysics(width, height, r, _friction);
    }

    ~CirclePhysics2D() {
      emp_assert(configured);
    //   delete max_pos;
      Clear();
    }

    // Call GetTypeID<type_name>() to get the ID associated with owner type type_name.
    template<typename T>
    constexpr static int GetTypeID() { return get_type_index<T, BODY_OWNERS...>(); }
    // Call GetTypeID(owner) to get the ID associated with 'owner'.
    template <typename T>
    constexpr static int GetTypeID(const T &) { return get_type_index<T, BODY_OWNERS...>(); }
    // Given a body and a BODY_OWNER type, is the body's owner of type BODY_OWNER?
    template <typename BODY_OWNER>
    bool IsBodyOwnerType(Body_t *body) {
      return body_owner_tt.template IsType<BODY_OWNER*>(*(body->GetTrackedOwnerPtr()));
    }
    // Cast body to type BODY_OWNER.
    template <typename BODY_OWNER>
    BODY_OWNER * ToBodyOwnerType(Body_t *body) {
      return body_owner_tt.template ToType<BODY_OWNER*>(*(body->GetTrackedOwnerPtr()));
    }

    CirclePhysics2D & Clear() {
      // @amlalejini: For now, physics is responsible for deleting bodies managed by physics.
      for (auto * body : body_set) {
        delete body;
      }
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
      CollisionResolutionFunction = [this](Body_t *body1, Body_t *body2)
                                          { this->DefaultCollisionResolution(body1, body2); };
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
      //body_ptr->AttachTrackedOwner(body_owner_tt.template New<OWNER_TYPE*>(body_owner));
      // TODO: make this less gross.
      // TODO: @ELD - I don't understand how the body doesn't already have an owner
      body_ptr->AttachTrackedOwner(body_owner_tt.template New<OWNER_TYPE*>(body_owner), body_owner);
      body_set.push_back(body_ptr);
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

    // Callback registration.
    void RegisterOnUpdateCallback(std::function<void()> callback) {
      on_update_signal.AddAction(callback);
    }

    // Handler registration.
    template <typename T1, typename T2>
    void RegisterCollisionHandler(std::function<void(T1*, T2*)> fun) {
      emp_assert(GetTypeID<T1>() >= 0 && GetTypeID<T2>() >= 0);
      body_owner_tt.AddFunction(fun);
      // If types are not the same, register same function but with diff arg ordering.
      if (GetTypeID<T1>() != GetTypeID<T2>()) {
        std::function<void(T2*, T1*)> f2 = [fun](T2 *t2, T1 *t1) { fun(t1, t2); };
        body_owner_tt.AddFunction(f2);
      }
    }

    // Allow setting of default collision resolution function.
    void SetDefaultCollisionResolutionFunction(CollisionResolutionFunction_t fun) {
      CollisionResolutionFunction = fun;
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
      // Test for collisions, resolving any found.
      if (max_radius > 0.0) TestCollisions();
      // Finalize bodies for this update (cleanup).
      FinalizeBodies();
    }
  };

  // @ELD: This is probably going to be an awful first pass
  // Currently wants a sequence of paired template arguments, a BODY_OWNER type
  // and the shape of the body that it owns.
  // One day we probably want the same owner type to be able to have multiple
  // shapes but we are not going there yet.
  template <typename... BODY_OWNERS>
  class MixedPhysics2D : public CirclePhysics2D<BODY_OWNERS...> {
    protected:

        //TODO: make more inclusive
        using body_tracker_t = TypeTracker<emp::PhysicsBody2D<emp::Circle>*, emp::PhysicsBody2D<emp::Rect>* >;
        using Body_t = emp::TrackedType;
        using body_owner_tracker_t = TypeTracker<BODY_OWNERS*...>;
        using CollisionResolutionFunction_t = std::function<void(Body_t *, Body_t *)>;

        body_tracker_t body_tt;
        body_owner_tracker_t body_owner_tt;
        CollisionResolutionFunction_t CollisionResolutionFunction;
        emp::vector<Body_t*> body_set;
        using CirclePhysics2D<BODY_OWNERS...>::max_pos;
        using CirclePhysics2D<BODY_OWNERS...>::max_radius;
        using CirclePhysics2D<BODY_OWNERS...>::friction;
        using CirclePhysics2D<BODY_OWNERS...>::configured;
        using CirclePhysics2D<BODY_OWNERS...>::random_ptr;
        using CirclePhysics2D<BODY_OWNERS...>::on_update_signal;

        using CirclePhysics2D<BODY_OWNERS...>::UpdateBodies;

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
          for (auto *tt_body : body_set) {
            auto body = body_tt.ToType(tt_body);
            emp_assert(body != nullptr);
            // Determine which sector the current body is in.
            const int cur_col = MakeRange(0, max_col).Valid(body->GetAnchor().GetX()/sector_width);
            const int cur_row = MakeRange(0, max_row).Valid(body->GetAnchor().GetY()/sector_height);
            // See if this body may collide with any of the bodies in this sector and in adjacent sectors.
            for (int k = std::max(0, cur_col-1); k <= std::min(cur_col+1, max_col); k++) {
              for (int j = std::max(0, cur_row-1); j <= std::min(cur_row+1, max_row); j++) {
                const int sector_id = k + num_cols * j;
                if (sector_set[sector_id].size() == 0) continue; // Nothing in current sector.
                for (int i = 0; i < sector_set[sector_id].size(); i++) {
                    auto body2 = body_tt.ToType(sector_set[sector_id][i]);
                    CollideBodies(body, body2);
                }
              }
            }
            // Add current body to the appropriate sector.
            const int cur_sector = cur_col + cur_row * num_cols;
            emp_assert(cur_sector < (int) sector_set.size());
            sector_set[cur_sector].push_back(body);
          }
        }

        bool CollideBodies(Body_t *body1, Body_t *body2) {
          // This is the narrow phase of collision detection/resolution between body1 & body2.
          //  (In circle physics, all bodies are circles.)
          // @amlalejini: Do we want links to imply no collision going forward? Maximal flexibility
          //   on collision resolution seems best. Some types of links might want to collide?
          // If bodies are linked, no collision.
          if (body_tt.ToType(body1)->IsLinked(*body2)) return false;
          // First: Body-touching math.
          Point dist = body_tt.ToType(body1)->GetShape().GetCenter() - body_tt.ToType(body2)->GetShape().GetCenter();
          const double sq_pair_dist = dist.SquareMagnitude();
          const double radius_sum = body_tt.ToType(body1)->GetShape().GetRadius() + body_tt.ToType(body2)->GetShape().GetRadius();
          const double sq_min_dist = radius_sum * radius_sum;
          // No touching, no collision.
          if (sq_pair_dist >= sq_min_dist) return false;
          // Must be touching, collision!
          body_tt.ToType(body1)->TriggerCollision(body2);   // Give bodies opportunity to respond to the collision.
          body_tt.ToType(body2)->TriggerCollision(body1);
          // Give owners opportunity to respond to collision. TODO: what if no owners?
          if (body_tt.ToType(body1)->IsColliding() || body_tt.ToType(body2)->IsColliding())
            body_owner_tt.RunFunction(body_tt.ToType(body1)->GetTrackedOwnerPtr(), body_tt.ToType(body2)->GetTrackedOwnerPtr());
          // If collision is not resolved yet, fall back to default behavior.
          if (body_tt.ToType(body1)->IsColliding() || body_tt.ToType(body2)->IsColliding())
            this->CollisionResolutionFunction(body_tt.ToType(body1), body_tt.ToType(body2));
          return true;
        }

        using CirclePhysics2D<BODY_OWNERS...>::DefaultCollisionResolution;
        using CirclePhysics2D<BODY_OWNERS...>::FinalizeBodies;

      public:
        MixedPhysics2D(): CirclePhysics2D<BODY_OWNERS...>() {
          emp_assert(sizeof...(BODY_OWNERS) > 0); // (TODO:? @amlalejini: should we allow for bodies with no owners?)
        }

        MixedPhysics2D(double width, double height, Random *r, double _friction)
            : CirclePhysics2D<BODY_OWNERS...>(width, height, r, _friction) {
          emp_assert(sizeof...(BODY_OWNERS) > 0); // (TODO: @amlalejini: see todo from default constructor)
          ConfigPhysics(width, height, r, _friction);
        }

        ~MixedPhysics2D() {;}

        using CirclePhysics2D<BODY_OWNERS...>::GetTypeID;
        using CirclePhysics2D<BODY_OWNERS...>::IsBodyOwnerType;
        using CirclePhysics2D<BODY_OWNERS...>::ToBodyOwnerType;
        using CirclePhysics2D<BODY_OWNERS...>::Clear;

        using CirclePhysics2D<BODY_OWNERS...>::GetHeight;
        using CirclePhysics2D<BODY_OWNERS...>::GetWidth;
        using CirclePhysics2D<BODY_OWNERS...>::GetBodySet;
        using CirclePhysics2D<BODY_OWNERS...>::GetConstBodySet;
        using CirclePhysics2D<BODY_OWNERS...>::ConfigPhysics;

        template <typename OWNER_TYPE>
        MixedPhysics2D & AddBody(OWNER_TYPE * body_owner) {
          emp_assert(configured);
          // TODO: check that owner type is in BODY_OWNERS... && check that is a PhysicsBodyOwner type.
          //
          body_owner->SetBodyCleanup(false); // Physics will take cleanup responsibility.
          // Make sure we attach body to (tracked) owner.
          //body_ptr->AttachTrackedOwner(body_owner_tt.template New<OWNER_TYPE*>(body_owner));
          // TODO: make this less gross.
        //   body_owner->GetBodyPtr()->AttachTrackedOwner(body_owner_tt.template New<OWNER_TYPE*>(body_owner), body_owner);

          Body_t * body_ptr = body_tt.New<typename OWNER_TYPE::Body_t* >(body_owner->GetBodyPtr());
          body_set.push_back(body_ptr);
          return *this;
        }

        template <typename OWNER_TYPE>
        MixedPhysics2D & RemoveBody(OWNER_TYPE * body_owner) {
          emp_assert(configured);
          // TODO: check that owner type is in in BODY_OWNERS... && check that is a PhysicsBodyOwner type.
          Body_t * body_ptr = body_owner->GetBodyPtr();
          // Make body owner responsible for cleaning up memory now.
          body_owner->SetBodyCleanup(true);
          // Make sure we body gets detached from owner.
          body_tt.ToType(body_ptr)->DetachTrackedOwner();
          // Delete from body list. (this doesn't call body's destructor)
          body_set.erase(std::remove_if(body_set.begin(), body_set.end(), [body_ptr](Body_t *body_i) { return body_i == body_ptr; }));
          return *this;
        }

        using CirclePhysics2D<BODY_OWNERS...>::RegisterOnUpdateCallback;
        using CirclePhysics2D<BODY_OWNERS...>::RegisterCollisionHandler;
        using CirclePhysics2D<BODY_OWNERS...>::SetDefaultCollisionResolutionFunction;
        using CirclePhysics2D<BODY_OWNERS...>::Update;


  };

}
#endif
