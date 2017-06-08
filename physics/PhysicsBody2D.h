//  This file is part of Empirical, https://github.com/devosoft/Empirical
//  Copyright (C) Michigan State University, 2016.
//  Released under the MIT Software license; see doc/LICENSE
//
//
// Body_Base is a base class, containing info unrelated to its shape or its owner.
// It *does* contain the set of signals that might need to be triggered, such as
// specific types of collisions.
//
// Body is a template class that takes SHAPE and OWNER as template arguments.
// It creates an intenral instance of SHAPE (which has a pointer back to Body_Base)
// and a pointer back to OWNER.
//
//
//  Development notes:
//  * If we are going to have a lot of links, we may want a better data structure than vector.
//    (if we don't have a lot, vector may be the best choice...)

#ifndef EMP_PHYSICSBODY_2D_H
#define EMP_PHYSICSBODY_2D_H

#include "../geometry/Point2D.h"
#include "../geometry/Shape2D.h"
#include "../control/Signal.h"

#include "../tools/TypeTracker.h"
#include "PhysicsBodyOwner.h"

#include <functional>

namespace emp {
  class PhysicsBody2D_Base;

  // TODO: Discuss the fate of BODY_LINK_TYPE. Should we still be using this? Or is there something
  //  better?
  //  * We need a way to handle different types of links.
  // At the moment, bodies can be linked in several ways.
  // DEFAULT -> Joined together with no extra meaning
  // REPRODUCTION -> "from" is gestating "to"
  // ATTACK -> "from" is trying to eat "to"
  // PARASITE -> "from" is stealing resources from "to"
  // CONSUME_RESOURCE -> "from" is eating "to" where "from" is an organism and "to" is a resource.
  enum class BODY_LINK_TYPE { DEFAULT, REPRODUCTION, ATTACK, PARASITE, CONSUME_RESOURCE };

  // Physics bodies can have links between one another.
  struct BodyLink {
    using PhysicsBody_t = PhysicsBody2D_Base;
    BODY_LINK_TYPE type;
    emp::Ptr<PhysicsBody_t> from;
    emp::Ptr<PhysicsBody_t> to;
    double cur_dist;
    double target_dist;
    double link_strength;
    bool destroy;
    BodyLink() : type(BODY_LINK_TYPE::DEFAULT), from(nullptr), to(nullptr), cur_dist(0),
                 target_dist(0), link_strength(0), destroy(false) { ; }
    BodyLink(BODY_LINK_TYPE t, PhysicsBody_t * _from, PhysicsBody_t * _to, double _cur_dist = 0,
              double _target_dist = 0, double _link_strength = 0)
              : type(t), from(_from), to(_to), cur_dist(_cur_dist), target_dist(_target_dist),
                link_strength(_link_strength), destroy(false) { ; }
    ~BodyLink() { ; }
  };

  class PhysicsBody2D_Base {
  protected:
    // Body properties:
    Point velocity; // Speed and direction of movement.
    double mass;
    double inv_mass;
    double pressure;
    double max_pressure;
    bool destroy = false; // Has this body been flagged for destruction?
    bool is_immobile;
    bool is_colliding = false;
    // Useful internal member variables:
    Point shift;            // How should this body be updated to minimize overlap?
    Point cum_shift;        // Build up of shift not yet acted upon.
    Point total_abs_shift;  // Total absolute-value of shifts (to calculate pressure)

    // Signals:
    // TODO
    Signal<void(emp::Ptr<BodyLink>)> on_link_update_signal;  // Triggers for each link updated on body.

    // Information about other bodies that this body is linked to:
    emp::vector<emp::Ptr<BodyLink>> from_links;  // Active links initiated (from) this body.
    emp::vector<emp::Ptr<BodyLink>> to_links;    // Active links targeting (to) this body.

    void RemoveFromLink(int link_id) {
      emp_assert(link_id >= 0 && link_id < (int) from_links.size());
      from_links[link_id] = from_links.back();
      from_links.pop_back();
    }

    void RemoveToLink(int link_id) {
      emp_assert(link_id >= 0 && link_id < (int) to_links.size());
      to_links[link_id] = to_links.back();
      to_links.pop_back();
    }

    PhysicsBody2D_Base() : mass(1.0), inv_mass(1.0 / mass), pressure(0.0), max_pressure(1.0), destroy(false), is_immobile(false) { ; }

  public:
    virtual ~PhysicsBody2D_Base() {
    //  std::cout << "In body destrcuto " << std::endl;
      // Delete all links
      RemoveAllLinks();
    //   std::cout << "Done with base detructor " << std::endl;
    }

    virtual Shape* GetShapePtr() {std::cout << "NOOOOOOOOOOOOOOOO!" << std::endl;};
    virtual Shape & GetShape() = 0 ;
    virtual const Shape & GetConstShape() const = 0;

    virtual const Point & GetVelocity() const { return velocity; }
    virtual const Point & GetAnchor() const = 0;
    virtual double GetMass() const { return mass; }
    virtual double GetInvMass() const { return inv_mass; }
    virtual double GetPressure() const { return pressure; }
    virtual double GetMaxPressure() const { return max_pressure; }
    virtual bool GetDestroyFlag() const { return destroy; }
    virtual bool ExceedsStressThreshold() const { return pressure > max_pressure; }
    virtual bool IsColliding() const { return is_colliding; }
    virtual bool IsImmobile() const { return is_immobile; }

    virtual void SetVelocity(double x, double y) { velocity.Set(x, y); }
    virtual void SetVelocity(const Point & v) { velocity = v; }
    virtual void SetMass(double m) { mass = m; mass == 0.0 ? inv_mass = 0 : inv_mass = 1.0 / mass; }
    virtual void SetPressure(double p) { pressure = p; }
    virtual void SetMaxPressure(double mp) { max_pressure = mp; }
    virtual void FlagForDestruction() { destroy = true; }
    virtual void SetImmobile(bool im) { is_immobile = im; }

    virtual void IncVelocity(const Point & offset) { velocity += offset; }
    virtual void DecVelocity(const Point & offset) { velocity -= offset; }
    virtual void AddShift(const Point & s) { shift += s; total_abs_shift += s.Abs(); }

    virtual void ResolveCollision() { is_colliding = true; }
    virtual void TriggerCollision(PhysicsBody2D_Base * other_body) {
      // TODO
      is_colliding = true;
      //on_collision_signal.Trigger(other_body);
    }

    // TODO: register more callbacks.
    virtual void RegisterOnLinkUpdateCallback(std::function<void(BodyLink*)> callback) {
      on_link_update_signal.AddAction(callback);
    }

    // Creating, testing, and unlinking other bodies.
    virtual bool IsLinkedFrom(const PhysicsBody2D_Base & link_body) const {
      for (auto cur_link : from_links) if (cur_link->to == &link_body) return true;
      return false;
    }
    virtual bool IsLinkedTo(const PhysicsBody2D_Base & link_body) const { return link_body.IsLinkedFrom(*this); }
    virtual bool IsLinked(const PhysicsBody2D_Base & link_body) const {
      return IsLinkedFrom(link_body) || IsLinkedTo(link_body);
    }
    virtual int GetLinkCount() const { return (int) (from_links.size() + to_links.size()); }

    // Add link FROM this TO link_body.
    virtual void AddLink(BODY_LINK_TYPE type, PhysicsBody2D_Base & link_body, double cur_dist, double target_dist, double link_strength = 0) {
      emp_assert(!IsLinked(link_body)); // Don't link twice!
      // Build connections in both directions.
      auto new_link = new BodyLink(type, this, &link_body, cur_dist, target_dist, link_strength);
      from_links.push_back(new_link);
      link_body.to_links.push_back(new_link);
    }
    virtual void RemoveLink(emp::Ptr<BodyLink> link) {
      // If link is to this body, trigger remove link from source body.
      if (link->to.Raw() == this) {
        link->from->RemoveLink(link);
        return;
      }
      // Otherwise, handle link cleanup here.
      // Remove the FROM link.
      for (int i = 0; i < (int) from_links.size(); i++) {
        if (from_links[i]->to == link->to) { RemoveFromLink(i); break; }
      }
      // Remove the TO link.
      const int to_size = (int) link->to->to_links.size();
      for (int i = 0; i < to_size; i++) {
        if (link->to->to_links[i]->from.Raw() == this) { link->to->RemoveToLink(i); break; }
      }
      link.Delete();
    }
    virtual void RemoveAllLinks() {
        // std::cout << "removing links" << from_links.size() << std::endl;
      while (from_links.size()) RemoveLink(from_links[0]);
      while (to_links.size()) RemoveLink(to_links[0]);
    }
    virtual const BodyLink & FindLink(const PhysicsBody2D_Base & link_body) const {
      emp_assert(IsLinked(link_body));
      for (auto link : from_links) if ( link->to.Raw() == &link_body) return *link;
      return link_body.FindLink(*this);
    }
    virtual BodyLink & FindLink(PhysicsBody2D_Base & link_body)  {
      emp_assert(IsLinked(link_body));
      for (auto link : from_links) if ( link->to.Raw() == &link_body) return *link;
      return link_body.FindLink(*this);
    }
    virtual emp::vector<emp::Ptr<BodyLink>> GetLinksToByType(BODY_LINK_TYPE link_type) {
      emp::vector<emp::Ptr<BodyLink>> links;
      for (auto link : this->to_links) {
        if (link->type == link_type) links.push_back(link);
      }
      return links;
    }
    virtual emp::vector<emp::Ptr<BodyLink>> GetLinksFromByType(BODY_LINK_TYPE link_type) {
      emp::vector<emp::Ptr<BodyLink>> links;
      for (auto link : this->from_links) {
        if (link->type == link_type) links.push_back(link);
      }
      return links;
    }
    virtual double GetLinkDist(const PhysicsBody2D_Base & link_body) const {
      emp_assert(IsLinked(link_body));
      return FindLink(link_body).cur_dist;
    }
    virtual double GetTargetLinkDist(const PhysicsBody2D_Base & link_body) const {
      emp_assert(IsLinked(link_body));
      return FindLink(link_body).target_dist;
    }
    virtual void ShiftLinkDist(PhysicsBody2D_Base & link_body, double change) {
      auto & link = FindLink(link_body);
      link.cur_dist += change;
    }
  };

  // Bodies can be different shapes and have owners.
  //  * Shape type comes in on template.
  //  * Owner must be TrackedType
  template<typename SHAPE_TYPE>
  class PhysicsBody2D : public PhysicsBody2D_Base {
  protected:
    using Shape_t = SHAPE_TYPE;
    using Owner_t = PhysicsBodyOwner_Base<PhysicsBody2D<Shape_t>>;

    emp::Ptr<Shape_t> shape_ptr = nullptr;
    emp::Ptr<TrackedType> tracked_owner = nullptr;
    emp::Ptr<Owner_t> body_owner = nullptr;
    bool has_owner = false;


  public:
    template<typename... ARGS>
    PhysicsBody2D(ARGS... args) : tracked_owner(nullptr), has_owner(false) {
      shape_ptr.New(Shape_t(std::forward<ARGS>(args)...));
    }
    ~PhysicsBody2D() {
        // std::cout << "In body derived dest" << std::endl;
      if (shape_ptr) {
        //   std::cout << "Deleting " << shape_ptr.Raw() << std::endl;
          shape_ptr.Delete();
      }
      // TODO: we need to tell owner (if we have an owner) that we've deleted this body.
      //      * Currently not a huge fan of this way of doing it.
      if (has_owner) body_owner->DetachBody();
    //   std::cout << "Done deleting body" << std::endl;
    }

    Shape_t* GetShapePtr() override { emp_assert(!shape_ptr.IsNull()); std::cout << "getting shape ptr" << std::endl; return shape_ptr; }
    Shape_t & GetShape() override { emp_assert(!shape_ptr.IsNull()); return *shape_ptr; }
    const Shape_t & GetConstShape() const override { return *shape_ptr; }

    TrackedType * GetTrackedOwnerPtr() { return tracked_owner; }

    bool HasOwner() const { return has_owner; }

    const Angle & GetOrientation() const { return shape_ptr->GetOrientation(); }
    const Point & GetAnchor() const override { return shape_ptr->GetCenter(); }

    void AttachTrackedOwner(emp::Ptr<TrackedType> ptr) {
      emp_assert(!ptr.IsNull() && has_owner != true);
      tracked_owner = ptr;
      has_owner = true;
    }
    void AttachTrackedOwner(emp::Ptr<TrackedType> ptr, emp::Ptr<Owner_t> o_ptr) {
    //   emp_assert(!ptr.IsNull() && has_owner != true, ptr.Raw(), has_owner);
    //   std::cout << "aabout to attach " << tracked_owner.DebugGetCount() << " " << body_owner.DebugGetCount() << " " << ptr.DebugGetCount() << " " << o_ptr.DebugGetCount() << std::endl;
    //   std::cout << tracked_owner.Raw() << " " << ptr.Raw() << std::endl;
    //   std::cout << body_owner.Raw() << " " << o_ptr.Raw() << std::endl;
      if (has_owner) {
          // So this is pretty awful - we can end up with a different tracked pointer
          // to the same owner
          tracked_owner.Delete();
      }
      tracked_owner = ptr;
      body_owner = o_ptr;
      has_owner = true;
    //   std::cout << "attaching..."<< tracked_owner.DebugGetCount() << " " << body_owner.DebugGetCount() << " " << ptr.DebugGetCount() << " " << o_ptr.DebugGetCount() <<std::endl;
    }

    void DetachTrackedOwner() {
      if (has_owner){
          tracked_owner.Delete();
          tracked_owner = nullptr;
      }
      body_owner = nullptr;
      has_owner = false;
    }

    // Update body.
    void Update(double friction = 0) {
      // Update links.
      for (int i = 0; i < (int) from_links.size(); ++i) {
        emp::Ptr<BodyLink> link = from_links[i];
        // Trigger on link update.
        on_link_update_signal.Trigger(link);
        // Is this link flagged for destruction?
        if (link->destroy) {
          RemoveLink(link);
          continue;
        }
        if (link->cur_dist == link->target_dist) continue;  // No adjustment needed.
        // If we're within the change_factor, just set the pair_dist to target.
        // TODO: get rid of magic numbers (how much should current distance be allowed to change per physics update).
        const double change_factor = 0.25;
        if (std::abs(link->cur_dist - link->target_dist) <= change_factor) {
            link->cur_dist = link->target_dist;
        } else {
          if (link->cur_dist < link->target_dist) link->cur_dist += change_factor;
          else link->cur_dist -= change_factor;
        }
      }
      // Move body by its velocity modified by friction.
      if (!is_immobile && velocity.NonZero()) {
        shape_ptr->Translate(velocity);
        const double velocity_mag = velocity.Magnitude();
        // If body is about to stop, go ahead and stop it.
        if (friction > velocity_mag) velocity.ToOrigin();
        else velocity *= 1.0 - (friction / velocity_mag);
      }
    }

    // Finalize body's position in world for physics update.
    void FinalizePosition(const Point & max_coords) {
      // What's the max x,y that this body is allowed to have?
      const double max_x = max_coords.GetX() - shape_ptr->GetRadius();
      const double max_y = max_coords.GetY() - shape_ptr->GetRadius();

      // Handle accumulated shift.
      if (!is_immobile) {
        cum_shift += shift;
        shape_ptr->Translate(cum_shift);
        cum_shift.ToOrigin();
      }

      // Calculate pressure (TODO: we might want to reconsider how this gets computed?)
      pressure = (total_abs_shift - shift.Abs()).SquareMagnitude();
      shift.ToOrigin();
      total_abs_shift.ToOrigin();

      // If this body is linked to another, enforce the distance between them.
      for (auto link : from_links) {
        // If bodies are directly on top of one another (centers are overlapping), move this a bit.
        if (GetAnchor() == link->to->GetAnchor()) shape_ptr->Translate(Point(0.01, 0.01));
        // Figure out how much each body needs to move so that cur_dist (updated during body update step) will be correct.
        const double start_dist = GetAnchor().Distance(link->to->GetAnchor());
        const double link_dist = link->cur_dist;
        const double frac_change = (1.0 - (link_dist / start_dist)) / 2.0;
        // Move things as necessary.
        Point dist_move = (GetAnchor() - link->to->GetAnchor()) * frac_change;
        shape_ptr->Translate(-dist_move);
        link->to->GetShapePtr()->Translate(dist_move);
      }

      // Adjust the organism so it stays within the bounding box of the world.
      if (shape_ptr->GetCenterX() < shape_ptr->GetRadius()) {
        shape_ptr->SetCenterX(shape_ptr->GetRadius());
        velocity.NegateX();
      } else if (shape_ptr->GetCenterX() > max_x) {
        shape_ptr->SetCenterX(max_x);
        velocity.NegateX();
      }
      if (shape_ptr->GetCenterY() < shape_ptr->GetRadius()) {
        shape_ptr->SetCenterY(shape_ptr->GetRadius());
        velocity.NegateY();
      } else if (shape_ptr->GetCenterY() > max_y) {
        shape_ptr->SetCenterY(max_y);
        velocity.NegateY();
      }
    }
  };
}


#endif
