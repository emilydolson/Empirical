/*
  PhysicsBodyOwner2D.h

  A base class that implements the necessary functions for body owners.

*/
#include "PhysicsBody2D.h"

#ifndef EMP_PHYSICSBODYOWNER_H
#define EMP_PHYSICSBODYOWNER_H

namespace emp {

  template<typename BODY_TYPE>
  class PhysicsBodyOwner_Base {
  protected:
    BODY_TYPE *body;
    bool has_body;
    // DISCUSS: @amlalejini: I don't like that body_cleanup flag is necessary. Can we design this out?
    bool body_cleanup; // Is this object responsible for body cleanup (deletion)?

    PhysicsBodyOwner_Base() : body_cleanup(true) { ; }

  public:
    virtual ~PhysicsBodyOwner_Base() {
      if (has_body && body_cleanup) {
        delete body;
        DetachBody();
      }
    }

    virtual BODY_TYPE * GetBodyPtr() { emp_assert(has_body); return body; };
    virtual BODY_TYPE & GetBody() { emp_assert(has_body); return *body; };
    virtual const BODY_TYPE & GetConstBody() const { emp_assert(has_body); return *body; }

    virtual bool HasBody() const { return has_body; }
    virtual void SetBodyCleanup(bool val) { body_cleanup = val; } // Allows other things to alert this object that the other thing will take care of cleanup.
    virtual bool GetBodyCleanup() const { return body_cleanup; }  // Is this object flagged as responsible for body cleanup?
    virtual void AttachBody(BODY_TYPE * in_body) {
      body = in_body;
      has_body = true;
    }
    virtual void DetachBody() {
      body = nullptr;
      has_body = false;
    }
    virtual void Evaluate() {
      if (body->GetDestroyFlag()) {
        delete body;
        DetachBody();
      }
    }
  };

}
#endif
