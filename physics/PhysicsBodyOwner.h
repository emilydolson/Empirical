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

  public:

    virtual ~PhysicsBodyOwner_Base() {
      if (has_body) {
        delete body;
        DetachBody();
      }
    }

    virtual BODY_TYPE * GetBodyPtr() { emp_assert(has_body); return body; };
    virtual BODY_TYPE & GetBody() { emp_assert(has_body); return *body; };
    virtual const BODY_TYPE & GetConstBody() const { emp_assert(has_body); return *body; }

    virtual bool HasBody() const { return has_body; }
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
