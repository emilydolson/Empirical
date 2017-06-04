//  This file is part of Empirical, https://github.com/devosoft/Empirical
//  Copyright (C) Michigan State University, 2016-2017.
//  Released under the MIT Software license; see doc/LICENSE
//
//
//  The Action class provides a simple mechanism to abstract functions from their
//  underlying type and provide run-time names.
//
//  Actions can be a bit heavyweight, but can easily be converted to more lightweight
//  std:function objects.
//
//
//  Developer notes:
//  * Create an ActionDefaults class that can take fewer args than expected and fill in rest.
//  * Allow for named arguments?

#ifndef EMP_CONTROL_ACTION
#define EMP_CONTROL_ACTION

#include <functional>
#include <string>

namespace emp {

  class ActionBase {
  protected:
    std::string name;

    ActionBase(const std::string & in_name) : name(in_name) { ; }

    // Protected: ActionBase should not be copied directly, only through derived class.
    ActionBase(const ActionBase &) = default;
    ActionBase(ActionBase &&) = default;
    ActionBase & operator=(const ActionBase &) = default;
    ActionBase & operator=(ActionBase &&) = default;
  public:
    virtual ~ActionBase() { ; }

    const std::string & GetName() const { return name; }
    virtual size_t GetArgCount() const = 0;

    // Clone() will produce a pointer to a full copy of an Action, going through derived version.
    virtual ActionBase * Clone() const = 0;
  };

  template <size_t ARG_COUNT>
  class ActionSize : public ActionBase {
  protected:
    ActionSize(const std::string & in_name) : ActionBase(in_name) { ; }
  public:
    size_t GetArgCount() const { return ARG_COUNT; }
  };

  template <typename... ARGS> class Action;

  template <typename... ARGS>
  class Action<void(ARGS...)> : public ActionSize<sizeof...(ARGS)> {
  protected:
    std::function<void(ARGS...)> fun;
  public:
    using this_t = Action<void(ARGS...)>;
    using parent_t = ActionSize<sizeof...(ARGS)>;

    Action(const std::function<void(ARGS...)> & in_fun, const std::string & in_name="")
      : parent_t(in_name), fun(in_fun) { ; }
    template <typename RETURN>
    Action(const std::function<RETURN(ARGS...)> & in_fun, const std::string & in_name="")
      : parent_t(in_name)
      , fun([in_fun](ARGS &&... args){in_fun(std::forward<ARGS>(args)...);}) { ; }
    Action(const this_t &) = default;
    Action(this_t &&) = default;

    this_t & operator=(const this_t &) = default;
    this_t & operator=(this_t &&) = default;

    const std::function<void(ARGS...)> & GetFun() const { return fun; };

    void Call(ARGS &&... args) { return fun(std::forward<ARGS>(args)...); }

    this_t * Clone() const { return new this_t(*this); }
  };


  template <typename RETURN, typename... ARGS>
  class Action<RETURN(ARGS...)> : public ActionSize<sizeof...(ARGS)> {
  protected:
    std::function<RETURN(ARGS...)> fun;
  public:
    using fun_t = RETURN(ARGS...);
    using this_t = Action<fun_t>;
    using parent_t = ActionSize<sizeof...(ARGS)>;

    Action(const std::function<RETURN(ARGS...)> & in_fun, const std::string & in_name="")
      : parent_t(in_name), fun(in_fun) { ; }
    Action(const this_t &) = default;
    Action(this_t &&) = default;

    this_t & operator=(const this_t &) = default;
    this_t & operator=(this_t &&) = default;

    const std::function<fun_t> & GetFun() const { return fun; };

    RETURN Call(ARGS &&... args) { return fun(std::forward<ARGS>(args)...); }

    this_t * Clone() const { return new this_t(*this); }
  };


  template <typename RETURN, typename... ARGS>
  auto make_action(const std::function<RETURN(ARGS...)> & in_fun, const std::string & name="") {
    return Action<RETURN(ARGS...)>(in_fun, name);
  }
}

#endif
