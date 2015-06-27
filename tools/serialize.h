#ifndef EMP_SERIALIZE_H
#define EMP_SERIALIZE_H

//////////////////////////////////////////////////////////////////////////////////////////
//
//  Tools to save and load data from classes.
//
//  All of the important information about a class is stored in a DataPod, which can be
//  used to restore the class at a later time.
//
//  In order to setup a target class to be able to be serialized into a pod, you must
//  add a macro to include the needed functionality.  For a basic class, use:
//
//   EMP_SETUP_DATAPOD(ClassName, var1, var2, ...)
//
//  Where ClassName is the target class' name and var1, var2, etc are the names of the
//  member variables that also need to be stored.  Note that member variables can either
//  be either built-in types or custom types that have also had DataPods setup in them.
//
//  If the target class is a derived class, you must use either:
//
//   EMP_SETUP_DATAPOD_D(ClassName, BassClassName, var1, var2, ...)
//
//     -or-
//
//   EMP_SETUP_DATAPOD_D2(ClassName, BassClass1Name, BaseClass2Name, var1, var2, ...)
//
//  ...depending on how many base classes it was derived from (currently max 2).
//
//  Note also that this macro must either go in the public section of the target class
//  definition, or the target class must be made a friend to the emp::serialize::DataPod
//  class.
//
//
//  Development Notes:
//  * To deal with pointers we should recurse, but keep map to new pointer locations.
//  * Add a mechanism to set value to constant rather than previous value.
//

#include <iostream>

#include "serialize_macros.h"

namespace emp {
namespace serialize {

  class DataPod {
  protected:
    std::ostream * os;
    std::istream * is;
    bool own_os;
    bool own_is;

    void ClearData() { 
      if (own_os && os) delete os;
      if (own_is && is) delete is;
    }
  public:
    DataPod(std::ostream & _os, std::istream & _is)
      : os(&_os), is(&_is), own_os(false), own_is(false) { ; }
    DataPod(std::iostream & _ios) : DataPod(_ios, _ios) { ; }

    // Allow move transfer of a DataPod.
    DataPod(DataPod && rhs) : os(rhs.os), is(rhs.is), own_os(rhs.own_os), own_is(rhs.own_is) {
      rhs.own_os = false; rhs.own_is=false;
    }
    DataPod & operator=(DataPod && rhs) {
      ClearData();
      os = rhs.os;  is = rhs.is;  own_os = rhs.own_os;  own_is = rhs.own_is;
      rhs.own_os = false; rhs.own_is=false;
      return *this;
    }

    // Make sure these are never accidentally created or copied.
    DataPod() = delete;
    DataPod(const DataPod &) = delete;

    ~DataPod() { ClearData(); }

    std::ostream & OStream() { return *os; }
    std::istream & IStream() { return *is; }
  };
  
  // Custom classes should run their build-in EMP_Store member function.
  template <typename T>
  auto StoreVar(DataPod & pod, T & var, bool) -> typename T::emp_load_return_type & {
    var.EMP_Store(pod);
    return pod;
  }

  // By default, just try sending the saved object to the DataPod's output stream.
  template <typename T>
  void StoreVar(DataPod & pod, T & var, int) {
    // @CAO for now use ':', but more generally we need to ensure uniquness.
    pod.OStream() << var << ':';
    emp_assert(pod.OStream());
  }
  

  // The SetupLoad() function determines what type of information a constructor needs from
  // a DataPod (based on the objects types) and returns that information.  By default, the
  // function will produce an instance of the needed type to trigger the copy constructor,
  // but if a constructor exists that takes a DataPod it will use that instead.

  // Use SFINAE technique to identify custom types.
  template <typename T>
  auto SetupLoad(DataPod & pod, T*, bool) -> typename T::emp_load_return_type & {
    return pod;
  }

  // Otherwise use default streams.
  template <typename T>
  auto SetupLoad(DataPod & pod, T*, int) -> T {
    T var;
    pod.IStream() >> var;
    pod.IStream().ignore(1);  // Ignore ':'
    emp_assert(pod.IStream() && "Make sure the DataPod is still okay.");
    return var;
  }

  // Use special load for strings.
  std::string SetupLoad(DataPod & pod, std::string *, bool) {
    std::string var;
    std::getline(pod.IStream(), var, ':');
    emp_assert(pod.IStream() && "Make sure the DataPod is still okay.");
    return var;
  }
  


  // Specialized initializers...

  namespace internal {

    // Base implementaion to specialize on.
    template <typename... IGNORE> struct serial_impl;  
  
    // Specialized to recurse, storing or loading individual vars.
    template <typename FIRST_TYPE, typename... OTHER_TYPES>
    struct serial_impl<FIRST_TYPE, OTHER_TYPES...> {
      static void Store(DataPod & pod, FIRST_TYPE & arg1, OTHER_TYPES&... others) {
        StoreVar(pod, arg1, true);
        serial_impl<OTHER_TYPES...>::Store(pod, others...);
      }
    };
    
    // End condition for recursion when no vars remaining.
    template <> struct serial_impl<> {
      static void Store(DataPod &) { ; }
      static void Load(DataPod &) { ; }
    };
  };
  
  template <typename... ARG_TYPES>
  void Store(DataPod & pod, ARG_TYPES&... args) {
    internal::serial_impl<ARG_TYPES...>::Store(pod, args...);
  }
  
  template <typename... ARG_TYPES>
  void Load(DataPod & pod, ARG_TYPES&... args) {
    internal::serial_impl<ARG_TYPES...>::Load(pod, args...);
  }
  
};
};

#endif
