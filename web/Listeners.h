//  This file is part of Empirical, https://github.com/devosoft/Empirical
//  Copyright (C) Michigan State University, 2017.
//  Released under the MIT Software license; see doc/LICENSE
//
//
//  A class for tracking font event listeners for Widgets


#ifndef EMP_WEB_LISTENERS_H
#define EMP_WEB_LISTENERS_H


#ifdef EMSCRIPTEN
#include <emscripten.h>
#endif

#include "../tools/string_utils.h"

#include <map>
#include <string>

namespace emp {
namespace web {

  class Listeners {
  private:
    std::map<std::string, size_t> listeners;  // Map triggers to callback IDs

  public:
    Listeners() { ; }
    Listeners(const Listeners &) = default;
    Listeners & operator=(const Listeners &) = default;

    ~Listeners(){Clear();}

    size_t GetSize() const { return listeners.size(); }

    // Use a pre-calculated function ID.
    Listeners & Set(const std::string & name, size_t fun_id) {
      emp_assert(!Has(name));
      listeners[name] = fun_id;
      return *this;
    }

    // Calculate its own function ID with JSWrap.
    template <typename... Ts>
    Listeners & Set(const std::string & name, const std::function<void(Ts... args)> & in_fun) {
      emp_assert(!Has(name));
      listeners[name] = JSWrap(in_fun);
      return *this;
    }


    bool Has(const std::string & event_name) const {
      return listeners.find(event_name) != listeners.end();
    }

    size_t GetID(const std::string & event_name) {
      emp_assert(Has(event_name));
      return listeners[event_name];
    }

    const std::map<std::string, size_t> & GetMap() const {
      return listeners;
    }

    void Clear() {
      // @CAO: Delete functions to be called.
      for (auto l : listeners) {
          JSDelete(l.second);
      }
      listeners.clear();
    }

    void Remove(const std::string & event_name) {
      // @CAO: Delete function to be called.
      listeners.erase(event_name);
    }

    // Apply all of the listeners.
    void Apply(const std::string & widget_id) {
      // Find the current object only once.
#ifdef EMSCRIPTEN
      EM_ASM_ARGS({
          var id = Pointer_stringify($0);
          emp_i.cur_obj = $( '#' + id );
        }, widget_id.c_str());
#endif

      for (auto event_pair : listeners) {
#ifdef EMSCRIPTEN
        EM_ASM_ARGS({
          var name = Pointer_stringify($0);
          emp_i.cur_obj.on( name, function(evt) { emp.Callback($1, evt); } );
        }, event_pair.first.c_str(), event_pair.second);
#else
        std::cout << "Setting '" << widget_id << "' listener '" << event_pair.first
                  << "' to '" << event_pair.second << "'.";
#endif
      }
    }


    // Apply a SPECIFIC listener.
    static void Apply(const std::string & widget_id,
                      const std::string event_name,
                      size_t fun_id) {
#ifdef EMSCRIPTEN
        EM_ASM_ARGS({
          var id = Pointer_stringify($0);
          var name = Pointer_stringify($1);
          $( '#' + id ).on( name, function(evt) { emp.Callback($2, evt); } );
        }, widget_id.c_str(), event_name.c_str(), fun_id);
#else
        std::cout << "Setting '" << widget_id << "' listener '" << event_name
                  << "' to function id '" << fun_id << "'.";
#endif
    }


    operator bool() const { return (bool) listeners.size(); }
  };


}
}


#endif
