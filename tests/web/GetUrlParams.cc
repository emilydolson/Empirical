//  This file is part of Empirical, https://github.com/devosoft/Empirical
//  Copyright (C) Michigan State University, 2015-2019.
//  Released under the MIT Software license; see doc/LICENSE
#include <functional>

#include "../../tests2/unit_tests.h"
#include "base/assert.h"
#include "web/init.h"
#include "web/JSWrap.h"

#include "base/assert.h"
#include "web/UrlParams.h"
#include "config/ArgManager.h"

int main() {

  emp::Initialize();

  EM_ASM({
    global.location = Object();
    global.location.search = (
      "test1=val1" +
      "&test4" +
      "&test2=val1" +
      "&test3=1+23" +
      "&test1=val2+val3" +
      "&_positional=p1+p2" +
      "&_positional=p3" +
      "&bad+bad=gone"
    );
  });

  emp::ArgManager am(emp::web::GetUrlParams());


  emp_assert(*am.UseArg("test1") == emp::vector<std::string>({"val1"}));
  emp_assert(*am.UseArg("test1") ==  emp::vector<std::string>({"val2","val3"}));
  emp_assert(!am.UseArg("test1"));

  emp_assert(*am.UseArg("test2") == emp::vector<std::string>({"val1"}));
  emp_assert(!am.UseArg("test2"));

  emp_assert(am.UseArg("test3") == emp::vector<std::string>({"1","23"}));
  emp_assert(!am.UseArg("test3"));

  emp_assert(*am.UseArg("test4") == emp::vector<std::string>());
  emp_assert(!am.UseArg("test4"));

  emp_assert(
    *am.UseArg("_positional") == emp::vector<std::string>({"p1","p2","p3"})
  );
  emp_assert(!am.UseArg("_positional"));

  emp_assert(
    *am.UseArg("_illegal") == emp::vector<std::string>({"bad","bad","gone"})
  );
  emp_assert(!am.UseArg("_illegal"));

  std::cout << "Success!" << std::endl;

}
