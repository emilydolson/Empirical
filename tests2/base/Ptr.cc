#include "../unit_tests.h"

#include "base/Ptr.h"

#include <sstream>
#include <iostream>
#include <functional>

void emp_test_main()
{
  //TEST_CASE("Test Ptr", "[base]")
  emp::SetPtrDebug();
  EMP_REQUIRE(emp::GetPtrDebug());

  int arr[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  emp::PtrInfo ptrInfo(arr, 10);
  EMP_REQUIRE(ptrInfo.IsArray());
  EMP_REQUIRE(ptrInfo.IsActive());
  EMP_REQUIRE(ptrInfo.OK());

  int arr1[5] = {-4, -3, -2, -1, 0};
  auto & tracker = emp::PtrTracker::Get();
  tracker.NewArray(arr1, 5 * sizeof(int));
  EMP_REQUIRE(tracker.HasPtr(arr1));
  size_t arr1ID = tracker.GetCurID(arr1);
  EMP_REQUIRE(tracker.IsActiveID(arr1ID));

  emp::Ptr<int> arr1ptr(arr1, 5, false);
  EMP_REQUIRE(!arr1ptr.IsNull());

  // attempts to delete const array fails, error: "discards qualifiers"

  arr1ptr.NewArray(10);
  //EMP_REQUIRE(arr1ptr.DebugGetArrayBytes() == (10 * sizeof(int)));

#ifdef EMP_TRACK_MEM
  size_t arr1ptrID = tracker.GetCurID(arr1ptr.Raw());
  EMP_REQUIRE(tracker.IsActiveID(arr1ptrID));
#endif
  arr1ptr.DeleteArray();
#ifdef EMP_TRACK_MEM
  EMP_REQUIRE(!tracker.IsActiveID(arr1ptrID));
#endif

  tracker.MarkDeleted(arr1ID);
  EMP_REQUIRE(!tracker.IsActiveID(arr1ID));

  int num = 123;
  int* num_ptr = &num;
  emp::Ptr<int> numPtr(num_ptr);
  emp::Ptr<int> numPtr2(num_ptr);
  EMP_REQUIRE( numPtr.operator==(num_ptr) );
  EMP_REQUIRE( numPtr.operator>=(num_ptr) );
  EMP_REQUIRE( numPtr.operator<=(num_ptr) );
  EMP_REQUIRE( !(numPtr.operator!=(numPtr2)) );
  EMP_REQUIRE( numPtr.operator>=(numPtr2) );
  EMP_REQUIRE( numPtr.operator<=(numPtr2) );


  //TEST_CASE("Another Test Ptr", "[base]")

  // Test default constructor.
  emp::Ptr<int> ptr1;
  ptr1.New();
  *ptr1 = 5;
  EMP_TEST_VALUE(*ptr1, 5);
  ptr1.Delete();

  // Test pointer constructor
  int * temp_int = new int;
  emp::Ptr<int> ptr2(temp_int, true);
  *ptr2 = 10;
  EMP_TEST_VALUE(*ptr2, 10);
  ptr2.Delete();

  // Test non-pointer object constructor
  int base_val = 15;
  emp::Ptr<int> ptr3(&base_val);
  EMP_TEST_VALUE(*ptr3, 15);
  base_val = 20;                 // Make sure pointed to value changes with original variable.
  EMP_TEST_VALUE(*ptr3, 20);
  
  // Test copy-constructor.
  emp::Ptr<int> ptr4(ptr3);
  EMP_TEST_VALUE(*ptr4, 20);
  *ptr4 = 25;                    // Change this new pointer...
  EMP_TEST_VALUE(*ptr4, 25);       // ...make sure it actually changed.
  EMP_TEST_VALUE(*ptr3, 25);       // ...make sure the other pointer reflects the change.
  EMP_TEST_VALUE(base_val, 25);    // ...make sure the original variable changed.
  
  // -- Test count tracking on emp::Ptr --
  // A bit of an odd set of tests... we need to create and destory pointers to make sure
  // that all of the counts are correct, so we're going to use arrays of pointers to them.

  emp::vector<emp::Ptr<char> *> ptr_set(10);
  ptr_set[0] = new emp::Ptr<char>;
  ptr_set[0]->New((char) 42);
  for (size_t i = 1; i < 10; i++) ptr_set[i] = new emp::Ptr<char>(*(ptr_set[0]));

  // Do we have a proper count of 10?
  #ifdef EMP_TRACK_MEM
  EMP_TEST_VALUE(ptr_set[0]->DebugGetCount(), 10);
  ptr_set[1]->New((char) 91);
  EMP_TEST_VALUE(ptr_set[0]->DebugGetCount(), 9);
  *(ptr_set[2]) = *(ptr_set[1]);
  EMP_TEST_VALUE(ptr_set[0]->DebugGetCount(), 8);
  EMP_TEST_VALUE(ptr_set[1]->DebugGetCount(), 2);

  ptr_set[3]->Delete();
  ptr_set[1]->Delete();
  #endif

  // Make sure that we are properly handling temporary pointers moved to uninitialized pointes.
  // (Previously breaking, now fixed.)
  int a = 9;
  emp::Ptr<int> ptr_a;
  ptr_a = emp::ToPtr(&a);
  int a_val = *(ptr_a);
  EMP_TEST_VALUE(a_val, 9);

  // Test casting to unsigned char
  emp::Ptr<uint32_t> ptr5;
  ptr5.New();
  *ptr5 = 1+1024;

  EMP_TEST_VALUE(*ptr5.Cast<unsigned char>(), 1);
  ptr5.Delete();

  // Test casting to const unsigned char
  emp::Ptr<uint32_t> ptr6;
  ptr6.New();
  *ptr6 = 6+1024;

  EMP_TEST_VALUE(*ptr6.Cast<const unsigned char>(), 6);
  ptr6.Delete();

  // Test casting to const unsigned char
  emp::Ptr<uint32_t> ptr7;
  ptr7.New();
  *ptr7 = 6+1024;
  const emp::Ptr<const unsigned char> ptr8 = ptr7.Cast<const unsigned char>();
  EMP_TEST_VALUE(*ptr8, 6);
  ptr7.Delete();

  // std::cout << ptr_set[0]->DebugGetCount() << std::endl;

//   // @CAO Make sure we don't delete below 0
//   // @CAO Make sure we don't delete below 1 if we own it
//   // @CAO Make sure we only delete if you own it
//   // @CAO Make sure not to delete twice!
//   // @CAO Make sure we don't add (as owner) a pointer we already own
//
//   // -- Do some direct tests on pointer trackers --
//
//   int * real_ptr1 = new int(1);  // Count of 2 in tracker
//   int * real_ptr2 = new int(2);  // Deleted in tracker
//   int * real_ptr3 = new int(3);  // Unknown to tracker
//   int * real_ptr4 = new int(4);  // Passively known to tracker (marked non-owner)
//   auto & tracker = emp::PtrTracker::Get();
//
//   tracker.New(real_ptr1);
//   tracker.Inc(real_ptr1);
//   tracker.Inc(real_ptr1);
//   tracker.Dec(real_ptr1);
//
//   tracker.New(real_ptr2);
//   tracker.MarkDeleted(real_ptr2);
//
//   tracker.Old(real_ptr4);
//
//   EMP_TEST_VALUE(tracker.HasPtr(real_ptr1), true);
//   EMP_TEST_VALUE(tracker.HasPtr(real_ptr2), true);
// //  EMP_TEST_VALUE(tracker.HasPtr(real_ptr3), false);  // Technically may be previous pointer re-used!
//   EMP_TEST_VALUE(tracker.HasPtr(real_ptr4), true);
//
//   EMP_TEST_VALUE(tracker.IsActive(real_ptr1), true);
//   EMP_TEST_VALUE(tracker.IsActive(real_ptr2), false);
// //  EMP_TEST_VALUE(tracker.IsActive(real_ptr3), false);
//   EMP_TEST_VALUE(tracker.IsActive(real_ptr4), true);
//
//   EMP_TEST_VALUE(tracker.IsOwner(real_ptr1), true);
//   EMP_TEST_VALUE(tracker.IsOwner(real_ptr2), true);
// //  EMP_TEST_VALUE(tracker.IsOwner(real_ptr3), false);
//   EMP_TEST_VALUE(tracker.IsOwner(real_ptr4), false);
//
//   EMP_TEST_VALUE(tracker.GetCount(real_ptr1), 2);
//   EMP_TEST_VALUE(tracker.GetCount(real_ptr2), 1);
// //  EMP_TEST_VALUE(tracker.GetCount(real_ptr3), 0);
//   EMP_TEST_VALUE(tracker.GetCount(real_ptr4), 1);


  // TEST_CASE("Replicate ptr bug", "[ptr]") {
  struct testA {
    int a = 9;
    emp::Ptr<int> GetA() {return emp::ToPtr(&a);}
  };

  struct testB {
    std::function<emp::Ptr<int>(void)> b_fun;
    emp::Ptr<int> b;

    void SetBFun(std::function<emp::Ptr<int>(void)> fun) {
      b_fun = fun;
    }

    void RunBFun() {
      b = b_fun();
    }

  };

  testA ta;
  testB tb;

  std::function<emp::Ptr<int>(void)> return_a = [&ta](){return ta.GetA();};
  tb.SetBFun(return_a);
  tb.RunBFun();
  EMP_TEST_VALUE(*(tb.b), 9);

}

//#endif // EMP_TRACK_MEM
//the folowing is calvin's notes