#include "header.hpp"

//file guards to prevent redefinition error
#ifndef CLASS_NAME
#define CLASS_NAME

// Random definition to avoid errors
using varType = int;

// template<class T>
// class ClassName (: public/private/protected ParentClassName1, public ParentClassName2)
class ClassName
// : classA(arg1) // run constructor of class A with arg1 before constructor of ClassName
{
  // if members are listed here, do not need keyword "private"
  varType member1_;
  varType member2_;

public:
  // declaring the constructor
  ClassName(varType params);
  // declaring the destructor
  ~ClassName();
  // convention for mutators: set and get
  void setVar(varType varIn);
  int getVar();
  // add "virtual" if inherited classes have functions with same names
  (virtual) varType accessFunction1(parameters);
  // defining a function inside here
  // then don't need to declare it
  varType Function2(varType params)
  {
    //sth here
  }

  // "this" pointer
  // Check https://www.geeksforgeeks.org/this-pointer-in-c/
  ClassName setMember1(varType a)
  {
    member1_ = a;
    return *this;
  }
  
  // "this" pointer will become a pointer to "const" object
  // you cannot change any member data, unless it is "mutable"
  // check stackoverflow.com/questions/751681
  void Foo() const
  {
    // Trying to change member value will lead to error
    member1_++;
  }

// if listed after "public", use keyword "private"
private:
  int member3_;
  float member4_;
}; // ClassName

// Constructor
// Usually used to set initial values
ClassName::ClassName(varType params)
{
  member1_ = params;
  member4_ = params;
}

// Destructor
// can't return a value, accept parameters
ClassName::~ClassName()
{
  //tasks to be completed before going out of scope
}

// for array member, have to set some default value in the constructor
// so that the memory space is allocated for the array

// template<class T>
// ClassName<T>::ClassName(varType params)

template<class T>
returnVariable ClassName<T>:: accessFunction1(paramsVarType parameters)
{
    function statements;
}

#endif CLASS_NAME
// use #pragma once instead of infdef-define-endif
// but not for ROS
