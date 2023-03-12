#include "header.hpp"

// file guards to prevent re-definition error
// header guard: prevent include a file multiple times
// in a single translation unit
// could use #pragma once instead, but not for ROS
#ifndef CLASS_NAME
#define CLASS_NAME

// class ClassName (: public/private/protected ParentClassName1, public ParentClassName2)
template<class T>
class ClassName
// : classA(arg1) // run constructor of class A with arg1 before constructor of ClassName
{
    // if members are listed here, do not need keyword "private"
    T member1_;
    T member2_;

public:
    ClassName(T params)           // constructor
    : member1_(params), member4_(params) {};
    // have to set some default value for array member
    // so that the memory space is allocated for the array

    ~ClassName();                 // destructor

    void setVar(T varIn);   // convention set mutator
    int getVar();                 // convention get mutator
    
    // Using "virtual" to set the API for inherited classes
    // Define but don't declare it
    virtual T accessFunction1(T varIn);
    
    T Function2(T params) {}

    ClassName setMember1(T a)
    {
        member1_ = a;
        return *this;   // "this" pointer (https://www.geeksforgeeks.org/this-pointer-in-c/)
    }
    
    // Add "const" to specifiy not changing current object
    // stackoverflow.com/questions/751681
    void Foo() const { member1_++; }  // will lead to error

  // if listed after "public", use keyword "private"
private:
    int member3_;
    float member4_;
}; // ClassName

template<class T>
T ClassName<T>::Function2(T params)
{
    member2_ += params;
}

#endif CLASS_NAME
