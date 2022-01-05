# C++ for dummies

## Table of Contents

- [Courses](#courses)
- [Compiling](#compiling)
- [Style Guide](#style-guide)
  - [Naming Conventions](#naming-conventions)
  - [Header Files](#header-files)
  - [Names and Order of Includes](#names-and-order-of-includes)
  - [Namespaces](#namespaces)
- [Best practice](#best-practice)
  - [Variable assignment and initialization](#variable-assignment-and-initialization)
  - [Constructor member initializer lists](#constructor-member-initializer-lists)
  - [Pointer](#pointer)
  - [Structure v/s Class](#structure-vs-class)
  - [POD - Plain Old Data](#pod---plain-old-data)
  - [Inline Specifier (since C++17)](#inline-specifier-since-c17)
  - [Explicit Specifier](#explicit-specifier)
  - [Smart Pointers](#smart-pointers)
  - [Virtual Specifier & Override Identifier](#virtual-specifier--override-identifier)
  - [Keyword auto](#keyword-auto)
  - [TODO](#todo)

## Courses

- [Systems Programming in C++, TUM](https://db.in.tum.de/teaching/ss21/c++praktikum/?lang=en)
- [Modern C++ (2020, Uni Bonn)](https://www.ipb.uni-bonn.de/cpp-course-2020/)
- [Modern C++ (Summer 2018, Uni Bonn)](https://youtube.com/playlist?list=PLgnQpQtFTOGR50iIOtO36nK6aNPtVq98C)

## Compiling

For simple test:

```bash
g++ main.cpp -o main.out
./main.out input.txt
```

In reality, use build system, such as [CMake](../cmakeFD/README.md):

TODO - Compiler path vs Include path in VSCode

RAII - Resource Acquisition is Initialization

-------

## Style Guide

- [Google Style Guide](https://google.github.io/styleguide/cppguide.html)
- [Modern C++ Coding Guidelines](https://github.com/Microsoft/AirSim/blob/master/docs/coding_guidelines.md)

### Naming Conventions

| Code Element          | Style                            |
|-----------------------|----------------------------------|
| File names            | Match case of class name in file |
| Parameters/Locals     | local_param_duck                 |
| Constants             | CONSTANT_DUCK                    |
| Namespace             | namespace_duck                   |
| Function name         | funcNameDuck                     |
| Class name            | ClassNameDuck                    |
| mem variables         | mem_var_duck_                    |
| Enums and its mems    | EnumsAndMem                      |

### Header Files

```cpp
#ifndef FOO_BAR_BAZ_H_
#define FOO_BAR_BAZ_H_
code...
#endif // FOO_BAR_BAZ_H_
```

```#pragma once``` is not supported if same header files exist at multiple places (ROS build system).

### Names and Order of Includes

- Related header
- C system headers
- C++ standard lib headers
- Other libraries's headers
- Your project headers
Example:

```cpp
#include "foo/server/fooserver.h"

#include <sys/types.h>
#include <unistd.h>

#include <string>
#include <vector>

#include "base/basictypes.h"
#include "base/commandlineflags.h"
#include "foo/server/bar.h"
```

### Namespaces

```cpp
namespace outer {
inline namespace inner {
  void foo();
}  // namespace inner
}  // namespace outer
```

-------

## Best practice

### Variable assignment and initialization

Source [Learn C++](https://www.learncpp.com/cpp-tutorial/variable-assignment-and-initialization/).

```cpp
// no initializer
int a;
// copy initialization
int b = 5;
// Direct initialization
int c( 6 );
// Uniform initialization / list initialization
int d { 7 };
```

- Copy initialization: was inherited from the C language.
- Direct initialization: For simple data types (like `int`), copy and direct initialization are essentially the same. For more complicated types, direct initialization tends to be more efficient than copy initialization.
- Uniform initialization / list initialization: Unfortunately, direct initialization can’t be used for all types of initialization (such as initializing an object with a list of data). To provide a more consistent initialization mechanism.

**In practice**:

- Favor initialization using braces whenever possible.
- But don't over-dramatic simple things: `int d{7};`

-------

### Constructor member initializer lists

Source: [Learn C++](https://www.learncpp.com/cpp-tutorial/constructor-member-initializer-lists/).

Instead of normal constructor initialization:

```cpp
Something::Something(int memIn1, double memIn2)
{
  mem1 = memIn1;
  mem2 = memIn2;
  mem3 = 'c';
};
```

Initializing member values with initializer lists is better:

```cpp
Something::Something(int memIn1, double memIn2)
: mem1{memIn1}, mem2{memIn2}, mem3{'c'} 
{
};
```

- The initializer list is inserted after the constructor parameters.
- It begins with a colon (:), separated by a comma, and does **NOT** end in a semicolon.
- Replaced by this initializer list, the assignments in the constructor body is no longer needed.

There are cases that initialization of data members inside constructor doesn’t work and Initializer List **MUST** be used (src [geeksforgeeks](https://www.geeksforgeeks.org/when-do-we-use-initializer-list-in-c/), [stackoverflow](https://stackoverflow.com/questions/926752/why-should-i-prefer-to-use-member-initialization-lists)):

- Non-static const data members
- Reference members
- Performance reasons
- Avoid unnecessary call to a default constructor
- etc.

```cpp
class Test {
  // non-static const data members
  const int t1;
  // reference members
  int &t2;

public:
  Test(int t1, int &t2)
  :t1(t1), t2(t2)
  {}  //Initializer list must be used
};
```

-------

### Pointer

| Expression | What is evaluated                     | Equivalent |
|:----------:|---------------------------------------|:----------:|
|     a.b    | mem b of object a                     |            |
|    a->b    | mem b of object pointed to by a       |   (*a).b   |
|    *a.b    | Value pointed to by mem b of object a |   *(a.b)   |

The points of pointer:

- The overhead memory cost  
  When assigning variables with values will copy that values to different memory space. Eg, `int a = 1, b = 1`, there are two memory space with value 1, type `int`. It could have just taken only one memory space for that value. For simply type like int, float, it wouldn't change much. But for instance/object of complex structure, class, it would reduce the memory cost significantly.

- Change multiple variables  
  In Python, it's easy to return variables of different types

  ```python
  def func_ab(c, d):
    return a, b
  ```

  For CPP, a function returns only 1 variable of 1 type. To change multiple variables, we use their pointers as input

  ```cpp
  void func_ab(&a, &b) {}
  ```

-------

### Structure v/s Class

Structure:

- Default access specifier will be `public`.
- The instance of the structure is known as "Structure variable".
- A `struct` is a **bundle**. A `struct` is several related elements that needed to be tied up together in a certain context.

Class:

- Default access specifier will be `private`.
- The instance of the class is known as "Object of the class".
- A `class` can **do things**.

-------

### POD - Plain Old Data

Src: [stackoverflow](https://stackoverflow.com/questions/146452/what-are-pod-types-in-c).

-------

### Inline Specifier (since C++17)

Src: [geeksforgeeks](https://www.geeksforgeeks.org/inline-functions-cpp/), [stackoverflow](https://stackoverflow.com/questions/1759300/when-should-i-write-the-keyword-inline-for-a-function-method).

This concerns with the overhead cost for switching time of small functions. When the inline function is called, the whole code of the inline function gets inserted or substituted. This substitution is performed by the C++ compiler at compile time.

Inlining is only a request to the compiler, not a command. There are cases that the compiler can ignore this request. Inline functions have advantages but also disadvantages (check the sources).

```cpp
inline int cube(int s)
{
  return s*s*s;
}

int main()
{
  std::cout << "The cube of 3 is: " << cube(3) << "\n";
  return 0;
}
```

For Class definition, you only need to add inline when defining it, not when declare it inside the class

```cpp
class S
{
public:
  // redundant to add inline here
  int square(int s); // declare the function
};
  
inline int S::square(int s) {} // use inline prefix
```

-------

### Explicit Specifier

Specifies that a constructor or conversion function (since C++11) or deduction guide (since C++17) is explicit, that is, it cannot be used for implicit conversions and copy-initialization.

The reason you might want to do this is to avoid accidental construction that can hide bugs. See example [implicit conversion](implicitConversion.cpp) and [explicit specifier](explicitSpecifier.cpp).

Src: [cppreference](https://en.cppreference.com/w/cpp/language/explicit), [stackoverflow](https://stackoverflow.com/questions/121162/what-does-the-explicit-keyword-mean), [ibm](https://www.ibm.com/docs/en/i/7.4?topic=only-explicit-conversion-constructors-c).

TODO - Should we almost always add `explicit` specifier.

-------

### Smart Pointers

Sources: [geeksforgeeks1](https://www.geeksforgeeks.org/smart-pointers-cpp/), [geeksforgeeks2](https://www.geeksforgeeks.org/auto_ptr-unique_ptr-shared_ptr-weak_ptr-2/).

As we’ve known, unconscious not deallocating a pointer causes a memory leak that may lead to crash of the program. For languages with **Garbage Collection Mechanisms** to smartly deallocate unused memory, like Java and C#, programmers don't have to worry about any memory leak. C++11 comes up with its own mechanism: Smart Pointer. When the object is destroyed, it frees the memory as well.

With `#include <memory>`:

- `std::unique_ptr` stores one pointer only. We can assign a different object by removing the current object from the pointer.
- `std::shared_ptr` allows more than one pointer pointing to this one object at a time and it’ll maintain a Reference Counter using `use_count()` method.
- `std::weak_ptr` also allows more than one pointer pointing at one object at a time, but without a Reference Counter.
- `std::auto_ptr` is replaced by `std::unique_ptr`, with similar functionality, improved security, added features and support for arrays.

Good practice:

- Use `make_shared` as a simple and more efficient way to create an object and a `shared_ptr` to manage shared access to the object at the same time. ([src](https://docs.microsoft.com/en-us/cpp/standard-library/memory-functions?view=msvc-170#make_shared))

```cpp
#include <memory>

class Example
{
  Example(argument)
  {};
};
std::shared_ptr<Example> msp = std::make_shared<Example>(argument);
std::unique_ptr<Example> mup = std::make_unique<Example>(argument);
```

Example: [smartPointer.cpp](smartPointer.cpp).

```bash
g++ smartPointer.cpp -o test && ./test && rm -f test
```

-------

### Virtual Specifier & Override Identifier

These two concerns with **Runtime polymorphism**.

Without "virtual" you get "early binding". With "virtual" you get "late binding". Decent examples explain it all: [stackoverflow](https://stackoverflow.com/questions/2391679/why-do-we-need-virtual-functions-in-c).

```cpp
class Base
{
public:
          void Method1() { std::cout << "Base::Method1" << std::endl; }
  virtual void Method2() { std::cout << "Base::Method2" << std::endl; }
};

class Derived : public Base
{
public:
  void Method1() { std::cout << "Derived::Method1" << std::endl; }
  void Method2() { std::cout << "Derived::Method2" << std::endl; }
};

Base* basePtr = new Derived ();
  //  Note - constructed as Derived, but pointer stored as Base*
basePtr->Method1 ();  //  Prints "Base::Method1"
basePtr->Method2 ();  //  Prints "Derived::Method2"
```

When using `virtual` functions, it is possible to make mistakes while declaring the member functions of the derived classes. Using the `override` identifier prompts the compiler to display error messages when these mistakes are made.

```cpp
class Base
{
public:
  virtual void Method2() { std::cout << "Base::Method2" << std::endl; }
};

class Derived : public Base
{
public:
  // override identifier will give Error for miss-typing Metod2
  void Metod2() { std::cout << "Derived::Method2" << std::endl; } override
};
```

-------

### Keyword auto

Use `auto` for cases to increase readability without creating confusion.

```cpp
//good : auto increases readability here
for(auto it = std::begin(v); it != std::end(v); ++it)//v could be array as well
{
  //..
}

// No type confusion
auto obj1 = new SomeType<OtherType>::SomeOtherType();
auto obj2 = std::make_shared<XyzType>(args...);
```

-------

### new and delete operator

asd

-------

### TODO

- set
- typedef
- using
- buffer
- mutex
- is_transparent [src](https://www.fluentcpp.com/2017/06/09/search-set-another-type-key/)
