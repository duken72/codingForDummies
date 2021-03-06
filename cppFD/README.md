<!-- omit in toc -->
# C++ for dummies

[![tokei](https://tokei.rs/b1/github/duken72/codingForDummies/cppFD)](https://github.com/duken72/codingForDummies/cppFD)

<!-- omit in toc -->
## Table of Contents

- [Courses](#courses)
- [Building](#building)
  - [Compiling](#compiling)
  - [Linking](#linking)
- [Style Guide](#style-guide)
  - [Naming Conventions](#naming-conventions)
  - [Header Files](#header-files)
  - [Names and Order of Includes](#names-and-order-of-includes)
  - [Namespaces](#namespaces)
  - [Documentation](#documentation)
- [Memory Management and Pointer](#memory-management-and-pointer)
  - [Variable types and sizes](#variable-types-and-sizes)
  - [Stack and Heap Memory](#stack-and-heap-memory)
  - [`new` and `delete` operator](#new-and-delete-operator)
  - [Pointer](#pointer)
  - [Smart Pointers](#smart-pointers)
- [Advanced Class Features](#advanced-class-features)
  - [Constructor member initializer lists](#constructor-member-initializer-lists)
  - [Structure v/s Class](#structure-vs-class)
  - [`virtual` & `override` identifiers](#virtual--override-identifiers)
- [Best practices, tips and notes](#best-practices-tips-and-notes)
  - [Variable assignment and initialization](#variable-assignment-and-initialization)
  - [`inline` specifier (since C++17)](#inline-specifier-since-c17)
  - [`explicit` specifier](#explicit-specifier)
  - [Keyword `auto`](#keyword-auto)
  - [Keywords `using` & `typedef`](#keywords-using--typedef)
  - [TODO](#todo)

-------

## Courses

- [The Cherno](https://youtube.com/playlist?list=PLlrATfBNZ98dudnM48yfGUldqGD0S4FFb)
- [Systems Programming in C++, TUM](https://db.in.tum.de/teaching/ss21/c++praktikum/?lang=en)
- [Modern C++ (2020, Uni Bonn)](https://www.ipb.uni-bonn.de/cpp-course-2020/)
- [Modern C++ (Summer 2018, Uni Bonn)](https://youtube.com/playlist?list=PLgnQpQtFTOGR50iIOtO36nK6aNPtVq98C)
- [C++ For Programmers, Udacity](https://classroom.udacity.com/courses/ud210)

-------

## Building

Building CPP implies compiling and linking.

For simple test:

```bash
g++ main.cpp -o main.out
g++ main.cpp -S #this will generate assemply file main.s
./main.out input.txt
```

### Compiling

Example of compiler, object file, binary file, assembly code: [The Cherno](https://youtu.be/3tIqpEmWMLI)

In reality, use build system, such as [CMake](../cmakeFD/README.md):

RAII - Resource Acquisition is Initialization

### Linking

Source: [The Cherno](https://youtu.be/H4s55GgAg0I)

When linking, the linker will find the function definitions. This is why you can simply include header files, which only have function declarations.

Also, if you define a function in header file, then include that header file in other files, it leads to linking error: multiple definition of a function. This concerns with using `(target_)link_libraries` in [CMake](../cmakeFD/README.md).

-------

## Style Guide

- [Google Style Guide](https://google.github.io/styleguide/cppguide.html)
- [Modern C++ Coding Guidelines](https://github.com/Microsoft/AirSim/blob/master/docs/coding_guidelines.md)

The ROS2 build system has a number of linters, which will complain about coding style violations.

```bash
# automatically fix them.
ament_uncrustify --reformat
# see the output in isolation
ament_cpplint
ament_cppcheck
```

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

Use [VS-Code snippet](../VSCodeFD/global.code-snippets) for this header guards:

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

For ROS2, there is an order to include header.

- Use `#include <sth>` for standard libs and files from other packages.
- Use `#include "sth"` for header files of current package.

### Namespaces

Create and use [VS-Code Snippets](../VSCodeFD/global.code-snippets)

```cpp
namespace outer {
namespace inner {
  void foo();
}  // namespace inner
}  // namespace outer
```

### Documentation

Use Doxygen style comments, if the function is non-obvious

- /// and /** */ comments for documentation purposes, @tags only support a single line!
- // style comments for notes and general comments.
- // Avoid trivial cluttering comments, prefer meaningful variable and function names.

```cpp
/**
 * Sum numbers in a vector.
 *
 * This sum is the arithmetic sum, not some other kind of sum that only
 * mathematicians have heard of.
 *
 * @param values Container whose values are summed.
 * @return sum of `values`, or 0.0 if `values` is empty.
 */
double sumInt(std::vector<double> values);

/**
 * @brief Sum numbers in a vector
 * @param values Container whose values are summed.
 * @return sum of `values`, or 0.0 if `values` is empty.
 * @throw Exception something
 */
double sumInt(std::vector<double> values);
```

-------

## Memory Management and Pointer

### Variable types and sizes

The primitive data types vary in the memory size that they occupy. 1 byte is equal to 8 bits. Be aware that sometimes, the actual memory size also depends on the compiler, or the data models ([32 bit systems v/s 64 bit systems](https://en.cppreference.com/w/cpp/language/types))

**Modifiers**:

- Signedness
  - `signed` (default) - target type will have signed representation, loosing 1 bit
  - `unsigned` - target type will have unsigned representation
- Size
  - `short` - target type will be optimized for space and will have width of at least 16 bits.
  - `long` - target type will have width of at least 32 bits.

|          Type           |        Size         |      Typical Range       |
|:----------------------: |:------------------: |:-----------------------: |
|         boolean         |   8 bits = 1 byte   |          $0-1$           |
|      (signed) char      |   8 bits = 1 byte   |    $-2^7$ to $2^7-1$     |
|      unsigned char      |   8 bits = 1 byte   |      $0$ to $2^8-1$      |
|      (signed) int       |  32 bits = 4 bytes  | $-2^{31}$ to $2^{31}-1$  |
|      unsigned int       |  32 bits = 4 bytes  |    $0$ to $2^{32}-1$     |
|   (signed) short int    |  16 bits = 2 bytes  | $-2^{15}$ to $2^{15}-1$  |
|   unsigned short int    |  16 bits = 2 bytes  |    $0$ to $2^{16}-1$     |
|    (signed) long int    |  32 bits = 4 bytes  | $-2^{31}$ to $2^{31}-1$  |
|    unsigned long int    |  32 bits = 4 bytes  |    $0$ to $2^{32}-1$     |
| (signed) long long int  |  64 bits = 8 bytes  | $-2^{63}$ to $2^{63}-1$  |
| unsigned long long int  |  64 bits = 8 bytes  |    $0$ to $2^{64}-1$     |
|          float          |  32 bits = 4 bytes  |  $3.4E\pm38$ (7 digits)  |
|         double          |  64 bits = 8 bytes  | $1.7E\pm308$ (15 digits) |
|       long double       |  96 bits = 12bytes  |                          |

**Notes**:

- One `boolean` actually just needs 1 bit, but you can only access byte, thus it still occupies 1 byte, which is 8 bits. We could however store 8 `bool`(s) in 1 byte.
- To differentiate types when assigning variables, use suffixes. Aware that these suffixes are case-insensitive: ie., `u` is equivalent to `U`, `UL` is equivalent to `uL, Ul, ul`.

| Data Type  |   Suffix   |       Meaning       |
|:---------: |:---------: |:------------------: |
|    int     |     U      |    unsigned int     |
|    int     |     L      |        long         |
|    int     |  UL / LU   |    unsigned long    |
|    int     |     LL     |      long long      |
|    int     | ULL / LLU  | unsigned long long  |
|   double   |     F      |        float        |
|   double   |     L      |     long double     |

-------

### Stack and Heap Memory

Source: [The Cherno](https://youtu.be/wJ1L2nSIV1s)

- Allocating on the stack is fast, simple as only one CPU commands.
- There is a lot happen behind the scene when assigning a value on heap memory. To find the free space, assign, book keeping, .. In case you have something very memory heavy, then you have to use the heap.

```cpp
// Allocate with the stack
int value = 5;

// Allocate with the heap
int* hvalue = new int;
*hvalue = 5;
```

-------

### `new` and `delete` operator

Sources: [stackoverflow1](https://stackoverflow.com/questions/679571/when-to-use-new-and-when-not-to-in-c), [stackoverflow2](https://stackoverflow.com/questions/655065/when-should-i-use-the-new-keyword-in-c), [stackoverflow3](https://stackoverflow.com/questions/392455/about-constructors-destructors-and-new-delete-operators-in-c-for-custom-object).

- The `new` operator does two things: allocating memory and calling the constructor. The **MAIN PURPOSE** of `new`, is to allocate memory, on the **HEAP** specifically.
- The `delete` operator calls the destructor and then frees the memory.
- Arrays created with `new []` must be destroyed with `delete[]`.
- Using `new`, the object created remains in existence until you `delete` it. Without using `new`, the object will be destroyed when it goes out of scope.
- Every time you type `new`, type `delete`.

```cpp
ptr_var = new data_type;
int * var = new int(7);

void foo()
{
  int i = 0;
} // i is now destroyed.

for (...)
{
  int i = 0;
} // p is destroyed after each loop

void foo(int size)
{
  Point* pointArray = new Point[size];
  delete[] pointArray;
  MyClass* myClass = new MyClass();
  delete myClass;
}
```

-------

### Pointer

| Expression | What is evaluated                     | Equivalent |
|:----------:|---------------------------------------|:----------:|
|   `a.b`    | member b of object a                  |            |
|  `a->b`    | member b of object pointed to by a    |  `(*a).b`  |
|  `*a.b`    | Value pointed to by mem b of object a |  `*(a.b)`  |

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

### Smart Pointers

Sources: [geeksforgeeks1](https://www.geeksforgeeks.org/smart-pointers-cpp/), [geeksforgeeks2](https://www.geeksforgeeks.org/auto_ptr-unique_ptr-shared_ptr-weak_ptr-2/).

As we???ve known, unconscious not deallocating a pointer causes a memory leak that may lead to crash of the program. For languages with **Garbage Collection Mechanisms** to smartly deallocate unused memory, like Java and C#, programmers don't have to worry about memory leak. C++11 comes up with its own mechanism: Smart Pointer. When the object is destroyed, it frees the memory as well.

With `#include <memory>`:

- `std::unique_ptr` stores one pointer only. We can assign a different object by removing the current object from the pointer.
- `std::shared_ptr` allows more than one pointer pointing to this one object at a time and it???ll maintain a Reference Counter using `use_count()` method.
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

## Advanced Class Features

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

There are cases that initialization of data members inside constructor doesn???t work and Initializer List **MUST** be used (src [geeksforgeeks](https://www.geeksforgeeks.org/when-do-we-use-initializer-list-in-c/), [stackoverflow](https://stackoverflow.com/questions/926752/why-should-i-prefer-to-use-member-initialization-lists)):

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

### Structure v/s Class

Structure:

- Default access specifier will be `public`.
- The instance of the structure is known as "Structure variable".
- A `struct` is a **bundle**. A `struct` is several related elements that needed to be tied up together in a certain context.
- Practical: When a collection of POD ([Plain Old Data](https://stackoverflow.com/questions/146452/what-are-pod-types-in-c)) is needed.

Class:

- Default access specifier will be `private`.
- The instance of the class is known as "Object of the class".
- A `class` can **do things**.
- Operators to work on new data type can be defined using special methods (over-loading operators??)
- One class can be used as the basis for definition of another (inheritance)
- Declaration of a var of the new class type requires just the name of the class: `classA varA;`, not: `struct structA varA;`
- If you use inheritance, don't use struct, use class.

-------

### `virtual` & `override` identifiers

These two concerns with **Runtime Polymorphism**.

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

## Best practices, tips and notes

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
- Uniform initialization / list initialization: To provide a more consistent initialization mechanism. Because, direct initialization can???t be used for all types of initialization (such as initializing an object with a list of data).

**In practice**:

- Favor initialization using curly braces whenever possible.
- But don't over-dramatic simple things: `int d{7};`

-------

### `inline` specifier (since C++17)

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

### `explicit` specifier

Specifies that a constructor or conversion function (since C++11) or deduction guide (since C++17) is explicit, that is, it cannot be used for implicit conversions and copy-initialization.

The reason you might want to do this is to avoid accidental construction that can hide bugs. See example [implicit conversion](specifierImplicit.cpp) and [explicit specifier](specifierExplicit.cpp).

Src: [cppreference](https://en.cppreference.com/w/cpp/language/explicit), [stackoverflow](https://stackoverflow.com/questions/121162/what-does-the-explicit-keyword-mean), [ibm](https://www.ibm.com/docs/en/i/7.4?topic=only-explicit-conversion-constructors-c).

TODO - Should we almost always add `explicit` specifier.

-------

### Keyword `auto`

Use `auto` for cases to increase readability without creating confusion.

```cpp
// good : auto increases readability here
// v could be array as well
for(auto it = std::begin(v); it != std::end(v); ++it) {}

// No type confusion
auto obj1 = new SomeType<OtherType>::SomeOtherType();
auto obj2 = std::make_shared<XyzType>(args...);
```

-------

### Keywords `using` & `typedef`

Purposes of `using` keyword in C++: [educative](https://www.educative.io/edpresso/what-is-the-using-keyword-in-cpp), [ibm](https://www.ibm.com/docs/en/zos/2.3.0?topic=only-using-declaration-class-members-c), [learncpp](https://www.learncpp.com/cpp-tutorial/using-declarations-and-using-directives/), [stackoverflow](https://stackoverflow.com/questions/20790932/what-is-the-logic-behind-the-using-keyword-in-c)

- `using declarations`: Bring a specific member from the namespace into the current scope.
  
  ```cpp
  int main()
  {
    using std::cout; // declare cout resolve to std::cout
    cout << "Hello world!"; // no std:: prefix is needed here!
    return 0;
  } // the using declaration expires here
  ```

- `using directive`: Bring all members from the namespace into the current scope.

  ```cpp
  using namespace std;
  ```

  In modern C++, using directives generally offer little benefit (saving some typing) compared to the risk. Because using directives import all of the names from a namespace (potentially including lots of names you???ll never use), the possibility for naming collisions to occur increases significantly (especially if you import the std namespace).

- Bring a base class method or variable into the current class???s scope.

- In C++11, the `using` keyword is used for `type alias`, which is identical to `typedef`. In many cases, `using` has improving readability, compared to the equivalent `typedef`, especially with pointer and template ([TypeDefs.hpp](TypeDefs.hpp)).

-------

### TODO

- unique_ptr (yes, again please)
- lvalue & rvalue
- buffer
- mutex
- is_transparent [src](https://www.fluentcpp.com/2017/06/09/search-set-another-type-key/)
