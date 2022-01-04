# C++ for dummies

TODO - Add TOC

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
| Parameters/Locals     | duke_the_duck                    |
| Constants             | DUKE_THE_DUCK                    |
| Namespace             | duke_the_duck                    |
| Function name         | dukeTheDuck                      |
| Class name            | DukeTheDuck                      |
| mem variables      | duke_the_duck_                   |
| Enums and its mems | DukeTheDuck                      |

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

### TODO

- vector emplace_back
- buffer
- mutex
- set
- override
- inline
- explicit
- std sharedptr, uniqueptr, weakptr <-> vector
- is_transparent [src](https://www.fluentcpp.com/2017/06/09/search-set-another-type-key/)
