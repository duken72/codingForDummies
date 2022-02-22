// Example
// https://github.com/ANYbotics/grid_map/blob/master/grid_map_core/include/grid_map_core/TypeDefs.hpp

#include <eigen3/Eigen/Core>

#pragma once

namespace sth
{

typedef Eigen::MatrixXf Matrix; 
using LENGTH = int;
// typedef int LENGTH;
typedef struct
{
  int scruples;
  int drams;
  int grains;
} WEIGHT;

// A pointer to a function returning void and accepting double
// C style typedef to make `PFD`
typedef void (*PFD)(double);
// `using`-based equivalent of the typedef above
using PF = void (*)(double);

using vector3d_t = double[3];
// Compilation error
// typedef double[3] vector3d_t;

// Type alias with template
// https://stackoverflow.com/questions/10747810/what-is-the-difference-between-typedef-and-using-in-c11
template <typename T>
struct whatever {};
template <typename T>
struct rebind
{
  typedef whatever<T> type; // to make it possible to substitue the whatever in future.
};
rebind<int>::type variable;
template <typename U>
struct bar
{
  typename rebind<U>::type _var_member;
};
// More simple with `using`
// template <typename T>
// using my_type = whatever<T>;
// my_type<int> variable;
// template <typename U>
// struct baz { my_type<U> _var_member; }

} // namespace sth


// Somewhere else
namespace sth
{
void func_test()
{
  LENGTH length, width, height;
  WEIGHT chicken;
};
} // namespace sth
