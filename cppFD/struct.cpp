#include <iostream>

using namespace std;

struct Employee
{
  string name;
  int age;
  float salary;
} e1, e2, e3, e4;

/*
Right at the end of the struct definition
and before the ending semicolon (;),
the optional field object_names can be used
to directly declare objects of the structure type.
*/ 

struct Rectangle
{
  int width, height;
  Rectangle(int w, int h)
  {
    width = w;
    height = h;
  }
  void areaOfRectangle()
  {
    cout<<"Area of Rectangle is: "<<(width*height);
  }
};

int main() {

  // Declaration
  // C style structure declaration
  struct Employee e3;
  // C++ style structure declaration
  Employee e4;

  // Initializing a structure
  // Assigning values before initializing
  struct Employee e1 = {"John", 32, 4200};
  // Assigning values after initializing
  struct Employee e2;
  e2.name = "Albert";
  e2.age = 32;
  e2.salary = 4200;

  // Accessing a Structure
  cout << "Age : " << e1.age << endl;
  cout << "Salary : " << e1.salary << endl;

  return 0;
}