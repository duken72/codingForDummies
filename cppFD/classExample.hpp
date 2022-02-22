#include "header.hpp"

class Student
{
  std::string name;
  int id;
  int gradDate;

public:
  void setName(std::string nameIn);
  void setId(int idIn);
  void setGradDate(int dateIn);
  std::string getName();
  int getId();
  int getGradDate();
};

// Functions that access and/or modify data values in classes are called mutators.
// need namespace classname:: to signify that the functions are part of the class definition
void Student::setName(std::string nameIn)
{
  name = nameIn;
}
void Student::setId(int idIn)
{
  id = idIn;
}
void Student::setGradDate(int gradDateIn)
{
  gradDate = gradDateIn;
}

std::string Student::getName()
{
  return name;
}
int Student::getId()
{
  return id; 
}
int Student::getGradDate()
{
  return gradDate;
}
