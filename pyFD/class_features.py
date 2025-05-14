# Some advance features of Python class

"""
Permission

public: can run from anywhere
static: doesn't need object to run
void: doesn't return anything
"""

# ------- #

"""
Class v/s Instance Variables

Class variable with class namespace
Instance variable with instance namespace
"""


class Car:
    # Class variable
    wheels = 4

    def __init__(self) -> None:
        self.mil = 10
        self.brand = "BMW"


Car.wheels = 5
car_1 = Car()
car_1.mil = 8

# ------- #

"""
Class v/s Instance methods

Instance methods:
- pass self
- use instance variables

Class methods:
- specify with @classmethod
- pass cls
- use class variables

Static methods
- specify with @staticmethod
- do not pass self, cls
"""


# Methods
class Student:
    school = "HUST"

    def __init__(self, m1, m2) -> None:
        self.m1 = m1
        self.m2 = m2

    def avg(self):
        return (self.m1 + self.m2) / 2

    @classmethod
    def getSchool(cls):
        return cls.school

    @staticmethod
    def info():
        print("Student is dumb.")

    # Accessor: fetch, get
    def get_m1(self):
        return self.m1

    # Mutator: change, set
    def set_m1(self, value):
        self.m1 = value

    """
    @property
    def sum(self):
        return [self.m1 + self.m2]

    @sum.setter
    """


s1 = Student(1, 5)
print(s1.avg())
print(Student.getSchool())
print(Student.info())
