#include <iostream>

struct structA
{
	char a;
	int b;
	float c;
	structA(char q, int w, float r) // constructor for structA
	: a(q), b(w), c(r) // a(q) run the constructor of class char, for var a, with input q
	{
		b = 4;
	}
private:
	char d;
	int e;
};

// could also declare the constructor outside
// structA::structA() {}

struct structA varA{'d', 5, 7.2};

// If use typedef struct, then don't need to add struct when declare a new var
typedef struct structB {};
structB varB;

// Use struct for plain-old-data structures without any class-like features
