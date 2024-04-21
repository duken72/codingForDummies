// Use struct for plain-old-data structures
// without any class-like features

struct structA {
  // public: as default
  char a;
  int b;
  float c;

  structA(char q, int w, float r)
      : a(q), b(w), c(r) // member initializer lists
  {
    b = 4;
  }

private:
  char d;
  int e;
} a1, a2, a3, a4;
/*
  Right at the end of the struct definition
  and before the ending semicolon (;),
  the optional field object_names can be used
  to directly declare objects of the structure type.
*/

typedef struct structB;

// Declaration
struct structA a5; // C style struct declaration
structB varB;      // C++ style struct declaration
