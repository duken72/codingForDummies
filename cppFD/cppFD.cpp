/*This comment helps shit.
have fun checking it.*/

//look at standard library folder
#include <iostream> 
//looks in current directory first
#include "header.hpp"

using variableType = int;
using std::cout, std::endl, std::cin, std::cerr;
using std::vector, std::string;
using std::set, std::unordered_set, std::unordered_map;
void commands() {};

int main()
{
  // Basic IO
  cout << "Hello World" << endl;
  cerr << "error message" << endl;
  // sizeof
  cout << "size of int is " << sizeof(int) << "\n";       //4
  cout << "size of double is " << sizeof(double) << "\n"; //8
  // Constant
  const int SHIT = 72;
  // can't change const variable, SHIT = 27;

  // Enumerated Constant
  enum ANIMALS {Dog, Duck, Dick}; //ANIMALS has 3 possible values
  ANIMALS duke; //duke as a variable of type ANIMALS
  duke = Duck;
  cout << duke << endl; //1

  // Formating output
  // #include <iomanip>
  cout << "ahoy" << std::setw(14) << "fuck" << "/t" << "you" << endl;
  
  // File IO
  // #include <fstream>
  // output stream
  string line;
  std::ofstream myfileI ("input.txt", std::ios::app);
  if (myfileI.is_open())
  {
    myfileI << "\nI am adding a line.\n";
    myfileI << "I am adding another line.\n";
    myfileI.close();
  }
  else cout << "Unable to open file for writing";
  // input stream
  // though it's already opened from above
  std::ifstream myfileO ("input.txt"); 
  if (myfileO.is_open()) {
    while(getline (myfileO,line)) {
      cout << line << '\n';
    }
    myfileO.close();
  }
  else cout << "Unable to open file for reading";

  // User inputs
  int var;
  cin >> var;
  // std::getline(std::cin, var);
  // endl = "\n" + flush output;
  
  // Prefix and Postfix
  int x, y;
  y = x++; // y = x, x=x+1
  y = ++x; // x=x+1, y=x
  
  // Logic Operators: True = 1, False = 0
  int D = 1, U = 1, K = 0, E = 0;
  std::string TorF[] = {"False", "True"};
  cout << D&&U << ' ' << U||K << ' ' << !E||K << '\n';
  
  // Control flow
  if (1==2) {     // if-else if-else
    //statements to execute
  } else if (1!=2) {
    //statements to execute
  } else {
    //statements to execute
  }
  switch(var) {                 // switch statements
    case(1): commands; break;   // if break, exit the switch loop
    case(2): commands;          // continue with next case's commands
    default: commands;          // if none of the above cases
  }
  for(int i=0; i<72; i++) {     // for loop
    commands;
  }  
  while(1==1) {                 // while loop
    commands;
  }
  do {                          // do-while loop
    commands;
  } while(1==1);
  for( ; ;) {                   // Infinite loop
    commands;
  }
  while(1) {                    // Infinite loop
    commands;
  }
  // break;     to exit the loop immediately
  // continue;  to exit that iteration, ignore lower commands in the loop, and start with the next iteration
    
  // Pointer
  // Check https://www.cplusplus.com/doc/tutorial/pointers/
  int val = 72;
  int *pointerToVal;    // Declare an int pointer
  pointerToVal = &val;  // pointerToVal is the pointer of val, DEREFERENCING
  cout << "Address of d is &d = " << &val << endl;
  cout << "Value at address &d is d =" << * pointerToVal << endl;
  int *p1, *p2;
  char c;
  (void *) &c;          // A char pointer
  
  // Array
  // type arrayName [] = {variables to be stored in the array};
  int searchKey[2] = {1, 2};
  // type arrayName[sizeDim1][sizeDim2]...[sizeDimN]
  int searchKey[2][3] = {1, 2, 3, 4, 5, 6};   
  // to access specifix index
  variableType arrayName[val][val];
  
  // Vectors and Iterators
  // https://www.geeksforgeeks.org/vector-in-cpp-stl/
  vector<int> vectorInts;
  vector<int>::iterator it;
  // Vector iterator: (c)(r)begin, (c)(r)end
  // Vector capacity: size, max_size, capacity,
  // resize, empty, shrink_to_fit, reserve
  // Vector modifier: assign, push_back, pop_back
  // insert, erase, swap, clear, emplace, emplace_back
  for (it = vectorInts.begin(); it != vectorInts.end(); ++it)
    cout << *it << " ";
  cout << endl;
  for (auto i : vectorInts)
    cout << i << " ";
  cout << endl;
  // Initialize a vector
  // https://www.geeksforgeeks.org/initialize-a-vector-in-cpp-different-ways/
  vector<int> vect;
  vect.push_back(10); //emplace_back
  int n = 3;
  vector<int> vect(n, 10);
  vector<int> vect{ 10, 20, 30 };
  vector<int> vect2(vect.begin(), vect.end());

  // Set
  // #include <set>
  // #include <unordered_set>
  // https://en.cppreference.com/w/cpp/container/set
  set<int> set_int;
  unordered_set<int> set_unord_int;
  // For descending order
  set<int, std::greater<int>> set_int;
  // set_int.begin, end, size, max_size, empty
  // set_int.insert, clear, emplace, erase
  // set_int.count, find, contains, lower_bound, etc

  // Unordered map
  // https://www.geeksforgeeks.org/unordered_map-in-cpp-stl/
  // https://en.cppreference.com/w/cpp/container/unordered_map
  // #include <unordered_map>
  unordered_map<string, int> umap;
  umap["GeeksforGeeks"] = 10;
  umap["Practice"] = 20;
  umap["Contribute"] = 30;
  for (auto x : umap)
    cout << x.first << " " << x.second << endl;
  unordered_map<string, string> u = {
        {"RED",   "#FF0000"},
        {"GREEN", "#00FF00"},
        {"BLUE",  "#0000FF"}
    };
  for( const auto& [key, value] : u )
    cout << key << " " << value << endl;
  
  return 0;
}

// Functions
// Declaration, before main, should be described in header.hpp
retVarType functionName(paramsVarType params);
// Definition, conventionally after main
// could also just put it in header.hpp 
retVarType functionName(paramsVarType params)
{
  commands;
  return params;
}
// to change value of variable, reference it by its pointer
// could also return the variable, but prior method is better
void function(int &var);
int main()
{
  int a = 7;
  increment(a);
}
void increment(int &input)
{
  input++;
}
// overloading function with different input argument types
// or number of input arguments

template<typename T>    // using a template
T functionName1(T input1, T input2)
{
  return input1 + input2;
}
