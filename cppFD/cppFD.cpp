/*This comment helps shit.
have fun checking it.*/

#include <iostream>       // check at standard library folder
#include "header.hpp"     // check in current directory first

using namespace std;      // not really recommended
using std::ios, std::map; // prefer this
using LL = long long;

void commands() {};       // dummy function

int main()
{
    // Basic IO
    cout << "Hello World" << endl;
    cerr << "error message" << endl;
    cout << "sizeof(int) = " << sizeof(int) << endl;       // 4
    cout << "sizeof(double) = " << sizeof(double) << endl; // 8
    // endl = "\n" + flush output
    // endl is slower than just "\n"

    // Constant
    const int SHIT = 72;

    // Enumerated Constant
    enum ANIMALS {Dog, Duck, Dick};
    ANIMALS duke;
    duke = Duck;          // 1
    cout << duke << endl; // 1

    // Formating output
    // #include <iomanip>
    cout << "ahoy" << std::setw(14) << "fuck" << "/t" << "you" << endl;
    
    // File IO
    // #include <fstream>
    string line;
    std::ofstream myfileI ("in.txt", ios::app);   // output stream
    if (myfileI.is_open()) {
        myfileI << "Adding another line.\n";
        myfileI.close();
    } else cout << "Unable to open file for writing";

    // actually it's already opened from above
    std::ifstream myfileO ("in.txt");          // input stream
    if (myfileO.is_open()) {
        while(getline (myfileO,line))
            cout << line << '\n';
        myfileO.close();
    } else cout << "Unable to open file for reading";

    // User inputs
    int var;
    cin >> var;
    // std::getline(std::cin, var);
    
    // Prefix and Postfix
    int x, y;
    y = x++; // y = x;      x = x + 1
    y = ++x; // x = x + 1;  y = x
    
    // Logic Operators: True = 1, False = 0
    int D = 1, U = 1, K = 0, E = 0;
    std::string TorF[] = {"False", "True"};
    cout << D && U << ' ' << U || K << ' ' << !E || K << '\n';
    
    // Control flow
    if (1 == 2) {                 // if - else if - else
        // statements
    } else if (1 != 2) {
        // statements
    } else {
        // statements
    }
    switch(var) {                 // switch statements
        case(1): commands; break;   // if break, exit the switch loop
        case(2): commands;          // continue with next case's commands
        default: commands;          // if none of the above cases
    }
    for(int i = 0; i < 72; i++) { // for loop
        commands;
    }
    while(1 == 1) {               // while loop
        commands;
    }
    do {                          // do-while loop
        commands;
    } while(1==1);
    // break;     exit the loop immediately
    // continue;  exit that iteration, start with the next iteration
      
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
    // NOTE: Compared to vector, array has less built-in functionalities
    // But vector is created on the heap, while array is on the stack
    // In other words, array is faster.
    int searchKey[2] = {1, 2};  // type arrayName [] = {variables};
    int searchKey[2][3];        // type arrayName[dim1][dim2]...[dimN]
    int arrayName[val][val];    // access specifix index
    
    // Vectors and Iterators
    // https://en.cppreference.com/w/cpp/container/vector
    // https://www.geeksforgeeks.org/vector-in-cpp-stl/
    vector<int> vectorInts;
    vector<int>::iterator it;
    for (it = vectorInts.begin(); it != vectorInts.end(); ++it)
        cout << *it << " ";
    for (auto i : vectorInts)
        cout << i << " ";
    // Initialize a vector
    // https://www.geeksforgeeks.org/initialize-a-vector-in-cpp-different-ways/
    vector<int> vect;
    vect.push_back(10); // emplace_back is faster, but be careful
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

    // Map (is a balanced tree)
    // #include <map>
    map<string, LL> omap;           // map<typeKey, typeValue>
    omap["Duke"] = 19951995199519951995;
    omap["Duck"] = 27272727272727272727;
    omap["Dick"] = 72727272727272727272;
    cout << omap["Dick"] << endl;   // faster search than using vector

    // Unordered map (is a hash table)
    // #include <unordered_map>
    // https://www.geeksforgeeks.org/unordered_map-in-cpp-stl/
    // https://en.cppreference.com/w/cpp/container/unordered_map
    unordered_map<string, LL> umap; // unordered_map<typeKey, typeValue>
    // NOTE: there has to be a hash function for the key
    // If not by std library, then create it (https://youtu.be/KiB0vRi2wlc)
    umap["Duke"] = 19951995199519951995;
    umap["Duck"] = 27272727272727272727;
    umap["Dick"] = 72727272727272727272;
    cout << umap["Dick"] << endl;     // faster than using map
    cout << umap.at("Dick") << endl;  // NOTE: better to access data
    for(const auto& [key, value] : umap)  // Structured binding (C++17)
        cout << key << " " << value << endl;
    for (auto x : umap)                   // Before C++17
        cout << x.first << " " << x.second << endl;
    
    // Threads and Tasks
    // #include <thread> (C++11)
    int i = 0, j = 0;
    auto lambda1 = [&](){ i += 2; };
    auto lambda2 = [&](){ j += 5; };

    return 0;
}

// Functions
// Declaration, before main, should be in header.hpp
template<class T>
T functionName(T params);
// Definition, conventionally after main
// could also just put it in header.hpp 
template<class T>
T functionName(T params)
{
    commands;
    return params;
}
// to change value of variable, reference it by its pointer
// could also return the variable, but prior method is better
void increment(int &input)
{
    input++;
}
// overloading function with different input argument types
// or number of input arguments
template<typename T>    // using a template
T functionName(T input1, T input2)
{
    return input1 + input2;
}
