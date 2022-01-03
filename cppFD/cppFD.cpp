/*This comment helps shit.
have fun checking it.*/

//look at standard library folder
#include <iostream> 
//looks in current directory first
#include "header.hpp" 

using variableType = int;

int main()
{
    // Basic IO
    std::cout << "Hello World" << std::endl;
    std::cerr << "error message" << std::endl;
    // sizeof
    std::cout << "size of int is " << sizeof(int) << "\n"; //4
    std::cout << "size of float is " << sizeof(float) << "\n"; //4
    std::cout << "size of double is " << sizeof(double) << "\n"; //8    
    // Constant
    const int shit = 72;
    // can't change const variable
    shit = 27;
    // Enumerated Constant
    enum ANIMALS {Dog, Duck, Dick}; //ANIMALS has 3 possible values
    ANIMALS duke; //duke as a variable of type ANIMALS
    duke = Duck;
    std::cout << duke << std::endl; //1
    
    // Variable assignment and initialization
    // no initializer
    int a;
    // copy initialization, inherited from the C language
    int b = 5;
    // Direct initialization
    int c( 6 );
    // Uniform initialization or list initialization
    int d { 7 };

    // Formating output
    // #include <iomanip>
    std::cout << "ahoy" << std::setw(14) << "fuck" << "/t" << "you" << std::endl;
    
    // File IO
    // #include <fstream>
    // output stream
    std::string line;
    std::ofstream myfileI ("input.txt", std::ios::app);
    if (myfileI.is_open())
    {
        myfileI << "\nI am adding a line.\n";
        myfileI << "I am adding another line.\n";
        myfileI.close();
    }
    else std::cout << "Unable to open file for writing";
    // input stream
    // though it's already opened from above
    std::ifstream myfileO ("input.txt"); 
    if (myfileO.is_open())
    {
        while(getline (myfileO,line))
        {
            std::cout << line << '\n';
        }
        myfileO.close();
    }
    else std::cout << "Unable to open file for reading";
    // User inputs
    int var;
    std::cin >> var;
    // std::getline(std::cin, var);
    // endl = "\n" + flush output;
    
    // Prefix and Postfix
    int x, y;
    y = x++; // y = x, x=x+1
    y = ++x; // x=x+1, y=x
    
    // Logic Operators
    // True = 1, False = 0
    int D = 1, U = 1, K = 0, E = 0;
    std::string TorF[] = {"False", "True"};
    std::cout << D&&U << std::endl;
    std::cout << U||K << std::endl;
    std::cout << !E||K << std::endl;
    
    // Control flow
    // if-else if-else
    if (1==2)
    {
        //statements to execute
    }
    else if (1!=2)
    {
        //statements to execute
    }
    else
    {
        //statements to execute
    }
    // switch statements
    switch(var)
    {
        //if break, exit the switch loop
        case(1): commands; break;
        // if no break, continue with next case's commands
        case(2): commands;
        //if none of the above cases
        default: commands;
    }
    // for loops
    // #include cstddef, size_t i =0
    for(int i=0; i<72; i++)
    {}
    
    while(1==1)
    {}
    
    do
    {}while(1==1);
        
    // Infinite loop
    for( ; ;)
    {}
    while(1)
    {}
    // break; to exit the loop immediately
    // continue; to exit that iteration, ignore lower commands in the loop, and start with the next iteration
     
    // Pointer
    // Check https://www.cplusplus.com/doc/tutorial/pointers/
    int val = 72;
    // Declare a pointer, that point to a int value
    int * pointerToVal; 
    // pointerToVal is the pointer of val, DEREFERENCING
    pointerToVal = &val;
    // above line inits the pointer with the address
    // would be wrong if init value pointed to *pointerToVal = &val;
    std::cout << "address of d is at &d = " << &val << std::endl;
    std::cout << "value at address &d is d =" << * pointerToVal << std::endl;
    // for char (void *) &givenChar
    /*Note:
    To declare two pointers
    use: int * p1, * p2;
    not: int * p1, p2;
    */
    
    // Array
    // type arrayName [] = {variables to be stored in the array};
    int searchKey[2] = {1, 2};
    // type arrayName[sizeDim1][sizeDim2]...[sizeDimN]
    int searchKey[2][3] = {1, 2, 3, 4, 5, 6};   
    // to access specifix index
    variableType arrayName[val][val];
    
    // vectors and iterators
    std::vector<int> vectorInts;
    std::vector<int>::iterator it;
    vectorInts.size();
    vectorInts.resize(val);
    //overwrite from the beginning of vector
    vectorInts.assign(2,7);
    //add one value at the end
    vectorInts.push_back(2);
    //set value at specific location
    vectorInts.insert(vectorInts.begin()+it,val);
    //set value at specific location
    vectorInts.emplace(vectorInts.begin()+it,val);
    //clear vector to 0 elements
    vectorInts.clear();
    vectorInts.erase(vectorInts.begin()+loc);
    vectorInts.erase(vectorInts.begin()+loc1, vectorInts.begin()+loc2);
    vectorInts.pop_back(); //remove last element
    
    for (it = vectorInts.begin(); it != vectorInts.end(); ++it)
        std::cout<<*it<<" ";
    
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

//tell the compiler that we are using a template, when delaring
template <typename T>
// then defining func:
T functionName1(T input1, T input2)
{
    return input1 + input2;
}
// template <typename T, typename U>
