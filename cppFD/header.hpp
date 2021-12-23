//standard lib folder, cout, cin, getline
#include <iostream>
#include <vector>
#include <fstream>
// formating output
#include <iomanip>
#include <time.h> 
#include <stdio.h>
#include <string>
#include <sstream>
#include <cmath>

// Random definition to avoid errors
// using namespace std;
using varType = int;
using retVarType = int;
using paramsVarType = int;
const int length_of_array = 7;

// Functions
// Declaration, should be described in header.hpp

// describe the function (purposes, inputs, outputs)
retVarType functionName(paramsVarType params1);
// Set variables we don't want to change as const
retVarType functionName(const paramsVarType input);
// Arrays as Function parameters
// Pass as a pointer
retVarType functionName(paramsVarType *arrayName);
// Pass as a sized array
retVarType functionName(paramsVarType arrayName[length_of_array]);
// Pass as an unsized array
retVarType functionName(paramsVarType arrayName[]);
