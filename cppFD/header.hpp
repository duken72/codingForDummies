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
#include <memory>
#include <set>
#include <unordered_set>
#include <unordered_map>

// Random definition to avoid errors
// using namespace std;
using varType = int;
using retVarType = int;
using paramsVarType = int;
const int LENGTH_OF_ARRAY = 7;

// Functions
// Declaration and description

// describe the function (purposes, inputs, outputs)
retVarType functionName(paramsVarType params1);
// Set variables we don't want to change as const
retVarType functionName(const paramsVarType input);
// Arrays as Function parameters
// Pass as a pointer
retVarType functionName(paramsVarType *arrayName);
// Pass as a sized array
retVarType functionName(paramsVarType arrayName[LENGTH_OF_ARRAY]);
// Pass as an unsized array
retVarType functionName(paramsVarType arrayName[]);
