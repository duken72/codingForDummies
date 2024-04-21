#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdio.h>
#include <string>
#include <thread>
#include <time.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

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
