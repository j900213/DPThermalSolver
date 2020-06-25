#include "stdlib.h"
#include "stdio.h"
#include "SolverStruct.h"

/*--- Public Functions ---*/
// Print out the input settings to the screen: Inital state, grids, etc.
void printInputInfo(SolverInput *InputPtr, real_T X0, real_T X0_round, uint16_t startIdx, real_T *StateVec, real_T *ControlVec);

// Print out the solution output to the screen: Optimal state trajectory, control policy, cost.
void printSolution(SolverInput *InputPtr, real_T X0_round, SolverOutput *OutputPtr);