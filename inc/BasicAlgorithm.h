#include "SolverStruct.h"
#include "MathFunction.h"
#include "SystemDynamic.h"
#include "BoundaryLine.h"
#include "AdaptiveGrid.h"
#include "PrintResult.h"

/*--- Public Functions ---*/
// The main algorithm running DP ---
// SolverInput *InputPtr:		Pointer to input structure
// SpeedDynParameter *ParaPtr	Pointer to parameter structure
// SolverOutput *OutputPtr:		Pointer to output structure in which the solution is stored
// real_T X0:					The initial state
// real_T Xfmax:				Upper bound for the final state
// real_T Xfmin:				Lower bound for the final state

void MagicBox(SolverInput *InputPtr, SpeedDynParameter *ParaPtr, EnvFactor *EnvPtr, SolverOutput *OutputPtr, real_T X0,
              real_T Xfmin, real_T Xfmax);