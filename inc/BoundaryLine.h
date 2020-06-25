#include "SolverStruct.h"


#ifndef BOUNDARY
#define BOUNDARY

typedef struct {
    real_T *upperBound;                         // Upper Boundary Line
    real_T *lowerBound;                         // Lower Boundary Line
    real_T (*boundMemo)[4];                     // Record the actual bounds over time
                                                // ... [lower bound], [upper bound], [lower index], [upper index]
}
        Boundary;

#endif // !BOUNDARY

/*--- Public Functions ---*/

// Allocate memory to boundary lines
void initBoundary(Boundary *BoundaryPtr);

// Copy the boundary line results to the output pointer
void copyBoundary(Boundary *BoundaryPtr, SolverOutput *OutputPtr);

// Free the boundary line memory
void freeBoundary(Boundary *BoundaryPtr);

// Simply copy the legal speed limits as boundaries
void normalBoundary(Boundary *BoundaryPtr, EnvFactor *EnvPtr);

// DP Optimization - Boundary Line (Drawing Part)
void ScreteWeapon(Boundary *BoundaryPtr, SolverInput *SolverInputPtr, SpeedDynParameter *ParaPtr, EnvFactor *EnvPtr,
                  real_T X0);

// DP Optimization - Update State Grid
