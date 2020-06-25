#include "SolverStruct.h"

/*--- Public Functions ---*/

// Create Box Edges Vector
void createBoxEdges(real_T *BoxEdgesVector, real_T const *StateVector, uint32_t N);

// Create Adaptive State Grid
void createStateGrid(real_T (*StateGrid)[NX], real_T *StateVec, uint16_t Nx, uint16_t Nhrz);