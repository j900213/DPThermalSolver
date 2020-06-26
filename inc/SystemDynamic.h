#include "SolverStruct.h"
#include "BoundaryLine.h"

/*--- Public Functions ---*/
// Get the Parameter Setting from the main.c
void PassParameters(SolverInput *InputPtr, DynParameter *ModelParaPtr, EnvFactor *EnvFactorPtr);

// Create State Grid [min, max]
void createStateVector(real_T *StateGrid, real_T min, real_T max, uint32_t N);

// Create Control Grid [min, max]
void createControlVector(real_T *ControlGrid, real_T min, real_T max, uint32_t N);

// Calculate the system dynamics
void systemDynamics(uint16_t Nu, real_T (*Xnext)[Nu], real_T (*ArcCost)[Nu], uint8_t (*InfFlag)[Nu], real_T const *StateVec,
                    real_T const *ControlVec, Boundary *BoundaryPtr, uint16_t N, uint16_t X0_index);