#include "../inc/AdaptiveGrid.h"

void createBoxEdges(real_T *BoxEdgesVector, real_T const *StateVector, uint32_t N) {
    uint32_t i;

    // The lowest and highest edge
    BoxEdgesVector[0] = StateVector[0];
    BoxEdgesVector[N] = StateVector[N - 1];

    // Calculate the half points
    for (i = 1; i < N; i++) {
        BoxEdgesVector[i] = (StateVector[i - 1] + StateVector[i]) / 2;
    }
}

void createStateGrid(real_T (*StateGrid)[NX], real_T *StateVec, uint16_t Nx, uint16_t Nhrz) {
    uint16_t i;
    for (i = 0; i <= Nhrz; i++) {
        memcpy(StateGrid[i], StateVec, Nx * sizeof(real_T));
    }
}