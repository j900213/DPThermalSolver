#include "../inc/SystemDynamic.h"
#include "../inc/MathFunction.h"

/*--- Static Variables ---*/
static const DynParameter *ModelParameter;
static EnvFactor *EnvironmentalFactor;
static SolverInput *SolverInputPtr;

#ifdef DYNCOUNTER
uint32_t counterDynamics = 0;
#endif // DYNCOUNTER

/*--- Public Function Definition ---*/
void PassParameters(SolverInput *InputPtr, DynParameter *ModelParaPtr, EnvFactor *EnvFactorPtr) {
    ModelParameter = ModelParaPtr;
    EnvironmentalFactor = EnvFactorPtr;
    SolverInputPtr = InputPtr;
}

void createStateVector(real_T *StateVector, real_T min, real_T max, uint32_t N) {
    real_T delta = (max - min) / (N - 1);
    uint32_t i;
    StateVector[0] = min;

    for (i = 1; i < N; i++) {
        StateVector[i] = StateVector[i - 1] + delta;
    }
}

void createControlVector(real_T *ControlVector, real_T min, real_T max, uint32_t N) {
    real_T delta = (max - min) / (N - 1);
    uint32_t i;
    ControlVector[0] = min;

    for (i = 1; i < N; i++) {
        ControlVector[i] = ControlVector[i - 1] + delta;
    }
}

void
systemDynamics(uint16_t Nu, real_T (*Xnext)[Nu], real_T (*ArcCost)[Nu], uint8_t (*InfFlag)[Nu], real_T const *StateVec,
               real_T const *ControlVec, Boundary *BoundaryPtr, uint16_t N, uint16_t X0_index) {

    // Grid Sizes
    uint16_t Nx = SolverInputPtr->GridSize.Nx;

    // Local Copy the State and Control grids
    real_T *Xin = (real_T *) malloc(Nx * sizeof(real_T));
    real_T *Uin = (real_T *) malloc(Nu * sizeof(real_T));
    memcpy(Xin, StateVec, Nx * sizeof(real_T));
    memcpy(Uin, ControlVec, Nu * sizeof(real_T));

    // Environmental Factors
#if defined(NORMALBOUND) || defined(CUSTOMBOUND)
    // Copy the boundary lines
    real_T Vmax_start = BoundaryPtr->upperBound[N];
    real_T Vmin_start = BoundaryPtr->lowerBound[N];
    real_T Vmax_end = BoundaryPtr->upperBound[N + 1];
    real_T Vmin_end = BoundaryPtr->lowerBound[N + 1];
#elif defined(NOBOUND)
    real_T Vmax_end = EnvironmentalFactor->Vmax_env[N + 1];
    real_T Vmin_end = EnvironmentalFactor->Vmin_env[N + 1];
#endif

#ifdef BOUNDCALIBRATION
    // Besides the points on the state grid, also consider the points on the boundary
    if (N > 0) {
        uint16_t minIdx = (uint16_t) findMaxLEQ(Xin, BoundaryPtr->boundMemo[N - 1][0], Nx);
        uint16_t maxIdx = (uint16_t) findMinGEQ(Xin, BoundaryPtr->boundMemo[N - 1][1], Nx);
        Xin[minIdx] = BoundaryPtr->boundMemo[N - 1][0];
        Xin[maxIdx] = BoundaryPtr->boundMemo[N - 1][1];
    }
#endif

    // Read the slope angle
    real_T angle = EnvironmentalFactor->Angle_env[N + 1];

    // Intermediate Variables
    real_T Pwh;
    real_T Pm;
    real_T Pinv;
    real_T Pdc;
    real_T Pbatt;
    real_T dt;

    // Hard Constraints
    real_T PDmax = SolverInputPtr->Constraint.PDmax;
    real_T PAmax = SolverInputPtr->Constraint.PAmax;

    // Parameters
    real_T m = ModelParameter->m;
    real_T g = ModelParameter->g;
    real_T crr = ModelParameter->crr;
    real_T CdA = ModelParameter->CdA;
    real_T ds = ModelParameter->ds;

    real_T penalty = ModelParameter->penalty;

    real_T eta_trans = ModelParameter->eta_trans;
    real_T eta_dc = ModelParameter->eta_dc;
    real_T alpha0 = ModelParameter->alpha0;
    real_T alpha1 = ModelParameter->alpha1;
    real_T alpha2 = ModelParameter->alpha2;
    real_T beta0 = ModelParameter->beta0;

    // Calculations (all the possibilities)
    uint16_t i;
    uint16_t j;

    // Preserve the initial state accuracy
    if (N == 0) {
        Xin[X0_index] = Xinitial;
    }

    for (i = 0; i < Nx; i++) {

        for (j = 0; j < Nu; j++) {
#if defined(NORMALBOUND) || defined(CUSTOMBOUND)
            // Only calculate the states within the boundaries
            if (Xin[i] > Vmax_start || Xin[i] > SolverInputPtr->Constraint.Vmax || Xin[i] < Vmin_start ||
                Xin[i] < SolverInputPtr->Constraint.Vmin) {
                InfFlag[i][j] = 1;
                continue;
            }
#endif

            Pwh = Uin[j] * Xin[i];

#ifdef DYNCOUNTER
            counterDynamics++;
#endif // DYNCOUNTER


            // First determine if the Pwh has already exceeded the limit
            // Acceleration
            if (Pwh > 0) {
                if (Pwh > PAmax)                    // if the wheel power exceeds the limit...
                {
                    InfFlag[i][j] = 1;                // mark it as 'infeasible'
                    continue;
                }

                Pm = Pwh / eta_trans;
                Pinv = ((1 - alpha1) - sqrt((alpha1 - 1) * (alpha1 - 1) - 4 * alpha2 * (alpha0 + Pm))) / (2 * alpha2);
                Pdc = Pinv / eta_dc;
                Pbatt = (1 - sqrt(1 - 4 * beta0 * Pdc)) / (2 * beta0);

            }
                // Deceleration
            else {
                if (Pwh < PDmax) {
                    InfFlag[i][j] = 1;
                    continue;
                }
                Pm = Pwh * eta_trans;
                Pinv = ((1 - alpha1) - sqrt((alpha1 - 1) * (alpha1 - 1) - 4 * alpha2 * (alpha0 + Pm))) / (2 * alpha2);
                Pdc = Pinv * eta_dc;
                Pbatt = (1 - sqrt(1 - 4 * beta0 * Pdc)) / (2 * beta0);

            }

            // Calculate the speed at the next step -> Check the derivation in the paper
            real_T X_squared = (2 * ds / m) * Uin[j] + (1 - 2 * ds * CdA / m) * Xin[i] * Xin[i] -
                               2 * ds * g * (sin(angle) + crr * cos(angle));


            // Check if the speed squared becomes smaller than 0
            if (X_squared < 0) {
                InfFlag[i][j] = 1;
                continue;
            }

            // The speed at the next step
            Xnext[i][j] = sqrt(X_squared);


            // Check if the speed result is inside the legal speed range and physical speed limits
            if (Xnext[i][j] > Vmax_end || Xnext[i][j] > SolverInputPtr->Constraint.Vmax || Xnext[i][j] < Vmin_end ||
                Xnext[i][j] < SolverInputPtr->Constraint.Vmin) {
                InfFlag[i][j] = 1;
                continue;
            }

            // Calculate dt
            dt = 2 * ds / (Xnext[i][j] + Xin[i]);

            // Arc Cost of this combination: Xin[i] and Uin[j]
            ArcCost[i][j] = (Pbatt + penalty) * dt;

        }
    }

    // Free the memory of local state and control vectors
    free(Xin);
    free(Uin);
}