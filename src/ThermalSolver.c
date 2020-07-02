#include "../inc/ThermalSolver.h"

/*--- Global Variables ---*/
uint16_t Nx;                                        // Problem sizes
uint16_t Nu;                                        // - they are changed based on running Speed or Thermal solver
uint16_t Nhrz;
uint16_t ResThermal;
Boundary BoundaryStruct;                            // boundary line structure

/*--- External Variables ---*/
real_T Xinitial;                                    // The initial state

/*--- Static Variables ---*/
static real_T *StateVec;                            // vector with all the admissible state values
static real_T *ControlVec;                          // vector with all the admissible control values
static SolverInput *SolverInputPtr;                 // local copy of the input pointer
static DynParameter *ParameterPtr;                  // local copy of the parameter pointer
static EnvFactor *EnvFactorPtr;                     // local copy of the Env factor pointer
static uint16_t X0_index;                           // the rounded X0
static Bridge *SolverBridgePtr;                     // local copy of the bridge pointer


/*--- Private Structure ---*/
typedef struct {
    real_T *CostToCome;                             // The most recent cost-to-come vector [Nx]
    uint16_t *startIdx;                             // All the possible starting indexes [Nx]
    uint16_t Nstart;                                // Number of possible starting points
    uint16_t (*Xn)[NT];                             // Optimal State Trajectory [ResThermal][Nx]
    real_T (*Un)[NT];                               // Optimal Control Policy [ResThermal][Nx]
}
        Solution;

typedef struct {
    real_T (*arcCost)[NT];                          // Vector with arc costs from one to another state [Nx][Nx]
    real_T (*arcU)[NT];                             // Vector with instant controls from one to another state [Nx][Nx]
    real_T (*arcX)[NT];                             // (Used in Adaptive Grid) Vector that notes the possible state to state [Nx][Nx]
}
        ArcProcess;


/*--- Private Functions ---*/
/*--- Using static to restrict functions in this file ---*/

// Initialize Solution Pointer
static void solutionStruct_init(Solution *SolutionPtr);

// Free Solution Pointer
static void solutionStruct_free(Solution *SolutionPtr);

// Find the optimal trajectory based on the Cost-to-Come Matrix calculated in calculate_min_costTocome function
static void findSolution(SolverOutput *OutputPtr, Solution *SolutionPtr, real_T Xfmin, real_T Xfmax);

// Provide the Initial State to the Solution Pointer
static void x0_init(Solution *SolutionPtr, real_T X0);

// Update all the feasible starting states at the current step
static void updateStartX(Solution *SolutionPtr, real_T *CostToCome);

// Initialize Arc Process Pointer
static void arcStruct_init(ArcProcess *ArcPtr);

// Free Arc Process Pointer
static void arcStruct_free(ArcProcess *ArcPtr);

// Calculate the Arc Costs
static void calculate_arc_cost(ArcProcess *ArcPtr, uint16_t N);

// Calculate the Cost-to-Come Values at each state
static void calculate_costTocome(Solution *SolutionPtr, uint16_t N);

/*--- Pubic Function Definition ---*/
void
thermalSolver(SolverInput *InputPtr, DynParameter *ParaPtr, EnvFactor *EnvPtr, SolverOutput *OutputPtr,
              Bridge *BridgePtr, real_T X0, real_T Xfmin, real_T Xfmax) {

    // Local copy of the input pointer
    SolverInputPtr = InputPtr;
    ParameterPtr = ParaPtr;
    EnvFactorPtr = EnvPtr;
    SolverBridgePtr = BridgePtr;

    // Global Copy of X0 (External)
    Xinitial = X0;

    // Global copy of the problem size (Thermal)
    Nx = SolverInputPtr->GridSize.Nt;
    Nu = SolverInputPtr->GridSize.Nq;
    Nhrz = SolverInputPtr->GridSize.Nhrz;
    ResThermal = SolverInputPtr->GridSize.ResThermal;

    // Allocate memory to State Vector and Control Vector
    StateVec = malloc(Nx * sizeof(real_T));
    ControlVec = malloc(Nu * sizeof(real_T));

    // Pass Parameters to SystemDynamics in order to perform calculation
    PassParameters(SolverInputPtr, ParameterPtr, EnvFactorPtr);

    // Initialize State Vector and Control Vector (based on given [Vmin, Vmax] and [Fmin, Fmax])
    createStateVector(StateVec, SolverInputPtr->Constraint.Tmin, SolverInputPtr->Constraint.Tmax, Nx);
    createControlVector(ControlVec, SolverInputPtr->Constraint.Qmin, SolverInputPtr->Constraint.Qmax, Nu);

    // Initialize Solution Structure
    Solution SolutionStruct;
    solutionStruct_init(&SolutionStruct);

    // Give the initial state X0 to the solution structure
    x0_init(&SolutionStruct, X0);
    X0_index = SolutionStruct.startIdx[0];
    real_T X0_round = StateVec[SolutionStruct.startIdx[0]];

    printf("The starting index: %d\n", X0_index);

    // Print Input Info
    printInputInfo(SolverInputPtr, X0, X0_round, SolutionStruct.startIdx[0], StateVec, ControlVec, Nx, Nu);

    // Obtain the Boundary Line
#ifdef CUSTOMBOUND
    initBoundary(&BoundaryStruct);
    customThermalBoundary(&BoundaryStruct, SolverInputPtr, ParameterPtr, EnvFactorPtr, BridgePtr, X0);
#elif defined NORMALBOUND
    initBoundary(&BoundaryStruct);
    normalThermalBoundary(&BoundaryStruct, EnvFactorPtr);
#endif


    // Find the minimum Cost-to-come value step by step
    uint16_t i;
    uint16_t j;
    for (i = 0; i < ResThermal; i++) {
        calculate_costTocome(&SolutionStruct, i);
    }

    // Retrieve the optimal solution
    findSolution(OutputPtr, &SolutionStruct, Xfmin, Xfmax);

#if defined(NORMALBOUND) || defined(CUSTOMBOUND)
    // Get the boundary line to the output pointer
    copyThermalBoundary(&BoundaryStruct, OutputPtr);
#endif

    for (i = 0; i <= RES_THERMAL; i++) {
        printf("Upper: %f, Lower: %f\n", OutputPtr->upperTempBound[i], OutputPtr->lowerTempBound[i]);
    }
    // Print Output Solution
    printThermalSolution(SolverInputPtr, X0_round, OutputPtr);

    // Free the memory
    solutionStruct_free(&SolutionStruct);
    free(StateVec);
    free(ControlVec);
#if defined(NORMALBOUND) || defined(CUSTOMBOUND)
    freeBoundary(&BoundaryStruct);
#endif
}

static void solutionStruct_init(Solution *SolutionPtr) {
    SolutionPtr->CostToCome = malloc(Nx * sizeof(real_T));
    SolutionPtr->startIdx = malloc(Nx * sizeof(uint16_t));

    SolutionPtr->Xn = malloc(sizeof(uint16_t[ResThermal][Nx]));
    SolutionPtr->Un = malloc(sizeof(real_T[ResThermal][Nx]));

    uint16_t i, j;

    // Initialize the most recent cost-to-come value and possible starting points
    for (i = 0; i < Nx; i++) {
        SolutionPtr->CostToCome[i] = SolverInputPtr->SolverLimit.infValue;
        SolutionPtr->startIdx[i] = 0;
    }

    // Initialize Optimal state trajectory and control policy
    for (i = 0; i < ResThermal; i++) {
        for (j = 0; j < Nx; j++) {
            SolutionPtr->Xn[i][j] = 0;
            SolutionPtr->Un[i][j] = NAN;
        }
    }

    // Initialize number of possible starting points
    SolutionPtr->Nstart = 0;
}

static void solutionStruct_free(Solution *SolutionPtr) {
    free(SolutionPtr->CostToCome);
    free(SolutionPtr->startIdx);
    free(SolutionPtr->Xn);
    free(SolutionPtr->Un);
}

static void x0_init(Solution *SolutionPtr, real_T X0) {
    // Round the initial state to the closet point in the state vector
    SolutionPtr->startIdx[0] = (uint16_t) findNearest(StateVec, X0, Nx);
    SolutionPtr->Nstart = 1;
}

static void updateStartX(Solution *SolutionPtr, real_T *CostToCome) {
    uint16_t counter = 0;
    uint16_t i;

    for (i = 0; i < Nx; i++) {
        if (CostToCome[i] < SolverInputPtr->SolverLimit.infValue) {
            SolutionPtr->startIdx[counter] = i;
            counter++;
        }
    }
    // Update the number of possible starting points at the next step
    SolutionPtr->Nstart = counter;
}

static void arcStruct_init(ArcProcess *ArcPtr) {
    uint16_t i, j;

    ArcPtr->arcCost = malloc(sizeof(real_T[Nx][Nx]));
    ArcPtr->arcU = malloc(sizeof(real_T[Nx][Nx]));

    // Initialize the arc costs
    for (i = 0; i < Nx; i++) {
        for (j = 0; j < Nx; j++) {
            ArcPtr->arcCost[i][j] = SolverInputPtr->SolverLimit.infValue;
        }
    }
}

static void arcStruct_free(ArcProcess *ArcPtr) {
    free(ArcPtr->arcCost);
    free(ArcPtr->arcU);
}

static void calculate_costTocome(Solution *SolutionPtr, uint16_t N)        // (N is iteration index)
{
    // Initialize arc process structure
    ArcProcess ArcStruct;
    arcStruct_init(&ArcStruct);

    // Calculate Arc Costs
    calculate_arc_cost(&ArcStruct, N);

    // Initialize cost-to-come vector
    real_T *CostToCome = malloc(Nx * sizeof(real_T));
    // used to store all the possible cost-to-come values to reach each state
    real_T *CostToBeComp = malloc(Nx * sizeof(real_T));

    uint16_t i;
    uint16_t j;


    // Initialize the Cost-to-Come Value at the current State Vector
    for (i = 0; i < Nx; i++) {
        CostToCome[i] = SolverInputPtr->SolverLimit.infValue;
    }

    for (i = 0; i < SolutionPtr->Nstart; i++) {
        uint16_t startIdx = SolutionPtr->startIdx[i];

        // If it is at the initial point
        if (N == 0) {
            // Since it is the first step, simply use arc costs as cost-to-come (from the starting index)
            memcpy(CostToBeComp, ArcStruct.arcCost[startIdx], Nx * sizeof(real_T));
        }
            // If it is not the initial state
        else {
            // we need to add the cost-to-come values to the arc costs
            for (j = 0; j < Nx; j++) {
                CostToBeComp[j] = SolutionPtr->CostToCome[startIdx] + ArcStruct.arcCost[startIdx][j];
            }
        }

        // Pick the minimum cost to be the cost-to-come value
        for (j = 0; j < Nx; j++) {
            if (CostToBeComp[j] < CostToCome[j]) {
                SolutionPtr->Xn[N][j] = startIdx;
                SolutionPtr->Un[N][j] = ArcStruct.arcU[startIdx][j];
                CostToCome[j] = CostToBeComp[j];
            }
        }
    }

    // Obtain the number of possible starting points at the next step.
    updateStartX(SolutionPtr, CostToCome);

    // Copy the output back
    memcpy(SolutionPtr->CostToCome, CostToCome, Nx * sizeof(real_T));

    // Free the intermediate memory
    free(CostToCome);
    free(CostToBeComp);
    arcStruct_free(&ArcStruct);
}

static void calculate_arc_cost(ArcProcess *ArcPtr, uint16_t N)    // N is iteration index
{
    uint16_t i;
    uint16_t j;

    // Establish 2-D arrays [Nx][Nu]
    real_T (*Xnext)[Nu] = malloc(sizeof(real_T[Nx][Nu]));
    real_T (*ArcCost)[Nu] = malloc(sizeof(real_T[Nx][Nu]));
    real_T (*Control)[Nu] = malloc(sizeof(real_T[Nx][Nu]));
    uint8_t (*InfFlag)[Nu] = malloc(sizeof(uint8_t[Nx][Nu]));

    uint32_t *FeasibleCounter = (uint32_t *) malloc(Nx * sizeof(uint32_t));
    uint32_t *idxSort = (uint32_t *) malloc(Nu * sizeof(uint32_t));

    for (i = 0; i < Nx; i++) {
        for (j = 0; j < Nu; j++) {
            Xnext[i][j] = 0;
            ArcCost[i][j] = 0;
            Control[i][j] = 0;
            InfFlag[i][j] = 0;
        }
    }

    // Calculate System Dynamics
    thermalDynamics(Nx, Nu, Xnext, ArcCost, InfFlag, StateVec, ControlVec, &BoundaryStruct, SolverBridgePtr, N,
                    X0_index);

    // Local Copy the State and Control grids
    real_T *StateVecCopy = (real_T *) malloc(Nx * sizeof(real_T));
    memcpy(StateVecCopy, StateVec, Nx * sizeof(real_T));

    // Count the number of feasible control signals per state
    uint32_t counter;

    for (i = 0; i < Nx; i++) {
        counter = 0;
        for (j = 0; j < Nu; j++) {
            if (InfFlag[i][j] == 1) {
                ArcCost[i][j] = SolverInputPtr->SolverLimit.infValue;
                Xnext[i][j] = 0.0;
            } else {
                Xnext[i][counter] = Xnext[i][j];
                ArcCost[i][counter] = ArcCost[i][j];
                InfFlag[i][counter] = InfFlag[i][j];
                Control[i][counter] = ControlVec[j];

                // Counting the number of feasible inputs
                counter++;
            }
        }
        // Store the number of feasible control inputs per starting state
        FeasibleCounter[i] = counter;
    }

    uint16_t iMin;
    uint16_t jMin;




#ifndef ADAPTIVEGRID

    // Interpolate the cost-to-come for states that can be reached
    for (i = 0; i < Nx; i++) {

        /*--- Point to the head of each row ---*/

        // Arc Cost (after interpolation) from one state to another (point-to-point)
        real_T *p2pCost = ArcPtr->arcCost[i];

        // Arc Control values (after interpolation) from one state to another (point-to-point)
        real_T *p2pControl = ArcPtr->arcU[i];

        // Per starting state (on the grid)
        // All these values are before interpolation
        // Xnext are not necessarily on the grid, we want them to be on the grid so that we can calculate cost-to-come grid
        real_T *Xnext_real = Xnext[i];
        real_T *ArcCost_real = ArcCost[i];
        real_T *Control_real = Control[i];

        // If there is only one control input possible
        if (FeasibleCounter[i] == 1) {

            // Find the nearest possible state on the state grid (Since it is not possible to interpolate with only one state)
            uint16_t idx = (uint16_t) findNearest(StateVecCopy, Xnext_real[0], Nx);

            // Store the cost and control to come to this point
            p2pCost[idx] = ArcCost_real[0];
            p2pControl[idx] = Control_real[0];
        } else if (FeasibleCounter[i] > 1) {
            // Sort the idx in the way the Xnext_real[0] to Xnext_real[FeasibleCounter[i]] is monotonically increasing
            sortIdx(Xnext_real, idxSort, FeasibleCounter[i]);

            // Reorder the actual vector based on the calculated idxSort
            reorderVector(Xnext_real, idxSort, FeasibleCounter[i]);
            reorderVector(ArcCost_real, idxSort, FeasibleCounter[i]);
            reorderVector(Control_real, idxSort, FeasibleCounter[i]);

            // Find the range of the states (in the StateVector) where interpolation can be calculated
            uint16_t idxMax = (uint16_t) findMaxLEQ(StateVecCopy, Xnext_real[FeasibleCounter[i] - 1], Nx);
            uint16_t idxMin = (uint16_t) findMinGEQ(StateVecCopy, Xnext_real[0], Nx);

            // Find the up and down points where we perform extrapolation to avoid shrinking the searching space
            //uint16_t idxUp = (uint16_t) findMinGEQ(StateVecCopy, Xnext_real[FeasibleCounter[i] - 1], Nx);
            //uint16_t idxDown = (uint16_t) findMaxLEQ(StateVecCopy, Xnext_real[0], Nx);

            // Calculate all the possible arc costs to each state (in the State Vector)
            LookupTable CostComeTable;
            lookuptable_init(&CostComeTable, Xnext_real, ArcCost_real, FeasibleCounter[i]);
            interpolation(&CostComeTable, StateVecCopy + idxMin, p2pCost + idxMin, idxMax - idxMin + 1);
            lookuptable_free(&CostComeTable);

            // Calculate all the possible arc control signals to each state (in the State Vector)
            LookupTable ControlComeTable;
            lookuptable_init(&ControlComeTable, Xnext_real, Control_real, FeasibleCounter[i]);
            interpolation(&ControlComeTable, StateVecCopy + idxMin, p2pControl + idxMin, idxMax - idxMin + 1);
            lookuptable_free(&ControlComeTable);

        }
    }
#endif

    free(Xnext);
    free(ArcCost);
    free(InfFlag);
    free(Control);
    free(FeasibleCounter);
    free(idxSort);
    free(StateVecCopy);

}

static void findSolution(SolverOutput *OutputPtr, Solution *SolutionPtr, real_T Xfmin, real_T Xfmax) {

    // Find the index range within the final state constraints
    uint16_t minIdx = (uint16_t) findMinGEQ(StateVec, Xfmin, Nx);
    uint16_t maxIdx = (uint16_t) findMaxLEQ(StateVec, Xfmax, Nx);

    real_T minCost = SolverInputPtr->SolverLimit.infValue;

    uint16_t i;
    int16_t k;
    uint16_t finalIdx;

    // Compare all the possible final cost-to-come values, take the minimum one as the minimum total cost
    for (i = minIdx; i <= maxIdx; i++) {
        if (SolutionPtr->CostToCome[i] < minCost) {
            minCost = SolutionPtr->CostToCome[i];
            finalIdx = i;
        }
    }

    //Minimum total cost
    OutputPtr->Cost = minCost;

    // Memory for the optimal state trajectory
    uint16_t *optimalstateIdx = (uint16_t *) calloc(ResThermal + 1, sizeof(uint16_t));

    // Load the final state index
    optimalstateIdx[ResThermal] = finalIdx;

    // Make sure there is at least one solution
    if (minCost < SolverInputPtr->SolverLimit.infValue) {
        // Retrieve the optimal index trajectory
        k = ResThermal - 1;

        while (k >= 0) {
            optimalstateIdx[k] = SolutionPtr->Xn[k][optimalstateIdx[k + 1]];
            k--;
        }

        // Find the optimal control policy and state trajectory
        for (i = 0; i < ResThermal; i++) {
            OutputPtr->To[i] = StateVec[optimalstateIdx[i + 1]];
            OutputPtr->Qo[i] = SolutionPtr->Un[i][optimalstateIdx[i + 1]];
        }
    }

    printf("%f\n", OutputPtr->Cost);
    // Free the memory
    free(optimalstateIdx);
}