#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "float.h"
#include "math.h"

#ifndef real_T                    // double type in MatLab
#define real_T double
#endif // !real_T

#ifndef STRUCT
#define STRUCT

// Problem sizes
//TODO: These 4 values require to be the same as defined in MatLab
#define HORIZON 200
#define NX 200
#define NU 500
#define BLOCK 1


// Choose pre-defined scenario (SCENE1/SCENE2)
#define SCENE2
// Turn on/off the counter of dynamics computation
#define DYNCOUNTER
// Turn on/off the counter of interpolation computation
#define INTERPOCOUNTER
// Turn on/off the counter of boundary calculations
#define BOUNDCOUNTER
// Choose the boundary line mode (NOBOUND/NORMALBOUND/CUSTOMBOUND)
#define CUSTOMBOUND
// Turn on/off boundary calibration
#define BOUNDCALIBRATION
// Turn on/off Adaptive grid method
#define ADAPTIVEGRID

/*--- External Variables ---*/
extern real_T Xinitial;

#ifdef DYNCOUNTER
extern uint32_t counterDynamics;
#endif // DYNCOUNTER

#ifdef INTERPOCOUNTER
extern uint32_t counterInterpo;
#endif // INTERPOCOUNTER

#ifdef BOUNDCOUNTER
extern uint32_t counterBound;
#endif // BOUNDCOUNTER

typedef struct {
    uint16_t Nx;                // Number of state grid points
    uint16_t Nu;                // Number of control grid points
    uint16_t Nhrz;                // Resolution of the horizon
}
        GridSetting;

typedef struct {
    real_T Vmax;                // Upper limit of the speed
    real_T Vmin;                // Lower limit of the speed
    real_T Fmax;                // Upper limit of the force
    real_T Fmin;                // Lower limit of the force
    real_T PAmax;               // Upper limit of the power when acceleration (at wheels)
    real_T PDmax;               // Upper limit of the power when deceleration (at wheels)
}
        LimitSetting;

typedef struct {
    real_T infValue;            // The cost for an infeasible value
}
        SolverSetting;

typedef struct {
    GridSetting GridSize;
    LimitSetting Constraint;
    SolverSetting SolverLimit;
}
        SolverInput;

typedef struct {
    real_T m;
    real_T g;
    real_T crr;
    real_T CdA;
    real_T ds;
    real_T eta_trans;
    real_T eta_dc;
    real_T alpha0;
    real_T alpha1;
    real_T alpha2;
    real_T beta0;
    // Tuning Parameter
    real_T penalty;
}
        SpeedDynParameter;

typedef struct {
    real_T Vmax_env[HORIZON + 1];
    real_T Vmin_env[HORIZON + 1];
    real_T Angle_env[HORIZON + 1];
    uint16_t endBlock[BLOCK];
}
        EnvFactor;

typedef struct {
    real_T Cost;                            // Total Cost
    real_T Vo[HORIZON];                    // Optimal State Trajectory
    real_T Fo[HORIZON];                    // Optimal Control Policy
    real_T upperBound[HORIZON + 1];        // Upper Boundary Line
    real_T lowerBound[HORIZON + 1];        // Lower Boundary Line
    real_T upperActual[HORIZON];
    real_T lowerActual[HORIZON];
}
        SolverOutput;

#endif // !STRUCT