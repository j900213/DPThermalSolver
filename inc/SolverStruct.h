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
// TODO: These values require to be the same as defined in MatLab
#define HORIZON 200
#define NV 200
#define NF 500
#define NT 50
#define NQ 100
#define BLOCK 4


// Choose pre-defined scenario (SCENE1/SCENE2)
#define SCENE1
// Turn on/off the counter of dynamics computation
#define DYNCOUNTER
// Turn on/off the counter of interpolation computation
#define INTERPOCOUNTER
// Turn on/off the counter of boundary calculations
#define BOUNDCOUNTER
// Choose the boundary line mode (NOBOUND/NORMALBOUND/CUSTOMBOUND)
#define NOBOUND
// Turn on/off boundary calibration
//#define BOUNDCALIBRATION
// Turn on/off Adaptive grid method
//#define ADAPTIVEGRID

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
    uint16_t Nv;                // Number of state grid points (Speed)
    uint16_t Nf;                // Number of control grid points (Speed)
    uint16_t Nt;                // Number of state grid points (Thermal)
    uint16_t Nq;                // Number of control grid points (Thermal)
    uint16_t Nhrz;              // Length of the horizon
}
        GridSetting;

typedef struct {
    real_T Vmax;                // Upper limit of the speed
    real_T Vmin;                // Lower limit of the speed
    real_T Fmax;                // Upper limit of the force
    real_T Fmin;                // Lower limit of the force
    real_T PAmax;               // Upper limit of the power when acceleration (at wheels)
    real_T PDmax;               // Upper limit of the power when deceleration (at wheels)
    real_T Tmax;                // Upper limit of the cabin temperature
    real_T Tmin;                // Lower limit of the cabin temperature
    real_T Tmax_inlet;          // Upper limit of the inlet temperature
    real_T Tmin_inlet;          // Lower limit of the inlet temperature
    real_T Qmin;                // Upper limit of the inlet heat flow
    real_T Qmax;                // Lower limit of the inlet heat flow
    real_T PACmax;              // Upper limit of the AC power
    real_T PACmin;              // Lower limit of the AC power
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
    // Speed
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

    // Thermal
    real_T Cth;
    real_T Rth;
    real_T Qsun;
    real_T Qpas;
    real_T Cp;
    real_T rho;
    real_T mDot;
    real_T CoP_pos;
    real_T CoP_neg;
    real_T Tamb;

    // Tuning Parameter
    real_T speedPenalty;
    real_T thermalPenalty;
}
        DynParameter;

typedef struct {
    real_T Vmax_env[HORIZON + 1];
    real_T Vmin_env[HORIZON + 1];
    real_T Angle_env[HORIZON + 1];
    uint16_t endBlock[BLOCK];
    real_T T_required[HORIZON + 1];
}
        EnvFactor;

typedef struct {
    real_T Cost;                            // Total Cost
    real_T Vo[HORIZON];                     // Optimal Speed Trajectory
    real_T Fo[HORIZON];                     // Optimal Speed Control Policy
    real_T To[HORIZON];                     // Optimal Thermal Trajectory
    real_T Qo[HORIZON];                     // Optimal Thermal Control Policy
    real_T upperBound[HORIZON + 1];         // Upper Boundary Line
    real_T lowerBound[HORIZON + 1];         // Lower Boundary Line
    real_T upperActual[HORIZON];
    real_T lowerActual[HORIZON];
}
        SolverOutput;

#endif // !STRUCT