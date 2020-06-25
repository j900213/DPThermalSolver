#include "./inc/SolverStruct.h"
#include "./inc/BasicAlgorithm.h"

int main()
{
    /*----------------------*/
    /*--- Initialization ---*/
    /*----------------------*/

    // Input Variable declaration (will be used in the BasicAlgorithm)
    SolverInput SolverInputPtr;						// solver inputs: Constraints, Grid Sizes, Road Profile, etc.
    SpeedDynParameter ModelParaPtr;					// solver inputs: Model parameters
    EnvFactor EnvFactorPtr;							// solver inputs: Environmental Factors (Legal speed limits, Angle of slopes)
    SolverOutput SolverOutputPtr;					// solver outputs: Minimum Total Cost, Optimal Speed Trajectory, Optimal Control Policy
    real_T X0;										// initial state (speed in this case)
    real_T Xfmin;									// final state constraints
    real_T Xfmax;

    // Final states constraint
    Xfmin = 0 / 3.6;
    Xfmax = 400 / 3.6;

    // Input Settings
    SolverInputPtr.GridSize.Nx = NX;
    SolverInputPtr.GridSize.Nu = NU;
    SolverInputPtr.GridSize.Nhrz = HORIZON;

    SolverInputPtr.Constraint.Vmax = 200 / 3.6;		// Physical Speed limits
    SolverInputPtr.Constraint.Vmin = 0.0;
    SolverInputPtr.Constraint.Fmax = 3e3;
    SolverInputPtr.Constraint.Fmin = -3e3;
    SolverInputPtr.Constraint.PAmax = 6e4;
    SolverInputPtr.Constraint.PDmax = -6e4;

    SolverInputPtr.SolverLimit.infValue = FLT_MAX;

    // System Dynamics Parameters
    ModelParaPtr.m = 2000;
    ModelParaPtr.g = 9.81;
    ModelParaPtr.CdA = 0.6;
    ModelParaPtr.crr = 0.006;

    ModelParaPtr.alpha0 = 785.0;
    ModelParaPtr.alpha1 = -43e-4;
    ModelParaPtr.alpha2 = 5.6e-7;
    ModelParaPtr.beta0 = 41e-7;
    ModelParaPtr.eta_dc = 0.99;
    ModelParaPtr.eta_trans = 0.98;

    // Tuning Parameter
    ModelParaPtr.ds = 10;
    ModelParaPtr.penalty = 11e4;

    // Environmental Information
    uint8_t numFactors = 3;
    uint16_t Nhrz = HORIZON;
    uint16_t i;

#ifdef SCENE1
    // Initial Speed
    X0 = 70/3.6;

    real_T Vmax_GPS_1 = 130 / 3.6;
    real_T Vmin_GPS_1 = 60 / 3.6;
    uint16_t endBlock_1 = 30;

    real_T Vmax_GPS_2 = 80 / 3.6;
    real_T Vmin_GPS_2 = 30 / 3.6;
    uint16_t endBlock_2 = 100;

    real_T Vmax_GPS_3 = 50 / 3.6;
    real_T Vmin_GPS_3 = 10 / 3.6;
    uint16_t endBlock_3 = 150;

    real_T Vmax_GPS_4 = 80 / 3.6;
    real_T Vmin_GPS_4 = 30 / 3.6;
    uint16_t endBlock_4 = Nhrz;

    EnvFactorPtr.endBlock[0] = endBlock_1;
    EnvFactorPtr.endBlock[1] = endBlock_2;
    EnvFactorPtr.endBlock[2] = endBlock_3;
    EnvFactorPtr.endBlock[3] = endBlock_4;

    for (i = 0; i <= Nhrz; i++)
    {
        if (i < endBlock_1)
        {
            EnvFactorPtr.Vmax_env[i] = Vmax_GPS_1;					// Legal Vmax
            EnvFactorPtr.Vmin_env[i] = Vmin_GPS_1;					// Legal Vmin
            EnvFactorPtr.Angle_env[i] = 0.0;						// Road slops
        }
        else if (i < endBlock_2)
        {
            EnvFactorPtr.Vmax_env[i] = Vmax_GPS_2;					// Legal Vmax
            EnvFactorPtr.Vmin_env[i] = Vmin_GPS_2;					// Legal Vmin
            EnvFactorPtr.Angle_env[i] = 0.0;						// Road slops
        }
        else if (i < endBlock_3)
        {
            EnvFactorPtr.Vmax_env[i] = Vmax_GPS_3;					// Legal Vmax
            EnvFactorPtr.Vmin_env[i] = Vmin_GPS_3;					// Legal Vmin
            EnvFactorPtr.Angle_env[i] = 0.0;						// Road slops
        }
        else
        {
            EnvFactorPtr.Vmax_env[i] = Vmax_GPS_4;					// Legal Vmax
            EnvFactorPtr.Vmin_env[i] = Vmin_GPS_4;					// Legal Vmin
            EnvFactorPtr.Angle_env[i] = 0.0;						// Road slops
        }
    }
#elif defined(SCENE2)
    // Initial Speed
    X0 = 0/3.6;

    real_T Vmax_GPS_1 = 130 / 3.6;
    real_T Vmin_GPS_1 = 0 / 3.6;

    for (i = 0; i <= Nhrz; i++)
    {
        EnvFactorPtr.Vmax_env[i] = Vmax_GPS_1;					// Legal Vmax
        EnvFactorPtr.Vmin_env[i] = Vmin_GPS_1;					// Legal Vmin
        EnvFactorPtr.Angle_env[i] = 0.0;						// Road slops
    }
#endif

    /*-------------------------*/
    /*--- Run the Algorithm ---*/
    /*-------------------------*/
    MagicBox(&SolverInputPtr, &ModelParaPtr, &EnvFactorPtr, &SolverOutputPtr, X0, Xfmin, Xfmax);

#ifdef DYNCOUNTER
    printf("The number of dynamics computation: %d\n", counterDynamics);
#endif // DYNCOUNTER

#ifdef INTERPOCOUNTER
    printf("The number of interpolation computation: %d\n", counterInterpo);
#endif // INTERPOCOUNTER

#ifdef BOUNDCOUNTER
    printf("The number of boundary computation: %d\n", counterBound);
#endif // BOUNDCOUNTER

    // Fix
    return 0;
}