%% Final state constraint
Vfmax = 400/3.6;
Vfmin = 0/3.6;
Tfmax = 30;
Tfmin = 10;

%% Gird Settings
% <<IMPORTANT!>> Nx, Nu, Nhrz has to be the same as NX, NU, HORIZON in SolverStruct.h
solverinput.GridSize.Nv = 200;
solverinput.GridSize.Nf = 500;
solverinput.GridSize.Nt = 500;
solverinput.GridSize.Nq = 100;
solverinput.GridSize.Nhrz = 200;
solverinput.GridSize.ResThermal = 20;

%% Limit Settings
solverinput.Constraint.Vmax = 200/3.6;
solverinput.Constraint.Vmin = 0.0;
solverinput.Constraint.Fmax = 3e3;
solverinput.Constraint.Fmin = -3e3;
solverinput.Constraint.PAmax = 6e4;
solverinput.Constraint.PDmax = -6e4;
solverinput.Constraint.Tmax = 30;
solverinput.Constraint.Tmin = 10;
solverinput.Constraint.Tmax_inlet = 30;
solverinput.Constraint.Tmin_inlet = 10;
solverinput.Constraint.Qmax = 2000;
solverinput.Constraint.Qmin = -2000;
solverinput.Constraint.PACmax = 2000/2.1379;

%% Solver Settings
% Big value (FLT_MAX) indicating infeasibility
solverinput.SolverLimit.infValue = 340282346638528859811704183484516925440.0;

%% Model Parameters
% Speed parameters
modelPara.m = 2000;
modelPara.g = 9.81;
modelPara.crr = 0.006;
modelPara.CdA = 0.6;
modelPara.ds = 10;
modelPara.eta_trans = 0.98;
modelPara.eta_dc = 0.99;
modelPara.alpha0 = 785.0;
modelPara.alpha1 = -43e-4;
modelPara.alpha2 = 5.6e-7;
modelPara.beta0 = 41e-7;

% Thermal parameters
modelPara.Cth = 1.1347e5;
modelPara.Rth = 0.015;
modelPara.Qsun = 150 * 1.7725;
modelPara.Qpas = 4 * 104.2975;
modelPara.Cp = 1.0035e3;
modelPara.rho = 1.1839;
modelPara.mDot = 0.0842;
modelPara.CoP_pos = 2.1379;
modelPara.CoP_neg = -2.1379;
modelPara.Tamb = 28;

% Tuning Penalty
modelPara.speedPenalty = 11e4;
modelPara.thermalPenalty = 10e6;


%% Scenario 1 - Speed
% Initial Speed
V0 = 70/3.6;

% Horizon
solverinput.GridSize.Nhrz = 200;

% GPS info
envFactor = struct;
envFactor.Vmax_env = zeros((solverinput.GridSize.Nhrz+1), 1);
envFactor.Vmin_env = zeros((solverinput.GridSize.Nhrz+1), 1);
envFactor.Angle_env = zeros((solverinput.GridSize.Nhrz+1), 1);

Vmax_GPS = [130/3.6 80/3.6 50/3.6 80/3.6];
Vmin_GPS = [60/3.6 30/3.6 10/3.6 30/3.6];
Angle_GPS = [0.0 0.0 0.0 0.0];
changePoint = [30 100 150];

envFactor.endBlock = [changePoint solverinput.GridSize.Nhrz];

for i = 1:(solverinput.GridSize.Nhrz+1)
    if i<=changePoint(1)
        envFactor.Vmax_env(i) = Vmax_GPS(1);
        envFactor.Vmin_env(i) = Vmin_GPS(1);
        envFactor.Angle_env(i) = Angle_GPS(1);
    elseif i<=changePoint(2)
        envFactor.Vmax_env(i) = Vmax_GPS(2);
        envFactor.Vmin_env(i) = Vmin_GPS(2);
        envFactor.Angle_env(i) = Angle_GPS(2);
    elseif i<=changePoint(3)
        envFactor.Vmax_env(i) = Vmax_GPS(3);
        envFactor.Vmin_env(i) = Vmin_GPS(3);
        envFactor.Angle_env(i) = Angle_GPS(3);
    else
        envFactor.Vmax_env(i) = Vmax_GPS(4);
        envFactor.Vmin_env(i) = Vmin_GPS(4);
        envFactor.Angle_env(i) = Angle_GPS(4);
    end 
end

%% Scenario 2 - Speed
% Initial Speed
% V0 = 0/3.6;
% 
% % Horizon
% solverinput.GridSize.Nhrz = 200;
% 
% % GPS info
% envFactor = struct;
% envFactor.Vmax_env = zeros((solverinput.GridSize.Nhrz+1), 1);
% envFactor.Vmin_env = zeros((solverinput.GridSize.Nhrz+1), 1);
% envFactor.Angle_env = zeros((solverinput.GridSize.Nhrz+1), 1);
% 
% Vmax_GPS = [130/3.6];
% Vmin_GPS = [0/3.6];
% Angle_GPS = [0.0];
% changePoint = [];
% 
% envFactor.endBlock = [changePoint solverinput.GridSize.Nhrz];
% 
% for i = 1:(solverinput.GridSize.Nhrz+1)
%         envFactor.Vmax_env(i) = Vmax_GPS(1);
%         envFactor.Vmin_env(i) = Vmin_GPS(1);
%         envFactor.Angle_env(i) = Angle_GPS(1);
% end

%% Scenario 1 - Thermal
T0 = 25;

envFactor.T_required = zeros((solverinput.GridSize.Nhrz+1), 1);

T_required = [26, 25, 25, 26];

for i = 1:(solverinput.GridSize.Nhrz+1)
    if i<=changePoint(1)
        envFactor.T_required(i) = T_required(1);
    elseif i<=changePoint(2)
        envFactor.T_required(i) = T_required(2);
    elseif i<=changePoint(3)
        envFactor.T_required(i) = T_required(3);
    else
        envFactor.T_required(i) = T_required(4);
    end 
end