%% Final state constraint
Xfmax = 400/3.6;
Xfmin = 0/3.6;

%% Gird Settings
% <<IMPORTANT!>> Nx, Nu, Nhrz has to be the same as NX, NU, HORIZON in SolverStruct.h
solverinput.GridSize.Nx = 200;
solverinput.GridSize.Nu = 500;
solverinput.GridSize.Nhrz = 200;

%% Limit Settings
solverinput.Constraint.Vmax = 200/3.6;
solverinput.Constraint.Vmin = 0.0;
solverinput.Constraint.Fmax = 3e3;
solverinput.Constraint.Fmin = -3e3;
% Max Power when acceleration and deceleration
solverinput.Constraint.PAmax = 6e4;
solverinput.Constraint.PDmax = -6e4;

%% Solver Settings
% Big value (FLT_MAX) indicating infeasibility
solverinput.SolverLimit.infValue = 340282346638528859811704183484516925440.0;

%% Model Parameters
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
modelPara.penalty = 11e4;


%% Environmental Info (Scenario 1)
% Initial Speed
X0 = 70/3.6;

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

%% Environmental Info (Scenario 2)
% Initial Speed
X0 = 0/3.6;

% Horizon
solverinput.GridSize.Nhrz = 200;

% GPS info
envFactor = struct;
envFactor.Vmax_env = zeros((solverinput.GridSize.Nhrz+1), 1);
envFactor.Vmin_env = zeros((solverinput.GridSize.Nhrz+1), 1);
envFactor.Angle_env = zeros((solverinput.GridSize.Nhrz+1), 1);

Vmax_GPS = [130/3.6];
Vmin_GPS = [0/3.6];
Angle_GPS = [0.0];
changePoint = [];

envFactor.endBlock = [changePoint solverinput.GridSize.Nhrz];

for i = 1:(solverinput.GridSize.Nhrz+1)
        envFactor.Vmax_env(i) = Vmax_GPS(1);
        envFactor.Vmin_env(i) = Vmin_GPS(1);
        envFactor.Angle_env(i) = Angle_GPS(1);
end

%% Environmental Info (Scenario 2)
% Initial Speed
X0 = 0/3.6;

% Horizon
solverinput.GridSize.Nhrz = 100;

% GPS info
envFactor = struct;
envFactor.Vmax_env = zeros((solverinput.GridSize.Nhrz+1), 1);
envFactor.Vmin_env = zeros((solverinput.GridSize.Nhrz+1), 1);
envFactor.Angle_env = zeros((solverinput.GridSize.Nhrz+1), 1);

Vmax_GPS = [130/3.6];
Vmin_GPS = [0/3.6];
Angle_GPS = [0.0];
changePoint = [];

envFactor.endBlock = [changePoint solverinput.GridSize.Nhrz];

for i = 1:(solverinput.GridSize.Nhrz+1)
        envFactor.Vmax_env(i) = Vmax_GPS(1);
        envFactor.Vmin_env(i) = Vmin_GPS(1);
        envFactor.Angle_env(i) = Angle_GPS(1);
end