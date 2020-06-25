%% Extract the results from Simulink
Fo_copy = Fo(1,:);
Vo_copy = Vo(1,:);
Vo_copy = [X0 Vo_copy];
upperBound_copy = upperBound(1,:);
lowerBound_copy = lowerBound(1,:);
upperActual_copy = [X0 upperActual(1,:)];
lowerActual_copy = [X0 lowerActual(1,:)];
MinCost = MinCost(1);

%% Local Copy of grid sizes from 'createSfunc.m'
NHrz = solverinput.GridSize.Nhrz;
Nhrz = solverinput.GridSize.Nhrz;
Nx = solverinput.GridSize.Nx;

%% Set up Upper and Lower bounds of speed
[a, N] = size(envFactor.endBlock);
[b, M] = size(Vmax_GPS);

endBlock = envFactor.endBlock;
endBlock(end) = endBlock(end) + 1;
turningPoint = [1 endBlock];

%% Verify the speed trajectory (based on the given control policy and inital speed)
% Parameter Settings (local copy)
m = modelPara.m;
g = modelPara.g;
CdA = modelPara.CdA;
crr = modelPara.crr;
ds = modelPara.ds;
eta_trans = modelPara.eta_trans;
eta_dc = modelPara.eta_dc;
alpha0 = modelPara.alpha0;
alpha1 = modelPara.alpha1;
alpha2 = modelPara.alpha2;
beta0 = modelPara.beta0;
penalty = modelPara.penalty;

angle = 0.0;

ActualSpeedTrajectory = zeros(1, NHrz+1);
ActualSpeedTrajectory(1) = X0;

for i = 1:NHrz
    Xk_plus_1 = sqrt((2*ds/m)*Fo_copy(i) + (1 - 2*ds*CdA/m)*(ActualSpeedTrajectory(i))^2 - 2*ds*g*(sin(angle)+crr*cos(angle)));
    ActualSpeedTrajectory(i+1) = Xk_plus_1;
end

%% Verify the Total cost (based on the given speed trajectory and control policy)
totalCost = 0;

for i = 1:NHrz
    
    P_wh = Vo_copy(i)*Fo_copy(i);
    
    if Fo_copy(i)>0
        % Acceleration
        P_m = P_wh/eta_trans;
        P_inv = ((1-alpha1)-sqrt((alpha1-1)^2 - 4*alpha2*(alpha0+P_m)))/(2*alpha2);
        P_dc = P_inv/eta_dc;
        P_batt = (1 - sqrt(1-4*beta0*P_dc))/(2*beta0);
        disp(i)
        disp(P_batt)
    else
        % Deceleration
        P_m = P_wh*eta_trans;
        P_inv = ((1-alpha1)-sqrt((alpha1-1)^2 - 4*alpha2*(alpha0+P_m)))/(2*alpha2);
        P_dc = P_inv*eta_dc;
        P_batt = (1 - sqrt(1-4*beta0*P_dc))/(2*beta0);
    end
    
    dt = 2*ds/(Vo_copy(i+1) + Vo_copy(i));
    
    totalCost = totalCost + (P_batt + penalty)*dt;
end

disp('Actual Cost based on the given trajectories:')
disp(totalCost)
disp('The Minimum cost is:')
disp(MinCost)

%% Plot
% Generate State Vector
delta = (solverinput.Constraint.Vmax - solverinput.Constraint.Vmin) / (Nx - 1);
StateVector = zeros(1, Nx);

for i = 2:Nx
    StateVector(i) = StateVector(i - 1) + delta;
end

hold on

% for i = 0:10
%     plot(i*ds, (StateVector')*3.6, "k.",'MarkerSize',8);
% end

% Plot speed trajectory, boundary, etc.
grid on;
line(1) = plot((0:Nhrz)*ds, Vo_copy(1:Nhrz+1)*3.6,'-','LineWidth',1.2, 'Color', [0.8500, 0.3250, 0.0980]);
line(2) = plot((0:Nhrz)*ds, ActualSpeedTrajectory(1:NHrz+1)*3.6,'LineWidth',1.2, 'Color', [0.9290, 0.6940, 0.1250]);

for i = 1:N
    yUpper = [Vmax_GPS(i), Vmax_GPS(i)];
    yLower = [Vmin_GPS(i), Vmin_GPS(i)];
    xIndex = [turningPoint(i), (turningPoint(i+1) - 1)];
    line(3) = plot(xIndex*ds, yUpper*3.6, 'LineWidth',2, 'Color', [0.25, 0.25, 0.25]);
    line(4) = plot(xIndex*ds, yLower*3.6, 'LineWidth',2, 'Color', [0.25, 0.25, 0.25]);
end
line(5) = plot((0:Nhrz)*ds, upperBound_copy(1:Nhrz+1)*3.6,'LineWidth',1.2, 'Color', [0.4660, 0.6740, 0.1880]);
line(6) = plot((0:Nhrz)*ds, lowerBound_copy(1:Nhrz+1)*3.6,'LineWidth',1.2, 'Color', [0.4660, 0.6740, 0.1880]);
line(7) = plot((0:Nhrz)*ds, upperActual_copy(1:Nhrz+1)*3.6,'r.','MarkerSize',5);
line(8) = plot((0:Nhrz)*ds, lowerActual_copy(1:Nhrz+1)*3.6,'r.','MarkerSize',5);

xlabel('Distance (m)');
ylabel('Speed (km/h)');
legend(line([1 2 3 5 7]),  'Calculated speed trajectpry', 'Actual speed trajectory','Legal speed limits', 'Calculated Boundary line', 'Calibration')
hold off;
