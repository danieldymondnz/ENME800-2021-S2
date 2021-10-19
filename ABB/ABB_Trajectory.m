syms D7

% % The tool at the Starting Position can be described as
% T_70_PO = [ 0   0   1   1000
%             0   1  0   0050
%             -1   0   0   1000 
%             0   0   0   1       ];
% 
% T_70_PA = [ 0   1   0   1125
%             1   0   0   0025
%             0   0   -1  0308
%             0   0   0   1       ];
% 
% % Determine T_60
% [T_subs, ~, ~, ~] = ABB_Config();
% T_76 = cell2sym(T_subs(7));
% T_76 = subs(T_76, D7, 165);
% T_76_inv = inverseTransMatrix(T_76);
% 
% T_60_PO = multiplyTransMatrix(T_70_PO, T_76_inv);
% T_60_PO = double(T_60_PO)
% 
% T_60_PA = multiplyTransMatrix(T_70_PA, T_76_inv);
% T_60_PA = double(T_60_PA)

% %% Solve for Position O
% [thetas_O] = ABB_IK_solveTheta(T_60_PO);
% 
% %% Solve for Position A
% [thetas_A] = ABB_IK_solveTheta(T_60_PA);

%% Solve for PO to PA Trajectory

% Find optimal path
%[tOI, tAI] = findOptimalPath(thetas_O, thetas_A);
POT = [0.0598    0.0437   -0.6301    0.1078   -0.5891    3.0519]
PAT = [0.0222   -0.6604   -0.4992         0    0.4112   -1.5486]

% Maximums from Datasheet
dT_MAX = deg2rad([175 175 175 360 360 500]);
T_MAX = 4;
ACC = 100;
%ddThetas not found

%% Peform a Binary Search on T to find correct element

[optTime, calc] = generateTrajectoryTiming(POT,PAT,0, 0, T_MAX, ACC)

% Extract Equations
a_matrix = calc(:, 2:5)
outputPlot = figure(1);
view(3);
hold on;

% Plot initial position
tMat = linspace(0, optTime, 10);
for frm=1:length(tMat)

    t = tMat(frm);
    thetas = zeros(1,7);
    for joint=1:6
        thetas(joint) = a_matrix(joint,1) + a_matrix(joint,2)*t + a_matrix(joint,3)*t^2 + a_matrix(joint,4)*t^3;
    
    end
    cla(outputPlot);
    op = ABB_Plot(thetas, 1, outputPlot);
    anim(frm) = getframe(outputPlot);

   % pause(1)
    
end

movie(anim,1)







