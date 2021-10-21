%%  Trajectory Planning Script

%% Load Config for robot
syms D7
[T_subs, ~, ~, ~] = ABB_Config();
T_76 = cell2sym(T_subs(7));
T_76 = subs(T_76, D7, 165);
T_76_inv = inverseTransMatrix(T_76);

%% Determine the Joint Angles at Position O
% The tool at the Starting Position can be described as T_70_PO
% Get in Wrist Frame and determine thetas
T_70_PO = [ 0   0   1   1000
            0   1   0   0050
            -1  0   0   1000 
            0   0   0   1    ];

T_60_PO = multiplyTransMatrix(T_70_PO, T_76_inv);
T_60_PO = double(T_60_PO);
[thetas_O] = ABB_IK_solveTheta(T_60_PO);

% The tool at Position A can be described as T_70_PA
% Get in Wrist Frame and determine thetas
T_70_PA = [ 0   1   0   1125
            1   0   0   0025
            0   0   -1  0308
            0   0   0   1       ];

T_60_PA = multiplyTransMatrix(T_70_PA, T_76_inv);
T_60_PA = double(T_60_PA);
[thetas_A] = ABB_IK_solveTheta(T_60_PA);

%% Solve for PO to PA Trajectory
% Find optimal path
[tOI, tAI] = findOptimalPath(thetas_O, thetas_A);
POT = thetas_O(tOI, :);
PAT = thetas_A(tOI, :);

%% Peform a Binary Search on T to find optimal time

T_MAX = 4;  % Maximum Time to Binary Search to
ACCURACY = 100;  % Accuracy (num of intervals to sample over this time)

% Calculate optimal time and coefficients for quadratic equations, extract
% a parameters for quadratic equations
[optTime, calc] = generateTrajectoryTiming(POT,PAT,0, 0, T_MAX, ACCURACY);
a_matrix = calc(:, 2:5);

%% Plot an Animation

FRAMES = 10;    % Number of desired frames for the animation

% Create plot and animation timing information
tMat = linspace(0, optTime, FRAMES);
pos = zeros(FRAMES, 3);
outputPlot = figure;
view(3);
hold on;

% Generate a plot for each position and capture frame
for frm=1:length(tMat)

    % Get the time at which this frame will sample for
    t = tMat(frm);

    % For each joint, determine the angle at time
    thetas = zeros(1,6);
    for joint=1:6
        thetas(joint) = a_matrix(joint,1) + a_matrix(joint,2)*t + a_matrix(joint,3)*t^2 + a_matrix(joint,4)*t^3;
    
    end

    % Clear old plot, draw the frame, and capture
    cla(outputPlot);
    [op, x, y, z] = ABB_Plot(thetas, 1, outputPlot);
    pos(frm, :) = [x, y, z];
    if frm > 1
        for n = 2:frm
            op = plot3([pos(n-1,1) pos(n,1)], [pos(n-1,2) pos(n,2)], [pos(n-1,3) pos(n,3)], 'magenta', 'LineWidth', 2);
        end
    end
    anim(frm) = getframe(outputPlot);
    
end

% Display animation
figure(2)
movie(anim,10)