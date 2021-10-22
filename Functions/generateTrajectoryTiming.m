function [optTime, calc] = generateTrajectoryTiming(Pi_thetas,Pf_thetas,Vi, Vf, T_max, samples)

    dT_MAX = deg2rad([175 175 175 360 360 500]);

    % Create a matrix of times to binary search
    tMat = linspace(0, T_max, samples + 1);

    % Determine the optimal time for the path
    [optTime] = performLoopIteration(tMat, Pi_thetas, Pf_thetas, Vi, Vf, dT_MAX, T_max, samples);

    % Calculate the a0-3 constants, and joint velocities based on the best
    % time
    calc = zeros(length(dT_MAX), 5);
    for joint=1:length(dT_MAX)
        [~, a0, a1, a2, a3, maxVel] = generateCubicTrajectory(Pi_thetas(joint), Pf_thetas(joint), Vi, Vf, optTime);
        calc(joint, :) = [maxVel, a0, a1, a2, a3];
    end
    
end

function [time] = performLoopIteration(tMat, Pi_thetas, Pf_thetas, Vi, Vf, dT_MAX, T_max, samples)

    % If only one item in Matrix, return index
    if length(tMat) == 1
        mid = tMat;
    elseif length(tMat) == 2
        mid = tMat(1);
    elseif length(tMat) == 3
        low = tMat(1);
        mid = tMat(2);
        up = tMat(2:3);
    else
        % Split the Matrix in the Middle
        [low, mid, up] = binarySplit(tMat);
    end

    % Solve for Joint Velocities for given timing
    % Pre allocate array to store max dTheta
    dTheta = zeros(1, length(dT_MAX));
    for joint=1:length(dT_MAX)
        [~, ~, ~, ~, ~, maxVel] = generateCubicTrajectory(Pi_thetas(joint), Pf_thetas(joint), Vi, Vf, mid);
        dTheta(joint) = maxVel;
    end

    % Compare to the maximums allowed
    exceed = abs(dTheta) > dT_MAX;

    if length(tMat) == 1
        if all(exceed(:) == 0)
            time = mid;
        else
            time = mid + (T_max/samples);
        end
        return

    elseif length(tMat) == 2
        if all(exceed(:) == 0)
            time = mid;
            return
        else
            time = tMat(2);
            return
        end
    
    % Otherwise, try recursion
    else
        % If any are not exceeded, decrease time
        if all(exceed(:) == 0)
            [time] = performLoopIteration(low, Pi_thetas, Pf_thetas, Vi, Vf, dT_MAX, T_max, samples);
        % If any are exceeded, increase time
        else
            [time] = performLoopIteration(up, Pi_thetas, Pf_thetas, Vi, Vf, dT_MAX, T_max, samples);
        end
    end
    
end

function [theta, a0, a1, a2, a3, maxVel] = generateCubicTrajectory(theta_i, theta_f, dTheta_i, dTheta_f, t_fi)
%   GENERATECUBICTRAJECTORY Uses a cubic function to generate the cubic
%   trajectory for a joint for theta_i to theta_f
%   theta_i : Initial angle for joint (rads)
%   theta_f : Final angle for joint (rads)
%   dTheta_i: Inital velocity for joint (rad s-1)
%   dTheta_f: Final velocity for joint (rad s-1)
%   t_fi    : The time taken for the motion (s)
    
    syms t;

    % Generate Coefficients
    a0 = theta_i;
    a1 = dTheta_i;
    a2 = (3 / t_fi^2) * (theta_f - theta_i) - (2 / t_fi) * dTheta_i - (1 / t_fi) * dTheta_f;
    a3 = -(2 / t_fi^3) * (theta_f - theta_i) + (1 / t_fi^2) * (dTheta_f + dTheta_i);
    
    % Generate Expressions for Pos, Vel, Accel
    theta = a0 + a1*t + a2*t^2 + a3*t^3;

    % Find the Maximum Velocity by usiing the differential and solving for
    % points of zero acceleration, then finding the largest abs value for 
    % velocity at these times.
    times = double(solve(2*a2 + 6*a3*t == 0, t));
    dTheta = vpa(a1 + 2*a2*times + 3*a3*times^2);
    [~, i] = max(abs(dTheta));
    maxVel = dTheta(i);

end

function [lowerArray, middle, upperArray] = binarySplit(in)
%%binarySplit Perform a binary split of an array of ordered values

    middleIndex = floor((length(in) / 2));
    middle = in(middleIndex);
    lowerArray = in(1:middleIndex);
    upperArray = in(middleIndex+1:length(in));

end