function [theta, a0, a1, a2, a3, maxVel] = generateCubicTrajectory(theta_i, theta_f, dTheta_i, dTheta_f, t_fi)
%GENERATECUBICTRAJECTORY Summary of this function goes here
%   Detailed explanation goes here
    
    syms t;

    % Generate Coefficients
    a0 = theta_i;
    a1 = dTheta_i;
    a2 = (3 / t_fi^2) * (theta_f - theta_i) - (2 / t_fi) * dTheta_i - (1 / t_fi) * dTheta_f;
    a3 = -(2 / t_fi^3) * (theta_f - theta_i) + (1 / t_fi^2) * (dTheta_f + dTheta_i);
    
    % Generate Expressions for Pos, Vel, Accel
    theta = a0 + a1*t + a2*t^2 + a3*t^3;
    dtheta = a1 + 2*a2*t + 3*a3*t^2;
    ddtheta = 2*a2 + 6*a3*t;

    % Solve for when ddTheta = 0
    times = double(solve(2*a2 + 6*a3*t == 0, t));

    % Find Vel
    dTheta = vpa(a1 + 2*a2*times + 3*a3*times^2);

    [~, i] = max(abs(dTheta));

    maxVel = dTheta(i);

end

