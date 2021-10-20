function [bestAindex, bestBindex] = findOptimalPath(posAThetas,posBThetas)
%FINDOPTIMALPATH Find the optimal path for given path candidates
%   Algorithm calculates for A1-6, A2-6, ... A6 and then sums
%   A crude method based on the idea that movement of A1 will affect
%   A2-6, A2 affect A3-6, etc...

    % Start with assumption that A(1) B(1) are best
    bestAindex = 1;
    bestBindex = 1;
    bestTheta = inf;

    % Calculate for all positions
    for ai = 1:height(posAThetas)
        for bi = 1:height(posBThetas)

            % Determine the delta in angle for each joint
            delThetaAB = posBThetas(bi, :) - posAThetas(ai, :);

            % Perform squaring algorithm
            alg = [ sumsqr(delThetaAB(:)) sumsqr(delThetaAB(2:6)) sumsqr(delThetaAB(3:6)) sumsqr(delThetaAB(4:6)) sumsqr(delThetaAB(5:6)) sumsqr(delThetaAB(6)) ];
            maxTheta = sum(alg);

            % If better than current best solution, append
            if maxTheta < bestTheta
                bestAindex = ai;
                bestBindex = bi;
                bestTheta = maxTheta;
            end

        end
    end

end

