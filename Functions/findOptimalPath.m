function [bestAindex, bestBindex] = findOptimalPath(posAThetas,posBThetas)
%FINDOPTIMALPATH Find the optimal path by theta
%   Detailed explanation goes here

    % Start with guess that A(1) B(1) are best
    bestAindex = 1;
    bestBindex = 1;
    bestTheta = 4*pi;

    % Calculate for 1,1
    for ai = 1:height(posAThetas)
        for bi = 1:height(posBThetas)
            delThetaAB = posBThetas(bi) - posAThetas(ai);
            maxTheta = sum(delThetaAB);
            if maxTheta < bestTheta
                bestAindex = ai;
                bestBindex = bi;
                bestTheta = maxTheta;
            end
        end
    end

end

